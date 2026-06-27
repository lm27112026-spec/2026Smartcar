#  RT1021 — 按键驱动控制

import gc, time, math
from machine import Pin
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
                   pause_encoder_ticker, resume_encoder_ticker, ENC_SCALE)
gc.collect()  # 导入关键模块前回收内存

from imu_motion import (update_angle, imu_read_safe,
                         reset_ang_vel_pid, stop_imu_ticker)

def _yaw():
    """动态获取 imu_motion.yaw（避免 from-import 值拷贝）"""
    return __import__('imu_motion').yaw

from key import capture, key_triggered, pet_watchdog
from uwb_position import UWBPosition

gc.collect()  # 导入相机模块前再次回收
from cam_control import CameraController

# ═══════════════════════════════════════════════════════════════
#  常量区域
# ═══════════════════════════════════════════════════════════════

LED_PIN  = 'C4'
SW2_PIN  = 'D9'

# ── C14: 前进 20cm ──
FORWARD_DIST_M   = 0.20    # 目标距离 (m)
FORWARD_SPEED    = 0.50    # 前进速度 (m/s)
FWD_TIMEOUT_S    = 10.0    # 超时 (s)
FWD_CTRL_DT      = 0.02    # 控制周期 (s)

# ── C14: UWB 平移 ──
UWB_LAT_SPEED    = 0.30    # 最大平移速度 (m/s)
UWB_X_DEADBAND   = 3.0     # X 方向死区 (cm)
UWB_LAT_P_GAIN   = 0.02    # X 误差 → 平移速度 P 增益
UWB_TIMEOUT_S    = 10.0    # 超时 (s)
UWB_INIT_TIMEOUT_S = 3.0   # UWB 初始化超时 (s)
UWB_CTRL_DT      = 0.02    # 控制周期 (s)

# ── 航向保持 ──
HEADING_KP       = 0.15    # 航向偏差 → wz P 增益
HEADING_DEADBAND = 2.0     # 航向死区 (度)
WZ_LIMIT         = 0.3     # wz 限幅 (归一化值)

# ── 启动: 记录原点后前进 20cm ──
STARTUP_FORWARD_DIST_CM = 20.0   # 目标距离 (cm)
STARTUP_FORWARD_SPEED   = 0.30   # 前进速度 (m/s)
STARTUP_TIMEOUT_S      = 5.0     # 超时 (s)
STARTUP_HEADING_DB     = 5.0     # 航向死区 (度)，偏差 5° 后自动回正

# ── 启动: 靠近目标 + 蓝牙信号 ──
WAIT_OK_TIMEOUT_MS       = 5000    # 等待从车 ok 应答超时 (ms)

# ── 启动: 收到 ok 后全速后退至黄线 ──
STARTUP_FULL_SPEED     = 1.00    # 全速后退速度（绝对值，m/s）
YELLOW_LINE_TIMEOUT_S  = 10.0    # 黄线检测超时 (s)

# ── 启动: 向 supplies 坐标靠近（UWB 直接 XY 控制） ──
SUPPLIES_KP           = 0.012   # 位置 P 增益 (参考 main_uwb.py)
SUPPLIES_DB           = 8.0     # 到位死区 (cm)
SUPPLIES_SLOW_DIST    = 20.0    # 减速距离 (cm)
SUPPLIES_MAX_SPEED    = 0.30    # 最大速度 (m/s)
SUPPLIES_TIMEOUT_S    = 20.0    # 超时 (s)
SUPPLIES_CTRL_DT      = 0.01    # 控制周期 (s)，参考 main_uwb.py
SUPPLIES_ARRIVAL_FRAMES   = 5      # 连续 N 帧在死区内算到达 supplies

# ── 物质区之字形搜索 ──
SEARCH_AREA_SIZE_CM      = 100.0   # 搜索区域边长 (cm)
SEARCH_ROW_STEP_CM       = 10.0    # 行间步进距离 (cm)
SEARCH_SPEED             = 0.30    # 搜索速度 (m/s)
SEARCH_ROW_TIMEOUT_S     = 30.0    # 单行搜索超时 (s)
SEARCH_STEP_TIMEOUT_S    = 10.0    # 步进超时 (s)
SEARCH_CTRL_DT           = 0.02    # 控制周期 (s)

# ── SW2 ──
SW2_DEBOUNCE_MS  = 50

# ── UWB 坐标记录 ──
origin   = None    # 起点坐标 (x, y)
supplies = (40.0, 50.0)    # 物资点固定坐标 (x, y)
_uwb_shared = None  # 共享 UWBPosition 实例
_cam_shared = None  # 共享 CameraController 实例
_TARGET_HEADING = None  # 全程锁定的目标航向

# ═══════════════════════════════════════════════════════════════
#  硬件初始化
# ═══════════════════════════════════════════════════════════════

led = Pin(LED_PIN, Pin.OUT, value=False)
sw2 = Pin(SW2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)

_sw2_last         = sw2.value()
_sw2_changed      = False
_sw2_stable_start = 0


def check_sw2():
    global _sw2_last, _sw2_changed, _sw2_stable_start
    val = sw2.value()
    if val != _sw2_last:
        _sw2_last = val
        _sw2_changed = True
        _sw2_stable_start = time.ticks_ms()
    if _sw2_changed and time.ticks_diff(time.ticks_ms(), _sw2_stable_start) >= SW2_DEBOUNCE_MS:
        _sw2_changed = False
        return True
    return False


# ═══════════════════════════════════════════════════════════════
#  蓝牙从车通信
# ═══════════════════════════════════════════════════════════════

from uart_master import MasterBT
bt = MasterBT()


# ═══════════════════════════════════════════════════════════════
#  工具函数与蓝牙安全接收
# ═══════════════════════════════════════════════════════════════

_imu_ticker_stopped = False
_imu_ok_count = 0     
_imu_fail_count = 0   


def _wait_for_follower_ok(timeout_ms=WAIT_OK_TIMEOUT_MS):
    """
    非阻塞式等待从车 ok。
    如果在等待期间检测到 SW2 拨码切换，返回 "abort" 用于触发手动跳过。
    """
    deadline = time.ticks_ms() + timeout_ms
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        pet_watchdog()  # 高频喂狗，防止系统硬复位
        if check_sw2(): # 检测 SW2 边沿变化
            print("  [BT] 等待过程中被 SW2 强制中断")
            return "abort"
        resp = bt.read_response()
        if resp == "ok":
            return "ok"
        time.sleep_ms(10)
    return "timeout"


def _read_imu_update_yaw():
    global _imu_ticker_stopped, _imu_ok_count, _imu_fail_count
    
    if not _imu_ticker_stopped:
        print("  [IMU] 停止 PIT3 ticker，切换到直接 SPI 读取...")
        stop_imu_ticker()
        _imu_ticker_stopped = True
    
    d = imu_read_safe()  
    if d is not None:
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        _imu_ok_count += 1
        if _imu_ok_count == 1:
            print("  [IMU] 首次成功 raw=(gx:{0:.1f} gy:{1:.1f} gz:{2:.1f}) yaw={3:.2f}°".format(
                d[3], d[4], d[5], _yaw()))
        if _imu_ok_count % 50 == 0:
            print("  [IMU] ok={}/fail={} yaw={:.2f}° gz_dps=({:+.1f},{:+.1f},{:+.1f})".format(
                _imu_ok_count, _imu_fail_count, _yaw(), d[3], d[4], d[5]))
        return True
    _imu_fail_count += 1
    if _imu_fail_count == 1:
        print("  [IMU] 首次读取失败（imu_read_safe 返回 None）")
    if _imu_fail_count % 50 == 0:
        print("  [IMU] fail count={} yaw={:.2f}°".format(_imu_fail_count, _yaw()))
    return False


def _lock_yaw():
    global _TARGET_HEADING
    if _TARGET_HEADING is not None:
        return _TARGET_HEADING
    for _ in range(20):  
        if _read_imu_update_yaw():
            _TARGET_HEADING = _yaw()
            print("  [HEADING] 全程航向锁定: {:.1f}°（偏差自动回正）".format(_TARGET_HEADING))
            return _TARGET_HEADING
        time.sleep_ms(10)
    _TARGET_HEADING = _yaw()
    print("  [HEADING] 全程航向锁定(兜底): {:.1f}°".format(_TARGET_HEADING))
    return _TARGET_HEADING


def _heading_correction(target_yaw, deadband=None):
    _read_imu_update_yaw()  

    if deadband is None:
        deadband = HEADING_DEADBAND

    error = target_yaw - _yaw()
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    if abs(error) < deadband:
        return 0.0

    wz = error * HEADING_KP
    return max(-WZ_LIMIT, min(wz, WZ_LIMIT))


def _encoder_reset():
    reset_encoder_filter()
    reset_wheel_pi()
    reset_ang_vel_pid()
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)


def _abort_check():
    pet_watchdog()
    if check_sw2():
        print("  [SW2] Abort requested")
        return True
    return False


# ═══════════════════════════════════════════════════════════════
#  UWB 共享实例管理
# ═══════════════════════════════════════════════════════════════

def _ensure_uwb():
    global _uwb_shared
    _reset_uwb_if_needed()
    if _uwb_shared is None:
        try:
            _uwb_shared = UWBPosition(uart_id=0, baudrate=115200, target_anchor="8834")
            print("  [UWB] 初始化中，等待首帧...")
            wait_start = time.ticks_ms()
            while _uwb_shared.get_frame_count() == 0:
                _uwb_shared.step()
                pet_watchdog()  
                if time.ticks_diff(time.ticks_ms(), wait_start) > 3000:
                    print("  [UWB] 首帧超时")
                    _uwb_shared.stop()
                    _uwb_shared = None
                    return None
                time.sleep_ms(10)
            if _uwb_shared.get_frame_count() > 0:
               print("  [UWB] 就绪 (frame={})".format(_uwb_shared.get_frame_count()))
        except Exception as e:
            print("  [UWB] 初始化失败:", e)
            _uwb_shared = None
    return _uwb_shared


def _reset_uwb_if_needed():
    """如果 UWB 超时，重置以便下次 _ensure_uwb() 重建。"""
    global _uwb_shared
    if _uwb_shared is not None and _uwb_shared.is_timeout():
        print("  [UWB] 超时，重置连接...")
        _uwb_shared.stop()
        _uwb_shared = None


# ═══════════════════════════════════════════════════════════════
#  摄像头共享实例管理
# ═══════════════════════════════════════════════════════════════

def _ensure_cam():
    global _cam_shared
    if _cam_shared is None:
        _cam_shared = CameraController(uart_id=7)
        print("[CAM] CameraController 就绪")
    return _cam_shared


# ═══════════════════════════════════════════════════════════════
#  C14 步骤 1: 前进 20cm（航向保持）
# ═══════════════════════════════════════════════════════════════

def _action_forward_20cm():
    """前进 20cm，IMU 航向闭环保持。返回: True=正常完成, False=中断"""
    print("  [FWD] Starting forward {:.0f}cm...".format(FORWARD_DIST_M * 100))

    target_heading = _lock_yaw()
    print("  [FWD] Heading locked: {:.1f}°".format(target_heading))

    total_pulses = [0, 0, 0, 0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0

    led.value(1)

    while True:
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > FWD_TIMEOUT_S:
            print("  [FWD] Timeout ({:.1f}s)".format(elapsed))
            led.value(0)
            return False

        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        for i in range(4):
            total_pulses[i] += abs(counts[i])

        wheel_dists = []
        for i in range(4):
            if ENC_SCALE[i] != 0:
                wheel_dists.append(total_pulses[i] / abs(ENC_SCALE[i]))
        avg_dist = sum(wheel_dists) / len(wheel_dists) if wheel_dists else 0.0

        if avg_dist >= FORWARD_DIST_M:
            print("  [FWD] Target reached! dist={:.2f}m pulses={}".format(
                avg_dist, total_pulses))
            led.value(0)
            return True

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [FWD] dist={0:.2f}m / {1:.2f}m  yaw={2:.2f}°".format(
                avg_dist, FORWARD_DIST_M, _yaw()))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        wz = _heading_correction(target_heading)

        try:
            rs = [counts[i] / ENC_SCALE[i] / FWD_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(FORWARD_SPEED, 0, wz, rs, FWD_CTRL_DT)
        except Exception as e:
            print("  [FWD] Drive error:", e)

        time.sleep_ms(int(FWD_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  启动步骤: 前进 20cm（航向保持，5° 偏差回正）
# ═══════════════════════════════════════════════════════════════

def _action_startup_forward():
    dist_m = STARTUP_FORWARD_DIST_CM / 100.0
    print("  [STARTUP] 前进 {:.0f}cm 开始...".format(STARTUP_FORWARD_DIST_CM))

    target_heading = _lock_yaw()
    print("  [STARTUP] 航向锁定: {:.1f}°  死区 {:.0f}° 偏差自动回正".format(
        target_heading, STARTUP_HEADING_DB))

    total_pulses = [0, 0, 0, 0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0

    led.value(1)

    while True:
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > STARTUP_TIMEOUT_S:
            print("  [STARTUP] 超时 ({:.1f}s)".format(elapsed))
            led.value(0)
            return False

        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        for i in range(4):
            total_pulses[i] += abs(counts[i])

        wheel_dists = []
        for i in range(4):
            if ENC_SCALE[i] != 0:
                wheel_dists.append(total_pulses[i] / abs(ENC_SCALE[i]))
        avg_dist = sum(wheel_dists) / len(wheel_dists) if wheel_dists else 0.0

        if avg_dist >= dist_m:
            print("  [STARTUP] 到达目标! dist={:.2f}m pulses={}".format(
                avg_dist, total_pulses))
            led.value(0)
            return True

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [STARTUP] dist={0:.2f}m / {1:.2f}m  yaw={2:.2f}°".format(
                avg_dist, dist_m, _yaw()))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        wz = _heading_correction(target_heading, deadband=STARTUP_HEADING_DB)

        try:
            rs = [counts[i] / ENC_SCALE[i] / FWD_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(STARTUP_FORWARD_SPEED, 0, wz, rs, FWD_CTRL_DT)
        except Exception as e:
            print("  [STARTUP] 驱动错误:", e)

        time.sleep_ms(int(FWD_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  修改重构: 摄像头跟随靠近（SW2 手动旁路，强制向下运行代码）
# ═══════════════════════════════════════════════════════════════

def see_and_push():
    """
    摄像头跟随目标 → 到达安全判定阈值（或中途丢目标） → 发送蓝牙通知从车。
    【手动旁路功能】：如果检测到 SW2 拨码切换，将强制退出蓝牙等待，直接返回 True 从而向下执行后续代码。
    """
    APPROACH_TIMEOUT_S    = 15.0   
    APPROACH_CTRL_DT      = 0.02   
    APPR_DEADBAND         = 0.5    
    APPROACH_THRESHOLD_CM = 6.0    # 防撞兜底阈值 (cm)，小于内环期望 (5cm) + 到达带宽 (2cm) 的上界
    CLOSE_LOSS_THRESHOLD_CM = 16.0  # 近距离丢锁判定阈值 (cm)，<=此值丢锁视为盲区遮挡到位

    print("\n  [APPROACH] === 靠近目标 & 蓝牙信号 ===")

    cam = _ensure_cam()
    cam.reset()

    target_heading = _lock_yaw()
    print("  [APPROACH] 航向锁定: {:.1f}°".format(target_heading))

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    has_found_once = False
    last_known_dist = 999.0   # 最后一次有效目标距离 (cm)，用于丢锁时的距离判定

    led.value(1)

    # 1. 车辆驱动闭环靠近目标逻辑
    while True:
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > APPROACH_TIMEOUT_S:
            print("  [APPROACH] 靠近超时 ({:.1f}s)".format(elapsed))
            led.value(0)
            return False

        ctrl = cam.step()

        if ctrl['state_msg']:
            print("\n" + ctrl['state_msg'])

        if ctrl['has_target']:
            has_found_once = True
            if 0 < ctrl['dist_cm']:
                last_known_dist = ctrl['dist_cm']  # 更新最近有效距离，供丢锁判定使用

        # ── [安全逻辑 1] 优先判定内环 PID 精确对齐到达 ──
        if ctrl['arrived']:
            print("\n  [APPROACH] ➔ 触发内环 PID 精确对齐到位（ctrl['arrived'] == True）")
            stop_all()
            break

        # ── [安全逻辑 1-B] 物理距离防撞安全兜底 ──
        if ctrl['has_target'] and 0 < ctrl['dist_cm'] <= APPROACH_THRESHOLD_CM:
            print("\n  [APPROACH] ➔ 物理距离 D:{:.1f}cm 已达到防撞兜底界限 (<= {:.1f}cm)，直接触发到达".format(
                ctrl['dist_cm'], APPROACH_THRESHOLD_CM))
            stop_all()
            break

        # ── [安全逻辑 2] 丢锁滤波中断判定（替换原有的丢锁快速中断判定） ──
        # 利用 ctrl['state'] == 2 (即内环 S_LOST) 检测连续丢失超过 500ms，防止单帧抖动误触发
        if has_found_once and ctrl['state'] == 2:
            if last_known_dist <= CLOSE_LOSS_THRESHOLD_CM:  # CLOSE_LOSS_THRESHOLD_CM 设为 16.0
                print("\n  [APPROACH] ➔ 目标在近距离 ({:.1f}cm <= {:.1f}cm) 连续丢失，判定为盲区遮挡到位！".format(
                    last_known_dist, CLOSE_LOSS_THRESHOLD_CM))
                stop_all()
                break  # 确认近距离到位，跳出驱动循环，前往发送蓝牙
            else:
                # 远距离丢锁视为失败，安全停下，退回主状态机去重新搜索，绝不向从车发抢跑蓝牙
                print("\n  [APPROACH] ➔ 目标在远距离 ({:.1f}cm) 异常丢失！未到达对齐范围，返回重新搜索。".format(
                    last_known_dist))
                stop_all()
                cam.reset()
                return False  # 返回 False，主程序会自动继续进行之字形搜索

        if ctrl['vx'] is not None and ctrl['vy'] is not None:
            wz = _heading_correction(target_heading, deadband=APPR_DEADBAND)
            try:
                rc = get_encoder_counts()
                if rc is not None and len(rc) >= 4:
                    rs = [rc[i] / ENC_SCALE[i] / APPROACH_CTRL_DT if ENC_SCALE[i] != 0 else 0
                          for i in range(4)]
                    omni_drive_closed_loop(ctrl['vx'], ctrl['vy'], wz, rs, APPROACH_CTRL_DT)
            except Exception as e:
                print("  [APPROACH] 驱动错误:", e)
        else:
            wz = _heading_correction(target_heading, deadband=APPR_DEADBAND)
            if abs(wz) > 0.001:
                try:
                    rc = get_encoder_counts()
                    if rc is not None and len(rc) >= 4:
                        rs = [rc[i] / ENC_SCALE[i] / APPROACH_CTRL_DT if ENC_SCALE[i] != 0 else 0
                              for i in range(4)]
                        omni_drive_closed_loop(0, 0, wz, rs, APPROACH_CTRL_DT)
                except Exception:
                    pass
            else:
                stop_all()

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            tgt_str = "T" if ctrl['has_target'] else "-"
            print("  [APPROACH] {0} X:{1:+5.1f}cm D:{2:5.1f}cm vx:{3:.2f} vy:{4:.2f} yaw={5:.2f}°".format(
                tgt_str, ctrl['x_cm'], ctrl['dist_cm'],
                ctrl['vx'] if ctrl['vx'] else 0.0,
                ctrl['vy'] if ctrl['vy'] else 0.0,
                _yaw()))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        time.sleep_ms(int(APPROACH_CTRL_DT * 1000))

    # 2. 蓝牙发送与等待确认闭环
    led.value(0)
    
    while True:
        # 每次检测 SW2。如果已被拨动，将其视为手动跳过（Bypass）信号，直接返回 True 从而向下执行后续代码！
        if check_sw2():
            print("  [APPROACH] 检测到 SW2 手动中断，强制跳过蓝牙等待，直接向下进行代码！")
            return True

        print("  [APPROACH] 向从车发送数字 0 (turn_right)...")
        try:
            bt.turn_right()
            print("  [APPROACH] 指令已发，等待从车 ok...")

            # 评估返回值（"ok", "abort", "timeout"）
            status = _wait_for_follower_ok(timeout_ms=WAIT_OK_TIMEOUT_MS)
            
            if status == "ok":
                print("  [APPROACH] 从车确认完毕 (ok)，准备过渡到全速倒车")
                return True
                
            elif status == "abort":
                print("  [APPROACH] 检测到 SW2 手动中断，强制跳过蓝牙等待，直接向下进行代码！")
                return True  # 返回 True，使主状态机向下运行后续代码
                
            else:
                # 仅在 "timeout" 时，才允许重新发送并进入下一次循环
                print("  [APPROACH] 等待从车 ok 超时 ({:.1f}s)，将重新尝试发送...".format(
                    WAIT_OK_TIMEOUT_MS / 1000.0))
                
        except Exception as e:
            print("  [APPROACH] 蓝牙发送异常:", e)
            time.sleep_ms(500)


# ═══════════════════════════════════════════════════════════════
#  启动步骤: 收到 ok 后全速前进至黄线停车
# ═══════════════════════════════════════════════════════════════

def _action_forward_until_yellow():
    """
    全速前进，同时摄像头检测黄线，检测到后停车。
    航向闭环保持，退出条件：检测到黄线 / 超时 / SW2 中断。
    """
    print("\n  [YELLOW] === 全速前进，等待黄线 ===")

    target_heading = _lock_yaw()
    print("  [YELLOW] 航向锁定: {:.1f}°  全速 {:.1f}m/s 前进".format(
        target_heading, STARTUP_FULL_SPEED))

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0

    led.value(1)

    while True:
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > YELLOW_LINE_TIMEOUT_S:
            print("  [YELLOW] 超时 ({:.1f}s)，未检测到黄线".format(elapsed))
            led.value(0)
            return False

        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        # ── 摄像头黄线检测 ──
        cam = _ensure_cam()
        ctrl = cam.step()
        if ctrl['line_flag']:
            stop_all()  
            time.sleep_ms(300)
            _encoder_reset()
            print("  [YELLOW] 检测到黄线，安全停车并重置编码器！")
            led.value(0)
            return True

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [YELLOW] 全速后退中... yaw={0:.2f}°  t={1:.1f}s".format(
                _yaw(), elapsed))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        wz = _heading_correction(target_heading)

        try:
            rs = [counts[i] / ENC_SCALE[i] / FWD_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(-STARTUP_FULL_SPEED, 0, wz, rs, FWD_CTRL_DT)
        except Exception as e:
            print("  [YELLOW] 驱动错误:", e)

        time.sleep_ms(int(FWD_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  启动步骤: 导航到 supplies 固定坐标（UWB XY + 航向锁）
# ═══════════════════════════════════════════════════════════════

def _action_goto_supplies_startup():
    uwb = _ensure_uwb()
    if uwb is None:
        print("  [GOTO] UWB 不可用")
        return False

    if supplies is None:
        print("  [GOTO] supplies 坐标未记录")
        return False

    target_x, target_y = supplies
    print("\n  [GOTO] === 导航到 supplies ({:.1f}, {:.1f}) ===".format(target_x, target_y))

    target_heading = _lock_yaw()
    print("  [GOTO] 航向锁定: {:.1f}°".format(target_heading))

    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    last_uwb_ms = start_ms
    loop_cnt = 0
    near_target_count = 0
    uwb_dead_start = 0   

    led.value(1)

    while True:
        if _abort_check():
            led.value(0)
            return False

        elapsed_total = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed_total > SUPPLIES_TIMEOUT_S:
            print("  [GOTO] 超时 ({:.1f}s)".format(elapsed_total))
            led.value(0)
            return False

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_uwb_ms) >= 50:
            uwb.step()
            last_uwb_ms = now_ms

        if uwb.is_timeout():
            if uwb_dead_start == 0:
                uwb_dead_start = time.ticks_ms()
            elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > 3000:
                print("  [GOTO] UWB 超时 3s+，重连中...")
                _reset_uwb_if_needed()
                new_uwb = _ensure_uwb()
                if new_uwb is not None:
                    uwb = new_uwb
                    uwb_dead_start = 0
                    last_uwb_ms = time.ticks_ms()
                    start_ms = time.ticks_ms()
                    last_print_ms = start_ms
                    print("  [GOTO] UWB 重连成功，继续导航")
                else:
                    print("  [GOTO] UWB 重连未就绪，继续等待...")
                    uwb_dead_start = time.ticks_ms()  
        else:
            uwb_dead_start = 0

        # ── UWB 离线时停车等待 ──
        uwb_alive = not uwb.is_timeout()
        if uwb_alive:
            curr_x, curr_y = uwb.get_position()
            error_x = target_x - curr_x
            error_y = target_y - curr_y
            dist = math.sqrt(error_x * error_x + error_y * error_y)
        else:
            error_x = 0.0
            error_y = 0.0
            dist = 999.0  

        if dist < SUPPLIES_DB:
            near_target_count += 1
            if near_target_count >= SUPPLIES_ARRIVAL_FRAMES:
                print("  [GOTO] 已到达 supplies ({:.1f}, {:.1f})  dist={:.1f}cm".format(
                    target_x, target_y, dist))
                led.value(0)
                return True
        else:
            near_target_count = 0

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [GOTO] pos=({0:.1f},{1:.1f}) target=({2:.1f},{3:.1f}) dist={4:.1f}cm yaw={5:.2f}°".format(
                curr_x, curr_y, target_x, target_y, dist, _yaw()))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        wz = _heading_correction(target_heading)

        yaw_rad = math.radians(_yaw())
        body_fwd  = -math.cos(yaw_rad) * error_x - math.sin(yaw_rad) * error_y
        body_right =  math.sin(yaw_rad) * error_x - math.cos(yaw_rad) * error_y

        vx_cmd = body_fwd * SUPPLIES_KP
        vy_cmd = body_right * SUPPLIES_KP

        if dist < SUPPLIES_SLOW_DIST and dist > 0:
            decay = (dist - SUPPLIES_DB) / (SUPPLIES_SLOW_DIST - SUPPLIES_DB)
            decay = max(0.0, min(1.0, decay))
            vx_cmd *= decay
            vy_cmd *= decay

        speed = math.sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd)
        if speed > SUPPLIES_MAX_SPEED:
            vx_cmd = vx_cmd / speed * SUPPLIES_MAX_SPEED
            vy_cmd = vy_cmd / speed * SUPPLIES_MAX_SPEED

        GOTO_MIN_SPEED = 0.06  
        if 0 < speed < GOTO_MIN_SPEED and dist > SUPPLIES_DB:
            vx_cmd = vx_cmd / speed * GOTO_MIN_SPEED
            vy_cmd = vy_cmd / speed * GOTO_MIN_SPEED

        try:
            rc = get_encoder_counts()
            if rc is None or len(rc) < 4:
                time.sleep_ms(5)
                continue
            rs = [rc[i] / ENC_SCALE[i] / SUPPLIES_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(vx_cmd, vy_cmd, wz, rs, SUPPLIES_CTRL_DT)
        except Exception as e:
            print("  [GOTO] 驱动错误:", e)

        time.sleep_ms(int(SUPPLIES_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  返回原点: UWB 导航到 origin 坐标（航向锁）
# ═══════════════════════════════════════════════════════════════

def _action_return_to_origin():
    global origin

    uwb = _ensure_uwb()
    if uwb is None:
        print("  [RTN] UWB 不可用")
        return False

    if origin is None:
        print("  [RTN] origin 坐标未记录")
        return False

    target_x, target_y = origin
    print("\n  [RTN] === 返回原点 ({:.1f}, {:.1f}) ===".format(target_x, target_y))

    target_heading = _lock_yaw()
    print("  [RTN] 航向锁定: {:.1f}°".format(target_heading))

    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    last_uwb_ms = start_ms
    loop_cnt = 0
    near_target_count = 0
    uwb_dead_start = 0   

    led.value(1)

    while True:
        if _abort_check():
            led.value(0)
            return False

        elapsed_total = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed_total > SUPPLIES_TIMEOUT_S:
            print("  [RTN] 超时 ({:.1f}s)".format(elapsed_total))
            led.value(0)
            return False

        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_uwb_ms) >= 50:
            uwb.step()
            last_uwb_ms = now_ms

        if uwb.is_timeout():
            if uwb_dead_start == 0:
                uwb_dead_start = time.ticks_ms()
            elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > 3000:
                print("  [RTN] UWB 超时 3s+，重连中...")
                _reset_uwb_if_needed()
                new_uwb = _ensure_uwb()
                if new_uwb is not None:
                    uwb = new_uwb
                    uwb_dead_start = 0
                    last_uwb_ms = time.ticks_ms()
                    start_ms = time.ticks_ms()
                    last_print_ms = start_ms
                    print("  [RTN] UWB 重连成功，继续导航")
                else:
                    print("  [RTN] UWB 重连未就绪，继续等待...")
                    uwb_dead_start = time.ticks_ms()
        else:
            uwb_dead_start = 0

        # ── UWB 离线时停车等待 ──
        uwb_alive = not uwb.is_timeout()
        if uwb_alive:
            curr_x, curr_y = uwb.get_position()
            error_x = target_x - curr_x
            error_y = target_y - curr_y
            dist = math.sqrt(error_x * error_x + error_y * error_y)
        else:
            error_x = 0.0
            error_y = 0.0
            dist = 999.0  

        if dist < SUPPLIES_DB:
            near_target_count += 1
            if near_target_count >= SUPPLIES_ARRIVAL_FRAMES:
                print("  [RTN] 已到达原点 ({:.1f}, {:.1f})  dist={:.1f}cm".format(
                    target_x, target_y, dist))
                led.value(0)
                return True
        else:
            near_target_count = 0

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [RTN] pos=({0:.1f},{1:.1f}) target=({2:.1f},{3:.1f}) dist={4:.1f}cm yaw={5:.2f}°".format(
                curr_x, curr_y, target_x, target_y, dist, _yaw()))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        wz = _heading_correction(target_heading)

        yaw_rad = math.radians(_yaw())
        body_fwd  = -math.cos(yaw_rad) * error_x - math.sin(yaw_rad) * error_y
        body_right =  math.sin(yaw_rad) * error_x - math.cos(yaw_rad) * error_y

        vx_cmd = body_fwd * SUPPLIES_KP
        vy_cmd = body_right * SUPPLIES_KP

        if dist < SUPPLIES_SLOW_DIST and dist > 0:
            decay = (dist - SUPPLIES_DB) / (SUPPLIES_SLOW_DIST - SUPPLIES_DB)
            decay = max(0.0, min(1.0, decay))
            vx_cmd *= decay
            vy_cmd *= decay

        speed = math.sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd)
        if speed > SUPPLIES_MAX_SPEED:
            vx_cmd = vx_cmd / speed * SUPPLIES_MAX_SPEED
            vy_cmd = vy_cmd / speed * SUPPLIES_MAX_SPEED

        if 0 < speed < 0.06 and dist > SUPPLIES_DB:
            vx_cmd = vx_cmd / speed * 0.06
            vy_cmd = vy_cmd / speed * 0.06

        try:
            rc = get_encoder_counts()
            if rc is None or len(rc) < 4:
                time.sleep_ms(5)
                continue
            rs = [rc[i] / ENC_SCALE[i] / SUPPLIES_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(vx_cmd, vy_cmd, wz, rs, SUPPLIES_CTRL_DT)
        except Exception as e:
            print("  [RTN] 驱动错误:", e)

        time.sleep_ms(int(SUPPLIES_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  启动步骤: 之字形搜索 100cm×100cm 物质区
# ═══════════════════════════════════════════════════════════════

def _action_search_supplies_area():
    """
    到达 supplies 后在 100cm×100cm 区域进行之字形搜索。
    全程摄像头检测，识别到物品立即返回 True。
    全部搜索完未检测到返回 False。
    """
    target_heading = _lock_yaw()
    print("\n  [SEARCH] === 之字形搜索 100cm×100cm 物质区 ===")
    print("  [SEARCH] 航向锁定: {:.1f}°".format(target_heading))

    AREA_CM   = SEARCH_AREA_SIZE_CM     
    STEP_CM   = SEARCH_ROW_STEP_CM      
    MAX_ROWS  = int(AREA_CM / STEP_CM)  

    led.value(1)

    for row in range(MAX_ROWS):
        direction   = "RIGHT" if (row % 2 == 0) else "LEFT"
        vy_sign     = 1.0 if direction == "RIGHT" else -1.0
        row_dist_m  = AREA_CM / 100.0   
        row_label   = "{}/{}".format(row + 1, MAX_ROWS)

        print("\n  [SEARCH] --- 第 {} 行 {} {:.0f}cm ---".format(
            row_label, direction, AREA_CM))

        # ── 本行横向搜索 ──
        total_pulses = [0, 0, 0, 0]
        row_start_ms = time.ticks_ms()
        row_last_print_ms = row_start_ms

        while True:
            if _abort_check():
                led.value(0)
                return False

            elapsed = time.ticks_diff(time.ticks_ms(), row_start_ms) / 1000.0
            if elapsed > SEARCH_ROW_TIMEOUT_S:
                print("  [SEARCH] 行 {} 超时 ({:.1f}s)".format(row_label, elapsed))
                led.value(0)
                return False

            counts = get_encoder_counts()
            if counts is None or len(counts) < 4:
                time.sleep_ms(5)
                continue

            for i in range(4):
                total_pulses[i] += abs(counts[i])

            wheel_dists = []
            for i in range(4):
                if ENC_SCALE[i] != 0:
                    wheel_dists.append(total_pulses[i] / abs(ENC_SCALE[i]))
            avg_dist = sum(wheel_dists) / len(wheel_dists) if wheel_dists else 0.0

            # ── 摄像头物品检测 ──
            cam = _ensure_cam()
            ctrl = cam.step()
            if ctrl['has_target']:
                stop_all()
                print("  [SEARCH] 摄像头识别到物品！")
                led.value(0)
                return True

            if avg_dist >= row_dist_m:
                print("  [SEARCH] 第 {} 行完成 dist={:.2f}m".format(row_label, avg_dist))
                break

            now = time.ticks_ms()
            if time.ticks_diff(now, row_last_print_ms) >= 500:
                row_last_print_ms = now
                print("  [SEARCH] 行{0} {1} dist={2:.2f}m / {3:.0f}cm yaw={4:.2f}°".format(
                    row_label, direction, avg_dist, AREA_CM, _yaw()))

            wz = _heading_correction(target_heading)

            try:
                rs = [counts[i] / ENC_SCALE[i] / SEARCH_CTRL_DT
                      if ENC_SCALE[i] != 0 else 0 for i in range(4)]
                omni_drive_closed_loop(0, vy_sign * SEARCH_SPEED, wz, rs, SEARCH_CTRL_DT)
            except Exception as e:
                print("  [SEARCH] 驱动错误:", e)

            time.sleep_ms(int(SEARCH_CTRL_DT * 1000))

        # ── 行间步进 ──
        if row >= MAX_ROWS - 1:
            break  

        print("  [SEARCH] 行间步进 {:.0f}cm...".format(STEP_CM))
        stop_all()
        time.sleep_ms(200)

        step_dist_m = STEP_CM / 100.0  
        step_total_pulses = [0, 0, 0, 0]
        step_start_ms = time.ticks_ms()
        step_last_print_ms = step_start_ms

        while True:
            if _abort_check():
                led.value(0)
                return False

            elapsed = time.ticks_diff(time.ticks_ms(), step_start_ms) / 1000.0
            if elapsed > SEARCH_STEP_TIMEOUT_S:
                print("  [SEARCH] 步进超时 ({:.1f}s)".format(elapsed))
                led.value(0)
                return False

            counts = get_encoder_counts()
            if counts is None or len(counts) < 4:
                time.sleep_ms(5)
                continue

            for i in range(4):
                step_total_pulses[i] += abs(counts[i])

            wheel_dists = []
            for i in range(4):
                if ENC_SCALE[i] != 0:
                    wheel_dists.append(step_total_pulses[i] / abs(ENC_SCALE[i]))
            avg_dist = sum(wheel_dists) / len(wheel_dists) if wheel_dists else 0.0

            # ── 摄像头检测 ──
            cam = _ensure_cam()
            ctrl = cam.step()
            if ctrl['has_target']:
                stop_all()
                print("  [SEARCH] 步进中识别到物品！")
                led.value(0)
                return True

            if avg_dist >= step_dist_m:
                print("  [SEARCH] 步进完成 dist={:.2f}m".format(avg_dist))
                break

            now = time.ticks_ms()
            if time.ticks_diff(now, step_last_print_ms) >= 500:
                step_last_print_ms = now
                print("  [SEARCH] 步进 dist={0:.2f}m / {1:.0f}cm yaw={2:.2f}°".format(avg_dist, STEP_CM, _yaw()))

            wz = _heading_correction(target_heading)

            try:
                rs = [counts[i] / ENC_SCALE[i] / SEARCH_CTRL_DT
                      if ENC_SCALE[i] != 0 else 0 for i in range(4)]
                omni_drive_closed_loop(SEARCH_SPEED, 0, wz, rs, SEARCH_CTRL_DT)
            except Exception as e:
                print("  [SEARCH] 驱动错误:", e)

            time.sleep_ms(int(SEARCH_CTRL_DT * 1000))

        stop_all()
        time.sleep_ms(200)

    print("  [SEARCH] 全部 {:.0f}cm×{:.0f}cm 区域搜索完成，未检测到物品".format(
        AREA_CM, AREA_CM))
    led.value(0)
    return False


# ═══════════════════════════════════════════════════════════════
#  复合型物资区搜索流（5次前移20cm重试机制）
# ═══════════════════════════════════════════════════════════════

def _execute_supplies_search_flow():
    """
    运行完整的物资区搜索及步进重试控制链。
    """
    for attempt in range(6):  # 0 为首次搜，1~5 为重试次数
        if attempt > 0:
            print("\n  [SEARCH] ➔ 第 {0}/5 次前移 20cm 重新搜索...".format(attempt))
            stop_all()
            time.sleep_ms(300)
            _encoder_reset()
            
            # 前移 20cm 重新搜索
            if not _action_forward_20cm():
                print("  [SEARCH] 步进前移过程中断，结束本次重试")
                return False
                
            stop_all()
            time.sleep_ms(300)
            _encoder_reset()
            
        # 运行完整的之字形搜索区域
        if _action_search_supplies_area():
            return True
            
    return False


# ═══════════════════════════════════════════════════════════════
#  C14 步骤 2: UWB 平移（航向保持）
# ═══════════════════════════════════════════════════════════════

def _action_uwb_translate():
    print("  [UWB] Starting translate toward anchor...")

    target_heading = _lock_yaw()
    print("  [UWB] Heading locked: {:.1f}°".format(target_heading))

    uwb = _ensure_uwb()
    if uwb is None:
        print("  [UWB] UWB not initialized")
        return False
    print("  [UWB] UWB ready, frame={}".format(uwb.get_frame_count()))

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    uwb_dead_start = 0   

    led.value(1)

    while True:
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > UWB_TIMEOUT_S:
            print("  [UWB] Translate timeout ({:.1f}s)".format(elapsed))
            led.value(0)
            return False

        uwb.step()
        x_cm, y_cm = uwb.get_position()

        if uwb.is_timeout():
            if uwb_dead_start == 0:
                uwb_dead_start = time.ticks_ms()
            elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > 3000:
                print("  [UWB] UWB 超时 3s+，尝试重连...")
                _reset_uwb_if_needed()
                new_uwb = _ensure_uwb()
                if new_uwb is not None:
                    uwb = new_uwb
                    uwb_dead_start = 0
                    print("  [UWB] UWB 重连成功")
                else:
                    print("  [UWB] UWB 重连失败，退出平移")
                    led.value(0)
                    return False
        else:
            uwb_dead_start = 0

        if abs(x_cm) < UWB_X_DEADBAND:
            print("  [UWB] Centered! X={:.1f}cm (deadband={:.1f}cm)".format(
                 x_cm, UWB_X_DEADBAND))
            led.value(0)
            return True

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            dist_cm, angle_deg = uwb.get_distance_angle()
            print("  [UWB] X={0:+.1f}cm Y={1:.1f}cm D={2:.1f}cm A={3:.1f}°  yaw={4:.2f}°".format(
                x_cm, y_cm, dist_cm, angle_deg, _yaw()))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        fwd_speed = x_cm * UWB_LAT_P_GAIN
        fwd_speed = max(-UWB_LAT_SPEED, min(fwd_speed, UWB_LAT_SPEED))

        wz = _heading_correction(target_heading)

        try:
            rc = get_encoder_counts()
            if rc is None or len(rc) < 4:
                time.sleep_ms(5)
                continue
            rs = [rc[i] / ENC_SCALE[i] / UWB_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(fwd_speed, 0, wz, rs, UWB_CTRL_DT)
        except Exception as e:
            print("  [UWB] Drive error:", e)

        time.sleep_ms(int(UWB_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  C14 组合动作: 前进 20cm → UWB 平移
# ═══════════════════════════════════════════════════════════════

def action_c14():
    print("\n" + "=" * 50)
    print("[C14] 前进 20cm → UWB 平移（航向保持）")
    print("=" * 50)

    pause_encoder_ticker()
    _encoder_reset()

    result = True

    if not _action_forward_20cm():
        result = False
    else:
        stop_all()
        time.sleep_ms(300)

        _encoder_reset()
        if not _action_uwb_translate():
            result = False

    stop_all()
    resume_encoder_ticker()
    led.value(0)

    if result:
        print("[C14] ✓ 完成")
    else:
        print("[C14] ✗ 中断")
    print("=" * 50 + "\n")


# ═══════════════════════════════════════════════════════════════
#  C8: 蓝牙向从车发送消息
# ═══════════════════════════════════════════════════════════════

def action_c8():
    print("\n" + "=" * 50)
    print("[C8] 蓝牙发送从车消息")
    print("=" * 50)

    try:
        bt.send_sync_move(0, 0, 0)
        print("[C8] 消息已发送")
    except Exception as e:
        print("[C8] 发送失败:", e)

    print("=" * 50 + "\n")


# ═══════════════════════════════════════════════════════════════
#  C9: 摄像头跟随靠近
# ═══════════════════════════════════════════════════════════════

def action_c9():
    print("\n" + "=" * 50)
    print("[C9] 摄像头跟随靠近")
    print("=" * 50)

    C9_TIMEOUT_S = 10.0
    C9_CTRL_DT   = 0.02

    pause_encoder_ticker()
    _encoder_reset()

    cam = _ensure_cam()
    cam.reset()

    target_heading = _lock_yaw()
    print("  [C9] 航向锁定: {:.1f}°".format(target_heading))

    start_ms = time.ticks_ms()
    loop_cnt = 0

    led.value(1)

    try:
        while True:
            if _abort_check():
                print("  [C9] SW2 中断")
                break

            elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            if elapsed > C9_TIMEOUT_S:
                print("  [C9] 超时 ({:.1f}s)".format(elapsed))
                break

            ctrl = cam.step()

            if ctrl['arrived']:
                stop_all()
                print("  [C9] 已到达目标！")
                break

            if ctrl['vx'] is not None and ctrl['vy'] is not None:
                wz = _heading_correction(target_heading)
                try:
                    rc = get_encoder_counts()
                    if rc is not None and len(rc) >= 4:
                        rs = [rc[i] / ENC_SCALE[i] / C9_CTRL_DT if ENC_SCALE[i] != 0 else 0
                              for i in range(4)]
                        omni_drive_closed_loop(ctrl['vx'], ctrl['vy'], wz, rs, C9_CTRL_DT)
                except Exception as e:
                    print("  [C9] 驱动错误:", e)
            else:
                wz = _heading_correction(target_heading)
                if abs(wz) > 0.001:
                    try:
                        rc = get_encoder_counts()
                        if rc is not None and len(rc) >= 4:
                            rs = [rc[i] / ENC_SCALE[i] / C9_CTRL_DT if ENC_SCALE[i] != 0 else 0
                                  for i in range(4)]
                        omni_drive_closed_loop(0, 0, wz, rs, C9_CTRL_DT)
                    except Exception:
                        pass
                else:
                    stop_all()

            loop_cnt += 1
            if loop_cnt % 50 == 0:
                gc.collect()

            time.sleep_ms(int(C9_CTRL_DT * 1000))

    finally:
        stop_all()
        resume_encoder_ticker()
        led.value(0)

    print("=" * 50 + "\n")


# ═══════════════════════════════════════════════════════════════
#  主程序流程控制（main）
# ═══════════════════════════════════════════════════════════════

def main():
    print("")
    print("=" * 50)
    print("  RT1021 — 按键驱动控制（重搜+丢标瞬锁版）")
    print("  启动: 原点 → 前进20cm → 导航supplies → 之字形复合重搜")
    print("=" * 50)
    print("  C14 (KEY3): 前进 20cm → UWB 平移")
    print("  C8  (KEY1): 蓝牙发送从车消息")
    print("  C9  (KEY2): 摄像头跟随靠近")
    print("  SW2 (D9)  : 强制退出")
    print("=" * 50)
    print("")
    
    # ── UWB 初始化 & 记录起点坐标 ──
    global origin
    uwb = _ensure_uwb()
    if uwb is not None and uwb.get_frame_count() > 0:
        origin = uwb.get_position()  
        print("  [ORIGIN] 起点坐标已记录: ({:.1f}, {:.1f})".format(
            origin[0], origin[1]))
    else:
        print("  [ORIGIN] UWB 未就绪，使用默认坐标")
        origin = (130.0, 262.0)

    # ── 记录原点后，前进 20cm（航向保持） ──
    pause_encoder_ticker()
    _encoder_reset()
    
    if not _action_startup_forward():
        print("  [STARTUP] 前进 20cm 被中断，准备返航...")
        _safe_return_and_exit()
        return

    stop_all()
    time.sleep_ms(300)
    _encoder_reset()

    # ── 导航到 supplies 固定坐标 ──
    # if not _action_goto_supplies_startup():
    #     print("  [STARTUP] 导航到 supplies 失败，准备返航...")
    #     _safe_return_and_exit()
    #     return

    # ── [Fix 2] 步骤间停顿与重置 ──
    # stop_all()
    # time.sleep_ms(300)
    # _encoder_reset()

    # ── 执行物资区之字形搜索（包含最多 5 次的前移重搜） ──
    found_target = _execute_supplies_search_flow()
    
    if not found_target:
        print("\n  [STARTUP] 整个物资区（包含前移重试）检索完毕，未发现任何目标！开始返航并结束...")
        _safe_return_and_exit()
        return

    print("  [STARTUP] 成功捕获目标，正式进入工作循环")

    # ═══════════════════════════════════════════════════════
    #  主干工作循环（退出条件：中途出错或搜完无目标返航）
    # ═══════════════════════════════════════════════════════
    cycle = 0
    while True:
        cycle += 1
        print("\n  [CYCLE] === 第 {} 轮工作开始 ===".format(cycle))

        stop_all()
        time.sleep_ms(300)
        _encoder_reset()

        # ① 靠近目标（支持手动拨 SW2 旁路直接向下运行）
        if not see_and_push():
            print("  [CYCLE] 视觉跟随/从车通信中断，退出主循环...")
            break

        stop_all()
        time.sleep_ms(300)
        _encoder_reset()

        # ② 全速后退至黄线停车
        if not _action_forward_until_yellow():
            print("  [CYCLE] 黄线检测中断/超时，退出主循环...")
            break

        stop_all()
        time.sleep_ms(300)
        _encoder_reset()

        # ── 后退完成后发送转向蓝牙信号（SW2 拨码同样支持手动旁路） ──
        bt_success = False
        while True:
            # 拨码直接绕过
            if check_sw2():
                print("  [CYCLE] 检测到 SW2 手动旁路信号，强制跳过转向等待，向下进行代码！")
                bt_success = True
                break
            try:
                bt.turn_left()
                print("  [CYCLE] turn_left 指令已发出，等待从车 ok...")
                
                status = _wait_for_follower_ok(timeout_ms=WAIT_OK_TIMEOUT_MS)
                if status == "ok":
                    print("  [CYCLE] 从车状态确认完成")
                    bt_success = True
                    break
                elif status == "abort":
                    print("  [CYCLE] 检测到 SW2 拨码中断信号，强制跳过转向等待，向下进行代码！")
                    bt_success = True  # 手动设置确认成功
                    break
                else:
                    print("  [CYCLE] 等待从车回复 ok 超时，重新发送指令...")
            except Exception as e:
                print("  [CYCLE] 蓝牙转向通信异常:", e)
                time.sleep_ms(500)

        if not bt_success:
            print("  [CYCLE] 转向通信流程被中断，退出主循环...")
            break

        stop_all()
        time.sleep_ms(300)
        _encoder_reset()

        # ③ 重新导航回到 supplies 固定起点坐标
        if not _action_goto_supplies_startup():
            print("  [CYCLE] 导航到 supplies 失败，退出主循环...")
            break

        stop_all()
        time.sleep_ms(300)
        _encoder_reset()

        # ④ 再次进行复合物资区重试搜索
        if not _execute_supplies_search_flow():
            print("\n  [CYCLE] 新一轮物资区（重搜）走完未发现任何目标，退出主循环...")
            break

        print("  [CYCLE] 成功重新捕获物品，准备进入下一轮。")

    print("\n  [MAIN] 主流程运行结束，触发自动返航机制...")
    _safe_return_and_exit()


# ═══════════════════════════════════════════════════════════════
#  安全返航并彻底停机退出 REPL 辅助函数
# ═══════════════════════════════════════════════════════════════

def _safe_return_and_exit():
    """
    导航回到 origin 点，清空硬件资源，关闭所有后台定时器，安全退回到 REPL。
    """
    print("\n  [RTN] ➔ 正在返航...")
    stop_all()
    time.sleep_ms(300)
    _encoder_reset()
    
    _action_return_to_origin()

    # 1. 彻底停机
    stop_all()
    _encoder_reset()
    
    # 2. 释放 UWB 物理端口连接
    global _uwb_shared
    if _uwb_shared is not None:
        _uwb_shared.stop()
        _uwb_shared = None
        
    # 3. 🔴 关键修改：绝对不要 resume 中断，而是强制暂停（Pause/Stop）所有定时器！
    try:
        pause_encoder_ticker()  # 暂停编码器定时器，防止后台高频触发
        stop_imu_ticker()       # 暂停 IMU 定时器
    except Exception:
        pass
        
    time.sleep_ms(100)
    print("\n[INFO] 已安全返航至起点。程序运行结束，已安全退回 REPL 命令行。")
    
    # 4. 🔴 关键修改：不要使用 raise SystemExit。正常 return 即可安全降落到 Thonny REPL
    return

# ═══════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════

main()