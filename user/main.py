#  RT1021 — 按键驱动控制

import gc, time, math
from machine import Pin
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
                   pause_encoder_ticker, resume_encoder_ticker, ENC_SCALE,
                   get_encoder_speeds_filtered)
gc.collect()  # 导入关键模块前回收内存

from imu_motion import (update_angle, imu_read_safe,
                         reset_ang_vel_pid, stop_imu_ticker)

def _yaw():
    """动态获取 imu_motion.yaw（避免 from-import 值拷贝）"""
    return __import__('imu_motion').yaw

from key import capture, key_triggered, pet_watchdog
from uwb_position import UWBPosition
from IMU_hold import HeadingHold

gc.collect()  # 导入相机模块前再次回收
from cam_control import CameraController, cam_approach
from uwb_control import goto_location

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
UWB_LAT_SPEED    = 0.50    # 最大平移速度 (m/s)
UWB_X_DEADBAND   = 3.0     # X 方向死区 (cm)
UWB_LAT_P_GAIN   = 0.02    # X 误差 → 平移速度 P 增益
UWB_TIMEOUT_S    = 10.0    # 超时 (s)
UWB_INIT_TIMEOUT_S = 3.0   # UWB 初始化超时 (s)
UWB_CTRL_DT      = 0.02    # 控制周期 (s)

# ── 航向保持（委托给 IMU_hold.HeadingHold）──
#   以下常量为旧版遗留，保留供参考；实际参数由 HeadingHold 管理
HEADING_KP       = 0.15    # 航向偏差 → wz P 增益
HEADING_KI       = 0.003   # 航向积分增益
HEADING_I_LIMIT  = 0.06    # 积分项限幅
HEADING_DEADBAND = 2.0     # 航向死区 (度)
WZ_LIMIT         = 0.3     # wz 限幅 (归一化值)

_heading_hold = None        # HeadingHold 实例（首次 _lock_yaw 时懒初始化）

# ── 启动: 记录原点后前进 20cm ──
STARTUP_FORWARD_DIST_CM = 20.0   # 目标距离 (cm)
STARTUP_FORWARD_SPEED   = 0.50   # 前进速度 (m/s)
STARTUP_TIMEOUT_S      = 5.0     # 超时 (s)
STARTUP_HEADING_DB     = 5.0     # 航向死区 (度)，偏差 5° 后自动回正

# ── 启动: 靠近目标 + 蓝牙信号 ──
BT_WAIT_DEADLINE_S       = 15.0    # 蓝牙阻塞等待兜底超时 (s)，防止死锁

# ── 启动: 收到 ok 后全速前进至 UWB X 距离 < -130cm ──
STARTUP_FULL_SPEED     = 1.00     # 全速前进速度（绝对值，m/s）
UWB_X_THRESHOLD_CM     = -130.0   # UWB X 轴距离阈值 (cm)，小于此值触发停车
UWB_X_SLOWDOWN_CM      = -80.0    # X < 此值时开始线性减速，防止冲出 UWB 覆盖
UWB_X_MIN_SPEED        = 0.25     # 接近阈值时的最低速度 (m/s)
UWB_X_TIMEOUT_S        = 15.0     # UWB X 距离检测超时 (s)
UWB_BACKUP_DIST_CM     = 20.0     # 触发后倒退距离 (cm)
UWB_BACKUP_SPEED       = 0.50     # 倒退速度 (m/s)
UWB_BACKUP_TIMEOUT_S   = 8.0      # 倒退超时 (s)
UWB_DEAD_TIMEOUT_S     = 2.0      # UWB 掉线容忍超时 (s)
UWB_DEAD_RECONNECT_MAX  = 4       # UWB 掉线最大重连次数

# ── 启动: 向 supplies 坐标靠近（UWB 直接 XY 控制） ──
SUPPLIES_KP           = 0.009   # 位置 P 增益 (降低以减缓接近终点时的冲力，防止过冲)
SUPPLIES_DB           = 10.0    # 到位死区 (cm)，略微放大使小车更容易平稳停下
SUPPLIES_SLOW_DIST    = 20.0    # 减速距离 (cm)
SUPPLIES_MAX_SPEED    = 0.50    # 最大速度 (m/s)
SUPPLIES_TIMEOUT_S    = 20.0    # 超时 (s)
SUPPLIES_CTRL_DT      = 0.01    # 控制周期 (s)，参考 main_uwb.py
SUPPLIES_ARRIVAL_FRAMES   = 5      # 连续 N 帧在死区内算到达 supplies
SUPPLIES_RETRY_MAX       = 3      # 导航到 supplies 失败最大重试次数

# ── 物质区之字形搜索 ──
SEARCH_AREA_SIZE_CM      = 100.0   # 搜索区域边长 (cm)
SEARCH_ROW_STEP_CM       = 40.0    # 行间步进距离 (cm)
SEARCH_SPEED             = 0.40    # 搜索速度 (m/s)
SEARCH_ROW_TIMEOUT_S     = 30.0    # 单行搜索超时 (s)
SEARCH_STEP_TIMEOUT_S    = 10.0    # 步进超时 (s)
SEARCH_CTRL_DT           = 0.02    # 控制周期 (s)

# ── SW2 ──
SW2_DEBOUNCE_MS  = 50

# ── UWB 坐标记录 ──
origin   = None    # 起点坐标 (x, y)
supplies = (50.0, 210.0)    # 物资点固定坐标 (x, y)
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


def is_sw2_active():
    """直接读取 SW2 拨码物理电平（拉低=激活），用于旁路判断。
    与 check_sw2() 不同：此函数不消费边缘事件，任何时刻调用都反映当前物理状态。"""
    return sw2.value() == 0


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
_maintain_yaw_fail = 0   # _maintain_yaw() 驱动异常计数（SPI 总线故障等）


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
            print("  [IMU] ok={}/fail={} yaw={:.2f}° gz_raw=({:+.1f},{:+.1f},{:+.1f})".format(
                _imu_ok_count, _imu_fail_count, _yaw(), d[3], d[4], d[5]))
        return True
    _imu_fail_count += 1
    if _imu_fail_count == 1:
        print("  [IMU] 首次读取失败（imu_read_safe 返回 None）")
    if _imu_fail_count % 50 == 0:
        print("  [IMU] fail count={} yaw={:.2f}°".format(_imu_fail_count, _yaw()))
    return False


def _get_heading_hold():
    """懒初始化 HeadingHold 实例（无 IMU 模式，由调用方外部喂 yaw）"""
    global _heading_hold
    if _heading_hold is None:
        _heading_hold = HeadingHold()
    return _heading_hold


def _lock_yaw():
    global _TARGET_HEADING
    if _TARGET_HEADING is not None:
        return _TARGET_HEADING
    hold = _get_heading_hold()
    for _ in range(20):  
        if _read_imu_update_yaw():
            _TARGET_HEADING = _yaw()
            hold.set_target(_TARGET_HEADING)
            print("  [HEADING] 全程航向锁定: {:.1f}°（偏差自动回正）".format(_TARGET_HEADING))
            return _TARGET_HEADING
        time.sleep_ms(10)
    _TARGET_HEADING = _yaw()
    hold.set_target(_TARGET_HEADING)
    print("  [HEADING] 全程航向锁定(兜底): {:.1f}°".format(_TARGET_HEADING))
    return _TARGET_HEADING


def _heading_correction(target_yaw, deadband=None, dt=None):
    """PI 航向修正：委托给 IMU_hold.HeadingHold"""
    if dt is None:
        dt = FWD_CTRL_DT
    _read_imu_update_yaw()  
    hold = _get_heading_hold()
    if target_yaw != hold.target:
        hold.set_target(target_yaw)
    wz, _, _ = hold.update(_yaw(), dt, deadband=deadband)
    return wz


def _maintain_yaw(target_heading):
    """单次 yaw 保持迭代：委托给 HeadingHold 计算 wz → 驱动电机。
    返回: True=已驱动, False=在死区内无需驱动
    """
    global _maintain_yaw_fail
    hold = _get_heading_hold()
    if target_heading != hold.target:
        hold.set_target(target_heading)

    _read_imu_update_yaw()
    wz, _, _ = hold.update(_yaw(), FWD_CTRL_DT)

    # 死区内释放电机，避免零速闭环引起高频颤抖
    if abs(wz) < 0.005:
        stop_all()
        return True

    try:
        rc = get_encoder_counts()
        if rc is not None and len(rc) >= 4:
            rs = [rc[i] / ENC_SCALE[i] / 0.02 if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(0, 0, wz, rs, 0.02)
    except Exception as e:
        _maintain_yaw_fail += 1
        if _maintain_yaw_fail == 1:
            print("  [YAW] _maintain_yaw 驱动异常(首次):", e)
        elif _maintain_yaw_fail % 50 == 0:
            print("  [YAW] _maintain_yaw 驱动异常累计={} 最近:".format(_maintain_yaw_fail), e)
        return False

    return True


def _pause_with_yaw_hold(target_heading, duration_ms):
    """暂停 duration_ms 毫秒，期间持续保持航向修正。
    确保暂停期间 yaw 不漂移。"""
    deadline = time.ticks_ms() + duration_ms
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        _maintain_yaw(target_heading)
        time.sleep_ms(10)
    stop_all()


def _encoder_reset():
    reset_encoder_filter()
    reset_wheel_pi()
    reset_ang_vel_pid()
    # 每步结束清零 HeadingHold PID 积分，防止碰撞后积分饱和扭动
    if _heading_hold is not None:
        _heading_hold.reset()
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
                if _TARGET_HEADING is not None:
                    _maintain_yaw(_TARGET_HEADING)  # 等待期间持续修正航向
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
    """如果 UWB 硬件断连（UART 死），重置以便下次 _ensure_uwb() 重建。"""
    global _uwb_shared
    if _uwb_shared is not None and _uwb_shared.is_timeout():
        if _uwb_shared.is_uart_alive():
            print("  [UWB] 帧超时但 UART 硬件正常，等待恢复...")
            return  # 不销毁实例，等帧自然恢复
        print("  [UWB] UART 硬件断连，重置连接...")
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
#  通用前进函数（编码器里程计 + IMU 航向闭环）
# ═══════════════════════════════════════════════════════════════

def _forward_distance(dist_m, speed, timeout_s, heading_deadband=None, label="FWD"):
    """通用前进：编码器距离闭环 + IMU 航向保持。"""
    print("  [{}] 前进 {:.0f}cm 开始...".format(label, dist_m * 100))

    target_heading = _lock_yaw()
    db_str = "  死区 {:.0f}°".format(heading_deadband) if heading_deadband is not None else ""
    print("  [{}] 航向锁定: {:.1f}°{}".format(label, target_heading, db_str))

    total_wheel_dists = [0.0, 0.0, 0.0, 0.0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0

    led.value(1)

    while True:
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > timeout_s:
            print("  [{}] 超时 ({:.1f}s)".format(label, elapsed))
            led.value(0)
            return False

        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        # 对单次增量脉冲取绝对值累加，彻底规避左右轮极性抵消的致命 Bug
        for i in range(4):
            if ENC_SCALE[i] != 0:
                total_wheel_dists[i] += abs(counts[i]) / abs(ENC_SCALE[i])
        avg_dist = sum(total_wheel_dists) / len(total_wheel_dists)

        if avg_dist >= dist_m:
            print("  [{}] 到达目标！dist={:.2f}m".format(label, avg_dist))
            led.value(0)
            return True

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [{}] dist={:.2f}m / {:.2f}m  yaw={:.2f}°".format(
                label, avg_dist, dist_m, _yaw()))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        wz = _heading_correction(target_heading, deadband=heading_deadband)

        try:
            rs = [counts[i] / ENC_SCALE[i] / FWD_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(speed, 0, wz, rs, FWD_CTRL_DT)
        except Exception as e:
            print("  [{}] 驱动错误:".format(label), e)

        time.sleep_ms(int(FWD_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  C14 步骤 1: 前进 20cm
# ═══════════════════════════════════════════════════════════════

def _action_forward_20cm():
    return _forward_distance(FORWARD_DIST_M, FORWARD_SPEED, FWD_TIMEOUT_S, label="FWD")


# ═══════════════════════════════════════════════════════════════
#  启动步骤: 前进 20cm
# ═══════════════════════════════════════════════════════════════

def _action_startup_forward():
    return _forward_distance(
        STARTUP_FORWARD_DIST_CM / 100.0, STARTUP_FORWARD_SPEED,
        STARTUP_TIMEOUT_S, STARTUP_HEADING_DB, label="STARTUP"
    )


# ═══════════════════════════════════════════════════════════════
#  辅助倒退执行闭环函数
# ═══════════════════════════════════════════════════════════════

def _execute_backup(target_heading):
    """执行倒退 UWB_BACKUP_DIST_CM 厘米的闭环辅助函数"""
    print("  [BACKUP] 开始倒退 {:.0f}cm...".format(UWB_BACKUP_DIST_CM))
    backup_dist_m = UWB_BACKUP_DIST_CM / 100.0
    backup_dists = [0.0, 0.0, 0.0, 0.0]
    backup_start_ms = time.ticks_ms()

    while True:
        if _abort_check():
            return

        b_elapsed = time.ticks_diff(time.ticks_ms(), backup_start_ms) / 1000.0
        if b_elapsed > UWB_BACKUP_TIMEOUT_S:
            print("  [BACKUP] 倒退超时 ({:.1f}s)，放弃倒退".format(b_elapsed))
            break

        bcounts = get_encoder_counts()
        if bcounts is None or len(bcounts) < 4:
            time.sleep_ms(5)
            continue

        # 倒退时同样采用绝对值累加，防止倒退距离计算抵消
        for i in range(4):
            if ENC_SCALE[i] != 0:
                backup_dists[i] += abs(bcounts[i]) / abs(ENC_SCALE[i])
        avg_backup = sum(backup_dists) / len(backup_dists)

        if abs(avg_backup) >= backup_dist_m:
            print("  [BACKUP] 倒退完成 dist={:.2f}m".format(abs(avg_backup)))
            break

        wz_b = _heading_correction(target_heading)
        try:
            brs = [bcounts[i] / ENC_SCALE[i] / FWD_CTRL_DT
                   if ENC_SCALE[i] != 0 else 0 for i in range(4)]
            omni_drive_closed_loop(-UWB_BACKUP_SPEED, 0, wz_b, brs, FWD_CTRL_DT)
        except Exception as e:
            print("  [BACKUP] 倒退驱动错误:", e)

        time.sleep_ms(int(FWD_CTRL_DT * 1000))

    stop_all()
    time.sleep_ms(300)
    _encoder_reset()


# ═══════════════════════════════════════════════════════════════
#  修改重构: 摄像头跟随靠近
# ═══════════════════════════════════════════════════════════════

def see_and_push():
    """
    完全复用 test_follow.py 的高平滑度控制链（动态实际 dt + 一阶低通滤波轮速 speeds），
    同时保留 main.py 必需的自动到达判定、蓝牙从车通信与状态机跳转。
    """
    print("\n  [APPROACH] === 靠近目标 & 蓝牙信号 ===")

    cam = _ensure_cam()
    
    # ── 🟢 引入动态时间状态，用于消除由于相机串口数据变延迟引起的轮速闭环剧烈抖动 ──
    drive_state = {"last_ms": None}

    # ── 回调函数：完全复用 test_follow.py 的高品质控制 ──
    def _lock_fn():
        return _lock_yaw()

    def _wz_fn(target, deadband):
        # 保持 100Hz (10ms) 的高响应航向更新
        return _heading_correction(target, deadband=deadband, dt=0.01)

    def _abort_fn():
        pet_watchdog()
        if check_sw2():
            print("  [SW2] Abort requested")
            return True
        return False

    def _drive_fn(vx, vy, wz, dt):
        # ── 🟢 完全复用 test_follow.py 的动态时间步理解算 ──
        now = time.ticks_ms()
        last_ms = drive_state["last_ms"]
        if last_ms is None:
            actual_dt = dt
        else:
            actual_dt = time.ticks_diff(now, last_ms) / 1000.0
            if actual_dt <= 0.001:  # 防止极限除零
                actual_dt = dt
        drive_state["last_ms"] = now

        rc = get_encoder_counts()
        if rc is not None and len(rc) >= 4:
            # ── 🟢 核心：完全复用 test_follow.py 的一阶低通滤波反馈 speeds ──
            speeds = get_encoder_speeds_filtered(actual_dt)
            omni_drive_closed_loop(vx, vy, wz, speeds, actual_dt)

    def _stop_fn():
        stop_all()

    def _led_fn(val):
        led.value(val)

    # 在靠近前重置一下时间状态
    drive_state["last_ms"] = None

    # ── 1. 调用底层的 cam_approach 靠近目标直到到达判定 ──
    arrived, reason = cam_approach(
        cam, _lock_fn, _wz_fn, _abort_fn,
        _drive_fn, _stop_fn, _led_fn
    )

    if not arrived:
        return False

    # ── 2. 释放 LED ──
    led.value(0)

    # ── 3. 蓝牙发送与等待确认闭环（main.py 必需的竞赛任务） ──
    if check_sw2():
        print("  [APPROACH] 检测到 SW2 手动中断，强制跳过蓝牙等待")
        return True

    print("  [APPROACH] 向从车发送数字 0 (turn_right)...")
    try:
        bt.turn_right()
        target_heading = _lock_yaw()
        print("  [APPROACH] 指令已发，等待从车 ok（兜底 {}s）...".format(BT_WAIT_DEADLINE_S))
        bt_wait_start = time.ticks_ms()
        while True:
            pet_watchdog()
            _maintain_yaw(target_heading)  # 等待期间持续修正航向
            if time.ticks_diff(time.ticks_ms(), bt_wait_start) > BT_WAIT_DEADLINE_S * 1000:
                print("  [APPROACH] 等待从车 ok 兜底超时 ({}s)，继续执行".format(BT_WAIT_DEADLINE_S))
                return True
            if check_sw2():
                print("  [APPROACH] SW2 中断，强制跳过")
                return True
            resp = bt.read_response()
            if resp == "ok":
                print("  [APPROACH] 从车确认完毕 (ok)")
                return True
            time.sleep_ms(10)
    except Exception as e:
        print("  [APPROACH] 蓝牙发送异常:", e)
        return False


# ═══════════════════════════════════════════════════════════════
#  核心修复: 全速前进至 UWB X 距离 < -130cm（支持移动中重连，依靠视觉兜底不停车）
# ═══════════════════════════════════════════════════════════════

def _action_forward_until_uwb_x():
    """
    全速前进，同时 UWB 判断 X 轴距离 或 摄像头识别黄线越界。
    🟢 核心修复：即使 UWB 掉线，小车也绝不停车或返回起点，而是降低到安全速度继续向前推进并尝试在移动中重连，
    同时完全依赖摄像头黄线检测作为绝对物理安全边界进行兜底拦截。
    """
    print("\n  [UWBX] === 全速前进，等待 UWB X < {:.0f}cm ===".format(UWB_X_THRESHOLD_CM))

    uwb = _ensure_uwb()
    if uwb is None:
        print("  [UWBX] 警告：UWB 初始不可用，将在前行移动中建立连接...")

    target_heading = _lock_yaw()
    print("  [UWBX] 航向锁定: {:.1f}°  开始直行推进".format(target_heading))

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    last_uwb_ms = start_ms
    loop_cnt = 0
    uwb_dead_start = 0
    uwb_dead_reconnect = 0

    # ── 摄像头黄线检测初始化 ──
    cam = _ensure_cam()
    cam.reset()
    armed_to_trigger = False
    yellow_lost_count = 0
    BLIND_TOLERANCE = 3
    last_cam_ms = start_ms
    ctrl = None

    led.value(1)

    while True:
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > UWB_X_TIMEOUT_S:
            # 即使掉线导致 UWB X 超时，由于已处于运行中，绝不返回原点，让其继续推进依靠黄线拦截
            print("  [UWBX] 超时 ({:.1f}s)，UWB 未达标，仅依靠视觉黄线推进兜底".format(elapsed))
            pass

        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        now = time.ticks_ms()
        
        # ── 1. UWB 与 摄像头步进（均在移动中高频执行） ──
        if uwb is not None and time.ticks_diff(now, last_uwb_ms) >= 50:
            uwb.step()
            last_uwb_ms = now

        if time.ticks_diff(now, last_cam_ms) >= 50:
            ctrl = cam.step()
            last_cam_ms = now

        # ── 2. 优先保障黄线越界视觉安全（核心安全兜底锁，即使 UWB 掉线，只要视觉检测到越界立刻停车倒退） ──
        line_detected = ctrl.get('line_flag', 0) if ctrl else 0
        if line_detected:
            armed_to_trigger = True
            yellow_lost_count = 0
        elif armed_to_trigger:
            yellow_lost_count += 1

        if armed_to_trigger and yellow_lost_count >= BLIND_TOLERANCE:
            stop_all()
            print("  [UWBX] 黄线越界确认 (连续{}帧) → 触发停车！".format(BLIND_TOLERANCE))
            _execute_backup(target_heading)
            return True

        # ── 3. UWB 掉线判定与移动中自动重建（🟢 核心：不调 stop_all，车不熄火，不重置主流程） ──
        uwb_is_active = True
        if uwb is None or uwb.is_timeout():
            uwb_is_active = False
            if uwb_dead_start == 0:
                uwb_dead_start = time.ticks_ms()
                print("  [UWBX] UWB 数据中断，车身保持安全巡航速度并于移动中尝试重连...")
            elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > UWB_DEAD_TIMEOUT_S * 1000:
                if uwb is not None and uwb.is_uart_alive():
                    print("  [UWBX] UART 仍活跃，延长移动等待 (帧过滤中)...")
                    uwb_dead_start = time.ticks_ms()  # 延长等待时间
                elif uwb_dead_reconnect < UWB_DEAD_RECONNECT_MAX:
                    uwb_dead_reconnect += 1
                    print("  [UWBX] UART 离线，移动中进行第 {}/{} 次硬件重连重建...".format(
                        uwb_dead_reconnect, UWB_DEAD_RECONNECT_MAX))
                    _reset_uwb_if_needed()
                    new_uwb = _ensure_uwb()
                    if new_uwb is not None:
                        uwb = new_uwb
                        uwb_dead_start = 0
                        last_uwb_ms = time.ticks_ms()
                        print("  [UWBX] UWB 移动中重连建立成功！")
                        uwb_is_active = True
                        continue
                    uwb_dead_start = time.ticks_ms()
                else:
                    # 重连用尽后依旧不熄火，保持匀速直行推进，直至触发黄线越界
                    print("  [UWBX] UWB 彻底离线，已进入视觉托管状态，继续安全车速行进...")
                    uwb_dead_start = time.ticks_ms()
        else:
            if uwb_dead_start != 0:
                print("  [UWBX] UWB 链路已自主恢复")
            uwb_dead_start = 0
            uwb_dead_reconnect = 0

        # ── 4. 根据 UWB 链路健康度动态决策车速 ──
        fwd_speed = UWB_X_MIN_SPEED
        if uwb_is_active:
            try:
                x_cm, y_cm = uwb.get_position()
                
                # UWB 正常到达
                if x_cm < UWB_X_THRESHOLD_CM:
                    stop_all()
                    print("  [UWBX] UWB X={:.1f}cm < {:.0f}cm，坐标触发停车！".format(
                        x_cm, UWB_X_THRESHOLD_CM))
                    _execute_backup(target_heading)
                    return True

                # 线性减速
                if x_cm < UWB_X_SLOWDOWN_CM:
                    slowdown_range = UWB_X_SLOWDOWN_CM - UWB_X_THRESHOLD_CM
                    ratio = (UWB_X_SLOWDOWN_CM - x_cm) / slowdown_range
                    ratio = max(0.0, min(1.0, ratio))
                    fwd_speed = STARTUP_FULL_SPEED - ratio * (STARTUP_FULL_SPEED - UWB_X_MIN_SPEED)
                else:
                    fwd_speed = STARTUP_FULL_SPEED
            except Exception as e:
                print("  [UWBX] get_position() 读取异常:", e)
                fwd_speed = UWB_X_MIN_SPEED
        else:
            # 🟢 掉线期间的安全匀速直行速度：继续推进
            fwd_speed = 0.35

        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            st = "HEALTHY" if uwb_is_active else "DROPPED(RUNNING)"
            print("  [UWBX] UWB_Link={} v={:.2f}m/s yaw={:.2f}° t={:.1f}s".format(
                st, fwd_speed, _yaw(), elapsed))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        wz = _heading_correction(target_heading)

        try:
            rs = [counts[i] / ENC_SCALE[i] / FWD_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(fwd_speed, 0, wz, rs, FWD_CTRL_DT)
        except Exception as e:
            print("  [UWBX] 驱动运行错误:", e)

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

    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    def _lock_fn():
        return _lock_yaw()

    def _wz_fn(target, db):
        return _heading_correction(target, deadband=db, dt=SUPPLIES_CTRL_DT)

    def _yaw_fn():
        return _yaw()

    def _abort_fn():
        pet_watchdog()
        if check_sw2():
            print("  [SW2] Abort requested")
            return True
        return False

    def _drive_fn(vx, vy, wz, dt):
        rc = get_encoder_counts()
        if rc is not None and len(rc) >= 4:
            rs = [rc[i] / ENC_SCALE[i] / dt if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(vx, vy, wz, rs, dt)

    def _stop_fn():
        stop_all()

    def _led_fn(val):
        led.value(val)

    arrived, reason = goto_location(
        uwb, target_x, target_y,
        _lock_fn, _wz_fn, _yaw_fn,
        _abort_fn, _drive_fn, _stop_fn,
        _led_fn, label="GOTO"
    )
    return arrived


# ═══════════════════════════════════════════════════════════════
#  返回原点: UWB 导航到 origin 坐标
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

    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    def _lock_fn():
        return _lock_yaw()

    def _wz_fn(target, db):
        return _heading_correction(target, deadband=db, dt=SUPPLIES_CTRL_DT)

    def _yaw_fn():
        return _yaw()

    def _abort_fn():
        pet_watchdog()
        if check_sw2():
            print("  [SW2] Abort requested")
            return True
        return False

    def _drive_fn(vx, vy, wz, dt):
        rc = get_encoder_counts()
        if rc is not None and len(rc) >= 4:
            rs = [rc[i] / ENC_SCALE[i] / dt if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(vx, vy, wz, rs, dt)

    def _stop_fn():
        stop_all()

    def _led_fn(val):
        led.value(val)

    arrived, reason = goto_location(
        uwb, target_x, target_y,
        _lock_fn, _wz_fn, _yaw_fn,
        _abort_fn, _drive_fn, _stop_fn,
        _led_fn, label="RTN"
    )
    return arrived


# ═══════════════════════════════════════════════════════════════
#  启动步骤: 之字形搜索 100cm×100cm 物质区
# ═══════════════════════════════════════════════════════════════

def _action_search_supplies_area():
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
        total_dists = [0.0, 0.0, 0.0, 0.0]
        row_start_ms = time.ticks_ms()
        row_last_print_ms = row_start_ms
        row_loop_cnt = 0

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

            # 对单次增量脉冲取绝对值累加，规避镜像接线极性抵消
            for i in range(4):
                if ENC_SCALE[i] != 0:
                    total_dists[i] += abs(counts[i]) / abs(ENC_SCALE[i])
            avg_dist = sum(total_dists) / len(total_dists)

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

            row_loop_cnt += 1
            if row_loop_cnt % 20 == 0:
                gc.collect()

            time.sleep_ms(int(SEARCH_CTRL_DT * 1000))

        # ── 行间步进 ──
        if row >= MAX_ROWS - 1:
            break  

        print("  [SEARCH] 行间步进 {:.0f}cm...".format(STEP_CM))
        stop_all()
        time.sleep_ms(200)

        step_dist_m = STEP_CM / 100.0  
        step_dists = [0.0, 0.0, 0.0, 0.0]
        step_start_ms = time.ticks_ms()
        step_last_print_ms = step_start_ms
        step_loop_cnt = 0

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

            # 对单次增量脉冲取绝对值累加，规避镜像接线极性抵消
            for i in range(4):
                if ENC_SCALE[i] != 0:
                    step_dists[i] += abs(counts[i]) / abs(ENC_SCALE[i])
            avg_dist = sum(step_dists) / len(step_dists)

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

            step_loop_cnt += 1
            if step_loop_cnt % 20 == 0:
                gc.collect()

            time.sleep_ms(int(SEARCH_CTRL_DT * 1000))

        stop_all()
        time.sleep_ms(200)

    print("  [SEARCH] 全部 {:.0f}cm×{:.0f}cm 区域搜索完成，未检测到物品".format(
        AREA_CM, AREA_CM))
    led.value(0)
    return False


# ═══════════════════════════════════════════════════════════════
#  物资区搜索流
# ═══════════════════════════════════════════════════════════════

def _execute_supplies_search_flow():
    return _action_search_supplies_area()


# ═══════════════════════════════════════════════════════════════
#  C14 步骤 2: UWB 平移（航向保持）- 支持掉线惯性滑动与重连
# ═══════════════════════════════════════════════════════════════

def _action_uwb_translate():
    print("  [UWB] Starting translate toward anchor...")

    target_heading = _lock_yaw()
    print("  [UWB] Heading locked: {:.1f}°".format(target_heading))

    uwb = _ensure_uwb()
    if uwb is None:
        print("  [UWB] 警告：UWB 初始不可用，将在前行移动中建立连接...")

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    uwb_dead_start = 0
    uwb_dead_reconnect = 0

    led.value(1)

    last_fwd_speed = 0.0

    while True:
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > UWB_TIMEOUT_S:
            print("  [UWB] 平移超时 ({:.1f}s)，结束本次平移动作".format(elapsed))
            led.value(0)
            return True # 平移作为辅助对齐动作，超时也不报错返回，继续后续逻辑

        if uwb is not None:
            uwb.step()
            try:
                x_cm, y_cm = uwb.get_position()
            except Exception as e:
                print("  [UWB] get_position() 异常:", e)
                _maintain_yaw(target_heading)
                time.sleep_ms(50)
                continue

        # UWB 平移掉线保护（惯性前移并后台重建）
        uwb_is_active = True
        if uwb is None or uwb.is_timeout():
            uwb_is_active = False
            if uwb_dead_start == 0:
                uwb_dead_start = time.ticks_ms()
                print("  [UWB] 数据中断，进行姿态锁定重连...")
            elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > UWB_DEAD_TIMEOUT_S * 1000:
                if uwb is not None and uwb.is_uart_alive():
                    print("  [UWB] UART 活跃，延长等待中...")
                    uwb_dead_start = time.ticks_ms()
                elif uwb_dead_reconnect < UWB_DEAD_RECONNECT_MAX:
                    uwb_dead_reconnect += 1
                    print("  [UWB] UART 掉线，第 {}/{} 次移动中硬件复位重建...".format(
                        uwb_dead_reconnect, UWB_DEAD_RECONNECT_MAX))
                    _reset_uwb_if_needed()
                    new_uwb = _ensure_uwb()
                    if new_uwb is not None:
                        uwb = new_uwb
                        uwb_dead_start = 0
                        print("  [UWB] UWB 移动中重连成功！")
                        uwb_is_active = True
                    else:
                        uwb_dead_start = time.ticks_ms()
                else:
                    print("  [UWB] 硬件重连均失败，以极低平移惯性盲走靠拢...")
                    uwb_dead_start = time.ticks_ms()
        else:
            uwb_dead_start = 0
            uwb_dead_reconnect = 0

        # 平移策略：掉线时使用衰减的最后已知平移速度，配合姿态锁定推进
        if uwb_is_active:
            if abs(x_cm) < UWB_X_DEADBAND:
                print("  [UWB] Centered! X={:.1f}cm (deadband={:.1f}cm)".format(
                     x_cm, UWB_X_DEADBAND))
                led.value(0)
                return True

            fwd_speed = x_cm * UWB_LAT_P_GAIN
            fwd_speed = max(-UWB_LAT_SPEED, min(fwd_speed, UWB_LAT_SPEED))
            last_fwd_speed = fwd_speed
        else:
            fwd_speed = last_fwd_speed * 0.90 # 掉线平移速度指数级衰减，防过度冲刺

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            if uwb_is_active:
                dist_cm, angle_deg = uwb.get_distance_angle()
                print("  [UWB] X={0:+.1f}cm Y={1:.1f}cm D={2:.1f}cm A={3:.1f}°  yaw={4:.2f}°".format(
                    x_cm, y_cm, dist_cm, angle_deg, _yaw()))
            else:
                print("  [UWB] LOST v_lat={:.2f}m/s yaw={:.2f}°".format(fwd_speed, _yaw()))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

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
            print("  [UWB] 驱动错误:", e)

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
        _pause_with_yaw_hold(_TARGET_HEADING, 300)

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
                _maintain_yaw(target_heading)  # 丢目标时仅维持航向

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
    print("  RT1021 — 按键驱动控制（移动重连+黄线视觉兜底版）")
    print("  启动: 原点 → 前进20cm → 导航supplies → 之字形复合重搜")
    print("=" * 50)
    print("  C14 (KEY3): 前进 20cm → UWB 平移")
    print("  C8  (KEY1): 蓝牙发送从车消息")
    print("  C9  (KEY2): 摄像头跟随靠近")
    print("  SW2 (D9)  : 强制退出")
    print("=" * 50)
    print("")
    
    # ── ⚠ 在进行任何闭环控制前，必须先暂停后台编码器定时器，避免底层资源竞争 [1] ──
    pause_encoder_ticker()
    _encoder_reset()

    # ── 🔒 提前锁定航向 ──
    _lock_yaw()

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

    # ── 记录原点后，前进 20cm ──
    if not _action_startup_forward():
        print("  [STARTUP] 前进 20cm 被中断，准备返航...")
        _safe_return_and_exit()
        return

    _pause_with_yaw_hold(_TARGET_HEADING, 100)
    _encoder_reset()

    # ── 导航到 supplies 固定坐标 ──
    if not _action_goto_supplies_startup():
        print("  [STARTUP] 导航到 supplies 失败，准备返航...")
        _safe_return_and_exit()
        return

    _pause_with_yaw_hold(_TARGET_HEADING, 100)
    _encoder_reset()

    # ── 执行物资区之字形搜索 ──
    found_target = _execute_supplies_search_flow()
    
    if not found_target:
        print("\n  [STARTUP] 物资区搜索完毕，未发现任何目标！开始返航并结束...")
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

        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()

        # ① 靠近目标
        if not see_and_push():
            print("  [CYCLE] 视觉跟随/从车通信中断，尝试回 supplies 重新搜索...")
            if not _goto_and_search_supplies():
                break
            continue

        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()

        # ② 全速前进至 UWB X 距离达标后停车并倒退（掉线不熄火，由黄线绝对兜底）
        if not _action_forward_until_uwb_x():
            print("  [CYCLE] 视觉或 UWB 触发异常，尝试回 supplies 重新搜索...")
            if not _goto_and_search_supplies():
                break
            continue

        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()

        # ── 后退完成后发送转向蓝牙信号 ──
        bt_success = False
        if check_sw2():
            print("  [CYCLE] 检测到 SW2 手动旁路信号，强制跳过转向")
            bt_success = True
        else:
            try:
                bt.turn_left()
                target_heading = _lock_yaw()
                print("  [CYCLE] turn_left 指令已发出，等待从车 ok（兜底 {}s）...".format(BT_WAIT_DEADLINE_S))
                bt_wait_start = time.ticks_ms()
                while True:
                    pet_watchdog()
                    _maintain_yaw(target_heading)  # 等待期间持续修正航向
                    if time.ticks_diff(time.ticks_ms(), bt_wait_start) > BT_WAIT_DEADLINE_S * 1000:
                        print("  [CYCLE] 等待从车 ok 兜底超时 ({}s)，继续执行".format(BT_WAIT_DEADLINE_S))
                        bt_success = True
                        break
                    if check_sw2():
                        print("  [CYCLE] 检测到 SW2 拨码中断信号，强制跳过")
                        bt_success = True
                        break
                    resp = bt.read_response()
                    if resp == "ok":
                        print("  [CYCLE] 从车状态确认完成")
                        bt_success = True
                        break
                    time.sleep_ms(10)
            except Exception as e:
                print("  [CYCLE] 蓝牙转向通信异常:", e)

        if not bt_success:
            print("  [CYCLE] 转向通信流程被中断，尝试回 supplies 重新搜索...")
            if not _goto_and_search_supplies():
                break
            continue

        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()

        # ③ 重新导航回到 supplies 固定起点坐标
        if not _retry_goto_supplies():
            print("  [CYCLE] 导航到 supplies 全部重试失败，退出主循环...")
            break

        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()

        # ④ 再次进行物资区搜索
        if not _execute_supplies_search_flow():
            print("\n  [CYCLE] 新一轮物资区搜索未发现任何目标，退出主循环...")
            break

        print("  [CYCLE] 成功重新捕获物品，准备进入下一轮。")

    print("\n  [MAIN] 主流程运行结束，触发自动返航机制...")
    _safe_return_and_exit()


# ═══════════════════════════════════════════════════════════════
#  安全返航并彻底停机退出 REPL 辅助函数
# ═══════════════════════════════════════════════════════════════

def _retry_goto_supplies():
    """带重试的导航到 supplies"""
    for attempt in range(SUPPLIES_RETRY_MAX):
        if attempt > 0:
            print("  [GOTO] 导航到 supplies 失败，第 {}/{} 次重试...".format(
                attempt + 1, SUPPLIES_RETRY_MAX))
            _pause_with_yaw_hold(_TARGET_HEADING, 500)
            _encoder_reset()
        if _action_goto_supplies_startup():
            return True
    print("  [GOTO] 导航到 supplies 全部 {} 次重试均失败".format(SUPPLIES_RETRY_MAX))
    return False


def _goto_and_search_supplies():
    """中间步骤失败后的恢复：导航回 supplies 并执行物资区搜索。"""
    print("\n  [RECOVER] 中间步骤中断，尝试返回 supplies 继续搜索...")
    _pause_with_yaw_hold(_TARGET_HEADING, 300)
    _encoder_reset()

    if not _retry_goto_supplies():
        print("  [RECOVER] 无法导航到 supplies")
        return False

    _pause_with_yaw_hold(_TARGET_HEADING, 100)
    _encoder_reset()

    found = _execute_supplies_search_flow()
    if found:
        print("  [RECOVER] 物资区重新搜索成功，继续下一轮工作循环")
    else:
        print("  [RECOVER] 物资区搜索完全枯竭")
    return found


def _safe_return_and_exit():
    """
    导航回到 origin 点，清空硬件资源，关闭所有后台定时器，安全退回到 REPL。
    """
    print("\n  [RTN] ➔ 正在返航...")
    _pause_with_yaw_hold(_TARGET_HEADING, 300)
    _encoder_reset()
    
    _action_return_to_origin()

    stop_all()
    _encoder_reset()
    
    global _uwb_shared
    if _uwb_shared is not None:
        _uwb_shared.stop()
        _uwb_shared = None
        
    try:
        pause_encoder_ticker()  # 暂停编码器定时器，防止后台高频触发
        stop_imu_ticker()       # 暂停 IMU 定时器
    except Exception:
        pass
        
    time.sleep_ms(100)
    print("\n[INFO] 已安全返航至起点。程序运行结束，已安全退回 REPL 命令行。")
    return

# ═══════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════

main()