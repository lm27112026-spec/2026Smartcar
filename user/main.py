"""
main.py — 按键驱动控制

【按键映射】
  C14 (KEY3) → action_c14(): 前进 20cm → UWB 平移（航向保持）
  C8  (KEY1) → action_c8():  通过蓝牙向从车发送消息
  C9  (KEY2) → action_c9():  摄像头靠近（代码空置）

【函数功能】

  顶层函数:
    main()                          — 主入口，初始化硬件 + 按键循环

  启动流程（main 中依次执行）:
    origin 记录                     — UWB 初始化，读取起点坐标
     _action_startup_forward()       — 前进 10cm（航向保持，5° 偏差自动回正）
     _action_goto_supplies_startup() — UWB 导航到 supplies 固定坐标（航向锁）
     _action_search_supplies_area()  — 之字形搜索 100cm×100cm 物质区（航向锁）
    
    === 首次检测到目标后进入循环（退出条件：搜索物质区未检测到）===
    while True:
        ① see_and_push()                     — 靠近 → turn_right → wait_ok
        ② _action_forward_until_yellow()      — 全速后退至黄线 → turn_left → wait_ok
        ③ _action_goto_supplies_startup()     — UWB 导航到 supplies 固定坐标
        ④ _action_search_supplies_area()      — 之字形搜索 100cm×100cm 物质区
        ⑤ 回到 ①

  按键动作:
    action_c14()                    — KEY3: 前进 20cm 后 UWB 平移
    action_c8()                     — KEY1: 向从车发送蓝牙指令
    action_c9()                     — KEY2: 摄像头靠近动作（空置）
    send_messages()                 — 蓝牙消息发送循环

  底层辅助:
    check_sw2()                     — SW2 消抖检测
    _read_imu_update_yaw()          — 读取 IMU 并更新偏航角
    _lock_yaw()                     — 锁定当前航向为目标
    _heading_correction(target_yaw) — P 控制纠偏（支持自定义 deadband）
    _encoder_reset()                — 编码器复位 + PWM 清零
    _abort_check()                  — SW2 + 超时退出检测
    _ensure_uwb()                   — 确保 UWB 已初始化，返回共享实例（懒加载）
    _reset_uwb_if_needed()          — UWB 超时时自动重置，下次调用重建
    _action_startup_forward()       — 记录原点后前进 10cm（航向保持）
    see_and_push()                  — 靠近目标 → 发数字 0 → 等待从车 ok
    _action_forward_until_yellow()  — 收到 ok 后全速后退，黄线检测后停车
    _action_goto_supplies_startup() — UWB 导航到 supplies 固定坐标（航向锁）
    _action_return_to_origin()     — 循环结束后 UWB 导航回到原点
    _action_forward_20cm()          — C14 步骤 1: 前进 20cm
    _action_uwb_translate()         — C14 步骤 2: UWB 平移纠偏

   参数常量（# ═══════════ 常量 ═══════════ 区域）:
     全局常量定义见下方常量区域（车速、距离、PID 参数等）

  UWB 坐标记录:
    origin                          — 起点坐标 (x, y)，在 main() 启动时通过 uwb_record() 记录
    supplies                        — 物资点固定坐标 (x, y)

【依赖】motor.py, imu_motion.py, key.py, uwb_position.py, uart_master.py, utils.py
【PIT 分配】PIT0=系统, PIT1=编码器(motor), PIT2=看门狗(key), PIT3=IMU
"""

import gc, time, math
from machine import Pin
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
                   pause_encoder_ticker, resume_encoder_ticker, ENC_SCALE)
from imu_motion import (update_angle, imu_get_safe, yaw,
                        reset_ang_vel_pid)
from key import capture, key_triggered, pet_watchdog
from uwb_position import UWBPosition

# ═══════════════════════════════════════════════════════════════
#  常量
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

# ── 启动: 记录原点后前进 10cm ──
STARTUP_FORWARD_DIST_CM = 10.0   # 目标距离 (cm)
STARTUP_FORWARD_SPEED   = 0.30   # 前进速度 (m/s)
STARTUP_TIMEOUT_S      = 5.0     # 超时 (s)
STARTUP_HEADING_DB     = 5.0     # 航向死区 (度)，偏差 5° 后自动回正

# ── 启动: 靠近目标 + 蓝牙信号 ──
WAIT_OK_TIMEOUT_MS       = 5000    # 等待从车 ok 应答超时 (ms)

# ── 启动: 收到 ok 后全速后退至黄线 ──
STARTUP_FULL_SPEED     = 1.00    # 全速后退 (m/s)
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
origin   = None    # 起点坐标 (x, y)，在特定时间通过 uwb_record() 记录
supplies = (50.0, 210.0)    # 物资点固定坐标 (x, y)
_uwb_shared = None  # 共享 UWBPosition 实例，通过 _ensure_uwb() 懒加载

# ═══════════════════════════════════════════════════════════════
#  硬件初始化
# ═══════════════════════════════════════════════════════════════

led = Pin(LED_PIN, Pin.OUT, value=False)
sw2 = Pin(SW2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)

# ── SW2 消抖状态 ──
_sw2_last         = sw2.value()
_sw2_changed      = False
_sw2_stable_start = 0


def check_sw2():
    """SW2 消抖检测（50ms 时间窗口）"""
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
#  工具函数
# ═══════════════════════════════════════════════════════════════

def _read_imu_update_yaw():
    """安全读取 IMU 并更新 yaw，返回是否成功"""
    d = imu_get_safe()
    if d is not None:
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        return True
    return False


def _lock_yaw():
    """锁定当前航向角，等待有效 IMU 数据后返回 yaw"""
    for _ in range(20):  # 最多等 200ms
        if _read_imu_update_yaw():
            return yaw
        time.sleep_ms(10)
    # 兜底：返回当前 yaw（可能是 0.0）
    return yaw


def _heading_correction(target_yaw, deadband=None):
    """
    计算航向纠偏 wz，使当前 yaw 趋近 target_yaw。
    
    参数:
        target_yaw: 目标航向角 (度)
        deadband:   航向死区 (度)，默认使用 HEADING_DEADBAND
    
    返回: wz (归一化值，-1~1)
    """
    if not _read_imu_update_yaw():
        return 0.0

    if deadband is None:
        deadband = HEADING_DEADBAND

    error = target_yaw - yaw
    # 角度差归一化到 [-180, 180]
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    if abs(error) < deadband:
        return 0.0

    wz = error * HEADING_KP
    return max(-WZ_LIMIT, min(wz, WZ_LIMIT))


def _encoder_reset():
    """重置编码器滤波器和轮子 PI，清空编码器缓冲区"""
    reset_encoder_filter()
    reset_wheel_pi()
    reset_ang_vel_pid()
    # 清空编码器缓冲区中的残余计数
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)


def _abort_check():
    """检查 SW2 是否触发退出，并喂狗。
    返回: True = 应中断当前动作
    """
    pet_watchdog()
    if check_sw2():
        print("  [SW2] Abort requested")
        return True
    return False


# ═══════════════════════════════════════════════════════════════
#  UWB 共享实例管理
# ═══════════════════════════════════════════════════════════════

def _ensure_uwb():
    """确保 UWB 已初始化，返回共享实例。首次调用时创建并等待首帧数据。"""
    global _uwb_shared
    _reset_uwb_if_needed()
    if _uwb_shared is None:
        try:
            _uwb_shared = UWBPosition(uart_id=0, baudrate=115200, target_anchor="8834")
            print("  [UWB] 初始化中，等待首帧...")
            wait_start = time.ticks_ms()
            while _uwb_shared.get_frame_count() == 0:
                _uwb_shared.step()
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
#  C14 步骤 1: 前进 20cm（航向保持）
# ═══════════════════════════════════════════════════════════════

def _action_forward_20cm():
    """前进 20cm，IMU 航向闭环保持。返回: True=正常完成, False=中断"""
    print("  [FWD] Starting forward {:.0f}cm...".format(FORWARD_DIST_M * 100))

    # 锁定初始航向
    target_heading = _lock_yaw()
    print("  [FWD] Heading locked: {:.1f}°".format(target_heading))

    # 距离累计（4 轮脉冲绝对值和）
    total_pulses = [0, 0, 0, 0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0

    led.value(1)

    while True:
        # ── 超时 / 退出检查 ──
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > FWD_TIMEOUT_S:
            print("  [FWD] Timeout ({:.1f}s)".format(elapsed))
            led.value(0)
            return False

        # ── 读取编码器（单次调用，共用脉冲数据） ──
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        # 累计距离 (m) — 用于到达判定
        for i in range(4):
            total_pulses[i] += abs(counts[i])

        wheel_dists = []
        for i in range(4):
            if ENC_SCALE[i] != 0:
                wheel_dists.append(total_pulses[i] / abs(ENC_SCALE[i]))
        avg_dist = sum(wheel_dists) / len(wheel_dists) if wheel_dists else 0.0

        # ── 到达判定 ──
        if avg_dist >= FORWARD_DIST_M:
            print("  [FWD] Target reached! dist={:.2f}m pulses={}".format(
                avg_dist, total_pulses))
            led.value(0)
            return True

        # ── 进度打印（每 500ms） ──
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [FWD] dist={:.2f}m / {:.2f}m  yaw={:.1f}°".format(
                avg_dist, FORWARD_DIST_M, yaw))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        # ── 航向纠偏 ──
        wz = _heading_correction(target_heading)

        # ── 闭环驱动: vx=前进速度, vy=0（无横向）, wz=航向纠偏 ──
        # 使用同一次 get_encoder_counts() 的 counts 计算速度
        try:
            rs = [counts[i] / ENC_SCALE[i] / FWD_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(FORWARD_SPEED, 0, wz, rs, FWD_CTRL_DT)
        except Exception as e:
            print("  [FWD] Drive error:", e)

        time.sleep_ms(int(FWD_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  启动步骤: 记录原点后前进 10cm（航向保持，5° 偏差回正）
# ═══════════════════════════════════════════════════════════════

def _action_startup_forward():
    """前进 10cm，IMU 航向闭环保持（5° 偏差自动回正）。返回: True=正常完成, False=中断"""
    dist_m = STARTUP_FORWARD_DIST_CM / 100.0
    print("  [STARTUP] 前进 {:.0f}cm 开始...".format(STARTUP_FORWARD_DIST_CM))

    # 锁定初始航向
    target_heading = _lock_yaw()
    print("  [STARTUP] 航向锁定: {:.1f}°  死区 {:.0f}° 偏差自动回正".format(
        target_heading, STARTUP_HEADING_DB))

    # 距离累计（4 轮脉冲绝对值和）
    total_pulses = [0, 0, 0, 0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0

    led.value(1)

    while True:
        # ── 超时 / 退出检查 ──
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > STARTUP_TIMEOUT_S:
            print("  [STARTUP] 超时 ({:.1f}s)".format(elapsed))
            led.value(0)
            return False

        # ── 读取编码器（单次调用，共用脉冲数据） ──
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        # 累计距离 (m) — 用于到达判定
        for i in range(4):
            total_pulses[i] += abs(counts[i])

        wheel_dists = []
        for i in range(4):
            if ENC_SCALE[i] != 0:
                wheel_dists.append(total_pulses[i] / abs(ENC_SCALE[i]))
        avg_dist = sum(wheel_dists) / len(wheel_dists) if wheel_dists else 0.0

        # ── 到达判定 ──
        if avg_dist >= dist_m:
            print("  [STARTUP] 到达目标! dist={:.2f}m pulses={}".format(
                avg_dist, total_pulses))
            led.value(0)
            return True

        # ── 进度打印（每 500ms） ──
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [STARTUP] dist={:.2f}m / {:.2f}m  yaw={:.1f}°".format(
                avg_dist, dist_m, yaw))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        # ── 航向纠偏（5° 死区内不动作，超过则自动回正） ──
        wz = _heading_correction(target_heading, deadband=STARTUP_HEADING_DB)

        # ── 闭环驱动: vx=前进速度, vy=0, wz=航向纠偏 ──
        try:
            rs = [counts[i] / ENC_SCALE[i] / FWD_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(STARTUP_FORWARD_SPEED, 0, wz, rs, FWD_CTRL_DT)
        except Exception as e:
            print("  [STARTUP] 驱动错误:", e)

        time.sleep_ms(int(FWD_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  启动步骤: 靠近目标 → 蓝牙信号通知从车
# ═══════════════════════════════════════════════════════════════

def see_and_push():
    """
    摄像头检测到目标后：靠近目标 → 到达指定距离 → turn_right 发数字 0 → 等待从车 ok。
    
    流程:
      1. 【占位】向目标靠近（运动控制代码待填写）
      2. 【占位】摄像头识别到与目标距离到达某值后
      3. 调用 bt.turn_right() 向从车发送数字 0
      4. 阻塞等待从车回复 "ok"
      5. 返回 True
    
    返回: True=完整流程完成, False=超时/中断
    """
    print("\n  [APPROACH] === 靠近目标 & 蓝牙信号 ===")

    # ── 锁定航向（靠近过程保持方向） ──
    target_heading = _lock_yaw()
    print("  [APPROACH] 航向锁定: {:.1f}°".format(target_heading))

    start_ms = time.ticks_ms()
    loop_cnt = 0

    # ═══════════════════════════════════════════════════════════
    #  阶段 1: 向目标靠近
    #  【待实现】填入实际靠近控制逻辑，例如：
    #   - from cam_follow import compute_control, reset_control
    #   - reset_control()
    #   - while True:
    #   -     if _abort_check(): return False
    #   -     elapsed = time.ticks_diff(...) / 1000
    #   -     if elapsed > STARTUP_APPROACH_TIMEOUT: ...
    #   -     cam_data = cam.receive()
    #   -     if cam_data is None: continue
    #   -     vx, vy, wz = compute_control(cam_data, target_heading)
    #   -     omni_drive_closed_loop(vx, vy, wz, rs, FWD_CTRL_DT)
    #   -     # 当距离满足条件时 break 进入阶段 2
    #   -     if cam_data['distance_cm'] < TARGET_DIST_CM:
    #   -         break
    # ═══════════════════════════════════════════════════════════
    print("  [APPROACH] 【占位】靠近目标运动代码待实现")

    # 占位：直接进入阶段 2（无实际靠近动作）
    # 后续将上方占位中的循环结果走到此处

    # ── 停轮 ──
    stop_all()
    print("  [APPROACH] 到达目标位置")

    # ═══════════════════════════════════════════════════════════
    #  阶段 2: 检查距离条件 → 调用 turn_right 发送数字 0
    #  【待实现】替换下方条件为实际摄像头距离判断，例如：
    #   - if cam_data is not None and cam_data['distance_cm'] < TARGET_DIST_CM:
    # ═══════════════════════════════════════════════════════════
    if True:  # ← 替换为实际距离判定条件（空置时始终执行以便调试）
        print("  [APPROACH] 距离条件满足，调用 turn_right 发送数字 0...")

        try:
            bt.turn_right()
            print("  [APPROACH] 数字 0 已发送，等待从车 ok...")

            # 阶段 3: 等待从车回复 "ok"
            ok_received = bt.wait_ok(timeout_ms=WAIT_OK_TIMEOUT_MS)
            if ok_received:
                print("  [APPROACH] 从车已确认 (ok)，流程完成")
                return True
            else:
                print("  [APPROACH] 等待从车 ok 超时 ({:.0f}s)".format(
                    WAIT_OK_TIMEOUT_MS / 1000))
                return False
        except Exception as e:
            print("  [APPROACH] 蓝牙通信失败:", e)
            return False
    else:
        # 距离条件不满足，由阶段 1 循环继续靠近
        print("  [APPROACH] 距离条件暂不满足，继续靠近（需循环逻辑）")
        return False


# ═══════════════════════════════════════════════════════════════
#  启动步骤: 收到 ok 后全速后退至黄线停车
# ═══════════════════════════════════════════════════════════════

def _action_forward_until_yellow():
    """
    收到从车 ok 后全速后退，直至摄像头识别到黄线后停车。
    
    航向闭环保持，摄像头黄线识别逻辑待实现（当前占位）。
    
    返回: True=检测到黄线已停车, False=超时/中断
    """
    print("\n  [YELLOW] === 全速后退，等待黄线 ===")

    # 锁定航向
    target_heading = _lock_yaw()
    print("  [YELLOW] 航向锁定: {:.1f}°  全速 {:.1f}m/s".format(
        target_heading, STARTUP_FULL_SPEED))

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0

    led.value(1)

    while True:
        # ── 超时 / 退出 ──
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > YELLOW_LINE_TIMEOUT_S:
            print("  [YELLOW] 超时 ({:.1f}s)，未检测到黄线".format(elapsed))
            led.value(0)
            return False

        # ── 读取编码器（闭环驱动需要） ──
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        # ═══════════════════════════════════════════════════════
        #  【待实现】摄像头黄线识别
        #  替换下方条件为实际的黄线检测逻辑，例如：
        #   - from cam_data import CamDataReceiver
        #   - cam = CamDataReceiver()
        #   - cam_data = cam.receive()
        #   - if cam_data is not None and cam_data.get('yellow_line', False):
        # ═══════════════════════════════════════════════════════
        if False:  # ← 替换为实际黄线检测条件
            print("  [YELLOW] 检测到黄线，停车！")
            led.value(0)
            return True

        # ── 进度打印（每 500ms） ──
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [YELLOW] 全速后退中... yaw={:.1f}°  t={:.1f}s".format(
                yaw, elapsed))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        # ── 航向纠偏 ──
        wz = _heading_correction(target_heading)

        # ── 闭环驱动: 全速后退 ──
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
    """
    启动流程中：UWB XY 控制导航到 supplies 固定坐标 (-50, 210)。
    
    IMU 航向在全程保持不变（锁定初始 yaw）。
    到达 supplies（死区内连续 N 帧）后返回 True。
    无 Phase 2 搜索，无摄像头检测。
    
    返回: True=已到达 supplies, False=超时/中断
    """

    uwb = _ensure_uwb()
    if uwb is None:
        print("  [GOTO] UWB 不可用")
        return False

    if supplies is None:
        print("  [GOTO] supplies 坐标未记录")
        return False

    target_x, target_y = supplies
    print("\n  [GOTO] === 导航到 supplies ({:.1f}, {:.1f}) ===".format(target_x, target_y))

    # ── 锁定航向（全程不变） ──
    target_heading = _lock_yaw()
    print("  [GOTO] 航向锁定: {:.1f}°".format(target_heading))

    # 清空编码器缓冲区
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    last_uwb_ms = start_ms
    loop_cnt = 0
    near_target_count = 0

    led.value(1)

    while True:
        # ── 超时 / 退出 ──
        if _abort_check():
            led.value(0)
            return False

        elapsed_total = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed_total > SUPPLIES_TIMEOUT_S:
            print("  [GOTO] 超时 ({:.1f}s)".format(elapsed_total))
            led.value(0)
            return False

        # ── 接收 UWB 数据（每 50ms） ──
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_uwb_ms) >= 50:
            uwb.step()
            last_uwb_ms = now_ms

        # ── 当前 UWB 坐标 ──
        curr_x, curr_y = uwb.get_position()

        error_x = target_x - curr_x
        error_y = target_y - curr_y
        dist = math.sqrt(error_x * error_x + error_y * error_y)

        # ── 到达判定 ──
        if dist < SUPPLIES_DB:
            near_target_count += 1
            if near_target_count >= SUPPLIES_ARRIVAL_FRAMES:
                print("  [GOTO] 已到达 supplies ({:.1f}, {:.1f})  dist={:.1f}cm".format(
                    target_x, target_y, dist))
                led.value(0)
                return True
        else:
            near_target_count = 0

        # ── 进度打印 ──
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [GOTO] pos=({:.1f},{:.1f}) target=({:.1f},{:.1f}) dist={:.1f}cm".format(
                curr_x, curr_y, target_x, target_y, dist))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        # ── 航向保持 ──
        _read_imu_update_yaw()
        wz = _heading_correction(target_heading)

        # ── 坐标系变换 + P 控制 ──
        yaw_rad = math.radians(yaw)
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

        # ── 驱动 ──
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
    """
    UWB XY 控制导航回到 origin 起点坐标。IMU 航向锁定不变。
    到达 origin（死区内连续 N 帧）后返回 True。
    返回: True=已到达 origin, False=超时/中断
    """
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

        curr_x, curr_y = uwb.get_position()

        error_x = target_x - curr_x
        error_y = target_y - curr_y
        dist = math.sqrt(error_x * error_x + error_y * error_y)

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
            print("  [RTN] pos=({:.1f},{:.1f}) target=({:.1f},{:.1f}) dist={:.1f}cm".format(
                curr_x, curr_y, target_x, target_y, dist))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        _read_imu_update_yaw()
        wz = _heading_correction(target_heading)

        yaw_rad = math.radians(yaw)
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
    IMU 航向全程锁定不变（lock 在进入时的 yaw）。

    搜索模式（从 supplies 点出发）：
      → 右 100cm → 前 10cm → 左 100cm → 前 10cm → 右 100cm → ...
      共 10 行（100cm / 10cm = 10 行），覆盖 100cm × 100cm

    全程摄像头检测，识别到物品立即返回 True。
    全部搜索完未检测到返回 False。

    返回: True=已检测到物品, False=全部搜索完未发现/中断
    """
    # 锁定航向
    target_heading = _lock_yaw()
    print("\n  [SEARCH] === 之字形搜索 100cm×100cm 物质区 ===")
    print("  [SEARCH] 航向锁定: {:.1f}°".format(target_heading))

    AREA_CM   = SEARCH_AREA_SIZE_CM     # 100.0
    STEP_CM   = SEARCH_ROW_STEP_CM      # 10.0
    MAX_ROWS  = int(AREA_CM / STEP_CM)  # 10 行

    led.value(1)

    for row in range(MAX_ROWS):
        direction   = "RIGHT" if (row % 2 == 0) else "LEFT"
        vy_sign     = 1.0 if direction == "RIGHT" else -1.0
        row_dist_m  = AREA_CM / 100.0   # 1.0m
        row_label   = "{}/{}".format(row + 1, MAX_ROWS)

        print("\n  [SEARCH] --- 第 {} 行 {} {:.0f}cm ---".format(
            row_label, direction, AREA_CM))

        # ─────────────────────────────────────────────────────
        #  本行横向搜索
        # ─────────────────────────────────────────────────────
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

            # ═══════════════════════════════════════════════════
            #  【待实现】摄像头物品检测
            #  替换下方条件为实际摄像头识别逻辑
            # ═══════════════════════════════════════════════════
            if False:  # ← 替换为实际摄像头检测条件
                stop_all()
                print("  [SEARCH] 摄像头识别到物品！")
                led.value(0)
                return True

            # ── 行到达判定 ──
            if avg_dist >= row_dist_m:
                print("  [SEARCH] 第 {} 行完成 dist={:.2f}m".format(row_label, avg_dist))
                break

            # ── 进度打印 ──
            now = time.ticks_ms()
            if time.ticks_diff(now, row_last_print_ms) >= 500:
                row_last_print_ms = now
                print("  [SEARCH] 行{} {} dist={:.2f}m / {:.0f}cm".format(
                    row_label, direction, avg_dist, AREA_CM))

            # ── 航向保持 ──
            _read_imu_update_yaw()
            wz = _heading_correction(target_heading)

            # ── 驱动: 纯横向移动 ──
            try:
                rs = [counts[i] / ENC_SCALE[i] / SEARCH_CTRL_DT
                      if ENC_SCALE[i] != 0 else 0 for i in range(4)]
                omni_drive_closed_loop(0, vy_sign * SEARCH_SPEED, wz, rs, SEARCH_CTRL_DT)
            except Exception as e:
                print("  [SEARCH] 驱动错误:", e)

            time.sleep_ms(int(SEARCH_CTRL_DT * 1000))

        # ─────────────────────────────────────────────────────
        #  行间步进（最后一行无需步进）
        # ─────────────────────────────────────────────────────
        if row >= MAX_ROWS - 1:
            break  # 最后一行完成，退出

        print("  [SEARCH] 行间步进 {:.0f}cm...".format(STEP_CM))
        stop_all()
        time.sleep_ms(200)

        step_dist_m = STEP_CM / 100.0  # 0.1m
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
            if False:  # ← 替换为实际摄像头检测条件
                stop_all()
                print("  [SEARCH] 步进中识别到物品！")
                led.value(0)
                return True

            # ── 步进到达判定 ──
            if avg_dist >= step_dist_m:
                print("  [SEARCH] 步进完成 dist={:.2f}m".format(avg_dist))
                break

            # ── 进度 ──
            now = time.ticks_ms()
            if time.ticks_diff(now, step_last_print_ms) >= 500:
                step_last_print_ms = now
                print("  [SEARCH] 步进 dist={:.2f}m / {:.0f}cm".format(avg_dist, STEP_CM))

            # ── 航向保持 ──
            _read_imu_update_yaw()
            wz = _heading_correction(target_heading)

            # ── 驱动: 纯前向移动 ──
            try:
                rs = [counts[i] / ENC_SCALE[i] / SEARCH_CTRL_DT
                      if ENC_SCALE[i] != 0 else 0 for i in range(4)]
                omni_drive_closed_loop(SEARCH_SPEED, 0, wz, rs, SEARCH_CTRL_DT)
            except Exception as e:
                print("  [SEARCH] 驱动错误:", e)

            time.sleep_ms(int(SEARCH_CTRL_DT * 1000))

        # 步进完成后短暂停轮
        stop_all()
        time.sleep_ms(200)

    # ── 全部搜索完毕 ──
    print("  [SEARCH] 全部 {:.0f}cm×{:.0f}cm 区域搜索完成，未检测到物品".format(
        AREA_CM, AREA_CM))
    led.value(0)
    return False


# ═══════════════════════════════════════════════════════════════
#  C14 步骤 2: UWB 平移（航向保持）
# ═══════════════════════════════════════════════════════════════

def _action_uwb_translate():
    """向 UWB 锚点平移（仅横向），IMU 航向闭环保持。返回: True=正常完成, False=中断"""
    print("  [UWB] Starting translate toward anchor...")

    # 锁定当前航向
    target_heading = _lock_yaw()
    print("  [UWB] Heading locked: {:.1f}°".format(target_heading))

    # ── 获取 UWB 共享实例 ──
    uwb = _ensure_uwb()
    if uwb is None:
        print("  [UWB] UWB not initialized")
        return False
    print("  [UWB] UWB ready, frame={}".format(uwb.get_frame_count()))

    # ── 平移主循环 ──
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0

    led.value(1)

    while True:
        # ── 超时 / 退出 ──
        if _abort_check():
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > UWB_TIMEOUT_S:
            print("  [UWB] Translate timeout ({:.1f}s)".format(elapsed))
            led.value(0)
            return False

        # ── 读取 UWB 数据 ──
        uwb.step()
        x_cm, y_cm = uwb.get_position()

        # ── 到达判定：X 方向已居中 ──
        if abs(x_cm) < UWB_X_DEADBAND:
            print("  [UWB] Centered! X={:.1f}cm (deadband={:.1f}cm)".format(
                 x_cm, UWB_X_DEADBAND))
            led.value(0)
            return True

        # ── 打印（每 500ms） ──
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            dist_cm, angle_deg = uwb.get_distance_angle()
            print("  [UWB] X={:+.1f}cm Y={:.1f}cm D={:.1f}cm A={:.1f}°  yaw={:.1f}°".format(
                x_cm, y_cm, dist_cm, angle_deg, yaw))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        # ── 计算纵向速度：P 控制 X 误差 → vx ──
        # X>0 → 锚点在前方 → vx>0 前进（前方=UWB -X，Y减小→向右）
        fwd_speed = x_cm * UWB_LAT_P_GAIN
        fwd_speed = max(-UWB_LAT_SPEED, min(fwd_speed, UWB_LAT_SPEED))

        # ── 航向纠偏 ──
        wz = _heading_correction(target_heading)

        # ── 闭环驱动: vx=纵向速度, vy=0, wz=航向纠偏 ──
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
    """C14 (KEY3): 前进 20cm → UWB 平移，全程航向保持"""
    print("\n" + "=" * 50)
    print("[C14] 前进 20cm → UWB 平移（航向保持）")
    print("=" * 50)

    # ── 编码器准备：停 ticker，手动接管 ──
    pause_encoder_ticker()
    _encoder_reset()

    result = True

    # ── 步骤 1: 前进 20cm ──
    if not _action_forward_20cm():
        result = False
    else:
        # ── 段间停顿 ──
        stop_all()
        time.sleep_ms(300)

        # ── 步骤 2: UWB 平移 ──
        _encoder_reset()
        if not _action_uwb_translate():
            result = False

    # ── 清理 ──
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
    """C8 (KEY1): 通过蓝牙向从车发送状态消息"""
    print("\n" + "=" * 50)
    print("[C8] 蓝牙发送从车消息")
    print("=" * 50)

    try:
        # 发送同步运动指令作为小车状态消息
        bt.send_sync_move(0, 0, 0)
        print("[C8] 消息已发送")
    except Exception as e:
        print("[C8] 发送失败:", e)

    print("=" * 50 + "\n")


# ═══════════════════════════════════════════════════════════════
#  C9: 摄像头靠近（代码空置）
# ═══════════════════════════════════════════════════════════════

def action_c9():
    """C9 (KEY2): 摄像头靠近 — 代码空置，待实现"""
    print("\n" + "=" * 50)
    print("[C9] 摄像头靠近 — 待实现")
    print("=" * 50)

    # TODO: 摄像头靠近逻辑待补充
    # 预留接口:
    #   from cam_data import CamDataReceiver, x_to_cm, y_to_distance
    #   from cam_follow import compute_control, reset_control
    #   ...

    print("[C9] 当前为空操作")
    print("=" * 50 + "\n")


# ═══════════════════════════════════════════════════════════════
#  send_messages — 条件触发蓝牙从车通信
# ═══════════════════════════════════════════════════════════════

# 发送间隔控制（避免消息风暴）
_SEND_INTERVAL_MS = 200       # 最小发送间隔 (ms)
_send_last_ms = 0

# 发送开关（可由其他模块/动作置位）
_send_enabled = True


def send_messages():
    """
    条件判断是否向从车发送蓝牙消息。
    主循环每迭代调用一次，内部通过 if 条件决定是否实际发送。

    【待实现】根据实际需求填写触发条件，可选方案：
      1. 距离触发:  UWB 距离 < 阈值 → 通知从车
      2. 状态触发:  C14 动作完成标志位 → 发送完成通知
      3. 周期性:    每 N 秒发送一次心跳/状态
      4. 组合条件:  多个条件 AND/OR 组合

    【待实现】根据协议填写具体消息内容，MasterBT 可用接口：
      bt.send_sync_move(vx, vy, wz)       — 同步运动指令（火抛）
      bt.send_pos_adjust(vx, vy, wz)      — 位置调整（等待 ACK）
      bt.send_pos_adjust_async(vx, vy, wz) — 位置调整（不等待 ACK）
      bt.send_emergency_stop()            — 紧急停止
      bt.send_imu_data(r, p, y, wx, wy, wz) — IMU 姿态遥测 (JSON)
    """
    global _send_last_ms

    # 发送开关关闭则直接返回
    if not _send_enabled:
        return

    # 发送间隔限制（防消息风暴）
    now = time.ticks_ms()
    if time.ticks_diff(now, _send_last_ms) < _SEND_INTERVAL_MS:
        return

    # ═══════════════════════════════════════════════════════════
    #  【待填写】触发条件判断
    #  根据实际需求取消注释并修改以下条件之一：
    # ═══════════════════════════════════════════════════════════

    should_send = False  # 默认不发送，替换为实际条件

    # --- 示例 1: 距离阈值触发（需在 C14/UWB 上下文中使用） ---
    # if uwb is not None:
    #     dist_cm, _ = uwb.get_distance_angle()
    #     if dist_cm < 50.0:  # 距离 < 50cm 时发送
    #         should_send = True

    # --- 示例 2: 状态标志触发（由 C14 动作完成后置位） ---
    # global _c14_done_flag
    # if _c14_done_flag:
    #     should_send = True
    #     _c14_done_flag = False  # 单次触发后清零

    # --- 示例 3: 周期性发送（每 1 秒一次） ---
    # if loop_cnt % 100 == 0:  # 100 × 10ms = 1s
    #     should_send = True

    # --- 示例 4: 自定义条件 ---
    # if <你的条件>:
    #     should_send = True

    # ═══════════════════════════════════════════════════════════
    #  满足条件 → 发送消息
    # ═══════════════════════════════════════════════════════════

    if should_send:
        try:
            # 【待填写】选择并取消注释以下消息类型之一：

            # --- 同步运动指令 ---
            # bt.send_sync_move(vx=0.0, vy=0.0, wz=0.0)

            # --- 位置调整指令（等待从机 ACK） ---
            # bt.send_pos_adjust(vx=0.0, vy=0.0, wz=0.0)

            # --- 位置调整指令（不等待 ACK） ---
            # bt.send_pos_adjust_async(vx=0.0, vy=0.0, wz=0.0)

            # --- 紧急停止 ---
            # bt.send_emergency_stop()

            # --- IMU 遥测数据（需先读取 IMU 获取 roll/pitch/yaw/wx/wy/wz） ---
            # bt.send_imu_data(roll=0.0, pitch=0.0, yaw=0.0,
            #                  wx=0.0, wy=0.0, wz=0.0)

            # --- 自定义消息（直接操作 bt._uart.write） ---
            # bt._uart.write(b"YOUR_CUSTOM_CMD\r\n")

            _send_last_ms = now

        except Exception as e:
            print("[SEND] 发送失败:", e)


# ═══════════════════════════════════════════════════════════════
#  主循环
# ═══════════════════════════════════════════════════════════════

def main():
    print("")
    print("=" * 50)
    print("  RT1021 — 按键驱动控制")
    print("  启动: 原点 → 前进10cm → 导航supplies → 之字形搜索")
    print("=" * 50)
    print("  C14 (KEY3): 前进 20cm → UWB 平移")
    print("  C8  (KEY1): 蓝牙发送从车消息")
    print("  C9  (KEY2): 摄像头靠近 (空置)")
    print("  SW2 (D9)  : 强制退出")
    print("=" * 50)
    print("")
    loop_cnt = 0

    # ── UWB 初始化 & 记录起点坐标（小车运动前） ──
    global origin
    uwb = _ensure_uwb()
    if uwb is not None and uwb.get_frame_count() > 0:
        uwb.uwb_record()
        origin = uwb.get_position()
        print("  [ORIGIN] 起点坐标已记录: ({:.1f}, {:.1f})".format(
            origin[0], origin[1]))
    else:
        print("  [ORIGIN] UWB 未就绪，使用默认坐标")
        origin = (120, 320)

    # ── 记录原点后，前进 10cm（航向保持，5° 偏差自动回正） ──
    pause_encoder_ticker()
    _encoder_reset()
    if _action_startup_forward():
        # ── 段间停顿 + 编码器复位 ──
        stop_all()
        time.sleep_ms(300)
        _encoder_reset()

        # ── 导航到 supplies 固定坐标（航向锁，全程 IMU 不变） ──
        if _action_goto_supplies_startup():
            # ── 段间停顿 ──
            stop_all()
            time.sleep_ms(300)
            _encoder_reset()

            # ── 到达 supplies 后之字形搜索 100cm×100cm 物质区 ──
            result = _action_search_supplies_area()
            if result:
                print("  [STARTUP] 摄像头检测到目标，平移完毕")

                # ═══════════════════════════════════════════════════════
                #  循环：靠近 → 后退 → 导航 → 搜索 → 再循环
                #  退出条件：物质区搜索完成或中断
                # ═══════════════════════════════════════════════════════
                cycle = 0
                while True:
                    cycle += 1
                    print("\n  [CYCLE] === 第 {} 轮 ===".format(cycle))

                    # ① 靠近目标 → 蓝牙信号通知从车
                    if not see_and_push():
                        print("  [CYCLE] 靠近/信号中断，退出循环")
                        break

                    # ② 全速后退至黄线
                    if not _action_forward_until_yellow():
                        print("  [CYCLE] 黄线检测中断/超时，退出循环")
                        break

                    # ── 后退完成后通知从车左转 ──
                    try:
                        bt.turn_left()
                        print("  [CYCLE] turn_left 已发送，等待从车 ok...")
                        if not bt.wait_ok(timeout_ms=WAIT_OK_TIMEOUT_MS):
                            print("  [CYCLE] 等待从车 ok 超时 ({:.0f}s)，退出循环".format(
                                WAIT_OK_TIMEOUT_MS / 1000))
                            break
                        print("  [CYCLE] 从车已确认 (ok)")
                    except Exception as e:
                        print("  [CYCLE] turn_left 通信失败:", e)
                        break

                    # ③ 导航到 supplies 固定坐标
                    if not _action_goto_supplies_startup():
                        print("  [CYCLE] 导航到 supplies 失败，退出循环")
                        break

                    # ④ 之字形搜索物质区 → 摄像头再检测
                    if not _action_search_supplies_area():
                        print("  [CYCLE] 物质区搜索完成/中断，退出循环")
                        break

                    # ⑤ 摄像头检测到物品 → 回到 ① 继续下一轮
                    print("  [CYCLE] 检测到物品，继续下一轮...")
            else:
                print("  [STARTUP] 物质区搜索完成/中断，进入主循环")
        else:
            print("  [STARTUP] 导航到 supplies 失败，进入主循环")
    else:
        print("  [STARTUP] 前进中断，进入主循环")

    # ── 循环退出后，返回原点 ──
    print("\n  [RTN] 循环结束，返回原点...")
    stop_all()
    time.sleep_ms(300)
    _encoder_reset()
    _action_return_to_origin()

    stop_all()
    _encoder_reset()
    resume_encoder_ticker()
    led.value(1)  # 就绪指示
    print("  等待按键...")
    print("")

    try:
        while True:
            # ── 看门狗 + 按键扫描 ──
            capture()
            pet_watchdog()

            # ── GC ──
            loop_cnt += 1
            if loop_cnt % 50 == 0:
                gc.collect()

            # ── 按键分发 ──
            if key_triggered(1):          # KEY1 (C8) → 蓝牙消息
                action_c8()
                led.value(1)
                continue

            if key_triggered(2):          # KEY2 (C9) → 摄像头靠近
                action_c9()
                led.value(1)
                continue

            if key_triggered(3):          # KEY3 (C14) → 前进 + UWB
                action_c14()
                led.value(1)
                continue

            # ── 条件触发蓝牙从车通信 ──
            send_messages()

            # ── SW2 强制退出 ──
            if check_sw2():
                print("\n[SW2] 退出")
                break

            time.sleep_ms(10)

    except Exception as e:
        print("\n[FATAL] 主循环异常:")
        try:
            import sys
            sys.print_exception(e)
        except Exception:
            print("  ", e)

    finally:
        stop_all()
        # ── 释放 UWB 共享实例 ──
        global _uwb_shared
        if _uwb_shared is not None:
            _uwb_shared.stop()
            _uwb_shared = None
        try:
            resume_encoder_ticker()
        except Exception:
            pass
        time.sleep_ms(50)
        print("\n系统已停止。")
        print("(可重新运行或进入 REPL)")

# ═══════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════

main()
