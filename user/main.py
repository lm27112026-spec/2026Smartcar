"""
main.py — 按键驱动控制

【按键功能】
  C14 (KEY3): 前进 20cm → UWB 平移（航向保持）
  C8  (KEY1): 通过蓝牙向从车发送消息
  C9  (KEY2): 摄像头靠近（代码空置）

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
from utils import normalize_angle

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

# ── SW2 ──
SW2_DEBOUNCE_MS  = 50

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


def _heading_correction(target_yaw):
    """
    计算航向纠偏 wz，使当前 yaw 趋近 target_yaw。
    返回: wz (归一化值，-1~1)
    """
    if not _read_imu_update_yaw():
        return 0.0

    error = target_yaw - yaw
    # 角度差归一化到 [-180, 180]
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    if abs(error) < HEADING_DEADBAND:
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
#  C14 步骤 2: UWB 平移（航向保持）
# ═══════════════════════════════════════════════════════════════

def _action_uwb_translate():
    """向 UWB 锚点平移（仅横向），IMU 航向闭环保持。返回: True=正常完成, False=中断"""
    print("  [UWB] Starting translate toward anchor...")

    # 锁定当前航向
    target_heading = _lock_yaw()
    print("  [UWB] Heading locked: {:.1f}°".format(target_heading))

    # ── 初始化 UWB ──
    from uwb_position import UWBPosition
    uwb = None
    try:
        uwb = UWBPosition(uart_id=0, baudrate=115200, target_anchor="8834")
    except Exception as e:
        print("  [UWB] Init failed:", e)
        return False

    # ── 等待首次有效 UWB 数据 ──
    print("  [UWB] Waiting for first data...")
    wait_start = time.ticks_ms()
    while uwb.get_frame_count() == 0:
        if _abort_check():
            uwb.stop()
            return False
        uwb.step()
        if time.ticks_diff(time.ticks_ms(), wait_start) > UWB_INIT_TIMEOUT_S * 1000:
            print("  [UWB] Init timeout — no data in {:.0f}s".format(UWB_INIT_TIMEOUT_S))
            uwb.stop()
            return False
        time.sleep_ms(10)

    print("  [UWB] Data acquired (frame {})".format(uwb.get_frame_count()))

    # ── 平移主循环 ──
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0

    led.value(1)

    while True:
        # ── 超时 / 退出 ──
        if _abort_check():
            led.value(0)
            uwb.stop()
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > UWB_TIMEOUT_S:
            print("  [UWB] Translate timeout ({:.1f}s)".format(elapsed))
            led.value(0)
            uwb.stop()
            return False

        # ── 读取 UWB 数据 ──
        uwb.step()
        x_cm, y_cm = uwb.get_position()

        # ── 到达判定：X 方向已居中 ──
        if abs(x_cm) < UWB_X_DEADBAND:
            print("  [UWB] Centered! X={:.1f}cm (deadband={:.1f}cm)".format(
                x_cm, UWB_X_DEADBAND))
            led.value(0)
            uwb.stop()
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

        # ── 计算横向速度：P 控制 X 误差 → vy ──
        # X>0 → 锚点在右侧 → vy>0 向右平移
        lat_speed = x_cm * UWB_LAT_P_GAIN
        lat_speed = max(-UWB_LAT_SPEED, min(lat_speed, UWB_LAT_SPEED))

        # ── 航向纠偏 ──
        wz = _heading_correction(target_heading)

        # ── 闭环驱动: vx=0（无前后）, vy=横向速度, wz=航向纠偏 ──
        try:
            rc = get_encoder_counts()
            if rc is None or len(rc) < 4:
                time.sleep_ms(5)
                continue
            rs = [rc[i] / ENC_SCALE[i] / UWB_CTRL_DT if ENC_SCALE[i] != 0 else 0
                  for i in range(4)]
            omni_drive_closed_loop(0, lat_speed, wz, rs, UWB_CTRL_DT)
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
    print("=" * 50)
    print("  C14 (KEY3): 前进 20cm → UWB 平移")
    print("  C8  (KEY1): 蓝牙发送从车消息")
    print("  C9  (KEY2): 摄像头靠近 (空置)")
    print("  SW2 (D9)  : 强制退出")
    print("=" * 50)
    print("  等待按键...")
    print("")

    led.value(1)  # 就绪指示
    loop_cnt = 0

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
