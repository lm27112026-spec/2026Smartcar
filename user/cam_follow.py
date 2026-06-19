"""
cam_follow.py — 摄像头目标跟随
【流程】
   1. FOLLOW：实时读取摄像头数据，PID 控制跟随
      - X 方向：保持目标居中（横向移动）
      - Y 方向：保持目标在设定距离（前后移动）
   2. STOPPED：到达目标距离 → 等目标移开重新跟随
   3. LOST：目标丢失 → 停车等待重新捕获
【安全】SW2 随时终止
【依赖】cam_data.py, motor.py, pid.py
"""

import gc, time, math
from machine import Pin
from cam_data import CamDataReceiver, x_to_cm, y_to_distance
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
                   enc_ticker, ENC_SCALE)
from pid import PID

# ── 状态机 ──
STATE_FOLLOW  = 0  # 跟随
STATE_STOPPED = 1  # 到达目标距离
STATE_LOST    = 2  # 目标丢失

# ── 硬件 ──
SWITCH2_PIN = 'D9'
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ── 跟随参数 ──
TARGET_DIST_CM  = 10.0   # 目标跟随距离 (cm)
STOP_DIST_CM    = 5.0    # 到达判定容差 (cm)
LOST_TIMEOUT_MS = 500    # 丢失超时 (ms)

# ── 速度限制 ──
MAX_VX = 0.4             # 最大横向速度 (m/s)
MAX_VY = 0.5             # 最大前后速度 (m/s)
MIN_SPEED = 0.15         # 最低速度（防死区）

# ── PID 参数 ──
# X 方向（横向居中）
PID_X = PID(kp=0.025, ki=0.010, kd=0.005,
            integral_limit=50, output_limit=1.0)

# Y 方向（距离保持）
PID_Y = PID(kp=0.040, ki=0.010, kd=0.0025,
            integral_limit=50, output_limit=1.0)

# ── 控制周期 ──
DT = 0.02  # 20ms

# ── 全局状态 ──
_state = STATE_LOST
last_target_ms = time.ticks_ms()

# ============================================================
#  初始化
# ============================================================
time.sleep_ms(100)
stop_all()
enc_ticker.stop()
for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)
reset_encoder_filter()
reset_wheel_pi()

recv = CamDataReceiver(uart_id=7)

print("=" * 50)
print("Camera Following")
print("Target distance: {:.0f}cm".format(TARGET_DIST_CM))
print("SW2 = exit")
print("=" * 50)


# ============================================================
#  速度限幅
# ============================================================
def clamp_speed(fwd, lat):
    """限幅并确保最低速度 (fwd=前进, lat=横向)"""
    # 前进限幅
    if abs(fwd) > MAX_VY:
        fwd = MAX_VY if fwd > 0 else -MAX_VY
    # 横向限幅
    if abs(lat) > MAX_VX:
        lat = MAX_VX if lat > 0 else -MAX_VX
    # 最低速度（仅在有运动意图时）
    if abs(fwd) > 0.01 and abs(fwd) < MIN_SPEED:
        fwd = MIN_SPEED if fwd > 0 else -MIN_SPEED
    if abs(lat) > 0.01 and abs(lat) < MIN_SPEED:
        lat = MIN_SPEED if lat > 0 else -MIN_SPEED
    return fwd, lat


# ============================================================
#  主循环
# ============================================================
loop_count = 0
last_print_ms = time.ticks_ms()

while True:
    # ── SW2 退出 ──
    if switch2.value() != state2:
        stop_all()
        enc_ticker.start(10)
        print("\n[EXIT] SW2 toggled.")
        break

    # ── 读取摄像头数据 ──
    data = recv.read()
    if data is None:
        time.sleep_ms(1)
        continue

    now = time.ticks_ms()

    # ── 状态判断 ──
    if data['is_target']:
        last_target_ms = now
        if _state == STATE_LOST:
            _state = STATE_FOLLOW
            reset_wheel_pi()
            PID_X.reset()
            PID_Y.reset()
            print("[LOST -> FOLLOW] Target captured!")
    else:
        # 目标丢失
        if _state == STATE_FOLLOW:
            if time.ticks_diff(now, last_target_ms) > LOST_TIMEOUT_MS:
                _state = STATE_LOST
                stop_all()
                print("[FOLLOW -> LOST] Target lost!")
                continue

    # ── 跟随控制 ──
    if _state == STATE_FOLLOW:
        # 计算误差（统一单位为 cm）
        x_cm = x_to_cm(data['x'])               # 横向偏移 (cm)
        actual_dist = y_to_distance(data['y'])   # 实际距离 (cm)
        x_error = x_cm                           # 横向误差：目标在左为负，右为正
        y_error = actual_dist - TARGET_DIST_CM   # 距离误差：正=太远，负=太近

        # PID 计算
        # 注意：motor.py 的 kinematics 是 vx=前进, vy=横向
        # PID.compute(setpoint, measurement) → error = setpoint - measurement
        # y_error > 0 表示目标太远，需要前进 (fwd > 0)
        # x_error > 0 表示目标在右，需要右移 (lat > 0)
        fwd_speed = PID_Y.compute(y_error, 0, DT)   # 前后：error = y_error
        lat_speed = PID_X.compute(x_error, 0, DT)   # 横向：error = x_error

        # 限幅
        fwd_speed, lat_speed = clamp_speed(fwd_speed, lat_speed)

        # 到达判定
        if abs(y_error) < STOP_DIST_CM and abs(x_cm) < 10:
            _state = STATE_STOPPED
            stop_all()
            print("[FOLLOW -> STOPPED] Arrived! dist={:.1f}cm".format(actual_dist))
        else:
            # 编码器闭环驱动 (vx=前进, vy=横向)
            rc = get_encoder_counts()
            rs = [rc[i] / ENC_SCALE[i] / DT for i in range(4)]
            omni_drive_closed_loop(fwd_speed, lat_speed, 0, rs, DT)

    elif _state == STATE_STOPPED:
        # 等待目标移开
        actual_dist = y_to_distance(data['y'])
        if abs(actual_dist - TARGET_DIST_CM) > STOP_DIST_CM * 2:
            _state = STATE_FOLLOW
            reset_wheel_pi()
            PID_X.reset()
            PID_Y.reset()
            print("[STOPPED -> FOLLOW] Target moved, resuming.")

    elif _state == STATE_LOST:
        stop_all()

    # ── 调试输出 ──
    if time.ticks_diff(now, last_print_ms) >= 300:
        x_cm = x_to_cm(data['x'])
        actual_dist = y_to_distance(data['y'])
        state_str = {0: "FOLLOW", 1: "STOP", 2: "LOST"}[_state]
        print("[#{:04d} {:s}] X:{:+6.1f}cm dist:{:5.1f}cm err:{:+5.1f}cm".format(
            recv.frame_count, state_str, x_cm, actual_dist,
            actual_dist - TARGET_DIST_CM))
        last_print_ms = now

    loop_count += 1
    if loop_count % 50 == 0:
        gc.collect()
    time.sleep_ms(1)

# ── 结束统计 ──
print("\n" + "=" * 50)
print("Session: {} frames".format(recv.frame_count))
print("=" * 50)
