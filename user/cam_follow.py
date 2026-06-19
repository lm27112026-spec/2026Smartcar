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
from cam_data import CamDataReceiver, y_to_distance
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
TARGET_DIST_CM  = 30.0   # 目标跟随距离 (cm)
STOP_DIST_CM    = 5.0    # 到达判定容差 (cm)
LOST_TIMEOUT_MS = 500    # 丢失超时 (ms)

# ── 速度限制 ──
MAX_VX = 0.4             # 最大横向速度 (m/s)
MAX_VY = 0.5             # 最大前后速度 (m/s)
MIN_SPEED = 0.15         # 最低速度（防死区）

# ── PID 参数 ──
# X 方向（横向居中）
PID_X = PID(kp=0.008, ki=0.001, kd=0.002,
            integral_limit=50, output_limit=1.0)

# Y 方向（距离保持）
PID_Y = PID(kp=0.012, ki=0.002, kd=0.003,
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
def clamp_speed(vx, vy):
    """限幅并确保最低速度"""
    # X 限幅
    if abs(vx) > MAX_VX:
        vx = MAX_VX if vx > 0 else -MAX_VX
    # Y 限幅
    if abs(vy) > MAX_VY:
        vy = MAX_VY if vy > 0 else -MAX_VY
    # 最低速度（仅在有运动意图时）
    if abs(vx) > 0.01 and abs(vx) < MIN_SPEED:
        vx = MIN_SPEED if vx > 0 else -MIN_SPEED
    if abs(vy) > 0.01 and abs(vy) < MIN_SPEED:
        vy = MIN_SPEED if vy > 0 else -MIN_SPEED
    return vx, vy


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
        # 计算误差
        x_error = data['x']              # 横向偏移：目标在左为负，右为正
        actual_dist = y_to_distance(data['y'])  # 实际距离
        y_error = actual_dist - TARGET_DIST_CM  # 距离误差：正=太远，负=太近

        # PID 计算
        vx = PID_X.compute(0, x_error, DT)   # 横向：目标居中时 vx=0
        vy = PID_Y.compute(0, y_error, DT)   # 前后：距离目标时 vy=0

        # 限幅
        vx, vy = clamp_speed(vx, vy)

        # 到达判定
        if abs(y_error) < STOP_DIST_CM and abs(x_error) < 10:
            _state = STATE_STOPPED
            stop_all()
            print("[FOLLOW -> STOPPED] Arrived! dist={:.1f}cm".format(actual_dist))
        else:
            # 编码器闭环驱动
            rc = get_encoder_counts()
            rs = [rc[i] / ENC_SCALE[i] / DT for i in range(4)]
            omni_drive_closed_loop(vx, vy, 0, rs, DT)

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
        actual_dist = y_to_distance(data['y'])
        state_str = {0: "FOLLOW", 1: "STOP", 2: "LOST"}[_state]
        print("[#{:04d} {:s}] X:{:+6.1f} dist:{:5.1f}cm err:{:+5.1f}".format(
            recv.frame_count, state_str, data['x'], actual_dist,
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
