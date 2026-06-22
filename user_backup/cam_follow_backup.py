"""
cam_follow.py — 摄像头目标跟随（级联 PID 控制）
【架构】
   外环: 位置 PID → 目标速度
   中环: 速度限幅/斜坡 → 平滑速度指令
   内环: motor.py 轮速 PI → PWM
【流程】
   1. FOLLOW：级联 PID 控制跟随
   2. STOPPED：到达目标距离
   3. LOST：目标丢失
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
STATE_FOLLOW  = 0
STATE_STOPPED = 1
STATE_LOST    = 2

# ── 硬件 ──
SWITCH2_PIN = 'D9'
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ── 跟随参数 ──
TARGET_DIST_CM  = 30.0   # 目标跟随距离 (cm)
STOP_DIST_CM    = 5.0    # 到达判定容差 (cm)
LOST_TIMEOUT_MS = 500    # 丢失超时 (ms)

# ── 速度限制 (m/s) ──
MAX_SPEED_FWD  = 0.50    # 最大前进速度
MAX_SPEED_LAT  = 0.45    # 最大横向速度
ACCEL_LIMIT    = 3.0     # 加速度限制 (m/s²) - 提高响应速度
MIN_SPEED      = 0.10    # 最低速度，防电机死区 (m/s)

# ── 控制周期 ──
DT = 0.02  # 20ms

# ============================================================
#  级联 PID 控制器
# ============================================================

class CascadePID:
    """级联 PID: 位置环 → 速度限幅 → 输出"""
    
    def __init__(self, kp, ki, kd, out_limit, accel_limit):
        self.pid = PID(kp=kp, ki=ki, kd=kd,
                       integral_limit=50, output_limit=out_limit)
        self.accel_limit = accel_limit
        self.prev_output = 0.0
    
    def compute(self, error, dt):
        """
        计算输出，带加速度限幅
        error: 位置误差 (cm)
        返回: 速度指令 (m/s)
        """
        # 位置 PID → 目标速度
        target_speed = self.pid.compute(0, error, dt)
        
        # 加速度限幅（平滑速度变化）
        delta = target_speed - self.prev_output
        max_delta = self.accel_limit * dt
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        output = self.prev_output + delta
        self.prev_output = output
        
        return output
    
    def reset(self):
        self.pid.reset()
        self.prev_output = 0.0


# ── 外环 PID（位置控制）──
# Y 方向（前进后退）
PID_FWD = CascadePID(
    kp=0.012, ki=0.003, kd=0.005,
    out_limit=MAX_SPEED_FWD,
    accel_limit=ACCEL_LIMIT
)

# X 方向（横向）
PID_LAT = CascadePID(
    kp=0.010, ki=0.002, kd=0.006,
    out_limit=MAX_SPEED_LAT,
    accel_limit=ACCEL_LIMIT
)

# ── 速度低通滤波 ──
SPEED_FILTER_ALPHA = 0.8   # 增大以减少延迟
filt_fwd = 0.0
filt_lat = 0.0
filt_fwd_sign = 0          # 记录上次符号，用于检测方向变化
filt_lat_sign = 0


def speed_filter(fwd, lat):
    """一阶低通滤波，方向变化时重置滤波器"""
    global filt_fwd, filt_lat, filt_fwd_sign, filt_lat_sign
    
    # 检测方向变化，变化时重置滤波器避免锁死
    new_sign = 1 if fwd > 0 else (-1 if fwd < 0 else 0)
    if new_sign != 0 and new_sign != filt_fwd_sign:
        filt_fwd = fwd  # 重置，不继承旧值
        filt_fwd_sign = new_sign
    else:
        filt_fwd = SPEED_FILTER_ALPHA * fwd + (1 - SPEED_FILTER_ALPHA) * filt_fwd
    
    new_sign_lat = 1 if lat > 0 else (-1 if lat < 0 else 0)
    if new_sign_lat != 0 and new_sign_lat != filt_lat_sign:
        filt_lat = lat
        filt_lat_sign = new_sign_lat
    else:
        filt_lat = SPEED_FILTER_ALPHA * lat + (1 - SPEED_FILTER_ALPHA) * filt_lat
    
    return filt_fwd, filt_lat


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
print("Camera Following (Cascade PID)")
print("Target: {:.0f}cm".format(TARGET_DIST_CM))
print("SW2 = exit")
print("=" * 50)


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
            PID_FWD.reset()
            PID_LAT.reset()
            filt_fwd = 0.0
            filt_lat = 0.0
            filt_fwd_sign = 0
            filt_lat_sign = 0
            print("[LOST -> FOLLOW] Captured!")
    else:
        if _state == STATE_FOLLOW:
            if time.ticks_diff(now, last_target_ms) > LOST_TIMEOUT_MS:
                _state = STATE_LOST
                stop_all()
                print("[FOLLOW -> LOST] Lost!")
                continue

    # ── 级联 PID 控制 ──
    if _state == STATE_FOLLOW:
        # 误差计算 (cm)
        x_cm = x_to_cm(data['x'])
        actual_dist = y_to_distance(data['y'])
        x_error = x_cm
        y_error = actual_dist - TARGET_DIST_CM
        
        # 外环：位置 PID → 目标速度
        cmd_fwd = PID_FWD.compute(y_error, DT)
        cmd_lat = PID_LAT.compute(x_error, DT)
        
        # 速度滤波
        cmd_fwd, cmd_lat = speed_filter(cmd_fwd, cmd_lat)
        
        # 最低速度防死区（仅有运动意图时生效）
        if abs(cmd_fwd) > 0.01 and abs(cmd_fwd) < MIN_SPEED:
            cmd_fwd = MIN_SPEED if cmd_fwd > 0 else -MIN_SPEED
        if abs(cmd_lat) > 0.01 and abs(cmd_lat) < MIN_SPEED:
            cmd_lat = MIN_SPEED if cmd_lat > 0 else -MIN_SPEED
        
        # 到达判定
        if abs(y_error) < STOP_DIST_CM and abs(x_cm) < 10:
            _state = STATE_STOPPED
            stop_all()
            print("[FOLLOW -> STOPPED] dist={:.1f}cm".format(actual_dist))
        else:
            # 内环：motor.py 闭环驱动
            rc = get_encoder_counts()
            rs = [rc[i] / ENC_SCALE[i] / DT for i in range(4)]
            omni_drive_closed_loop(cmd_fwd, cmd_lat, 0, rs, DT)

    elif _state == STATE_STOPPED:
        actual_dist = y_to_distance(data['y'])
        if abs(actual_dist - TARGET_DIST_CM) > STOP_DIST_CM * 2:
            _state = STATE_FOLLOW
            reset_wheel_pi()
            PID_FWD.reset()
            PID_LAT.reset()
            filt_fwd = 0.0
            filt_lat = 0.0
            filt_fwd_sign = 0
            filt_lat_sign = 0
            print("[STOPPED -> FOLLOW] Resuming.")

    elif _state == STATE_LOST:
        stop_all()

    # ── 调试输出 ──
    if time.ticks_diff(now, last_print_ms) >= 300:
        x_cm = x_to_cm(data['x'])
        actual_dist = y_to_distance(data['y'])
        state_str = {0: "FOLLOW", 1: "STOP", 2: "LOST"}[_state]
        print("[#{:04d} {:s}] X:{:+5.1f}cm dist:{:5.1f}cm fwd:{:+.3f} lat:{:+.3f}".format(
            recv.frame_count, state_str, x_cm, actual_dist,
            PID_FWD.prev_output, PID_LAT.prev_output))
        last_print_ms = now

    loop_count += 1
    if loop_count % 50 == 0:
        gc.collect()
    time.sleep_ms(1)

# ── 结束 ──
print("\n" + "=" * 50)
print("Session: {} frames".format(recv.frame_count))
print("=" * 50)
