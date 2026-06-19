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
【复用】compute_control() 可直接被 main.py 调用
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

# ═══════════════════════════════════════════════════════════════
#  常量
# ═══════════════════════════════════════════════════════════════

STATE_FOLLOW  = 0
STATE_STOPPED = 1
STATE_LOST    = 2

TARGET_DIST_CM  = 30.0   # 目标跟随距离 (cm)
STOP_DIST_CM    = 5.0    # 到达判定容差 (cm)
LOST_TIMEOUT_MS = 500    # 丢失超时 (ms)

MAX_SPEED_FWD  = 0.50    # 最大前进速度 (m/s)
MAX_SPEED_LAT  = 0.45    # 最大横向速度 (m/s)
ACCEL_LIMIT    = 3.0     # 加速度限制 (m/s²)
MIN_SPEED      = 0.10    # 最低速度，防电机死区 (m/s)
DT             = 0.02    # 控制周期 (s)

# ═══════════════════════════════════════════════════════════════
#  CascadePID 类
# ═══════════════════════════════════════════════════════════════

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
        # compute(0, error) 配合 y_to_distance() 坐标映射形成正确方向
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


# ═══════════════════════════════════════════════════════════════
#  PID 实例（backup 已验证参数）
# ═══════════════════════════════════════════════════════════════

PID_FWD = CascadePID(
    kp=0.012, ki=0.003, kd=0.004,
    out_limit=MAX_SPEED_FWD,
    accel_limit=ACCEL_LIMIT
)

PID_LAT = CascadePID(
    kp=0.010, ki=0.002, kd=0.003,
    out_limit=MAX_SPEED_LAT,
    accel_limit=ACCEL_LIMIT
)

# ═══════════════════════════════════════════════════════════════
#  速度低通滤波
# ═══════════════════════════════════════════════════════════════

SPEED_FILTER_ALPHA = 0.8
filt_fwd = 0.0
filt_lat = 0.0
filt_fwd_sign = 0
filt_lat_sign = 0


def speed_filter(fwd, lat):
    """一阶低通滤波，方向变化时重置滤波器"""
    global filt_fwd, filt_lat, filt_fwd_sign, filt_lat_sign

    new_sign = 1 if fwd > 0 else (-1 if fwd < 0 else 0)
    if new_sign != 0 and new_sign != filt_fwd_sign:
        filt_fwd = fwd
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


def _reset_filter():
    """重置滤波全局状态（内部使用）"""
    global filt_fwd, filt_lat, filt_fwd_sign, filt_lat_sign
    filt_fwd = 0.0
    filt_lat = 0.0
    filt_fwd_sign = 0
    filt_lat_sign = 0


# ═══════════════════════════════════════════════════════════════
#  状态机变量（模块级，与 backup 完全一致）
# ═══════════════════════════════════════════════════════════════

_state = STATE_LOST
last_target_ms = time.ticks_ms()


# ═══════════════════════════════════════════════════════════════
#  API — 供 main.py 调用
# ═══════════════════════════════════════════════════════════════

def reset_control(reset_state=False):
    """重置控制状态。
    reset_state=True: 同时复位状态机到 LOST（模式切换时使用）
    """
    global _state, last_target_ms
    if reset_state:
        _state = STATE_LOST
        last_target_ms = time.ticks_ms()
    PID_FWD.reset()
    PID_LAT.reset()
    _reset_filter()


def compute_control(x_cm, actual_dist, is_target, now_ms=None):
    """
    一帧视觉追踪控制（与 backup 主循环逻辑完全一致）。
    
    返回: dict {
        'cmd_fwd':    float|None,   # 前进速度指令 (m/s)
        'cmd_lat':    float|None,   # 横向速度指令 (m/s)
        'cmd_fwd_raw': float,        # PID 原始输出（滤波前）
        'cmd_lat_raw': float,
        'state':      int,           # 当前状态
        'arrived':    bool,          # 是否到达目标
        'state_msg':  str|None,      # 状态变化消息
    }
    """
    global _state, last_target_ms, filt_fwd, filt_lat, filt_fwd_sign, filt_lat_sign

    if now_ms is None:
        now_ms = time.ticks_ms()

    result = {
        'cmd_fwd':     None,
        'cmd_lat':     None,
        'cmd_fwd_raw': 0.0,
        'cmd_lat_raw': 0.0,
        'state':       _state,
        'arrived':     False,
        'state_msg':   None,
    }

    # ── 状态判断（与 backup 第176-195行完全一致）──
    if is_target:
        last_target_ms = now_ms
        if _state == STATE_LOST:
            _state = STATE_FOLLOW
            reset_wheel_pi()
            PID_FWD.reset()
            PID_LAT.reset()
            filt_fwd = 0.0
            filt_lat = 0.0
            filt_fwd_sign = 0
            filt_lat_sign = 0
            result['state_msg'] = "[LOST -> FOLLOW] Captured!"
    else:
        if _state == STATE_FOLLOW:
            if time.ticks_diff(now_ms, last_target_ms) > LOST_TIMEOUT_MS:
                _state = STATE_LOST
                stop_all()
                result['state_msg'] = "[FOLLOW -> LOST] Lost!"
                result['state'] = _state
                return result

    result['state'] = _state

    # ── FOLLOW 状态：级联 PID 控制（与 backup 第198-227行完全一致）──
    if _state == STATE_FOLLOW:
        x_error = x_cm
        y_error = actual_dist - TARGET_DIST_CM

        # 外环：位置 PID → 目标速度
        cmd_fwd = PID_FWD.compute(y_error, DT)
        cmd_lat = PID_LAT.compute(x_error, DT)

        result['cmd_fwd_raw'] = cmd_fwd
        result['cmd_lat_raw'] = cmd_lat

        # 速度滤波
        cmd_fwd, cmd_lat = speed_filter(cmd_fwd, cmd_lat)

        # 最低速度防死区
        if abs(cmd_fwd) > 0.01 and abs(cmd_fwd) < MIN_SPEED:
            cmd_fwd = MIN_SPEED if cmd_fwd > 0 else -MIN_SPEED
        if abs(cmd_lat) > 0.01 and abs(cmd_lat) < MIN_SPEED:
            cmd_lat = MIN_SPEED if cmd_lat > 0 else -MIN_SPEED

        # 到达判定
        if abs(y_error) < STOP_DIST_CM and abs(x_cm) < 10:
            _state = STATE_STOPPED
            stop_all()
            result['arrived'] = True
            result['state_msg'] = "[FOLLOW -> STOPPED] dist={:.1f}cm".format(actual_dist)
            result['state'] = _state
            return result

        result['cmd_fwd'] = cmd_fwd
        result['cmd_lat'] = cmd_lat

    # ── STOPPED 状态（与 backup 第229-240行完全一致）──
    elif _state == STATE_STOPPED:
        if abs(actual_dist - TARGET_DIST_CM) > STOP_DIST_CM * 2:
            _state = STATE_FOLLOW
            reset_wheel_pi()
            PID_FWD.reset()
            PID_LAT.reset()
            filt_fwd = 0.0
            filt_lat = 0.0
            filt_fwd_sign = 0
            filt_lat_sign = 0
            result['state_msg'] = "[STOPPED -> FOLLOW] Resuming."
            result['state'] = _state

    # ── LOST 状态（与 backup 第242-243行完全一致）──
    elif _state == STATE_LOST:
        stop_all()

    return result


# ═══════════════════════════════════════════════════════════════
#  独立运行入口（仅当直接运行 cam_follow.py 时执行）
# ═══════════════════════════════════════════════════════════════

def _standalone():
    """独立运行模式 — 与 backup 完全一致"""

    # ── 硬件 ──
    SWITCH2_PIN = 'D9'
    switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
    state2 = switch2.value()

    # ── 初始化（与 backup 第136-151行完全一致）──
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

    # ── 主循环（与 backup 第157-258行完全一致）──
    loop_count = 0
    last_print_ms = time.ticks_ms()

    while True:
        if switch2.value() != state2:
            stop_all()
            enc_ticker.start(10)
            print("\n[EXIT] SW2 toggled.")
            break

        data = recv.read()
        if data is None:
            time.sleep_ms(1)
            continue

        now = time.ticks_ms()

        # ── 调用核心控制 ──
        x_cm = x_to_cm(data['x'])
        actual_dist = y_to_distance(data['y'])
        ctrl = compute_control(x_cm, actual_dist, data['is_target'], now)

        if ctrl['state_msg']:
            print(ctrl['state_msg'])

        if ctrl['cmd_fwd'] is not None:
            rc = get_encoder_counts()
            rs = [rc[i] / ENC_SCALE[i] / DT for i in range(4)]
            omni_drive_closed_loop(ctrl['cmd_fwd'], ctrl['cmd_lat'], 0, rs, DT)

        if time.ticks_diff(now, last_print_ms) >= 300:
            state_str = {0: "FOLLOW", 1: "STOP", 2: "LOST"}[_state]
            print("[#{:04d} {:s}] X:{:+5.1f}cm dist:{:5.1f}cm fwd:{:+.3f} lat:{:+.3f}".format(
                recv.frame_count, state_str, x_cm, actual_dist,
                PID_FWD.prev_output, PID_LAT.prev_output))
            last_print_ms = now

        loop_count += 1
        if loop_count % 50 == 0:
            gc.collect()
        time.sleep_ms(1)

    print("\n" + "=" * 50)
    print("Session: {} frames".format(recv.frame_count))
    print("=" * 50)


if __name__ == '__main__':
    _standalone()
