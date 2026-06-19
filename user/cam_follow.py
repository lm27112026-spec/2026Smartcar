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
【复用】compute_control() 可直接被 main.py 调用，无需重复代码
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
#  常量（可被外部导入）
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

SPEED_FILTER_ALPHA = 0.8

# ═══════════════════════════════════════════════════════════════
#  CascadePID 类（可被外部导入）
# ═══════════════════════════════════════════════════════════════

class CascadePID:
    """级联 PID: 位置环 → 加速度限幅 → 输出"""

    def __init__(self, kp, ki, kd, out_limit, accel_limit):
        self.pid = PID(kp=kp, ki=ki, kd=kd,
                       integral_limit=50, output_limit=out_limit)
        self.accel_limit = accel_limit
        self.prev_output = 0.0

    def compute(self, error, dt):
        """
        计算输出，带加速度限幅
        error: 位置误差 (cm)，>0=太远需前进，<0=太近需后退
        返回: 速度指令 (m/s)
        """
        # 位置 PID → 目标速度
        # compute(setpoint, measurement): error = setpoint - measurement
        # 当 error(位置误差)>0(太远) → PID输出>0 → 前进
        target_speed = self.pid.compute(error, 0, dt)

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
#  PID 实例（模块级，供 compute_control() 使用）
# ═══════════════════════════════════════════════════════════════

_pid_fwd = CascadePID(
    kp=0.012, ki=0.003, kd=0.005,
    out_limit=MAX_SPEED_FWD,
    accel_limit=ACCEL_LIMIT
)

_pid_lat = CascadePID(
    kp=0.010, ki=0.002, kd=0.006,
    out_limit=MAX_SPEED_LAT,
    accel_limit=ACCEL_LIMIT
)

# ═══════════════════════════════════════════════════════════════
#  速度低通滤波（可被外部导入）
# ═══════════════════════════════════════════════════════════════

_filt_fwd = 0.0
_filt_lat = 0.0
_filt_fwd_sign = 0
_filt_lat_sign = 0


def speed_filter(fwd, lat):
    """一阶低通滤波，方向变化时重置滤波器"""
    global _filt_fwd, _filt_lat, _filt_fwd_sign, _filt_lat_sign

    new_sign = 1 if fwd > 0 else (-1 if fwd < 0 else 0)
    if new_sign != 0 and new_sign != _filt_fwd_sign:
        _filt_fwd = fwd
        _filt_fwd_sign = new_sign
    else:
        _filt_fwd = SPEED_FILTER_ALPHA * fwd + (1 - SPEED_FILTER_ALPHA) * _filt_fwd

    new_sign_lat = 1 if lat > 0 else (-1 if lat < 0 else 0)
    if new_sign_lat != 0 and new_sign_lat != _filt_lat_sign:
        _filt_lat = lat
        _filt_lat_sign = new_sign_lat
    else:
        _filt_lat = SPEED_FILTER_ALPHA * lat + (1 - SPEED_FILTER_ALPHA) * _filt_lat

    return _filt_fwd, _filt_lat


def reset_speed_filter():
    """重置速度滤波全局状态"""
    global _filt_fwd, _filt_lat, _filt_fwd_sign, _filt_lat_sign
    _filt_fwd = 0.0
    _filt_lat = 0.0
    _filt_fwd_sign = 0
    _filt_lat_sign = 0


# ═══════════════════════════════════════════════════════════════
#  控制状态（模块级）
# ═══════════════════════════════════════════════════════════════

_ctrl_state = STATE_LOST
_last_target_ms = 0


def reset_control(reset_state=False):
    """重置控制状态。
    reset_state=True: 同时重置状态机到 LOST（模式切换时使用）
    reset_state=False: 仅重置 PID 和滤波（跟踪中重捕获时使用）
    """
    global _ctrl_state, _last_target_ms
    if reset_state:
        _ctrl_state = STATE_LOST
        _last_target_ms = 0
    _pid_fwd.reset()
    _pid_lat.reset()
    reset_speed_filter()


# ═══════════════════════════════════════════════════════════════
#  核心 API — compute_control()
#  被 main.py 直接调用，一次调用 = 一帧视觉追踪控制
# ═══════════════════════════════════════════════════════════════

def compute_control(x_cm, actual_dist, is_target, now_ms=None):
    """
    一帧视觉追踪控制。
    
    参数:
        x_cm:        横向偏移 (cm)，x_to_cm() 的结果
        actual_dist: 实际距离 (cm)，y_to_distance() 的结果
        is_target:   是否检测到目标 (bool)
        now_ms:      当前时间戳 (ms)，None 则自动获取
    
    返回:
        dict: {
            'cmd_fwd':    float,   # 前进速度指令 (m/s)，None 表示停车
            'cmd_lat':    float,   # 横向速度指令 (m/s)
            'state':      int,     # 当前状态 (STATE_FOLLOW/STOPPED/LOST)
            'arrived':    bool,    # 是否到达目标（应切换到 STOPPED）
            'state_msg':  str|None,# 状态变化消息（用于打印）
        }
    """
    global _ctrl_state, _last_target_ms

    if now_ms is None:
        now_ms = time.ticks_ms()

    result = {
        'cmd_fwd':   None,
        'cmd_lat':   None,
        'state':     _ctrl_state,
        'arrived':   False,
        'state_msg': None,
    }

    # ── 状态判断 ──
    if is_target:
        _last_target_ms = now_ms
        if _ctrl_state == STATE_LOST:
            _ctrl_state = STATE_FOLLOW
            reset_wheel_pi()
            reset_control()
            result['state_msg'] = "[LOST -> FOLLOW] Captured!"
    else:
        if _ctrl_state == STATE_FOLLOW:
            if time.ticks_diff(now_ms, _last_target_ms) > LOST_TIMEOUT_MS:
                _ctrl_state = STATE_LOST
                stop_all()
                result['state_msg'] = "[FOLLOW -> LOST] Lost!"
                result['state'] = _ctrl_state
                return result

    result['state'] = _ctrl_state

    # ── FOLLOW 状态：级联 PID 控制 ──
    if _ctrl_state == STATE_FOLLOW:
        x_error = x_cm
        y_error = actual_dist - TARGET_DIST_CM

        # 外环：位置 PID → 目标速度
        cmd_fwd = _pid_fwd.compute(y_error, DT)
        cmd_lat = _pid_lat.compute(x_error, DT)

        # 速度滤波
        cmd_fwd, cmd_lat = speed_filter(cmd_fwd, cmd_lat)

        # 最低速度防死区
        if abs(cmd_fwd) > 0.01 and abs(cmd_fwd) < MIN_SPEED:
            cmd_fwd = MIN_SPEED if cmd_fwd > 0 else -MIN_SPEED
        if abs(cmd_lat) > 0.01 and abs(cmd_lat) < MIN_SPEED:
            cmd_lat = MIN_SPEED if cmd_lat > 0 else -MIN_SPEED

        # 到达判定
        if abs(y_error) < STOP_DIST_CM and abs(x_cm) < 10:
            _ctrl_state = STATE_STOPPED
            stop_all()
            result['arrived'] = True
            result['state_msg'] = "[FOLLOW -> STOPPED] dist={:.1f}cm".format(actual_dist)
            result['state'] = _ctrl_state
            return result

        result['cmd_fwd'] = cmd_fwd
        result['cmd_lat'] = cmd_lat

    # ── STOPPED 状态：等待目标移开 ──
    elif _ctrl_state == STATE_STOPPED:
        if abs(actual_dist - TARGET_DIST_CM) > STOP_DIST_CM * 2:
            _ctrl_state = STATE_FOLLOW
            reset_wheel_pi()
            reset_control()
            result['state_msg'] = "[STOPPED -> FOLLOW] Resuming."
            result['state'] = _ctrl_state

    # ── LOST 状态：停车等待 ──
    elif _ctrl_state == STATE_LOST:
        stop_all()

    return result


# ═══════════════════════════════════════════════════════════════
#  硬件初始化（供独立运行时使用）
# ═══════════════════════════════════════════════════════════════

SWITCH2_PIN = 'D9'
switch2 = None
state2 = 0
_recv = None

def init_hardware():
    """初始化摄像头跟随硬件（编码器接管 + CamDataReceiver）"""
    global switch2, state2, _recv

    switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
    state2  = switch2.value()

    time.sleep_ms(100)
    stop_all()
    enc_ticker.stop()
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)
    reset_encoder_filter()
    reset_wheel_pi()

    _recv = CamDataReceiver(uart_id=7)


# ═══════════════════════════════════════════════════════════════
#  独立运行入口
# ═══════════════════════════════════════════════════════════════

def standalone_main():
    """独立运行 cam_follow.py 时的主循环"""
    init_hardware()
    reset_control(reset_state=True)

    print("=" * 50)
    print("Camera Following (Cascade PID)")
    print("Target: {:.0f}cm".format(TARGET_DIST_CM))
    print("SW2 = exit")
    print("=" * 50)

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
        data = _recv.read()
        if data is None:
            time.sleep_ms(1)
            continue

        now = time.ticks_ms()

        # ── 调用核心控制 ──
        x_cm = x_to_cm(data['x'])
        actual_dist = y_to_distance(data['y'])
        ctrl = compute_control(x_cm, actual_dist, data['is_target'], now)

        # ── 状态变化消息 ──
        if ctrl['state_msg']:
            print(ctrl['state_msg'])

        # ── 驱动电机 ──
        if ctrl['cmd_fwd'] is not None:
            rc = get_encoder_counts()
            rs = [rc[i] / ENC_SCALE[i] / DT for i in range(4)]
            omni_drive_closed_loop(ctrl['cmd_fwd'], ctrl['cmd_lat'], 0, rs, DT)

        # ── 调试输出 ──
        if time.ticks_diff(now, last_print_ms) >= 300:
            state_str = {0: "FOLLOW", 1: "STOP", 2: "LOST"}[ctrl['state']]
            print("[#{:04d} {:s}] X:{:+5.1f}cm dist:{:5.1f}cm fwd:{:+.3f} lat:{:+.3f}".format(
                _recv.frame_count, state_str, x_cm, actual_dist,
                _pid_fwd.prev_output, _pid_lat.prev_output))
            last_print_ms = now

        loop_count += 1
        if loop_count % 50 == 0:
            gc.collect()
        time.sleep_ms(1)

    print("\n" + "=" * 50)
    print("Session: {} frames".format(_recv.frame_count))
    print("=" * 50)


# ── 作为脚本直接运行时执行独立模式 ──
if __name__ == '__main__':
    standalone_main()
