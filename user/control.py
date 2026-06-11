"""control.py — 级联 PID 控制器（外环位置环 + 内环速度环委托给 motor.py）"""

import time
from pid import PID
from motor import get_encoder_counts, get_encoder_speeds_filtered, omni_drive_closed_loop, enc_ticker, stop_all

# === 控制参数 ===
TARGET_DIST = 70

PID_EX_KP   = 0.020
PID_EX_KI   = 0.0
PID_EX_KD   = 0.0
PID_DIST_KP = 1.0
PID_DIST_KI = 0.0
PID_DIST_KD = 0.0
PID_ROLL_KP = 0.02
PID_ROLL_KI = 0.001
PID_ROLL_KD = 0.0
PID_I_LIMIT   = 0.5
PID_OUT_LIMIT = 0.4

DT_CLAMP     = 0.1
DT_FALLBACK  = 0.02


class CascadeController:
    """级联 PID 控制器：外环(位置) → omni_drive_closed_loop(内环速度)"""

    def __init__(self):
        self.pid_ex   = PID(kp=PID_EX_KP,   ki=PID_EX_KI,   kd=PID_EX_KD,
                            integral_limit=PID_I_LIMIT, output_limit=PID_OUT_LIMIT)
        self.pid_dist = PID(kp=PID_DIST_KP, ki=PID_DIST_KI, kd=PID_DIST_KD,
                            integral_limit=PID_I_LIMIT, output_limit=PID_OUT_LIMIT)
        self.pid_roll = PID(kp=PID_ROLL_KP, ki=PID_ROLL_KI, kd=PID_ROLL_KD,
                            integral_limit=PID_I_LIMIT, output_limit=PID_OUT_LIMIT)
        self._last_time = time.ticks_ms()
        enc_ticker.stop()
        _ = get_encoder_counts()

    def step(self, ex_f, dist_f, roll_f, tracking):
        """
        ex_f: 画面横向偏移量 → 对应平移 vx
        dist_f: 距离偏移量 → 对应前后移动 vy
        roll_f: 角度偏移量 → 对应自转 wz
        tracking: 是否检测到目标
        """
        now_ms = time.ticks_ms()
        dt = time.ticks_diff(now_ms, self._last_time) * 0.001
        self._last_time = now_ms
        if dt > DT_CLAMP:
            dt = DT_FALLBACK

        actual_speeds = get_encoder_speeds_filtered(dt)

        if tracking:
            vx_out = self.pid_ex.compute(setpoint=0, measurement=ex_f, dt=dt)
            vy_out = self.pid_dist.compute(setpoint=TARGET_DIST, measurement=dist_f, dt=dt)
            wz_out = self.pid_roll.compute(setpoint=0, measurement=roll_f, dt=dt)
            vx_out = max(-0.8, min(vx_out, 0.8))
            vy_out = max(-0.8, min(vy_out, 0.8))
            wz_out = max(-0.6, min(wz_out, 0.6))
        else:
            vx_out = vy_out = wz_out = 0.0
            self.pid_ex.reset()
            self.pid_dist.reset()
            self.pid_roll.reset()

        omni_drive_closed_loop(vx_out, vy_out, wz_out, actual_speeds, dt)
        return vx_out, vy_out, wz_out, actual_speeds, dt

    def emergency_stop(self):
        stop_all()
        enc_ticker.start(10)
        self.pid_ex.reset()
        self.pid_dist.reset()
        self.pid_roll.reset()
