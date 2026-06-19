"""control.py — 级联 PID 控制器（外环位置环 + 内环速度环委托给 motor.py）

【IMU 读取策略】
  step() 不自行调用 imu.get() / imu.read()，而是由 main.py 在每次迭代顶部
  统一读取 IMU 并传入 actual_wz_dps。这保证了 SPI 访问集中在调用方，便于
  错误处理和看门狗恢复。
【NaN 防护】
  所有 PID 输出过 clamp 前检查 NaN，发现 NaN 立即降级为零速输出。
"""

import time, math
from pid import PID
from motor import get_encoder_counts, get_encoder_speeds_filtered, omni_drive_closed_loop, stop_all
from imu_motion import get_angular_velocity, angular_velocity_control, reset_ang_vel_pid

# === 控制参数 ===
TARGET_DIST = 10   # 视觉逼近目标距离（cm），匹配 dist_f <= 10 模式切换条件

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

# 角速度换算：归一化 wz=1.0 对应 MAX_WZ_DPS dps（需与 imu_motion.py 保持一致）
MAX_WZ_DPS = 384.0  # 标定值：wz=0.3 时实测 115.3 dps → 115.3/0.3 = 384


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
        _ = get_encoder_counts()
        # IMU 角速度闭环初始化由 caller (main.py) 在 enter_visual() 中完成
        reset_ang_vel_pid()

    def step(self, ex_f, dist_f, roll_f, tracking, actual_wz_dps=0.0):
        """
        ex_f: 画面横向偏移量 → 对应平移 vx
        dist_f: 距离偏移量 → 对应前后移动 vy
        roll_f: 角度偏移量 → 对应自转 wz
        tracking: 是否检测到目标
        actual_wz_dps: 当前 Z 轴角速度 (dps)，由调用方提供。
                       调用方应在每次迭代前统一读取 IMU 并计算此值。
                       传入 0.0 等效于关闭角速度闭环（降级）。
        """
        now_ms = time.ticks_ms()
        dt = time.ticks_diff(now_ms, self._last_time) * 0.001
        self._last_time = now_ms
        if dt > DT_CLAMP or dt <= 0:
            dt = DT_FALLBACK

        actual_speeds = get_encoder_speeds_filtered(dt)

        try:
            if tracking:
                # ── NaN 防护：任一输入为 NaN 则降级为零速 ──
                if (math.isnan(ex_f) or math.isnan(dist_f) or math.isnan(roll_f)
                        or math.isnan(actual_wz_dps)):
                    vx_out = vy_out = 0.0
                    wz_corrected = 0.0
                    self.pid_ex.reset()
                    self.pid_dist.reset()
                    self.pid_roll.reset()
                    reset_ang_vel_pid()
                else:
                    vx_out = self.pid_ex.compute(setpoint=0, measurement=ex_f, dt=dt)
                    vy_out = -self.pid_dist.compute(setpoint=TARGET_DIST, measurement=dist_f, dt=dt)
                    wz_out = self.pid_roll.compute(setpoint=0, measurement=roll_f, dt=dt)
                    vx_out = max(-0.8, min(vx_out, 0.8))
                    vy_out = max(-0.8, min(vy_out, 0.8))
                    wz_out = max(-0.6, min(wz_out, 0.6))

                    # ── 角速度闭环：视觉 PID 输出的 wz → 目标 dps → 角速度 PID 纠正 → 归一化 wz ──
                    target_dps = wz_out * MAX_WZ_DPS
                    wz_corrected = angular_velocity_control(target_dps, actual_wz_dps, dt)
            else:
                vx_out = vy_out = 0.0
                wz_corrected = 0.0
                self.pid_ex.reset()
                self.pid_dist.reset()
                self.pid_roll.reset()
                reset_ang_vel_pid()

            omni_drive_closed_loop(vx_out, vy_out, wz_corrected, actual_speeds, dt)
            return vx_out, vy_out, wz_corrected, actual_speeds, dt

        except Exception:
            # step() 内部异常 → 急停 + 降级返回零速
            stop_all()
            self.pid_ex.reset()
            self.pid_dist.reset()
            self.pid_roll.reset()
            reset_ang_vel_pid()
            return 0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 0.0], dt

    def emergency_stop(self):
        stop_all()
        self.pid_ex.reset()
        self.pid_dist.reset()
        self.pid_roll.reset()
        reset_ang_vel_pid()


