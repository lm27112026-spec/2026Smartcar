"""
motion.py - 全闭环运动控制层
================================
三层闭环融合控制，实现抗干扰直线行驶：

  第1层 航向锁定  — IMU yaw → PID → wz 修正
  第2层 横向纠偏  — 编码器横向速度积分 → PID → vy 修正
  第3层 轮速闭环  — motor.py 前馈 + 每轮 PI（崎岖路面自适应）

特性：-
  - 被手推撞后自动恢复原路径（航向 + 横向双向纠偏）
  - 崎岖/不平路面稳定直行（每轮独立 PI 速度闭环）
  - 编码器里程计精确距离控制（末端自动减速）

使用方法：
  from motion import go_forward_straight
  go_forward_straight(1.0, speed=0.3)  # 前进 1 米

依赖：
  motor.py  — get_encoder_counts, omni_drive_closed_loop, WHEEL_PI, ...
  imu_motion.py — imu, update_angle, yaw
  pid.py    — PID
"""

import gc, time
from machine import Pin
from smartcar import ticker
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    ENC_SCALE, MAX_SPEED_MPS, WHEEL_PI,
    LED_PIN, SWITCH2_PIN,
)
from imu_motion import imu, update_angle
import imu_motion
from pid import PID

# ============================================================
#  编码器捕获定时器（模块级，一次性创建，避免逐次分配硬件资源）
# ============================================================
_ENC_PIT = ticker(1)
_ENC_PIT.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)

# ============================================================
#  PID 参数（根据实际车况微调）
# ============================================================

# --- 航向 PID：度误差 → wz（-1~1）---
HEADING_KP        = 0.15
HEADING_KI        = 0.002
HEADING_KD        = 0.0
HEADING_INT_LIM   = 0.2
HEADING_OUT_LIM   = 0.5

# --- 横向位置 PID：米误差 → vy（-1~1）---
LATERAL_KP        = 3.0
LATERAL_KI        = 0.5
LATERAL_KD        = 0.0
LATERAL_INT_LIM   = 0.3
LATERAL_OUT_LIM   = 0.4

# ============================================================
#  内部：编码器低通滤波状态
# ============================================================
_spd_filt   = [0.0, 0.0, 0.0, 0.0]
_spd_first  = True
_SPD_ALPHA  = 0.4          # 滤波系数（越小越平滑，越大响应越快）


def _reset_filter():
    """复位滤波状态（每次运动开始前调用）"""
    global _spd_filt, _spd_first
    _spd_filt  = [0.0, 0.0, 0.0, 0.0]
    _spd_first = True


def _reset_all_state():
    """清除积分/滤波状态 + 重置 IMU 航向（每轮重新锁定向 0°）"""
    # 编码器速度滤波
    _reset_filter()
    # IMU 姿态重置（yaw=0 后重新锁定向，消除跨轮次漂移累积）
    imu_motion.yaw = 0.0
    imu_motion.last_time = 0
    imu_motion.gz_filtered = 0.0
    # motor.py 每轮 PI 积分
    for pi in WHEEL_PI:
        pi.reset()


def _read_encoders(dt):
    """
    每周期只调用一次：读取 4 路编码器，返回 (原始脉冲增量, 滤波速度 m/s)
    内部维护一阶低通滤波状态
    """
    global _spd_filt, _spd_first
    raw = get_encoder_counts()                              # 消费编码器数据
    spd = [raw[i] / ENC_SCALE[i] / dt for i in range(4)]
    if _spd_first:
        _spd_filt  = spd[:]
        _spd_first = False
    else:
        _spd_filt = [
            _SPD_ALPHA * p + (1 - _SPD_ALPHA) * r
            for p, r in zip(_spd_filt, spd)
        ]
    return raw, _spd_filt


def _estimate_body_vy(actual_spd):
    """
    4 轮速度（m/s）→ 本体坐标系横向速度（m/s）
    正值 = 向右平移
    前向运动学（矩阵伪逆）：vy = (rf - lf - lb + rb) / 4
    """
    w = [actual_spd[i] / MAX_SPEED_MPS[i] for i in range(4)]
    vy_norm = (w[0] - w[1] - w[2] + w[3]) / 4.0
    avg_max = sum(MAX_SPEED_MPS) / 4.0
    return vy_norm * avg_max


# ============================================================
#  核心：全闭环直线行驶
# ============================================================

def go_forward_straight(target_m, speed=0.3):
    """
    全闭环直线行驶：航向锁定 + 横向纠偏 + 前向距离控制

    参数:
        target_m : 目标前进距离（米），例如 1.0 = 前进 1 米
        speed    : 巡航速度 0~1（默认 0.3），末端自动减速

    返回:
        True  — 到达目标距离
        False — SWITCH2 按下或超时退出

    控制原理:
        ┌──────────┐     ┌──────────┐     ┌──────────┐
        │ IMU yaw  │────▶│ 航向 PID │────▶│   wz     │
        └──────────┘     └──────────┘     │          │
        ┌──────────┐     ┌──────────┐     │ omni_    │
        │ 编码器→  │────▶│ 横向 PID │────▶│ drive_   │──▶ 电机
        │ 横向位置 │     └──────────┘     │ closed_  │
        └──────────┘                      │ loop     │
        ┌──────────┐                      │          │
        │ 编码器→  │───────────────────▶│   vx     │
        │ 前向距离 │                      └──────────┘
        └──────────┘

    示例:
        go_forward_straight(0.60)   # 前进 60cm
    """
    # ===== 硬件初始化 =====
    led    = Pin(LED_PIN, Pin.OUT, value=True)
    switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
    s2st   = switch2.value()

    # 启动编码器捕获定时器（模块级 _ENC_PIT，避免逐次创建硬件定时器）
    _ENC_PIT.stop()
    _ENC_PIT.start(10)

    # 清除上次运行的所有积分状态（陀螺仪 + 编码器 + 轮速 PI）
    _reset_all_state()

    # ===== 编码器归零 & 里程计初始化 =====
    for _ in range(5):
        encoder_rf.get(); encoder_lf.get(); encoder_lb.get(); encoder_rb.get()
        time.sleep_ms(10)

    total_counts  = [0, 0, 0, 0]    # 4 轮脉冲累加
    lateral_pos   = 0.0             # 横向偏离累计（米），目标 = 0
    forward_dist  = 0.0             # 前进距离累计（米）

    # ===== 锁定起始航向 =====
    # 第1次 update_angle：初始化 last_time（由于 _reset_all_state 已设 last_time=0，返回不更新 yaw）
    d = imu.read()
    update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    # 第2次 update_angle：真正积分当前 gyro 数据，yaw ≈ 0°（因为 _reset_all_state 已设 yaw=0）
    d = imu.read()
    update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    target_yaw = imu_motion.yaw

    # ===== PID 控制器 =====
    heading_pid = PID(
        kp=HEADING_KP, ki=HEADING_KI, kd=HEADING_KD,
        integral_limit=HEADING_INT_LIM, output_limit=HEADING_OUT_LIM,
    )
    lateral_pid = PID(
        kp=LATERAL_KP, ki=LATERAL_KI, kd=LATERAL_KD,
        integral_limit=LATERAL_INT_LIM, output_limit=LATERAL_OUT_LIM,
    )

    # ===== 参数 =====
    DT       = 0.02                # 控制周期 20ms（与 motor.CTRL_DT 一致）
    start_ms = time.ticks_ms()
    timeout_s = target_m / 0.03 + 15   # 动态超时（慢速预留充足时间）

    print("\n[go_forward_straight] target={:.0f}cm  speed={:.1f}  yaw={:.1f}°".format(
        target_m * 100, speed, target_yaw))
    print("  Press SWITCH2 to stop.\n")

    # ===== 主控制循环 =====
    while True:
        # ---- 退出条件 ----
        if time.ticks_diff(time.ticks_ms(), start_ms) > timeout_s * 1000:
            print("Timeout! {:.0f}s elapsed.".format(timeout_s))
            stop_all(); _ENC_PIT.stop(); led.off()
            _reset_all_state()
            return False
        if switch2.value() != s2st:
            print("SW2 stop")
            stop_all(); _ENC_PIT.stop(); led.off()
            _reset_all_state()
            return False

        # ---- 1. IMU 姿态更新 ----
        d = imu.read()
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

        # ---- 2. 编码器读取（单次，复用：距离 + 速度 + 横向估计）----
        raw_counts, actual_spd = _read_encoders(DT)

        # 脉冲 → 前进距离（4 轮平均）
        for i in range(4):
            total_counts[i] += raw_counts[i]
        wheel_dists = [
            abs(total_counts[i]) / abs(ENC_SCALE[i])
            for i in range(4)
        ]
        forward_dist = sum(wheel_dists) / 4.0

        # 横向速度 → 横向位置累计（正值 = 右偏）
        vy_body = _estimate_body_vy(actual_spd)
        lateral_pos += vy_body * DT

        # ---- 3. 航向 PID：保持直线方向 ----
        heading_error = target_yaw - imu_motion.yaw
        heading_error = (heading_error + 180) % 360 - 180   # 包裹到 [-180,180)
        wz = heading_pid.compute(0, heading_error, DT)

        # ---- 4. 横向位置 PID：锁定原路径（横向偏离 → 0）----
        vy_correction = lateral_pid.compute(0, lateral_pos, DT)

        # ---- 5. 前向速度（末端 15cm 线性减速，最低 0.10 克服静摩擦）----
        remaining = target_m - forward_dist
        if remaining < 0.15:
            vx_cmd = max(0.10, speed * remaining / 0.15)
        else:
            vx_cmd = speed

        # ---- 6. 闭环驱动（前馈 + 每轮 PI 速度闭环）----
        omni_drive_closed_loop(vx_cmd, vy_correction, wz, actual_spd)

        # ---- 7. 到达目标 ----
        if forward_dist >= target_m:
            stop_all(); _ENC_PIT.stop(); led.off()
            _reset_all_state()
            print("\n>>> Reached target {:.0f}cm <<<".format(target_m * 100))
            return True

        # ---- 8. 调试打印（每 500ms）----
        tick = time.ticks_diff(time.ticks_ms(), start_ms)
        if tick % 500 < 20:
            led.toggle()
            print("t={:.1f}s yaw={:.1f}° lat={:+.4f}m fwd={:.3f}m | wz={:+.2f} vy={:+.3f} vx={:.2f}".format(
                tick / 1000.0, imu_motion.yaw, lateral_pos, forward_dist,
                wz, vy_correction, vx_cmd))

        time.sleep_ms(20)
        gc.collect()


# ============================================================
#  直接运行示例：前进 60cm
#  烧录此文件后上电即执行，或 import motion 时不执行
# ============================================================
if __name__ == "__main__":
    print("=== motion.py standalone demo ===")
    go_forward_straight(0.60, speed=0.3)
    print("=== Demo finished ===")
