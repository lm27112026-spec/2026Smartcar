"""
10_position_closed_loop.py — IMU + 编码器位置闭环测试
【功能】
  使用编码器里程计 + IMU 航向保持，精确行驶到目标距离。
【方法】
  1) 锁定初始航向
  2) 距离 PID (kp=1.0) 输出 vy 速度指令
  3) 航向 PID (kp=0.02) 输出 wz 旋转修正
  4) 到达目标 ±2cm 自动停车
【用法】
  拨动 SWITCH2 退出
"""

import gc, time
from machine import Pin
from pid import PID
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    ENC_SCALE, LED_PIN, SWITCH2_PIN, MAX_PWM, MAX_SPEED_MPS,
)
import imu_motion

# ── 硬件 ──
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ── 参数 ──
TARGET_DIST_M = 0.50        # 目标距离 50cm
DT = 0.02                   # 控制周期 20ms
TIMEOUT_S = 10
PRINT_INTERVAL_MS = 200

# 距离 PID
DIST_KP = 1.0               # 1m 误差 → 0.4 速度指令（被 output_limit 钳）
DIST_KI = 0.5               # 消除残余距离误差（快速积累）
DIST_OUTPUT_LIMIT = 0.30    # 最大速度 0.30×0.43=0.13 m/s

# 航向 PID（沿用 Layer 2 验证过的参数）
HEADING_KP = 0.15
HEADING_KI = 0.005
YAW_DEADBAND = 0.3
WZ_LIMIT = 0.5

# ── 初始化 ──
stop_all()
enc_ticker.stop()
time.sleep_ms(50)

for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)

# ── IMU 初始化 & 锁定航向 ──
for _ in range(10):
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

target_heading = imu_motion.yaw
print("\n" + "=" * 60)
print("  Position Closed-Loop Test (IMU + Encoder)")
print("  Target: {:.0f}cm | Heading: {:.1f} deg".format(
    TARGET_DIST_M * 100, target_heading))
print("  Dist PID: Kp={:.1f} Ki={:.1f} | Limit={:.1f}".format(
    DIST_KP, DIST_KI, DIST_OUTPUT_LIMIT))
print("  Head PID: Kp={:.2f} Ki={:.3f} | Deadband={:.1f}".format(
    HEADING_KP, HEADING_KI, YAW_DEADBAND))
print("=" * 60)

# ── PID 初始化 ──
pid_dist = PID(kp=DIST_KP, ki=DIST_KI, kd=0.0,
               integral_limit=0.5, output_limit=DIST_OUTPUT_LIMIT)

# ── 航向 PID 状态 ──
heading_integral = 0.0
prev_heading_deviation = 0.0

# ── 主循环 ──
total_counts = [0, 0, 0, 0]
start_ms = time.ticks_ms()
last_print_ms = start_ms

print("\n  Dist PID: 目标距离 ↓ → 实际越近 → vy 越慢 → 平滑停车")
print("-" * 70)
print("  {:>5s}  {:>6s}  {:>7s} {:>7s} {:>6s} {:>6s}".format(
    "time", "dist", "vy", "yaw", "wz", "spd"))
print("-" * 70)

while True:
    now_ms = time.ticks_ms()
    elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

    # 超时
    if elapsed_s > TIMEOUT_S:
        print("\n  TIMEOUT!")
        break

    # SWITCH2 中止
    if switch2.value() != state2:
        print("\n  SWITCH2 aborted")
        break

    # ---- 编码器读取（一次读取，同时用于速度和距离）----
    raw_counts = get_encoder_counts()
    raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]
    for i in range(4):
        total_counts[i] += raw_counts[i]

    wheel_dist = [abs(total_counts[i]) / ENC_SCALE[i] for i in range(4)]
    current_dist = sum(wheel_dist) / 4

    # ---- IMU 航向 ----
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    heading_deviation = target_heading - imu_motion.yaw
    while heading_deviation > 180: heading_deviation -= 360
    while heading_deviation < -180: heading_deviation += 360

    # 跨 ±180° 边界时重置积分（防止积分跳变）
    if abs(heading_deviation - prev_heading_deviation) > 180:
        heading_integral = 0.0
    prev_heading_deviation = heading_deviation

    # ---- 航向 PID ----
    if abs(heading_deviation) > YAW_DEADBAND:
        heading_integral += heading_deviation * DT
        heading_integral = max(-3.0, min(heading_integral, 3.0))
        wz = (HEADING_KP * heading_deviation +
              HEADING_KI * heading_integral)
        wz = max(-WZ_LIMIT, min(wz, WZ_LIMIT))
    else:
        wz = 0.0
        heading_integral = 0.0

    # ---- 距离 PID ----
    vy = pid_dist.compute(setpoint=TARGET_DIST_M, measurement=current_dist, dt=DT)
    # 最低速度防摩擦死区（仅未到目标时生效）
    if current_dist < TARGET_DIST_M and vy < 0.08:
        vy = 0.08

    # ---- 闭环驱动 ----
    omni_drive_closed_loop(vy, 0, wz, raw_speeds, DT)

    # ---- 打印 ----
    if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
        last_print_ms = now_ms
        avg_speed = sum(abs(s) for s in raw_speeds) / 4
        print("  {:5.1f}s  {:5.1f}cm  {:+.3f}  {:6.1f} {:+.3f} {:5.3f}".format(
            elapsed_s, current_dist * 100, vy,
            imu_motion.yaw, wz, avg_speed))

    # ---- 停车判断 ----
    if current_dist >= TARGET_DIST_M:
        break

    time.sleep_ms(int(DT * 1000))
    gc.collect()

# ── 停止 & 报告 ──
omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
stop_all()
enc_ticker.start(10)
led.off()

yaw_drift = imu_motion.yaw - target_heading
while yaw_drift > 180: yaw_drift -= 360
while yaw_drift < -180: yaw_drift += 360

print("\n" + "=" * 60)
print("  Position Loop Report")
print("=" * 60)
print("  Target distance = {:.0f}cm".format(TARGET_DIST_M * 100))
print("  Final distance  = {:.1f}cm".format(current_dist * 100))
print("  Yaw drift       = {:+.1f} deg".format(yaw_drift))
print("  Duration        = {:.1f}s".format(
    time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0))
print("=" * 60)
