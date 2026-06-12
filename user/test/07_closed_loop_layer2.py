"""
07_closed_loop_layer2.py - Layer 2: 轮速 PI + IMU 航向 PID
【功能】
  在 Layer 1 基础上加入 IMU 航向保持，验证直线行驶不跑偏。
【方法】
  1) 启动时锁定当前航向作为目标
  2) 每 20ms 读取 IMU yaw，计算航向误差
  3) 航向 PID 输出 wz 修正量
  4) omni_drive_closed_loop(vx, 0, wz) 驱动
  5) 行驶 25cm 后停止，报告距离差和偏航角
"""

import gc, time, math
from machine import Pin
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    ENC_SCALE, LED_PIN, SWITCH2_PIN, MAX_PWM,
)
import imu_motion

# ============================================================
#  航向 PID 参数
# ============================================================
HEADING_KP = 0.02      # P 增益（先从保守值开始）
HEADING_KI = 0.001     # I 增益
HEADING_KD = 0.0       # D 增益
YAW_DEADBAND = 1.0     # 死区（度）：<1° 不纠偏
WZ_LIMIT = 0.3         # wz 限幅

# ============================================================
#  常量
# ============================================================
DRIVE_SPEED = 0.3
TARGET_DIST_M = 0.25
TIMEOUT_S = 15
PRINT_INTERVAL_MS = 500

# ============================================================
#  初始化
# ============================================================
stop_all()
time.sleep_ms(50)

led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

enc_ticker.stop()

# 归零编码器
for _ in range(5):
    get_encoder_counts()
    time.sleep_ms(10)

# ============================================================
#  IMU 初始化 & 锁定起始航向
# ============================================================
print("\n" + "=" * 60)
print("  Layer 2: Wheel PI + Heading PID")
print("  Speed: {:.1f} | Target: {:.0f}cm".format(DRIVE_SPEED, TARGET_DIST_M * 100))
print("  Heading PID: Kp={:.3f} Ki={:.4f} Kd={:.1f}".format(
    HEADING_KP, HEADING_KI, HEADING_KD))
print("  Deadband: {:.1f} deg | wz limit: {:.1f}".format(YAW_DEADBAND, WZ_LIMIT))
print("=" * 60)
print("")

# 读取 IMU 并校准
for _ in range(5):
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

target_heading = imu_motion.yaw
print("  Target heading: {:.2f} deg".format(target_heading))
print("  {:>6s}  {:>7s} {:>7s} {:>7s} {:>7s}  {:>7s} {:>6s}".format(
    "t(s)", "RF_spd", "LF_spd", "LB_spd", "RB_spd", "yaw", "wz"))
print("-" * 75)

# ============================================================
#  航向 PID 状态
# ============================================================
heading_integral = 0.0
prev_heading_error = 0.0

# ============================================================
#  主循环
# ============================================================
total_counts = [0, 0, 0, 0]
start_ms = time.ticks_ms()
last_print_ms = start_ms

while True:
    now_ms = time.ticks_ms()
    elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

    # 超时
    if elapsed_s > TIMEOUT_S:
        print("\n  TIMEOUT!")
        break

    # SWITCH2 中止
    if switch2.value() != state2:
        print("\n  SW2 aborted")
        break

    # ---- IMU 更新航向 ----
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

    # ---- 航向 PID ----
    heading_error = target_heading - imu_motion.yaw
    while heading_error > 180: heading_error -= 360
    while heading_error < -180: heading_error += 360

    if abs(heading_error) > YAW_DEADBAND:
        heading_integral += heading_error * 0.02
        heading_integral = max(-0.5, min(heading_integral, 0.5))  # 积分限幅
        d_error = heading_error - prev_heading_error
        wz = (HEADING_KP * heading_error +
              HEADING_KI * heading_integral +
              HEADING_KD * d_error)
        wz = max(-WZ_LIMIT, min(wz, WZ_LIMIT))
    else:
        wz = 0.0
    prev_heading_error = heading_error

    # ---- 编码器读取（一次采集，同时用于速度反馈和距离累计）----
    raw_counts = get_encoder_counts()
    raw_speeds = [raw_counts[i] / ENC_SCALE[i] / 0.02 for i in range(4)]
    for i in range(4):
        total_counts[i] += raw_counts[i]

    # ---- 闭环驱动 ----
    omni_drive_closed_loop(DRIVE_SPEED, 0, wz, raw_speeds, 0.02)

    wheel_dist = [abs(total_counts[i]) / ENC_SCALE[i] for i in range(4)]
    avg_dist = sum(wheel_dist) / 4

    # ---- 定期打印 ----
    if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
        last_print_ms = now_ms
        print("  {:6.2f}  {:7.4f} {:7.4f} {:7.4f} {:7.4f}  {:7.2f} {:6.3f}".format(
            elapsed_s, raw_speeds[0], raw_speeds[1], raw_speeds[2], raw_speeds[3],
            imu_motion.yaw, wz))

    # ---- 目标距离 ----
    if avg_dist >= TARGET_DIST_M:
        break

    time.sleep_ms(20)
    gc.collect()

# ============================================================
#  停止 & 报告
# ============================================================
omni_drive_closed_loop(0, 0, 0, [0,0,0,0], 0.02)
stop_all()
enc_ticker.start(10)
led.off()

actual_duration_s = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
final_yaw = imu_motion.yaw
yaw_drift = final_yaw - target_heading
while yaw_drift > 180: yaw_drift -= 360
while yaw_drift < -180: yaw_drift += 360

print("\n" + "=" * 60)
print("  Layer 2 Report ({:.2f}s)".format(actual_duration_s))
print("=" * 60)

wheel_names = ["RF", "LF", "LB", "RB"]
for i in range(4):
    dist_m = abs(total_counts[i]) / ENC_SCALE[i]
    print("  {} = {:.4f}m ({:.1f}cm)".format(
        wheel_names[i], dist_m, dist_m * 100))

distances = [abs(total_counts[i]) / ENC_SCALE[i] for i in range(4)]
spread_m = max(distances) - min(distances)
spread_cm = spread_m * 100

print("-" * 60)
print("  Max-Min spread = {:.4f}m ({:.1f}cm)".format(spread_m, spread_cm))
print("  Yaw drift = {:+.2f} deg".format(yaw_drift))

if spread_cm < 1.5 and abs(yaw_drift) < 2.0:
    print("  PASS: Spread {:.1f}cm + Yaw {:.1f}deg".format(spread_cm, yaw_drift))
    print("  -> Proceed to Layer 3 (lateral control).")
elif spread_cm < 3.0 and abs(yaw_drift) < 5.0:
    print("  MARGINAL: Spread {:.1f}cm, Yaw {:.1f}deg".format(spread_cm, yaw_drift))
else:
    print("  NEEDS TUNING: Spread {:.1f}cm, Yaw {:.1f}deg".format(spread_cm, yaw_drift))
    print("  -> Adjust HEADING_KP / deadband / wz_limit.")
print("=" * 60)
