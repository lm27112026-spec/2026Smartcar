"""
08_closed_loop_layer3.py - Layer 3: 完整闭环（轮速 PI + 航向 PID + 距离控制）
【功能】
  完整闭环系统：轮速保持 + 航向保持 + 精确距离停车。
【方法】
  1) omni_drive_closed_loop + heading PID（同 Layer 2）
  2) 编码器里程计精确计算行驶距离
  3) 到达目标距离时自动停车
  4) 报告：距离误差、偏航漂移、轮间差异
【判据】
  PASS:    距离误差 < 2cm 且 yaw < 2°
  MARGINAL: 距离误差 2-5cm 或 yaw 2-5°
  FAIL:    距离误差 > 5cm 或 yaw > 5°
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
HEADING_KP = 0.02
HEADING_KI = 0.001
HEADING_KD = 0.0
YAW_DEADBAND = 1.0
WZ_LIMIT = 0.3

# ============================================================
#  常量
# ============================================================
DRIVE_SPEED = 0.3
TARGET_DIST_M = 0.25
TIMEOUT_S = 15
PRINT_INTERVAL_MS = 200

# ============================================================
#  初始化
# ============================================================
stop_all()
time.sleep_ms(50)

led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

enc_ticker.stop()

for _ in range(5):
    get_encoder_counts()
    time.sleep_ms(10)

# ============================================================
#  IMU 初始化 & 锁定航向
# ============================================================
print("\n" + "=" * 60)
print("  Layer 3: Full Closed-Loop (Speed + Heading + Distance)")
print("  Speed: {:.1f} | Target: {:.0f}cm".format(DRIVE_SPEED, TARGET_DIST_M * 100))
print("  Heading PID: Kp={:.3f} Ki={:.4f}".format(HEADING_KP, HEADING_KI))
print("=" * 60)
print("")

for _ in range(5):
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

target_heading = imu_motion.yaw
print("  Target heading: {:.2f} deg".format(target_heading))
print("  {:>6s}  {:>7s} {:>7s} {:>7s} {:>7s}  {:>7s} {:>6s} {:>6s}".format(
    "t(s)", "RF_spd", "LF_spd", "LB_spd", "RB_spd", "yaw", "wz", "dist"))
print("-" * 85)

# ============================================================
#  PID 状态
# ============================================================
heading_integral = 0.0
prev_heading_error = 0.0

# ============================================================
#  主循环
# ============================================================
total_counts = [0, 0, 0, 0]
start_ms = time.ticks_ms()
last_print_ms = start_ms
reached = False

while True:
    now_ms = time.ticks_ms()
    elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

    if elapsed_s > TIMEOUT_S:
        print("\n  TIMEOUT!")
        break

    if switch2.value() != state2:
        print("\n  SW2 aborted")
        break

    # ---- IMU ----
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

    # ---- 航向 PID ----
    heading_error = target_heading - imu_motion.yaw
    while heading_error > 180: heading_error -= 360
    while heading_error < -180: heading_error += 360

    if abs(heading_error) > YAW_DEADBAND:
        heading_integral += heading_error * 0.02
        heading_integral = max(-0.5, min(heading_integral, 0.5))
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
        print("  {:6.2f}  {:7.4f} {:7.4f} {:7.4f} {:7.4f}  {:7.2f} {:6.3f} {:5.2f}".format(
            elapsed_s, raw_speeds[0], raw_speeds[1], raw_speeds[2], raw_speeds[3],
            imu_motion.yaw, wz, avg_dist * 100))

    # ---- 精确距离停车 ----
    if avg_dist >= TARGET_DIST_M and not reached:
        reached = True
        omni_drive_closed_loop(0, 0, 0, [0,0,0,0], 0.02)
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
print("  Layer 3 Report ({:.2f}s)".format(actual_duration_s))
print("=" * 60)

wheel_names = ["RF", "LF", "LB", "RB"]
for i in range(4):
    dist_m = abs(total_counts[i]) / ENC_SCALE[i]
    print("  {} = {:.4f}m ({:.1f}cm)".format(
        wheel_names[i], dist_m, dist_m * 100))

distances = [abs(total_counts[i]) / ENC_SCALE[i] for i in range(4)]
avg_distance = sum(distances) / 4
spread_m = max(distances) - min(distances)
spread_cm = spread_m * 100
distance_error_cm = (avg_distance - TARGET_DIST_M) * 100

print("-" * 60)
print("  Target distance  = {:.1f}cm".format(TARGET_DIST_M * 100))
print("  Actual distance  = {:.1f}cm".format(avg_distance * 100))
print("  Distance error   = {:+.1f}cm".format(distance_error_cm))
print("  Wheel spread     = {:.1f}cm".format(spread_cm))
print("  Yaw drift        = {:+.2f} deg".format(yaw_drift))

abs_err = abs(distance_error_cm)
if abs_err < 2.0 and abs(yaw_drift) < 2.0:
    print("\n  PASS: Error {:.1f}cm < 2cm, Yaw {:.1f}deg < 2deg".format(abs_err, abs(yaw_drift)))
    print("  -> System ready for autonomous navigation!")
elif abs_err < 5.0 and abs(yaw_drift) < 5.0:
    print("\n  MARGINAL: Error {:.1f}cm, Yaw {:.1f}deg".format(abs_err, abs(yaw_drift)))
else:
    print("\n  FAIL: Error {:.1f}cm, Yaw {:.1f}deg".format(abs_err, abs(yaw_drift)))
print("=" * 60)
