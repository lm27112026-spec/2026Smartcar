"""
05_closed_loop_layer1.py - Layer 1: 轮速 PI 闭环测试
【功能】
  仅测试轮速 PI 控制器（前馈 + 反馈），不加航向/横向控制。
  验证 4 轮在闭环下速度是否一致，消除机械差异。
【方法】
  1) omni_drive_closed_loop(0.3, 0, 0) 驱动
  2) 每 200ms 打印 4 轮目标速度、实际速度、PWM
  3) 行驶 25cm 后停止，输出报告
【判据】
  PASS: 4 轮最终距离差 < 1.5cm
  MARGINAL: 距离差 1.5-3.0cm
  FAIL: 距离差 >= 3.0cm
"""

import gc, time
from machine import Pin
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    get_encoder_speeds_filtered,
    enc_ticker,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    ENC_SCALE, LED_PIN, SWITCH2_PIN, MAX_PWM,
)

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

# 归零编码器
for _ in range(5):
    get_encoder_counts()
    time.sleep_ms(10)

# ============================================================
#  Header
# ============================================================
print("\n" + "=" * 60)
print("  Layer 1: Wheel Speed PI Closed-Loop Test")
print("  Speed: {:.1f} | Target: {:.0f}cm | Timeout: {}s".format(
    DRIVE_SPEED, TARGET_DIST_M * 100, TIMEOUT_S))
print("  SWITCH2 to abort")
print("=" * 60)
print("")
print("  {:>6s}  {:>8s} {:>8s} {:>8s} {:>8s}".format(
    "t(s)", "RF", "LF", "LB", "RB"))
print("  {:>6s}  {:>8s} {:>8s} {:>8s} {:>8s}".format(
    "", "target", "target", "target", "target"))
print("-" * 60)

# ============================================================
#  主循环
# ============================================================
total_counts = [0, 0, 0, 0]
start_ms = time.ticks_ms()
last_print_ms = start_ms
last_ctrl_ms = start_ms

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

    # 闭环驱动
    omni_drive_closed_loop(DRIVE_SPEED, 0, 0)

    # 编码器累计
    counts = get_encoder_counts()
    for i in range(4):
        total_counts[i] += counts[i]

    # 距离计算
    wheel_dist = [abs(total_counts[i]) / ENC_SCALE[i] for i in range(4)]
    avg_dist = sum(wheel_dist) / 4

    # 定期打印
    if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
        last_print_ms = now_ms
        speeds = get_encoder_speeds_filtered(0.02)
        print("  {:6.2f}  {:7.4f} {:7.4f} {:7.4f} {:7.4f}".format(
            elapsed_s, speeds[0], speeds[1], speeds[2], speeds[3]))

    # 目标距离
    if avg_dist >= TARGET_DIST_M:
        break

    time.sleep_ms(10)
    gc.collect()

# ============================================================
#  停止 & 报告
# ============================================================
omni_drive_closed_loop(0, 0, 0)
stop_all()
enc_ticker.start(10)
led.off()

actual_duration_s = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0

print("\n" + "=" * 60)
print("  Layer 1 Report ({:.2f}s)".format(actual_duration_s))
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

if spread_cm < 1.5:
    print("  PASS: Spread {:.1f}cm < 1.5cm threshold.".format(spread_cm))
    print("  -> Proceed to Layer 2 (heading hold).")
elif spread_cm < 3.0:
    print("  MARGINAL: Spread {:.1f}cm (1.5-3.0cm).".format(spread_cm))
    print("  -> Proceed to Layer 2, but note mechanical asymmetry.")
else:
    print("  FAIL: Spread {:.1f}cm >= 3.0cm.".format(spread_cm))
    print("  -> Check: wheel screws, tire wear, motor mount, encoder calibration.")
print("=" * 60)
