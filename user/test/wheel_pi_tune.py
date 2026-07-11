"""
wheel_pi_tune.py — 轮速 PI 调参助手
【功能】闭环驱动前进，实时打印四轮目标速度 vs 实际速度
【用法】架空车轮 → 观察响应 → 调整 motor.py 中 WHEEL_PI 的 kp/ki → 重跑
【目标】
  - 实际速度快速跟上目标（无大超调）
  - 稳态误差 < 0.01 m/s
  - 无持续震荡
"""
import gc, time
from machine import Pin
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker, ENC_SCALE,
    reset_wheel_pi, LED_PIN, SWITCH2_PIN, MAX_PWM,
    MAX_SPEED_MPS, PWM_PER_MPS, WHEEL_PI,
)

DRIVE_SPEED = 0.3      # 目标速度 (m/s)
TEST_TIME_S = 8        # 测试时长 (秒)
DT = 0.02              # 控制周期

switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

print("=" * 60)
print("  Wheel PI Tune — Speed Step Response")
print("  Target: {:.1f} m/s  |  Duration: {}s".format(DRIVE_SPEED, TEST_TIME_S))
print("  Current Kp={} Ki={}".format(WHEEL_PI[0].kp, WHEEL_PI[0].ki))
print("  ⚠ 架空车轮！")
print("=" * 60)
print("")
print("  {:>5s}  {:>7s} {:>7s} {:>7s} {:>7s}  {:>7s} {:>7s} {:>7s} {:>7s}".format(
    "t(s)", "RF_tgt", "RF_act", "LF_tgt", "LF_act",
    "LB_tgt", "LB_act", "RB_tgt", "RB_act"))
print("  " + "-" * 85)

stop_all()
time.sleep_ms(50)

enc_ticker.stop()
for _ in range(5):
    get_encoder_counts()
    time.sleep_ms(10)

reset_wheel_pi()
start = time.ticks_ms()
last_print = start

while True:
    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, start) / 1000.0
    if elapsed >= TEST_TIME_S or switch2.value() != state2:
        break

    raw = get_encoder_counts()
    speeds = [raw[i] / ENC_SCALE[i] / DT for i in range(4)]

    omni_drive_closed_loop(DRIVE_SPEED, 0, 0, speeds, DT)

    if time.ticks_diff(now, last_print) >= 200:
        last_print = now
        # 目标速度: norms * MAX_SPEED_MPS
        norms = [1, -1, -1, 1]  # forward: RF+ LF- LB- RB+
        targets = [norms[i] * MAX_SPEED_MPS[i] * DRIVE_SPEED for i in range(4)]
        print("  {:5.1f}  {:7.3f} {:7.3f} {:7.3f} {:7.3f}  {:7.3f} {:7.3f} {:7.3f} {:7.3f}".format(
            elapsed,
            targets[0], speeds[0], targets[1], speeds[1],
            targets[2], speeds[2], targets[3], speeds[3]))

    time.sleep_ms(int(DT * 1000))
    gc.collect()

omni_drive_closed_loop(0, 0, 0, [0,0,0,0], DT)
stop_all()
enc_ticker.start(10)

# ── 分析 ──
print("\n" + "=" * 60)
print("  Tuning Guide:")
print("  - 实际速度跟不上目标 → 增大 kp")
print("  - 持续震荡           → 减小 kp")
print("  - 稳态有误差不消除   → 增大 ki")
print("  - 超调过大           → 减小 kp 或加 kd")
print("=" * 60)
