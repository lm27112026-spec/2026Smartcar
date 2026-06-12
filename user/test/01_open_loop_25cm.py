"""
01_open_loop_25cm.py - 开环直线运动诊断测试（25cm 短距版）

【功能】
  以相同的开环 PWM 同时驱动 4 个全向轮，不使用任何运动学/反馈，
  当任一车轮行驶达 25cm 时停止，输出各轮实际行驶距离和 IMU 偏航角漂移。
  用于诊断机械/电机不对称性。

【评判标准（25cm 短距）】
  - PASS:    4 轮最大距离差 < 1.0cm（4%）且偏航漂移 < 2.0°
  - MARGINAL: 距离差 1.0-3.0cm — 机械有偏差但可用
  - FAIL:    距离差 >= 3.0cm  → 机械对齐/电机/车轮问题
  - FAIL:    偏航漂移 >= 2.0° → 电机响应曲线或重心偏移（车轮对称也救不了）

【硬件要求】
  - 平坦地面（推荐桌面或地砖）
  - 标记起点线，用卷尺确认 25cm 终点
  - 车体初始朝向与行驶方向一致
  - 注意：场地极限 60cm，本测试只跑 25cm 留充足余量

【退出方式】
  - 按 SWITCH2 立即停止并输出报告
  - 10s 超时自动停止（25cm 短距足够）
"""

import gc, time
from machine import Pin
from motor import (
    set_motor, stop_all, omni_drive, omni_kinematics,
    get_encoder_counts, enc_ticker,
    LED_PIN, SWITCH2_PIN,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB,
    ENC_SCALE, MAX_SPEED_MPS, MAX_PWM,
)
import imu_motion

SWITCH2_PIN = 'D9'
LED_PIN = 'C4'

# ============================================================
#  Startup safety
# ============================================================
stop_all()
time.sleep_ms(50)

# ============================================================
#  Initialize LED + SWITCH2
# ============================================================
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ============================================================
#  Ticker (encoder capture timer)
# ============================================================
enc_ticker.stop()

# ============================================================
#  Zero encoders (drain residual pulses)
# ============================================================
for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)

# ============================================================
#  Initialize IMU yaw baseline
# ============================================================
d = imu_motion.imu.read()
imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
yaw_start = imu_motion.yaw

# ============================================================
#  Constants
# ============================================================
TEST_PWM      = 10000   # 60% MAX_PWM, mid-range
TARGET_DIST_M = 0.25    # 25cm — short-distance test
CONTROL_INTERVAL_S = 0.02
TIMEOUT_S     = 10      # 25cm 短距 10s 足够

# PASS/FAIL 阈值（25cm 短距按比例缩放，机械绝对偏差 1cm 已明显）
SPREAD_PASS_M  = 0.010  # < 1cm  → PASS
SPREAD_FAIL_M  = 0.030  # >= 3cm → FAIL
YAW_PASS_DEG   = 2.0    # < 2°   → PASS

# ============================================================
#  Header
# ============================================================
print("\n" + "=" * 60)
print("  Open-Loop 25cm Straight-Line Test")
print("  All wheels at {:d} PWM ({}% max)".format(TEST_PWM, TEST_PWM * 100 // MAX_PWM))
print("  Target distance: {:.0f} cm".format(TARGET_DIST_M * 100))
print("  SWITCH2 to exit | {}s timeout".format(TIMEOUT_S))
print("=" * 60)

# ============================================================
#  Drive — open-loop PWM with correct kinematics direction
# ============================================================
norms = omni_kinematics(1, 0, 0)  # [1, -1, -1, 1]
for i, m in enumerate((MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB)):
    set_motor(m, int(norms[i] * TEST_PWM))

# ============================================================
#  Main control loop
# ============================================================
total_counts = [0, 0, 0, 0]
start_ms = time.ticks_ms()
tick_count = 0
elapsed = 0.0

while True:
    # -- Check timeout --
    elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
    if elapsed > TIMEOUT_S:
        print("Timeout ({:.0f}s) reached — stopping.".format(TIMEOUT_S))
        break

    # -- Check SWITCH2 --
    if switch2.value() != state2:
        print("SW2 stop — user exit.")
        break

    # -- Read encoder deltas --
    counts = get_encoder_counts()

    for i in range(4):
        total_counts[i] += counts[i]

    # -- Per-wheel distance (abs to handle sign conventions) --
    wheel_dist = [
        abs(total_counts[0]) / abs(ENC_SCALE[0]),
        abs(total_counts[1]) / abs(ENC_SCALE[1]),
        abs(total_counts[2]) / abs(ENC_SCALE[2]),
        abs(total_counts[3]) / abs(ENC_SCALE[3]),
    ]
    max_dist = max(wheel_dist)

    # -- Read IMU and update angle --
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

    # -- Progress print every 5 ticks (~100ms) --
    if tick_count % 5 == 0:
        print("t={:.1f}s  RF={:.3f}m  LF={:.3f}m  LB={:.3f}m  RB={:.3f}m  yaw={:+.1f}deg".format(
            elapsed, wheel_dist[0], wheel_dist[1], wheel_dist[2], wheel_dist[3],
            imu_motion.yaw - yaw_start))

    # -- Check target --
    if max_dist >= TARGET_DIST_M:
        print("Target reached ({:.0f}cm) — stopping.".format(TARGET_DIST_M * 100))
        break

    tick_count += 1
    time.sleep_ms(20)
    gc.collect()

# ============================================================
#  Cleanup
# ============================================================
for m in (MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB):
    set_motor(m, 0)
stop_all()
enc_ticker.start(10)
led.off()

# ============================================================
#  Final report
# ============================================================
spread    = max(wheel_dist) - min(wheel_dist)
mean_d    = sum(wheel_dist) / 4
yaw_drift = imu_motion.yaw - yaw_start

print("\n" + "=" * 60)
print("  Open-Loop 25cm Test Report")
print("=" * 60)
print("  Per-wheel distance traveled:")
print("    RF = {:.3f} m  ({:.1f} cm)".format(wheel_dist[0], wheel_dist[0] * 100))
print("    LF = {:.3f} m  ({:.1f} cm)".format(wheel_dist[1], wheel_dist[1] * 100))
print("    LB = {:.3f} m  ({:.1f} cm)".format(wheel_dist[2], wheel_dist[2] * 100))
print("    RB = {:.3f} m  ({:.1f} cm)".format(wheel_dist[3], wheel_dist[3] * 100))
print("  Mean  = {:.3f} m  ({:.1f} cm)".format(mean_d, mean_d * 100))
print("  Max-Min spread = {:.3f} m  ({:.1f} cm, {:.1f}%)".format(
    spread, spread * 100, spread / mean_d * 100 if mean_d > 0 else 0))
print("  Yaw drift over {:.1f}s = {:+.1f}deg".format(elapsed, yaw_drift))
print("-" * 60)

if spread < SPREAD_PASS_M and abs(yaw_drift) < YAW_PASS_DEG:
    print("  PASS: 4 wheels symmetric (spread {:.1f}cm < 1cm), mechanical alignment OK.".format(
        spread * 100))
    print("  -> Proceed to 02_encoder_static test.")
elif spread < SPREAD_FAIL_M:
    print("  MARGINAL: spread {:.1f}cm in 1-3cm range. Mechanical asymmetry present.".format(
        spread * 100))
    print("  -> Check: wheel screws, tire pressure, motor mount symmetry.")
    print("  -> Can still proceed but expect visible drift in closed-loop.")
else:
    print("  FAIL: Wheel-to-wheel spread {:.1f}cm >= 3cm threshold.".format(spread * 100))
    print("  -> Mechanical alignment / motor mismatch / wheel diameter difference.")
    print("  -> Check: wheel screws, tire pressure, motor mount, encoder AB wiring.")

if abs(yaw_drift) >= YAW_PASS_DEG and spread < SPREAD_PASS_M:
    print("  NOTE: Wheels are symmetric but yaw drift {:.1f}deg in 25cm.".format(yaw_drift))
    print("  -> Not a wheel issue. Likely: motor response curve mismatch,")
    print("    or center-of-mass offset, or IMU mounting position.")
print("=" * 60)
