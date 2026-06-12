"""
03_imu_static_1min.py - IMU 陀螺仪零偏漂移评估（60 秒静止测试）
【功能】
  在机器人完全静止（平坦台面，禁止触摸）条件下，让 IMU 温度稳定后，
  对 yaw 进行 60 秒积分，量化陀螺仪零偏漂移。
【判据】
  PASS:     < 0.5 °/min  → 适合航向保持闭环
  MARGINAL: < 2.0 °/min  → 短时间可用
  FAIL:     >= 2.0 °/min → 需要更长的校准窗口或排查振动
【硬件状态】
  机器人平放于桌面，轮子悬空或静止，60 秒内切勿触碰。
  远离风扇、马达等振动源。
"""

import gc, time
from machine import Pin
import imu_motion

# ============================================================
#  引脚 & 外设初始化
# ============================================================

SWITCH2_PIN = 'D9'
LED_PIN = 'C4'

led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ============================================================
#  温度稳定等待（10 秒）
# ============================================================

print("\n" + "=" * 60)
print("  IMU Static Yaw Drift Test (60s)")
print("=" * 60)
print("")
print("  机器人放在平坦表面，不要触碰！")
print("  等待 IMU 温度稳定 10 秒...")
print("  拨动 SWITCH2 可提前跳过等待")

state2 = switch2.value()
for i in range(10, 0, -1):
    print("  {} 秒 ...".format(i))
    time.sleep_ms(1000)
    if switch2.value() != state2:
        print("  SW2 拨动 — 跳过等待")
        break

print("  温度稳定完成。")
print("")

# ============================================================
#  IMU 校准信息 & 状态重置
# ============================================================

# imu_motion 模块导入时已完成 200 样本（0.4s）零偏校准
print("gyro offset: gx={:.3f}  gy={:.3f}  gz={:.3f}".format(
    imu_motion.gyro_offset_x, imu_motion.gyro_offset_y, imu_motion.gyro_offset_z))

# 清零姿态 / 滤波器状态，从头开始积分
imu_motion.yaw = 0.0
imu_motion.last_time = 0
imu_motion.gz_filtered = 0.0

# update_angle 首次调用只初始化 last_time，第二次开始积分
d = imu_motion.imu.read()
imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
d = imu_motion.imu.read()
imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

# ============================================================
#  常量
# ============================================================

TEST_DURATION_S = 60
SAMPLE_INTERVAL_MS = 20        # 50 Hz，匹配主控制周期
REPORT_EVERY_S = 10

SAMPLES_TOTAL = int(TEST_DURATION_S * 1000 / SAMPLE_INTERVAL_MS)     # 3000
SAMPLES_PER_REPORT = int(REPORT_EVERY_S * 1000 / SAMPLE_INTERVAL_MS)  # 500

# ============================================================
#  主循环：静止积分 60 秒
# ============================================================

print("")
print("  Yaw drift checkpoints ({}s intervals):".format(REPORT_EVERY_S))

start_ms = time.ticks_ms()
iteration = 0
aborted = False

while iteration < SAMPLES_TOTAL:
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    iteration += 1

    # 每 REPORT_EVERY_S 秒报告一次漂移
    if iteration % SAMPLES_PER_REPORT == 0:
        t = iteration * SAMPLE_INTERVAL_MS / 1000.0
        print("  [t={:.0f}s] yaw drift = {:+.2f}°  (rate: {:.3f}°/min)".format(
            t, imu_motion.yaw, imu_motion.yaw * 60.0 / t))

    # 检测 SWITCH2 提前终止
    if switch2.value() != state2:
        aborted = True
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        print("  SW2 toggled — partial result: {:.1f}s elapsed".format(elapsed))
        break

    time.sleep_ms(SAMPLE_INTERVAL_MS)
    gc.collect()

# ============================================================
#  清理
# ============================================================

led.off()

# ============================================================
#  最终报告
# ============================================================

drift_60s = imu_motion.yaw
elapsed_actual = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
rate_per_min = drift_60s * 60.0 / elapsed_actual if elapsed_actual > 0 else 0

print("\n" + "=" * 60)
print("  IMU Static Yaw Drift Test Report")
print("=" * 60)
print("  Actual test duration: {:.1f}s".format(elapsed_actual))
print("  Yaw drift after {:.0f}s = {:+.2f}°".format(elapsed_actual, drift_60s))
print("  Drift rate = {:.3f}°/min".format(rate_per_min))
print("-" * 60)
if abs(rate_per_min) < 0.5:
    print("  PASS: Drift rate {:.2f}°/min < 0.5°/min threshold.".format(rate_per_min))
    print("  -> IMU is suitable for heading-hold closed-loop control.")
elif abs(rate_per_min) < 2.0:
    print("  MARGINAL: Drift rate {:.2f}°/min (acceptable for short runs <30s).".format(rate_per_min))
    print("  -> Increase gyro calibration window to 5+ seconds with 1+ second warmup.")
    print("  -> Or accept drift and rely on encoder-based heading instead.")
else:
    print("  FAIL: Drift rate {:.2f}°/min > 2°/min.".format(rate_per_min))
    print("  -> IMU has poor zero-bias stability, OR pickup of mechanical vibration,")
    print("     OR current calibration window (0.4s) is far too short.")
    print("  -> Recalibrate with 5-10s window after 10s warmup, re-test.")
    print("  -> If still bad, IMU may be damaged or have poor temperature compensation.")
print("=" * 60)
print("")
