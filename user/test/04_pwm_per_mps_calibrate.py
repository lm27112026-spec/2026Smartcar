"""
04_pwm_per_mps_calibrate.py - PWM_PER_MPS 标定测试
【功能】
  以固定 PWM 驱动 5 秒，测量实际行驶距离，计算每轮的 PWM_PER_MPS。
【方法】
  1) omni_drive(0.3, 0, 0) 开环驱动
  2) 5 秒后停止，记录各轮编码器脉冲
  3) 计算: 速度 = 距离 / 时间, PWM_PER_MPS = 实际PWM / 速度
【使用】
  运行此脚本，5 秒内不要触碰小车。结束后查看输出的 PWM_PER_MPS 值。
"""

import gc, time
from machine import Pin
from motor import (
    omni_drive, stop_all, get_encoder_counts,
    enc_ticker,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    ENC_SCALE, LED_PIN, SWITCH2_PIN, MAX_PWM, 
)

# ============================================================
#  常量
# ============================================================
DRIVE_SPEED = 0.3          # 归一化速度（0~1）
TEST_DURATION_S = 5.0      # 标定时间（秒）
ACTUAL_PWM = int(DRIVE_SPEED * MAX_PWM)  # 实际 PWM = 15000 初始化
# ============================================================
stop_all()
time.sleep_ms(50)

led = Pin(LED_PIN, Pin.OUT, value=True)

enc_ticker.stop()

# 归零编码器
print("Zeroing encoders...")
for _ in range(5):
    get_encoder_counts()
    time.sleep_ms(10)

# ============================================================
#  开始标定
# ============================================================
print("\n" + "=" * 60)
print("  PWM_PER_MPS Calibration Test")
print("  Drive speed: {:.1f} (PWM = {:d})".format(DRIVE_SPEED, ACTUAL_PWM))
print("  Duration: {:.0f} seconds".format(TEST_DURATION_S))
print("  DO NOT touch the robot during test!")
print("=" * 60)

total_counts = [0, 0, 0, 0]
start_ms = time.ticks_ms()

# 开环驱动
omni_drive(DRIVE_SPEED, 0, 0)
led.on()

# 等待指定时间
while True:
    elapsed_s = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
    if elapsed_s >= TEST_DURATION_S:
        break
    
    counts = get_encoder_counts()
    for i in range(4):
        total_counts[i] += counts[i]
    
    time.sleep_ms(20)
    gc.collect()

# 停止
omni_drive(0, 0, 0)
stop_all()
enc_ticker.start(10)
led.off()

actual_duration_s = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0

# ============================================================
#  计算结果
# ============================================================
print("\n" + "=" * 60)
print("  Calibration Results ({:.2f}s)".format(actual_duration_s))
print("=" * 60)

wheel_names = ["RF", "LF", "LB", "RB"]
distances = []
speeds = []
pwm_per_mps = []

for i in range(4):
    dist_m = abs(total_counts[i]) / ENC_SCALE[i]
    speed_mps = dist_m / actual_duration_s
    if speed_mps > 0:
        ppms = ACTUAL_PWM / speed_mps
    else:
        ppms = 0
    distances.append(dist_m)
    speeds.append(speed_mps)
    pwm_per_mps.append(ppms)
    
    print("  {}: counts={:5d}  dist={:.4f}m  speed={:.4f}m/s  PWM_PER_MPS={:.0f}".format(
        wheel_names[i], total_counts[i], dist_m, speed_mps, ppms))

avg_ppms = sum(pwm_per_mps) / 4
print("-" * 60)
print("  Average PWM_PER_MPS = {:.0f}".format(avg_ppms))
print("")
print("  Copy this to motor.py:")
print("  PWM_PER_MPS = [{}, {}, {}, {}]".format(
    int(pwm_per_mps[0]), int(pwm_per_mps[1]),
    int(pwm_per_mps[2]), int(pwm_per_mps[3])))
print("=" * 60)
