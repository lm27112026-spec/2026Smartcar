"""
06_enc_scale_calibrate.py - ENC_SCALE 标定测试（修正版）
【方法】
  1) omni_drive(0.3, 0, 0) 跑 2 秒，边跑边累积编码器脉冲
  2) 停车后打印 4 轮编码器脉冲
  3) 你用卷尺量起点到终点的距离
  4) 脚本自动算出每轮的 ENC_SCALE = 脉冲数 / 实测距离
"""

import gc, time
from machine import Pin
from motor import (
    omni_drive, stop_all, get_encoder_counts,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    enc_ticker,
    LED_PIN, SWITCH2_PIN,
)

# ============================================================
#  初始化
# ============================================================
stop_all()
time.sleep_ms(50)

led = Pin(LED_PIN, Pin.OUT, value=True)

# 暂停 motor.py 的自动 ticker，避免偷脉冲
enc_ticker.stop()

# 归零
for _ in range(5):
    get_encoder_counts()
    time.sleep_ms(10)

# ============================================================
#  运行 2 秒（边跑边累积脉冲）
# ============================================================
print("\n" + "=" * 60)
print("  ENC_SCALE Calibration (2s run)")
print("  Mark your start line now!")
print("=" * 60)

total_counts = [0, 0, 0, 0]
start_ms = time.ticks_ms()

omni_drive(0.3, 0, 0)
led.on()

while True:
    elapsed_ms = time.ticks_diff(time.ticks_ms(), start_ms)
    if elapsed_ms >= 2000:
        break
    
    counts = get_encoder_counts()
    for i in range(4):
        total_counts[i] += counts[i]
    
    time.sleep_ms(20)
    gc.collect()

omni_drive(0, 0, 0)
stop_all()
enc_ticker.start(10)
led.off()

# ============================================================
#  打印脉冲
# ============================================================
print("\n  Encoder pulses (2s):")
print("    RF = {}".format(total_counts[0]))
print("    LF = {}".format(total_counts[1]))
print("    LB = {}".format(total_counts[2]))
print("    RB = {}".format(total_counts[3]))
print("")
print("  NOW MEASURE THE DISTANCE from start to end!")
print("  (use tape measure, in meters)")
print("")

# ============================================================
#  输入实测距离并计算
# ============================================================
try:
    dist_str = input("  Enter measured distance (m): ")
    measured_m = float(dist_str)
except:
    print("  Invalid input. Using 0.")
    measured_m = 0.0

if measured_m > 0:
    print("\n" + "=" * 60)
    print("  ENC_SCALE Results (measured = {:.3f}m)".format(measured_m))
    print("=" * 60)
    
    wheel_names = ["RF", "LF", "LB", "RB"]
    raw_counts = [abs(total_counts[i]) for i in range(4)]
    enc_scales = [c / measured_m for c in raw_counts]
    
    for i in range(4):
        print("  {}: counts={:5d}  ENC_SCALE={:.0f}  (was 833/840/853/817)".format(
            wheel_names[i], raw_counts[i], enc_scales[i]))
    
    avg_scale = sum(enc_scales) / 4
    print("-" * 60)
    print("  Average = {:.0f}".format(avg_scale))
    print("")
    print("  New ENC_SCALE:")
    print("  ENC_SCALE = [{}, {}, {}, {}]".format(
        int(enc_scales[0]), int(enc_scales[1]),
        int(enc_scales[2]), int(enc_scales[3])))
    print("=" * 60)
else:
    print("  No distance entered. Cannot calculate.")
