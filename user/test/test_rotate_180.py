"""
test_rotate_180.py — 单独测试旋转 180°
【功能】验证梯形速度曲线 + 前馈旋转能否精准到位
【用法】烧录后在空地运行，观察小车旋转
        拨动 SWITCH2 随时终止
"""

import gc, time
from machine import Pin
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker, ENC_SCALE, LED_PIN, SWITCH2_PIN,
)
import imu_motion

led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

DT = 0.01
TIMEOUT_S = 15
PRINT_MS = 100

ROT_MAX_RATE = 65
ROT_MAX_ACCEL = 90
ROT_DEADBAND = 3.0
ROT_FF = 0.006
ROT_FB = 0.004

# 初始化
stop_all()
enc_ticker.stop()
time.sleep_ms(50)
for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)
for _ in range(10):
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

gyro_offset_z = imu_motion.gyro_offset_z

print("\n" + "=" * 60)
print("  Test: Rotate 180°")
print("  max_rate={:.0f}°/s  accel={:.0f}°/s²  ff={:.4f}  fb={:.4f}".format(
    ROT_MAX_RATE, ROT_MAX_ACCEL, ROT_FF, ROT_FB))
print("=" * 60)

# 锁定当前航向
start_yaw = imu_motion.yaw
target_yaw = start_yaw + 180
while target_yaw > 180: target_yaw -= 360
while target_yaw < -180: target_yaw += 360

print("  start={:.1f}°  target={:.1f}°".format(start_yaw, target_yaw))
print("  {:>5s}  {:>7s}  {:>7s}  {:>7s}  {:>7s}  {:>7s}".format(
    "time", "gz", "tgt", "err", "wz", "yaw"))
print("  " + "-" * 55)

gz_f = 0.0
start_ms = time.ticks_ms()
last_print_ms = start_ms

while True:
    now_ms = time.ticks_ms()
    elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

    if elapsed_s > TIMEOUT_S:
        print("  TIMEOUT"); break
    if switch2.value() != state2:
        print("  SW2 stop"); break

    # t=0ms: IMU 第 1 次
    d0 = imu_motion.imu.read()
    imu_motion.update_angle(d0[0], d0[1], d0[2], d0[3], d0[4], d0[5])

    time.sleep_ms(5)

    # t=5ms: IMU 第 2 次 + 控制
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

    err = target_yaw - imu_motion.yaw
    while err > 180: err -= 360
    while err < -180: err += 360

    if abs(err) <= ROT_DEADBAND:
        break

    ideal = (2 * ROT_MAX_ACCEL * abs(err)) ** 0.5
    tgt_rate = min(ideal, ROT_MAX_RATE)
    if err < 0: tgt_rate = -tgt_rate

    gz_dps = (d[5] - gyro_offset_z) / 16.4
    gz_f = gz_dps if gz_f == 0.0 else 0.5 * gz_f + 0.5 * gz_dps

    wz = tgt_rate * ROT_FF + ROT_FB * (tgt_rate - gz_f)
    wz = max(-0.7, min(wz, 0.7))

    rc = get_encoder_counts()
    rs = [rc[i] / ENC_SCALE[i] / DT for i in range(4)]
    omni_drive_closed_loop(0, 0, wz, rs, DT)

    if time.ticks_diff(now_ms, last_print_ms) >= PRINT_MS:
        last_print_ms = now_ms
        print("  {:5.1f}s  {:7.1f}  {:7.1f}  {:7.1f}  {:+.4f}  {:7.1f}".format(
            elapsed_s, gz_f, tgt_rate, err, wz, imu_motion.yaw))

    time.sleep_ms(5)

# 停车
omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
_ = get_encoder_counts()
dlt = imu_motion.yaw - start_yaw
while dlt > 180: dlt -= 360
while dlt < -180: dlt += 360
err_deg = dlt - 180
while err_deg > 180: err_deg -= 360
while err_deg < -180: err_deg += 360

print("  " + "-" * 55)
print("  Result: {:.1f}°  error={:+.1f}°".format(dlt, err_deg))
print("=" * 60)

stop_all()
enc_ticker.start(10)
led.off()
