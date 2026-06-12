"""
12_angular_accel_test.py — 角加速度闭环测试（模型法）
【功能】 目标角加速度 → 积分得目标速率 → 速率 PI 跟踪
【原理】 不测 α（微分噪声太大），改用速率跟踪实现加速度控制
         target_rate = ∫ target_α · dt
         gz 实测速率 → 速率 PI → wz
【验证】 +90°/s² 加速 → gz 线性爬升到 ~90°/s
         −90°/s² 减速 → gz 线性降到 ~0°/s
【用法】 拨动 SWITCH2 退出
"""

import gc, time
from machine import Pin
from pid import PID
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker, ENC_SCALE, LED_PIN, SWITCH2_PIN,
)
import imu_motion

led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

DT = 0.02
TIMEOUT_S = 5
PRINT_INTERVAL_MS = 100

RATE_FF_GAIN = 0.006     # wz / (°/s)，来自实测：0.4 wz ≈ 65°/s → 0.0062
RATE_FB_KP = 0.004       # 小 P 修正残差（原 0.002→0.004，减小跟踪滞后）
GZ_FILTER_ALPHA = 0.5

GYRO_SENSITIVITY = 16.4
MAX_RATE = 65             # deg/s 物理上限
MAX_ACCEL = 90            # deg/s²
ANGLE_DEADBAND = 3        # <3° 到位

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
print("  Trapezoid Rotation Test (FF + P feedback)")
print("  gyro_offset_z = {:.1f}".format(gyro_offset_z))
print("  MAX_RATE={:.0f}°/s  FF_gain={:.4f}  FB_kp={:.4f}".format(
    MAX_RATE, RATE_FF_GAIN, RATE_FB_KP))
print("=" * 60)


def rotate_by_angle(target_delta, label):
    """
    target_delta: 旋转角度 deg（正=左转）
    梯形速度曲线 + 前馈 wz + P 修正
    """
    for _ in range(5):
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        time.sleep_ms(10)

    start_yaw = imu_motion.yaw
    target_yaw = start_yaw + target_delta
    while target_yaw > 180: target_yaw -= 360
    while target_yaw < -180: target_yaw += 360

    gz_filtered = 0.0
    start_ms = time.ticks_ms()
    last_print_ms = start_ms

    print("\n  ── [{:s}] rotate {:+.0f}°  start={:.1f}°  target={:.1f}° ──".format(
        label, target_delta, start_yaw, target_yaw))
    print("  {:>5s}  {:>7s}  {:>7s}  {:>7s}  {:>7s}  {:>7s}".format(
        "time", "gz_f", "tgt_rate", "err", "wz", "yaw"))

    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        if elapsed_s > TIMEOUT_S:
            print("  TIMEOUT")
            return False

        if switch2.value() != state2:
            print("  SW2 stop")
            return False

        # 角度误差
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        angle_err = target_yaw - imu_motion.yaw
        while angle_err > 180: angle_err -= 360
        while angle_err < -180: angle_err += 360

        if abs(angle_err) <= ANGLE_DEADBAND:
            break

        # 梯形速度曲线
        ideal_rate = (2 * MAX_ACCEL * abs(angle_err)) ** 0.5
        target_rate = min(ideal_rate, MAX_RATE)
        if angle_err < 0:
            target_rate = -target_rate

        # 陀螺仪速率（滤波）
        gz_raw = d[5]
        gz_dps = (gz_raw - gyro_offset_z) / GYRO_SENSITIVITY
        if gz_filtered == 0.0:
            gz_filtered = gz_dps
        else:
            gz_filtered = GZ_FILTER_ALPHA * gz_filtered + (1 - GZ_FILTER_ALPHA) * gz_dps

        # 前馈 + P 反馈（不用 I 防振荡）
        wz_ff = target_rate * RATE_FF_GAIN
        wz_fb = RATE_FB_KP * (target_rate - gz_filtered)
        wz = wz_ff + wz_fb
        wz = max(-0.7, min(wz, 0.7))

        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]
        omni_drive_closed_loop(0, 0, wz, raw_speeds, DT)

        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
            last_print_ms = now_ms
            print("  {:5.1f}s  {:7.1f}  {:7.1f}  {:7.1f}  {:+.4f}  {:7.1f}".format(
                elapsed_s, gz_filtered, target_rate, angle_err, wz, imu_motion.yaw))

        time.sleep_ms(int(DT * 1000))
        gc.collect()

    omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
    _ = get_encoder_counts()

    actual_delta = imu_motion.yaw - start_yaw
    while actual_delta > 180: actual_delta -= 360
    while actual_delta < -180: actual_delta += 360
    print("  >>> [{:s}] done: {:.1f}°  error={:+.1f}° <<<".format(
        label, actual_delta, actual_delta - target_delta))
    return True


# ── 测试：右转 90° → 转回来 ──
rotate_by_angle(-90, "RIGHT 90°")
time.sleep_ms(500)
rotate_by_angle(90, "BACK 90°")

omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
stop_all()
enc_ticker.start(10)
led.off()
print("\n=== Done ===")

omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
stop_all()
enc_ticker.start(10)
led.off()
print("\n=== Done ===")
