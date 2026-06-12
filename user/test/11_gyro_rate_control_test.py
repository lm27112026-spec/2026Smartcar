"""
11_gyro_rate_control_test.py — 角速度闭环测试
【功能】 使用陀螺仪 gz 做旋转速率 PI 控制
【原理】 目标角速度 (deg/s) → PI 控制器 → wz → omni_drive_closed_loop
         陀螺仪反馈实际角速度，闭环维持恒定旋转速率
【验证】 1) 设定 90°/s 左转 2 秒 → 应转约 180°
         2) 设定 -90°/s 右转 2 秒 → 应回转约 180°
【用法】 拨动 SWITCH2 退出
"""

import gc, time
from machine import Pin
from pid import PID
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker,
    ENC_SCALE, LED_PIN, SWITCH2_PIN,
)
import imu_motion

# ── 硬件 ──
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ── 参数 ──
DT = 0.02
TIMEOUT_S = 5
PRINT_INTERVAL_MS = 100

# 角速度 PI
RATE_KP = 0.012
RATE_KI = 0.010
RATE_OUTPUT_LIMIT = 0.7
RATE_INTEGRAL_LIMIT = 80       # 积分上限匹配 °/s 量级

# 陀螺仪 gz 低通滤波（抑制电机振动噪声）
GZ_FILTER_ALPHA = 0.5       # 0.5 比 0.3 更快响应

# GYRO_SENSITIVITY: 从 imu_motion.py 同步
GYRO_SENSITIVITY = 16.4     # LSB/(°/s)，手册 ±2000dps 量程

# ── 初始化 ──
stop_all()
enc_ticker.stop()
time.sleep_ms(50)

for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)

# IMU 初始化（获取校准后的 gyro_offset_z）
for _ in range(10):
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

gyro_offset_z = imu_motion.gyro_offset_z

print("\n" + "=" * 60)
print("  Gyro Rate PI Control Test")
print("  gyro_offset_z = {:.1f}".format(gyro_offset_z))
print("  Rate PI: Kp={:.4f} Ki={:.4f}".format(RATE_KP, RATE_KI))
print("=" * 60)


def test_rate(target_dps, duration_s, label):
    """
    target_dps: 目标角速度 deg/s（正=左转/CCW）
    duration_s: 持续时间（秒）
    """
    pid_rate = PID(kp=RATE_KP, ki=RATE_KI, kd=0.0,
                   integral_limit=RATE_INTEGRAL_LIMIT,
                   output_limit=RATE_OUTPUT_LIMIT)

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    gz_filtered = 0.0  # 滤波器状态

    # yaw 初始值（用于对比总转角）
    yaw_start = imu_motion.yaw

    print("\n  ── Target: {:+.0f} deg/s for {:.1f}s ──".format(target_dps, duration_s))
    print("  {:>5s}  {:>8s}  {:>8s}  {:>8s}  {:>7s}".format(
        "time", "gz_dps", "target", "wz", "yaw"))

    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        if elapsed_s > duration_s + TIMEOUT_S:
            print("  TIMEOUT")
            break

        if elapsed_s >= duration_s:
            break  # 达到设定时间

        if switch2.value() != state2:
            print("  SW2 stop")
            return False

        # 读 IMU 原始陀螺仪数据
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

        # gz → deg/s（低通滤波抑制振动噪声）
        gz_raw = d[5]
        gz_dps = (gz_raw - gyro_offset_z) / GYRO_SENSITIVITY
        if gz_filtered == 0.0:
            gz_filtered = gz_dps
        else:
            gz_filtered = GZ_FILTER_ALPHA * gz_filtered + (1 - GZ_FILTER_ALPHA) * gz_dps

        # 角速度 PI（使用滤波后的 gz）
        wz = pid_rate.compute(setpoint=target_dps, measurement=gz_filtered, dt=DT)

        # 读编码器（旋转时车轮也在转，速度内环需要反馈）
        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]

        # 闭环驱动（纯旋转，无平移）
        omni_drive_closed_loop(0, 0, wz, raw_speeds, DT)

        # 打印
        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
            last_print_ms = now_ms
            print("  {:5.1f}s  {:8.1f}  {:8.1f}  {:+.4f}  {:7.1f}".format(
                elapsed_s, gz_filtered, target_dps, wz, imu_motion.yaw))

        time.sleep_ms(int(DT * 1000))
        gc.collect()

    # 结果
    yaw_end = imu_motion.yaw
    yaw_delta = yaw_end - yaw_start
    while yaw_delta > 180: yaw_delta -= 360
    while yaw_delta < -180: yaw_delta += 360

    expected = target_dps * min(elapsed_s, duration_s)
    print("  ── Result: yaw moved {:.1f}°  expected {:.1f}°  error {:.1f}° ──".format(
        yaw_delta, expected, yaw_delta - expected))
    return True


# ── 测试序列 ──
# 1) 左转 90°/s，2 秒 → 预期转约 180°
test_rate(90, 2.0, "CCW 90dps × 2s")

# 2) 暂停
time.sleep_ms(500)

# 3) 右转 90°/s，2 秒 → 预期回转约 180°
test_rate(-90, 2.0, "CW 90dps × 2s")

# ── 清理 ──
omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
stop_all()
enc_ticker.start(10)
led.off()
print("\n=== Done ===")
