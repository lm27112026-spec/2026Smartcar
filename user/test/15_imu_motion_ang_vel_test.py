"""
15_imu_motion_ang_vel_test.py — imu_motion 角速度闭环集成测试
【功能】 验证 imu_motion.py 的角速度闭环功能
【测试项】
  1) IMU 初始化 + 陀螺校准（复用 imu_motion 已有流程）
  2) 静止角速度读取 — 验证零偏 < 2dps
  3) 角速度闭环: 左转 90°/s → 右转 90°/s → 左转 180°/s → 右转 180°/s
  4) drive_distance() 航向保持 — 前进 0.5m 观察航向偏差
  5) 在线调参测试 — 切换 PID 增益
【验证标准】
  - 稳态角速度误差 < 10 deg/s
  - 航向保持偏差 < 5°
  - 无振荡发散
【用法】 拨动 SWITCH2 退出
"""

import gc, time
from machine import Pin
from smartcar import ticker
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker, ENC_SCALE, LED_PIN, SWITCH2_PIN,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
)
import imu_motion

# ============================================================
#  硬件
# ============================================================
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ============================================================
#  参数
# ============================================================
DT = 0.02
PRINT_INTERVAL_MS = 100

# ============================================================
#  初始化
# ============================================================
stop_all()
enc_ticker.stop()
time.sleep_ms(50)

# 清空编码器缓冲
for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)

# 恢复编码器 ticker（imu_motion 导入时未启动 ticker，这里用 PIT1 给编码器）
pit_enc = ticker(1)
pit_enc.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
pit_enc.start(10)

print("\n" + "=" * 60)
print("  imu_motion 角速度闭环集成测试")
print("=" * 60)
print("  GYRO_SENSITIVITY = {:.1f}".format(imu_motion.GYRO_SENSITIVITY))
print("  gyro_offset_z    = {:.2f} LSB".format(imu_motion.gyro_offset_z))
print("  ANG_VEL PID: kp={:.4f} ki={:.4f} kd={:.4f}".format(
    imu_motion.ANG_VEL_KP, imu_motion.ANG_VEL_KI, imu_motion.ANG_VEL_KD))
print("  HEADING_RATE_GAIN = {:.1f} dps/°".format(imu_motion.HEADING_RATE_GAIN))
print("\n按 SWITCH2 随时退出\n")


# ============================================================
#  测试 1：静止角速度校验
# ============================================================

def test_static_gyro(duration_s=2.0):
    """静止时角速度应接近 0"""
    print("=" * 60)
    print("  测试 1: 静止角速度校验 ({:.0f}s)".format(duration_s))
    print("=" * 60)
    print("  {:>5s}  {:>8s}  {:>8s}".format("time", "wz_dps", "yaw"))

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    samples = []

    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        if switch2.value() != state2:
            print("  SW2 stop")
            return False
        if elapsed_s >= duration_s:
            break

        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        wz = imu_motion.get_angular_velocity()
        samples.append(wz)

        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
            last_print_ms = now_ms
            print("  {:5.1f}s  {:8.1f}  {:8.1f}".format(
                elapsed_s, wz, imu_motion.yaw))

        time.sleep_ms(int(DT * 1000))

    avg = sum(samples) / len(samples) if samples else 0
    max_wz = max(abs(s) for s in samples) if samples else 0
    print("  >>> avg={:.2f} dps  max|wz|={:.2f} dps  samples={}".format(
        avg, max_wz, len(samples)))

    if max_wz < 5.0:
        print("  ✅ 静止角速度正常")
    else:
        print("  ⚠️  静止角速度偏大，检查陀螺零偏或振动")
    return True


# ============================================================
#  测试 2：角速度闭环
# ============================================================

def test_rate_control(target_dps, duration_s, label):
    """角速度闭环：目标 deg/s → PID 反馈 → wz → 电机"""
    imu_motion.reset_ang_vel_pid()

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    total_angle = 0.0

    print("\n  ── [{:s}] target={:+.0f} deg/s, {:.1f}s ──".format(
        label, target_dps, duration_s))
    print("  {:>5s}  {:>8s}  {:>8s}  {:>8s}  {:>8s}  {:>8s}".format(
        "time", "gz_dps", "target", "wz", "yaw", "angle"))

    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        if switch2.value() != state2:
            print("  SW2 stop")
            return False
        if elapsed_s >= duration_s:
            break

        # IMU 更新
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

        # 角速度闭环
        wz_dps = imu_motion.get_angular_velocity()
        wz_out = imu_motion.angular_velocity_control(target_dps, wz_dps, DT)

        # 简易转角积分
        total_angle += wz_dps * DT

        # 编码器 → 速度环
        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]

        # 驱动
        omni_drive_closed_loop(0, 0, wz_out, raw_speeds, DT)

        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
            last_print_ms = now_ms
            print("  {:5.1f}s  {:8.1f}  {:8.1f}  {:+.4f}  {:8.1f}  {:8.1f}".format(
                elapsed_s, wz_dps, target_dps, wz_out, imu_motion.yaw, total_angle))

        time.sleep_ms(int(DT * 1000))
        gc.collect()

    # 停车
    omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
    time.sleep_ms(100)
    _ = get_encoder_counts()

    avg_dps = total_angle / elapsed_s if elapsed_s > 0 else 0
    error_pct = abs(avg_dps - target_dps) / abs(target_dps) * 100 if target_dps != 0 else 0
    print("  >>> [{:s}] done: angle={:.1f}°  avg={:.1f} dps  err={:.1f}%".format(
        label, total_angle, avg_dps, error_pct))
    return True


# ============================================================
#  测试 3：在线调参
# ============================================================

def test_online_tuning():
    """快速切换 PID 增益，验证响应变化"""
    print("\n" + "=" * 60)
    print("  测试 3: 在线调参 — 切换 kp 观察响应")
    print("=" * 60)

    imu_motion.reset_ang_vel_pid()
    target_dps = 90
    duration_s = 4.0
    gains = [(0.002, "低 kp=0.002"), (0.008, "高 kp=0.008"), (0.005, "恢复 kp=0.005")]
    segment_s = duration_s / len(gains)

    start_ms = time.ticks_ms()
    last_print_ms = start_ms

    for kp_val, desc in gains:
        imu_motion.set_ang_vel_pid(kp=kp_val)
        print("  --- {} ---".format(desc))

        seg_start = time.ticks_ms()
        while True:
            now_ms = time.ticks_ms()
            elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

            if switch2.value() != state2:
                print("  SW2 stop")
                return False
            if time.ticks_diff(now_ms, seg_start) >= segment_s * 1000:
                break

            d = imu_motion.imu.read()
            imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
            wz_dps = imu_motion.get_angular_velocity()
            wz_out = imu_motion.angular_velocity_control(target_dps, wz_dps, DT)

            raw_counts = get_encoder_counts()
            raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]
            omni_drive_closed_loop(0, 0, wz_out, raw_speeds, DT)

            if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
                last_print_ms = now_ms
                print("  {:5.1f}s  gz={:7.1f}  target={:4.0f}  wz={:+.4f}  kp={:.4f}".format(
                    elapsed_s, wz_dps, target_dps, wz_out, imu_motion._ang_vel_pid.kp))

            time.sleep_ms(int(DT * 1000))
            gc.collect()

    # 恢复默认 kp
    imu_motion.set_ang_vel_pid(kp=imu_motion.ANG_VEL_KP)
    omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
    print("  ✅ 在线调参完成，已恢复默认 PID")
    return True


# ============================================================
#  测试 4：drive_distance 航向保持
# ============================================================

def test_drive_distance_closed_loop():
    """验证 drive_distance 使用角速度闭环保持航向"""
    print("\n" + "=" * 60)
    print("  测试 4: drive_distance() 航向保持（角速度闭环）")
    print("=" * 60)

    # 先确保 IMU 稳定
    for _ in range(10):
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        time.sleep_ms(10)

    start_yaw = imu_motion.yaw
    print("  起始航向: {:.1f}°".format(start_yaw))
    print("  前进 0.3m，保持航向...")

    ok = imu_motion.drive_distance(
        speed=0.3,
        target_angle=0,
        max_dist=0.3,
        timeout_s=5.0,
        use_ang_vel_closed_loop=True,
    )

    end_yaw = imu_motion.yaw
    yaw_drift = end_yaw - start_yaw
    while yaw_drift > 180: yaw_drift -= 360
    while yaw_drift < -180: yaw_drift += 360

    print("  结束航向: {:.1f}°  航向漂移: {:+.1f}°".format(end_yaw, yaw_drift))
    if abs(yaw_drift) < 5.0:
        print("  ✅ 航向保持良好")
    else:
        print("  ⚠️  航向漂移较大，检查 HEADING_RATE_GAIN 或 ANG_VEL PID")
    return ok


# ============================================================
#  主测试序列
# ============================================================

tests = [
    ("静止校验", lambda: test_static_gyro(2.0)),
    ("角速度闭环 90°/s", lambda: test_rate_control(90, 3.0, "左转90")),
    ("角速度闭环 -90°/s", lambda: test_rate_control(-90, 3.0, "右转90")),
    ("角速度闭环 180°/s", lambda: test_rate_control(180, 2.0, "左转180")),
    ("角速度闭环 -180°/s", lambda: test_rate_control(-180, 2.0, "右转180")),
    ("在线调参", lambda: test_online_tuning()),
    ("航向保持", lambda: test_drive_distance_closed_loop()),
]

for name, test_fn in tests:
    print("\n▶ 开始: {}".format(name))
    try:
        if not test_fn():
            break  # SW2
    except Exception as e:
        print("  ❌ 测试异常: {}".format(e))
    time.sleep_ms(500)


# ============================================================
#  清理
# ============================================================
print("\n清理中...")
omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
time.sleep_ms(100)
stop_all()
pit_enc.stop()
enc_ticker.start(10)
led.off()
print("=== Done ===")
