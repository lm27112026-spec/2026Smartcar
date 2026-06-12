"""
uart_move.py — 闭环路线移动（IMU 航向保持 + 编码器位置 PID）
【路线】 右移 30cm → 前进 60cm → 旋转 180° → 右移 30cm
【控制】 距离 PID：精确停在目标距离
         航向 PID：直线段锁航向，旋转段锁目标角度
         所有移动均使用闭环速度 + IMU 反馈
【安全】 SWITCH2 随时终止
"""

import gc, time
from machine import Pin
from pid import PID
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker,
    ENC_SCALE, LED_PIN, SWITCH2_PIN, MAX_SPEED_MPS,
)
import imu_motion

# ── 硬件 ──
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ── 全局参数 ──
DT = 0.02                    # 控制周期 20ms
TIMEOUT_S = 30               # 每段最长等待
PRINT_INTERVAL_MS = 200

# 直线段：距离 PID
DIST_KP = 1.0
DIST_KI = 0.5
DIST_OUTPUT_LIMIT = 0.30     # 0.30 × 0.43 = 0.13 m/s
DIST_MIN_SPEED = 0.08        # 防摩擦死区最低速度

# 直线段：航向保持 PID
HDG_KP = 0.15
HDG_KI = 0.005
HDG_DEADBAND = 0.3           # <0.3° 不纠偏
HDG_WZ_LIMIT = 0.5           # 最大旋转速度
HDG_INTEGRAL_LIMIT = 3.0

# 旋转段：梯形速度曲线 + 前馈 + P 反馈
ROT_MAX_RATE = 65            # deg/s 最大旋转速率
ROT_MAX_ACCEL = 90           # deg/s² 最大角加速度
ROT_DEADBAND = 3.0           # <3° 视为到位
ROT_FF_GAIN = 0.006          # wz / (°/s)，实测 0.4wz≈65°/s
ROT_FB_KP = 0.004            # P 修正残差
GZ_FILTER_ALPHA = 0.5


# ============================================================
#  一、初始化
# ============================================================
stop_all()
enc_ticker.stop()
time.sleep_ms(50)

for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)

# IMU 初始化
for _ in range(10):
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

_initial_heading = imu_motion.yaw


# ============================================================
#  二、直线移动（闭环距离 PID + 航向保持）
# ============================================================
def move_straight(vx_dir, vy_dir, target_m, label):
    """
    vx_dir: vx 方向符号（+1/-1/0）
    vy_dir: vy 方向符号（+1/-1/0）
    target_m: 目标移动距离（米）
    返回 True=成功，False=中止
    """
    # 清零 & 锁航向
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    for _ in range(5):
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        time.sleep_ms(10)

    target_heading = imu_motion.yaw

    pid_dist = PID(kp=DIST_KP, ki=DIST_KI, kd=0.0,
                   integral_limit=0.5, output_limit=DIST_OUTPUT_LIMIT)

    heading_integral = 0.0
    prev_heading_deviation = 0.0
    total_counts = [0, 0, 0, 0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms

    print("\n  ── [{:s}] ──".format(label))
    print("  {:>5s}  {:>6s}  {:>7s} {:>7s} {:>6s}".format(
        "time", "dist", "spd", "yaw", "wz"))

    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        if elapsed_s > TIMEOUT_S:
            print("  TIMEOUT!")
            return False

        if switch2.value() != state2:
            print("  SW2 stop")
            return False

        # 编码器：一次读取，同时用于速度和距离
        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]
        for i in range(4):
            total_counts[i] += raw_counts[i]

        wheel_dist = [abs(total_counts[i]) / ENC_SCALE[i] for i in range(4)]
        current_dist = sum(wheel_dist) / 4

        # IMU 航向
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        heading_deviation = target_heading - imu_motion.yaw
        while heading_deviation > 180: heading_deviation -= 360
        while heading_deviation < -180: heading_deviation += 360

        if abs(heading_deviation - prev_heading_deviation) > 180:
            heading_integral = 0.0
        prev_heading_deviation = heading_deviation

        # 航向 PID
        if abs(heading_deviation) > HDG_DEADBAND:
            heading_integral += heading_deviation * DT
            heading_integral = max(-HDG_INTEGRAL_LIMIT,
                                   min(heading_integral, HDG_INTEGRAL_LIMIT))
            wz = HDG_KP * heading_deviation + HDG_KI * heading_integral
            wz = max(-HDG_WZ_LIMIT, min(wz, HDG_WZ_LIMIT))
        else:
            wz = 0.0
            heading_integral = 0.0

        # 距离 PID
        speed_cmd = pid_dist.compute(setpoint=target_m, measurement=current_dist, dt=DT)
        if current_dist < target_m and speed_cmd < DIST_MIN_SPEED:
            speed_cmd = DIST_MIN_SPEED

        vx = vx_dir * speed_cmd
        vy = vy_dir * speed_cmd

        # 闭环驱动
        omni_drive_closed_loop(vx, vy, wz, raw_speeds, DT)

        # 打印
        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
            last_print_ms = now_ms
            avg_speed = sum(abs(s) for s in raw_speeds) / 4
            print("  {:5.1f}s  {:5.1f}cm  {:5.3f}  {:6.1f} {:+.3f}".format(
                elapsed_s, current_dist * 100, avg_speed, imu_motion.yaw, wz))

        if current_dist >= target_m:
            break

        time.sleep_ms(int(DT * 1000))
        gc.collect()

    yaw_drift = imu_motion.yaw - target_heading
    while yaw_drift > 180: yaw_drift -= 360
    while yaw_drift < -180: yaw_drift += 360
    print("  >>> [{:s}] done: {:.1f}cm  yaw_drift={:+.1f}° <<<".format(
        label, current_dist * 100, yaw_drift))
    return True


# ============================================================
#  三、旋转（梯形速度曲线 + 前馈 + P 反馈）
# ============================================================
def rotate_to(target_delta, label):
    """
    target_delta: 目标旋转角度（正=左转/逆时针，负=右转/顺时针）
    梯形曲线：加速→匀速→减速，前馈 wz + P 修正，角度到位停车
    返回 True=成功，False=中止
    """
    for _ in range(5):
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        time.sleep_ms(10)

    start_yaw = imu_motion.yaw
    target_yaw = start_yaw + target_delta
    while target_yaw > 180: target_yaw -= 360
    while target_yaw < -180: target_yaw += 360

    gyro_offset_z = imu_motion.gyro_offset_z
    gz_filtered = 0.0
    start_ms = time.ticks_ms()
    last_print_ms = start_ms

    print("\n  ── [{:s}] rotate {:+.0f}°  start={:.1f}°  target={:.1f}° ──".format(
        label, target_delta, start_yaw, target_yaw))
    print("  {:>5s}  {:>7s}  {:>7s}  {:>7s}  {:>7s}  {:>7s}".format(
        "time", "gz_f", "tgt", "err", "wz", "yaw"))

    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        if elapsed_s > TIMEOUT_S:
            print("  TIMEOUT")
            return False

        if switch2.value() != state2:
            print("  SW2 stop")
            return False

        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

        angle_err = target_yaw - imu_motion.yaw
        while angle_err > 180: angle_err -= 360
        while angle_err < -180: angle_err += 360

        if abs(angle_err) <= ROT_DEADBAND:
            break

        # 梯形速度曲线：v = min(√(2a|e|), vmax)
        ideal = (2 * ROT_MAX_ACCEL * abs(angle_err)) ** 0.5
        target_rate = min(ideal, ROT_MAX_RATE)
        if angle_err < 0:
            target_rate = -target_rate

        # 陀螺仪速率
        gz_raw = d[5]
        gz_dps = (gz_raw - gyro_offset_z) / 16.4
        if gz_filtered == 0.0:
            gz_filtered = gz_dps
        else:
            gz_filtered = GZ_FILTER_ALPHA * gz_filtered + (1 - GZ_FILTER_ALPHA) * gz_dps

        # 前馈 + P 反馈
        wz = target_rate * ROT_FF_GAIN + ROT_FB_KP * (target_rate - gz_filtered)
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

    _ = get_encoder_counts()

    actual_delta = imu_motion.yaw - start_yaw
    while actual_delta > 180: actual_delta -= 360
    while actual_delta < -180: actual_delta += 360
    print("  >>> [{:s}] done: {:.1f}°  error={:+.1f}° <<<".format(
        label, actual_delta, actual_delta - target_delta))
    return True


# ============================================================
#  四、主流程
# ============================================================
print("\n" + "=" * 60)
print("  Closed-Loop Route: RIGHT 30 → FWD 60 → ROT 180 → RIGHT 30")
print("  Dist KP={:.1f} KI={:.1f}  Head KP={:.2f} KI={:.3f}".format(
    DIST_KP, DIST_KI, HDG_KP, HDG_KI))
print("  Rot: trapezoid  max={:.0f}°/s  accel={:.0f}°/s²  ff={:.4f}  fb={:.4f}".format(
    ROT_MAX_RATE, ROT_MAX_ACCEL, ROT_FF_GAIN, ROT_FB_KP))
print("  SWITCH2 to abort")
print("=" * 60)

running = True

# [1/4] 右移 30cm → vx=0, vy=+1（右移）
if running:
    running = move_straight(0, 1, 0.30, "1/4 RIGHT 30cm")

# [2/4] 前进 60cm → vx=+1, vy=0（前进）
if running:
    running = move_straight(1, 0, 0.60, "2/4 FORWARD 60cm")

# [3/4] 旋转 180°（左转=正方向）
if running:
    running = rotate_to(180, "3/4 ROTATE 180")

# [4/4] 右移 30cm
if running:
    running = move_straight(0, 1, 0.30, "4/4 RIGHT 30cm")

# ============================================================
#  五、清理
# ============================================================
omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
stop_all()
enc_ticker.start(10)
led.off()
print("\n=== Done ===")
