"""
uart_move.py — 闭环路线：右移30 → 前进60 → 旋转180° → 右移30
【控制】
  直线段：距离 PID + 编码器反馈精确停车
         所有方向均有航向保持（yaw 偏差 → 角速度闭环纠正）
  旋转段：梯形速度曲线 + 角速度闭环（imu_motion.angular_velocity_control）
【安全】SWITCH2 随时终止
【依赖】imu_motion 角速度闭环（需先完成 MAX_WZ_DPS 标定）
"""

import gc, time
from machine import Pin
from pid import PID
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker, ENC_SCALE, LED_PIN, SWITCH2_PIN, MAX_SPEED_MPS,
)
import imu_motion
from imu_motion import (
    update_angle, get_angular_velocity, angular_velocity_control,
    reset_ang_vel_pid, MAX_WZ_DPS,
)

# ── 硬件 ──
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ── 全局参数 ──
DT = 0.01                   # 控制周期 10ms
TIMEOUT_S = 30
PRINT_MS = 200

# 距离 PID
DIST_KP = 1.0
DIST_KI = 0.5
DIST_OUT_LIMIT = 0.30
MIN_SPEED = 0.1
    
# 航向 PID（外环：航向偏差 → 目标 dps，内环由 imu_motion 角速度闭环处理）
HDG_KP = 0.08     # 航向偏差 (°) → 目标 dps：1° → 30dps，5° → 150dps
HDG_DB = 0.5      # 航向死区（度）

# 旋转参数（梯形速度曲线）
ROT_MAX_RATE = 150   # dps（角速度闭环已验证 120dps 误差 < 15%，150 也应在范围内）
ROT_MAX_ACCEL = 90   # dps/s²
ROT_DEADBAND = 3.0   # 到位判定（度）

# ── 初始化 ──
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


# ============================================================
#  直线移动（vx=前进, vy=横向）
#  横移不用航向 PID（避免 vy+wz 干涉）
# ============================================================
def move_straight(vx_dir, vy_dir, target_m, label, use_heading=True):
    """返回 True=成功"""
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    pid_dist = PID(kp=DIST_KP, ki=DIST_KI, kd=0.0,
                   integral_limit=0.5, output_limit=DIST_OUT_LIMIT)

    total_counts = [0, 0, 0, 0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms

    # 航向状态
    target_heading = None
    reset_ang_vel_pid()

    if use_heading:
        for _ in range(5):
            d = imu_motion.imu.read()
            update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
            time.sleep_ms(10)
        target_heading = imu_motion.yaw

    print("\n  ── [{:s}] ──".format(label))

    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        if elapsed_s > TIMEOUT_S:
            print("  TIMEOUT")
            return False
        if switch2.value() != state2:
            print("  SW2 stop")
            return False

        # 单次 IMU 读取（减少延迟，提高航向保持响应）
        wz = 0.0
        if use_heading:
            d = imu_motion.imu.read()
            update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
            hdg_err = target_heading - imu_motion.yaw
            while hdg_err > 180: hdg_err -= 360
            while hdg_err < -180: hdg_err += 360
            # 航向偏差 → 目标 dps → 角速度闭环
            if abs(hdg_err) > HDG_DB:
                target_dps = hdg_err * HDG_KP * MAX_WZ_DPS
                target_dps = max(-180, min(target_dps, 180))
                wz = angular_velocity_control(target_dps, get_angular_velocity(), DT)

        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]
        for i in range(4):
            total_counts[i] += raw_counts[i]

        wd = [abs(total_counts[i]) / ENC_SCALE[i] for i in range(4)]
        dist = sum(wd) / 4

        speed_cmd = pid_dist.compute(setpoint=target_m, measurement=dist, dt=DT)
        if dist < target_m and speed_cmd < MIN_SPEED:
            speed_cmd = MIN_SPEED

        vx = vx_dir * speed_cmd
        vy = vy_dir * speed_cmd

        omni_drive_closed_loop(vx, vy, wz, raw_speeds, DT)

        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_MS:
            last_print_ms = now_ms
            print("  {:4.1f}s  {:5.1f}cm  spd={:.3f}  yaw={:.1f}  wz={:+.3f}".format(
                elapsed_s, dist * 100, sum(abs(s) for s in raw_speeds) / 4,
                imu_motion.yaw if use_heading else 0, wz))

        if dist >= target_m:
            break

        time.sleep_ms(10)

    print("  >>> [{:s}] done: {:.1f}cm <<<".format(label, dist * 100))
    return True


# ============================================================
#  旋转（梯形曲线 + 前馈）
# ============================================================
def rotate_to(target_delta, label):
    """返回 True=成功（角速度闭环 + 梯形速度曲线）"""
    reset_ang_vel_pid()
    for _ in range(5):
        d = imu_motion.imu.read()
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        time.sleep_ms(10)

    start_yaw = imu_motion.yaw
    target_yaw = start_yaw + target_delta
    while target_yaw > 180: target_yaw -= 360
    while target_yaw < -180: target_yaw += 360

    start_ms = time.ticks_ms()
    last_print_ms = start_ms

    print("\n  ── [{:s}] rotate {:+.0f}° ──".format(label, target_delta))

    while True:
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, start_ms) / 1000.0 > TIMEOUT_S:
            print("  TIMEOUT"); return False
        if switch2.value() != state2:
            print("  SW2 stop"); return False

        # IMU 读取（使用 imu_motion 的 update_angle + _wz_dps）
        d = imu_motion.imu.read()
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

        err = target_yaw - imu_motion.yaw
        while err > 180: err -= 360
        while err < -180: err += 360
        if abs(err) <= ROT_DEADBAND:
            break

        # 梯形速度曲线：目标角速度 (dps)
        ideal = (2 * ROT_MAX_ACCEL * abs(err)) ** 0.5
        tgt_rate = min(ideal, ROT_MAX_RATE)
        if err < 0: tgt_rate = -tgt_rate

        # 角速度闭环（替代了手动 FF+P 滤波代码）
        actual_dps = get_angular_velocity()
        wz = angular_velocity_control(tgt_rate, actual_dps, DT)

        rc = get_encoder_counts()
        rs = [rc[i] / ENC_SCALE[i] / DT for i in range(4)]
        omni_drive_closed_loop(0, 0, wz, rs, DT)

        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_MS:
            last_print_ms = now_ms
            print("  {:4.1f}s  gz={:+.0f}  tgt={:+.0f}  err={:+.0f}  wz={:+.3f}  yaw={:.0f}".format(
                time.ticks_diff(now_ms, start_ms) / 1000.0,
                actual_dps, tgt_rate, err, wz, imu_motion.yaw))

        time.sleep_ms(int(DT * 1000))

    omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
    _ = get_encoder_counts()
    dlt = imu_motion.yaw - start_yaw
    while dlt > 180: dlt -= 360
    while dlt < -180: dlt += 360
    err_deg = dlt - target_delta
    while err_deg > 180: err_deg -= 360
    while err_deg < -180: err_deg += 360
    print("  >>> [{:s}] done: {:.1f}°  err={:+.1f}° <<<".format(label, dlt, err_deg))
    return True


# ============================================================
#  主流程
# ============================================================
print("\n" + "=" * 50)
print("  Route: RIGHT 30 → FWD 60 → ROT 180 → RIGHT 30")
print("=" * 50)

ok = move_straight(1, 0, 0.30, "1/4 RIGHT 30cm", use_heading=True)
if ok:
    ok = move_straight(1, 0, 0.60, "2/4 FWD 60cm", use_heading=True)
if ok:
    ok = rotate_to(180, "3/4 ROT 180°")
if ok:
    ok = move_straight(0, 1, 0.30, "4/4 RIGHT 30cm", use_heading=True)

omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
stop_all()
enc_ticker.start(10)
led.off()
print("\n=== Done ===")
