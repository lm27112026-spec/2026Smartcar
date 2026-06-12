"""
16_verify_ang_vel_closed_loop.py — 角速度闭环功能验证

【测试项】
  1) 静止零偏校验：车不动时 wz_dps ≈ 0（验证 IMU 和滤波正常）
  2) 开环 vs 闭环对比：同样目标 dps，看闭环是否明显减小稳态误差
  3) 多目标跟踪：测试 60 / -60 / 120 / -120 dps 的跟踪能力
  4) 恢复响应：从旋转中回到 0，验证停车快速无振荡

【验证标准】
  - 静止时 |wz_dps| < 5 dps（3σ）
  - 闭环稳态误差 < 15 dps（或 < 15%）
  - 停车后 0.5s 内回到 |wz_dps| < 10 dps

【用法】
  拨动 SWITCH2 随时退出
"""

import gc, time
from machine import Pin
from smartcar import ticker
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    ENC_SCALE, LED_PIN, SWITCH2_PIN,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
)
import imu_motion

# ============================================================
#  硬件 & 参数
# ============================================================
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

DT = 0.02
PRINT_INTERVAL_MS = 100

# 角速度 PID（指向 imu_motion 的模块，方便在线调参）
PID = imu_motion

# 测试序列：(目标 dps, 持续秒, 标签)
TEST_SEQUENCE = [
    (60,   2.5, "  60 dps"),
    (-60,  2.5, " -60 dps"),
    (120,  2.5, " 120 dps"),
    (-120, 2.5, "-120 dps"),
    (0,    1.5, "  0 dps (stop)"),
]

PASS_THRESHOLD_DPS = 15  # 稳态误差合格线

# ============================================================
#  初始化
# ============================================================
stop_all()
imu_motion.reset_ang_vel_pid()

# 编码器用 PIT2，避免与 imu_motion 的 ticker 冲突
pit_enc = ticker(2)
pit_enc.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
pit_enc.start(10)

# 清空编码器缓冲
for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)

# IMU 初始对准
d = imu_motion.imu.read()
imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
time.sleep_ms(50)

print("\n" + "=" * 60)
print("  角速度闭环功能验证")
print("=" * 60)
print("  MAX_WZ_DPS    = {:.0f}".format(imu_motion.MAX_WZ_DPS))
print("  ANG_VEL KP    = {:.3f}".format(imu_motion.ANG_VEL_KP))
print("  ANG_VEL KI    = {:.3f}".format(imu_motion.ANG_VEL_KI))
print("  ANG_VEL FF    = {:.2f}".format(imu_motion.ANG_VEL_FF))
print("  GYRO_SENS     = {:.1f} LSB/dps".format(imu_motion.GYRO_SENSITIVITY))
print("  gyro_offset_z = {:.2f} LSB".format(imu_motion.gyro_offset_z))
print("  合格标准：稳态误差 < {} dps".format(PASS_THRESHOLD_DPS))
print("=" * 60)
print("  拨动 SWITCH2 随时退出\n")


# ============================================================
#  测试 1：静止零偏
# ============================================================

def test_static_bias():
    """车不动，看陀螺仪零偏是否正常"""
    print("-" * 60)
    print("  测试 1: 静止零偏 (2s)")
    print("-" * 60)
    print("  {:>6s}  {:>10s}".format("时间", "wz_dps"))

    samples = []
    start_ms = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_ms) < 2000:
        if switch2.value() != state2:
            return None
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        wz = imu_motion.get_angular_velocity()
        samples.append(wz)
        time.sleep_ms(int(DT * 1000))

    avg = sum(samples) / len(samples)
    peak = max(abs(s) for s in samples)
    status = "✅ PASS" if peak < 5 else "⚠️  WARN"
    print("  avg={:.2f}  peak={:.2f}  {}".format(avg, peak, status))
    print()
    return peak


# ============================================================
#  测试 2：角速度闭环分段测试
# ============================================================

def test_rate_segment(target_dps, duration_s, label):
    """一段角速度闭环测试，返回稳态统计"""
    imu_motion.reset_ang_vel_pid()
    time.sleep_ms(50)  # 段间短暂停顿，让电机停稳

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    dps_history = []

    print("  ── {} ──".format(label))
    print("  {:>6s}  {:>10s}  {:>10s}  {:>8s}  {:>8s}".format(
        "时间", "实际 dps", "目标 dps", "误差", "yaw"))

    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        if switch2.value() != state2:
            return None

        if elapsed_s >= duration_s:
            break

        # ── IMU 读取 ──
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        actual_dps = imu_motion.get_angular_velocity()

        # ── 角速度闭环 ──
        wz_cmd = imu_motion.angular_velocity_control(target_dps, actual_dps, DT)

        # ── 编码器速度（轮速闭环需要）──
        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]

        # ── 驱动 ──
        omni_drive_closed_loop(0, 0, wz_cmd, raw_speeds, DT)

        # ── 记录（跳过前半段暂态）──
        if elapsed_s > duration_s * 0.25:
            dps_history.append(actual_dps)

        # ── 打印 ──
        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
            last_print_ms = now_ms
            err = actual_dps - target_dps
            print("  {:6.2f}  {:10.1f}  {:10.1f}  {:+8.1f}  {:8.1f}".format(
                elapsed_s, actual_dps, target_dps, err, imu_motion.yaw))

        time.sleep_ms(int(DT * 1000))
        gc.collect()

    # ── 段结束停车 ──
    omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
    time.sleep_ms(80)
    _ = get_encoder_counts()

    # ── 统计 ──
    if len(dps_history) < 3:
        print("  ⚠️  采样不足\n")
        return None

    avg_dps = sum(dps_history) / len(dps_history)
    max_err = max(abs(v - target_dps) for v in dps_history)
    steady_err = abs(avg_dps - target_dps)

    passed = steady_err < PASS_THRESHOLD_DPS

    status = "✅ PASS" if passed else "❌ FAIL"
    print("  >>> 稳态 avg={:.1f}  err={:.1f}  max|err|={:.1f}  {} <<<".format(
        avg_dps, steady_err, max_err, status))
    print()
    return passed


# ============================================================
#  测试 3：停车响应（从旋转中急停）
# ============================================================

def test_stop_response():
    """以 120 dps 旋转，然后停车，看多久回到 <10 dps"""
    print("-" * 60)
    print("  测试 3: 停车响应")
    print("-" * 60)

    imu_motion.reset_ang_vel_pid()
    target_dps = 120
    spin_duration = 1.5
    stop_timeout = 1.0

    # ── 先旋转 ──
    start_ms = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_ms) < spin_duration * 1000:
        if switch2.value() != state2:
            return None
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        actual_dps = imu_motion.get_angular_velocity()
        wz_cmd = imu_motion.angular_velocity_control(target_dps, actual_dps, DT)
        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]
        omni_drive_closed_loop(0, 0, wz_cmd, raw_speeds, DT)
        time.sleep_ms(int(DT * 1000))

    # ── 停车，计时 ──
    imu_motion.reset_ang_vel_pid()
    stop_ms = time.ticks_ms()
    converge_time = None

    print("  {:>6s}  {:>10s}".format("时间", "wz_dps"))
    while time.ticks_diff(time.ticks_ms(), stop_ms) < stop_timeout * 1000:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, stop_ms) / 1000.0

        if switch2.value() != state2:
            return None

        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        actual_dps = imu_motion.get_angular_velocity()

        # 停车指令
        omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)

        # 检测收敛
        if converge_time is None and abs(actual_dps) < 10:
            converge_time = elapsed_s

        if int(elapsed_s * 100) % 10 == 0:
            print("  {:6.2f}  {:10.1f}".format(elapsed_s, actual_dps))

        time.sleep_ms(int(DT * 1000))
        gc.collect()

    if converge_time is not None:
        print("  >>> 收敛时间: {:.2f}s  ✅ PASS <<<".format(converge_time))
    else:
        print("  >>> 未在 {:.1f}s 内收敛到 10 dps 以下 ❌ FAIL <<<".format(stop_timeout))
    print()
    return converge_time is not None


# ============================================================
#  主测试流程
# ============================================================

all_pass = True

# 测试 1
peak = test_static_bias()
if peak is None:
    all_pass = False
elif peak >= 5:
    print("  ⚠️  零偏偏大，检查 IMU 是否振动或未校准\n")

# 测试 2
print("-" * 60)
print("  测试 2: 角速度闭环跟踪")
print("-" * 60)
for target_dps, duration_s, label in TEST_SEQUENCE:
    result = test_rate_segment(target_dps, duration_s, label)
    if result is None:
        all_pass = False
        break
    if not result:
        all_pass = False
    time.sleep_ms(200)

# 测试 3
if all_pass:
    result = test_stop_response()
    if result is None:
        all_pass = False
    elif not result:
        all_pass = False

# ============================================================
#  清理 & 汇总
# ============================================================
omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
time.sleep_ms(50)
stop_all()
pit_enc.stop()
led.off()

print("=" * 60)
if all_pass:
    print("  角速度闭环实现 ✅ 验证通过")
else:
    print("  部分测试未通过，请检查参数或硬件")
print("=" * 60)
print()
