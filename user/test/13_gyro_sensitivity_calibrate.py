"""
13_gyro_sensitivity_calibrate.py — GYRO_SENSITIVITY 标定测试
【原理 / Principle】
  陀螺仪灵敏度 GYRO_SENSITIVITY (LSB/(°/s)) 将原始陀螺仪值转换为 °/s。
  如果该值不准确，yaw 积分会产生比例误差。
  本脚本通过已知角度旋转，反向推算出正确的灵敏度：
    new_sensitivity = Σ((gz_raw - offset) * dt) / actual_deg

  即：用已知的实际旋转角度，除以原始陀螺仪积分（去除零偏），
      得到每个 LSB 对应的实际角速度，从而校正灵敏度常数。

【方法 / Method】
  1) 温度稳定 3 秒 + 陀螺仪零偏校准（1000 样本）
  2) 记录旋转过程中的原始 gz 数据
     - manual: SW2 开始/停止，用户手动旋转至已知角度
     - auto:   小车自动旋转，记录 gz
  3) 计算校正后的灵敏度
  4) 验证：用新灵敏度重新积分，对比实际角度
  5) 结果自动保存到 sensitivity_log.txt

【用法 / Usage】
  烧录后在平坦桌面运行。
  拨动 SWITCH2 随时终止。
  运行前确保 IMU 温度与环境平衡（开机后等待 10s 以上）。

【硬件状态 / Hardware】
  机器人平放于桌面。
  Manual 模式：用手缓慢旋转底盘至 ~360°（保持水平，避免倾斜）。
  Auto  模式：轮子着地，确保周围有足够空间，自动旋转 360°。
"""

import gc, time, math
from machine import Pin
from seekfree import IMU660RX

# ============================================================
#  常量 / Constants
# ============================================================

GYRO_SENSITIVITY_OLD = 16.4       # 当前灵敏度 LSB/(°/s)，±2000dps 量程
WARMUP_S = 3.0                     # 温度稳定等待时间（秒）
CALIB_SAMPLES = 1000               # 零偏校准采样数
RECORD_INTERVAL_MS = 5             # 记录采样间隔（毫秒）

# Auto 模式
AUTO_TARGET_DEG = 360.0            # 预设目标角度（度）
AUTO_ROT_SPEED = 0.35              # 自动旋转速度（wz）

# Manual 模式
MANUAL_EXPECTED_DEG = 360.0        # 手动模式预期角度（度）
MANUAL_TIMEOUT_S = 10.0            # 手动模式超时（秒）

# ============================================================
#  引脚 & 外设初始化 / Pin & Peripheral Init
# ============================================================

LED_PIN = 'C4'
SWITCH2_PIN = 'D9'

led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

imu = IMU660RX()

def wait_sw2_toggle(prompt_text):
    """等待用户拨动 SWITCH2，返回拨动后的状态"""
    global state2
    print(prompt_text)
    while switch2.value() == state2:
        time.sleep_ms(20)
    state2 = switch2.value()
    print("  → SW2 detected, continuing...")

# ============================================================
#  打印头 / Header
# ============================================================

print("\n" + "=" * 60)
print("  GYRO_SENSITIVITY Calibration")
print("  Current sensitivity = {:.1f} LSB/(deg/s)".format(GYRO_SENSITIVITY_OLD))
print("=" * 60)

# ============================================================
#  模式选择 / Mode Selection
# ============================================================

print("")
print("  Select mode:")
print("    'm' or 'manual'  → Manual rotation by hand")
print("    'a' or 'auto'    → Automatic rotation by motor")
print("    SW2 anytime to abort")
print("")

mode = ""
while mode not in ("m", "manual", "a", "auto"):
    try:
        raw = input("  Enter mode (m/a): ").strip().lower()
        if raw in ("m", "manual"):
            mode = "manual"
        elif raw in ("a", "auto"):
            mode = "auto"
        else:
            print("  Invalid input. Enter 'm' or 'a'.")
    except:
        print("  Input error. Try again.")

is_auto = (mode == "auto")
mode_label = "AUTO" if is_auto else "MANUAL"
print("  Mode selected: {}".format(mode_label))
print("")

# ============================================================
#  步骤 1：温度稳定 + 零偏校准
#  Step 1: Temperature stabilization + zero offset calibration
# ============================================================

print("=" * 60)
print("  Step 1: Warmup + Gyro Zero Offset Calibration")
print("=" * 60)

# 温度稳定倒计时
print("  Stabilizing IMU temperature for {:.0f}s...".format(WARMUP_S))
for i in range(int(WARMUP_S), 0, -1):
    print("  {}s ...".format(i))
    time.sleep_ms(1000)
    if switch2.value() != state2:
        print("  SW2 detected — aborting.")
        led.off()
        raise SystemExit(0)

print("  Warmup done.")
print("")

# 零偏校准：采集 CALIB_SAMPLES 个样本
print("  Calibrating gyro zero offset ({} samples)...".format(CALIB_SAMPLES))
_gz_sum = 0

for i in range(CALIB_SAMPLES):
    d = imu.read()          # d = [ax, ay, az, gx, gy, gz]
    _gz_sum += d[5]
    time.sleep_ms(2)
    gc.collect()

gyro_offset_z = _gz_sum / CALIB_SAMPLES
print("  gyro_offset_z = {:.3f} LSB".format(gyro_offset_z))
print("  (old GYRO_SENSITIVITY = {:.1f})".format(GYRO_SENSITIVITY_OLD))
print("")

# ============================================================
#  步骤 2：记录旋转过程中的 gz 数据
#  Step 2: Record gz data during rotation
# ============================================================

print("=" * 60)
print("  Step 2: Record gz during rotation")
print("=" * 60)

# 清空残余 IMU 数据
for _ in range(5):
    _ = imu.read()
    time.sleep_ms(10)

# 记录变量
start_time_ms = 0
prev_time_ms = 0
raw_integral = 0.0         # Σ(gz_raw * dt)  LSB·s
debiased_integral = 0.0    # Σ((gz_raw - offset) * dt)  LSB·s
sample_count = 0
recording = False

if is_auto:
    # ── Auto 模式：电机自动旋转 ──
    from motor import omni_drive, stop_all, get_encoder_counts, enc_ticker

    stop_all()
    enc_ticker.stop()
    time.sleep_ms(50)

    print("")
    print("  Auto rotation: omni_drive(0, 0, {:.2f}) for {:.1f}s".format(
        AUTO_ROT_SPEED, MANUAL_TIMEOUT_S))
    print("  Target rotation: ~{:.0f} deg (preset)".format(AUTO_TARGET_DEG))
    print("  Recording gz at {}ms intervals...".format(RECORD_INTERVAL_MS))
    print("  Starting in 1 second...")
    time.sleep_ms(1000)

    omni_drive(0, 0, AUTO_ROT_SPEED)   # 纯旋转
    led.on()

    start_time_ms = time.ticks_ms()
    prev_time_ms = start_time_ms
    recording = True

    timeout_ms = int(MANUAL_TIMEOUT_S * 1000)

    while True:
        now_ms = time.ticks_ms()
        elapsed_ms = time.ticks_diff(now_ms, start_time_ms)

        if elapsed_ms >= timeout_ms:
            print("  Timeout ({:.1f}s) — stopping rotation.".format(MANUAL_TIMEOUT_S))
            break

        if switch2.value() != state2:
            print("  SW2 detected — aborting rotation.")
            break

        # 采样
        d = imu.read()
        gz_raw = d[5]
        dt = time.ticks_diff(now_ms, prev_time_ms) / 1000.0
        prev_time_ms = now_ms

        raw_integral += gz_raw * dt
        debiased_integral += (gz_raw - gyro_offset_z) * dt
        sample_count += 1

        time.sleep_ms(RECORD_INTERVAL_MS)
        gc.collect()

    omni_drive(0, 0, 0)
    stop_all()
    enc_ticker.start(10)
    led.off()

    actual_deg = AUTO_TARGET_DEG   # auto mode uses preset
    print("  Rotation stopped. Samples captured: {}".format(sample_count))
    print("")

else:
    # ── Manual 模式：用户手动旋转 ──
    print("")
    print("  Manual mode: You will rotate the car by hand.")
    print("  1. Press SW2 to START recording")
    print("  2. Slowly rotate the car ~360 deg (keep horizontal)")
    print("  3. Press SW2 again to STOP recording")
    print("  4. Enter the actual rotation angle")
    print("")

    # 等待 SW2 开始
    wait_sw2_toggle("  Press SW2 to START recording...")
    led.on()

    start_time_ms = time.ticks_ms()
    prev_time_ms = start_time_ms
    recording = True
    timeout_ms = int(MANUAL_TIMEOUT_S * 1000)

    print("  Recording started! Slowly rotate the car now...")
    print("  Press SW2 to STOP recording.")

    # 主记录循环
    while True:
        now_ms = time.ticks_ms()
        elapsed_ms = time.ticks_diff(now_ms, start_time_ms)

        if elapsed_ms >= timeout_ms:
            print("  Timeout ({:.1f}s) — auto-stop.".format(MANUAL_TIMEOUT_S))
            break

        if switch2.value() != state2:
            state2 = switch2.value()
            print("  SW2 detected — stopping recording.")
            break

        # 采样
        d = imu.read()
        gz_raw = d[5]
        dt = time.ticks_diff(now_ms, prev_time_ms) / 1000.0
        prev_time_ms = now_ms

        raw_integral += gz_raw * dt
        debiased_integral += (gz_raw - gyro_offset_z) * dt
        sample_count += 1

        time.sleep_ms(RECORD_INTERVAL_MS)
        gc.collect()

    led.off()
    recording = False

    print("  Recording stopped. Samples captured: {}".format(sample_count))
    print("")

    # 步骤 4：用户输入实际角度
    while True:
        try:
            deg_str = input("  Enter actual rotation angle (deg): ")
            actual_deg = float(deg_str)
            if actual_deg <= 0:
                print("  Angle must be positive. Try again.")
                continue
            break
        except:
            print("  Invalid number. Try again (e.g. 360).")

    print("  Actual angle: {:.1f} deg".format(actual_deg))

# ============================================================
#  步骤 3 & 5：计算结果并输出
#  Step 3 & 5: Calculate results and output
# ============================================================

print("")
print("=" * 60)
print("  Step 3 & 5: Sensitivity Calculation & Results")
print("=" * 60)

if sample_count < 2:
    print("  ERROR: Too few samples ({}) — cannot compute.".format(sample_count))
    led.off()
    raise SystemExit(1)

# 用去零偏积分计算新灵敏度
new_sensitivity = debiased_integral / actual_deg if actual_deg > 0 else 0
# 用原始积分（含零偏）计算对比灵敏度
raw_sensitivity = raw_integral / actual_deg if actual_deg > 0 else 0

# 用旧灵敏度估算的角度
old_estimated_deg = debiased_integral / GYRO_SENSITIVITY_OLD
# 用新灵敏度反推的角度（应 ≈ actual_deg）
new_estimated_deg = debiased_integral / new_sensitivity if new_sensitivity > 0 else 0

# 误差百分比
error_pct = (old_estimated_deg - actual_deg) / actual_deg * 100 if actual_deg > 0 else 0
drift_pct = abs(raw_integral - debiased_integral) / abs(debiased_integral) * 100 if abs(debiased_integral) > 1e-9 else 0

# 平均采样间隔
total_duration_s = time.ticks_diff(prev_time_ms, start_time_ms) / 1000.0
dt_avg = total_duration_s / sample_count if sample_count > 0 else 0

# ── 打印详细结果 ──
print("")
print("  ============================================")
print("  GYRO_SENSITIVITY Calibration Report")
print("  Mode: {}".format(mode_label))
print("  ============================================")
print("  Sampling:")
print("    Samples          = {}".format(sample_count))
print("    Duration         = {:.3f}s".format(total_duration_s))
print("    Avg dt           = {:.1f}ms".format(dt_avg * 1000))
print("  --------------------------------------------")
print("  Integrals (gz):")
print("    Raw integral     = {:.2f} LSB*s".format(raw_integral))
print("    Debiased integral= {:.2f} LSB*s".format(debiased_integral))
print("    Zero-bias drift  = {:.3f}%".format(drift_pct))
print("  --------------------------------------------")
print("  Angle Comparison:")
print("    Actual angle     = {:.1f} deg".format(actual_deg))
print("    Old-sens estim.  = {:.1f} deg".format(old_estimated_deg))
print("    Error            = {:.1f} deg ({:+.2f}%)".format(
    old_estimated_deg - actual_deg, error_pct))
print("  --------------------------------------------")
print("  Sensitivity Results:")
print("    Old sensitivity  = {:.1f} LSB/(deg/s)".format(GYRO_SENSITIVITY_OLD))
print("    New sensitivity  = {:.1f} LSB/(deg/s)".format(new_sensitivity))
print("    Raw sensitivity  = {:.1f} LSB/(deg/s)".format(raw_sensitivity))
print("  --------------------------------------------")
print("  Verification:")
print("    Re-integrate(new)= {:.1f} deg".format(new_estimated_deg))
print("  ============================================")

# 建议
print("  -- Recommended Actions --")
if abs(error_pct) < 1.0:
    print("  [OK] Current sensitivity is accurate (< 1% error).")
    print("    No change needed.")
elif abs(error_pct) < 5.0:
    print("  [!] Moderate error ({:+.1f}%). Consider updating:".format(error_pct))
    print("    GYRO_SENSITIVITY = {:.1f}".format(new_sensitivity))
else:
    print("  [X] Large error ({:+.1f}%). Strongly recommend updating:".format(error_pct))
    print("    GYRO_SENSITIVITY = {:.1f}".format(new_sensitivity))
print("")

# ============================================================
#  保存结果到日志 / Save Results to sensitivity_log.txt
# ============================================================

LOG_FILE = "sensitivity_log.txt"

try:
    with open(LOG_FILE, "a") as f:
        timestamp = time.localtime()
        ts_str = "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            timestamp[0], timestamp[1], timestamp[2],
            timestamp[3], timestamp[4], timestamp[5])

        f.write("=" * 60 + "\n")
        f.write("  Calibration run: {}\n".format(ts_str))
        f.write("  Mode: {}\n".format(mode_label))
        f.write("  Samples: {}  Duration: {:.3f}s\n".format(sample_count, total_duration_s))
        f.write("  Actual angle: {:.1f} deg\n".format(actual_deg))
        f.write("  Old sensitivity:  {:.1f} LSB/(deg/s)\n".format(GYRO_SENSITIVITY_OLD))
        f.write("  New sensitivity:  {:.1f} LSB/(deg/s)\n".format(new_sensitivity))
        f.write("  Raw sensitivity:  {:.1f} LSB/(deg/s)\n".format(raw_sensitivity))
        f.write("  Old estimate:     {:.1f} deg  (error: {:+.1f} deg / {:+.2f}%)\n".format(
            old_estimated_deg, old_estimated_deg - actual_deg, error_pct))
        f.write("  Zero-bias drift:  {:.3f}%\n".format(drift_pct))
        f.write("  gyro_offset_z:    {:.3f} LSB\n".format(gyro_offset_z))
        f.write("  Avg dt:           {:.1f}ms\n".format(dt_avg * 1000))
        f.write("=" * 60 + "\n\n")

    print("  [OK] Results appended to '{}'.".format(LOG_FILE))
except Exception as e:
    print("  [X] Failed to write log: {}".format(e))
    print("    (Log will not be saved, but results are printed above.)")

# ============================================================
#  清理 / Cleanup
# ============================================================

led.off()
print("")
print("  Done. LED off. You may reset the board.")
print("")
