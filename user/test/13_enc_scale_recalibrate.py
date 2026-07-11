"""
13_enc_scale_recalibrate.py - ENC_SCALE 重新标定脚本（两阶段 + 日志）
【两阶段流程】
  阶段1(DRIVE)：小车自动跑 3 档速度，记录脉冲到临时文件
               控制台打印脉冲数据，用户手动测量距离
  阶段2(CALC)：用户把实测距离填入 MEASURED_DISTANCE_M，重跑脚本
               自动读取临时文件 + 计算 ENC_SCALE + 写日志
【使用】
  第1次运行：小车跑完 → 控制台看到脉冲 → 用卷尺量距离
  修改 MEASURED_DISTANCE_M = 实测距离(米)
  第2次运行：自动计算 ENC_SCALE → 输出结果 + 日志文件
"""
import gc, time, os
from machine import Pin
from motor import (
    set_motor, stop_all, get_encoder_counts,
    enc_ticker,
    ENC_SCALE, LED_PIN, MAX_PWM,
    MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB,
)
import imu_motion

# ============================================================
#  ⚠ 配置参数 — 第2次运行前修改 MEASURED_DISTANCE_M
# ============================================================
MEASURED_DISTANCE_M = 0.0   # ← 第1次跑完后填入实测距离（米），再重跑

SPEED_STEPS       = [15000, 25000, 35000]
STEP_DURATION_MS  = 3000
PAUSE_BETWEEN_MS  = 1000
WHEEL_NAMES       = ["RF", "LF", "LB", "RB"]
LOG_DIR           = "user/test"
PULSE_CACHE_FILE  = "user/test/_pulse_cache.txt"

# ── IMU 航向 PI 参数 ──
HEADING_KP   = 0.25
HEADING_KI   = 0.005
YAW_DEADBAND = 1.0
WZ_LIMIT     = 0.35

# ── 左右电机平衡因子 ──
# 值越小则该轮越慢。地板模式：PWM = FLOOR + (pwm - FLOOR) × 因子
# 避免 PWM 过低导致电机不转
BALANCE_RF = 1.0
BALANCE_LF = 0.70
BALANCE_LB = 0.55
BALANCE_RB = 1.0
MIN_PWM_FLOOR = 8000   # 最低 PWM，低于此值电机无法启动

# ============================================================
#  文件工具
# ============================================================
def ensure_dir(path):
    parts = path.strip("/").split("/")
    cur = ""
    for p in parts:
        cur = cur + "/" + p if cur else p
        try:
            os.listdir(cur)
        except OSError:
            try:
                os.mkdir(cur)
            except OSError:
                pass

def write_file(filepath, lines):
    # 预创建目录
    d = filepath.rsplit("/", 1)[0] if "/" in filepath else "."
    if d != ".":
        ensure_dir(d)
    with open(filepath, "w") as f:
        for line in lines:
            f.write(str(line) + "\n")

def read_file_lines(filepath):
    try:
        with open(filepath, "r") as f:
            return f.read().strip().split("\n")
    except:
        return None

# ============================================================
#  IMU 辅助
# ============================================================
def init_imu(n=10):
    for _ in range(n):
        try:
            d = imu_motion.imu.read()
            imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        except:
            pass
        time.sleep_ms(10)
    return imu_motion.yaw

def read_imu():
    try:
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    except:
        pass
    return imu_motion.yaw

def heading_wz(target_yaw, current_yaw, integral):
    e = target_yaw - current_yaw
    while e > 180: e -= 360
    while e < -180: e += 360
    if abs(e) < YAW_DEADBAND:
        return 0.0, 0.0
    integral += e * 0.02
    integral = max(-1.0, min(integral, 1.0))
    wz = HEADING_KP * e + HEADING_KI * integral
    wz = max(-WZ_LIMIT, min(wz, WZ_LIMIT))
    return wz, integral

# ============================================================
#  单档行驶（IMU 航向闭环）
# ============================================================
def drive_one_step(pwm_val, target_heading, duration_ms):
    """返回 (脉冲[4], 偏航漂移)"""
    counts = [0, 0, 0, 0]
    integral = 0.0
    start = time.ticks_ms()
    last_print = start

    # 地板模式 PWM
    base = [0, 0, 0, 0]
    for idx, (bal, neg) in enumerate([
        (BALANCE_RF, False), (BALANCE_LF, True),
        (BALANCE_LB, True), (BALANCE_RB, False)]):
        if pwm_val <= MIN_PWM_FLOOR or bal >= 1.0:
            v = pwm_val
        else:
            above = pwm_val - MIN_PWM_FLOOR
            v = MIN_PWM_FLOOR + int(above * bal)
        base[idx] = -v if neg else v

    print("    PWM: RF={} LF={} LB={} RB={}".format(base[0], base[1], base[2], base[3]))

    while True:
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, start)
        if elapsed >= duration_ms:
            break

        # ── IMU（超时保护）──
        wz = 0.0
        try:
            yaw = read_imu()
            wz, integral = heading_wz(target_heading, yaw, integral)
        except:
            pass

        # 纠偏
        corr = int(abs(wz) * MAX_PWM * 0.5)
        if corr > pwm_val // 2:
            corr = pwm_val // 2

        if wz > 0:
            pwms = [base[0]+corr, base[1]+corr, base[2]+corr, base[3]+corr]
        elif wz < 0:
            pwms = [base[0]-corr, base[1]-corr, base[2]-corr, base[3]-corr]
        else:
            pwms = base

        set_motor(MOTOR_RF, max(0, min(MAX_PWM, pwms[0])))
        set_motor(MOTOR_LF, max(-MAX_PWM, min(0, pwms[1])))
        set_motor(MOTOR_LB, max(-MAX_PWM, min(0, pwms[2])))
        set_motor(MOTOR_RB, max(0, min(MAX_PWM, pwms[3])))

        # 编码器
        try:
            raw = get_encoder_counts()
            for i in range(4):
                counts[i] += raw[i]
        except:
            pass

        # 每秒打印一次进度
        if time.ticks_diff(now, last_print) >= 1000:
            last_print = now
            print("    t={:.0f}s pulses: RF={} LF={} LB={} RB={}".format(
                elapsed/1000.0, counts[0], counts[1], counts[2], counts[3]))

        time.sleep_ms(20)
        gc.collect()

    drift = read_imu() - target_heading
    while drift > 180: drift -= 360
    while drift < -180: drift += 360
    return counts, drift

# ============================================================
#  阶段1：采集脉冲
# ============================================================
def phase_drive():
    stop_all()
    time.sleep_ms(50)
    led = Pin(LED_PIN, Pin.OUT, value=True)
    enc_ticker.stop()
    for _ in range(5):
        get_encoder_counts()
        time.sleep_ms(10)

    print("\n" + "=" * 60)
    print("  PHASE 1: DRIVE — collecting encoder pulses")
    print("  Current ENC_SCALE: {}".format(ENC_SCALE))
    print("  Balance: RF={} LF={} LB={} RB={} (floor={})".format(
        BALANCE_RF, BALANCE_LF, BALANCE_LB, BALANCE_RB, MIN_PWM_FLOOR))
    print("=" * 60)

    print("\n  Initializing IMU...")
    target = init_imu(10)
    print("  Locked heading: {:.2f} deg".format(target))

    all_data = []
    total = [0, 0, 0, 0]
    yaw_log = []

    for idx, pwm in enumerate(SPEED_STEPS):
        print("\n  -- Step {}/{}: PWM={} --".format(idx+1, len(SPEED_STEPS), pwm))
        time.sleep_ms(300)
        for _ in range(3):
            get_encoder_counts()
            time.sleep_ms(10)

        counts, drift = drive_one_step(pwm, target, STEP_DURATION_MS)
        stop_all()
        led.off()

        all_data.append((pwm, counts))
        for i in range(4):
            total[i] += counts[i]
        yaw_log.append(drift)

        print("  Pulses: RF={} LF={} LB={} RB={} | drift={:+.2f}deg".format(
            counts[0], counts[1], counts[2], counts[3], drift))

        if idx < len(SPEED_STEPS) - 1:
            time.sleep_ms(PAUSE_BETWEEN_MS)

    enc_ticker.start(10)

    # 保存脉冲缓存
    cache = []
    cache.append("total_duration_ms={}".format(len(SPEED_STEPS) * STEP_DURATION_MS))
    cache.append("yaw_log={}".format(",".join(str(round(y, 2)) for y in yaw_log)))
    cache.append("total={}".format(",".join(str(c) for c in total)))
    for pwm, counts in all_data:
        cache.append("step:{}:{}".format(pwm, ",".join(str(c) for c in counts)))
    write_file(PULSE_CACHE_FILE, cache)

    dur = len(SPEED_STEPS) * STEP_DURATION_MS / 1000.0
    print("\n" + "=" * 60)
    print("  PHASE 1 COMPLETE — Pulse Data")
    print("  Runtime: {:.1f}s".format(dur))
    print("  RF={:6d}  LF={:6d}  LB={:6d}  RB={:6d}".format(
        total[0], total[1], total[2], total[3]))
    print("  Yaw drift: {}".format([" {:+5.1f} ".format(y) for y in yaw_log]))
    print("")
    print("  >>> NOW measure the distance with tape measure! <<<")
    print("  >>> Edit MEASURED_DISTANCE_M at top of this file <<<")
    print("  >>> Then re-run this script to calculate ENC_SCALE <<<")
    print("=" * 60)

# ============================================================
#  阶段2：计算并写日志
# ============================================================
def phase_calculate():
    lines = read_file_lines(PULSE_CACHE_FILE)
    if lines is None:
        print("\n  ERROR: No pulse cache found. Run DRIVE phase first!")
        return

    total = []
    all_data = []
    yaw_log = []
    total_dur_ms = 0
    for line in lines:
        if line.startswith("total_duration_ms="):
            total_dur_ms = int(line.split("=")[1])
        elif line.startswith("yaw_log="):
            yaw_log = [float(x) for x in line.split("=")[1].split(",")]
        elif line.startswith("total="):
            total = [int(x) for x in line.split("=")[1].split(",")]
        elif line.startswith("step:"):
            parts = line.split(":")
            pwm = int(parts[1])
            counts = [int(x) for x in parts[2].split(",")]
            all_data.append((pwm, counts))

    if not total:
        print("\n  ERROR: Cache file corrupted. Re-run DRIVE phase.")
        return

    m = MEASURED_DISTANCE_M
    total_dur_s = total_dur_ms / 1000.0

    print("\n" + "=" * 60)
    print("  PHASE 2: CALCULATE ENC_SCALE")
    print("=" * 60)
    print("  Measured distance: {:.3f}m".format(m))
    print("  Total duration:    {:.1f}s".format(total_dur_s))
    print("  Old ENC_SCALE: {}".format(ENC_SCALE))

    raw = [abs(total[i]) for i in range(4)]
    ts = [c / m if m > 0 else 0 for c in raw]
    print("\n  -- Overall (total pulses) --")
    for i in range(4):
        print("    {}: pulses={:6d}  ENC_SCALE={:.1f}  (old={})".format(
            WHEEL_NAMES[i], raw[i], ts[i], ENC_SCALE[i]))
    print("    Average = {:.1f}".format(sum(ts)/4))

    pss = [[], [], [], []]
    print("\n  -- Per-step breakdown --")
    for si, (pw, sc) in enumerate(all_data):
        sd = m * STEP_DURATION_MS / (total_dur_s * 1000)
        sa = [abs(c) for c in sc]
        ssc = [c / sd if sd > 0 else 0 for c in sa]
        print("    Step{} (PWM={}): [{:.0f}, {:.0f}, {:.0f}, {:.0f}]".format(
            si+1, pw, ssc[0], ssc[1], ssc[2], ssc[3]))
        for i in range(4):
            pss[i].append(ssc[i])

    avg = [sum(s)/len(s) if s else 0 for s in pss]
    print("\n  -- Multi-speed averaged --")
    for i in range(4):
        print("    {}: ENC_SCALE = {:.1f}  (was {})".format(
            WHEEL_NAMES[i], avg[i], ENC_SCALE[i]))
    print("    Average = {:.1f}".format(sum(avg)/4))

    new = [int(round(s)) for s in avg]
    print("\n  ════════════════════════════════════════════════")
    print("  RECOMMENDED: ENC_SCALE = [{}, {}, {}, {}]".format(*new))
    print("  ════════════════════════════════════════════════")

    # ── 日志 ──
    ts_ms = time.ticks_ms()
    log = []
    log.append("=" * 60)
    log.append("ENC_SCALE Recalibration Log")
    log.append("=" * 60)
    log.append("Timestamp: {}ms".format(ts_ms))
    log.append("")
    log.append("--- Config ---")
    log.append("SPEED_STEPS: {}".format(SPEED_STEPS))
    log.append("Step duration: {}ms".format(STEP_DURATION_MS))
    log.append("Total duration: {:.1f}s".format(total_dur_s))
    log.append("Old ENC_SCALE: {}".format(ENC_SCALE))
    log.append("Heading KP/KI: {}/{}".format(HEADING_KP, HEADING_KI))
    log.append("Balance: RF={} LF={} LB={} RB={}".format(
        BALANCE_RF, BALANCE_LF, BALANCE_LB, BALANCE_RB))
    log.append("PWM floor: {}".format(MIN_PWM_FLOOR))
    log.append("")
    log.append("--- Measured ---")
    log.append("Distance: {:.4f}m".format(m))
    log.append("Yaw drift: {}".format(
        ["{:+.2f}".format(y) for y in yaw_log]))
    log.append("")
    log.append("--- Raw Pulses ---")
    log.append("Total: RF={} LF={} LB={} RB={}".format(*total))
    for si, (pw, sc) in enumerate(all_data):
        log.append("Step{} (PWM={}): RF={} LF={} LB={} RB={}".format(
            si+1, pw, sc[0], sc[1], sc[2], sc[3]))
    log.append("")
    log.append("--- Per-step ENC_SCALE ---")
    for si, (pw, sc) in enumerate(all_data):
        sd = m * STEP_DURATION_MS / (total_dur_s * 1000)
        sa = [abs(c) for c in sc]
        ssc = [c/sd if sd>0 else 0 for c in sa]
        log.append("Step{} (PWM={}): [{:.0f}, {:.0f}, {:.0f}, {:.0f}]".format(
            si+1, pw, ssc[0], ssc[1], ssc[2], ssc[3]))
    log.append("")
    log.append("--- Final ---")
    log.append("Multi-speed averaged ENC_SCALE:")
    for i in range(4):
        log.append("  {}: {:.1f} (old={})".format(WHEEL_NAMES[i], avg[i], ENC_SCALE[i]))
    log.append("")
    log.append("RECOMMENDED: ENC_SCALE = [{}, {}, {}, {}]".format(*new))
    log.append("")
    log.append("--- Apply ---")
    log.append("Edit user/motor.py ENC_SCALE line, then validate.")
    log.append("=" * 60)

    fname = "enc_scale_log_{}.txt".format(ts_ms)
    write_file(LOG_DIR + "/" + fname, log)
    print("\n  [LOG] {}".format(LOG_DIR + "/" + fname))
    print("  Done. Copy the ENC_SCALE line to user/motor.py")
    print("=" * 60)

# ============================================================
#  入口
# ============================================================
try:
    if MEASURED_DISTANCE_M > 0:
        phase_calculate()
    else:
        phase_drive()
except KeyboardInterrupt:
    print("\n  User aborted.")
finally:
    stop_all()
    try:
        enc_ticker.start(10)
    except:
        pass
