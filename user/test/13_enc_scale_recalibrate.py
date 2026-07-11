"""
13_enc_scale_recalibrate.py - ENC_SCALE 重新标定脚本（带日志）
【功能】
  多速前进标定法：分别以 3 档速度各跑 3 秒，收集编码器脉冲，
  用户输入实测距离后自动计算每轮 ENC_SCALE，并将结果写入日志文件。
【方法】
  1) 以 PWM 15000 / 25000 / 35000 各前进 3 秒
  2) 每档记录 4 轮编码器脉冲
  3) 停车后用户用卷尺测量起点→终点距离
  4) 脚本计算各档位 ENC_SCALE，取加权平均
  5) 结果保存到 user/test/logs/enc_scale_log_YYYYMMDD_HHMMSS.txt
【使用】
  运行前：用记号笔在地面画起始线，准备卷尺
  运行时：小车会自动跑 3 次，每次之间暂停 1 秒
  结束后：测量总距离，输入脚本提示处
"""

import gc, time, os
from machine import Pin
from motor import (
    omni_drive, stop_all, get_encoder_counts,
    enc_ticker,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    ENC_SCALE, LED_PIN, SWITCH2_PIN, MAX_PWM,
)

# ============================================================
#  配置参数
# ============================================================
SPEED_STEPS = [15000, 25000, 35000]   # 三档 PWM
STEP_DURATION_MS = 3000                # 每档运行时间（毫秒）
PAUSE_BETWEEN_MS = 1000                # 档位间暂停时间（毫秒）
WHEEL_NAMES = ["RF", "LF", "LB", "RB"]
LOG_DIR = "user/test/logs"

# ============================================================
#  日志工具
# ============================================================
def ensure_log_dir():
    """确保日志目录存在"""
    try:
        os.listdir(LOG_DIR)
    except OSError:
        try:
            os.mkdir("user/test/logs")
        except OSError:
            try:
                os.mkdir("user/test")
                os.mkdir("user/test/logs")
            except OSError:
                pass

def get_timestamp():
    """获取时间戳字符串（板卡无 RTC 时用运行计数）"""
    return time.ticks_ms()

def write_log(filename, lines):
    """将内容写入日志文件"""
    try:
        ensure_log_dir()
        filepath = LOG_DIR + "/" + filename
        with open(filepath, "w") as f:
            for line in lines:
                f.write(line + "\n")
        print("  [LOG] Saved: {}".format(filepath))
        return True
    except Exception as e:
        print("  [LOG] Error: {}".format(e))
        return False

# ============================================================
#  主流程
# ============================================================
def main():
    stop_all()
    time.sleep_ms(50)

    led = Pin(LED_PIN, Pin.OUT, value=True)

    # 暂停自动采集，手动接管编码器
    enc_ticker.stop()
    for _ in range(5):
        get_encoder_counts()
        time.sleep_ms(10)

    print("\n" + "=" * 60)
    print("  ENC_SCALE Recalibration (Multi-Speed)")
    print("  Current ENC_SCALE: {}".format(ENC_SCALE))
    print("=" * 60)

    # ── 多速标定循环 ──
    all_pulse_data = []   # [(pwm, [counts×4]), ...]
    total_pulses = [0, 0, 0, 0]

    for step_idx, pwm_val in enumerate(SPEED_STEPS):
        print("\n  ── Step {}/{}: PWM = {} ──".format(
            step_idx + 1, len(SPEED_STEPS), pwm_val))
        print("  Ready... marking start line!")
        time.sleep_ms(500)

        # 归零
        for _ in range(3):
            get_encoder_counts()
            time.sleep_ms(10)

        # 驱动
        speed_norm = pwm_val / MAX_PWM
        omni_drive(speed_norm, 0, 0)
        led.on()

        step_counts = [0, 0, 0, 0]
        start_ms = time.ticks_ms()

        while True:
            elapsed = time.ticks_diff(time.ticks_ms(), start_ms)
            if elapsed >= STEP_DURATION_MS:
                break
            counts = get_encoder_counts()
            for i in range(4):
                step_counts[i] += counts[i]
            time.sleep_ms(20)
            gc.collect()

        # 停车
        omni_drive(0, 0, 0)
        stop_all()
        led.off()

        # 记录
        all_pulse_data.append((pwm_val, step_counts[:]))
        for i in range(4):
            total_pulses[i] += step_counts[i]

        print("  Done. Pulses: RF={} LF={} LB={} RB={}".format(
            step_counts[0], step_counts[1], step_counts[2], step_counts[3]))

        # 档位间暂停
        if step_idx < len(SPEED_STEPS) - 1:
            print("  Pausing {}ms before next step...".format(PAUSE_BETWEEN_MS))
            time.sleep_ms(PAUSE_BETWEEN_MS)

    # 恢复自动采集
    enc_ticker.start(10)

    # ── 打印总脉冲 ──
    total_duration_s = len(SPEED_STEPS) * STEP_DURATION_MS / 1000.0
    print("\n" + "=" * 60)
    print("  Total pulses ({:.1f}s driving):".format(total_duration_s))
    print("    RF = {:6d}".format(total_pulses[0]))
    print("    LF = {:6d}".format(total_pulses[1]))
    print("    LB = {:6d}".format(total_pulses[2]))
    print("    RB = {:6d}".format(total_pulses[3]))
    print("")
    print("  NOW MEASURE THE TOTAL DISTANCE!")
    print("  (from first start line to final stop, in meters)")
    print("  Example: if you ran 3 steps of 3s each, measure the full path.")
    print("=" * 60)

    # ── 用户输入距离 ──
    try:
        dist_str = input("\n  Enter total measured distance (m): ")
        measured_m = float(dist_str)
    except:
        print("  Invalid input. Aborting.")
        return

    if measured_m <= 0:
        print("  Distance must be > 0. Aborting.")
        return

    # ── 计算 ENC_SCALE ──
    print("\n" + "=" * 60)
    print("  ENC_SCALE Recalibration Results")
    print("=" * 60)
    print("  Measured distance: {:.3f}m".format(measured_m))
    print("  Total duration:    {:.1f}s".format(total_duration_s))
    print("")

    # 基于总脉冲计算
    raw_counts = [abs(total_pulses[i]) for i in range(4)]
    total_enc_scales = [c / measured_m if measured_m > 0 else 0 for c in raw_counts]

    print("  ── Overall (total pulses) ──")
    for i in range(4):
        print("    {}: pulses={:6d}  ENC_SCALE={:.1f}  (old={})".format(
            WHEEL_NAMES[i], raw_counts[i], total_enc_scales[i], ENC_SCALE[i]))

    avg_scale = sum(total_enc_scales) / 4
    print("    Average = {:.1f}".format(avg_scale))

    # 基于各档位分别计算（取平均）
    per_step_scales = [[], [], [], []]
    print("\n  ── Per-step breakdown ──")
    for step_idx, (pwm_val, step_counts) in enumerate(all_pulse_data):
        step_dist = measured_m * STEP_DURATION_MS / (total_duration_s * 1000)
        step_abs = [abs(c) for c in step_counts]
        step_scales = [c / step_dist if step_dist > 0 else 0 for c in step_abs]
        print("    Step {} (PWM={}): scales = [{:.0f}, {:.0f}, {:.0f}, {:.0f}]".format(
            step_idx + 1, pwm_val,
            step_scales[0], step_scales[1], step_scales[2], step_scales[3]))
        for i in range(4):
            per_step_scales[i].append(step_scales[i])

    # 各轮取多速平均
    averaged_scales = [sum(s) / len(s) if s else 0 for s in per_step_scales]

    print("\n  ── Multi-speed averaged ──")
    for i in range(4):
        print("    {}: ENC_SCALE = {:.1f}  (was {})".format(
            WHEEL_NAMES[i], averaged_scales[i], ENC_SCALE[i]))
    avg_averaged = sum(averaged_scales) / 4
    print("    Overall average = {:.1f}".format(avg_averaged))

    # 推荐值
    new_scale = [int(round(s)) for s in averaged_scales]
    print("\n  ════════════════════════════════════════════════")
    print("  NEW ENC_SCALE (recommended):")
    print("  ENC_SCALE = [{}, {}, {}, {}]".format(
        new_scale[0], new_scale[1], new_scale[2], new_scale[3]))
    print("  ════════════════════════════════════════════════")

    # ── 生成日志内容 ──
    log_lines = []
    log_lines.append("=" * 60)
    log_lines.append("ENC_SCALE Recalibration Log")
    log_lines.append("=" * 60)
    log_lines.append("Timestamp: {}ms since boot".format(get_timestamp()))
    log_lines.append("")

    log_lines.append("--- Configuration ---")
    log_lines.append("SPEED_STEPS (PWM): {}".format(SPEED_STEPS))
    log_lines.append("Step duration: {}ms".format(STEP_DURATION_MS))
    log_lines.append("Total duration: {:.1f}s".format(total_duration_s))
    log_lines.append("Old ENC_SCALE: {}".format(ENC_SCALE))
    log_lines.append("")

    log_lines.append("--- Measured ---")
    log_lines.append("Total distance: {:.4f}m".format(measured_m))
    log_lines.append("")

    log_lines.append("--- Raw Pulses ---")
    log_lines.append("Total: RF={} LF={} LB={} RB={}".format(
        total_pulses[0], total_pulses[1], total_pulses[2], total_pulses[3]))
    for idx, (pwm_val, step_counts) in enumerate(all_pulse_data):
        log_lines.append("Step{} (PWM={}): RF={} LF={} LB={} RB={}".format(
            idx + 1, pwm_val,
            step_counts[0], step_counts[1], step_counts[2], step_counts[3]))
    log_lines.append("")

    log_lines.append("--- Per-step ENC_SCALE ---")
    for step_idx, (pwm_val, step_counts) in enumerate(all_pulse_data):
        step_dist = measured_m * STEP_DURATION_MS / (total_duration_s * 1000)
        step_abs = [abs(c) for c in step_counts]
        step_scales = [c / step_dist if step_dist > 0 else 0 for c in step_abs]
        log_lines.append("Step{} (PWM={}): [{:.0f}, {:.0f}, {:.0f}, {:.0f}]".format(
            step_idx + 1, pwm_val,
            step_scales[0], step_scales[1], step_scales[2], step_scales[3]))
    log_lines.append("")

    log_lines.append("--- Final Result ---")
    log_lines.append("Multi-speed averaged ENC_SCALE:")
    for i in range(4):
        log_lines.append("  {}: {:.1f} (old={})".format(
            WHEEL_NAMES[i], averaged_scales[i], ENC_SCALE[i]))
    log_lines.append("")
    log_lines.append("Recommended NEW ENC_SCALE:")
    log_lines.append("ENC_SCALE = [{}, {}, {}, {}]".format(
        new_scale[0], new_scale[1], new_scale[2], new_scale[3]))
    log_lines.append("")
    log_lines.append("--- Notes ---")
    log_lines.append("To apply: edit user/motor.py, update ENC_SCALE line.")
    log_lines.append("Then re-run this script to validate.")
    log_lines.append("=" * 60)

    # ── 写入日志文件 ──
    ts = get_timestamp()
    log_filename = "enc_scale_log_{}.txt".format(ts)
    write_log(log_filename, log_lines)

    print("\n  Done. Copy the ENC_SCALE line to user/motor.py")
    print("=" * 60)


try:
    main()
except KeyboardInterrupt:
    print("\n  User aborted.")
finally:
    stop_all()
    try:
        enc_ticker.start(10)
    except:
        pass
