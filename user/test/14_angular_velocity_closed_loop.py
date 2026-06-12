"""
14_angular_velocity_closed_loop.py — 角速度闭环测试
【功能】 验证 angular_velocity.py 模块的角速度闭环控制
【原理】 目标角速度 (deg/s) → 角速度 PID (IMU 陀螺仪反馈) → wz → omni_drive_closed_loop
【测试序列】
  1) 静止:      target=0     → 验证无漂移
  2) 左转 90°/s: target=90   → gz 稳定在 90 deg/s
  3) 右转 90°/s: target=-90  → gz 稳定在 -90 deg/s
  4) 左转 180°/s: target=180 → 验证高速跟踪
  5) 静止恢复:   target=0     → 验证能快速停止
【验证标准】 稳态误差 < 10 deg/s，无振荡
【用法】 拨动 SWITCH2 退出
"""

import gc, time
from machine import Pin
from smartcar import ticker as _Ticker
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker, ENC_SCALE, LED_PIN, SWITCH2_PIN,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
)
from angular_velocity import (
    init_angular_vel,
    get_angular_velocity,
    angular_velocity_control,
    stop as angvel_stop,
    reset_pid,
)
from seekfree import IMU660RX
init_angular_vel(pit_id=1, imu_type=IMU660RX.TYPE_RC)

# ============================================================
#  硬件
# ============================================================
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ============================================================
#  参数
# ============================================================
DT = 0.02                      # 控制周期 20ms
PRINT_INTERVAL_MS = 100        # 打印间隔
TEST_DURATION_S = 3.0          # 每个目标持续时间

# IMU 型号（根据实际模块选择）
# IMU660RX.TYPE_RA / TYPE_RB / TYPE_RC / TYPE_AUTO
IMU_TYPE = None  # None=自动检测，或指定 IMU660RX.TYPE_RC

# 测试序列: (目标角速度 deg/s, 持续时间 s, 标签)
TEST_SEQUENCE = [
    (0,     1.0,  "静止验证"),
    (90,    3.0,  "左转 90°/s"),
    (-90,   3.0,  "右转 90°/s"),
    (180,   2.0,  "左转 180°/s"),
    (-180,  2.0,  "右转 180°/s"),
    (0,     1.5,  "恢复静止"),
]

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

# 初始化角速度模块（含 IMU 校准）
print("\n" + "=" * 60)
print("  角速度闭环测试")
print("=" * 60)

try:
    init_angular_vel(pit_id=1, pit_period_ms=10, imu_type=IMU_TYPE)
except Exception as e:
    print("[FATAL] IMU init failed: {}".format(e))
    stop_all()
    raise SystemExit(1)

# 恢复编码器 ticker（motor.py 的 enc_ticker 被 angular_velocity 模块的 pit 占用）
# 这里使用 PIT2 避免与 angular_velocity 的 PIT1 冲突

_pit_enc = _Ticker(2)   # 用 PIT2 避免与 angular_velocity 的 PIT1 冲突
_pit_enc.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
_pit_enc.start(10)

print("\n按 SWITCH2 随时退出\n")


# ============================================================
#  测试函数
# ============================================================

def run_test(target_dps, duration_s, label):
    """运行一段角速度闭环测试"""
    reset_pid()  # 每段开始时重置 PID

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    total_angle = 0.0
    last_wz_dps = 0.0
    sample_count = 0

    print("\n  ── [{:s}] target={:+.0f} deg/s, {:.1f}s ──".format(
        label, target_dps, duration_s))
    print("  {:>5s}  {:>8s}  {:>8s}  {:>8s}  {:>8s}  {:>8s}".format(
        "time", "gz_dps", "target", "wz", "spd_rf", "angle"))
    
    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        # 退出条件
        if switch2.value() != state2:
            print("  SW2 stop")
            return False
        if elapsed_s >= duration_s:
            break

        # ── 角速度闭环 ──
        wz_dps = get_angular_velocity()                   # IMU 反馈
        wz_out = angular_velocity_control(target_dps, wz_dps, DT)  # PID

        # 积分转角（用于粗略验证）
        total_angle += wz_dps * DT
        last_wz_dps = wz_dps
        sample_count += 1

        # ── 编码器速度反馈 ──
        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]

        # ── 驱动（纯旋转，无平移）──
        omni_drive_closed_loop(0, 0, wz_out, raw_speeds, DT)

        # ── 打印 ──
        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
            last_print_ms = now_ms
            print("  {:5.1f}s  {:8.1f}  {:8.1f}  {:+.4f}  {:8.3f}  {:8.1f}".format(
                elapsed_s, wz_dps, target_dps, wz_out, raw_speeds[0], total_angle))

        time.sleep_ms(int(DT * 1000))
        gc.collect()

    # 段结束 — 停车缓冲
    omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
    time.sleep_ms(100)
    _ = get_encoder_counts()

    # 统计
    if sample_count > 10:
        avg_error = abs(target_dps) - abs(total_angle / max(elapsed_s, 0.001)) if target_dps != 0 else 0
        print("  >>> [{:s}] done: angle={:.1f}°  avg_gz={:.1f} dps <<<".format(
            label, total_angle, total_angle / max(elapsed_s, 0.001)))

    return True


# ============================================================
#  主测试序列
# ============================================================

for target_dps, duration_s, label in TEST_SEQUENCE:
    if not run_test(target_dps, duration_s, label):
        break  # SW2 退出
    time.sleep_ms(300)  # 段间暂停


# ============================================================
#  清理
# ============================================================
print("\n清理中...")
omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
time.sleep_ms(100)
stop_all()
_pit_enc.stop()
angvel_stop()
enc_ticker.start(10)   # 恢复 motor.py 的编码器 ticker
led.off()
print("=== Done ===")
