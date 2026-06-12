"""
motor_test.py — 速度环调试工具
【功能】逐一测试 6 个基本运动方向，用闭环 PID 驱动，对比目标速度与编码器实测速度。
【用法】烧录后观察每段运动的轮速跟踪情况：
        目标速度 → 实际速度的收敛过程和稳态误差。
【安全】拨码开关 D9 随时中止。
"""
from machine import Pin
import time, gc
from motor import (
    omni_drive_closed_loop,
    get_encoder_counts,
    get_encoder_speeds_filtered,
    enc_ticker,
    stop_all,
    reset_wheel_pi,
    omni_kinematics,
    MAX_SPEED_MPS,
)

# ── 硬件 ────────────────────────────────────────────────────

switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ── 参数 ────────────────────────────────────────────────────

SPEED  = 0.15       # 平移速度指令
TURN   = 0.10       # 旋转速度指令
TIME   = 3000       # 每段时长 ms
PAUSE  = 800        # 段间暂停 ms
DT     = 0.02       # 控制周期 20ms (50Hz)
PRINT_IVAL = 200    # 打印间隔 ms

# ── 六向测试序列 ────────────────────────────────────────────
# (名称,  vx,  vy,  wz,  应观察到的现象)

SEQUENCE = [
    ("前进",   SPEED,  0,  0,    "四轮同速转动，车体向前 ↑"),
    ("后退",   -SPEED, 0,  0,    "四轮同速转动，车体向后 ↓"),
    ("右移",  0,  SPEED,    0,    "对角轮对转，车体右移 →"),
    ("左移",  0, -SPEED,    0,    "对角轮对转，车体左移 ←"),
    ("左自转", 0,  0,    TURN,   "四轮同向，车体逆时针 ↺"),
    ("右自转", 0,  0,   -TURN,   "四轮同向，车体顺时针 ↻"),
]

LOOP = True   # True=循环测试, False=跑一轮


def arrow(vx, vy, wz):
    if wz > 0:  return '<'   # ↺ (逆时针)
    if wz < 0:  return '>'   # ↻ (顺时针)
    if vx > 0:  return '>'
    if vx < 0:  return '<'
    if vy > 0:  return '^'
    if vy < 0:  return 'v'
    return '.'


def main():
    print("=" * 60)
    print("  Velocity Loop Debug (速度环调试)")
    print("  目标速度 vs 编码器实测 — 观测 PI 跟踪效果")
    print("  Switch D9 to stop")
    print("=" * 60)

    stop_all()
    enc_ticker.stop()
    _ = get_encoder_counts()  # 清空编码器启动累积值
    round_num = 0

    while True:
        round_num += 1
        print("\n  ======== Round {} ========".format(round_num))

        for name, vx, vy, wz, expected in SEQUENCE:
            if switch2.value() != state2:
                print("\n  [Switch] 中止。")
                stop_all()
                return

            # ── 段间复位 ──
            reset_wheel_pi()
            gc.collect()

            # ── 打印表头 + 目标速度 ──
            if vx != 0 or vy != 0 or wz != 0:
                norms = omni_kinematics(vx, vy, wz)
                targets = [n * MAX_SPEED_MPS[i] for i, n in enumerate(norms)]
                a = arrow(vx, vy, wz)
                print("\n  ── [{}] vx={:+.2f} vy={:+.2f} wz={:+.2f}  {}  {} ──".format(
                    name, vx, vy, wz, a, expected))
                print("  tgt(m/s) lf:{:+.3f} rf:{:+.3f} lb:{:+.3f} rb:{:+.3f}".format(*targets))
                print("  {:>5s} | {:>8s} {:>8s} {:>8s} {:>8s} | {:>6s} {:>6s} {:>6s} {:>6s}".format(
                    "time", "lf_act", "rf_act", "lb_act", "rb_act",
                    "e_lf", "e_rf", "e_lb", "e_rb"))
                print("  " + "-" * 65)
            else:
                # 零速停车
                print("\n  ── [{}] 停车 ──".format(name))
                targets = [0, 0, 0, 0]

            # ── 控制循环 ──
            seg_start = time.ticks_ms()
            last_print = seg_start

            while time.ticks_diff(time.ticks_ms(), seg_start) < TIME:
                if switch2.value() != state2:
                    print("\n  [Switch] 中止。")
                    stop_all()
                    return

                actual = get_encoder_speeds_filtered(DT)
                omni_drive_closed_loop(vx, vy, wz, actual, DT)

                now = time.ticks_ms()
                if time.ticks_diff(now, last_print) >= PRINT_IVAL:
                    elapsed = time.ticks_diff(now, seg_start) * 0.001
                    errors = [t - a for t, a in zip(targets, actual)]
                    print("  {:4.1f}s | {:+8.3f} {:+8.3f} {:+8.3f} {:+8.3f} | {:+5.3f} {:+5.3f} {:+5.3f} {:+5.3f}".format(
                        elapsed,
                        actual[0], actual[1], actual[2], actual[3],
                        errors[0], errors[1], errors[2], errors[3]))
                    last_print = now

                time.sleep_ms(int(DT * 1000))

            # ── 段间停车 ──
            stop_all()
            if name != SEQUENCE[-1][0] or not LOOP:
                print("  ── 暂停 {}ms ──".format(PAUSE))
                pause_start = time.ticks_ms()
                while time.ticks_diff(time.ticks_ms(), pause_start) < PAUSE:
                    if switch2.value() != state2:
                        print("\n  [Switch] 中止。")
                        stop_all()
                        return
                    time.sleep_ms(50)

        if not LOOP:
            break

    print("\n  调试完成。")
    stop_all()


try:
    main()
except KeyboardInterrupt:
    print("\n  用户中止。")
finally:
    stop_all()
    enc_ticker.start(10)

