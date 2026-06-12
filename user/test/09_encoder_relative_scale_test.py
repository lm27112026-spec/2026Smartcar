"""
09_encoder_relative_scale_test.py — 相对比例法编码器标定测试
【功能】通过原地旋转测试四轮编码器的一致性
【原理】原地旋转时四轮理论速度相同，脉冲增量应接近 1:1:1:1
【用法】拨动开关 D9 开始测试，观察四轮比例输出
【优点】无需测量距离，只需观察比例即可判断标定准确性
"""

from machine import Pin
import time
from motor import (
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    omni_drive_closed_loop,
    get_encoder_counts,
    stop_all,
    reset_wheel_pi,
    ENC_SCALE,
    MAX_SPEED_MPS
)

# ── 硬件 ──
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ── 参数 ──
ROTATION_SPEED = 0.3   # 旋转速度指令 (wz)
ROTATION_TIME = 8000   # 旋转时间 ms
DT = 0.02              # 控制周期 20ms


def wait_for_switch():
    """等待开关状态变化"""
    while switch2.value() == state2:
        time.sleep_ms(50)
    time.sleep_ms(200)  # 消抖


def main():
    print("=" * 60)
    print("  相对比例法编码器标定测试")
    print("  原地旋转 → 对比四轮脉冲增量")
    print("=" * 60)

    # 清空编码器历史数据
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    print("\n  按下开关 D9 开始旋转测试...")
    wait_for_switch()

    print("  开始原地旋转 ({:.1f}s)...".format(ROTATION_TIME / 1000))

    # 旋转测试
    reset_wheel_pi()
    start_time = time.ticks_ms()

    total_counts = [0, 0, 0, 0]
    while time.ticks_diff(time.ticks_ms(), start_time) < ROTATION_TIME:
        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]
        omni_drive_closed_loop(0, 0, ROTATION_SPEED, raw_speeds, DT)
        for i in range(4):
            total_counts[i] += raw_counts[i]
        time.sleep_ms(int(DT * 1000))

    stop_all()
    time.sleep_ms(500)  # 等待完全停止

    # 使用循环中累加的总脉冲数
    delta_rf, delta_lf, delta_lb, delta_rb = total_counts

    print("\n  旋转完成，脉冲增量:")
    print("  RF:{:6d}  LF:{:6d}  LB:{:6d}  RB:{:6d}".format(
        delta_rf, delta_lf, delta_lb, delta_rb))

    # 计算比例
    total = delta_rf + delta_lf + delta_lb + delta_rb
    if total == 0:
        print("\n  [错误] 无脉冲增量，请检查编码器连接")
        return

    ratio_rf = delta_rf / total
    ratio_lf = delta_lf / total
    ratio_lb = delta_lb / total
    ratio_rb = delta_rb / total

    ideal_ratio = 0.25

    print("\n  四轮比例分析:")
    print("  " + "-" * 55)
    print("  轮子 | 实际比例 | 理想比例 | 偏差   | 状态")
    print("  " + "-" * 55)

    for name, ratio in [("RF", ratio_rf), ("LF", ratio_lf),
                        ("LB", ratio_lb), ("RB", ratio_rb)]:
        diff = ratio - ideal_ratio
        diff_percent = diff / ideal_ratio
        status = "正常" if abs(diff_percent) < 0.03 else "偏差"
        print("  {:4s} | {:6.2%}  | {:6.2%}  | {:+5.2%} | {}".format(
            name, ratio, ideal_ratio, diff_percent, status))

    print("  " + "-" * 55)

    # 建议新的 ENC_SCALE
    print("\n  建议调整方案:")
    print("  当前 ENC_SCALE = {}".format(ENC_SCALE))

    # 计算建议值（基于比例修正）
    avg_scale = sum(ENC_SCALE) / 4
    new_scale = [
        int(avg_scale * ideal_ratio / ratio_rf),
        int(avg_scale * ideal_ratio / ratio_lf),
        int(avg_scale * ideal_ratio / ratio_lb),
        int(avg_scale * ideal_ratio / ratio_rb)
    ]

    print("  建议 ENC_SCALE = {}".format(new_scale))
    print("\n  说明：比例偏大的轮子需要减小 ENC_SCALE")
    print("        比例偏小的轮子需要增大 ENC_SCALE")

    # 误差评估
    max_diff = max(abs(r - ideal_ratio) for r in [ratio_rf, ratio_lf, ratio_lb, ratio_rb])
    if max_diff < 0.015:
        print("\n  评估：四轮一致性良好，编码器标定基本准确")
    elif max_diff < 0.03:
        print("\n  评估：存在轻微偏差，可考虑微调 ENC_SCALE")
    else:
        print("\n  评估：偏差较大，建议重新标定 ENC_SCALE")


try:
    main()
except KeyboardInterrupt:
    print("\n  用户中止")
finally:
    stop_all()
