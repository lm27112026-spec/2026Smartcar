"""
min_pwm_calibrate.py — 电机死区扫描（MIN_PWM 标定）
【方法】二分搜索 × 3轮取平均，消除随机误差
【安全】架空车轮！
"""
from machine import Pin
import time, gc
from motor import (
    set_motor, stop_all, MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB,
    get_encoder_counts, get_encoder_speeds,
)

switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

MOTORS = [
    ("LF", MOTOR_LF, 1),
    ("RF", MOTOR_RF, 0),
    ("LB", MOTOR_LB, 2),
    ("RB", MOTOR_RB, 3),
]

LOW   = 3000
HIGH  = 15000
DT    = 0.05
PASSES = 3  # 每方向重复次数


def motor_moving(motor, pwm, sign, enc_idx):
    set_motor(motor, pwm * sign)
    time.sleep_ms(200)
    spd = get_encoder_speeds(DT)
    set_motor(motor, 0)
    time.sleep_ms(50)
    return abs(spd[enc_idx]) > 0.001


def find_threshold(motor, enc_idx, sign, label):
    lo, hi = LOW, HIGH
    print("    {} ".format(label), end="")
    while hi - lo > 500:
        mid = (lo + hi) // 2
        if motor_moving(motor, mid, sign, enc_idx):
            hi = mid
            print(".", end="")
        else:
            lo = mid
            print("_", end="")
        if switch2.value() != state2:
            return 0
        gc.collect()
    return hi


def main():
    print("=" * 60)
    print("  MIN_PWM Scan ({} passes, binary search)".format(PASSES))
    print("  Range: {}~{}".format(LOW, HIGH))
    print("=" * 60)

    stop_all()
    time.sleep_ms(50)
    for _ in range(5):
        get_encoder_counts()
        time.sleep_ms(10)

    # 收集多轮数据: min_pwm[wheel][dir] = [val1, val2, val3]
    all_data = [[[], []] for _ in range(4)]

    for p in range(PASSES):
        if switch2.value() != state2:
            break
        print("\n  === Pass {}/{} ===".format(p+1, PASSES))
        for mi, (name, motor, idx) in enumerate(MOTORS):
            if switch2.value() != state2:
                break
            print("  {}:".format(name), end="")
            pos = find_threshold(motor, idx, 1, "+")
            neg = find_threshold(motor, idx, -1, "-")
            if pos > 0: all_data[mi][0].append(pos)
            if neg > 0: all_data[mi][1].append(neg)
            print(" => +{} -{}".format(pos, neg))

    # 每轮取正反转平均，再取各轮中值
    results = []
    print("\n" + "=" * 60)
    print("  Final Results (avg of {} passes):".format(PASSES))
    for mi, (name, _, _) in enumerate(MOTORS):
        pos_vals = all_data[mi][0]
        neg_vals = all_data[mi][1]
        if pos_vals and neg_vals:
            pos_avg = sum(pos_vals) // len(pos_vals)
            neg_avg = sum(neg_vals) // len(neg_vals)
            avg = (pos_avg + neg_avg) // 2
        else:
            pos_avg = neg_avg = avg = 0
        print("  {}: +{}  -{}  => {}".format(name, pos_avg, neg_avg, avg))
        results.append(avg)

    print("")
    print("  填入 motor.py:")
    print("  MIN_PWM = {}".format(results))
    print("")
    print("  或使用统一值（保守）：")
    print("  MIN_PWM_POS = {}".format(max(results)))
    print("  MIN_PWM_NEG = {}".format(max(results)))
    print("=" * 60)

try:
    main()
except KeyboardInterrupt:
    print("\n  中止")
except Exception as e:
    print("\n  错误: {}".format(e))
finally:
    stop_all()
