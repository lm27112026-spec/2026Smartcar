"""
calibrate.py — MAX_SPEED_MPS 标定
【目的】满占空比驱动各轮，从编码器实测最高速度，反推 PWM_PER_MPS。
【方法】逐步增大 PWM，记录稳态速度，取稳定最大值。
【安全】拨码开关 D9 随时中止。⚠ 全速运行，请架空车轮！
"""
from machine import Pin
import time
from motor import (
    set_motor, stop_all, MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB,
    get_encoder_counts, get_encoder_speeds,
    ENC_SCALE, enc_ticker,
)

switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

MOTORS = [
    ("LF", MOTOR_LF, 1),   # 编码器索引: 1=LF
    ("RF", MOTOR_RF, 0),   # 编码器索引: 0=RF
    ("LB", MOTOR_LB, 2),   # 编码器索引: 2=LB
    ("RB", MOTOR_RB, 3),   # 编码器索引: 3=RB
]

PWM_STEPS = [5000, 10000, 15000, 20000, 25000, 30000, 35000, 40000, 45000, 50000]
DT = 0.1
HOLD = 1000  # 每个 PWM 挡位保持 1s


def measure_speed(motor, pwm, enc_idx, duration_ms=1000):
    """驱动单个电机指定 PWM，返回稳态速度 m/s"""
    _ = get_encoder_counts()  # prime
    set_motor(motor, pwm)
    time.sleep_ms(500)  # 等待稳定

    speeds = []
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        spd = get_encoder_speeds(DT)
        speeds.append(abs(spd[enc_idx]))
        time.sleep_ms(int(DT * 1000))

    set_motor(motor, 0)
    time.sleep_ms(200)

    if speeds:
        return sum(speeds) / len(speeds)
    return 0.0


def main():
    print("=" * 60)
    print("  MAX_SPEED_MPS Calibration")
    print("  逐步增大 PWM 测各轮稳态速度")
    print("  ⚠ 架空车轮！Switch D9 to stop")
    print("=" * 60)

    stop_all()

    # 暂停 ticker 防止偷脉冲
    enc_ticker.stop()
    for _ in range(5):
        get_encoder_counts()
        time.sleep_ms(10)

    for name, motor, idx in MOTORS:
        if switch2.value() != state2:
            break

        enc_scale = ENC_SCALE[idx]
        print("\n  ── 轮子 {} (ENC_SCALE={}) ──".format(name, enc_scale))
        print("  {:>6s} | {:>10s} | {:>8s}".format(
            "PWM", "速度 m/s", "PWM/mps"))
        print("  " + "-" * 35)

        max_speed = 0.0

        for pwm in PWM_STEPS:
            if switch2.value() != state2:
                break

            speed = measure_speed(motor, pwm, idx)
            pwm_per_mps = pwm / speed if speed > 0.001 else 0

            print("  {:6d} | {:10.4f} | {:8.1f}".format(
                pwm, speed, pwm_per_mps))

            if speed > max_speed:
                max_speed = speed

            if speed > 0.8 and pwm > 30000:
                # 已达高速，可以提前结束
                pass

        print("\n  >>> {} 最高速: {:.3f} m/s".format(name, max_speed))
        print("  >>> 建议 MAX_SPEED_MPS = {:.3f}".format(max_speed))
        print("  >>> 建议 PWM_PER_MPS  = {:.0f}".format(
            50000 / max_speed if max_speed > 0 else 0))

    enc_ticker.start(10)
    print("\n" + "=" * 60)
    print("  标定完成。将上述值填入 motor.py")
    print("=" * 60)
    stop_all()


try:
    main()
except KeyboardInterrupt:
    print("\n  用户中止。")
finally:
    stop_all()
    try:
        enc_ticker.start(10)
    except:
        pass
