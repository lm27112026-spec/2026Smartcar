"""
motor_test.py — 4 电机正反转逐个测试
【功能】依次驱动每个电机：正转 2s → 停 0.5s → 反转 2s → 停 0.5s
【用法】将小车架空（轮子悬空），运行此脚本，观察每个轮子是否正常正反转
【安全】拨码开关 D9 随时中止
"""
import time
from machine import *
from smartcar import *
from seekfree import *

time.sleep_ms(100)

print("=" * 60)
print("  Motor Test — 4 电机正反转逐个测试")
print("=" * 60)
print("  REAL TYPE    : " + BOARD_TYPE)
print("  BOARD VERSION: " + BOARD_VERSION)
print("")

# ── 硬件 ────────────────────────────────────────────────────
SWITCH2_PIN = 'D9'
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ── 电机引脚定义（TB6612，与 motor.py 完全一致）─────────────
# 电机位置     PWM脚      方向A      方向B      电机编号
# 左前 (LF)    C24        D4         D5         M3
# 右前 (RF)    C26        D6         D7         M4
# 左后 (LB)    C20        C30        C31        M2
# 右后 (RB)    B26        C28        C29        M1

pwm_lf = PWM("C24", 15000, duty_u16=0)
pwm_rf = PWM("C26", 15000, duty_u16=0)
pwm_lb = PWM("C20", 15000, duty_u16=0)
pwm_rb = PWM("B26", 15000, duty_u16=0)

pin_d4  = Pin("D4",  Pin.OUT, value=0)
pin_d5  = Pin("D5",  Pin.OUT, value=0)
pin_d6  = Pin("D6",  Pin.OUT, value=0)
pin_d7  = Pin("D7",  Pin.OUT, value=0)
pin_c30 = Pin("C30", Pin.OUT, value=0)
pin_c31 = Pin("C31", Pin.OUT, value=0)
pin_c28 = Pin("C28", Pin.OUT, value=0)
pin_c29 = Pin("C29", Pin.OUT, value=0)

# 电机元组 (PWM, 方向A, 方向B)
MOTOR_LF = (pwm_lf, pin_d4,  pin_d5)
MOTOR_RF = (pwm_rf, pin_d6,  pin_d7)
MOTOR_LB = (pwm_lb, pin_c30, pin_c31)
MOTOR_RB = (pwm_rb, pin_c28, pin_c29)

MOTOR_LIST  = [MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB]
MOTOR_NAMES = ['LF (左前 M3)', 'RF (右前 M4)', 'LB (左后 M2)', 'RB (右后 M1)']

# ── 电机控制函数 ────────────────────────────────────────────
def set_motor(motor, duty_val):
    """duty_val > 0 正转, < 0 反转, == 0 停止"""
    pwm, dir_a, dir_b = motor
    if duty_val > 0:
        dir_a.value(0)
        dir_b.value(1)
        pwm.duty_u16(int(duty_val))
    elif duty_val < 0:
        dir_a.value(1)
        dir_b.value(0)
        pwm.duty_u16(int(-duty_val))
    else:
        dir_a.value(0)
        dir_b.value(0)
        pwm.duty_u16(0)

def stop_all():
    for m in MOTOR_LIST:
        set_motor(m, 0)

# ── 测试参数 ────────────────────────────────────────────────
TEST_DUTY = 20000       # PWM 占空比 (~30%)
FWD_TIME  = 2000        # 正转时长 ms
REV_TIME  = 2000        # 反转时长 ms
PAUSE     = 500         # 段间暂停 ms

print("-" * 60)
print("  测试参数: PWM={} (~{:.0f}%)".format(TEST_DUTY, TEST_DUTY / 65535 * 100))
print("  请将小车架空，确保轮子可以自由转动")
print("  拨动 SW2 (D9) 可随时中止")
print("-" * 60)
print("")

# ── 测试流程 ────────────────────────────────────────────────
for i in range(4):
    if switch2.value() != state2:
        print("\n  [SW2] 用户中止。")
        break

    motor = MOTOR_LIST[i]
    name  = MOTOR_NAMES[i]

    print("=" * 60)
    print("  [{}] {}".format(i + 1, name))
    print("=" * 60)

    # ── 正转 ──
    print("  正转 +{} duty，持续 {}s...".format(TEST_DUTY, FWD_TIME / 1000))
    set_motor(motor, TEST_DUTY)
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < FWD_TIME:
        if switch2.value() != state2:
            break
        time.sleep_ms(100)
    set_motor(motor, 0)
    time.sleep_ms(PAUSE)
    print("  -> 正转结束。")

    # ── 反转 ──
    print("  反转 -{} duty，持续 {}s...".format(TEST_DUTY, REV_TIME / 1000))
    set_motor(motor, -TEST_DUTY)
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < REV_TIME:
        if switch2.value() != state2:
            break
        time.sleep_ms(100)
    set_motor(motor, 0)
    time.sleep_ms(PAUSE)
    print("  -> 反转结束。")
    print("")

# ── 完成 ────────────────────────────────────────────────────
stop_all()

print("=" * 60)
print("  测试完成！")
print("=" * 60)
print("")
print("  确认清单：")
print("  [ ] LF (左前 M3) — 正转 √  反转 √")
print("  [ ] RF (右前 M4) — 正转 √  反转 √")
print("  [ ] LB (左后 M2) — 正转 √  反转 √")
print("  [ ] RB (右后 M1) — 正转 √  反转 √")
print("")
print("=" * 60)
