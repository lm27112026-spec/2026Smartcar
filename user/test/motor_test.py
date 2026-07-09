"""
motor_test.py — 6 方向全向运动开环测试
【功能】逐一测试 6 个基本运动方向（前进/后退/左移/右移/左自转/右自转）
        开环 PWM 驱动，无需编码器，直接观察小车运动方向
【用法】将小车放在地面，运行脚本观察每段运动方向是否正确
【安全】拨码开关 D9 随时中止
"""
import time
from machine import *
from smartcar import *
from seekfree import *

time.sleep_ms(100)

print("=" * 60)
print("  6-Direction Omni Motion Test")
print("=" * 60)
print("  REAL TYPE    : " + BOARD_TYPE)
print("  BOARD VERSION: " + BOARD_VERSION)
print("")

# ── 硬件 ────────────────────────────────────────────────────
SWITCH2_PIN = 'D9'
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ── 电机引脚定义（TB6612）───────────────────────────────────
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

MOTOR_LF = (pwm_lf, pin_d4,  pin_d5)
MOTOR_RF = (pwm_rf, pin_d6,  pin_d7)
MOTOR_LB = (pwm_lb, pin_c30, pin_c31)
MOTOR_RB = (pwm_rb, pin_c28, pin_c29)

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
    for m in (MOTOR_LF, MOTOR_RF, MOTOR_LB, MOTOR_RB):
        set_motor(m, 0)

# ── 全向运动学（开环）───────────────────────────────────────
MAX_PWM = 50000

def omni_kinematics(vx, vy, wz):
    """逆运动学：底盘速度 → 四轮归一化速度比"""
    w_rf =  vx - vy + wz
    w_lf = -vx - vy + wz
    w_lb = -vx + vy + wz
    w_rb =  vx + vy + wz
    return [w_lf, w_rf, w_lb, w_rb]

def omni_drive(vx, vy, wz, max_pwm=MAX_PWM):
    """开环全向驱动：直接根据 vx,vy,wz 计算 PWM 输出"""
    speeds = omni_kinematics(vx, vy, wz)
    max_speed = max(abs(s) for s in speeds)
    scale = 1.0
    if max_speed > 1.0:
        scale = 1.0 / max_speed
    pwm_vals = [int(s * scale * max_pwm) for s in speeds]
    set_motor(MOTOR_LF, pwm_vals[0])
    set_motor(MOTOR_RF, pwm_vals[1])
    set_motor(MOTOR_LB, pwm_vals[2])
    set_motor(MOTOR_RB, pwm_vals[3])

# ── 测试参数 ────────────────────────────────────────────────
SPEED  = 0.30       # 平移速度比例 (0~1)
TURN   = 0.3       # 旋转速度比例 (0~1)
MAX_PWM_RUN = 50000 # 运行 PWM 上限
RUN_TIME = 3000     # 每段运行 ms
PAUSE    = 1000     # 段间暂停 ms

# ── 六向测试序列 ────────────────────────────────────────────
SEQUENCE = [
    ("前进 ↑",     SPEED,  0,      0,      "四轮同向转动，车体向前"),
    ("后退 ↓",    -SPEED,  0,      0,      "四轮同向转动，车体向后"),
    ("右移 →",     0,       SPEED,  0,      "对角轮对转，车体右移"),
    ("左移 ←",     0,      -SPEED,  0,      "对角轮对转，车体左移"),
    ("左自转 ↺",   0,       0,      TURN,   "四轮同向，车体逆时针转"),
    ("右自转 ↻",   0,       0,     -TURN,   "四轮同向，车体顺时针转"),
]

LOOP = True  # True=循环, False=跑一轮

print("-" * 60)
print("  测试参数: SPEED={:.2f}  TURN={:.2f}  PWM_MAX={}".format(SPEED, TURN, MAX_PWM_RUN))
print("  每段运行 {}s，间隔 {}s".format(RUN_TIME / 1000, PAUSE / 1000))
print("  拨动 SW2 (D9) 可随时中止")
print("-" * 60)
print("")

# ── 测试主循环 ──────────────────────────────────────────────
stop_all()
round_num = 0

while True:
    round_num += 1
    print("=" * 60)
    print("  Round {}".format(round_num))
    print("=" * 60)

    for name, vx, vy, wz, desc in SEQUENCE:
        if switch2.value() != state2:
            print("\n  [SW2] 用户中止。")
            stop_all()
            raise SystemExit

        print("")
        print("  ── [{}] {} ──".format(name, desc))
        print("  vx={:+.2f}  vy={:+.2f}  wz={:+.2f}".format(vx, vy, wz))

        # 驱动电机
        omni_drive(vx, vy, wz, MAX_PWM_RUN)
        start = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), start) < RUN_TIME:
            if switch2.value() != state2:
                print("  [SW2] 中止。")
                stop_all()
                raise SystemExit
            time.sleep_ms(100)

        # 停车 + 暂停
        stop_all()
        print("  -> 停车，暂停 {}s...".format(PAUSE / 1000))

        pause_start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), pause_start) < PAUSE:
            if switch2.value() != state2:
                print("  [SW2] 中止。")
                stop_all()
                raise SystemExit
            time.sleep_ms(100)

    print("")
    if not LOOP:
        break

print("=" * 60)
print("  测试完成！")
print("=" * 60)
stop_all()
