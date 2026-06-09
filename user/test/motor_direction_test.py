"""
motor_direction_test.py - 逐个测试电机方向
【功能】
  依次让每个电机正转，让你观察并记录实际运动方向
【使用】
  运行脚本，观察每个电机转动时小车的移动方向
  记录结果，用于修正运动学模型
"""

import time
from machine import *
from smartcar import *
from seekfree import *

time.sleep_ms(100)

print("=" * 60)
print("  Motor Direction Test")
print("=" * 60)
print("  REAL TYPE    : " + BOARD_TYPE)
print("  BOARD VERSION: " + BOARD_VERSION)
print("")

LED_PIN     = 'C4'
SWITCH2_PIN = 'D9'

led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ============================================================
#  电机引脚定义（TB6612）
# ============================================================

pwm_rf = PWM("C26", 15000, duty_u16=0)
pwm_lf = PWM("C24", 15000, duty_u16=0)
pwm_lb = PWM("C20", 15000, duty_u16=0)
pwm_rb = PWM("B26", 15000, duty_u16=0)

pin_d6  = Pin("D6",  Pin.OUT, value=0)
pin_d7  = Pin("D7",  Pin.OUT, value=0)
pin_d4  = Pin("D4",  Pin.OUT, value=0)
pin_d5  = Pin("D5",  Pin.OUT, value=0)
pin_c30 = Pin("C30", Pin.OUT, value=0)
pin_c31 = Pin("C31", Pin.OUT, value=0)
pin_c28 = Pin("C28", Pin.OUT, value=0)
pin_c29 = Pin("C29", Pin.OUT, value=0)

# 电机元组 (PWM, 方向A, 方向B)
MOTOR_RF = (pwm_rf, pin_d6,  pin_d7)
MOTOR_LF = (pwm_lf, pin_d4,  pin_d5)
MOTOR_LB = (pwm_lb, pin_c30, pin_c31)
MOTOR_RB = (pwm_rb, pin_c28, pin_c29)

MOTOR_LIST = [MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB]
MOTOR_NAMES = ['RF (右前)', 'LF (左前)', 'LB (左后)', 'RB (右后)']

# ============================================================
#  电机控制函数
# ============================================================

def set_motor(motor, duty_u16):
    pwm, dir_a, dir_b = motor
    if duty_u16 > 0:
        dir_a.value(0)
        dir_b.value(1)
        pwm.duty_u16(int(duty_u16))
    elif duty_u16 < 0:
        dir_a.value(1)
        dir_b.value(0)
        pwm.duty_u16(int(-duty_u16))
    else:
        dir_a.value(0)
        dir_b.value(0)
        pwm.duty_u16(0)

def stop_all():
    for m in MOTOR_LIST:
        set_motor(m, 0)

# ============================================================
#  测试流程
# ============================================================

TEST_DUTY = 20000  # 20000/65535 = 30%

print("-" * 60)
print("  测试方法：")
print("  1. 将小车放在地面，轮子可以自由转动")
print("  2. 每个电机会正转 2 秒")
print("  3. 观察并记录小车的移动方向")
print("-" * 60)
print("")
print("  按 SWITCH2 可跳过当前电机")
print("")

for i in range(4):
    if switch2.value() != state2:
        break
    
    motor = MOTOR_LIST[i]
    name = MOTOR_NAMES[i]
    
    print("=" * 60)
    print("  Motor {}: {}".format(i+1, name))
    print("=" * 60)
    
    # 正转
    print("  正转 +{} duty，持续 2 秒...".format(TEST_DUTY))
    set_motor(motor, TEST_DUTY)
    
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < 2000:
        if switch2.value() != state2:
            break
        time.sleep_ms(100)
    
    set_motor(motor, 0)
    time.sleep_ms(500)
    print("  正转结束。")
    
    # 反转
    print("  反转 -{} duty，持续 2 秒...".format(TEST_DUTY))
    set_motor(motor, -TEST_DUTY)
    
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < 2000:
        if switch2.value() != state2:
            break
        time.sleep_ms(100)
    
    set_motor(motor, 0)
    time.sleep_ms(500)
    print("  反转结束。")
    print("")

# ============================================================
#  完成
# ============================================================

stop_all()
led.off()

print("=" * 60)
print("  测试完成")
print("=" * 60)
print("")
print("  请观察并记录每个电机正转时的实际方向：")
print("")
print("  | 电机位置 | 正转时小车移动方向 |")
print("  |----------|-------------------|")
print("  | RF (右前) |                   |")
print("  | LF (左前) |                   |")
print("  | LB (左后) |                   |")
print("  | RB (右后) |                   |")
print("")
print("  方向代码：前/后/左/右/旋转/无反应")
print("=" * 60)
print("")
