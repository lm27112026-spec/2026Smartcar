"""
motor_diag.py - 电机硬件诊断（直接使用 MOTOR_CONTROLLER）
【目的】
  绕过 motor.py 抽象层，直接用 seekfree.MOTOR_CONTROLLER 测试硬件
  用于排查是 motor.py 引脚定义问题还是硬件供电问题
【使用】
  运行此脚本，观察电机是否转动
  ⚠ 轮子需悬空！
"""

import gc, time
from machine import *
from smartcar import *
from seekfree import *

time.sleep_ms(100)

print("=" * 60)
print("  Motor Hardware Diagnostic")
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
#  根据开发板类型选择电机通道
# ============================================================

MOTOR_CHANNEL1 = None
MOTOR_CHANNEL2 = None
MOTOR_CHANNEL3 = None
MOTOR_CHANNEL4 = None

if BOARD_TYPE == 'RT1021_144P_BTB':
    MOTOR_CHANNEL1 = MOTOR_CONTROLLER.PWM_C30_DIR_C31
    MOTOR_CHANNEL2 = MOTOR_CONTROLLER.PWM_C28_DIR_C29
    MOTOR_CHANNEL3 = MOTOR_CONTROLLER.PWM_D4_DIR_D5
    MOTOR_CHANNEL4 = MOTOR_CONTROLLER.PWM_D6_DIR_D7
elif BOARD_TYPE == 'RT1021_144P_2P54':
    MOTOR_CHANNEL1 = MOTOR_CONTROLLER.PWM_C30_DIR_C31
    MOTOR_CHANNEL2 = MOTOR_CONTROLLER.PWM_C28_DIR_C29
    MOTOR_CHANNEL3 = MOTOR_CONTROLLER.PWM_D4_DIR_D5
    MOTOR_CHANNEL4 = MOTOR_CONTROLLER.PWM_D6_DIR_D7

print("  Motor Channels:")
print("    CH1: {}".format(MOTOR_CHANNEL1))
print("    CH2: {}".format(MOTOR_CHANNEL2))
print("    CH3: {}".format(MOTOR_CHANNEL3))
print("    CH4: {}".format(MOTOR_CHANNEL4))
print("")

# ============================================================
#  显示帮助信息
# ============================================================

MOTOR_CONTROLLER.help()
print("")

# ============================================================
#  初始化电机（使用 MOTOR_CONTROLLER 类）
# ============================================================

print("-" * 60)
print("  Initializing motors with MOTOR_CONTROLLER class...")
print("-" * 60)

try:
    motor_1 = MOTOR_CONTROLLER(MOTOR_CHANNEL1, 15000, duty=0, invert=False)
    motor_2 = MOTOR_CONTROLLER(MOTOR_CHANNEL2, 15000, duty=0, invert=True)
    motor_3 = MOTOR_CONTROLLER(MOTOR_CHANNEL3, 15000, duty=0, invert=False)
    motor_4 = MOTOR_CONTROLLER(MOTOR_CHANNEL4, 15000, duty=0, invert=True)
    
    motor_1.info()
    motor_2.info()
    motor_3.info()
    motor_4.info()
    
    print("  >> All 4 motors initialized successfully.")
except Exception as e:
    print("  >> ERROR: Motor init failed: {}".format(e))
    led.off()
    raise SystemExit(1)

# ============================================================
#  测试 1：逐个电机正反转
# ============================================================

print("")
print("-" * 60)
print("  [Test 1] Individual Motor Forward/Reverse")
print("-" * 60)

motors = [motor_1, motor_2, motor_3, motor_4]
names = ['CH1', 'CH2', 'CH3', 'CH4']
duty_levels = [3000, 5000, 8000]  # 不同占空比

for i, motor in enumerate(motors):
    if switch2.value() != state2:
        break
    
    print("  Motor {} ({}):".format(names[i], i+1))
    
    for duty in duty_levels:
        if switch2.value() != state2:
            break
        
        # 正转
        print("    Forward +{:<5d}...".format(duty), end="")
        motor.duty(duty)
        time.sleep_ms(800)
        motor.duty(0)
        time.sleep_ms(200)
        print("done")
        
        # 反转
        print("    Reverse -{:<5d}...".format(duty), end="")
        motor.duty(-duty)
        time.sleep_ms(800)
        motor.duty(0)
        time.sleep_ms(200)
        print("done")
    
    print("")

# ============================================================
#  测试 2：所有电机同时转动
# ============================================================

print("-" * 60)
print("  [Test 2] All Motors Simultaneous")
print("-" * 60)

print("  All forward +5000 for 2 seconds...", end="")
for m in motors:
    m.duty(5000)
time.sleep_ms(2000)
for m in motors:
    m.duty(0)
time.sleep_ms(300)
print("done")

print("  All reverse -5000 for 2 seconds...", end="")
for m in motors:
    m.duty(-5000)
time.sleep_ms(2000)
for m in motors:
    m.duty(0)
time.sleep_ms(300)
print("done")

# ============================================================
#  测试 3：PWM 线性度（电机 1）
# ============================================================

print("")
print("-" * 60)
print("  [Test 3] PWM Linearity (Motor CH1)")
print("-" * 60)

pwm_values = [0, 2000, 4000, 6000, 8000, 10000, 0]
for duty in pwm_values:
    if switch2.value() != state2:
        break
    motor_1.duty(duty)
    time.sleep_ms(500)
    print("    duty={:>6d} -> motor running".format(duty))
motor_1.duty(0)

# ============================================================
#  清理
# ============================================================

for m in motors:
    m.duty(0)

led.off()

print("")
print("=" * 60)
print("  Diagnostic Complete")
print("=" * 60)
print("  If motors did NOT move:")
print("    1. Check driver board power supply (external power)")
print("    2. Check driver board power LED")
print("    3. Check motor wiring (PWM, DIR, GND)")
print("    4. Check motor cable connections")
print("=" * 60)
print("")
