"""
motor_pwm_test.py - 测试双PWM模式电机通道
【功能】
  测试 PWM_C30_PWM_C31 等双PWM通道是否能驱动电机
  （适用于 HIP4082 等双PWM驱动芯片）
【使用】
  运行此脚本，观察电机是否转动
"""

import time
from machine import *
from seekfree import *

time.sleep_ms(100)

print("=" * 60)
print("  Motor Dual-PWM Channel Test")
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
#  定义两组通道
# ============================================================

# DRV8701 模式 (PWM + DIR)
drv_channels = [
    ("DRV: C30_DIR_C31", MOTOR_CONTROLLER.PWM_C30_DIR_C31),
    ("DRV: C28_DIR_C29", MOTOR_CONTROLLER.PWM_C28_DIR_C29),
    ("DRV: D4_DIR_D5",   MOTOR_CONTROLLER.PWM_D4_DIR_D5),
    ("DRV: D6_DIR_D7",   MOTOR_CONTROLLER.PWM_D6_DIR_D7),
]

# HIP4082 模式 (双PWM)
hip_channels = [
    ("HIP: C30_PWM_C31", MOTOR_CONTROLLER.PWM_C30_PWM_C31),
    ("HIP: C28_PWM_C29", MOTOR_CONTROLLER.PWM_C28_PWM_C29),
    ("HIP: D4_PWM_D5",   MOTOR_CONTROLLER.PWM_D4_PWM_D5),
    ("HIP: D6_PWM_D7",   MOTOR_CONTROLLER.PWM_D6_PWM_D7),
]

# ============================================================
#  测试函数
# ============================================================

def test_channel(name, ch, duty=5000, duration_ms=2000):
    """测试单个通道"""
    if switch2.value() != state2:
        return False
    
    print("  Testing {}...".format(name), end="")
    try:
        m = MOTOR_CONTROLLER(ch, 15000, duty=0)
        m.duty(duty)
        time.sleep_ms(duration_ms)
        m.duty(0)
        time.sleep_ms(300)
        print(" done (did motor move?)")
        return True
    except Exception as e:
        print(" ERROR: {}".format(e))
        return False

# ============================================================
#  主测试流程
# ============================================================

print("-" * 60)
print("  Phase 1: Test DRV (PWM+DIR) channels")
print("-" * 60)
print("  观察并记录哪个电机在转")
print("")

for name, ch in drv_channels:
    if switch2.value() != state2:
        break
    test_channel(name, ch)
    time.sleep_ms(500)

if switch2.value() != state2:
    led.off()
    raise SystemExit(0)

print("")
print("-" * 60)
print("  Phase 2: Test HIP (Dual-PWM) channels")
print("-" * 60)
print("  观察并记录哪个电机在转")
print("")

for name, ch in hip_channels:
    if switch2.value() != state2:
        break
    test_channel(name, ch)
    time.sleep_ms(500)

# ============================================================
#  完成
# ============================================================

led.off()

print("")
print("=" * 60)
print("  Test Complete")
print("=" * 60)
print("  如果 HIP (双PWM) 通道能让电机转动，")
print("  说明你的驱动板是 HIP4082 类型，")
print("  需要把 motor_test.py 的通道改为 PWM_xx_PWM_xx 格式。")
print("=" * 60)
print("")
