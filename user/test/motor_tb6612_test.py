"""
motor_tb6612_test.py - TB6612 驱动板引脚映射诊断
【功能】
  TB6612 使用 PWM + 2个方向引脚 (IN1, IN2) 控制：
    IN1=0, IN2=1 → 正转
    IN1=1, IN2=0 → 反转
    IN1=0, IN2=0 → 停止
  本脚本逐一尝试所有可能的引脚组合，找到正确的映射。
【使用】
  运行脚本，观察哪个组合能让电机转动
"""

import time
from machine import *
from smartcar import *
from seekfree import *

time.sleep_ms(100)

print("=" * 60)
print("  TB6612 Pin Mapping Diagnostic")
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
#  TB6612 引脚组合测试
#  每个电机需要：1个PWM引脚 + 2个方向引脚
# ============================================================

# 可能的 PWM 引脚
pwm_pins = ['B26', 'C20', 'C24', 'C26']

# 可能的方向引脚对
dir_pairs = [
    ('C28', 'C29'),
    ('C30', 'C31'),
    ('D4',  'D5'),
    ('D6',  'D7'),
]

print("  测试所有 PWM + DIR 组合")
print("  每个组合测试 1 秒，观察哪个电机在转")
print("  按 SWITCH2 跳过当前组合")
print("")

tested = 0
found = 0

for pwm_name in pwm_pins:
    if switch2.value() != state2:
        break
    
    for dir_a, dir_b in dir_pairs:
        if switch2.value() != state2:
            break
        
        tested += 1
        
        # 创建 PWM 和方向引脚
        try:
            pwm = PWM(pwm_name, 15000, duty_u16=0)
            pin_a = Pin(dir_a, Pin.OUT, value=0)
            pin_b = Pin(dir_b, Pin.OUT, value=0)
        except Exception as e:
            continue
        
        combo = "PWM={} DIR={}/{}".format(pwm_name, dir_a, dir_b)
        print("  [{:>2d}] {} ...".format(tested, combo), end="")
        
        # 正转测试
        pin_a.value(0)
        pin_b.value(1)
        pwm.duty_u16(15000)  # 23% duty
        
        time.sleep_ms(800)
        
        # 停止
        pin_a.value(0)
        pin_b.value(0)
        pwm.duty_u16(0)
        time.sleep_ms(200)
        
        # 反转测试
        pin_a.value(1)
        pin_b.value(0)
        pwm.duty_u16(15000)
        
        time.sleep_ms(800)
        
        # 停止
        pin_a.value(0)
        pin_b.value(0)
        pwm.duty_u16(0)
        time.sleep_ms(300)
        
        print(" done")
        
        # 清理
        pwm.deinit()
        pin_a.value(0)
        pin_b.value(0)

# ============================================================
#  完成
# ============================================================

led.off()

print("")
print("=" * 60)
print("  Test Complete")
print("=" * 60)
print("  Tested {} combinations".format(tested))
print("")
print("  如果某个组合能让电机转动，")
print("  请记录对应的引脚，更新 motor.py 中的电机定义。")
print("=" * 60)
print("")
