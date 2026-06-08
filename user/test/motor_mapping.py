"""
motor_mapping.py - 电机通道映射诊断
【功能】
  逐个激活 4 个 MOTOR_CHANNEL，让你确认哪个通道对应哪个物理电机
【使用】
  运行脚本，观察哪个电机在转，记录对应关系
  按 SWITCH2 停止当前通道，进入下一个
"""

import gc, time
from machine import *
from smartcar import *
from seekfree import *

time.sleep_ms(100)

print("=" * 60)
print("  Motor Channel Mapping Diagnostic")
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
#  所有可用的电机通道
# ============================================================

channels = [
    ("PWM_C30_DIR_C31", MOTOR_CONTROLLER.PWM_C30_DIR_C31),
    ("PWM_C28_DIR_C29", MOTOR_CONTROLLER.PWM_C28_DIR_C29),
    ("PWM_D4_DIR_D5",   MOTOR_CONTROLLER.PWM_D4_DIR_D5),
    ("PWM_D6_DIR_D7",   MOTOR_CONTROLLER.PWM_D6_DIR_D7),
]

# ============================================================
#  测试每个通道
# ============================================================

print("  对于每个通道，电机会转动 3 秒。")
print("  观察并记录哪个物理电机在转。")
print("  按 SWITCH2 跳过当前通道。")
print("")

for idx, (name, ch) in enumerate(channels):
    if switch2.value() != state2:
        break
    
    print("-" * 60)
    print("  Channel {}: {}".format(idx + 1, name))
    print("-" * 60)
    
    try:
        m = MOTOR_CONTROLLER(ch, 15000, duty=0)
        
        # 正转 3 秒
        print("    Forward +5000 for 3s...", end="")
        m.duty(5000)
        
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 3000:
            if switch2.value() != state2:
                break
            time.sleep_ms(100)
        
        m.duty(0)
        print(" done")
        
        # 反转 3 秒
        print("    Reverse -5000 for 3s...", end="")
        m.duty(-5000)
        
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < 3000:
            if switch2.value() != state2:
                break
            time.sleep_ms(100)
        
        m.duty(0)
        print(" done")
        
        # 等待用户确认
        print("    ( pause 2s - 请记录哪个电机在转 )")
        time.sleep_ms(2000)
        
    except Exception as e:
        print("    ERROR: {}".format(e))
    
    print("")

# ============================================================
#  完成
# ============================================================

led.off()

print("=" * 60)
print("  Mapping Complete")
print("=" * 60)
print("  请根据观察结果填写映射表：")
print("")
print("  Channel            | 物理位置 (RF/LF/LB/RB)")
print("  -------------------|------------------------")
print("  PWM_C30_DIR_C31    | ______")
print("  PWM_C28_DIR_C29    | ______")
print("  PWM_D4_DIR_D5      | ______")
print("  PWM_D6_DIR_D7      | ______")
print("")
print("  然后更新 motor_test.py 中的 MOTOR_CH 定义。")
print("=" * 60)
print("")
