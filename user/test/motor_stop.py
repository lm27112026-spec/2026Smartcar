"""
motor_stop.py - 紧急停止所有电机
【功能】
  强制停止所有电机，清除 PWM 信号
【使用】
  如果电机失控，立即运行此脚本
"""

import time
from machine import *
from seekfree import *

time.sleep_ms(100)

print("Stopping all motors...")

# 停止所有可能的电机通道
channels = [
    MOTOR_CONTROLLER.PWM_C30_DIR_C31,
    MOTOR_CONTROLLER.PWM_C28_DIR_C29,
    MOTOR_CONTROLLER.PWM_D4_DIR_D5,
    MOTOR_CONTROLLER.PWM_D6_DIR_D7,
]

for ch in channels:
    try:
        m = MOTOR_CONTROLLER(ch, 15000, duty=0)
        m.duty(0)
        print("  Stopped: {}".format(ch))
    except:
        pass

# 也停止通过 smartcar PWM 直接控制的电机
try:
    from motor import stop_all
    stop_all()
    print("  motor.stop_all() called")
except:
    pass

print("All motors stopped.")
print("If RF motor is still running, hardware issue suspected.")
