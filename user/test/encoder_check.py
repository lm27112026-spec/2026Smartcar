"""
encoder_check.py - LF 电机正转/反转循环测试
【功能】
  仅控制 LF（左前）电机正转 2 秒 → 反转 2 秒 → 循环
【使用】烧录运行，按 SWITCH2 退出
"""

import gc, time, math
from machine import Pin
from smartcar import ticker
from motor import (
    set_motor, stop_all, MAX_PWM,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB,
    ENC_SCALE,
)

TEST_PWM = 30000

SWITCH2_PIN = 'D9'
LED_PIN = 'C4'

led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

pit = ticker(1)
pit.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
pit.start(10)

print("\n" + "=" * 55)
print("  仅 LF 电机：正转 2s → 反转 2s → 循环")
print("  按 SWITCH2 退出")
print("=" * 55)

lf_pwm = TEST_PWM
wheel_names  = ["RF", "LF", "LB", "RB"]
wheel_motors = [MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB]

for i in range(4):
    if switch2.value() != state2:
        break
    # 正转 2 秒
    print("{} 正转".format(wheel_names[i]))
    set_motor(wheel_motors[i], TEST_PWM)
    time.sleep_ms(2000)
    
    if switch2.value() != state2:
        break
    
    # 停 1 秒
    set_motor(wheel_motors[i], 0)
    time.sleep_ms(1000)
    
    if switch2.value() != state2:
        break
    
    # 反转 2 秒
    print("{} 反转".format(wheel_names[i]))
    set_motor(wheel_motors[i], -TEST_PWM)
    time.sleep_ms(2000)
    
    # 停 2 秒
    set_motor(wheel_motors[i], 0)
    time.sleep_ms(2000)
    

stop_all()
pit.stop()
print("Exit.")
