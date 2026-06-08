import time
from machine import Pin
from motor import set_motor, stop_all, MAX_PWM, MOTOR_LB

TEST_PWM = 30000
SWITCH2_PIN = 'D9'
LED_PIN = 'C4'

led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

print("LF 电机一直正转，按 SWITCH2 退出")
set_motor(MOTOR_LB, TEST_PWM)

while True:
    if switch2.value() != state2:
        break
    time.sleep_ms(10)   # 防止占用 CPU 过高

stop_all()
print("退出")