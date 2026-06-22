"""
test_move.py — 开环运动测试
测试 vx, vy 方向与 UWB 坐标系的关系
"""

import time
from machine import Pin
from motor import omni_drive, stop_all, LED_PIN, SWITCH2_PIN
import imu_motion
from imu_motion import update_angle
from uwb_position import UWBPosition

# 硬件
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# UWB 初始化
uwb = UWBPosition(target_anchor="8834")

# IMU 热身
for _ in range(10):
    d = imu_motion.imu.read()
    if d is not None:
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

print("=== Open Loop Move Test ===")
print("Waiting for UWB data...")

# 等待 UWB 数据
while uwb.get_frame_count() == 0:
    uwb.step()
    time.sleep_ms(10)

x0, y0 = uwb.get_position()
print("Start position: ({:.1f}, {:.1f})".format(x0, y0))
print("")
print("Testing: vx=0.3, vy=0 for 2 seconds")
print("Watch UWB coordinates to see direction")
print("")

time.sleep(1)

# 测试 1: 只给 vx
start = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), start) < 2000:
    if switch2.value() != state2:
        break
    omni_drive(0.3, 0, 0)  # vx=0.3, vy=0, wz=0
    uwb.step()
    time.sleep_ms(10)

stop_all()
time.sleep(0.5)

x1, y1 = uwb.get_position()
print("After vx=0.3: ({:.1f}, {:.1f})".format(x1, y1))
print("  dx={:.1f}, dy={:.1f}".format(x1-x0, y1-y0))
print("")

time.sleep(1)

# 测试 2: 只给 vy
print("Testing: vx=0, vy=0.3 for 2 seconds")
print("")

start = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), start) < 2000:
    if switch2.value() != state2:
        break
    omni_drive(0, 0.3, 0)  # vx=0, vy=0.3, wz=0
    uwb.step()
    time.sleep_ms(10)

stop_all()
time.sleep(0.5)

x2, y2 = uwb.get_position()
print("After vy=0.3: ({:.1f}, {:.1f})".format(x2, y2))
print("  dx={:.1f}, dy={:.1f}".format(x2-x1, y2-y1))
print("")

print("=== Test Complete ===")
print("Compare dx/dy to determine correct axis mapping")
uwb.stop()
