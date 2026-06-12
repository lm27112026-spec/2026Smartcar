"""
motion_test.py - 6 方向运动测试
【功能】
  测试 omni_drive 的 6 个基础运动方向
  每个方向运行 1.5 秒后停止 0.5 秒
【使用】烧录运行，按 SWITCH2 退出
"""

import gc, time
from machine import Pin
from motor import (
    omni_drive, stop_all,
    LED_PIN, SWITCH2_PIN,
)

LED_PIN     = 'C4'
SWITCH2_PIN = 'D9'


led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

SPEED = 0.3
RUN_MS = 1500
REST_MS = 500

# (vx, vy, wz, 中文描述)
motions = [
    ( SPEED,  0,     0,    "前进  vx=+0.3"),
    (-SPEED,  0,     0,    "后退  vx=-0.3"),
    ( 0,     -SPEED, 0,    "左移  vy=-0.3"),
    ( 0,      SPEED, 0,    "右移  vy=+0.3"),
    ( 0,      0,     SPEED, "左旋  wz=+0.3"),
    ( 0,      0,    -SPEED, "右旋  wz=-0.3"),
]

print("=" * 50)
print("  6 方向运动测试")
print("  每个方向运行 1.5s → 休息 0.5s")
print("  按 SWITCH2 随时退出")
print("=" * 50)
print()
print("观察每个命令实际产生的运动方向，填下表：")
print("  命令           → 实际方向")
print("  vx=+0.3 前   → ?")
print("  vx=-0.3 后   → ?")
print("  vy=-0.3 左   → ?")
print("  vy=+0.3 右   → ?")
print("  wz=+0.3 左旋 → ?")
print("  wz=-0.3 右旋 → ?")
print()

for vx, vy, wz, desc in motions:
    if switch2.value() != state2:
        break
    
    print(">>> {}".format(desc))
    omni_drive(vx, vy, wz)
    led.on()
    time.sleep_ms(RUN_MS)
    
    omni_drive(0, 0, 0)
    led.off()
    time.sleep_ms(REST_MS)

stop_all()
print("\n测试完成")
