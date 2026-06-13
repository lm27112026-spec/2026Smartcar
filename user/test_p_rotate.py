"""
test_p_rotate.py — 原地旋转，读 P 值

【原理】小车原地 360° 自转，每 45° 停 2 秒读 P_filt。
锚点固定不动，这样 P 值只取决于车头朝向相对锚点的方向。

如果 P=0 时车头正对锚点 → motor_angle = P + 90?

【使用】
1. 锚点放在任意固定位置（建议距离 1m 外）
2. 小车放在原地不动（轮子不转）
3. 烧录运行
"""

import time
from motor import omni_drive, stop_all

from machine import Pin
from uwb_tracker import UWBTracker

uwb = UWBTracker(uart_id=0, baudrate=115200, target_anchor="8834")
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# 原地右旋（wz=+0.3），用 omni_drive(vx=0, vy=0, wz=0.3)
print("=" * 60)
print("原地旋转 — 读 P 值")
print("观察哪个朝向时 P_filt = 0")
print("=" * 60)

try:
    input("按 Enter 开始旋转...")

    # 先疯狂读帧直到稳定
    for _ in range(20):
        uwb.get_command()
        time.sleep_ms(10)

    print("\n开始旋转...按 SW2 停止\n")
    
    # 原地旋转
    start_ms = time.ticks_ms()
    last_print = start_ms
    
    while True:
        if switch2.value() != state2:
            break

        omni_drive(0, 0, 0.3)  # 右旋
        cmd = uwb.get_command()
        
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) >= 200:
            d = uwb.get_distance()
            a = uwb.get_angle()
            p = uwb._p_filt if uwb._p_filt is not None else 0
            print("  P_filt={:>+.0f}°  D={:.0f}cm  get_angle={:>+.0f}°".format(
                p, d*100, a))
            last_print = now
        
        time.sleep_ms(10)

except Exception as e:
    print("Error:", e)
finally:
    stop_all()
    uwb.stop()
    print("Done")

