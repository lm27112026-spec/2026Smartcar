"""
test_angle_scan.py — 快速确定 P 值与实际方向的对应关系

【原理】
小车原地依次尝试多个前进方向（0, 45, 90, 135, 180, -135, -90, -45°），
每个方向跑 0.5 秒，同时打印 P_filt 和 motor_angle。

如果某个方向让 P_filt → 0（或接近 0），
说明那个方向正指向锚点 → 那个 motor_angle 就是正确的变换结果。

【使用】
1. 锚点放在任意固定位置
2. 烧录运行，观察哪个方向 P_filt 最接近 0
"""

import time
from motor import omni_move_by_angle, stop_all
from machine import UART, Pin
from uwb_tracker import UWBTracker

uwb = UWBTracker(uart_id=0, baudrate=115200, target_anchor="8834")
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# 8 个候选方向（omni_move_by_angle 数学坐标系）
# 0=右，90=前，180=左，-90=后
DIRECTIONS = [0, 45, 90, 135, 180, -135, -90, -45]
RUN_MS = 800
REST_MS = 500

print("=" * 60)
print("Angle Scan — 找 P_filt 最接近 0 的方向")
print("=" * 60)
print("  方向角 | motor_angle | P_filt | 实际移动")
print("-" * 60)

try:
    for d in DIRECTIONS:
        if switch2.value() != state2:
            print("SW2 exit")
            break

        stop_all()
        time.sleep_ms(REST_MS)

        # 清空滤波值
        uwb._p_filt = None
        uwb._d_filt = None

        # 向这个方向移动
        omni_move_by_angle(0.25, d)

        # 跑 0.8 秒，记录最后一帧的 P_filt
        last_p = None
        start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start) < RUN_MS:
            cmd = uwb.get_command()
            if uwb._frame_count > 0:
                last_p = uwb._p_filt
            time.sleep_ms(10)

        if last_p is not None:
            flag = " ← 最接近 0!" if abs(last_p) < 30 else ""
            print("  {:>4d}°     | {:>4d}°        | {:>+.0f}°    {}".format(d, d, last_p, flag))
        else:
            print("  {:>4d}°     | {:>4d}°        | 无数据")

        stop_all()

except Exception as e:
    print("Error:", e)
finally:
    stop_all()
    uwb.stop()
    print("=" * 60)
    print("Done")

