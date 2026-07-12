# mem_test.py
import gc, sys

# 1. 强行把可能存在的残留缓存全部清理掉
for m in ['motor', 'imu_motion', 'key', 'uwb_position', 'IMU_hold', 'cam_control']:
    if m in sys.modules:
        del sys.modules[m]

gc.collect()
print("1. 初始空闲堆内存: {} 字节".format(gc.mem_free()))

import motor
gc.collect()
print("2. 加载 motor 后: {} 字节 (消耗: {})".format(gc.mem_free(), 78864 - gc.mem_free() if 'prev' not in locals() else 0))
prev = gc.mem_free()

import imu_motion
gc.collect()
print("3. 加载 imu_motion 后: {} 字节 (消耗: {})".format(gc.mem_free(), prev - gc.mem_free()))
prev = gc.mem_free()

import key
gc.collect()
print("4. 加载 key 后: {} 字节 (消耗: {})".format(gc.mem_free(), prev - gc.mem_free()))
prev = gc.mem_free()

import uwb_position
gc.collect()
print("5. 加载 uwb_position 后: {} 字节 (消耗: {})".format(gc.mem_free(), prev - gc.mem_free()))
prev = gc.mem_free()

import IMU_hold
gc.collect()
print("6. 加载 IMU_hold 后: {} 字节 (消耗: {})".format(gc.mem_free(), prev - gc.mem_free()))
prev = gc.mem_free()

import cam_control
gc.collect()
print("7. 加载 cam_control 后: {} 字节 (消耗: {})".format(gc.mem_free(), prev - gc.mem_free()))