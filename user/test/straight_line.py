"""
straight_line.py - 直线行驶 + 航向保持
======================================
【功能】小车朝指定方向直行，偏航超过阈值自动回正
【约定】DIRECTION 使用数学约定（0=前，90=右，180=后，-90=左）
【使用】上电自动开始，到达距离自动停止
"""

import time, math
from machine import Pin
import imu_motion
from motor import (
    omni_drive_closed_loop, get_encoder_counts, stop_all,
    ENC_SCALE, SWITCH2_PIN
)
from pid import PID
from utils import normalize_angle

# ============================================================
#  用户配置（可修改）
# ============================================================
# 航向锁定：启动时自动锁定当前朝向
DISTANCE  = 0.25     # 目标距离（米）
SPEED     = 0.33     # 前进速度（归一化 0~1，约 0.15 m/s）
DEADBAND  = 10.0     # 航向死区（度）：偏超此值才回正

# ============================================================
#  PID 参数
# ============================================================
HEADING_KP = 0.02
HEADING_KI = 0.001
WZ_LIMIT   = 0.3
LOOP_DT    = 0.02    # 控制周期 20ms

# ============================================================
#  初始化
# ============================================================
print("\n" + "=" * 50)
print("  Straight Line + Heading Hold")
print("  Heading Hold  DISTANCE=%.2fm  DEADBAND=%.0f" % (DISTANCE, DEADBAND))
print("=" * 50)

# 清空编码器残留（motor.py 已有全局 ticker 在后台捕获）
for _ in range(5):
    get_encoder_counts()
    time.sleep_ms(5)

# IMU（使用 imu_motion 模块级实例，不重新创建）
d = imu_motion.imu.read()
imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
time.sleep_ms(50)

# PID
heading_pid = PID(kp=HEADING_KP, ki=HEADING_KI, kd=0,
                  integral_limit=50, output_limit=WZ_LIMIT)
heading_pid.reset()

# 初始化 IMU 航向
d = imu_motion.imu.read()
imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

# 锁定起始航向
target_heading = imu_motion.yaw

# 距离追踪
total_counts = [0, 0, 0, 0]
avg_enc_scale = sum(ENC_SCALE) / 4

# SWITCH2（拨码开关，上拉=未拨动=1）
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
base_state = switch2.value()

start_ms = time.ticks_ms()
last_print = start_ms

print("  Starting...")

try:
    while True:
        # ---- 1. 读取编码器（消费累计脉冲）----
        counts = get_encoder_counts()

        # ---- 2. 距离计算 ----
        for i in range(4):
            total_counts[i] += abs(counts[i])
        dist = sum(total_counts) / 4 / avg_enc_scale

        # ---- 0. SW2 退出 ----
        if switch2.value() != base_state:
            print("\n  SW2 aborted")
            break

        if dist >= DISTANCE:
            print("\n  *** Reached %.3fm (actual %.3fm) ***" % (DISTANCE, dist))
            break

        # ---- 3. 读取 IMU ----
        d = imu_motion.imu.read()
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

        # ---- 4. 航向误差（约定修正：yaw正=左转，target=-DIRECTION）----
        heading_error = normalize_angle(-target_heading - imu_motion.yaw)

        # ---- 5. 航向 PID ----
        if abs(heading_error) > DEADBAND:
            wz = heading_pid.compute(0, heading_error, LOOP_DT)
        else:
            wz = 0.0
            heading_pid.reset()

        # ---- 6. 计算当前轮速（从同一批 counts）----
        actual_speeds = [counts[i] / ENC_SCALE[i] / LOOP_DT for i in range(4)]

        # ---- 7. 闭环驱动 ----
        omni_drive_closed_loop(SPEED, 0, wz, actual_speeds, LOOP_DT)

        # ---- 8. 打印（500ms）----
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print) >= 500:
            print("  d=%.3f  yaw=%.1f  err=%.1f  wz=%.3f" %
                  (dist, imu_motion.yaw, heading_error, wz))
            last_print = now

        time.sleep_ms(int(LOOP_DT * 1000))

except KeyboardInterrupt:
    print("\n  Interrupted")
finally:
    stop_all()
    print("  Done.")
