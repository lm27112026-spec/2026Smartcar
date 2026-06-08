"""
main.py - 闭环运动控制（编码器 + IMU + PID 融合）
【功能】
  - 初始化所有硬件模块
  - 每 20ms 执行一次控制循环：
    1. 读取 IMU → 更新姿态角
    2. 读取编码器 → 计算各轮转速
    3. 速度斜坡（平滑启动）
    4. 航向 PID → yaw 保持
    5. omni_drive_closed_loop → 闭环驱动
  - SWITCH2 退出
【依赖】motor（电机+编码器）、imu_motion（姿态）、pid（控制器）
"""

import gc, time
from machine import Pin
from smartcar import *
from seekfree import *
from motor import (
    omni_drive_closed_loop, stop_all, reset_encoders,
    get_encoder_speeds_filtered,
    LED_PIN, SWITCH2_PIN,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
)
from imu_motion import imu, update_angle
import imu_motion
from pid import PID

# ============================================================
#  一、初始化
# ============================================================

time.sleep_ms(100)
stop_all()

led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ============================================================
#  二、PID 参数
# ============================================================

# 航向 PID：yaw 误差（度）→ wz（归一化 -1~1）
# 注意：error_yaw 是度数，kp 按度数设计（不是弧度）
heading_pid = PID(
    kp=0.10,            # 1° 误差 → 0.10 wz
    ki=0.0005,          # 慢积分消除稳态偏置
    kd=0.0,             # 关掉微分（陀螺仪噪声大）
    integral_limit=0.1,
    output_limit=0.5,
)

# ============================================================
#  三、运动参数
# ============================================================

TARGET_SPEED = 0.3      # 目标速度 (0~1)
RAMP_RATE = 0.015       # 每 20ms 增加量（约 0.4s 到目标）
current_speed = 0.0
CONTROL_INTERVAL_S = 0.02

# ============================================================
#  四、编码器清零
# ============================================================

print("Calibrating encoders...")
time.sleep_ms(500)
reset_encoders()
time.sleep_ms(100)

# ============================================================
#  五、定时器（每 10ms 硬件捕获）
# ============================================================

ticker_flag = False
ticker_count = 0

def ticker_handler(t):
    global ticker_flag, ticker_count
    ticker_flag = True
    ticker_count = (ticker_count + 1) % 100

pit = ticker(1)
pit.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
pit.callback(ticker_handler)
pit.start(10)

# ============================================================
#  六、主控制循环（每 20ms 执行一次）
# ============================================================

print("\n=== Closed-Loop Fusion Control ===")
print("Target speed: {:.1f}, heading hold enabled".format(TARGET_SPEED))
print("Toggle SWITCH2 to stop.\n")

target_yaw = imu_motion.yaw  # 锁定当前航向

while True:
    # 按 SWITCH2 退出
    if switch2.value() != state2:
        stop_all()
        pit.stop()
        print("Stopped.")
        break

    if ticker_flag and ticker_count % 2 == 0:
        # ----- 1. IMU 读取 & 姿态更新 -----
        data = imu.read()
        update_angle(data[0], data[1], data[2],
                     data[3], data[4], data[5])

        # ----- 2. 编码器速度读取 -----
        actual_spd = get_encoder_speeds_filtered(CONTROL_INTERVAL_S)

        # ----- 3. 速度斜坡（平滑启动）-----
        if current_speed < TARGET_SPEED:
            current_speed += RAMP_RATE
            if current_speed > TARGET_SPEED:
                current_speed = TARGET_SPEED

        # ----- 4. 航向 PID（保持直线）-----
        error_yaw = target_yaw - imu_motion.yaw
        error_yaw = (error_yaw + 180) % 360 - 180  # 包裹到 [-180, 180)
        wz = heading_pid.compute(0, -error_yaw)

        # ----- 5. 闭环驱动（前馈 + 每轮 PI 反馈 + 航向修正）-----
        omni_drive_closed_loop(current_speed, 0, wz, actual_spd)

        # ----- 6. 调试打印（每 100ms）-----
        if ticker_count % 10 == 0:
            led.toggle()
            print("yaw={:6.1f}  spd={:.3f}  wz={:+.3f}  act={:.3f} {:.3f} {:.3f} {:.3f}".format(
                imu_motion.yaw, current_speed, wz,
                actual_spd[0], actual_spd[1], actual_spd[2], actual_spd[3]))

        ticker_flag = False

    gc.collect()
 