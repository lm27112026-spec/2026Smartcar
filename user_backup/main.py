"""
main.py - 主调度层（编码器 + IMU + PID + PWM 融合）
【层】主调度层
【功能】
  - 初始化所有硬件模块
  - 每 20ms 执行一次控制循环：
      1. 读取 IMU → 更新姿态角（roll/pitch/yaw）
      2. 读取编码器 → 计算各轮转速
      3. 航向 PID → 计算旋转修正量 wz
      4. 速度 PID → 计算基础 PWM
      5. omni_drive 输出到电机
  - SWITCH2 退出
【依赖】motor（电机+编码器）、imu_motion（姿态）、pid（控制器）
"""

import gc, time
from machine import Pin
from smartcar import *
from seekfree import *
from motor import (
    omni_drive, omni_move_by_angle, stop_all, reset_encoders,
    get_encoder_counts, get_encoder_speeds,
    LED_PIN, SWITCH2_PIN, MAX_PWM,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
)
from imu_motion import imu, update_angle
import imu_motion
from pid import PID

# ============================================================
#  一、初始化（LED、SWITCH、IMU 由各模块导入时自动完成）
# ============================================================

time.sleep_ms(100)

stop_all()  # 确保所有电机初始为停止状态

led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ============================================================
#  二、PID 参数配置（先给出保守初始值，实测后调整）
# ============================================================

# 航向 PID：保持当前 yaw 角，输出 wz 旋转修正量（归一化 -1~1）
heading_pid = PID(
    kp=0.5, ki=0.0, kd=0.0,
    integral_limit=0, output_limit=0.5,
)

# 速度 PID（暂用开环，待标定编码器脉冲→速度换算后再启用）
# speed_pid = PID(kp=0.3, ki=0.01, kd=0.0, integral_limit=2000, output_limit=MAX_PWM)

TARGET_SPEED = 0.3
CONTROL_INTERVAL_MS = 20
CONTROL_INTERVAL_S = 0.02

# ============================================================
#  三、编码器基准清零
# ============================================================

print("Calibrating encoders...")
time.sleep_ms(500)
reset_encoders()
time.sleep_ms(100)

# ============================================================
#  四、定时器 —— 硬件捕获 IMU + 编码器（每 10ms 自动 capture）
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
#  五、主控制循环（每 20ms 执行一次）
# ============================================================

print("\n=== Fusion Control Loop ===")
print("Target speed: {:.1f}, heading hold enabled".format(TARGET_SPEED))
print("Toggle SWITCH2 to stop.\n")

target_yaw = imu_motion.yaw  # 锁定启动时的航向为目标值

while True:
    if switch2.value() != state2:
        omni_move_by_angle(0, 0, 0)
        pit.stop()
        print("Test program stop.")
        break

    if ticker_flag and ticker_count % 2 == 0:
        # ----- 1. 读取 IMU → 更新姿态 -----
        data = imu.read()
        update_angle(data[0], data[1], data[2],
                     data[3], data[4], data[5])

        # ----- 2. 读取编码器 → 转速（米/秒）-----
        speeds = get_encoder_speeds(CONTROL_INTERVAL_S)

        # ----- 3. 航向 PID（误差预先包裹到 [-180, 180)）-----
        error_yaw = target_yaw - imu_motion.yaw
        error_yaw = (error_yaw + 180) % 360 - 180
        wz = -heading_pid.compute(0, -error_yaw)

        # ----- 4. 输出到电机 -----
        omni_drive(TARGET_SPEED, 0, wz)

        # ----- 5. 调试打印（每 100ms 一次）-----
        if ticker_count % 10 == 0:
            led.toggle()
            print("yaw={:6.1f}  gz_raw={:5d}  gz_dps={:+.1f}  wz={:+.3f}".format(
                imu_motion.yaw, data[5], data[5] / 16.4, wz))

        ticker_flag = False

    gc.collect()

