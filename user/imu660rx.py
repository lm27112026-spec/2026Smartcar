"""
imu660rx.py - IMU660RX 传感器数据读取与角度解算演示
功能：读取 IMU660RX 的加速度计和陀螺仪原始数据，通过互补滤波解算 pitch 和 roll 角度，
      以 200ms 周期打印 acc/gyro/angle 数据到串口，不涉及电机控制。
与 imu_motion.py 的区别：
  - 仅作传感器数据显示，无电机驱动
  - 仅解算 pitch 和 roll，不解算 yaw
  - 定时器中使用标志位+计数方式，每 20 个 tick 输出一次
  - 不包含运动控制逻辑
"""

import gc, time, math
from machine import *
from smartcar import *
from seekfree import *

ACCEL_SENSITIVITY = 4096
GYRO_SENSITIVITY = 16.4

time.sleep_ms(100)

print("REAL TYPE : " + BOARD_TYPE)
print("BOARD VERSION : " + BOARD_VERSION)

LED_PIN = 'C4'
SWITCH2_PIN = 'D9'

print("LED_PIN     : " + LED_PIN)
print("SWITCH2_PIN : " + SWITCH2_PIN)

led     = Pin(LED_PIN, Pin.OUT, value = True)
switch2 = Pin(SWITCH2_PIN, Pin.IN , pull = Pin.PULL_UP_47K)
state2  = switch2.value()

IMU660RX.help()

imu = IMU660RX()
imu.info()

imu_data = imu.read()

ticker_flag = False
ticker_count = 0

pitch = 0.0
roll = 0.0
last_time = 0
filter_alpha = 0.98

def time_pit_handler(ticker_obj):
    global ticker_flag
    global ticker_count
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

def update_angle(ax, ay, az, gx, gy, gz):
    global pitch, roll, last_time

    current_time = time.ticks_ms()
    if last_time == 0:
        last_time = current_time
        return

    dt = (current_time - last_time) * 0.001
    last_time = current_time

    ax_g = ax / ACCEL_SENSITIVITY
    ay_g = ay / ACCEL_SENSITIVITY
    az_g = az / ACCEL_SENSITIVITY

    pitch_accel = math.atan2(-ax_g, math.sqrt(ay_g**2 + az_g**2)) * 180.0 / math.pi
    roll_accel = math.atan2(ay_g, az_g) * 180.0 / math.pi

    gx_dps = gx / GYRO_SENSITIVITY
    gy_dps = gy / GYRO_SENSITIVITY

    pitch = filter_alpha * (pitch + gy_dps * dt) + (1 - filter_alpha) * pitch_accel
    roll = filter_alpha * (roll + gx_dps * dt) + (1 - filter_alpha) * roll_accel

pit1 = ticker(1)
pit1.capture_list(imu)
pit1.callback(time_pit_handler)
pit1.start(10)

while True:
    if (ticker_flag and ticker_count % 20 == 0):
        led.toggle()
        print("acc = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[0], imu_data[1], imu_data[2]))
        print("gyro = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[3], imu_data[4], imu_data[5]))

        update_angle(imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4], imu_data[5])
        print("angle = roll:{:>6.1f}, pitch:{:>6.1f}.".format(roll, pitch))

        ticker_flag = False

    if switch2.value() != state2:
        pit1.stop()
        print("Test program stop.")
        break

    gc.collect()
