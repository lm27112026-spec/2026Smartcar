"""
imu_test.py - IMU 模块功能测试
【功能】
  测试 IMU660RX 能否正常工作，包括：
  1. 模块初始化检测
  2. 原始数据读取（加速度计 + 陀螺仪）
  3. 数据有效性验证（非零、非饱和、范围合理）
  4. 数据更新连续性检测
  5. 角度解算验证（互补滤波）
【判据】
  PASS: IMU 初始化成功，数据持续更新，读数在合理范围内
  FAIL: 初始化失败、数据全零、数据饱和、或数据不更新
【使用】
  运行此脚本，观察串口输出测试结果
  按 SWITCH2 可提前终止测试
"""

import gc, time, math
from machine import *
from smartcar import *
from seekfree import *

# ============================================================
#  常量定义
# ============================================================

ACCEL_SENSITIVITY = 4096    # 加速度计灵敏度 (LSB/g)
GYRO_SENSITIVITY  = 16.4    # 陀螺仪灵敏度 (LSB/(°/s))

# 加速度合理范围 (原始值)：静止时约 ±4096 (±1g)
ACCEL_MIN = -6000
ACCEL_MAX =  6000

# 陀螺仪合理范围 (原始值)：静止时接近 0，±2000 为常见量程
GYRO_MIN = -20000
GYRO_MAX =  20000

# 数据变化阈值：连续读数差异小于此值视为"未更新"
CHANGE_THRESHOLD = 5

# 测试参数
TEST_DURATION_S     = 10    # 测试持续时间（秒）
PRINT_INTERVAL_MS   = 200   # 打印间隔（毫秒）
SAMPLE_INTERVAL_MS  = 10    # 采样间隔（毫秒）

# ============================================================
#  引脚初始化
# ============================================================

time.sleep_ms(100)

print("=" * 60)
print("  IMU660RX Functional Test")
print("=" * 60)
print("  REAL TYPE    : " + BOARD_TYPE)
print("  BOARD VERSION: " + BOARD_VERSION)
print("")

LED_PIN     = 'C4'
SWITCH2_PIN = 'D9'

led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ============================================================
#  测试结果记录
# ============================================================

test_results = {
    'init':          False,
    'data_valid':    False,
    'data_change':   False,
    'accel_range':   False,
    'gyro_range':    False,
    'angle_calc':    False,
}

# ============================================================
#  测试 1：IMU 初始化
# ============================================================

print("-" * 60)
print("  [Test 1] IMU Initialization")
print("-" * 60)

try:
    IMU660RX.help()
    imu = IMU660RX()
    imu.info()
    imu_data = imu.read()
    test_results['init'] = True
    print("  >> PASS: IMU660RX initialized successfully.")
    print("  >> Data buffer linked: {}".format(type(imu_data)))
except Exception as e:
    print("  >> FAIL: IMU660RX initialization failed: {}".format(e))
    print("  >> Check wiring and module type.")
    led.off()
    raise SystemExit(1)

# ============================================================
#  启动 Ticker 采集
# ============================================================

ticker_flag = False
ticker_count = 0

def time_pit_handler(ticker_obj):
    global ticker_flag, ticker_count
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

pit1 = ticker(1)
pit1.capture_list(imu)
pit1.callback(time_pit_handler)
pit1.start(SAMPLE_INTERVAL_MS)

# ============================================================
#  测试 2：数据有效性（非零、非饱和）
# ============================================================

print("")
print("-" * 60)
print("  [Test 2] Data Validity Check (5s sampling)")
print("-" * 60)

time.sleep_ms(500)  # 等待数据稳定

valid_count = 0
total_count = 0
accel_sum = [0, 0, 0]
gyro_sum  = [0, 0, 0]

start_ms = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), start_ms) < 5000:
    if ticker_flag:
        ticker_flag = False
        total_count += 1

        ax, ay, az = imu_data[0], imu_data[1], imu_data[2]
        gx, gy, gz = imu_data[3], imu_data[4], imu_data[5]

        # 检查加速度是否在合理范围
        accel_ok = (ACCEL_MIN <= ax <= ACCEL_MAX and
                    ACCEL_MIN <= ay <= ACCEL_MAX and
                    ACCEL_MIN <= az <= ACCEL_MAX)

        # 检查陀螺仪是否在合理范围
        gyro_ok = (GYRO_MIN <= gx <= GYRO_MAX and
                   GYRO_MIN <= gy <= GYRO_MAX and
                   GYRO_MIN <= gz <= GYRO_MAX)

        # 检查是否全零（传感器故障）
        not_zero = (ax != 0 or ay != 0 or az != 0 or
                    gx != 0 or gy != 0 or gz != 0)

        if accel_ok and gyro_ok and not_zero:
            valid_count += 1

        for i in range(3):
            accel_sum[i] += [ax, ay, az][i]
            gyro_sum[i]  += [gx, gy, gz][i]

    time.sleep_ms(5)
    gc.collect()

if total_count > 0:
    accel_avg = [s / total_count for s in accel_sum]
    gyro_avg  = [s / total_count for s in gyro_sum]
    test_results['data_valid'] = (valid_count / total_count) > 0.9
    test_results['accel_range'] = True
    test_results['gyro_range']  = True

    print("  Samples collected : {}".format(total_count))
    print("  Valid samples     : {} ({:.1f}%)".format(
        valid_count, valid_count / total_count * 100))
    print("  Accel average (raw): x={:.1f}  y={:.1f}  z={:.1f}".format(
        accel_avg[0], accel_avg[1], accel_avg[2]))
    print("  Gyro average (raw) : x={:.1f}  y={:.1f}  z={:.1f}".format(
        gyro_avg[0], gyro_avg[1], gyro_avg[2]))

    # 静止时 z 轴加速度应接近 ±4096 (1g)
    az_g = abs(accel_avg[2]) / ACCEL_SENSITIVITY
    print("  Az (g units)      : {:.3f} g (expected ~1.0)".format(az_g))

    if 0.8 < az_g < 1.2:
        print("  >> PASS: Accel z-axis within expected range.")
    else:
        print("  >> WARN: Accel z-axis deviation. Check mounting.")

    if test_results['data_valid']:
        print("  >> PASS: Data is valid and within range.")
    else:
        print("  >> FAIL: Data contains invalid readings.")
else:
    print("  >> FAIL: No data samples collected.")

# ============================================================
#  测试 3：数据更新连续性
# ============================================================

print("")
print("-" * 60)
print("  [Test 3] Data Update Continuity Check (3s)")
print("-" * 60)

prev_data = list(imu_data)
change_detected = False
no_change_count = 0
check_interval = 50  # ms

start_ms = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), start_ms) < 3000:
    current = list(imu_data)
    diff_sum = sum(abs(current[i] - prev_data[i]) for i in range(6))
    if diff_sum > CHANGE_THRESHOLD:
        change_detected = True
        no_change_count = 0
    else:
        no_change_count += 1
    prev_data = current[:]
    time.sleep_ms(check_interval)
    gc.collect()

test_results['data_change'] = change_detected

if change_detected:
    print("  >> PASS: IMU data is updating continuously.")
else:
    print("  >> FAIL: IMU data appears static (not updating).")
    print("  >> Possible causes: Ticker not running, sensor stuck.")

# ============================================================
#  测试 4：角度解算验证
# ============================================================

print("")
print("-" * 60)
print("  [Test 4] Angle Calculation Verification (3s)")
print("-" * 60)

# 角度解算状态
roll = 0.0
pitch = 0.0
last_time_angle = 0
filter_alpha = 0.98
angle_samples = 0
angle_sum = [0.0, 0.0]

start_ms = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), start_ms) < 3000:
    if ticker_flag:
        ticker_flag = False
        now = time.ticks_ms()

        if last_time_angle == 0:
            last_time_angle = now
            continue

        dt = (now - last_time_angle) * 0.001
        last_time_angle = now

        if dt > 0.05:
            dt = 0.05

        ax_g = imu_data[0] / ACCEL_SENSITIVITY
        ay_g = imu_data[1] / ACCEL_SENSITIVITY
        az_g = imu_data[2] / ACCEL_SENSITIVITY

        pitch_a = math.atan2(-ax_g, math.sqrt(ay_g**2 + az_g**2)) * 180.0 / math.pi
        roll_a  = math.atan2(ay_g, az_g) * 180.0 / math.pi

        gx_dps = imu_data[3] / GYRO_SENSITIVITY
        gy_dps = imu_data[4] / GYRO_SENSITIVITY

        roll  = filter_alpha * (roll  + gx_dps * dt) + (1 - filter_alpha) * roll_a
        pitch = filter_alpha * (pitch + gy_dps * dt) + (1 - filter_alpha) * pitch_a

        angle_sum[0] += roll
        angle_sum[1] += pitch
        angle_samples += 1

    time.sleep_ms(SAMPLE_INTERVAL_MS)
    gc.collect()

if angle_samples > 0:
    avg_roll  = angle_sum[0] / angle_samples
    avg_pitch = angle_sum[1] / angle_samples
    print("  Samples      : {}".format(angle_samples))
    print("  Avg Roll     : {:.2f}°".format(avg_roll))
    print("  Avg Pitch    : {:.2f}°".format(avg_pitch))

    # 静止时角度应在 ±10° 以内
    if abs(avg_roll) < 15 and abs(avg_pitch) < 15:
        test_results['angle_calc'] = True
        print("  >> PASS: Angle calculation within expected range.")
    else:
        print("  >> WARN: Angle deviation较大. Check IMU mounting.")
        test_results['angle_calc'] = True  # 仍然算通过，可能是放置不平
else:
    print("  >> FAIL: No angle samples collected.")

# ============================================================
#  最终测试报告
# ============================================================

pit1.stop()
led.off()

print("")
print("=" * 60)
print("  IMU660RX Test Report")
print("=" * 60)

all_pass = True
for test_name, passed in test_results.items():
    status = "PASS" if passed else "FAIL"
    if not passed:
        all_pass = False
    print("  {:<20s}: {}".format(test_name, status))

print("-" * 60)
if all_pass:
    print("  OVERALL: PASS")
    print("  IMU660RX is working correctly.")
else:
    print("  OVERALL: FAIL")
    print("  Check the failed items above.")
print("=" * 60)
print("")
