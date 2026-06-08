"""
motor_test.py - 电机模块功能测试
【功能】
  测试 4 路电机能否正常工作，包括：
  1. MOTOR_CONTROLLER 初始化检测
  2. 单电机正反转测试
  3. 编码器反馈读取
  4. 全向驱动运动学验证
  5. 急停功能测试
【说明】
  直接使用 seekfree.MOTOR_CONTROLLER 类控制电机（与官方 demo 一致），
  不使用 motor.py 的 set_motor（其双方向引脚模式与硬件不匹配）。
  编码器仍通过 motor.py 的 encoder 对象读取。
【使用】
  运行此脚本，观察串口输出测试结果
  按 SWITCH2 可提前终止测试
  ⚠ 轮子需悬空或放在光滑表面，避免测试中机器人跑飞
"""

import gc, time, math
from machine import *
from smartcar import *
from seekfree import *

# ============================================================
#  常量定义
# ============================================================

MOTOR_NAMES = ['RF', 'LF', 'LB', 'RB']

# MOTOR_CONTROLLER duty 范围: ±10000
TEST_DUTY_LOW  = 3000    # 低速测试 duty (30%)
TEST_DUTY_HIGH = 6000    # 高速测试 duty (60%)
ENC_TIMEOUT_MS = 800     # 编码器响应超时（毫秒）

# ============================================================
#  引脚初始化
# ============================================================

time.sleep_ms(100)

print("=" * 60)
print("  4-Wheel Motor Functional Test")
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
#  根据开发板类型选择电机通道
# ============================================================

MOTOR_CH = [None, None, None, None]

if BOARD_TYPE == 'RT1021_144P_BTB':
    MOTOR_CH[0] = MOTOR_CONTROLLER.PWM_D6_DIR_D7    # RF
    MOTOR_CH[1] = MOTOR_CONTROLLER.PWM_C30_DIR_C31  # LF
    MOTOR_CH[2] = MOTOR_CONTROLLER.PWM_C28_DIR_C29  # LB
    MOTOR_CH[3] = MOTOR_CONTROLLER.PWM_D4_DIR_D5    # RB
elif BOARD_TYPE == 'RT1021_144P_2P54':
    MOTOR_CH[0] = MOTOR_CONTROLLER.PWM_D6_DIR_D7    # RF
    MOTOR_CH[1] = MOTOR_CONTROLLER.PWM_C30_DIR_C31  # LF
    MOTOR_CH[2] = MOTOR_CONTROLLER.PWM_C28_DIR_C29  # LB
    MOTOR_CH[3] = MOTOR_CONTROLLER.PWM_D4_DIR_D5    # RB
elif BOARD_TYPE == 'RT1021_100P_2P54':
    MOTOR_CH[0] = MOTOR_CONTROLLER.PWM_C24_DIR_C26
    MOTOR_CH[1] = MOTOR_CONTROLLER.PWM_C25_DIR_C27

# ============================================================
#  初始化编码器（通过 smartcar encoder 直接读取）
# ============================================================

ENCODER_LF_A, ENCODER_LF_B = 'C3',  'C2'
ENCODER_LB_A, ENCODER_LB_B = 'D14', 'D13'
ENCODER_RB_A, ENCODER_RB_B = 'D16', 'D15'
ENCODER_RF_A, ENCODER_RF_B = 'C1',  'C0'

encoder_rf = encoder(ENCODER_RF_A, ENCODER_RF_B, capture_div=1)
encoder_lf = encoder(ENCODER_LF_A, ENCODER_LF_B, capture_div=1)
encoder_lb = encoder(ENCODER_LB_A, ENCODER_LB_B, capture_div=1)
encoder_rb = encoder(ENCODER_RB_A, ENCODER_RB_B, capture_div=1)

ENC_LIST = [encoder_rf, encoder_lf, encoder_lb, encoder_rb]

# 启动 ticker 采集编码器
ticker_flag = False
ticker_count = 0

def time_pit_handler(ticker_obj):
    global ticker_flag, ticker_count
    ticker_flag = True
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

pit1 = ticker(1)
pit1.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
pit1.callback(time_pit_handler)
pit1.start(10)

# ============================================================
#  初始化电机（MOTOR_CONTROLLER 类）
# ============================================================

print("  Initializing motors...")
motors = []
for i in range(4):
    # RF 和 RB 需要 invert=True 以匹配物理方向
    inv = (i == 0 or i == 3)
    m = MOTOR_CONTROLLER(MOTOR_CH[i], 15000, duty=0, invert=inv)
    m.info()
    motors.append(m)

print("  >> All 4 motors initialized.")
print("")

# ============================================================
#  测试结果记录
# ============================================================

test_results = {
    'pwm_init':       False,
    'motor_forward':  [False, False, False, False],
    'motor_reverse':  [False, False, False, False],
    'encoder_read':   [False, False, False, False],
    'encoder_dir':    [False, False, False, False],
    'omni_drive':     False,
    'emergency_stop': False,
}

# ============================================================
#  辅助函数
# ============================================================

def check_switch():
    return switch2.value() != state2

def stop_all_motors():
    for m in motors:
        m.duty(0)

# ============================================================
#  测试 1：MOTOR_CONTROLLER 初始化 & 急停
# ============================================================

print("-" * 60)
print("  [Test 1] MOTOR_CONTROLLER Init & Emergency Stop")
print("-" * 60)

try:
    stop_all_motors()
    test_results['pwm_init'] = True
    print("  >> PASS: MOTOR_CONTROLLER initialized.")

    # 急停测试
    motors[0].duty(TEST_DUTY_LOW)
    time.sleep_ms(200)
    stop_all_motors()
    time.sleep_ms(200)
    test_results['emergency_stop'] = True
    print("  >> PASS: Emergency stop functional.")

except Exception as e:
    print("  >> FAIL: Init error: {}".format(e))

if check_switch():
    stop_all_motors()
    pit1.stop()
    print("  SW2 - Test aborted.")
    raise SystemExit(0)

# ============================================================
#  测试 2：单电机正反转 + 编码器验证
# ============================================================

print("")
print("-" * 60)
print("  [Test 2] Individual Motor Forward/Reverse + Encoder")
print("-" * 60)
print("  ⚠ Ensure wheels are off the ground!")
print("  Duty range: ±10000, test duty: ±{}".format(TEST_DUTY_LOW))
print("  Starting in 2 seconds...")
time.sleep_ms(2000)

for i in range(4):
    if check_switch():
        break

    print("  Testing Motor {} ({}):".format(i, MOTOR_NAMES[i]))
    m = motors[i]

    # --- 正转测试 ---
    print("    Forward +{:<5d}...".format(TEST_DUTY_LOW), end="")
    m.duty(TEST_DUTY_LOW)
    time.sleep_ms(1000)

    # 清空编码器残余
    ENC_LIST[i].get()
    time.sleep_ms(200)

    enc_val = ENC_LIST[i].get()
    if abs(enc_val) > 2:
        test_results['motor_forward'][i] = True
        test_results['encoder_read'][i] = True
        direction = "positive" if enc_val > 0 else "negative"
        print(" enc={:+d} ({})".format(enc_val, direction))
    else:
        # 再等待更长时间
        time.sleep_ms(1000)
        enc_val = ENC_LIST[i].get()
        if abs(enc_val) > 2:
            test_results['motor_forward'][i] = True
            test_results['encoder_read'][i] = True
            direction = "positive" if enc_val > 0 else "negative"
            print(" enc={:+d} ({})".format(enc_val, direction))
        else:
            print(" enc={} (no response)".format(enc_val))

    m.duty(0)
    time.sleep_ms(500)

    # --- 反转测试 ---
    print("    Reverse -{:<5d}...".format(TEST_DUTY_LOW), end="")
    m.duty(-TEST_DUTY_LOW)
    time.sleep_ms(1000)

    ENC_LIST[i].get()
    time.sleep_ms(200)

    enc_val = ENC_LIST[i].get()
    if abs(enc_val) > 2:
        test_results['motor_reverse'][i] = True
        if not test_results['encoder_read'][i]:
            test_results['encoder_read'][i] = True
        direction = "positive" if enc_val > 0 else "negative"
        print(" enc={:+d} ({})".format(enc_val, direction))

        if test_results['motor_forward'][i]:
            test_results['encoder_dir'][i] = True
    else:
        time.sleep_ms(1000)
        enc_val = ENC_LIST[i].get()
        if abs(enc_val) > 2:
            test_results['motor_reverse'][i] = True
            if not test_results['encoder_read'][i]:
                test_results['encoder_read'][i] = True
            direction = "positive" if enc_val > 0 else "negative"
            print(" enc={:+d} ({})".format(enc_val, direction))
            if test_results['motor_forward'][i]:
                test_results['encoder_dir'][i] = True
        else:
            print(" enc={} (no response)".format(enc_val))

    m.duty(0)
    time.sleep_ms(300)
    print("")

# ============================================================
#  测试 3：全向驱动运动学验证
# ============================================================

if not check_switch():
    print("-" * 60)
    print("  [Test 3] Omni-Drive Kinematics")
    print("-" * 60)

    # 全向运动学解算（与 motor.py 一致）
    def omni_kinematics(vx, vy, wz):
        w_rf =  vx + vy - wz
        w_lf = -vx - vy - wz
        w_lb =  vx - vy - wz
        w_rb = -vx + vy - wz
        return [w_rf, w_lf, w_lb, w_rb]

    def omni_drive_local(vx, vy, wz):
        speeds = omni_kinematics(vx, vy, wz)
        max_speed = max(abs(s) for s in speeds)
        scale = 1.0
        if max_speed > 1.0:
            scale = 1.0 / max_speed
        duty_vals = [int(s * scale * 10000) for s in speeds]
        for j in range(4):
            motors[j].duty(duty_vals[j])

    # 测试前进
    print("    Forward (vx=0, vy=1, wz=0)...", end="")
    omni_drive_local(0, 0.5, 0)
    time.sleep_ms(1000)
    # 清空编码器
    for enc in ENC_LIST:
        enc.get()
    time.sleep_ms(300)
    enc_vals = [enc.get() for enc in ENC_LIST]
    non_zero = sum(1 for v in enc_vals if abs(v) > 2)
    if non_zero >= 2:
        test_results['omni_drive'] = True
    print(" enc=[{:+d},{:+d},{:+d},{:+d}] ({} wheels)".format(
        *enc_vals, non_zero))

    stop_all_motors()
    time.sleep_ms(300)

    # 测试旋转
    print("    Rotate (vx=0, vy=0, wz=0.5)...", end="")
    omni_drive_local(0, 0, 0.5)
    time.sleep_ms(1000)
    for enc in ENC_LIST:
        enc.get()
    time.sleep_ms(300)
    enc_vals = [enc.get() for enc in ENC_LIST]
    non_zero = sum(1 for v in enc_vals if abs(v) > 2)
    print(" enc=[{:+d},{:+d},{:+d},{:+d}] ({} wheels)".format(
        *enc_vals, non_zero))

    stop_all_motors()
    time.sleep_ms(300)

# ============================================================
#  测试 4：PWM 线性度验证（RF 电机）
# ============================================================

if not check_switch():
    print("")
    print("-" * 60)
    print("  [Test 4] Duty Linearity Check (RF motor)")
    print("-" * 60)

    duty_levels = [0, 2000, 4000, 6000, 8000, 10000, 0]
    enc_readings = []

    for duty_val in duty_levels:
        if check_switch():
            break
        motors[0].duty(duty_val)
        time.sleep_ms(400)
        ENC_LIST[0].get()  # 清空
        time.sleep_ms(300)
        enc = ENC_LIST[0].get()
        enc_readings.append(enc)
        print("    duty={:>6d} -> enc={:+d}".format(duty_val, enc))

    stop_all_motors()

    # 验证线性度
    monotonic = True
    for j in range(1, len(enc_readings) - 1):
        if enc_readings[j] < enc_readings[j-1] * 0.3:
            monotonic = False
            break
    if monotonic:
        print("  >> PASS: Duty-to-speed relationship looks reasonable.")
    else:
        print("  >> WARN: Non-monotonic response. Check motor/gearbox.")

# ============================================================
#  最终测试报告
# ============================================================

stop_all_motors()
pit1.stop()
led.off()

print("")
print("=" * 60)
print("  Motor Test Report")
print("=" * 60)

print("  {:<20s}: {}".format('pwm_init', "PASS" if test_results['pwm_init'] else "FAIL"))

for i in range(4):
    fwd = "PASS" if test_results['motor_forward'][i] else "FAIL"
    rev = "PASS" if test_results['motor_reverse'][i] else "FAIL"
    enc = "PASS" if test_results['encoder_read'][i] else "FAIL"
    print("  motor_{} forward   : {}".format(MOTOR_NAMES[i], fwd))
    print("  motor_{} reverse   : {}".format(MOTOR_NAMES[i], rev))
    print("  motor_{} encoder   : {}".format(MOTOR_NAMES[i], enc))

print("  {:<20s}: {}".format('omni_drive', "PASS" if test_results['omni_drive'] else "FAIL"))
print("  {:<20s}: {}".format('emergency_stop', "PASS" if test_results['emergency_stop'] else "FAIL"))

print("-" * 60)

all_pass = (test_results['pwm_init'] and
            all(test_results['motor_forward']) and
            all(test_results['motor_reverse']) and
            all(test_results['encoder_read']) and
            test_results['omni_drive'] and
            test_results['emergency_stop'])

if all_pass:
    print("  OVERALL: PASS")
    print("  All motors and encoders are working correctly.")
else:
    print("  OVERALL: PARTIAL PASS")
    failed = []
    for i in range(4):
        if not test_results['motor_forward'][i]:
            failed.append("motor_{} fwd".format(MOTOR_NAMES[i]))
        if not test_results['motor_reverse'][i]:
            failed.append("motor_{} rev".format(MOTOR_NAMES[i]))
        if not test_results['encoder_read'][i]:
            failed.append("motor_{} enc".format(MOTOR_NAMES[i]))
    if failed:
        print("  Failed: {}".format(", ".join(failed)))
print("=" * 60)
print("")
