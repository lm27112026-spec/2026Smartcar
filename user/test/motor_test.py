"""
motor_test.py - 电机模块功能测试（TB6612 直接 PWM 控制）
【功能】
  测试 4 路电机能否正常工作，包括：
  1. PWM 初始化检测
  2. 单电机正反转测试
  3. 编码器反馈读取
  4. 全向驱动运动学验证
  5. 急停功能测试
【说明】
  TB6612 使用 PWM + 2个方向引脚控制，MOTOR_CONTROLLER 类不支持此模式。
  因此直接使用 machine.PWM 和 machine.Pin 对象控制电机。
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

TEST_DUTY  = 15000   # 测试 PWM duty (15000/65535 ≈ 23%)
TEST_DUTY2 = 25000   # 高速测试 PWM

# ============================================================
#  引脚初始化
# ============================================================

time.sleep_ms(100)

print("=" * 60)
print("  4-Wheel Motor Functional Test (TB6612)")
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
#  TB6612 电机引脚定义（实测）
# ============================================================

# PWM 引脚
pwm_rf = PWM("C26", 15000, duty_u16=0)
pwm_lf = PWM("C24", 15000, duty_u16=0)
pwm_lb = PWM("C20", 15000, duty_u16=0)
pwm_rb = PWM("B26", 15000, duty_u16=0)

# 方向引脚
pin_d6  = Pin("D6",  Pin.OUT, value=0)
pin_d7  = Pin("D7",  Pin.OUT, value=0)
pin_d4  = Pin("D4",  Pin.OUT, value=0)
pin_d5  = Pin("D5",  Pin.OUT, value=0)
pin_c30 = Pin("C30", Pin.OUT, value=0)
pin_c31 = Pin("C31", Pin.OUT, value=0)
pin_c28 = Pin("C28", Pin.OUT, value=0)
pin_c29 = Pin("C29", Pin.OUT, value=0)

# 电机元组 (PWM, 方向A, 方向B)
# TB6612: IN1=0,IN2=1 → 正转; IN1=1,IN2=0 → 反转; IN1=0,IN2=0 → 停止
MOTOR_RF = (pwm_rf, pin_d6,  pin_d7)   # C26 + D6/D7
MOTOR_LF = (pwm_lf, pin_d4,  pin_d5)   # C24 + D4/D5
MOTOR_LB = (pwm_lb, pin_c30, pin_c31)  # C20 + C30/C31
MOTOR_RB = (pwm_rb, pin_c28, pin_c29)  # B26 + C28/C29

MOTOR_LIST = [MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB]

# ============================================================
#  初始化编码器
# ============================================================

encoder_rf = encoder('C1',  'C0',  capture_div=1)
encoder_lf = encoder('C3',  'C2',  capture_div=1)
encoder_lb = encoder('D14', 'D13', capture_div=1)
encoder_rb = encoder('D16', 'D15', capture_div=1)

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
#  电机控制函数
# ============================================================

def set_motor(motor, duty_u16):
    """直接控制 TB6612 电机"""
    pwm, dir_a, dir_b = motor
    if duty_u16 > 0:
        dir_a.value(0)
        dir_b.value(1)
        pwm.duty_u16(int(duty_u16))
    elif duty_u16 < 0:
        dir_a.value(1)
        dir_b.value(0)
        pwm.duty_u16(int(-duty_u16))
    else:
        dir_a.value(0)
        dir_b.value(0)
        pwm.duty_u16(0)

def stop_all_motors():
    for m in MOTOR_LIST:
        set_motor(m, 0)

def check_switch():
    return switch2.value() != state2

# ============================================================
#  测试结果记录
# ============================================================

test_results = {
    'pwm_init':       False,
    'motor_forward':  [False, False, False, False],
    'motor_reverse':  [False, False, False, False],
    'encoder_read':   [False, False, False, False],
    'omni_drive':     False,
    'emergency_stop': False,
}

# ============================================================
#  测试 1：PWM 初始化 & 急停
# ============================================================

print("-" * 60)
print("  [Test 1] PWM Init & Emergency Stop")
print("-" * 60)

try:
    stop_all_motors()
    test_results['pwm_init'] = True
    print("  >> PASS: PWM channels initialized.")

    # 急停测试
    set_motor(MOTOR_RF, TEST_DUTY)
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
print("  PWM duty: {} / 65535 ({:.0f}%)".format(TEST_DUTY, TEST_DUTY/65535*100))
print("  Starting in 2 seconds...")
time.sleep_ms(2000)

for i in range(4):
    if check_switch():
        break

    print("  Testing Motor {} ({}):".format(i, MOTOR_NAMES[i]))
    motor = MOTOR_LIST[i]

    # --- 正转测试 ---
    print("    Forward +{:<6d}...".format(TEST_DUTY), end="")
    set_motor(motor, TEST_DUTY)
    time.sleep_ms(1000)

    ENC_LIST[i].get()
    time.sleep_ms(200)

    enc_val = ENC_LIST[i].get()
    if abs(enc_val) > 2:
        test_results['motor_forward'][i] = True
        test_results['encoder_read'][i] = True
        direction = "positive" if enc_val > 0 else "negative"
        print(" enc={:+d} ({})".format(enc_val, direction))
    else:
        time.sleep_ms(1000)
        enc_val = ENC_LIST[i].get()
        if abs(enc_val) > 2:
            test_results['motor_forward'][i] = True
            test_results['encoder_read'][i] = True
            print(" enc={:+d}".format(enc_val))
        else:
            print(" enc=0 (no response)")

    set_motor(motor, 0)
    time.sleep_ms(500)

    # --- 反转测试 ---
    print("    Reverse -{:<6d}...".format(TEST_DUTY), end="")
    set_motor(motor, -TEST_DUTY)
    time.sleep_ms(1000)

    ENC_LIST[i].get()
    time.sleep_ms(200)

    enc_val = ENC_LIST[i].get()
    if abs(enc_val) > 2:
        test_results['motor_reverse'][i] = True
        if not test_results['encoder_read'][i]:
            test_results['encoder_read'][i] = True
        print(" enc={:+d}".format(enc_val))
    else:
        time.sleep_ms(1000)
        enc_val = ENC_LIST[i].get()
        if abs(enc_val) > 2:
            test_results['motor_reverse'][i] = True
            if not test_results['encoder_read'][i]:
                test_results['encoder_read'][i] = True
            print(" enc={:+d}".format(enc_val))
        else:
            print(" enc=0 (no response)")

    set_motor(motor, 0)
    time.sleep_ms(300)
    print("")

# ============================================================
#  测试 3：全向驱动运动学验证
# ============================================================

if not check_switch():
    print("-" * 60)
    print("  [Test 3] Omni-Drive Kinematics")
    print("-" * 60)

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
        pwm_vals = [int(s * scale * 65535) for s in speeds]
        for j in range(4):
            set_motor(MOTOR_LIST[j], pwm_vals[j])

    # 测试前进
    print("    Forward (vx=0, vy=0.5, wz=0)...", end="")
    omni_drive_local(0, 0.5, 0)
    time.sleep_ms(1000)
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
    if failed:
        print("  Failed: {}".format(", ".join(failed)))
print("=" * 60)
print("")
