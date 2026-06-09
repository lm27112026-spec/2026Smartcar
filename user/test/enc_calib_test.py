"""
enc_calib_test.py - 编码器标定测试（自包含版）
【功能】
  标定 ENC_SCALE 值：
  1. 以固定 PWM 驱动 4 个电机
  2. 记录编码器脉冲数
  3. 计算每圈脉冲数（ENC_SCALE）
【使用】
  运行脚本，拨动 SWITCH2 开始
  ⚠ 轮子需悬空，标记轮子起始位置
"""

import gc, time, math
from machine import *
from smartcar import *
from seekfree import *

time.sleep_ms(100)

print("=" * 60)
print("  Encoder Calibration Test (Self-Contained)")
print("=" * 60)
print("  REAL TYPE    : " + BOARD_TYPE)
print("  BOARD VERSION: " + BOARD_VERSION)
print("")

# 引脚定义
LED_PIN     = 'C4'
SWITCH2_PIN = 'D9'

led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)

# TB6612 电机引脚（自包含）
pwm_rf = PWM("C26", 15000, duty_u16=0)
pwm_lf = PWM("C24", 15000, duty_u16=0)
pwm_lb = PWM("C20", 15000, duty_u16=0)
pwm_rb = PWM("B26", 15000, duty_u16=0)

pin_d6  = Pin("D6",  Pin.OUT, value=0)
pin_d7  = Pin("D7",  Pin.OUT, value=0)
pin_d4  = Pin("D4",  Pin.OUT, value=0)
pin_d5  = Pin("D5",  Pin.OUT, value=0)
pin_c30 = Pin("C30", Pin.OUT, value=0)
pin_c31 = Pin("C31", Pin.OUT, value=0)
pin_c28 = Pin("C28", Pin.OUT, value=0)
pin_c29 = Pin("C29", Pin.OUT, value=0)

MOTOR_RF = (pwm_rf, pin_d6,  pin_d7)
MOTOR_LF = (pwm_lf, pin_d4,  pin_d5)
MOTOR_LB = (pwm_lb, pin_c30, pin_c31)
MOTOR_RB = (pwm_rb, pin_c28, pin_c29)

MOTOR_LIST = [MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB]

# 编码器（自包含）
encoder_rf = encoder('C1',  'C0',  capture_div=1)
encoder_lf = encoder('C3',  'C2',  capture_div=1)
encoder_lb = encoder('D14', 'D13', capture_div=1)
encoder_rb = encoder('D16', 'D15', capture_div=1)

ENC_LIST = [encoder_rf, encoder_lf, encoder_lb, encoder_rb]

ticker_flag = False
def ticker_handler(t):
    global ticker_flag
    ticker_flag = True

pit = ticker(1)
pit.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
pit.callback(ticker_handler)
pit.start(10)

# 电机控制函数
def set_motor(motor, duty_u16):
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

# 测试参数
TEST_PWM      = 15000
TEST_TIME_S   = 5.0
WHEEL_DIA_CM  = 6.5

# 主测试
print("-" * 60)
print("  测试参数：")
print("    PWM = {} ({:.0f}%)".format(TEST_PWM, TEST_PWM / 65535 * 100))
print("    时间 = {:.1f} 秒".format(TEST_TIME_S))
print("    轮子直径 = {} cm".format(WHEEL_DIA_CM))
print("-" * 60)
print("")
print("  拨动 SWITCH2 开始测试...")
print("")

prev_sw2 = switch2.value()
while switch2.value() == prev_sw2:
    time.sleep_ms(10)
time.sleep_ms(200)

print("  测试开始！电机启动...")
print("")

for enc in ENC_LIST:
    enc.get()

for m in MOTOR_LIST:
    set_motor(m, TEST_PWM)

total_counts = [0, 0, 0, 0]
start_ms = time.ticks_ms()
last_print_ms = 0

print("  时间    RF脉冲    LF脉冲    LB脉冲    RB脉冲")
print("  " + "-" * 55)

sw2_start = switch2.value()
while time.ticks_diff(time.ticks_ms(), start_ms) < TEST_TIME_S * 1000:
    if switch2.value() != sw2_start:
        break
    
    if ticker_flag:
        ticker_flag = False
        
        counts = [enc.get() for enc in ENC_LIST]
        for i in range(4):
            total_counts[i] += counts[i]
        
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 200:
            last_print_ms = now
            t = time.ticks_diff(now, start_ms) / 1000.0
            print("  {:.1f}s  {:>+6}  {:>+6}  {:>+6}  {:>+6}".format(
                t, total_counts[0], total_counts[1], total_counts[2], total_counts[3]))
    
    time.sleep_ms(5)
    gc.collect()

stop_all_motors()
pit.stop()

print("")
print("-" * 60)
print("  测试结束！总脉冲数：")
print("    RF: {}".format(total_counts[0]))
print("    LF: {}".format(total_counts[1]))
print("    LB: {}".format(total_counts[2]))
print("    RB: {}".format(total_counts[3]))
print("-" * 60)
print("")

wheel_circum_m = math.pi * WHEEL_DIA_CM / 100.0
print("  轮子周长 = {:.4f} m (直径 {} cm)".format(wheel_circum_m, WHEEL_DIA_CM))
print("")

print("  ENC_SCALE 计算（每圈脉冲数）：")
print("  请记录每个轮子转了多少圈：")
print("  ENC_SCALE = 总脉冲数 / 圈数")
print("")
N = 10
for i, name in enumerate(["RF", "LF", "LB", "RB"]):
    if total_counts[i] > 0:
        enc_scale = total_counts[i] / N
        print("    {} ENC_SCALE = {} / {} = {:.1f}".format(
            name, total_counts[i], N, enc_scale))
    else:
        print("    {} ENC_SCALE = 0 (无脉冲！)".format(name))

print("")
print("-" * 60)
print("  使用方法：")
print("  1. 运行前标记轮子位置")
print("  2. 拨动 SWITCH2 开始，电机转 5 秒")
print("  3. 数每个轮子转了几圈")
print("  4. ENC_SCALE = 总脉冲 / 圈数")
print("  5. 更新 motor.py 中的 ENC_SCALE")
print("-" * 60)
print("")
