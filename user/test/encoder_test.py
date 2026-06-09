"""
encoder_test.py - 编码器读数测试
【功能】
  测试 4 路编码器能否正确读取脉冲：
  1. 静止时编码器读数应为 0
  2. 单电机正转时对应编码器应有正/负脉冲
  3. 编码器计数方向是否与电机方向匹配
【使用】
  运行脚本，观察编码器读数
  按 SWITCH2 可提前终止测试
  ⚠ 轮子需悬空
"""

import time
from machine import *
from smartcar import *
from seekfree import *

time.sleep_ms(100)

print("=" * 60)
print("  Encoder Read Test")
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
#  编码器初始化
# ============================================================

encoder_rf = encoder('C1',  'C0',  capture_div=1)
encoder_lf = encoder('C3',  'C2',  capture_div=1)
encoder_lb = encoder('D14', 'D13', capture_div=1)
encoder_rb = encoder('D16', 'D15', capture_div=1)

ENC_LIST = [encoder_rf, encoder_lf, encoder_lb, encoder_rb]
ENC_NAMES = ['RF', 'LF', 'LB', 'RB']

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
#  电机引脚定义（TB6612）
# ============================================================

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

# ============================================================
#  电机控制函数
# ============================================================

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

def stop_all():
    for m in MOTOR_LIST:
        set_motor(m, 0)

def check_switch():
    return switch2.value() != state2

# ============================================================
#  测试 1：静止时编码器读数
# ============================================================

print("-" * 60)
print("  [Test 1] Static Encoder Reading")
print("-" * 60)
print("  确保轮子静止，观察编码器读数...")
print("")

# 清空编码器缓冲
for _ in range(5):
    for enc in ENC_LIST:
        enc.get()
    time.sleep_ms(10)

# 读取静止状态下的编码器值
static_values = [enc.get() for enc in ENC_LIST]
print("  静止编码器读数: [{:+d}, {:+d}, {:+d}, {:+d}]".format(*static_values))

all_zero = all(abs(v) < 3 for v in static_values)
if all_zero:
    print("  >> PASS: 静止时编码器读数接近 0")
else:
    print("  >> WARN: 静止时编码器有残余读数，可能有干扰")
print("")

if check_switch():
    stop_all()
    pit1.stop()
    raise SystemExit(0)

# ============================================================
#  测试 2：单电机正转时编码器读数
# ============================================================

print("-" * 60)
print("  [Test 2] Single Motor Forward + Encoder Reading")
print("-" * 60)
print("  ⚠ 轮子需悬空！")
print("  每个电机正转 1 秒，记录编码器脉冲")
print("")

TEST_DUTY = 20000  # 30% duty

results = []

for i in range(4):
    if check_switch():
        break
    
    motor = MOTOR_LIST[i]
    name = ENC_NAMES[i]
    
    print("  Motor {} 正转...".format(name), end="")
    
    # 清空编码器
    for enc in ENC_LIST:
        enc.get()
    time.sleep_ms(50)
    
    # 正转 1 秒，期间持续读取编码器
    set_motor(motor, TEST_DUTY)
    time.sleep_ms(500)
    
    # 中途读取一次
    mid_vals = [enc.get() for enc in ENC_LIST]
    time.sleep_ms(500)
    
    # 停止后读取最终值
    set_motor(motor, 0)
    time.sleep_ms(200)
    
    enc_vals = [enc.get() for enc in ENC_LIST]
    results.append(enc_vals)
    
    print(" 中途=[{:+d},{:+d},{:+d},{:+d}] 最终=[{:+d},{:+d},{:+d},{:+d}]".format(
        mid_vals[0], mid_vals[1], mid_vals[2], mid_vals[3],
        enc_vals[0], enc_vals[1], enc_vals[2], enc_vals[3]))
    time.sleep_ms(500)

print("")
print("  汇总结果：")
print("  | 电机 | RF脉冲 | LF脉冲 | LB脉冲 | RB脉冲 |")
print("  |------|--------|--------|--------|--------|")
for i, name in enumerate(ENC_NAMES):
    if i < len(results):
        print("  | {} | {:+d} | {:+d} | {:+d} | {:+d} |".format(
            name, results[i][0], results[i][1], results[i][2], results[i][3]))

print("")
print("  判断编码器方向是否正确：")
for i, name in enumerate(ENC_NAMES):
    if i < len(results):
        own_enc = results[i][i]  # 第i个电机正转时，第i个编码器的读数
        if own_enc > 50:
            print("    {} 正转 → {} 编码器正计数 ({:+d}) ✓".format(name, name, own_enc))
        elif own_enc < -50:
            print("    {} 正转 → {} 编码器负计数 ({:+d}) → 需要翻转符号".format(name, name, own_enc))
        else:
            print("    {} 正转 → {} 编码器无响应 ({:+d}) ✗".format(name, name, own_enc))

# ============================================================
#  测试 3：编码器连续读数稳定性
# ============================================================

if not check_switch():
    print("")
    print("-" * 60)
    print("  [Test 3] Encoder Stability (2 seconds)")
    print("-" * 60)
    print("  同时驱动 4 个电机，观察编码器读数稳定性")
    print("")
    
    # 4 个电机同时正转
    for m in MOTOR_LIST:
        set_motor(m, TEST_DUTY)
    
    print("  时间   RF脉冲   LF脉冲   LB脉冲   RB脉冲")
    print("  " + "-" * 50)
    
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < 2000:
        if check_switch():
            break
        
        if ticker_flag:
            ticker_flag = False
            t = time.ticks_diff(time.ticks_ms(), start)
            vals = [enc.get() for enc in ENC_LIST]
            print("  {:>4d}ms {:+d}   {:+d}   {:+d}   {:+d}".format(
                t, vals[0], vals[1], vals[2], vals[3]))
        
        time.sleep_ms(50)
    
    stop_all()
    time.sleep_ms(300)

# ============================================================
#  完成
# ============================================================

stop_all()
pit1.stop()
led.off()

print("")
print("=" * 60)
print("  Encoder Test Complete")
print("=" * 60)
print("")
print("  根据测试结果，需要确认：")
print("  1. 静止时编码器读数是否接近 0")
print("  2. 每个电机正转时，对应编码器的脉冲符号是否正确")
print("  3. 如果编码器符号相反，需要调整 ENC_SCALE 的正负号")
print("=" * 60)
print("")
