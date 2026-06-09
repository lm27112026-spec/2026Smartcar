"""
speed闭环_test.py - 速度闭环测试
【功能】
  测试 PI 速度闭环控制：
  1. 设定目标速度（归一化 0~1）
  2. 使用 omni_drive_closed_loop 驱动
  3. 实时显示目标速度 vs 实际速度
  4. 验证闭环控制稳定性
【使用】
  运行脚本，观察速度跟踪效果
  按 SWITCH2 可提前终止
  ⚠ 轮子需悬空
"""

import gc, time
from machine import *
from smartcar import *
from seekfree import *
from motor import (
    omni_drive_closed_loop, stop_all,
    get_encoder_speeds_filtered,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    WHEEL_PI, MAX_SPEED_MPS, CTRL_DT,
    LED_PIN, SWITCH2_PIN,
)

LED_PIN = 'C4'
SWITCH2_PIN = 'D9'

time.sleep_ms(100)

print("=" * 60)
print("  Speed Closed-Loop Test")
print("=" * 60)
print("  REAL TYPE    : " + BOARD_TYPE)
print("  BOARD VERSION: " + BOARD_VERSION)
print("")

led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ============================================================
#  编码器 ticker
# ============================================================

pit = ticker(1)
pit.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)

ticker_flag = False
def ticker_handler(t):
    global ticker_flag
    ticker_flag = True

pit.callback(ticker_handler)
pit.start(10)

# ============================================================
#  测试参数
# ============================================================

TARGET_SPEEDS = [0.0, 0.1, 0.2, 0.3, 0.4, 0.3, 0.2, 0.1, 0.0]  # 速度斜坡
STEP_TIME_S   = 3.0    # 每个速度台阶持续时间（秒）
PRINT_INTERVAL = 200    # 打印间隔（毫秒）

# ============================================================
#  主测试循环
# ============================================================

print("-" * 60)
print("  速度斜坡测试：0 → 0.1 → 0.2 → 0.3 → 0.4 → 0.3 → 0.2 → 0.1 → 0")
print("  每个台阶持续 {:.1f} 秒".format(STEP_TIME_S))
print("-" * 60)
print("")
print("  时间    目标Vx   实际RF    实际LF    实际LB    实际RB   平均")
print("  " + "-" * 70)

start_ms = time.ticks_ms()
last_print_ms = 0

for target_vx in TARGET_SPEEDS:
    if switch2.value() != state2:
        break
    
    step_start = time.ticks_ms()
    
    while time.ticks_diff(time.ticks_ms(), step_start) < STEP_TIME_S * 1000:
        if switch2.value() != state2:
            break
        
        if ticker_flag:
            ticker_flag = False
            
            # 读取编码器速度
            actual = get_encoder_speeds_filtered(CTRL_DT)
            
            # 闭环驱动
            omni_drive_closed_loop(target_vx, 0, 0, actual)
            
            # 打印
            now = time.ticks_ms()
            if time.ticks_diff(now, last_print_ms) >= PRINT_INTERVAL:
                last_print_ms = now
                t = time.ticks_diff(now, start_ms) / 1000.0
                avg = sum(actual) / 4.0
                print("  {:.1f}s  {:+.3f}  {:+.4f}  {:+.4f}  {:+.4f}  {:+.4f}  {:+.4f}".format(
                    t, target_vx, actual[0], actual[1], actual[2], actual[3], avg))
        
        time.sleep_ms(5)
        gc.collect()

# ============================================================
#  停止并报告
# ============================================================

stop_all()
pit.stop()
led.off()

print("")
print("=" * 60)
print("  Speed Closed-Loop Test Complete")
print("=" * 60)
print("")
print("  观察要点：")
print("  1. 实际速度是否能跟踪目标速度")
print("  2. 4 个轮子速度是否一致")
print("  3. 是否有明显的超调或振荡")
print("=" * 60)
print("")
