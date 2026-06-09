"""
go_forward_1m.py — 编码器里程计前进 0.255 米

【用途】
  使用 4 路编码器脉冲累加计算行驶距离，开环驱动全向轮机器人
  前进 0.255 米（25.5cm）后自动停止。

【工作流程】
  1. 初始化 LED/SWITCH2/定时器
  2. 编码器归零（多次读取清空增量缓冲）
  3. omni_drive(vx=0.3) 开环前进
   4. 每 20ms 累加 4 路编码器脉冲 → abs(count)/abs(ENC_SCALE) → 距离（OVERSHOOT_FRACTION 比例补偿）
  5. 平均距离 ≥ 0.15m 或 SWITCH2 按下或超时 30s → 停止

【依赖】
  motor.py（omni_drive, stop_all, encoder_rf/lf/lb/rb, ENC_SCALE, LED_PIN, SWITCH2_PIN）

【使用方法】
  将此文件烧录至 RT1021-MicroPython 开发板，上电自动运行。
"""

import gc
import time
from machine import Pin
from smartcar import ticker
from motor import (
    omni_drive, stop_all, get_encoder_counts,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    ENC_SCALE, LED_PIN, SWITCH2_PIN,
)

# ============================================================
#  一、启动安全：确保所有电机处于停止状态
# ============================================================
stop_all()
time.sleep_ms(50)

# ============================================================
#  二、GPIO 初始化
# ============================================================
led     = Pin(LED_PIN, Pin.OUT, value=True)          # LED 指示运行
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)  # SWITCH2 急停按键
state2  = switch2.value()                             # 保存初始按键状态

# ============================================================
#  三、定时器初始化（10ms 硬件捕获编码器脉冲）
# ============================================================
pit = ticker(1)
pit.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
pit.start(10)  # 10ms 硬件捕获周期

# ============================================================
#  四、配置常量
# ============================================================
TARGET_DIST_M     = 0.15   # 目标前进距离（米）
DRIVE_SPEED       = 0.3     # 开环驱动速度（归一化 0~1）
CONTROL_INTERVAL_S = 0.02   # 控制/采样周期（20ms）
TIMEOUT_S         = 30      # 超时保护（秒）

# -----------------------------------------------------------
#  惯性超调补偿方式
#  实测发现超调量随目标距离增加而增大（并非固定值）。
#  改用比例补偿：超调 ≈ 目标距离 × 10%（以 25.5cm 为准校准）
#  这样 15cm / 20cm / 25cm 都能保持较好精度。
#  如果换了地面材质或速度，重跑标定。
# -----------------------------------------------------------
OVERSHOOT_FRACTION = 0.10     # 超调比例（以 25.5cm 为准校准）

# -----------------------------------------------------------
#  校准系数 CALIB_FACTOR
#  ENC_SCALE 是出厂标定值，你车上的实际脉冲/米可能不同。
#  如果实际跑出的距离是目标距离的 N 倍，就把 CALIB_FACTOR 设为 N。
#  例如：目标 25.5cm，实测跑了 50cm（约 2x），设 CALIB_FACTOR = 2.0
#        目标 25.5cm，实测跑了 12cm（约 0.5x），设 CALIB_FACTOR = 0.5
#  校准方法：运行 encoder_calibrate_by_tape.py 可精确标定 ENC_SCALE
# -----------------------------------------------------------
CALIB_FACTOR = 1.0         # ← 实测偏差倍数，根据实际调整

# ============================================================
#  五、编码器归零
#     连续读取 5 次，清空编码器增量缓冲
# ============================================================
print("Zeroing encoders...")
for _ in range(5):
    get_encoder_counts()
    time.sleep_ms(10)

# ============================================================
#  六、主循环：前进 + 编码器里程计
# ============================================================
print("\n=== Go Forward {:.0f}cm ===".format(TARGET_DIST_M * 100))
print("Drive speed: {:.1f}, Control interval: {:.0f}ms".format(
    DRIVE_SPEED, CONTROL_INTERVAL_S * 1000))
print("Toggle SWITCH2 or wait {:.0f}s timeout to stop.\n".format(TIMEOUT_S))

# 开环前进（vx=DRIVE_SPEED, vy=0, wz=0）
omni_drive(DRIVE_SPEED, 0, 0)

# 四轮脉冲累加器 [rf, lf, lb, rb]
total_counts = [0, 0, 0, 0]
start_ms = time.ticks_ms()
led.on()

while True:
    # ---- 超时检测 ----
    if time.ticks_diff(time.ticks_ms(), start_ms) > TIMEOUT_S * 1000:
        print("Timeout! {:.0f}s elapsed.".format(TIMEOUT_S))
        break

    # ---- SWITCH2 急停检测 ----
    if switch2.value() != state2:
        print("SW2 stop — button toggled.")
        break

    # ---- 读取 4 路编码器增量脉冲（统一走 get_encoder_counts，自动处理符号）----
    counts = get_encoder_counts()

    # ---- 累加脉冲 ----
    for i in range(4):
        total_counts[i] += counts[i]

    # ---- 脉冲 → 距离（米），用 abs 处理符号差异 ----
    wheel_dist = [
        abs(total_counts[i]) / (abs(ENC_SCALE[i]) * CALIB_FACTOR)
        for i in range(4)
    ]

    # ---- 四轮平均距离 ----
    if all(ENC_SCALE):
        avg_dist = sum(wheel_dist) / 4
    else:
        avg_dist = 0.0

    # ---- 串口打印进度 ----
    print("dist={:.3f}m  counts={}  goal={:.3f}m  overshoot={:.1f}%".format(
        avg_dist, total_counts, TARGET_DIST_M, OVERSHOOT_FRACTION * 100))

    # ---- 目标距离到达判断（按比例提前发停车指令）----
    if avg_dist >= (TARGET_DIST_M * (1.0 - OVERSHOOT_FRACTION)):
        print(">>> Reached target {:.0f}cm! <<<".format(TARGET_DIST_M * 100))
        break

    # ---- 等待下一个控制周期 ----
    time.sleep_ms(int(CONTROL_INTERVAL_S * 1000))
    gc.collect()

# ============================================================
#  七、停机清理
# ============================================================
omni_drive(0, 0, 0)   # 清零运动指令
stop_all()             # 硬件急停（所有电机方向引脚+PWM 置 0）
pit.stop()             # 停止定时器
led.off()              # LED 熄灭
print("Stopped.")
