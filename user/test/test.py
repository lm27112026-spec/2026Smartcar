"""
test.py - 小车：右移30cm → 前进60cm → 向后转180° → 右移30cm
按下 SWITCH2 随时退出

【运动映射】（与 go_forward_1m.py / motion_test.py 一致）
  vx = +0.3 → 前进
  vy = +0.3 → 右移
  wz = +0.3 → 左旋（逆时针）

【依赖】
  motor.py（电机/编码器）、imu_motion.py（IMU yaw 姿态解算）
"""

import gc, time
from machine import Pin
from smartcar import ticker
from motor import (
    omni_drive, stop_all,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
    ENC_SCALE, LED_PIN, SWITCH2_PIN,
)
from imu_motion import imu, update_angle
import imu_motion

# ============================================================
#  一、初始化
# ============================================================
stop_all()
time.sleep_ms(50)

led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# 定时器（10ms 硬件捕获编码器脉冲，不影响 IMU 读取）
pit = ticker(1)
pit.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
pit.start(10)

# ============================================================
#  二、参数
# ============================================================
DRIVE_SPEED        = 0.3      # 开环驱动速度（归一化 0~1）
CONTROL_INTERVAL_S = 0.02     # 直线采样周期 20ms
OVERSHOOT_FRACTION = 0.10     # 10% 惯性超调补偿
TIMEOUT_S          = 30       # 超时保护
ROTATE_WZ          = 0.3      # 旋转角速度（左旋）

# ============================================================
#  三、编码器归零
# ============================================================
def zero_encoders():
    """连续读取清空编码器增量缓冲"""
    for _ in range(5):
        encoder_rf.get()
        encoder_lf.get()
        encoder_lb.get()
        encoder_rb.get()
        time.sleep_ms(10)

# ============================================================
#  四、直线移动（编码器里程计）
# ============================================================
def move_distance(vx, vy, target_m, label):
    """
    开环驱动 + 编码器里程计，到达目标距离后停机
    vx, vy:   运动方向（归一化 -1~1），vx=前进 vy=右移
    target_m: 目标距离（米）
    label:    串口标签
    返回 True=到达目标，False=SWITCH2/超时退出
    """
    zero_encoders()

    total_counts = [0, 0, 0, 0]
    start_ms = time.ticks_ms()

    omni_drive(vx, vy, 0)

    while True:
        if time.ticks_diff(time.ticks_ms(), start_ms) > TIMEOUT_S * 1000:
            print("Timeout! {:.0f}s elapsed.".format(TIMEOUT_S))
            stop_all()
            return False

        if switch2.value() != state2:
            print("SW2 stop")
            stop_all()
            return False

        counts = [
            encoder_rf.get(),
            encoder_lf.get(),
            encoder_lb.get(),
            encoder_rb.get(),
        ]
        for i in range(4):
            total_counts[i] += counts[i]

        wheel_dist = [
            abs(total_counts[i]) / abs(ENC_SCALE[i])
            for i in range(4)
        ]
        avg_dist = sum(wheel_dist) / 4

        print("{} dist={:.3f}m  goal={:.3f}m".format(
            label, avg_dist, target_m))

        if avg_dist >= target_m * (1.0 - OVERSHOOT_FRACTION):
            print(">>> {} done <<<".format(label))
            stop_all()
            time.sleep_ms(500)
            return True

        time.sleep_ms(int(CONTROL_INTERVAL_S * 1000))
        gc.collect()

# ============================================================
#  五、旋转 180°（IMU yaw 测量）
# ============================================================
def rotate_180(wz, label):
    """
    开环旋转 + IMU yaw 测量，旋转约 180° 后停机
    wz:    旋转方向（归一化 -1~1），+0.3=左旋
    label: 串口标签
    返回 True=到达目标，False=SWITCH2/超时退出
    """
    # 读取 IMU 刷新当前姿态
    d = imu.read()
    update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    start_yaw = imu_motion.yaw

    start_ms = time.ticks_ms()
    omni_drive(0, 0, wz)

    TARGET_DEG = 180.0

    while True:
        if time.ticks_diff(time.ticks_ms(), start_ms) > TIMEOUT_S * 1000:
            print("Timeout!")
            stop_all()
            return False

        if switch2.value() != state2:
            print("SW2 stop")
            stop_all()
            return False

        d = imu.read()
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

        delta = imu_motion.yaw - start_yaw
        delta = (delta + 180) % 360 - 180  # 包裹到 [-180, 180)

        print("{} delta={:.1f}°  target={:.0f}°".format(
            label, abs(delta), TARGET_DEG))

        if abs(delta) >= TARGET_DEG * (1.0 - OVERSHOOT_FRACTION):
            print(">>> {} done <<<".format(label))
            stop_all()
            time.sleep_ms(500)
            return True

        time.sleep_ms(20)
        gc.collect()

# ============================================================
#  六、主流程
# ============================================================
print("\n=== Test: Right → Forward → Rotate 180° → Right ===")
print("Drive: {:.1f}  Rotate: {:.1f}  Overshoot: {:.0f}%".format(
    DRIVE_SPEED, ROTATE_WZ, OVERSHOOT_FRACTION * 100))
print("Press SWITCH2 to exit at any time.\n")

running = True

# 第 1 步：向右平移 30cm
if running:
    running = move_distance(0, DRIVE_SPEED, 0.30, "[1/4 RIGHT 30cm]")

# 第 2 步：向前走 60cm
if running:
    running = move_distance(DRIVE_SPEED, 0, 0.60, "[2/4 FORWARD 60cm]")

# 第 3 步：向后转 180°
if running:
    running = rotate_180(ROTATE_WZ, "[3/4 ROTATE 180°]")

# 第 4 步：再向右走 30cm
if running:
    move_distance(0, DRIVE_SPEED, 0.30, "[4/4 RIGHT 30cm]")

# ============================================================
#  七、清理
# ============================================================
led.off()
pit.stop()
stop_all()
print("=== Done ===")
