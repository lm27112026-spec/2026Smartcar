"""
main_test.py — 三段式路径测试
【路径】
  1) 前进 90cm
  2) 右平移 150cm（车头朝向不变）
  3) 前进 50cm
【方法】
  编码器里程计 + IMU 航向保持 + 距离 PID 闭环
  参考 10_position_closed_loop.py 的控制逻辑
【硬件】
  全向轮麦克纳姆底盘 + IMU660RX + 4路编码器
【退出】
  拨动 SWITCH2 随时中止当前段
"""

import gc, time
from machine import Pin
from pid import PID
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    reset_wheel_pi, reset_encoder_filter, enc_ticker,
    ENC_SCALE, LED_PIN, SWITCH2_PIN,
)
import imu_motion
from imu_motion import imu_get_safe
from key import capture, pet_watchdog

# ============================================================
#  硬件
# ============================================================
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
time.sleep_ms(100)  # 等引脚电平稳定
# state2 在 init 完成后读取（见主程序前）

# ============================================================
#  控制参数（沿用 10_position_closed_loop.py 验证过的值）
# ============================================================
DT = 0.02                    # 控制周期 20ms
PRINT_INTERVAL_MS = 200

# 距离 PID
DIST_KP = 1.0
DIST_KI = 0.5
DIST_OUTPUT_LIMIT = 0.30     # 最大速度

# 航向 PID
HEADING_KP = 0.08      # P 增益（0.15 过冲振荡 → 降到 0.08）
HEADING_KI = 0.03      # I 增益（0.005 太小无作用 → 提到 0.03）
HEADING_KD = 0.002     # D 增益（抑制振荡）
YAW_DEADBAND = 0.5     # 死区（0.3 太敏感 → 放宽到 0.5°）
WZ_LIMIT = 0.45        # max wz（0.5 纠正力过强 → 降到 0.45）

# 超时基准：按 0.3m/s 最大速度 + 50% 余量估算
TIMEOUT_BASE_S = 15


# ============================================================
#  辅助函数
# ============================================================

def _read_imu():
    """读取 IMU 并更新姿态角（使用 imu_get_safe 避免 SPI 总线冲突）"""
    d = imu_get_safe()
    if d is not None:
        imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])


def _reset_encoders():
    """排空编码器残余脉冲（手动 capture 即可，无需 ticker）"""
    for _ in range(5):
        get_encoder_counts()
        time.sleep_ms(10)


# ============================================================
#  核心：行驶到指定距离
# ============================================================

def move_distance(target_dist_m, direction='forward', heading_lock=None):
    """
    编码器里程计 + IMU 航向保持，精确行驶到目标距离。

    参数:
      target_dist_m  目标距离（米），如 0.90
      direction      运动方向:
                       'forward' — 前进（vx 方向）
                       'right'   — 右平移（vy 方向）
      heading_lock   锁定航向角（度），None 则锁定当前航向

    返回:
      最终航向角（度），供下一段续传
    """
    _reset_encoders()
    # 重置控制状态（防止上一段的积分/滤波器残留导致漂移）
    reset_wheel_pi()
    reset_encoder_filter()

    # 锁定航向
    if heading_lock is None:
        heading_lock = imu_motion.yaw
    target_heading = heading_lock

    # 超时 = 基础时间 + 按距离线性增加
    timeout_s = TIMEOUT_BASE_S + target_dist_m * 10

    # PID 初始化
    pid_dist = PID(kp=DIST_KP, ki=DIST_KI, kd=0.0,
                   integral_limit=0.5, output_limit=DIST_OUTPUT_LIMIT)

    heading_integral = 0.0
    prev_heading_deviation = 0.0

    # 编码器累计
    total_counts = [0, 0, 0, 0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms

    # 段标识
    if direction == 'forward':
        seg_label = "前进"
    elif direction == 'right':
        seg_label = "右移"
    else:
        seg_label = "左移"

    print("\n  [段] {} {:.0f}cm (航向 {:.1f}°)".format(
        seg_label, target_dist_m * 100, target_heading))
    print("  {:>5s}  {:>6s}  {:>7s} {:>7s} {:>6s}".format(
        "time", "dist", "speed", "yaw", "wz"))
    print("  " + "-" * 50)

    # ── 主控制循环 ──
    current_dist = 0.0  # 初始化，防止 SW2 提前 break 导致 NameError
    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        # 超时
        if elapsed_s > timeout_s:
            print("  TIMEOUT! ({:.0f}s)".format(timeout_s))
            break

        # 看门狗喂狗 + 按键采集
        capture()
        pet_watchdog()

        # SWITCH2 中止
        if switch2.value() != state2:
            print("  SWITCH2 aborted")
            break

        # ── 编码器读取 ──
        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]
        for i in range(4):
            total_counts[i] += raw_counts[i]

        wheel_dist = [abs(total_counts[i]) / ENC_SCALE[i] for i in range(4)]
        current_dist = sum(wheel_dist) / 4

        # ── IMU 航向 ──
        _read_imu()
        heading_deviation = target_heading - imu_motion.yaw
        while heading_deviation > 180:
            heading_deviation -= 360
        while heading_deviation < -180:
            heading_deviation += 360

        # 跨 ±180° 边界时重置积分（防止积分跳变）
        if abs(heading_deviation - prev_heading_deviation) > 180:
            heading_integral = 0.0

        # ── 航向 PID ──
        if abs(heading_deviation) > YAW_DEADBAND:
            heading_integral += heading_deviation * DT
            heading_integral = max(-2.0, min(heading_integral, 2.0))
            heading_derivative = (heading_deviation - prev_heading_deviation) / DT
            wz = (HEADING_KP * heading_deviation +
                  HEADING_KI * heading_integral +
                  HEADING_KD * heading_derivative)
            wz = max(-WZ_LIMIT, min(wz, WZ_LIMIT))
        else:
            wz = 0.0
            # 保留积分值不清零（清零会导致在死区边界反复振荡 → 漂移）

        prev_heading_deviation = heading_deviation  # 保存供下一帧 D 项使用

        # ── 距离 PID ──
        speed = pid_dist.compute(setpoint=target_dist_m,
                                 measurement=current_dist, dt=DT)
        # 最低速度防摩擦死区（仅未到目标时生效）
        if current_dist < target_dist_m and speed < 0.08:
            speed = 0.08

        # ── 闭环驱动（方向映射）──
        if direction == 'forward':
            omni_drive_closed_loop(speed, 0, wz, raw_speeds, DT)
        elif direction == 'right':
            omni_drive_closed_loop(0, speed, wz, raw_speeds, DT)
        else:  # left
            omni_drive_closed_loop(0, -speed, wz, raw_speeds, DT)

        # ── 定期打印 ──
        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
            last_print_ms = now_ms
            print("  {:5.1f}s  {:5.1f}cm  {:+.3f}  {:6.1f} {:+.3f}".format(
                elapsed_s, current_dist * 100, speed,
                imu_motion.yaw, wz))

        # ── 停车判断 ──
        if current_dist >= target_dist_m:
            print("  -> 目标到达 ({:.1f}cm)".format(current_dist * 100))
            break

        time.sleep_ms(int(DT * 1000))
        gc.collect()

    # ── 停止 & 报告 ──
    omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
    stop_all()

    distance_error = (current_dist - target_dist_m) * 100
    yaw_drift = imu_motion.yaw - target_heading
    while yaw_drift > 180:
        yaw_drift -= 360
    while yaw_drift < -180:
        yaw_drift += 360

    print("  完成: 实际 {:.1f}cm (误差 {:+.1f}cm), 偏航 {:+.1f}°".format(
        current_dist * 100, distance_error, yaw_drift))

    return imu_motion.yaw


# ============================================================
#  主程序
# ============================================================

# ── 启动安全 ──
stop_all()
time.sleep_ms(50)

# ── IMU 热身 & 陀螺仪校准 ──
print("IMU 初始化...")
for _ in range(10):
    _read_imu()
    time.sleep_ms(10)

initial_heading = imu_motion.yaw

# 所有硬件初始化完成后读取 SW2 初始状态
state2 = switch2.value()

print("\n" + "=" * 60)
print("  三段式路径测试")
print("  路径: 前进90cm -> 右移150cm -> 前进50cm")
print("  初始航向: {:.1f}°".format(initial_heading))
print("  SWITCH2 随时退出")
print("=" * 60)

# ── 段 1: 前进 90cm ──
heading_1 = move_distance(0.90, direction='forward', heading_lock=initial_heading)
time.sleep(0.5)

# ── 段 2: 右平移 150cm（车头朝向不变）──
heading_2 = move_distance(1.50, direction='right', heading_lock=heading_1)
time.sleep(0.5)

# ── 段 3: 前进 50cm ──
heading_3 = move_distance(0.50, direction='forward', heading_lock=heading_2)

# ── 总结 ──
total_yaw_drift = heading_3 - initial_heading
while total_yaw_drift > 180:
    total_yaw_drift -= 360
while total_yaw_drift < -180:
    total_yaw_drift += 360

print("\n" + "=" * 60)
print("  路径测试完成!")
print("  最终航向: {:.1f}° | 全程偏航: {:+.1f}°".format(heading_3, total_yaw_drift))
print("=" * 60)

led.off()
enc_ticker.start(10)  # 恢复编码器自动采集
