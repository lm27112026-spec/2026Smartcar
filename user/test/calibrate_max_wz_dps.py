"""
calibrate_max_wz_dps.py — 标定 MAX_WZ_DPS

【原理】
  MAX_WZ_DPS = 实际角速度 (dps) / 归一化 wz

  让车以固定的 wz 值旋转，从陀螺仪读取实际 dps，计算比值。

【步骤】
  1. 把车放在地面，确保周围无障碍物
  2. 运行本脚本
  3. 车会以 wz=0.3 左旋约 3 秒，然后停车
  4. 串口打印出标定结果
  5. 把结果填入 imu_motion.py 和 control.py 的 MAX_WZ_DPS

【用法】
  拨动 SWITCH2 随时退出

【注意事项】
  - 确保陀螺仪零偏已校准（imu_motion.py 导入时会自动做）
  - 地面尽量平坦
  - 测试完立刻停车，防止线缆缠绕
"""

import gc, time
from machine import Pin
from smartcar import ticker
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker, ENC_SCALE, LED_PIN, SWITCH2_PIN,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
)
import imu_motion

# ============================================================
#  参数
# ============================================================
CALIB_WZ = 0.3           # 测试用的归一化 wz（建议 0.2~0.4，太小信噪比低，太大危险）
DURATION_S = 3.0         # 旋转持续时间（秒）
DT = 0.02                # 控制周期
PRINT_INTERVAL_MS = 100  # 打印间隔

# ============================================================
#  硬件
# ============================================================
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ============================================================
#  初始化
# ============================================================
stop_all()
enc_ticker.stop()
time.sleep_ms(50)

# 清空编码器缓冲
for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)

# 用 PIT2 做编码器采集（避免与 imu_motion 的 ticker 冲突）
pit_enc = ticker(2)
pit_enc.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
pit_enc.start(10)

# IMU 初始对准（读取一次建立 update_angle 基准）
d = imu_motion.imu.read()
imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
time.sleep_ms(50)

print("\n" + "=" * 60)
print("  MAX_WZ_DPS 标定")
print("=" * 60)
print("  目标 wz = {:.1f} (归一化值)".format(CALIB_WZ))
print("  持续时间 = {:.1f}s".format(DURATION_S))
print("  按 SWITCH2 随时退出")
print("=" * 60)
print("")

# ============================================================
#  标定运行
# ============================================================

dps_samples = []
start_ms = time.ticks_ms()
last_print_ms = start_ms

print("  {:>6s}  {:>10s}  {:>10s}".format("时间(s)", "wz_dps(°/s)", "yaw(°)"))

while True:
    now_ms = time.ticks_ms()
    elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

    # 退出条件
    if switch2.value() != state2:
        print("\n  SW2 退出")
        break
    if elapsed_s >= DURATION_S:
        break

    # ── 读 IMU 陀螺仪 ──
    d = imu_motion.imu.read()
    imu_motion.update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    actual_dps = imu_motion.get_angular_velocity()
    dps_samples.append(actual_dps)

    # ── 编码器速度（闭环需要）──
    raw_counts = get_encoder_counts()
    raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]

    # ── 闭环驱动 ──
    omni_drive_closed_loop(0, 0, CALIB_WZ, raw_speeds, DT)

    # ── 打印 ──
    if time.ticks_diff(now_ms, last_print_ms) >= PRINT_INTERVAL_MS:
        last_print_ms = now_ms
        print("  {:6.2f}  {:10.1f}  {:10.1f}".format(
            elapsed_s, actual_dps, imu_motion.yaw))

    time.sleep_ms(int(DT * 1000))
    gc.collect()

# ============================================================
#  停车
# ============================================================
omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
time.sleep_ms(100)
stop_all()
pit_enc.stop()
led.off()

# ============================================================
#  计算标定结果
# ============================================================
if len(dps_samples) > 10:
    # 去掉前 20% 的暂态数据（电机加速过程不平稳）
    steady_start = int(len(dps_samples) * 0.2)
    steady_samples = dps_samples[steady_start:]

    avg_dps = sum(steady_samples) / len(steady_samples)
    calculated_max_wz_dps = abs(avg_dps) / abs(CALIB_WZ)

    print("\n" + "=" * 60)
    print("  标定结果")
    print("=" * 60)
    print("  采样数:          {} (稳态 {} 个)".format(len(dps_samples), len(steady_samples)))
    print("  稳态平均 dps:    {:.1f} °/s".format(avg_dps))
    print("  MAX_WZ_DPS =    {:.1f}  (dps per unit wz)".format(calculated_max_wz_dps))
    print("")
    print("  请将以下值更新到两个文件：")
    print("    user/imu_motion.py  第 131 行:  MAX_WZ_DPS = {:.0f}".format(calculated_max_wz_dps))
    print("    user/control.py     第 27 行:   MAX_WZ_DPS = {:.0f}".format(calculated_max_wz_dps))
    print("")
    print("  建议将两个文件的值取整后保持一致。")
    print("=" * 60)
else:
    print("\n  采样不足，无法计算。")

print("\n=== Done ===")
