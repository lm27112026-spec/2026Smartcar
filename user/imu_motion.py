"""
imu_motion.py - 姿态解算层
SPI模式
【层】姿态解算层
【功能】
  - 读取 IMU660RX 加速度计 & 陀螺仪原始数据
  - 互补滤波解算 roll、pitch、yaw（航向角）
  - 提供 drive_distance() 带航向保持的直线行驶
【依赖】motor.py（omni_drive）、pid.py（PID）
【使用】
  from imu_motion import update_angle, yaw, drive_distance
"""

import gc, time, math
from machine import *
from smartcar import *
from seekfree import *
from motor import omni_drive, SWITCH2_PIN, encoder_rf, encoder_lf, encoder_lb, encoder_rb, ENC_SCALE
from pid import PID

# ============================================================
#  一、IMU 常数 & 初始化
# ============================================================

ACCEL_SENSITIVITY = 4096
GYRO_SENSITIVITY = 14.2

HEADING_KP     = 0.10     # P 增益
YAW_DEADBAND   = 1.0      # 死区（度）：<1° 不纠偏
WZ_LIMIT       = 0.3      # wz 限幅
CTRL_DT        = 0.02     # 控制周期 20ms


time.sleep_ms(100)
imu = IMU660RX()
imu_data = imu.read()

# ============================================================
#  陀螺仪零偏校准（启动时静止采样）
# ============================================================

GYRO_CALIB_SAMPLES = 500

# 延时 1 秒等待陀螺仪稳定（ticker 启动瞬间可能有数据跳变）
time.sleep_ms(1000)

print("Calibrating gyro zero offset ({} samples)...".format(GYRO_CALIB_SAMPLES))

_gx_sum = 0
_gy_sum = 0
_gz_sum = 0
for _ in range(GYRO_CALIB_SAMPLES):
    d = imu.read()
    _gx_sum += d[3]
    _gy_sum += d[4]
    _gz_sum += d[5]
    time.sleep_ms(2)

gyro_offset_x = _gx_sum / GYRO_CALIB_SAMPLES
gyro_offset_y = _gy_sum / GYRO_CALIB_SAMPLES
gyro_offset_z = _gz_sum / GYRO_CALIB_SAMPLES
print("gyro offset: gx={:.2f}  gy={:.2f}  gz={:.2f}".format(
    gyro_offset_x, gyro_offset_y, gyro_offset_z))

# ============================================================
#  IMU Ticker 自动采集（PIT3，不与 key/enc/watchdog 冲突）
#  文档推荐方式：Ticker 绑定 capture_list，主循环用 imu.get() 读缓冲区
#  注意：imu.get() 在 ticker 停止/缓冲区过期时可能回退到 SPI 直读，
#        而 SPI 驱动在事务期间屏蔽全局中断。详见 main.py 的 ticker 管理。
# ============================================================
from smartcar import ticker as _imu_ticker_cls
_imu_pit = _imu_ticker_cls(3)   # PIT3：不与 PIT0(系统)/PIT1(编码器) 冲突
                                  # 看门狗已改用 machine.WDT()，不占用 PIT
_imu_pit.capture_list(imu)
_imu_pit.start(10)               # 10ms 采集周期 = 100Hz


def stop_imu_ticker():
    """停止 PIT3 IMU 自动采集 — 在 VISUAL_TRACK 等模式下切换到手动 imu.read()"""
    global _imu_pit
    try:
        _imu_pit.stop()
    except Exception:
        pass


def start_imu_ticker():
    """恢复 PIT3 IMU 自动采集（10ms 周期）"""
    global _imu_pit
    try:
        _imu_pit.start(10)
    except Exception:
        pass


def imu_read_safe():
    """安全读取 IMU 原始数据。
    封装 imu.read() 的 try-except，返回 None 表示读取失败。
    注意：imu.read() 内部做 SPI 事务。若 SPI 被电机噪声破坏，
          smartcar 库驱动在 __disable_irq() 后陷入轮询死等，
          此 try-except 无法捕获。此时由 key.py 的 machine.WDT()
          硬件看门狗在 3s 后强制复位 MCU（不受全局中断关闭影响）。
    """
    try:
        d = imu.read()
        if d is None or len(d) < 6:
            return None
        # 检查是否有无效值
        for v in d:
            if v is None:
                return None
        return d
    except Exception:
        return None


def imu_get_safe():
    """安全读取 IMU 缓冲区数据。
    封装 imu.get() 的 try-except，返回 None 表示读取失败。
    注意：imu.get() 在 ticker 停止时可能回退到 SPI 直读（同 imu.read()），
          此时同样可能触发 __disable_irq() 后挂死。由 machine.WDT() 恢复。
    """
    try:
        d = imu.get()
        if d is None or len(d) < 6:
            return None
        for v in d:
            if v is None:
                return None
        return d
    except Exception:
        return None

# ============================================================
#  二、姿态角解算（互补滤波）
# ============================================================

roll = pitch = yaw = 0.0
last_time = 0
filter_alpha = 0.98

# 陀螺仪 Z 轴低通滤波（平滑噪声数据）
# 运动中陀螺仪有大量电机干扰噪声，需要强滤波
gz_filtered = 0.0
gz_filter_alpha = 0.5  # 更强抑制（0.3=超强，0.5=强，0.7=中等，0.9=弱）

# ── gz 中值滤波：3 样本滑动窗口剔除单帧 EMI 尖峰 ──
_gz_hist = [0.0, 0.0, 0.0]
_gz_hist_idx = 0

# 最新滤波后的 Z 轴角速度 (dps)，供角速度闭环使用
_wz_dps = 0.0


def update_angle(ax, ay, az, gx, gy, gz):
    """
    互补滤波更新 roll/pitch/yaw
    ax, ay, az: 加速度计原始值
    gx, gy, gz: 陀螺仪原始值
    """
    global roll, pitch, yaw, last_time, gz_filtered, _wz_dps
    global _gz_hist, _gz_hist_idx
    now = time.ticks_ms()
    if last_time == 0:
        last_time = now
        gz_filtered = gz  # 首次初始化
        _gz_hist = [gz, gz, gz]  # 初始化中值窗
        return
    dt = (now - last_time) * 0.001
    last_time = now

    if dt > 0.5:
        dt = 0.05

    ax_g = ax / ACCEL_SENSITIVITY
    ay_g = ay / ACCEL_SENSITIVITY
    az_g = az / ACCEL_SENSITIVITY

    pitch_a = math.atan2(-ax_g, math.sqrt(ay_g**2 + az_g**2)) * 180.0 / math.pi
    roll_a  = math.atan2(ay_g, az_g) * 180.0 / math.pi

    gx_dps = (gx - gyro_offset_x) / GYRO_SENSITIVITY
    gy_dps = (gy - gyro_offset_y) / GYRO_SENSITIVITY

    # ── gz 中值滤波：3 样本窗口剔除电机 EMI 单帧尖峰 ──
    _gz_hist[_gz_hist_idx] = gz
    _gz_hist_idx = (_gz_hist_idx + 1) % 3
    gz_med = sorted(_gz_hist)[1]  # 取中值，自动丢弃极值

    # 对中值后的 gz 进行低通滤波
    gz_filtered = gz_filter_alpha * gz_filtered + (1 - gz_filter_alpha) * gz_med
    gz_dps = (gz_filtered - gyro_offset_z) / GYRO_SENSITIVITY
    _wz_dps = gz_dps  # 供外部角速度闭环使用

    roll  = filter_alpha * (roll  + gx_dps * dt) + (1 - filter_alpha) * roll_a
    pitch = filter_alpha * (pitch + gy_dps * dt) + (1 - filter_alpha) * pitch_a
    yaw  += gz_dps * dt
    yaw = ((yaw + 180) % 360) - 180


# ============================================================
#  三、角速度闭环控制（IMU 陀螺仪 Z 轴反馈）
# ============================================================

# 归一化 wz → dps 换算因子：wz=1.0 时车体旋转角速度 (deg/s)
# 需要实测标定：让车以固定 wz 旋转，读取陀螺仪实际 dps
MAX_WZ_DPS = 384.0  # 标定值：wz=0.3 时实测 115.3 dps → 115.3/0.3 = 384

# 角速度闭环 PID（工作在 dps 空间）
ANG_VEL_KP = 0.4          # P 增益（从 0.5 调高，加快响应）
ANG_VEL_KI = 0.01          # I 增益（从 0.3 调高，更快消除静差）
ANG_VEL_KD = 0.0
ANG_VEL_FF = 0.92         # 前馈系数（从 0.85 调高，正转偏低的问题）
ANG_VEL_OUTPUT_LIMIT = 300   # PID 输出限幅 (dps)
ANG_VEL_INTEGRAL_LIMIT = 150  # 积分限幅 (dps·s)（给 KI 更多空间）

_ang_vel_pid = PID(kp=ANG_VEL_KP, ki=ANG_VEL_KI, kd=ANG_VEL_KD,
                   integral_limit=ANG_VEL_INTEGRAL_LIMIT,
                   output_limit=ANG_VEL_OUTPUT_LIMIT)


def get_angular_velocity():
    """返回当前 Z 轴角速度 (dps)"""
    return _wz_dps


def angular_velocity_control(target_dps, actual_dps, dt):
    """
    角速度 PID 闭环 + 前馈
    target_dps: 目标角速度 (deg/s)
    actual_dps: 实际角速度 (deg/s)，来自陀螺仪
    dt:         控制周期 (s)
    返回:       wz 归一化值 (-1~1)，可直接喂给 omni_drive_closed_loop
    """
    # 前馈
    ff = target_dps * ANG_VEL_FF
    # PID 反馈
    fb = _ang_vel_pid.compute(target_dps, actual_dps, dt)
    # 合成 dps
    wz_dps = ff + fb
    wz_dps = max(-ANG_VEL_OUTPUT_LIMIT, min(wz_dps, ANG_VEL_OUTPUT_LIMIT))
    # 转换为归一化 wz
    wz = wz_dps / MAX_WZ_DPS
    return max(-1.0, min(wz, 1.0))


def reset_ang_vel_pid():
    """重置角速度 PID 积分"""
    _ang_vel_pid.reset()


def set_ang_vel_pid(kp=None, ki=None, kd=None):
    """在线调整角速度 PID 增益"""
    if kp is not None:
        _ang_vel_pid.kp = kp
    if ki is not None:
        _ang_vel_pid.ki = ki
    if kd is not None:
        _ang_vel_pid.kd = kd


# ============================================================
#  四、运动控制（航向保持行驶）
# ============================================================



def drive_distance(speed, target_angle, max_dist=999.0, timeout_s=999.0):
    """
    直线行驶（P-only 航向保持 + 编码器里程计）
    speed:       前进速度 (0~1)
    target_angle: 行驶方向（度），0=前进（同 omni_drive 语义，传给 vy）
    max_dist:     目标距离（米），到达自动停
    timeout_s:    超时自动停
    按 SWITCH2 可提前退出
    """
    led = Pin('C4', Pin.OUT, value=True)
    switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
    state2 = switch2.value()
    
    # ===== 启动 ticker（编码器）=====
    pit_enc = ticker(1)
    pit_enc.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
    pit_enc.start(10)

    
    # 清空残余值
    for _ in range(5):
        encoder_rf.get(); encoder_lf.get(); encoder_lb.get(); encoder_rb.get()
        imu.get()
        time.sleep_ms(5)

    # 锁定起始航向
    d = imu.get()
    update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    target_heading = yaw

    # 编码器里程计初始化
    total_counts = [0, 0, 0, 0]
    start_ms = time.ticks_ms()

    while True:
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0

        # 退出条件
        if switch2.value() != state2:
            break
        if elapsed > timeout_s:
            break

        # 编码器累计距离
        counts = [encoder_rf.get(), encoder_lf.get(), encoder_lb.get(), encoder_rb.get()]
        for i in range(4):
            total_counts[i] += counts[i]

        wheel_dist = [
            abs(total_counts[i]) / abs(ENC_SCALE[i])
            for i in range(4) if ENC_SCALE[i] != 0
        ]
        avg_dist = sum(wheel_dist) / len(wheel_dist) if wheel_dist else 0.0

        if avg_dist >= max_dist:
            pit_enc.stop()
            omni_drive(0, 0, 0)
            return True

        # IMU 更新航向
        d = imu.get()
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

        # P 控制
        error = target_heading - yaw
        while error > 180: error -= 360
        while error < -180: error += 360

        if abs(error) > YAW_DEADBAND:
            wz = error * HEADING_KP
            wz = max(-WZ_LIMIT, min(wz, WZ_LIMIT))
        else:
            wz = 0.0

        omni_drive(speed, target_angle, wz)
        led.toggle()
        time.sleep_ms(20)

    # 超时/手动退出
    pit_enc.stop()
    omni_drive(0, 0, 0)
    return False




