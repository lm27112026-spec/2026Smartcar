"""
imu.py - IMU660RX 实时角度读取与校准模块
【层】传感器驱动层（替代 imu_motion.py 的 IMU 部分）
【功能】
  - IMU660RX 加速度计 & 陀螺仪原始数据读取
  - 互补滤波解算 roll、pitch、yaw（航向角）
  - 启动时自动陀螺仪零偏校准
  - 运行时归零参考校准（set_zero_reference）
  - PIT ticker 自动采集（可配置通道）
【依赖】machine, smartcar, seekfree（硬件库）
【使用】
  from imu import IMU
  imu = IMU()
  imu.start()
  while True:
      imu.update()
      roll, pitch, yaw = imu.get_angles()
      print("angle = roll:{:>6.1f}, pitch:{:>6.1f}, yaw:{:>6.1f}".format(roll, pitch, yaw))
【校准流水线】
  imu.read() → 减去 gyro_offset → gz 低通滤波 → 互补滤波 → 减去 zero_reference → 输出
"""
import gc, time, math
from machine import Pin
from smartcar import ticker as _ticker_cls
from seekfree import IMU660RX


# ═══════════════════════════════════════════════════════════════
#  常数（与 imu_motion.py 保持一致，可被构造函数覆盖）
# ═══════════════════════════════════════════════════════════════

ACCEL_SENSITIVITY = 4096       # LSB/g（±8g 量程）
GYRO_SENSITIVITY  = 14.2       # LSB/(deg/s)（实战标定值；±2000dps 手册值为 16.4）

FILTER_ALPHA   = 0.98          # 互补滤波器 α（陀螺仪权重）
GZ_LPF_ALPHA   = 0.70          # Z 轴陀螺仪低通滤波器 α

GYRO_CALIB_SAMPLES = 500       # 陀螺仪零偏校准采样数
CALIB_DELAY_MS     = 2         # 校准间隔 (ms)

YAW_WRAP_MIN = -180.0          # 偏航角折叠范围 [-180, 180)
YAW_WRAP_MAX =  180.0


# ═══════════════════════════════════════════════════════════════
#  工具函数
# ═══════════════════════════════════════════════════════════════

def _wrap_angle(angle, low=-180.0, high=180.0):
    """将角度折叠到 [low, high) 区间"""
    span = high - low
    return ((angle - low) % span) + low


# ═══════════════════════════════════════════════════════════════
#  IMU 类
# ═══════════════════════════════════════════════════════════════

class IMU:
    """
    IMU660RX 传感器封装：实时角度读取 + 校准

    参数：
        pit_channel      : PIT 定时器通道（默认 3，与 imu_motion.py 一致）
        period_ms        : 采集周期 (ms)，默认 10ms = 100Hz
        capture_div      : 采集分频，默认 1（每次触发采集）
        accel_sensitivity: 加速度计灵敏度 LSB/g
        gyro_sensitivity : 陀螺仪灵敏度 LSB/(deg/s)，实战标定值 14.2
        filter_alpha     : 互补滤波器 α（越大越信任陀螺仪）
        gz_lpf_alpha     : Z 轴陀螺仪低通滤波器 α
        calibrate_on_init: 是否在 __init__ 中自动校准（默认 True）
        calib_samples    : 陀螺仪零偏校准采样数
        imu_type         : IMU 型号 IMU660RX.TYPE_x (AUTO, RA, RB, RC)
        imu_instance     : 外部 IMU660RX 实例（若提供则不创建新对象）
    """

    # ── 属性（类型提示仅作文档用）────────────────────────────

    roll:  float = 0.0
    pitch: float = 0.0
    yaw:   float = 0.0

    gyro_offset_x: float = 0.0
    gyro_offset_y: float = 0.0
    gyro_offset_z: float = 0.0

    ref_roll:  float = 0.0
    ref_pitch: float = 0.0
    ref_yaw:   float = 0.0

    gz_filtered: float = 0.0
    _wz_dps:     float = 0.0
    _wx_dps:     float = 0.0
    _wy_dps:     float = 0.0

    _last_time: int = 0
    _imu_data:  list = None     # 绑定到 imu.get() 的缓冲区引用

    def __init__(self,
                 pit_channel: int = 3,
                 period_ms: int = 10,
                 capture_div: int = None,
                 accel_sensitivity: float = ACCEL_SENSITIVITY,
                 gyro_sensitivity: float = GYRO_SENSITIVITY,
                 filter_alpha: float = FILTER_ALPHA,
                 gz_lpf_alpha: float = GZ_LPF_ALPHA,
                 calibrate_on_init: bool = True,
                 calib_samples: int = GYRO_CALIB_SAMPLES,
                 imu_type=None,
                 imu_instance=None):

        # ── 参数存储 ──
        self._pit_channel = pit_channel
        self._period_ms   = period_ms
        self._capture_div = capture_div
        self.accel_sensitivity = accel_sensitivity
        self.gyro_sensitivity  = gyro_sensitivity
        self.filter_alpha   = filter_alpha
        self.gz_lpf_alpha   = gz_lpf_alpha
        self._calib_samples = calib_samples

        # ── 角度状态 ──
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0

        # ── 校准状态 ──
        self.gyro_offset_x = 0.0
        self.gyro_offset_y = 0.0
        self.gyro_offset_z = 0.0
        self.ref_roll  = 0.0
        self.ref_pitch = 0.0
        self.ref_yaw   = 0.0
        self._calibrated_gyro = False

        # ── 滤波器状态 ──
        self.gz_filtered = 0.0
        self._wz_dps = 0.0
        self._wx_dps = 0.0
        self._wy_dps = 0.0
        self._last_time = 0

        # ── 硬件初始化 ──
        time.sleep_ms(100)  # 上电稳定延迟

        if imu_instance is not None:
            self._imu = imu_instance
        else:
            # 与现有 imu_motion.py 一致：尽量用空构造让 AUTO 自动检测
            if imu_type is not None:
                self._imu = IMU660RX(capture_div=capture_div, imu_type=imu_type)
            else:
                self._imu = IMU660RX()

        # 获取 IMU 数据缓冲区（绑定到 imu.get()，之后不再重复调用）
        self._imu_data = self._imu.get()

        # ── Ticker ──
        self._pit = None
        self._running = False

        # ── 自动校准 ──
        if calibrate_on_init:
            self.calibrate_gyro_zero(calib_samples)


    # ═══════════════════════════════════════════════════════════
    #  校准
    # ═══════════════════════════════════════════════════════════

    def calibrate_gyro_zero(self, samples: int = None):
        """
        陀螺仪零偏校准（需保持静止）
        采样 samples 次原始陀螺仪数据，取均值作为零偏。

        返回: (offset_x, offset_y, offset_z)
        """
        if samples is None:
            samples = self._calib_samples

        # 停止 ticker（如果正在运行，imu.read() 需要直接控制 SPI）
        was_running = self._running
        if was_running:
            self.stop()

        print("校准陀螺仪零偏 ({} 次采样)...".format(samples), end='')

        gx_sum = 0.0
        gy_sum = 0.0
        gz_sum = 0.0

        for i in range(samples):
            d = self._imu.read()
            gx_sum += d[3]
            gy_sum += d[4]
            gz_sum += d[5]
            time.sleep_ms(CALIB_DELAY_MS)
            if (i + 1) % 100 == 0:
                print('.', end='')

        self.gyro_offset_x = gx_sum / samples
        self.gyro_offset_y = gy_sum / samples
        self.gyro_offset_z = gz_sum / samples
        self._calibrated_gyro = True

        print(" 完成")
        print("陀螺仪零偏: gx={:.2f}  gy={:.2f}  gz={:.2f}".format(
            self.gyro_offset_x, self.gyro_offset_y, self.gyro_offset_z))

        # 恢复 ticker（如果之前运行中）
        if was_running:
            self.start()

        return (self.gyro_offset_x, self.gyro_offset_y, self.gyro_offset_z)


    def set_zero_reference(self):
        """
        将当前姿态设为参考零点
        之后的 get_angles() 返回相对于当前姿态的角度变化量。
        适用于 "车头朝向校准" 或 "启动姿态归零" 场景。
        """
        self.ref_roll  = self.roll
        self.ref_pitch = self.pitch
        self.ref_yaw   = self.yaw
        print("归零参考已设置: roll={:.1f} pitch={:.1f} yaw={:.1f}".format(
            self.ref_roll, self.ref_pitch, self.ref_yaw))


    def clear_zero_reference(self):
        """移除归零参考，输出原始角度"""
        self.ref_roll  = 0.0
        self.ref_pitch = 0.0
        self.ref_yaw   = 0.0
        print("归零参考已清除")


    def set_home_yaw(self, yaw_deg: float = 0.0):
        """
        设置偏航角的"前方向"
        调用后 get_angles() 返回的 yaw 以 yaw_deg 为参考方向
        """
        self.ref_yaw = self.yaw - yaw_deg


    def is_calibrated(self) -> bool:
        """陀螺仪零偏校准是否已完成"""
        return self._calibrated_gyro


    # ═══════════════════════════════════════════════════════════
    #  实时角度读取
    # ═══════════════════════════════════════════════════════════

    def update(self, data=None):
        """
        单步角度更新
        读取 IMU 数据 → 互补滤波 → 更新 roll/pitch/yaw

        参数 data: 可选的 [ax,ay,az,gx,gy,gz]（若提供则跳过硬件读取）
        """
        if data is None:
            data = self._imu_data  # ticker 自动更新的缓冲区

        ax, ay, az = data[0], data[1], data[2]
        gx, gy, gz = data[3], data[4], data[5]

        self._update_angle(ax, ay, az, gx, gy, gz)


    def _update_angle(self, ax, ay, az, gx, gy, gz):
        """
        互补滤波更新 roll/pitch/yaw
        算法与 imu_motion.py:update_angle() 完全一致
        """
        now = time.ticks_ms()
        if self._last_time == 0:
            self._last_time = now
            self.gz_filtered = gz  # 首次初始化
            return

        dt = (now - self._last_time) * 0.001
        self._last_time = now

        # 防止 dt 异常（如 ticker 暂停后恢复）
        if dt > 0.5:
            dt = 0.05

        # ── 加速度计 → 重力方向角度 ──
        ax_g = ax / self.accel_sensitivity
        ay_g = ay / self.accel_sensitivity
        az_g = az / self.accel_sensitivity

        pitch_accel = math.atan2(-ax_g, math.sqrt(ay_g**2 + az_g**2)) * 180.0 / math.pi
        roll_accel  = math.atan2( ay_g, az_g) * 180.0 / math.pi

        # ── 陀螺仪 → 角速度 (deg/s) ──
        gx_dps = (gx - self.gyro_offset_x) / self.gyro_sensitivity
        gy_dps = (gy - self.gyro_offset_y) / self.gyro_sensitivity

        # Z 轴低通滤波（抑制电机噪声）
        self.gz_filtered = (
            self.gz_lpf_alpha * self.gz_filtered +
            (1.0 - self.gz_lpf_alpha) * gz
        )
        gz_dps = (self.gz_filtered - self.gyro_offset_z) / self.gyro_sensitivity

        # 存储角速度供外部读取
        self._wz_dps = gz_dps
        self._wx_dps = gx_dps
        self._wy_dps = gy_dps

        # ── 互补滤波 ──
        alpha = self.filter_alpha
        self.roll  = alpha * (self.roll  + gx_dps * dt) + (1.0 - alpha) * roll_accel
        self.pitch = alpha * (self.pitch + gy_dps * dt) + (1.0 - alpha) * pitch_accel

        # ── 偏航角（纯陀螺仪积分）──
        self.yaw += gz_dps * dt
        self.yaw = _wrap_angle(self.yaw, YAW_WRAP_MIN, YAW_WRAP_MAX)


    def get_angles(self):
        """
        返回校准后的欧拉角
        返回值: (roll, pitch, yaw) 单位: 度，范围: roll∈[-180,180), pitch∈[-90,90], yaw∈[-180,180)
        """
        r = self.roll  - self.ref_roll
        p = self.pitch - self.ref_pitch
        y = _wrap_angle(self.yaw - self.ref_yaw, YAW_WRAP_MIN, YAW_WRAP_MAX)
        return (r, p, y)


    def get_angular_velocity(self):
        """
        返回三轴角速度
        返回值: (wx, wy, wz) 单位: deg/s
        """
        return (self._wx_dps, self._wy_dps, self._wz_dps)


    def get_raw(self):
        """
        返回原始传感器数据
        返回值: [ax, ay, az, gx, gy, gz]（ADC 原始值）
        """
        return self._imu_data[:6]


    def get_safe(self):
        """
        安全读取 IMU 数据
        封装 imu.read() 的 try-except，返回 None 表示读取失败。

        注意：imu.read() 内部做 SPI 事务。若 SPI 被电机噪声破坏，
              smartcar 库驱动在 __disable_irq() 后陷入轮询死等，
              此 try-except 无法捕获。此时由 machine.WDT() 硬件看门狗恢复。
        """
        try:
            d = self._imu.read()
            if d is None or len(d) < 6:
                return None
            for v in d[:6]:
                if v is None:
                    return None
            return d
        except Exception:
            return None


    def get_buf_safe(self):
        """
        安全读取 IMU 缓冲区数据（ticker 自动采集模式）
        封装 imu.get() 的 try-except，返回 None 表示读取失败。

        注意：imu.get() 在 ticker 停止时可能回退到 SPI 直读，
              同样可能触发 __disable_irq() 后挂死。
        """
        try:
            d = self._imu.get()
            if d is None or len(d) < 6:
                return None
            for v in d[:6]:
                if v is None:
                    return None
            return d
        except Exception:
            return None


    # ═══════════════════════════════════════════════════════════
    #  Ticker 控制
    # ═══════════════════════════════════════════════════════════

    def start(self, period_ms: int = None):
        """
        启动 PIT ticker 自动采集

        参数 period_ms: 采集周期 (ms)，默认使用构造函数中的值
        """
        if period_ms is None:
            period_ms = self._period_ms

        if self._running:
            return

        self._pit = _ticker_cls(self._pit_channel)
        self._pit.capture_list(self._imu)
        self._pit.start(period_ms)
        self._running = True


    def stop(self):
        """停止 PIT ticker 自动采集"""
        if not self._running:
            return
        try:
            self._pit.stop()
        except Exception:
            pass
        self._pit = None
        self._running = False


    def is_running(self) -> bool:
        """Ticker 是否正在运行"""
        return self._running


    # ═══════════════════════════════════════════════════════════
    #  配置
    # ═══════════════════════════════════════════════════════════

    def set_filter_alpha(self, alpha: float):
        """设置互补滤波器 α（0~1，越大越信任陀螺仪，越小平滑但延迟大）"""
        self.filter_alpha = max(0.0, min(alpha, 1.0))


    def set_gz_lpf_alpha(self, alpha: float):
        """设置 Z 轴陀螺仪低通滤波器 α（0~1，越小滤波越强）"""
        self.gz_lpf_alpha = max(0.0, min(alpha, 1.0))


    def set_gyro_sensitivity(self, sensitivity: float):
        """设置陀螺仪灵敏度 LSB/(deg/s)"""
        self.gyro_sensitivity = sensitivity


    def set_accel_sensitivity(self, sensitivity: float):
        """设置加速度计灵敏度 LSB/g"""
        self.accel_sensitivity = sensitivity


    def info(self):
        """打印 IMU 状态信息"""
        print("=" * 50)
        print("IMU660RX 状态")
        print("  陀螺仪零偏校准: {}".format("完成" if self._calibrated_gyro else "未校准"))
        print("  陀螺仪零偏: gx={:.2f} gy={:.2f} gz={:.2f}".format(
            self.gyro_offset_x, self.gyro_offset_y, self.gyro_offset_z))
        print("  归零参考: roll={:.1f} pitch={:.1f} yaw={:.1f}".format(
            self.ref_roll, self.ref_pitch, self.ref_yaw))
        print("  互补滤波 α={:.2f}, gz_lpf α={:.2f}".format(
            self.filter_alpha, self.gz_lpf_alpha))
        print("  灵敏度: accel={:.0f} LSB/g, gyro={:.1f} LSB/(deg/s)".format(
            self.accel_sensitivity, self.gyro_sensitivity))
        print("  Ticker: PIT{} {} ({}ms)".format(
            self._pit_channel, "运行中" if self._running else "已停止", self._period_ms))
        r, p, y = self.get_angles()
        wz = self._wz_dps
        print("  角度: roll={:>6.1f}° pitch={:>6.1f}° yaw={:>6.1f}°".format(r, p, y))
        print("  角速度: wz={:>6.1f} deg/s".format(wz))
        print("=" * 50)


