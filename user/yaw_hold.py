"""
yaw_hold.py - IMU 闭环航向保持控制器

【算法】
  1. 从 imu.get_angles() 获取当前 yaw
  2. 误差折叠: ((raw_err + 180) % 360) - 180  →  限制在 [-180, 180)
  3. 死区判定: |error| < deadband → 清零 PID 积分
  4. PID 计算: target=0, measurement=yaw_error → wz 输出
  5. WZ 限幅: wz ∈ [-wz_max, wz_max]

【使用】
    from yaw_hold import YawHolder
    from imu import IMU

    imu = IMU()
    imu.start()
    imu.set_zero_reference()

    holder = YawHolder(imu, kp=0.01, ki=0.001)
    holder.set_target(90)  # 目标航向 90°

    while True:
        imu.update()
        wz = holder.compute(dt=0.01)
        omni_drive_closed_loop(0, 0, wz, speeds, dt)
"""

from pid import PID


# ═══════════════════════════════════════════════════════════════
#  默认参数
# ═══════════════════════════════════════════════════════════════

DEF_P  = 0.01               # 比例增益
DEF_I  = 0.001              # 积分增益
DEF_D  = 0.0                # 微分增益 (通常不用)
DEF_DZ = 2.0                # 死区 (°)  误差<死区→清零积分+输出0
DEF_WZ = 0.20               # WZ 限幅 (归一化速度)
I_LIM  = 50                 # PID内部积分限幅


# ═══════════════════════════════════════════════════════════════
#  角度工具
# ═══════════════════════════════════════════════════════════════

def _fold_angle(error):
    """
    将角度误差折叠到 [-180, 180) 区间
    
    借鉴 led: ((raw_err + 180) % 360) - 180
    """
    return ((error + 180.0) % 360.0) - 180.0


# ═══════════════════════════════════════════════════════════════
#  YawHolder 类
# ═══════════════════════════════════════════════════════════════

class YawHolder:
    """
    IMU 闭环航向保持控制器

    借鉴 led 的双模式设计:
      - yaw PID 模式: 持续修正航向偏差
      - 死区模式: 误差小于阈值时清零积分、输出 0

    参数:
        imu:        IMU 实例（需已调用 start() + set_zero_reference()）
        kp:         比例增益 (默认 0.01)
        ki:         积分增益 (默认 0.001)
        kd:         微分增益 (默认 0.0)
        deadband:   死区角度 (°)，误差小于此值时输出 0 (默认 2.0)
        wz_max:     WZ 输出限幅 (归一化速度，默认 0.20)
    """

    def __init__(self, imu,
                 kp=DEF_P, ki=DEF_I, kd=DEF_D,
                 deadband=DEF_DZ, wz_max=DEF_WZ):
        self._imu = imu
        self._deadband = deadband
        self._wz_max = wz_max

        self._pid = PID(
            kp=kp, ki=ki, kd=kd,
            integral_limit=I_LIM,
            output_limit=wz_max
        )

        self._target_yaw = 0.0    # 目标航向 (°)
        self._current_wz = 0.0    # 当前 WZ 输出
        self._is_holding = False  # 是否正在保持航向

    # ── 属性 ─────────────────────────────────────────────────

    @property
    def target_yaw(self):
        """目标航向 (°)"""
        return self._target_yaw

    @property
    def current_wz(self):
        """当前 WZ 输出"""
        return self._current_wz

    @property
    def is_aligned(self):
        """是否已对准目标航向（在死区范围内）"""
        return self._is_holding

    # ── 目标设置 ─────────────────────────────────────────────

    def set_target(self, yaw_deg):
        """
        设置目标航向
        
        参数:
            yaw_deg: 目标角度 (°)，范围任意（内部自动折叠到 [-180, 180)）
        
        说明:
            调用后重置 PID 积分，避免旧误差累积
        """
        self._target_yaw = _fold_angle(yaw_deg)  # 折叠到 [-180,180) 与 get_angles() 范围一致
        self._pid.reset()
        self._current_wz = 0.0
        self._is_holding = False

    def set_target_relative(self, delta_deg):
        """
        相对旋转（在当前位置基础上增加 delta_deg 度）
        
        参数:
            delta_deg: 相对角度 (°)，正=顺时针，负=逆时针
        """
        _, _, yaw = self._imu.get_angles()
        self.set_target(yaw + delta_deg)

    def set_target_current(self):
        """将当前朝向设为目标（即锁定当前朝向）"""
        _, _, yaw = self._imu.get_angles()
        self.set_target(yaw)

    # ── 控制计算 ─────────────────────────────────────────────

    def compute(self, dt=0.01):
        """
        计算 WZ 旋转速度指令
        
        参数:
            dt: 控制周期 (s)，默认 0.01 (100Hz)
        
        返回:
            float: WZ 旋转速度指令（归一化速度）
                   0 表示已对准或无需修正
        """
        _, _, yaw = self._imu.get_angles()

        # 1. 误差计算 + 折叠到 [-180, 180)
        raw_err = yaw - self._target_yaw
        yaw_err = _fold_angle(raw_err)

        # 2. 死区判定（借鉴 led: 死区内清零积分）
        if abs(yaw_err) < self._deadband:
            self._pid.reset()
            self._current_wz = 0.0
            self._is_holding = True
            return 0.0

        self._is_holding = False

        # 3. PID 控制: setpoint=0, measurement=yaw_err
        #    → PID 输出 = Kp*(0 - yaw_err) + Ki*∫(0-yaw_err) + Kd*d(0-yaw_err)/dt
        #    → 等价于 Kp*(-error) + ...  → 负反馈消除误差
        wz = self._pid.compute(0.0, yaw_err, dt)

        # 4. 限幅（PID 内部已做 output_limit，此处做二次保险）
        if wz > self._wz_max:
            wz = self._wz_max
        elif wz < -self._wz_max:
            wz = -self._wz_max

        self._current_wz = wz
        return wz

    # ── 状态查询 ─────────────────────────────────────────────

    def get_yaw_error(self):
        """
        返回当前航向误差 (°)
        
        返回:
            float: 误差角度，范围 [-180, 180)
                  >0: 需要顺时针旋转（正 WZ）
                  <0: 需要逆时针旋转（负 WZ）
        """
        _, _, yaw = self._imu.get_angles()
        return _fold_angle(yaw - self._target_yaw)

    def reset(self):
        """重置控制器状态（PID 积分清零）"""
        self._pid.reset()
        self._current_wz = 0.0
        self._is_holding = False

    def info(self):
        """打印当前状态"""
        _, _, yaw = self._imu.get_angles()
        err = self.get_yaw_error()
        print("YawHolder: target={:.1f}°  current={:.1f}°  err={:.1f}°  wz={:.3f}  aligned={}".format(
            self._target_yaw, yaw, err, self._current_wz, self._is_holding))


