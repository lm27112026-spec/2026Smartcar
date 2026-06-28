"""
rotate.py — 旋转控制模块（IMU 闭环，无像素依赖）
【层】控制层
【职责】
  Rotator     : 主动旋转机动（转到指定角度、绕轴转圈）
  HeadingHold : 偏航保持（跟随过程中锁定车身朝向）

── API ──
  class Rotator(ctrl):
    __init__            → 绑定 CascadeController + 自初始化 IMU
    zero()              → 当前朝向归零
    rotate_to_abs(deg)  → 转到绝对角度，返回 (yaw, ok)
    rotate_by(delta)    → 相对旋转，返回 (yaw, ok)
    stop()              → 停止 IMU ticker

  class HeadingHold(imu, target_yaw_deg):
    compute(dt)         → (wz, yaw, yaw_err)  每帧调用
    set_target(deg)     → 修改目标偏航角
    reset()             → 清零 PID 状态
"""
import time, math
from imu import IMU
from pid import PID

# ============================================================
#  旋转参数 
# ============================================================

# ── P 控制器 ──
ROT_KP       = 0.015    # P 增益：wz = KP × 角度误差(°)

# ── 转速限制 ──
ROT_WZ_MAX   = 0.20     # 最大旋转速度（归一化值，0~1）
ROT_WZ_MIN   = 0.03     # 最小旋转速度（0.01→0.03，末期不拖沓）

# ── 到位判定 ──
ROT_DEADBAND = 4.0      # 死区(°)：跳过末端 2~4° 低速漂移段，提早停

# ── 超时保护 ──
ROT_TIMEOUT  = 8000     # 超时(ms)：超时后强制停止

# ── IMU 设置 ──
IMU_CALIB_SAMPLES = 300 # 陀螺仪零偏校准采样数
IMU_PERIOD_MS     = 10  # ticker 采集周期(ms)

# ── 绕轴半径 ──
# 旋转时机器人横向平移量，直接控制绕圈大小
# vy += radius * wz （radius 在接近目标时自动缩小）
PIVOT_RADIUS       = 2.8   # 最大绕轴半径，越大绕圈越大，0=自转

PIVOT_TAPER_ANGLE  = 30.0  # 角度误差小于此值时开始削减横向 (°)
PIVOT_TAPER_MIN    = 3.0   # 角度误差小于此值时横向归零 (°)

# ============================================================
#  偏航保持 
# ============================================================

HOLD_KP       = 0.15     # P 增益（与 main.py 调优值一致）
HOLD_KI       = 0.15     # I 增益（dt归一化：0.15×0.02=0.003/次，等价 main.py 的 HEADING_KI=0.003）
HOLD_KD       = 0.0      # D 增益（实战中不需要）
HOLD_DEADBAND = 2.0      # 死区(°): 误差≤2°不反应
HOLD_WZ_MAX   = 0.30     # wz 输出限幅（归一化值）
HOLD_I_LIMIT  = 0.40     # 积分限幅（PID 内部原始值）；I 输出 = HOLD_KI × 限幅 = 0.15×0.4 = 0.06，与旧版 HEADING_I_LIMIT 等效


class Rotator:
    """IMU 闭环绕轴旋转控制器（使用外部共享 IMU，不独占）。

    用法:
        rot = Rotator(imu, ctrl, switch_check=None)
        final_yaw, ok = rot.rotate_to_abs(90)  # 阻塞转到 90°
        final_yaw, ok = rot.rotate_by(45)      # 相对旋转 45°
    """

    def __init__(self, imu: IMU, ctrl,
                 switch_check=None):
        """
        imu          : 外部共享 IMU 实例（已 start，由调用方管理生命周期）
        ctrl         : CascadeController 实例
        switch_check : 可选的拨码检查回调，返回 True=中止。
                       用于在旋转过程中检查开关状态。
        """
        self._imu = imu
        self._ctrl = ctrl
        self._switch_check = switch_check

    # ── 角度归零 ──────────────────────────────────────────

    def zero(self):
        """将当前朝向设为 0°"""
        self._imu.set_zero_reference()
        print("  [Rotator] 朝向归零")

    # ── 旋转到绝对角度 ────────────────────────────────────

    def rotate_to_abs(self, target_deg):
        """
        阻塞旋转到绝对角度（相对归零朝向）。
        target_deg: 目标角度 (°)，正=逆时针，范围 [-180, 180]

        返回: (final_yaw, ok) —— ok=False 表示超时或开关中止
        """
        direction = "↺" if target_deg >= 0 else "↻"
        print("\n  [Rotator] 目标: {:+.0f}° {}".format(target_deg, direction))
        print("  {:>5s} | {:>7s} | {:>5s} | {:>5s} | {:>4s}".format(
            "t(s)", "yaw", "err", "wz", "rad"))
        print("  " + "-" * 38)

        t0 = time.ticks_ms()
        lap = t0
        aborted = False

        while time.ticks_diff(time.ticks_ms(), t0) < ROT_TIMEOUT:
            # ── 拨码开关中止 ──
            if self._switch_check is not None and self._switch_check():
                print("  [Rotator] 开关中止")
                aborted = True
                break

            self._imu.update()
            _, _, yaw = self._imu.get_angles()

            err = target_deg - yaw
            if abs(err) < ROT_DEADBAND:
                break

            # P 控制 → 限幅
            wz = ROT_KP * err
            if abs(wz) > ROT_WZ_MAX:
                wz = ROT_WZ_MAX if wz > 0 else -ROT_WZ_MAX
            elif abs(wz) < ROT_WZ_MIN and abs(wz) > 0:
                wz = ROT_WZ_MIN if wz > 0 else -ROT_WZ_MIN

            # 绕轴半径渐缩
            err_abs = abs(err)
            if err_abs < PIVOT_TAPER_MIN:
                radius = 0.0
            elif err_abs < PIVOT_TAPER_ANGLE:
                ratio = (err_abs - PIVOT_TAPER_MIN) / (PIVOT_TAPER_ANGLE - PIVOT_TAPER_MIN)
                radius = PIVOT_RADIUS * ratio
            else:
                radius = PIVOT_RADIUS

            self._ctrl.step(0.0, 100.0, 0.0, False, wz=wz, vy_extra=radius * wz)

            now = time.ticks_ms()
            if time.ticks_diff(now, lap) >= 200:
                lap = now
                print("  {:4.1f} | {:7.2f} | {:5.2f} | {:5.3f} r={:.1f}".format(
                    time.ticks_diff(now, t0) * 0.001, yaw, err, wz, radius))

            time.sleep_ms(10)

        # ── 结束：只停电机，不毁 PID ──
        from motor import stop_all
        stop_all()

        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        _, _, yaw = self._imu.get_angles()

        ok = not aborted and elapsed < ROT_TIMEOUT
        status = "✓" if ok else ("⚠中止" if aborted else "⚠超时")
        print("  {:>5s} → yaw={:+.1f}° ({:.1f}s)".format(status, yaw, elapsed * 0.001))
        return yaw, ok

    # ── 旋转相对角度 ──────────────────────────────────────

    def rotate_by(self, delta_deg):
        """
        旋转相对角度（从当前位置转 delta_deg 度）
        delta_deg: 旋转角度 (°)，正=逆时针
        """
        _, _, current = self._imu.get_angles()
        target = current + delta_deg
        # 折叠到 [-180, 180)
        target = ((target + 180) % 360) - 180
        return self.rotate_to_abs(target)

    # ── 停止 ──────────────────────────────────────────────

    def stop(self):
        """停止旋转（仅停电机，不管理 IMU 生命周期）"""
        from motor import stop_all
        stop_all()


# ═══════════════════════════════════════════════════════════════
#  HeadingHold — IMU 偏航保持（跟随过程中锁定车身朝向）
# ═══════════════════════════════════════════════════════════════

class HeadingHold:
    """IMU 偏航保持 PI 控制器。

    与 Rotator 不同：HeadingHold 不做主动旋转机动，
    而是在跟随过程中持续修正车身朝向，使其锁定在目标角度。

    用法（无 IMU，外部喂 yaw — main.py 场景）:
        hold = HeadingHold()
        hold.set_target(_yaw())          # 锁定当前朝向
        while True:
            _read_imu_update_yaw()
            wz, yaw, yaw_err = hold.update(_yaw(), dt=0.02)
            ctrl.step(ex, dist, roll, tracking, wz=wz)

    用法（有 IMU，内部自动读取）:
        hold = HeadingHold(imu, target_yaw_deg=90.0)
        while True:
            wz, yaw, yaw_err = hold.compute(dt=0.02)
            ctrl.step(ex, dist, roll, tracking, wz=wz)
    """

    def __init__(self, imu: IMU = None,
                 target_yaw_deg: float = 0.0,
                 kp: float = HOLD_KP, ki: float = HOLD_KI, kd: float = HOLD_KD,
                 deadband: float = HOLD_DEADBAND, wz_max: float = HOLD_WZ_MAX,
                 integral_limit: float = HOLD_I_LIMIT):
        """
        imu            : 已初始化的 IMU 实例（可选，None=由调用方通过 update() 喂 yaw）
        target_yaw_deg : 目标偏航角 (°)，相对 IMU 归零参考
        kp/ki/kd       : PID 增益（ki 为 dt 归一化值，非原始值）
        deadband       : 死区 (°)，误差 ≤ deadband 时清零积分、输出 0
        wz_max         : wz 输出限幅（归一化值）
        integral_limit : 积分限幅（归一化值）
        """
        self._imu = imu
        self._target = target_yaw_deg
        self._deadband = deadband
        self._pid = PID(kp=kp, ki=ki, kd=kd,
                        integral_limit=integral_limit,
                        output_limit=wz_max)

    # ── 属性 ──────────────────────────────────────────────

    @property
    def target(self) -> float:
        """当前目标偏航角 (°)"""
        return self._target

    # ── 公开方法 ──────────────────────────────────────────

    def set_target(self, deg: float):
        """修改目标偏航角 (°)，同时重置 PID 积分避免旧误差累积"""
        self._target = deg
        self._pid.reset()

    def set_deadband(self, deg: float):
        """修改死区阈值 (°)"""
        self._deadband = deg

    def update(self, yaw: float, dt: float, deadband: float = None):
        """
        使用外部提供的 yaw 角度计算 wz 修正（无需 IMU 实例）。
        适用于调用方自己管理 IMU 读取管线的场景。

        参数:
            yaw      : 当前偏航角 (°)
            dt       : 控制周期 (s)，用于 PID 积分/微分计算
            deadband : 可选死区 (°)，None 则使用实例默认值

        返回:
            (wz, yaw, yaw_err)
              wz      : 旋转指令（归一化值），死区内为 0
              yaw     : 当前偏航角 (°)，原样返回
              yaw_err : 折叠后的误差 (°)，范围 [-180, 180)
        """
        db = deadband if deadband is not None else self._deadband

        raw_err = yaw - self._target
        yaw_err = ((raw_err + 180) % 360) - 180   # 折叠到 [-180, 180)

        if abs(yaw_err) <= db:
            self._pid.reset()   # 死区内清零积分，防止下次扰动过冲（main.py 实践）
            return 0.0, yaw, yaw_err

        wz = self._pid.compute(0.0, yaw_err, dt)
        return wz, yaw, yaw_err

    def compute(self, dt: float):
        """
        每帧调用：读取 IMU 角度 → 计算误差 → PID 输出 wz。
        需要构造函数中提供 imu 实例。

        参数:
          dt: 控制周期 (s)

        返回:
          (wz, yaw, yaw_err)  — 同 update()
        """
        if self._imu is None:
            raise RuntimeError(
                "HeadingHold.compute() 需要 IMU 实例；"
                "若无 IMU，请改用 update(yaw, dt)"
            )
        self._imu.update()
        _, _, yaw = self._imu.get_angles()
        return self.update(yaw, dt)

    def reset(self):
        """清零 PID 积分和微分状态"""
        self._pid.reset()


