"""
IMU_hold.py — IMU 偏航保持模块

【层】控制层
【职责】
  HeadingHold : 偏航保持 P 控制器（依赖 io/imu.py 的 IMU 类）
  滤波/校准/ticker → 由 io/imu.py 的 IMU 类统一管理，本模块不重复实现
【依赖】io/imu.py、pid.py
【使用】
  hold = HeadingHold(imu)
  wz, yaw, err = hold.compute(dt)
"""
import gc, time, math
from imu import IMU
from pid import PID

# ── 偏航保持参数（对齐 led 项目）──
HOLD_KP       = 0.01	 # P 增益 — 2°误差时 wz≈0.002（
HOLD_KI       = 0.0      # I 增益 — 偏航保持无需积分
HOLD_KD       = 0.001    # D 增益 — 阻尼抑制过冲震荡
HOLD_DEADBAND = 3.0      # 死区(°) — 误差≤3°不反应
HOLD_WZ_MAX   = 0.45     # wz 输出限幅
HOLD_I_LIMIT  = 0.40     # 积分限幅（KI=0 时无效，保留兼容）


class HeadingHold:
    """IMU 偏航保持 P 控制器。

    两种使用模式:
      1. 外部喂 yaw（main.py）:  HeadingHold() + hold.update(yaw, dt, deadband)
      2. 内部自读 IMU（新）:      HeadingHold(imu) + hold.compute(dt)
    """

    def __init__(self, imu: IMU = None,
                 target_yaw_deg: float = 0.0,
                 kp: float = HOLD_KP, ki: float = HOLD_KI, kd: float = HOLD_KD,
                 deadband: float = HOLD_DEADBAND, wz_max: float = HOLD_WZ_MAX,
                 integral_limit: float = HOLD_I_LIMIT):
        self._imu = imu
        self._target = target_yaw_deg
        self._deadband = deadband
        self._pid = PID(kp=kp, ki=ki, kd=kd,
                        integral_limit=integral_limit,
                        output_limit=wz_max)
        self._yaw = 0.0  # 外部喂入的 yaw 缓存

    @property
    def target(self) -> float:
        return self._target

    def set_target(self, deg: float):
        self._target = deg
        self._pid.reset()

    def update(self, yaw: float, dt: float, deadband: float = None):
        """外部喂入 yaw，计算航向修正 wz（main.py 兼容接口）。
        返回 (wz, yaw, yaw_err)。"""
        db = deadband if deadband is not None else self._deadband
        raw_err = yaw - self._target
        yaw_err = ((raw_err + 180) % 360) - 180

        if abs(yaw_err) <= db:
            return 0.0, yaw, yaw_err

        wz = self._pid.compute(0.0, yaw_err, dt)
        return wz, yaw, yaw_err

    def compute(self, dt: float):
        """内部自读 IMU → 算误差 → PID 输出 wz。返回 (wz, yaw, yaw_err)。"""
        if self._imu is None:
            raise RuntimeError("HeadingHold: imu 未传入，请使用 update() 外部喂 yaw")
        self._imu.update()
        _, _, yaw = self._imu.get_angles()

        raw_err = yaw - self._target
        yaw_err = ((raw_err + 180) % 360) - 180

        if abs(yaw_err) <= self._deadband:
            return 0.0, yaw, yaw_err

        wz = self._pid.compute(0.0, yaw_err, dt)
        return wz, yaw, yaw_err

    def reset(self):
        self._pid.reset()
