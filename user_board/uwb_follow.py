"""
uwb_follow.py — UWB 跟随控制与导航参数（统一管理）

【层】控制层
【职责】
  UWBFollowController: 从 goto_location() 提取的纯跟随控制逻辑
    - 机体坐标系变换 (全局误差 → 前后/横向速度)
    - 减速曲线 (接近目标线性衰减)
    - 速度限幅 + 最小速度保持 (克服静摩擦)
    - 控制输出死区 (极小指令置零，防电机抖动)
    - 速度低通滤波 (平滑指令，减少突变冲击)

  GOTO_* 参数: 所有导航行为参数集中于此。
    调参只改本文件，uwb_control.goto_location() 从本文件导入。
"""
import math

# ═══════════════════════════════════════════════════════════════
#  导航控制参数
# ═══════════════════════════════════════════════════════════════
# ── P 控制 ──
GOTO_KP              = 0.02    # 位置误差 → 速度 P 增益

# ── 到达判定 (全局坐标单边区间) ──
GOTO_ZONE_DX          = 5.0    # X 单边容差 (cm) — target ≤ x ≤ target+DX
GOTO_ZONE_DY          = 5.0    # Y 单边容差 (cm) — target ≤ y ≤ target+DY
GOTO_SLOW_DIST       = 50.0    # 减速起始距离 (cm) — 提前减速给刹车留余量，防过冲

# ── 物资区坐标 ──
GOTO_SUPPLIES_X       = 50.0    # 物资区 X 坐标 (cm)
GOTO_SUPPLIES_Y       = 210.0   # 物资区 Y 坐标 (cm)

# ── 减速曲线参数 ──
GOTO_BRAKE_POWER = 2.0     # 减速曲线幂次 — 编码器前馈已提供精准位置，温和减速即可

# ── 速度限幅 ──
GOTO_MAX_SPEED       = 0.50    # 最大速度 (m/s)
GOTO_MIN_SPEED       = 0.06    # 最小速度 (m/s)，克服静摩擦

# ── 输出死区 + 低通滤波 ──
GOTO_OUTPUT_DEADZONE = 0.02   # 输出死区 (m/s) — 低于此值的速度指令置零，防电机抖动
GOTO_LPF_ALPHA       = 0.70   # 低通滤波系数 (0~1, 越小越平滑但延迟越大) — 提速减少减速拖尾

# ── 导航流程参数 ──
GOTO_TIMEOUT_S       = 20.0    # 总导航超时 (s)
GOTO_CTRL_DT         = 0.01    # 控制周期 (s)
GOTO_ARRIVAL_FRAMES  = 2       # 连续 N 帧在 zone 内判定到达（防噪声误判）
GOTO_UWB_STEP_MS     = 30      # uwb.step() 调用间隔 (ms) — 提速获取更新鲜的 UWB 帧
GOTO_UWB_DEAD_TIMEOUT_S = 5.0  # UWB 离线超时 (s)，超时后尝试重连
GOTO_UWB_MAX_RECONNECT  = 3     # UWB 最大重连次数
GOTO_UWB_RECONNECT_WAIT_MS = 500  # 重连后等待数据恢复 (ms)
GOTO_PRINT_INTERVAL_MS = 100   # 状态打印间隔 (ms) — 0=每帧打印(洪水), 100=10Hz 调试

class UWBFollowController:
    """UWB 跟随控制器 — 仅计算速度，不到达判定（由 goto_location 统一 zone 检查）。

    用法:
        ctrl = UWBFollowController()
        vx, vy = ctrl.compute(error_x, error_y, yaw_deg, dist)
    """

    def __init__(self, kp=None, slow_dist=None,
                 max_speed=None, min_speed=None,
                 output_deadzone=None, lpf_alpha=None,
                 brake_power=None):
        self.kp = GOTO_KP if kp is None else kp
        self.slow_dist = GOTO_SLOW_DIST if slow_dist is None else slow_dist
        self.max_speed = GOTO_MAX_SPEED if max_speed is None else max_speed
        self.min_speed = GOTO_MIN_SPEED if min_speed is None else min_speed
        self.output_deadzone = GOTO_OUTPUT_DEADZONE if output_deadzone is None else output_deadzone
        self.lpf_alpha = GOTO_LPF_ALPHA if lpf_alpha is None else lpf_alpha
        self.brake_power = GOTO_BRAKE_POWER if brake_power is None else brake_power

        self._vx_filt = 0.0
        self._vy_filt = 0.0

    def compute(self, error_x, error_y, yaw_deg, dist):
        """计算速度指令 — 不到达判定。

        返回: (vx, vy)
        """
        yaw_rad = math.radians(yaw_deg)
        body_fwd   = -math.cos(yaw_rad) * error_x - math.sin(yaw_rad) * error_y
        body_right  =  math.sin(yaw_rad) * error_x - math.cos(yaw_rad) * error_y

        vx_cmd = body_fwd * self.kp
        vy_cmd = body_right * self.kp

        # 非线性减速曲线: (dist/slow_dist)^power — 远距保速, 近距急刹
        if 0 < dist < self.slow_dist:
            ratio = dist / self.slow_dist
            if ratio < 0.0:   ratio = 0.0
            elif ratio > 1.0: ratio = 1.0
            decay = ratio ** self.brake_power
            vx_cmd *= decay
            vy_cmd *= decay

        # 速度限幅
        speed = math.sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd)
        if speed > self.max_speed:
            vx_cmd = vx_cmd / speed * self.max_speed
            vy_cmd = vy_cmd / speed * self.max_speed

        # 最小速度保持
        if 0 < speed < self.min_speed and dist > GOTO_ZONE_DX:
            vx_cmd = vx_cmd / speed * self.min_speed
            vy_cmd = vy_cmd / speed * self.min_speed

        # 输出死区
        if abs(vx_cmd) < self.output_deadzone: vx_cmd = 0.0
        if abs(vy_cmd) < self.output_deadzone: vy_cmd = 0.0

        # LPF 平滑
        self._vx_filt = self.lpf_alpha * vx_cmd + (1.0 - self.lpf_alpha) * self._vx_filt
        self._vy_filt = self.lpf_alpha * vy_cmd + (1.0 - self.lpf_alpha) * self._vy_filt

        return self._vx_filt, self._vy_filt

    def reset(self):
        self._vx_filt = 0.0
        self._vy_filt = 0.0

    def reset_lpf(self):
        """仅重置低通滤波器状态 — 用于进入 zone 时砍掉速度惯性拖尾"""
        self._vx_filt = 0.0
        self._vy_filt = 0.0
