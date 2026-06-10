"""
kalman_filter.py — 一维卡尔曼滤波器（三通道独立滤波）
【层】信号处理层
【参考】beijixiong_gimbal / kalman_filter.c (Hongxi Wang)
【功能】
  - 1 状态标量卡尔曼滤波器 —— 无需矩阵库，适合 MicroPython
  - 继承参考实现的 P_min 防过收敛机制
  - 三通道独立滤波器: ex(像素) / dist(距离) / roll(倾角)
【原理】一维卡尔曼等价于自适应低通滤波器，增益 K 根据估计不确定性自动调节
【使用】
  from kalman_filter import KalmanFilter1D, CameraKalmanFilter
  kf = CameraKalmanFilter()
  ex_f, dist_f, roll_f = kf.update(ex, dist, roll)
【参数调优指南】
  Q (过程噪声) : 越大 → 跟踪越快 → 滤波越弱。反映目标真实运动速度。
  R (量测噪声) : 越大 → 越平滑 → 响应越慢。反映传感器噪声水平。
  P_min        : 防止协方差收敛到 0 导致滤波器"失聪"，值越大越灵敏。
  P0           : 初始协方差，越大则初始几帧更信任量测值。
"""

# ── 单通道卡尔曼滤波器 ──────────────────────────────────────

class KalmanFilter1D:
    """一维（1 状态）卡尔曼滤波器，对标量信号降噪"""

    def __init__(self, Q=1.0, R=10.0, P0=50.0, x0=0.0, P_min=0.01,
                 name='kf'):
        """
        Q     : 过程噪声协方差 —— 反映状态自身的随机游走强度
        R     : 量测噪声协方差 —— 反映传感器的噪声方差
        P0    : 初始估计误差协方差 —— 大值 = 初期更信任量测
        x0    : 初始状态估计
        P_min : 最小协方差下限 —— 防止滤波器"过度收敛"后无法跟踪状态变化
        name  : 标签，用于调试
        """
        self.Q = Q
        self.R = R
        self.P_min = P_min
        self.name = name

        self.x_hat = x0       # 状态估计（滤波输出）
        self.P = P0           # 估计误差协方差
        self.K = 0.0          # 卡尔曼增益（只读，用于观测收敛情况）

        self._initialized = False if x0 == 0.0 and P0 > 10.0 else True

    def update(self, z, valid=True):
        """
        输入量测值 z，返回滤波估计值。
        valid=False 时跳过量测更新，仅维持当前估计。
        """
        if not valid:
            return self.x_hat

        # 首次有效量测：直接初始化
        if not self._initialized:
            self.x_hat = z
            self._initialized = True
            return self.x_hat

        # ── 1. 计算卡尔曼增益 ──
        #    K = P / (P + R)
        self.K = self.P / (self.P + self.R)

        # ── 2. 状态更新（量测修正）──
        #    x_hat = x_hat + K * (z - x_hat)
        innovation = z - self.x_hat
        self.x_hat = self.x_hat + self.K * innovation

        # ── 3. 协方差更新 ──
        #    P = (1 - K) * P + Q
        self.P = (1.0 - self.K) * self.P + self.Q

        # ── 4. 防过收敛（继承自参考实现）──
        #    当 P 过小时，滤波器对新量测几乎无响应 → 失聪
        if self.P < self.P_min:
            self.P = self.P_min

        return self.x_hat

    def predict_only(self):
        """仅预测（无有效量测时调用）：保持估计不变，注入过程噪声"""
        self.P = self.P + self.Q
        if self.P < self.P_min:
            self.P = self.P_min
        return self.x_hat

    def reset(self, x0=0.0, P0=50.0):
        """重置滤波器状态"""
        self.x_hat = x0
        self.P = P0
        self.K = 0.0
        self._initialized = (P0 <= 10.0)

    def status(self):
        """返回单行状态字符串，用于调试"""
        return "{}: x={:+.2f} P={:.3f} K={:.3f}".format(
            self.name, self.x_hat, self.P, self.K)


# ── 三通道摄像头滤波器 ──────────────────────────────────────

class CameraKalmanFilter:
    """三个独立的一维卡尔曼滤波器，分别处理 ex / dist / roll"""

    # 默认参数 —— 需在实车上根据噪声水平调优
    #            Q    R    P_min   (量测噪声 R 应 ≈ 传感器方差)
    _DEFAULT_PARAMS = {
        'ex':   dict(Q=1.5,  R=12.0, P_min=0.3),   # 像素偏移，噪声约 ±3.5 px
        'dist': dict(Q=1.0,  R=8.0,  P_min=0.3),   # 距离值，噪声约 ±2.8
        'roll': dict(Q=0.5,  R=15.0, P_min=0.2),   # 倾角，噪声约 ±3.9°
    }

    def __init__(self, dt=None):
        """
        dt : 保留参数（用于未来扩展二状态模型），当前未使用
        """
        self.dt = dt

        # 创建三个独立滤波器
        p = self._DEFAULT_PARAMS
        self.kf_ex   = KalmanFilter1D(name='ex',   **p['ex'])
        self.kf_dist = KalmanFilter1D(name='dist', **p['dist'])
        self.kf_roll = KalmanFilter1D(name='roll', **p['roll'])

    def update(self, ex, dist, roll, valid=True):
        """
        输入三个原始通道值，返回滤波后的 (ex_f, dist_f, roll_f)。
        valid=False 时所有通道仅预测、不更新。
        """
        ex_f   = self.kf_ex.update(ex, valid)
        dist_f = self.kf_dist.update(dist, valid)
        roll_f = self.kf_roll.update(roll, valid)
        return ex_f, dist_f, roll_f

    def predict_only(self):
        """当无有效量测时，仅维持预测"""
        return (self.kf_ex.predict_only(),
                self.kf_dist.predict_only(),
                self.kf_roll.predict_only())

    def reset(self):
        """重置全部三个滤波器"""
        for kf in (self.kf_ex, self.kf_dist, self.kf_roll):
            kf.reset()

    def status_string(self):
        """返回三通道滤波器状态"""
        return " | ".join(kf.status() for kf in
                          (self.kf_ex, self.kf_dist, self.kf_roll))

    def gains(self):
        """返回三通道当前卡尔曼增益 (K_ex, K_dist, K_roll)"""
        return (self.kf_ex.K, self.kf_dist.K, self.kf_roll.K)
