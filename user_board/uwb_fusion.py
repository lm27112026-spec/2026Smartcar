"""
uwb_fusion.py — UWB + 编码器互补滤波融合（编码器前馈预测版）

【层】控制层
【职责】编码器位移零延迟反映 + UWB 快速修正漂移，消除 DW3000 测距延时影响。

【核心算法】
  enc += v_global * dt * 100                    ← 编码器位移 100% 保留，零衰减零延迟
  enc += uwb_gain * (uwb - enc)                 ← UWB 快速修正漂移 (8%/帧, τ≈125ms)
  output = enc                                   ← 直接输出，不加权
【参数选择】
  uwb_gain = 0.08 — 每帧 UWB 修正 8% 的漂移偏差
  - 漂移修正 τ ≈ -dt/ln(1-0.08) ≈ 125ms (比旧 τ=195ms 快 1.5x)
  - 静止噪声渗透 8% → ±15cm×0.08 = ±1.2cm (比旧方案 0.75cm 稍大，可接受)
  - 运动稳态 offset ≈ dx/0.08 ≈ 5cm @0.5m/s (enc 超前 UWB ≈ 超前实际位置 → 提前减速 → 防过冲)

【量纲约定】
  UWB: cm, 编码器速度: m/s → ×100 转 cm, 输出: cm
"""

import math


class UWBComplementaryFilter:
    """编码器前馈 + UWB 快速修正滤波器。

    参数:
        uwb_gain: UWB 修正速率 (0~1)。越大越快修正漂移，但噪声渗透更多。
                  推荐: 0.08 (漂移修正 τ≈125ms, 噪声 ±1.2cm)
    """

    def __init__(self, uwb_gain=0.08):
        self.uwb_gain = uwb_gain
        self.x = None   # 融合后 X 坐标 (cm)
        self.y = None   # 融合后 Y 坐标 (cm)

    def update(self, uwb_x, uwb_y, enc_vx, enc_vy, yaw_deg, dt):
        """一次融合更新。

        参数:
            uwb_x, uwb_y : UWB 绝对坐标 (cm)
            enc_vx, enc_vy: 编码器底盘线速度 (m/s, 车体坐标系)
            yaw_deg       : 当前航向角 (度)
            dt            : 控制周期 (秒)

        返回:
            (x, y) 融合后坐标 (cm)
        """
        # 1. 坐标系转换：车体速度 → UWB 全局坐标系
        yaw_rad = math.radians(yaw_deg)
        vx_global = enc_vx * math.cos(yaw_rad) - enc_vy * math.sin(yaw_rad)
        vy_global = enc_vx * math.sin(yaw_rad) + enc_vy * math.cos(yaw_rad)

        # 位移: m/s × s × 100 → cm
        dx = vx_global * dt * 100.0
        dy = vy_global * dt * 100.0

        if self.x is None:
            self.x = uwb_x
            self.y = uwb_y
            return self.x, self.y

        # 2. 编码器位移 100% 保留（零衰减，零惯性延迟）
        self.x += dx
        self.y += dy

        # 3. UWB 快速修正漂移（拉回速度 = 8%/帧，天然有界不发散）
        self.x += self.uwb_gain * (uwb_x - self.x)
        self.y += self.uwb_gain * (uwb_y - self.y)

        return self.x, self.y

    def reset(self):
        """重置为未初始化状态（用于 UWB 断连重连后完全重启）。"""
        self.x = None
        self.y = None

    def reinit(self, x, y):
        """直接设位置，不经过 None→init 分支。
        
        用于状态跳转后快速重对齐。与 reset() 的区别：
        - reset(): self.x=None → 下一帧 update 走 init 分支，self.x=uwb(延时)
        - reinit(): self.x=给定值 → 下一帧正常累加编码器位移，无 init 延迟
        """
        self.x = x
        self.y = y
