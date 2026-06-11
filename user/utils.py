"""
utils.py - 公共工具函数
【功能】
  - normalize_angle: 角度归一化到 [-180, 180]
  - limit_value: 数值限幅
"""

import math

def normalize_angle(angle_deg):
    """角度归一化到 [-180, 180]"""
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg

def limit_value(val, limit):
    """限幅"""
    return max(-limit, min(val, limit))
