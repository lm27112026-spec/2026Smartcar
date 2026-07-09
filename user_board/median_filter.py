"""
median_filter.py — 滑动窗口中值滤波器（独立滤波模块）

【层】控制层
【职责】通用滑动窗口中值 + 速度门限收尾滤波器，供 uwb_control 等模块调用

两级处理:
  1. 滑动窗口取中值 — 抑制脉冲噪声
  2. 输出变化率限制 (max_step) — 帧间输出变化不超过 max_step，
     防止中值因窗口数据替换产生跳变，平滑收尾

参数:
    window_size : 滑动窗口大小
    max_step    : 单帧最大允许变化量（与输入同量纲）；0=不限速

【使用】
  from median_filter import MedianFilter
  mf = MedianFilter(window_size=7, max_step=10.0)
  val = mf.update(raw_value)
  mf.reset()
"""


class MedianFilter:
    """滑动窗口中值滤波器 + 速度门限收尾"""

    def __init__(self, window_size=5, max_step=0.0):
        self._buf = []
        self._window = window_size
        self._max_step = max_step
        self._last_out = None

    def update(self, value):
        self._buf.append(value)
        if len(self._buf) > self._window:
            self._buf.pop(0)
        tmp = sorted(self._buf)
        n = len(tmp)
        med = tmp[n // 2]

        # ── 速度门限收尾：限制帧间输出变化量 ──
        if self._max_step > 0 and self._last_out is not None:
            delta = med - self._last_out
            if delta > self._max_step:
                med = self._last_out + self._max_step
            elif delta < -self._max_step:
                med = self._last_out - self._max_step

        self._last_out = med
        return med

    def reset(self):
        self._buf.clear()
        self._last_out = None
