"""
ticker.py - 定时器驱动模块（基于 smartcar 库 ticker 类）
【参考】Coreboard_Demo/E08_ticker_demo.py
【功能】
  - 封装 ticker 创建、回调、计数
  - 支持 capture_list 绑定（编码器、IMU、按键等）
  - 提供分频触发判断（如每 N 次触发一次）
"""

from smartcar import ticker


class Ticker:
    def __init__(self, pit_id=1, period_ms=10, count_mod=100):
        self.period_ms = period_ms
        self.count_mod = count_mod
        self.flag = False
        self.count = 0
        self._pit = ticker(pit_id)
        self._pit.callback(self._handler)

    def _handler(self, t):
        self.flag = True
        self.count = (self.count + 1) % self.count_mod

    def bind_capture(self, *objs):
        self._pit.capture_list(*objs)

    def start(self):
        self._pit.start(self.period_ms)

    def stop(self):
        self._pit.stop()

    def triggered(self, mod=1):
        if self.flag and self.count % mod == 0:
            return True
        return False

    def clear(self):
        self.flag = False
