"""
UWB 跟踪模块 — UWBTracker 类
====================================================
功能:
  1. 通过 UART0 (115200) 接收 TWR 基站数据
  2. 解析 JSON 帧，提取距离 (D) 和角度 (P)
  3. 低通滤波，死区处理，目标锚点过滤
  4. 接近停车状态机：≤0.20m 停车，>0.25m 重新启动
  5. 超时检测：>800ms 无数据视为超时

硬件接线:
  UART0 — RT1021 默认串口 (TX/RX)

使用示例:
  tracker = UWBTracker()
  while True:
      cmd = tracker.get_command()
      if cmd:
          speed, angle = cmd
          # 驱动电机...
      if tracker.is_timeout():
          # 停车...
      time.sleep_ms(10)
"""

import gc, time, json
from machine import UART


class UWBTracker:
    """UWB 跟踪控制器 — 接收 TWR 数据，返回 (速度, 角度) 指令。"""

    def __init__(self, uart_id=0, baudrate=115200, target_anchor="8834",
                 stop_dist_m=0.20):
        # --- UART 初始化 ---
        self._uart = UART(uart_id)
        self._uart.init(baudrate=baudrate, bits=8, parity=None, stop=1)

        # --- 缓冲区 ---
        self._rx_line = bytearray()

        # --- 滤波状态 ---
        self._P_FILT_ALPHA = 0.3
        self._D_FILT_ALPHA = 0.3
        self._p_filt = None  # None = 未收到首帧，收到后直接赋值
        self._d_filt = None  # 同上

        # --- 目标锚点 ---
        self._target_anchor = target_anchor

        # --- 接近停车参数 ---
        self._APPROACH_SPEED = 0.30
        self._STOP_DIST_M = stop_dist_m
        self._RESTART_DIST_M = 0.25
        self._ANGLE_DEADBAND = 10
        self._is_stopped = False

        # --- 超时检测 ---
        self._TIMEOUT_MS = 800
        self._last_data_ms = time.ticks_ms()
        self._timeout_stopped = False
        self._frame_count = 0

        print("=== UWBTracker ready (UART{} {} baud, anchor={}) ===".format(
            uart_id, baudrate, target_anchor))

    @staticmethod
    def _parse_json_line(line_str):
        """从可能带有前缀的字符串中提取 JSON 并解析。"""
        try:
            idx = line_str.find('{')
            if idx < 0:
                return None
            return json.loads(line_str[idx:])
        except Exception:
            return None

    def get_command(self):
        """
        非阻塞读取 UART → 解析 TWR 帧 → 滤波 → 状态机。
        返回 (speed, angle_deg) 或 None。
        """
        # --- 非阻塞读取字节到缓冲区 ---
        last_cmd = None
        if self._uart is not None and self._uart.any():
            raw = self._uart.read(self._uart.any())
            if raw:
                for i in range(len(raw)):
                    b = raw[i]

                    # 行结束: \\r (0x0D) 或 \\n (0x0A)
                    if b == 0x0D or b == 0x0A:
                        if len(self._rx_line) > 0:
                            # bytearray → str
                            line_str = ''
                            for c in self._rx_line:
                                line_str += chr(c)

                            result = self._process_line(line_str)
                            self._rx_line = bytearray()
                            if result is not None:
                                last_cmd = result  # 保存最后有效命令，继续处理同批剩余帧
                        continue

                    self._rx_line.append(b)
                    if len(self._rx_line) > 200:
                        self._rx_line = bytearray()

        return last_cmd

    def _process_line(self, line_str):
        """处理一行文本：解析 JSON → 滤波 → 状态机。"""
        data = self._parse_json_line(line_str)
        if data is None or 'TWR' not in data:
            return None

        twr = data['TWR']
        anchor = twr.get('a16', '?')

        # 目标锚点过滤
        if self._target_anchor is not None and str(anchor) != self._target_anchor:
            return None

        d_cm = twr.get('D', 0)
        angle_p = twr.get('P', 0)

        # 有效帧 — 更新计时
        self._frame_count += 1
        self._last_data_ms = time.ticks_ms()
        self._timeout_stopped = False

        # --- 低通滤波（首帧直接赋值，不从 0 开始发散） ---
        if self._p_filt is None:
            self._p_filt = angle_p
        else:
            self._p_filt = self._P_FILT_ALPHA * angle_p + (1 - self._P_FILT_ALPHA) * self._p_filt

        if self._d_filt is None:
            self._d_filt = float(d_cm)
        else:
            self._d_filt = self._D_FILT_ALPHA * d_cm + (1 - self._D_FILT_ALPHA) * self._d_filt

        # 电机角度 = 取反
        angle_for_motor = -self._p_filt

        # 死区
        if abs(angle_for_motor) < self._ANGLE_DEADBAND:
            angle_for_motor = 0

        dist_m_filt = self._d_filt / 100.0

        # Debug 打印（匹配 uwb_following.py 风格）
        print("[{}] a={} D={} D_filt={:.1f} P_raw={:+.0f} P_filt={:+.0f} -> mot={:+.0f}".format(
            self._frame_count, anchor, d_cm, self._d_filt,
            angle_p, self._p_filt, angle_for_motor))

        # --- 接近停车状态机 ---
        if self._is_stopped:
            if dist_m_filt > self._RESTART_DIST_M:
                self._is_stopped = False
                print("UWBTracker: restart (dist={:.2f}m)".format(dist_m_filt))
                return (self._APPROACH_SPEED, angle_for_motor)
        else:
            if dist_m_filt <= self._STOP_DIST_M:
                self._is_stopped = True
                print("UWBTracker: stopped (dist={:.2f}m)".format(dist_m_filt))
                return None
            else:
                return (self._APPROACH_SPEED, angle_for_motor)

        return None

    def is_timeout(self):
        """如果超过 800ms 未收到有效帧，返回 True。"""
        if time.ticks_diff(time.ticks_ms(), self._last_data_ms) > self._TIMEOUT_MS:
            if not self._timeout_stopped:
                self._timeout_stopped = True
                print("UWBTracker: timeout — no data for {}ms".format(self._TIMEOUT_MS))
            return True
        return False

    def get_distance(self):
        """返回滤波后的距离（米）。未收到首帧时返回 inf。"""
        if self._d_filt is None:
            return float('inf')
        return self._d_filt / 100.0

    def get_angle(self):
        """返回滤波后的角度（度，取反后用于电机）。"""
        if self._p_filt is None:
            return 0.0
        return -self._p_filt

    def is_near_target(self):
        """返回是否已接近目标（距离 ≤ 停车距离）。
        仅在收到过至少一帧有效 UWB 数据后生效，避免初始值误触发。"""
        if self._frame_count == 0:
            return False
        return self._d_filt / 100.0 <= self._STOP_DIST_M

    def stop(self):
        """停止 UART 并释放资源。"""
        print("UWBTracker: stopping UART...")
        if self._uart is not None:
            try:
                self._uart.deinit()
            except Exception:
                pass
            self._uart = None
        print("UWBTracker: stopped.")
