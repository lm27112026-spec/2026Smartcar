"""
UWB 跟踪模块 — UWBTracker 类
====================================================
功能:
   1. 通过 UART0 (115200) 接收 TWR 基站数据
   2. 解析 JSON 帧，提取距离 (D) 和相对坐标 (Xcm, Ycm)
   3. 用 atan2(Xcm, Ycm) 计算方向角，低通滤波距离，死区处理
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

import gc, time, json, math
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

        # --- 滤波状态（D 低通；Xcm/Ycm 低通防近距离跳变） ---
        self._D_FILT_ALPHA = 0.3
        self._XY_FILT_ALPHA = 0.25
        self._d_filt = None  # None = 未收到首帧
        self._x_filt = None
        self._y_filt = None

        # --- 目标锚点 ---
        self._target_anchor = target_anchor

        # --- 接近停车参数 ---
        self._APPROACH_SPEED = 0.50
        self._FULL_SPEED_DIST_M = 0.50   # ≥此距离时全速
        self._MIN_APPROACH_SPEED = 0.25  # 最低逼近速度
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

    def _ramp_speed(self, dist_m):
        """距离越近速度越慢：STOP_DIST(0.20)→0 线性到 FULL_SPEED_DIST(0.50)→APPROACH_SPEED。
        最低不低于 _MIN_APPROACH_SPEED，防止 PWM 低于电机死区导致卡住（开环 omni_drive
        无死区补偿，低速时反转轮尤其容易停转）。"""
        if dist_m >= self._FULL_SPEED_DIST_M:
            return self._APPROACH_SPEED
        if dist_m <= self._STOP_DIST_M:
            return 0.0
        t = (dist_m - self._STOP_DIST_M) / (self._FULL_SPEED_DIST_M - self._STOP_DIST_M)
        speed = self._APPROACH_SPEED * t
        if speed < self._MIN_APPROACH_SPEED and dist_m > self._STOP_DIST_M:
            return self._MIN_APPROACH_SPEED
        return speed

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
        x_cm = twr.get('Xcm', 0)
        y_cm = twr.get('Ycm', 0)

        # 有效帧 — 更新计时
        self._frame_count += 1
        self._last_data_ms = time.ticks_ms()
        self._timeout_stopped = False

        # --- 距离低通滤波（首帧直接赋值） ---
        if self._d_filt is None:
            self._d_filt = float(d_cm)
        else:
            self._d_filt = self._D_FILT_ALPHA * d_cm + (1 - self._D_FILT_ALPHA) * self._d_filt

        # hypot(Xcm,Ycm) 仅诊断用（和 D 不一致时辅助判断）
        coord_d = math.sqrt(x_cm * x_cm + y_cm * y_cm)

        # --- Xcm/Ycm 低通滤波（防近距离单帧跳变） ---
        if self._x_filt is None:
            self._x_filt = float(x_cm)
            self._y_filt = float(y_cm)
        else:
            self._x_filt = self._XY_FILT_ALPHA * x_cm + (1 - self._XY_FILT_ALPHA) * self._x_filt
            self._y_filt = self._XY_FILT_ALPHA * y_cm + (1 - self._XY_FILT_ALPHA) * self._y_filt

        # --- 用滤波后的Xcm/Ycm计算方向角 ---
        # atan2(x, y): x>0=锚点在右侧, x<0=左侧, y>0=前方
        self._last_x = self._x_filt
        self._last_y = self._y_filt
        angle_to_target = math.atan2(-self._x_filt, self._y_filt) * 180.0 / math.pi
        self._last_angle_to_target = angle_to_target

        # 电机角度 = atan2 结果 → omni_move_by_angle 坐标系（0°=右、90°=前）
        angle_for_motor = 90 - angle_to_target

        # 死区（防止 Xcm ≈ 0 时方向微抖）
        if abs(angle_for_motor) < self._ANGLE_DEADBAND:
            angle_for_motor = 0

        dist_m_filt = self._d_filt / 100.0
        ramp_speed = self._ramp_speed(dist_m_filt)

        # Debug 打印（含 hypot 对比 + 滤波后坐标 + 实际速度）
        print("[{}] a={} D={} hypot={:.0f} Df={:.1f} X={} Y={} xf={:.0f} yf={:.0f} ang={:+.0f}° -> mot={:+.0f}° speed={:.2f}".format(
            self._frame_count, anchor, d_cm, coord_d, self._d_filt,
            x_cm, y_cm, self._x_filt, self._y_filt, angle_to_target, angle_for_motor, ramp_speed))

        # --- 接近停车状态机（带速度衰减） ---
        if self._is_stopped:
            if dist_m_filt > self._RESTART_DIST_M:
                self._is_stopped = False
                speed = self._ramp_speed(dist_m_filt)
                print("UWBTracker: restart (dist={:.2f}m, speed={:.2f})".format(dist_m_filt, speed))
                return (speed, angle_for_motor)
        else:
            if dist_m_filt <= self._STOP_DIST_M:
                self._is_stopped = True
                print("UWBTracker: stopped (dist={:.2f}m)".format(dist_m_filt))
                return None
            else:
                speed = self._ramp_speed(dist_m_filt)
                return (speed, angle_for_motor)

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
        """返回锚点相对车头的方向（度），正=右侧，负=左侧。"""
        return getattr(self, '_last_angle_to_target', 0.0)

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
