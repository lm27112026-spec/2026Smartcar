"""
uwb_position.py — UWB 定位模块
【功能】
  - 通过 UART0 (115200) 接收 TWR 基站数据
  - 解析 JSON 帧，提取距离和相对坐标
  - 互补滤波器平滑数据
  - 条件触发存储：当移动距离超过阈值时自动保存坐标到 location 数组
  - SW2 按键退出：按下 SW2 时程序自动退出
  - 提供位置查询和历史记录管理接口
【使用】
  pos = UWBPosition()
  while True:
      if not pos.step():  # 返回 False 表示 SW2 被按下
          break
      x, y = pos.get_position()
      dist, angle = pos.get_distance_angle()
      if pos.is_timeout():
          print("UWB timeout")
      time.sleep_ms(10)
  pos.stop()
【数据格式】
  location 数组中每个元素为字典：
  {
      'x': float,      # X坐标 (cm)
      'y': float,      # Y坐标 (cm)
      'distance': float, # 距离 (cm)
      'angle': float,    # 角度 (度)
      'timestamp': int   # 时间戳 (ms)
  }
【依赖】无外部依赖，仅使用标准库
【按键】SW2 (D9 引脚) - 边沿触发，按下后程序退出
"""

import gc, time, math, json
from machine import UART, Pin


class UWBPosition:
    """UWB 定位器 — 连续接收 UWB 数据，提供坐标输出和条件存储。"""

    # ── 滤波系数 ──
    D_FILT_ALPHA = 0.15      # 距离低通
    XY_FILT_ALPHA = 0.10     # 坐标低通
    ANGLE_FILT_ALPHA = 0.10  # 角度低通

    # ── 超时 ──
    TIMEOUT_MS = 800

    # ── 条件存储参数 ──
    STORE_DISTANCE_CM = 10.0  # 移动超过此距离才存储 (cm)

    # ── 按键引脚 ──
    SWITCH2_PIN = 'D9'

    def __init__(self, uart_id=0, baudrate=115200, target_anchor="8834"):
        """
        初始化 UWB 定位模块。
        
        参数:
            uart_id: UART 编号 (默认 0)
            baudrate: 波特率 (默认 115200)
            target_anchor: 目标锚点 ID (默认 "8834")
        """
        # ── UART 初始化 ──
        self._uart = UART(uart_id)
        self._uart.init(baudrate=baudrate, bits=8, parity=None, stop=1)
        self._rx_line = bytearray()
        self._target_anchor = target_anchor

        # ── SW2 按键初始化 ──
        self._switch2 = Pin(self.SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
        self._switch2_state = self._switch2.value()  # 记录初始状态
        self._switch2_pressed = False

        # ── 滤波状态变量 ──
        self._d_filt = None
        self._x_filt = None
        self._y_filt = None
        self._angle_filt = None

        # ── 时间管理 ──
        self._last_data_ticks = time.ticks_ms()
        self._timeout_stopped = False
        self._frame_count = 0

        # ── 位置存储 ──
        self.location = []  # 位置历史数组
        self._last_store_x = None
        self._last_store_y = None

        # ── 当前坐标 ──
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_distance = 0.0
        self._current_angle = 0.0

        print("=== UWBPosition ready (UART{} {} baud, anchor={}) ===".format(
            uart_id, baudrate, target_anchor))

    # ============================================================
    #  内部：JSON 解析
    # ============================================================
    @staticmethod
    def _parse_json_line(line_str):
        """解析 JSON 行数据。"""
        try:
            idx = line_str.find('{')
            if idx < 0:
                return None
            return json.loads(line_str[idx:])
        except Exception:
            return None

    # ============================================================
    #  内部：计算两点间距离
    # ============================================================
    @staticmethod
    def _calculate_distance(x1, y1, x2, y2):
        """计算两点间的欧几里得距离。"""
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # ============================================================
    #  公共接口：单次步进（每次主循环调用一次）
    # ============================================================
    def step(self):
        """
        单次迭代：读 UART → 滤波 → 条件存储。
        
        可在主循环中每 10ms 调用一次。
        
        返回:
            bool: True 表示程序应继续运行，False 表示 SW2 被按下应退出
        """
        # ── SW2 按键检测（边沿触发） ──
        current_switch = self._switch2.value()
        if current_switch != self._switch2_state:
            self._switch2_state = current_switch
            self._switch2_pressed = True
            print("SW2 pressed - program will exit")
            return False

        # ── 超时保护 ──
        if time.ticks_diff(time.ticks_ms(), self._last_data_ticks) > self.TIMEOUT_MS:
            if not self._timeout_stopped:
                self._timeout_stopped = True
                print("UWBPosition: timeout — no data for {}ms".format(self.TIMEOUT_MS))

        # ── 非阻塞读取 UART ──
        if self._uart is not None and self._uart.any():
            raw = self._uart.read(self._uart.any())
            if raw:
                for i in range(len(raw)):
                    b = raw[i]

                    # 行结束符
                    if b == 0x0D or b == 0x0A:
                        if len(self._rx_line) > 0:
                            line_str = ''
                            for c in self._rx_line:
                                line_str += chr(c)
                            self._process_line(line_str)
                            self._rx_line = bytearray()
                        continue

                    self._rx_line.append(b)
                    if len(self._rx_line) > 200:
                        self._rx_line = bytearray()

        return True

    # ============================================================
    #  内部：处理一行 UWB 数据（滤波 + 条件存储）
    # ============================================================
    def _process_line(self, line_str):
        """处理一行 UWB 数据。"""
        data = self._parse_json_line(line_str)
        if data is None or 'TWR' not in data:
            return

        twr = data['TWR']
        anchor = twr.get('a16', '?')
        d_cm = twr.get('D', 0)
        x_cm = twr.get('Xcm', 0)
        y_cm = twr.get('Ycm', 0)

        # ── 跳过非目标锚点 ──
        if self._target_anchor is not None and str(anchor) != self._target_anchor:
            return

        # ── 目标锚点：更新计时 ──
        self._frame_count += 1
        if self._frame_count % 50 == 0:
            gc.collect()
        self._last_data_ticks = time.ticks_ms()
        self._timeout_stopped = False

        # ── 低通滤波 ──
        if self._x_filt is None:
            self._x_filt = float(x_cm)
            self._y_filt = float(y_cm)
        else:
            self._x_filt = self.XY_FILT_ALPHA * x_cm + (1 - self.XY_FILT_ALPHA) * self._x_filt
            self._y_filt = self.XY_FILT_ALPHA * y_cm + (1 - self.XY_FILT_ALPHA) * self._y_filt

        angle_to_target = math.atan2(-self._x_filt, self._y_filt) * 180.0 / math.pi
        if self._angle_filt is None:
            self._angle_filt = angle_to_target
        else:
            self._angle_filt = self.ANGLE_FILT_ALPHA * angle_to_target + (
                1 - self.ANGLE_FILT_ALPHA) * self._angle_filt

        if self._d_filt is None:
            self._d_filt = float(d_cm)
        else:
            self._d_filt = self.D_FILT_ALPHA * d_cm + (1 - self.D_FILT_ALPHA) * self._d_filt

        # ── 更新当前坐标 ──
        self._current_x = self._x_filt
        self._current_y = self._y_filt
        self._current_distance = self._d_filt
        self._current_angle = self._angle_filt

        # ── 条件存储：移动距离超过阈值时自动保存 ──
        self._check_and_store()

        # ── Debug 打印 ──
        if self._frame_count % 10 == 0:  # 每10帧打印一次，减少输出
            print("[{}] a={} D={:.1f} X={:.1f} Y={:.1f} ang={:+.0f}° loc_cnt={}".format(
                self._frame_count, anchor, self._d_filt,
                self._x_filt, self._y_filt, self._angle_filt,
                len(self.location)))

    # ============================================================
    #  内部：检查并存储坐标
    # ============================================================
    def _check_and_store(self):
        """检查是否满足存储条件，满足则存储当前坐标。"""
        # 首次收到数据时直接存储
        if self._last_store_x is None:
            self._store_position()
            return

        # 计算与上次存储位置的距离
        dist = self._calculate_distance(
            self._last_store_x, self._last_store_y,
            self._current_x, self._current_y
        )

        # 超过阈值则存储
        if dist >= self.STORE_DISTANCE_CM:
            self._store_position()

    # ============================================================
    #  内部：存储当前位置
    # ============================================================
    def _store_position(self):
        """将当前位置存储到 location 数组。"""
        point = {
            'x': self._current_x,
            'y': self._current_y,
            'distance': self._current_distance,
            'angle': self._current_angle,
            'timestamp': time.ticks_ms()
        }
        self.location.append(point)
        self._last_store_x = self._current_x
        self._last_store_y = self._current_y
        print("Position stored: ({:.1f}, {:.1f}) - Total: {}".format(
            self._current_x, self._current_y, len(self.location)))

    # ============================================================
    #  公共查询接口
    # ============================================================

    def get_position(self):
        """
        获取当前滤波后的坐标。
        
        返回:
            (x, y): 元组，x 和 y 坐标 (cm)
        """
        return (self._current_x, self._current_y)

    def get_distance_angle(self):
        """
        获取当前滤波后的距离和角度。
        
        返回:
            (distance, angle): 元组，距离 (cm) 和角度 (度)
        """
        return (self._current_distance, self._current_angle)

    def get_raw_position(self):
        """
        获取未滤波的原始坐标（最后一次接收到的原始数据）。
        
        返回:
            (x, y): 元组，原始 x 和 y 坐标 (cm)
        """
        if self._x_filt is None:
            return (0.0, 0.0)
        # 注意：这里返回的是滤波后的值，因为原始值没有单独保存
        # 如需原始值，需要添加额外的变量存储
        return (self._x_filt, self._y_filt)

    def is_timeout(self):
        """超过 TIMEOUT_MS 未收到有效帧返回 True。"""
        if time.ticks_diff(time.ticks_ms(), self._last_data_ticks) > self.TIMEOUT_MS:
            if not self._timeout_stopped:
                self._timeout_stopped = True
                print("UWBPosition: timeout — no data for {}ms".format(self.TIMEOUT_MS))
            return True
        return False

    def is_switch2_pressed(self):
        """
        检查 SW2 按键是否被按下。
        
        返回:
            bool: True 表示 SW2 被按下过，False 表示未按下
        """
        return self._switch2_pressed

    def reset_switch2(self):
        """重置 SW2 按键状态。"""
        self._switch2_pressed = False
        self._switch2_state = self._switch2.value()

    def get_frame_count(self):
        """返回已处理的帧数。"""
        return self._frame_count

    def get_location_count(self):
        """返回已存储的位置点数量。"""
        return len(self.location)

    def get_last_stored_position(self):
        """
        获取最后一个存储的位置点。
        
        返回:
            dict 或 None: 最后一个位置点字典，无数据返回 None
        """
        if len(self.location) == 0:
            return None
        return self.location[-1]

    def get_location_history(self):
        """
        获取完整的位置历史记录。
        
        返回:
            list: location 数组的副本
        """
        return self.location.copy()

    def clear_location_history(self):
        """清空位置历史记录。"""
        self.location.clear()
        self._last_store_x = None
        self._last_store_y = None
        print("Location history cleared.")

    def set_store_distance(self, distance_cm):
        """
        设置条件存储的触发距离阈值。
        
        参数:
            distance_cm: 触发距离 (cm)，移动超过此距离自动存储
        """
        self.STORE_DISTANCE_CM = distance_cm
        print("Store distance threshold set to {:.1f} cm".format(distance_cm))

    def manual_store(self):
        """
        手动存储当前位置（不受距离阈值限制）。
        
        返回:
            bool: 是否成功存储
        """
        if self._x_filt is None:
            print("No UWB data received yet.")
            return False
        self._store_position()
        return True

    # ============================================================
    #  清理
    # ============================================================
    def stop(self):
        """停止 UART。"""
        print("UWBPosition: stopping...")
        if self._uart is not None:
            try:
                self._uart.deinit()
            except Exception:
                pass
            self._uart = None
        print("UWBPosition: stopped.")

    def __del__(self):
        """析构函数，确保资源释放。"""
        self.stop()


# ============================================================
#  简单测试代码
# ============================================================
if __name__ == '__main__':
    pos = UWBPosition()
    print("UWB Position Test Started")
    print("Move the tag to see coordinates change...")
    print("Location will be stored when tag moves > {} cm".format(pos.STORE_DISTANCE_CM))
    print("Press SW2 to exit program")
    
    try:
        while True:
            # step() 返回 False 表示 SW2 被按下
            if not pos.step():
                print("SW2 detected - exiting...")
                break
            
            x, y = pos.get_position()
            dist, angle = pos.get_distance_angle()
            
            # 每秒打印一次当前状态
            if pos.get_frame_count() % 100 == 0:
                print("Current: X={:.1f}cm Y={:.1f}cm D={:.1f}cm A={:.1f}° | Stored: {} points".format(
                    x, y, dist, angle, pos.get_location_count()))
            
            time.sleep_ms(10)
    
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    
    finally:
        print("Total stored positions: {}".format(pos.get_location_count()))
        pos.stop()
