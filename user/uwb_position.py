"""
uwb_position.py — UWB 定位模块
【功能】
  - 通过 UART0 (115200) 接收 TWR 基站数据
  - 解析 JSON 帧，提取距离和相对坐标
  - 互补滤波器平滑数据
  - 条件触发存储：当移动距离超过阈值时自动保存坐标到 location 数组
  - 提供位置查询和历史记录管理接口
【使用】
  pos = UWBPosition()
  while True:
      pos.step()  # 内部处理 UART + 滤波 + 条件存储
      x, y = pos.get_position()
      dist, angle = pos.get_distance_angle()
      if pos.is_timeout():
          print("UWB timeout")
      time.sleep_ms(10)
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
"""

import gc, time, math, json
from machine import Pin, UART


class LKF1D:
    """一维极简自包含卡尔曼滤波器"""

    def __init__(self, q=1.8, r=8.0):
        self.q = q  # 过程噪声协方差 (越大越灵敏)
        self.r = r  # 测量噪声协方差 (越大越平滑)
        self.x = None  # 状态估计值
        self.p = 1.0   # 估计协方差

    def update(self, measurement):
        if self.x is None:
            self.x = measurement
            return self.x
        # 预测
        p_prior = self.p + self.q
        # 更新
        k = p_prior / (p_prior + self.r)
        self.x = self.x + k * (measurement - self.x)
        self.p = (1.0 - k) * p_prior
        return self.x

    def reset(self):
        self.x = None
        self.p = 1.0


class MedianFilter:
    """滑动窗口中值滤波器"""

    def __init__(self, window_size=5):
        self._buf = []
        self._window = window_size

    def update(self, value):
        self._buf.append(value)
        if len(self._buf) > self._window:
            self._buf.pop(0)
        tmp = sorted(self._buf)
        n = len(tmp)
        return tmp[n // 2]

    def reset(self):
        self._buf.clear()


class UWBPosition:
    """UWB 定位器 — 采用中值剔噪 + 轻量卡尔曼平滑的双重低延迟滤波体系"""

    # ── 滤波窗口调优 ──
    MEDIAN_WINDOW = 5         

    # ── [新增] 默认关闭自动轨迹记录，彻底消除 GC 碎片与约 80KB 内存开销 ──
    ENABLE_LOCATION_HISTORY = False  # 调试或需要画轨迹时可手动改为 True

    # ── 突变剔除阈值（配合小车物理加速，防止数据断锁冻结） ──
    OUTLIER_DIST_CM = 35.0    # 距离突变阈值 (cm)
    OUTLIER_XY_CM = 30.0      # XY 突变阈值 (cm)
    OUTLIER_RAW_XY_CM = 45.0  # 原始数据突变阈值 (cm)

    TIMEOUT_MS = 2000             # 从 800ms 放宽到 2000ms，避免短暂 EMI/噪声爆发误判断连
    RAW_TIMEOUT_MS = 5000         # 原始UART数据超时 (ms) — 区分"硬件断连"与"数据帧校验拒绝"
    OUTLIER_STALE_MS = 500        # 参考值过期阈值 (ms) — 超过此时间跳过突变检测，打破拒绝死锁
    
    STORE_DISTANCE_CM = 10.0  
    
    # [Fix 2] 防止 location 数组过大导致内存溢出 (OOM)，设置最大缓存上限
    MAX_LOCATION_POINTS = 500  

    # ── 按键引脚 ──
    SWITCH2_PIN = 'D9'

    def __init__(self, uart_id=0, baudrate=115200, target_anchor="8834"):
        self._uart = UART(uart_id)
        self._uart.init(baudrate=baudrate, bits=8, parity=None, stop=1)
        self._rx_line = bytearray()
        self._target_anchor = target_anchor

        self._switch2 = Pin(self.SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
        self._switch2_state = self._switch2.value()  
        self._switch2_pressed = False

        # ── 状态变量 ──
        self._d_filt = None
        self._x_filt = None
        self._y_filt = None
        self._angle_filt = None

        # ── 中值滤波器 ──
        self._med_d = MedianFilter(self.MEDIAN_WINDOW)
        self._med_x = MedianFilter(self.MEDIAN_WINDOW)
        self._med_y = MedianFilter(self.MEDIAN_WINDOW)

        # ── 内置自包含卡尔曼平滑器 ──
        self._kf_x = LKF1D(q=1.8, r=8.0)
        self._kf_y = LKF1D(q=1.8, r=8.0)
        self._kf_d = LKF1D(q=2.0, r=6.0)
        self._kf_angle = LKF1D(q=2.5, r=5.0)

        # ── 上一次有效值参考 ──
        self._last_valid_d = None
        self._last_valid_x = None
        self._last_valid_y = None
        self._last_raw_x = None  
        self._last_raw_y = None

        # ── 运行时间与诊断 ──
        self._last_data_ticks = time.ticks_ms()
        self._last_rx_ticks = time.ticks_ms()     # UART原始字节活跃时间戳 (区分硬件断连)
        self._last_raw_ticks = 0                  # Layer1 参考值更新时间戳
        self._last_valid_ticks = 0                # Layer2 参考值更新时间戳
        self._timeout_stopped = False
        self._timeout_start_ms = 0     
        self._frame_count = 0
        self._uart_rx_count = 0  
        self._step_count = 0     
        self._reject_raw_jump = 0  
        self._reject_med_jump = 0  

        # ── 位置存储 ──
        self.location = []  
        self._last_store_x = None
        self._last_store_y = None

        # ── 当前坐标 ──
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_distance = 0.0
        self._current_angle = 0.0

        # ── 原始测量值 (滤波前，用于对比诊断) ──
        self._raw_x = None
        self._raw_y = None
        self._raw_d = None

        print("=== UWBPosition ready (UART{} {} baud, anchor={}) ===".format(
            uart_id, baudrate, target_anchor))

    @staticmethod
    def _parse_json_line(line_str):
        try:
            idx = line_str.find('{')
            if idx < 0:
                return None
            return json.loads(line_str[idx:])
        except Exception:
            return None

    @staticmethod
    def _calculate_distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def step(self):
        if self._frame_count < 5:
            self._switch2_state = self._switch2.value()
        else:
            current_switch = self._switch2.value()
            if current_switch != self._switch2_state:
                self._switch2_state = current_switch
                self._switch2_pressed = True
                print("SW2 pressed - program will exit")
                return False

        # [Fix 7] 分离职责，纯查询与副作用分离
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, self._last_data_ticks) > self.TIMEOUT_MS:
            self._handle_timeout(now_ms)

        if self._uart is not None and self._uart.any():
            raw = self._uart.read(self._uart.any())
            if raw:
                self._uart_rx_count += len(raw)
                self._last_rx_ticks = time.ticks_ms()  # 记录原始UART活跃时间，即使帧被校验拒绝
                for i in range(len(raw)):
                    b = raw[i]

                    if b == 0x0D or b == 0x0A:
                        if len(self._rx_line) > 0:
                            # [Fix 6] 抛弃逐字拼接的高开销循环，使用 MicroPython 优化的一次性 decode
                            try:
                                line_str = self._rx_line.decode('utf-8')
                                self._process_line(line_str)
                            except Exception:
                                pass # 忽略可能由串口物理噪声引起的帧解析异常
                            self._rx_line = bytearray()
                        continue

                    self._rx_line.append(b)
                    if len(self._rx_line) > 200:
                        self._rx_line = bytearray()
        else:
            if self._step_count > 0 and self._step_count % 200 == 0:
                rx_age = time.ticks_diff(time.ticks_ms(), self._last_rx_ticks)
                print("[diag] step={} frame={} uart_bytes={} rx_total={} rx_age={}ms rej_raw={} rej_med={}".format(
                    self._step_count, self._frame_count,
                    self._uart.any() if self._uart else -1,
                    self._uart_rx_count, rx_age,
                    self._reject_raw_jump, self._reject_med_jump))

        self._step_count += 1
        if self._step_count % 500 == 0:  # 从200延长到500，降低GC频率减少UART数据丢失风险
            uart_pending = self._uart.any() if self._uart else 0
            if uart_pending > 32:
                print("[diag] GC with uart_pending={} — possible buffer pressure".format(uart_pending))
            gc.collect()
        return True

    def _process_line(self, line_str):
        data = self._parse_json_line(line_str)
        if data is None or 'TWR' not in data:
            return

        twr = data['TWR']
        anchor = twr.get('a16', '?')
        try:
            d_cm = float(twr.get('D', 0))
            x_cm = float(twr.get('Xcm', 0))
            y_cm = float(twr.get('Ycm', 0))
        except (ValueError, TypeError):
            return  

        if self._target_anchor is not None and str(anchor) != self._target_anchor:
            return

        # ── 第 1 层：原始值跳变检测（参考值过期则跳过，打破拒绝死锁） ──
        if self._last_raw_x is not None:
            raw_stale = time.ticks_diff(time.ticks_ms(), self._last_raw_ticks) > self.OUTLIER_STALE_MS
            if not raw_stale:
                if abs(x_cm - self._last_raw_x) > self.OUTLIER_RAW_XY_CM or \
                   abs(y_cm - self._last_raw_y) > self.OUTLIER_RAW_XY_CM:
                    self._reject_raw_jump += 1
                    return  # 跳变过大拒绝本帧
        # [Fix 1] 此处不再直接赋值原始参考值，而是将赋值移至所有检测通过（L285 附近）

        # ── [Fix 3] 第 2 层：在数据污染中值滤波器前，先用原始值完成基于最后有效帧的突变检测 ──
        # 此处适当放宽 1.2 倍阈值作为补偿
        # [Fix 9] 参考值过期保护：若上一有效帧超过 OUTLIER_STALE_MS，跳过突变检测
        if self._last_valid_d is not None:
            valid_stale = time.ticks_diff(time.ticks_ms(), self._last_valid_ticks) > self.OUTLIER_STALE_MS
            if not valid_stale:
                if abs(d_cm - self._last_valid_d) > self.OUTLIER_DIST_CM * 1.2:
                    self._reject_med_jump += 1
                    return  
                if abs(x_cm - self._last_valid_x) > self.OUTLIER_XY_CM * 1.2 or \
                   abs(y_cm - self._last_valid_y) > self.OUTLIER_XY_CM * 1.2:
                    self._reject_med_jump += 1
                    return  

        # [Fix 1] 校验全通过，安全地更新 Layer 1 参考基准，防止误杀
        self._last_raw_x = x_cm
        self._last_raw_y = y_cm
        self._last_raw_ticks = time.ticks_ms()  # [Fix 9] 记录参考值更新时间

        self._frame_count += 1
        self._last_data_ticks = time.ticks_ms()
        self._timeout_stopped = False

        # ── 保存原始测量值（滤波前，供 standalone 对比诊断用）──
        self._raw_x = x_cm
        self._raw_y = y_cm
        self._raw_d = d_cm

        # ── [Fix 3] 第 3 层：通过突变校验后，再推入中值窗做二次细节平滑 ──
        d_med = self._med_d.update(d_cm)
        x_med = self._med_x.update(x_cm)
        y_med = self._med_y.update(y_cm)

        # ── 第 4 层：使用内置卡尔曼平滑滤波（已注释，排查用） ──
        # self._x_filt = self._kf_x.update(x_med)
        # self._y_filt = self._kf_y.update(y_med)
        # self._d_filt = self._kf_d.update(d_med)
        self._x_filt = x_med
        self._y_filt = y_med
        self._d_filt = d_med

        # ── 角度解算 ──
        angle_raw = math.atan2(self._x_filt, self._y_filt) * 180.0 / math.pi

        # ── [Fix 5] 角度卡尔曼差值极界 wrap-around 处理（已注释，排查用）──
        # if self._angle_filt is not None:
        #     diff = angle_raw - self._angle_filt
        #     if diff > 180.0:
        #         angle_raw -= 360.0
        #     elif diff < -180.0:
        #         angle_raw += 360.0
        # 
        # self._angle_filt = self._kf_angle.update(angle_raw)
        self._angle_filt = angle_raw

        # ── 更新历史有效值 ──
        self._last_valid_d = d_med
        self._last_valid_x = x_med
        self._last_valid_y = y_med
        self._last_valid_ticks = time.ticks_ms()  # [Fix 9] 记录有效帧时间戳，供Layer2过期判断

        # ── 更新对外的当前坐标 ──
        self._current_x = self._x_filt
        self._current_y = self._y_filt
        self._current_distance = self._d_filt
        self._current_angle = self._angle_filt

        # ── 🟢 修改点：仅在开启历史记录开关时，才进行位置差异比对与存储 ──
        if self.ENABLE_LOCATION_HISTORY:
            self._check_and_store()

        if self._frame_count % 10 == 0:
            print("[{}] a={} D={:.1f} X={:.1f} Y={:.1f} ang={:+.0f}° loc_cnt={} [NO_KF]".format(
                self._frame_count, anchor, self._d_filt,
                self._x_filt, self._y_filt, self._angle_filt,
                len(self.location)))

    def _check_and_store(self):
        if self._last_store_x is None:
            self._store_position()
            return

        dist = self._calculate_distance(
            self._last_store_x, self._last_store_y,
            self._current_x, self._current_y
        )

        if dist >= self.STORE_DISTANCE_CM:
            self._store_position()

    def _store_position(self):
        point = {
            'x': self._current_x,
            'y': self._current_y,
            'distance': self._current_distance,
            'angle': self._current_angle,
            'timestamp': time.ticks_ms()
        }
        self.location.append(point)
        
        # [Fix 2] FIFO 淘汰最旧的记录点，保障在长距离测试中不会发生堆碎片导致的 OOM
        if len(self.location) > self.MAX_LOCATION_POINTS:
            self.location.pop(0)

        self._last_store_x = self._current_x
        self._last_store_y = self._current_y
        print("Position stored: ({:.1f}, {:.1f}) - Total: {}".format(
            self._current_x, self._current_y, len(self.location)))

    # ============================================================
    #  接口函数
    # ============================================================
    def get_position(self):
        """获取当前卡尔曼滤波输出的估计位置坐标。"""
        return (self._current_x, self._current_y)

    def get_distance_angle(self):
        return (self._current_distance, self._current_angle)

    def get_filtered_position(self):
        """[Fix 4] 语义修正：获取最后一帧经卡尔曼滤波稳定后的位置（原 get_raw_position 修正）"""
        if self._x_filt is None:
            return (0.0, 0.0)
        return (self._x_filt, self._y_filt)

    def get_raw_position(self):
        """[Fix 4] 保留原名字别名兼容层，防止调用方（如旧版主控代码）报错"""
        return self.get_filtered_position()

    def get_latest_raw(self):
        """[Fix 4] 语义修正：获取最后一帧通过校验的物理原始测量值坐标"""
        return (self._last_raw_x, self._last_raw_y)

    def get_raw_measurement(self):
        """获取最后一帧有效数据的原始解析值（滤波前），用于对比诊断。
        返回: (raw_x, raw_y, raw_d, filt_x, filt_y, filt_d) 或 None"""
        if self._raw_x is None:
            return None
        return (self._raw_x, self._raw_y, self._raw_d,
                self._x_filt, self._y_filt, self._d_filt)

    def is_timeout(self):
        """[Fix 7] 纯只读属性查询方法，不再掺杂状态改变的副作用"""
        return time.ticks_diff(time.ticks_ms(), self._last_data_ticks) > self.TIMEOUT_MS

    def is_uart_alive(self):
        """[Fix 9] 检查UART硬件层是否仍有数据流入。
        用于区分'硬件真正断连'与'数据帧校验拒绝'两种场景。
        若UART仍有字节流入但is_timeout()为True，说明是校验层问题而非硬件断连。"""
        return time.ticks_diff(time.ticks_ms(), self._last_rx_ticks) <= self.RAW_TIMEOUT_MS

    def _handle_timeout(self, now_ms):
        """[Fix 7] 剥离出的超时动作处理内部私有方法"""
        if not self._timeout_stopped:
            self._timeout_stopped = True
            self._timeout_start_ms = now_ms
            uart_bytes = self._uart.any() if self._uart else -1
            rx_age = time.ticks_diff(now_ms, self._last_rx_ticks)
            alive = "UART_ALIVE" if self.is_uart_alive() else "UART_DEAD"
            print("UWBPosition: timeout — uart_bytes={} frame={} rx_age={}ms {} rej={}/{}".format(
                uart_bytes, self._frame_count, rx_age, alive,
                self._reject_raw_jump, self._reject_med_jump))

    def get_frame_count(self):
        return self._frame_count

    def get_location_count(self):
        return len(self.location)

    def get_last_stored_position(self):
        if len(self.location) == 0:
            return None
        return self.location[-1]

    def get_location_history(self):
        return self.location.copy()

    def clear_location_history(self):
        self.location.clear()
        self._last_store_x = None
        self._last_store_y = None
        print("Location history cleared.")

    def set_store_distance(self, distance_cm):
        self.STORE_DISTANCE_CM = distance_cm
        print("Store distance threshold set to {:.1f} cm".format(distance_cm))

    def uwb_record(self):
        if self._x_filt is None:
            print("No UWB data received yet.")
            return False
        self._store_position()
        return True

    def reset_uart(self):
        if self._uart is not None:
            try:
                self._uart.deinit()
            except Exception:
                pass
        time.sleep_ms(100)
        self._uart = UART(0)
        self._uart.init(baudrate=115200, bits=8, parity=None, stop=1)
        self._rx_line = bytearray()
        self._last_data_ticks = time.ticks_ms()
        self._last_rx_ticks = time.ticks_ms()     # 重置UART活跃时间戳
        self._last_raw_ticks = 0
        self._last_valid_ticks = 0
        self._timeout_stopped = False
        self._timeout_start_ms = 0
        self._reject_raw_jump = 0
        self._reject_med_jump = 0
        
        self._med_d.reset()
        self._med_x.reset()
        self._med_y.reset()
        self._kf_x.reset()
        self._kf_y.reset()
        self._kf_d.reset()
        self._kf_angle.reset()
        self._last_valid_d = None
        self._last_valid_x = None
        self._last_valid_y = None
        self._last_raw_x = None
        self._last_raw_y = None
        print("UWBPosition: UART reinitialized and filters reset")

    def is_alive(self):
        return not self._timeout_stopped

    def stop(self):
        """纯引用释放 — 仅置空 UART 引用，不触发物理 deinit，杜绝 GC 析构死锁"""
        self._uart = None

    def __del__(self):
        self.stop()


# ═══════════════════════════════════════════════════════════════
#  Standalone 模式：监测 或 导航到固定坐标
#  运行方式: 在 MicroPython REPL 中 import uwb_position 或直接执行
#  配置: 修改下方 NAVIGATE_TO 和导航参数
# ═══════════════════════════════════════════════════════════════
if __name__ == '__main__':

    # ── 🎯 导航目标配置 ──
    # 设为 None → 纯监测模式（滤波前后实时对比）
    # 设为 (x_cm, y_cm) → 导航到指定 UWB 坐标点
    NAVIGATE_TO = None          # 例: (0.0, 150.0) 表示导航到 (0, 150) cm

    # ── 导航控制参数（仅在导航模式生效） ──
    NAV_KP              = 0.009   # 位置误差 → 速度 P 增益
    NAV_DB              = 10.0    # 到位死区 (cm)
    NAV_SLOW_DIST       = 20.0    # 减速起始距离 (cm)
    NAV_MAX_SPEED       = 0.50    # 最大速度 (m/s)
    NAV_MIN_SPEED       = 0.06    # 最小速度 (m/s)
    NAV_CTRL_DT         = 0.02    # 控制周期 (s)
    NAV_TIMEOUT_S       = 30.0    # 超时 (s)
    NAV_ARRIVAL_FRAMES  = 5       # 连续 N 帧在死区内判定到达
    NAV_HEADING_KP      = 0.8     # 航向 P 增益
    NAV_HEADING_DB      = 2.0     # 航向死区 (度)

    # ═══════════════════════════════════════════════════════════
    #  导航模式
    # ═══════════════════════════════════════════════════════════
    if NAVIGATE_TO is not None:
        # 条件导入 — 仅导航模式需要电机/IMU 模块
        from motor import (stop_all, omni_drive_closed_loop,
                           get_encoder_counts, ENC_SCALE)
        from imu_motion import update_angle, imu_read_safe
        import imu_motion

        def _yaw():
            return imu_motion.yaw

        def _read_imu_update_yaw():
            try:
                gx, gy, gz = imu_read_safe()
                if gx is not None:
                    update_angle(gx, gy, gz, 0.01)
                    return True
            except Exception:
                pass
            return False

        def _heading_correction(target_yaw):
            """简单 P 控制航向修正"""
            error = target_yaw - _yaw()
            # 角度 wrap
            if error > 180.0:
                error -= 360.0
            elif error < -180.0:
                error += 360.0
            if abs(error) < NAV_HEADING_DB:
                return 0.0
            return error * NAV_HEADING_KP

        # ── 初始化 ──
        pos = UWBPosition()
        target_x, target_y = NAVIGATE_TO

        # 锁定航向
        for _ in range(20):
            if _read_imu_update_yaw():
                break
            time.sleep_ms(10)
        target_heading = _yaw()

        print("")
        print("=" * 65)
        print("  UWB Navigate — Target ({:.1f}, {:.1f}) cm".format(target_x, target_y))
        print("  KP={:.3f}  DB={:.1f}cm  MaxSpd={:.2f}m/s  Heading={:.1f}°".format(
            NAV_KP, NAV_DB, NAV_MAX_SPEED, target_heading))
        print("=" * 65)
        print("{:>6} | {:>8} {:>8} | {:>8} {:>8} | {:>7} {:>7} | {:>7} | {}".format(
            "Frame", "CurX", "CurY", "ErrX", "ErrY",
            "Vx", "Vy", "Dist", "Status"))
        print("-" * 65)

        start_ms = time.ticks_ms()
        last_print_ms = start_ms
        loop_cnt = 0
        near_count = 0

        while True:
            # ── UWB 数据采集 ──
            if not pos.step():
                print("\n  [NAV] SW2 按下，中断导航")
                stop_all()
                break

            # ── 超时检测 ──
            elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            if elapsed > NAV_TIMEOUT_S:
                print("  [NAV] 超时 ({:.1f}s)".format(elapsed))
                stop_all()
                break

            # ── 获取当前位置 ──
            cx, cy = pos.get_position()
            if cx is None or cy is None:
                time.sleep_ms(10)
                continue

            # ── 位置误差 (世界坐标系) ──
            err_x = target_x - cx
            err_y = target_y - cy
            dist = math.sqrt(err_x * err_x + err_y * err_y)

            # ── 到达判定：连续 N 帧在死区内 ──
            if dist < NAV_DB:
                near_count += 1
                if near_count >= NAV_ARRIVAL_FRAMES:
                    print("\n  [NAV] ✅ 到达目标！dist={:.1f}cm  cur=({:.1f},{:.1f})".format(
                        dist, cx, cy))
                    stop_all()
                    break
            else:
                near_count = 0

            # ── 机体坐标系转换 ──
            yaw_rad = math.radians(_yaw())
            cos_y = math.cos(yaw_rad)
            sin_y = math.sin(yaw_rad)

            # 世界误差 → 机体速度指令 (旋转矩阵)
            vx_world = err_x * NAV_KP
            vy_world = err_y * NAV_KP
            vx = vx_world * cos_y + vy_world * sin_y
            vy = -vx_world * sin_y + vy_world * cos_y

            # ── 速度限幅 ──
            spd = math.sqrt(vx * vx + vy * vy)
            if spd > NAV_MAX_SPEED:
                scale = NAV_MAX_SPEED / spd
                vx *= scale
                vy *= scale
            elif spd < NAV_MIN_SPEED and spd > 0.001:
                # 距离较远时保证最小速度克服静摩擦
                if dist > NAV_SLOW_DIST:
                    scale = NAV_MIN_SPEED / spd
                    vx *= scale
                    vy *= scale

            # ── 航向修正 ──
            _read_imu_update_yaw()
            wz = _heading_correction(target_heading)

            # ── 编码器速度闭环驱动 ──
            counts = get_encoder_counts()
            if counts and len(counts) >= 4:
                try:
                    rs = [counts[i] / ENC_SCALE[i] / NAV_CTRL_DT
                          if ENC_SCALE[i] != 0 else 0 for i in range(4)]
                    omni_drive_closed_loop(vx, vy, wz, rs, NAV_CTRL_DT)
                except Exception as e:
                    print("  [NAV] 驱动异常:", e)

            # ── 状态打印 ──
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_print_ms) >= 500:
                last_print_ms = now_ms
                raw = pos.get_raw_measurement()
                raw_str = ""
                if raw:
                    raw_str = " | raw=({:.1f},{:.1f})".format(raw[0], raw[1])
                status = "TIMEOUT" if pos.is_timeout() else ("NEAR" if near_count > 0 else "OK")
                print("{:>6} | {:>8.1f} {:>8.1f} | {:>+7.1f} {:>+7.1f} | {:>+6.2f} {:>+6.2f} | {:>6.1f} | {}{}".format(
                    pos._frame_count, cx, cy, err_x, err_y,
                    vx, vy, dist, status, raw_str))

            # ── GC ──
            loop_cnt += 1
            if loop_cnt % 50 == 0:
                gc.collect()
            time.sleep_ms(int(NAV_CTRL_DT * 1000))

        pos.stop()
        print("  [NAV] 已停止")

    # ═══════════════════════════════════════════════════════════
    #  纯监测模式（NAVIGATE_TO = None）
    # ═══════════════════════════════════════════════════════════
    else:
        pos = UWBPosition()
        print("")
        print("=" * 75)
        print("  UWB Position Monitor — Raw vs Filtered 实时对比")
        print("  按下 SW2 键退出")
        print("=" * 75)
        print("{:>6} | {:>8} {:>8} {:>8} | {:>8} {:>8} {:>8} | {:>7} {:>7} {:>7} | {}".format(
            "Frame", "RawX", "RawY", "RawD",
            "FiltX", "FiltY", "FiltD",
            "dX", "dY", "dD", "Status"))
        print("-" * 75)

        last_print_ms = time.ticks_ms()
        PRINT_INTERVAL_MS = 200

        while True:
            if not pos.step():
                print("\n  [MONITOR] SW2 按下，退出监测")
                break

            raw = pos._raw_x
            if raw is None:
                time.sleep_ms(10)
                continue

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print_ms) < PRINT_INTERVAL_MS:
                time.sleep_ms(10)
                continue
            last_print_ms = now

            rx = pos._raw_x
            ry = pos._raw_y
            rd = pos._raw_d
            fx = pos._x_filt
            fy = pos._y_filt
            fd = pos._d_filt

            dx = rx - fx if (rx is not None and fx is not None) else 0.0
            dy = ry - fy if (ry is not None and fy is not None) else 0.0
            dd = rd - fd if (rd is not None and fd is not None) else 0.0

            status = "TIMEOUT" if pos.is_timeout() else "OK"
            if pos.is_timeout():
                uart_alive = "UART_OK" if pos.is_uart_alive() else "NO_UART"
                status = "{}/{}".format(status, uart_alive)

            print("{:>6} | {:>8.1f} {:>8.1f} {:>8.1f} | {:>8.1f} {:>8.1f} {:>8.1f} | {:>+6.1f} {:>+6.1f} {:>+6.1f} | {}".format(
                pos._frame_count, rx, ry, rd,
                fx, fy, fd,
                dx, dy, dd, status))

        pos.stop()
        print("  [MONITOR] 已停止")