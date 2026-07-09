"""
uwb_control.py — UWB 统一模块（定位 + 导航控制）

【层】
  - 定位层：通过 UART0 (115200) 接收 TWR 基站数据，解析 JSON 帧，经门限剔噪 +
    中值平滑滤波后输出稳定坐标
  - 导航层：基于定位坐标的 PID 闭环导航到目标点，含航向保持、机体坐标系变换、
    到达死区判定、减速曲线、速度限幅及 UWB 离线检测与自动恢复

【职责】
  - MedianFilter：滑动窗口中值滤波器
  - UWBPosition：UART 数据采集、帧解析、双层门限跳变检测、中值滤波、条件位置存储
  - goto_location()：通用 UWB 闭环导航函数
   - GOTO_* 参数：全部集中于 uwb_follow.py，调试时修改该文件常量

【使用】
  # ── 定位 ──
  pos = UWBPosition()
  while True:
      pos.step()
      x, y = pos.get_position()
      dist, angle = pos.get_distance_angle()
      if pos.is_timeout():
          print("UWB timeout")
      time.sleep_ms(10)

  # ── 导航 ──
  arrived, reason = goto_location(
      uwb, target_x, target_y,
      lock_heading_fn, calc_wz_fn, get_yaw_fn,
      should_abort_fn, drive_fn, stop_fn,
      led_fn=led_fn, label="GOTO"
  )

【数据格式】
  location 数组中每个元素为字典：
  {
      'x': float,      # X坐标 (cm)
      'y': float,      # Y坐标 (cm)
      'distance': float, # 距离 (cm)
      'angle': float,    # 角度 (度)
      'timestamp': int   # 时间戳 (ms)
  }
"""

import gc, time, math, json
from machine import Pin, UART

# ── 独立滤波与控制器模块 ──
from median_filter import MedianFilter
from uwb_follow import (UWBFollowController,
                        GOTO_ZONE_DX, GOTO_ZONE_DY,
                        GOTO_KP, GOTO_SLOW_DIST,
                        GOTO_MAX_SPEED, GOTO_MIN_SPEED,
                        GOTO_OUTPUT_DEADZONE, GOTO_LPF_ALPHA,
                        GOTO_TIMEOUT_S, GOTO_CTRL_DT, GOTO_ARRIVAL_FRAMES,
                        GOTO_UWB_STEP_MS, GOTO_UWB_DEAD_TIMEOUT_S,
                        GOTO_UWB_MAX_RECONNECT, GOTO_UWB_RECONNECT_WAIT_MS,
                        GOTO_PRINT_INTERVAL_MS)

# ── UWB + 编码器互补滤波融合 ──
from uwb_fusion import UWBComplementaryFilter
from motor import get_chassis_speeds_from_raw



# ═══════════════════════════════════════════════════════════════
#  UWBPosition — 定位器
# ═══════════════════════════════════════════════════════════════

class UWBPosition:
    """UWB 定位器 — 采用门限剔噪 + 中值平滑的低延迟滤波体系

    滤波链（3 层）:
      Layer 1: 原始值跳变检测 (OUTLIER_RAW_XY_CM) — 丢弃突变帧
      Layer 2: 有效帧突变检测 (OUTLIER_DIST_CM / OUTLIER_XY_CM) — 丢弃突变帧
      Layer 3: 滑动窗口中值滤波 + 速度门限收尾 (MEDIAN_WINDOW / MEDIAN_MAX_STEP) — 抑制噪声 + 限制帧间跳变
    """

    # ── 滤波窗口调优 ──
    MEDIAN_WINDOW = 4
    MEDIAN_MAX_STEP = 0         # 不限速 — 帧间限速平滑职责已迁移至互补滤波 (uwb_fusion.py, α=0.95)

    # ── 突变剔除阈值（配合小车物理加速，防止数据断锁冻结） ──
    OUTLIER_DIST_CM = 20.0    # 距离突变阈值 (cm)
    OUTLIER_XY_CM = 15.0      # XY 突变阈值 (cm)
    OUTLIER_RAW_XY_CM = 30.0  # 原始数据突变阈值 (cm)

    TIMEOUT_MS = 2000             # 从 800ms 放宽到 2000ms，避免短暂 EMI/噪声爆发误判断连
    RAW_TIMEOUT_MS = 5000         # 原始UART数据超时 (ms) — 区分"硬件断连"与"数据帧校验拒绝"
    OUTLIER_STALE_MS = 500        # 参考值过期阈值 (ms) — 超过此时间跳过突变检测，打破拒绝死锁
    
    STORE_DISTANCE_CM = 10.0  
    
    # [Fix 2] 防止 location 数组过大导致内存溢出 (OOM)，设置最大缓存上限
    MAX_LOCATION_POINTS = 100  

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

        # ── 状态变量（中值滤波输出，直接对外）──
        self._d_filt = None
        self._x_filt = None
        self._y_filt = None
        self._angle_filt = None

        # ── 中值滤波器（唯一平滑层，含速度门限收尾）──
        self._med_d = MedianFilter(self.MEDIAN_WINDOW, self.MEDIAN_MAX_STEP)
        self._med_x = MedianFilter(self.MEDIAN_WINDOW, self.MEDIAN_MAX_STEP)
        self._med_y = MedianFilter(self.MEDIAN_WINDOW, self.MEDIAN_MAX_STEP)

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
        self._reject_zero = 0      # 零值帧拦截计数（硬件丢信号全0）  

        # ── 位置存储 ──
        self.location = []  
        self._last_store_x = None
        self._last_store_y = None

        # ── 当前坐标 ──
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_distance = 0.0
        self._current_angle = 0.0

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
                print("[diag] step={} frame={} uart_bytes={} rx_total={} rx_age={}ms rej_raw={} rej_med={} rej_zero={}".format(
                    self._step_count, self._frame_count,
                    self._uart.any() if self._uart else -1,
                    self._uart_rx_count, rx_age,
                    self._reject_raw_jump, self._reject_med_jump, self._reject_zero))

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

        # ── 零值拦截：硬件丢信号时输出全 0 默认值，丢弃不污染滤波器 ──
        if d_cm == 0.0:
            self._reject_zero += 1
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

        # ── [Fix 3] 第 3 层：通过突变校验后，推入中值窗平滑 ──
        d_med = self._med_d.update(d_cm)
        x_med = self._med_x.update(x_cm)
        y_med = self._med_y.update(y_cm)

        # ── 中值输出直接作为最终滤波值（KF 已移除）──
        self._x_filt = x_med
        self._y_filt = y_med
        self._d_filt = d_med

        # ── 角度解算（基于中值坐标）──
        angle_raw = math.atan2(self._x_filt, self._y_filt) * 180.0 / math.pi

        # ── 角度 wrap-around 处理 + 中值平滑 ──
        if self._angle_filt is not None:
            diff = angle_raw - self._angle_filt
            if diff > 180.0:
                angle_raw -= 360.0
            elif diff < -180.0:
                angle_raw += 360.0

        self._angle_filt = angle_raw

        # ── 更新历史有效值（中值输出）──
        self._last_valid_d = d_med
        self._last_valid_x = x_med
        self._last_valid_y = y_med
        self._last_valid_ticks = time.ticks_ms()  # [Fix 9] 记录有效帧时间戳，供Layer2过期判断

        # ── 更新对外的当前坐标 ──
        self._current_x = self._x_filt
        self._current_y = self._y_filt
        self._current_distance = self._d_filt
        self._current_angle = self._angle_filt

        self._check_and_store()

        if self._frame_count % 10 == 0:
            print("[{}] a={} D={:.1f} X={:.1f} Y={:.1f} ang={:+.0f}° loc_cnt={}".format(
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
        """获取中值滤波输出位置（门限剔噪 + 中值平滑后的坐标）"""
        if self._x_filt is None:
            return (0.0, 0.0)
        return (self._x_filt, self._y_filt)

    def get_raw_position(self):
        """别名兼容层 — 等效 get_filtered_position()"""
        return self.get_filtered_position()

    def get_latest_raw(self):
        """[Fix 4] 语义修正：获取最后一帧通过校验的物理原始测量值坐标"""
        return (self._last_raw_x, self._last_raw_y)

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
            print("UWBPosition: timeout — uart_bytes={} frame={} rx_age={}ms {} rej={}/{}/{}".format(
                uart_bytes, self._frame_count, rx_age, alive,
                self._reject_raw_jump, self._reject_med_jump, self._reject_zero))

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
        self._reject_zero = 0
        
        self._med_d.reset()
        self._med_x.reset()
        self._med_y.reset()
        self._last_valid_d = None
        self._last_valid_x = None
        self._last_valid_y = None
        self._last_raw_x = None
        self._last_raw_y = None
        print("UWBPosition: UART reinitialized and filters reset")

    def is_alive(self):
        return not self._timeout_stopped

    def stop(self):
        print("UWBPosition: stopping...")
        if self._uart is not None:
            try:
                self._uart.deinit()
            except Exception:
                pass
            self._uart = None
        print("UWBPosition: stopped.")

    def __del__(self):
        self.stop()


# ═══════════════════════════════════════════════════════════════
#  goto_location() — 通用 UWB 导航到目标坐标
# ═══════════════════════════════════════════════════════════════

def goto_location(uwb, target_x, target_y,
                  lock_heading_fn, calc_wz_fn, get_yaw_fn,
                  should_abort_fn, drive_fn, stop_fn,
                  get_encoder_counts_fn=None, enc_scale=None,
                  drive_with_speeds_fn=None,
                  fusion=None,
                  led_fn=None, label="GOTO", on_progress=None,
                  verbose=True):
    """UWB 闭环导航到目标坐标点（含编码器互补滤波融合）。

    参数（回调函数，由调用方注入）:
        uwb:              UWBPosition 实例
        target_x:         float  目标 X 坐标 (cm)
        target_y:         float  目标 Y 坐标 (cm)
        lock_heading_fn:  () -> float              锁定并返回目标航向角 (度)
        calc_wz_fn:       (target_deg) -> float  计算航向修正 wz（死区由 IMU_hold 全局控制）
        get_yaw_fn:       () -> float              获取当前航向角 (度)
        should_abort_fn:  () -> bool               检查 SW2/看门狗中断
        drive_fn:         (vx, vy, wz, dt) -> None 驱动电机（UWB 离线时使用）
        stop_fn:          () -> None               停止电机
        get_encoder_counts_fn: () -> [int×4]       编码器脉冲增量（用于融合）
        enc_scale:        [float×4]                编码器标定因子 (脉冲/m)
        drive_with_speeds_fn: (vx,vy,wz,dt,actual) -> None  驱动并传入预计算轮速
        led_fn:           (bool) -> None           LED 控制，None 不控制
        label:            str                      打印前缀标签
        on_progress:      (dist_cm, x, y) -> None  每帧位置更新回调

    到达判定: 全局坐标单边区间 + 多帧对齐
        -GOTO_ZONE_DX ≤ error_x ≤ 0  &&  -GOTO_ZONE_DY ≤ error_y ≤ 0
        即 target_x ≤ x ≤ target_x + GOTO_ZONE_DX
        连续 GOTO_ARRIVAL_FRAMES 帧满足才判定到达（防 UWB 噪声单帧误判）

    返回:
        (arrived: bool, reason: str)
    """
    # ── 编码器回调可用性检测 ──
    enc_available = (get_encoder_counts_fn is not None and enc_scale is not None)
    drive_uses_speeds = (drive_with_speeds_fn is not None)
    print("\n  [{}] === 导航到目标 ({:.1f}, {:.1f}) ===".format(label, target_x, target_y))

    target_heading = lock_heading_fn()
    if verbose:
        print("  [{}] 航向锁定: {:.1f}°".format(label, target_heading))

    # ── 创建跟随控制器（读取当前模块常量，支持 state_machine 动态覆写）──
    ctrl = UWBFollowController()
    if verbose:
        print("  [{}] 控制器: kp={} slow={} vmax={} vmin={} zone=({},{}) dz={} lpf={}".format(
            label, ctrl.kp, ctrl.slow_dist,
            ctrl.max_speed, ctrl.min_speed,
            GOTO_ZONE_DX, GOTO_ZONE_DY,
            ctrl.output_deadzone, ctrl.lpf_alpha))

    if led_fn:
        led_fn(True)

    # ── 实例化 / 复用融合滤波器 ──
    if fusion is None:
        fusion = UWBComplementaryFilter(uwb_gain=0.08) if enc_available else None
    if fusion is not None and verbose:
        print("  [{}] 编码器前馈融合已就绪 (uwb_gain={:.2f})".format(label, fusion.uwb_gain))

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    last_uwb_ms = start_ms
    loop_cnt = 0
    uwb_dead_start = 0
    uwb_reconnect_count = 0
    arrival_count = 0  # 连续在 zone 内的帧数（多帧对齐判定）

    while True:
        # ── 中断检测 ──
        if should_abort_fn():
            if led_fn:
                led_fn(False)
            return (False, 'aborted')

        # ── 总超时检测 ──
        elapsed_total = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed_total > GOTO_TIMEOUT_S:
            print("  [{}] 超时 ({:.1f}s)".format(label, elapsed_total))
            if led_fn:
                led_fn(False)
            return (False, 'timeout')

        # ── UWB 数据采集（按固定间隔） ──
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_uwb_ms) >= GOTO_UWB_STEP_MS:
            uwb.step()
            last_uwb_ms = now_ms

        # ── UWB 离线检测与自动重连 ──
        if uwb.is_timeout():
            if uwb_dead_start == 0:
                uwb_dead_start = time.ticks_ms()
                if uwb.is_uart_alive():
                    print("  [{}] UWB 帧超时但 UART 正常，等待噪声消退...".format(label))
                else:
                    print("  [{}] UWB UART 硬件断连，等待恢复...".format(label))
            elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > GOTO_UWB_DEAD_TIMEOUT_S * 1000:
                # [Fix 10] UART 活着时不做 reset_uart()，只延长等待
                if uwb.is_uart_alive():
                    print("  [{}] UART 仍活跃，延长等待...".format(label))
                    uwb_dead_start = time.ticks_ms()
                elif uwb_reconnect_count < GOTO_UWB_MAX_RECONNECT:
                    uwb_reconnect_count += 1
                    print("  [{}] UART 断连超时 {:.0f}s，第 {}/{} 次尝试重连...".format(
                        label, GOTO_UWB_DEAD_TIMEOUT_S,
                        uwb_reconnect_count, GOTO_UWB_MAX_RECONNECT))
                    try:
                        uwb.reset_uart()
                        # 分段等待 + 保持航向（避免 500ms 盲等）
                        for _ in range(GOTO_UWB_RECONNECT_WAIT_MS // 50):
                            time.sleep_ms(50)
                            wz = calc_wz_fn(target_heading)
                            if abs(wz) > 0.001:
                                drive_fn(0, 0, wz, GOTO_CTRL_DT)
                        # 等待首帧数据
                        wait_start = time.ticks_ms()
                        while uwb.get_frame_count() == 0 or uwb.is_timeout():
                            uwb.step()
                            if time.ticks_diff(time.ticks_ms(), wait_start) > 3000:
                                break
                            # 保持航向
                            wz = calc_wz_fn(target_heading)
                            if abs(wz) > 0.001:
                                drive_fn(0, 0, wz, GOTO_CTRL_DT)
                            time.sleep_ms(10)
                        if not uwb.is_timeout() and uwb.get_frame_count() > 0:
                            print("  [{}] UWB 重连成功，继续导航".format(label))
                            uwb_dead_start = 0
                            last_uwb_ms = time.ticks_ms()
                            ctrl.reset()  # 重连后重置 LPF 状态
                            continue
                    except Exception as e:
                        print("  [{}] UWB 重连异常:".format(label), e)
                    uwb_dead_start = time.ticks_ms()  # 重置计时，准备下次重连
                else:
                    print("  [{}] UWB 全部 {} 次重连均失败，放弃导航".format(
                        label, GOTO_UWB_MAX_RECONNECT))
                    if led_fn:
                        led_fn(False)
                    return (False, 'uwb_lost')
            # ── 离线期间保持航向（死区由 IMU_hold.HOLD_DEADBAND 全局控制）──
            wz = calc_wz_fn(target_heading)
            if abs(wz) > 0.001:
                drive_fn(0, 0, wz, GOTO_CTRL_DT)
            time.sleep_ms(int(GOTO_CTRL_DT * 1000))
            continue
        else:
            uwb_dead_start = 0
            uwb_reconnect_count = 0  # 恢复后重置重连计数

        # ── 获取 UWB 原始值（旁路中值滤波，减 ~2帧延迟）──
        _rx, _ry = uwb.get_latest_raw()
        raw_x = _rx if _rx is not None else uwb.get_position()[0]
        raw_y = _ry if _ry is not None else uwb.get_position()[1]
        yaw_deg = get_yaw_fn()
        
        # ── 自动重同步：状态跳转后位置变化 >50cm 时直接设位置，不经过 None 初始化 ──
        if fusion is not None and fusion.x is not None:
            if abs(fusion.x - raw_x) > 50.0 or abs(fusion.y - raw_y) > 50.0:
                fusion.reinit(raw_x, raw_y)
        
        # ── 编码器互补滤波：读取编码器一次，同时供给融合器和驱动闭环 ──
        wheel_spd = None  # 预初始化，离线场景可能不读取
        if enc_available:
            enc_counts = get_encoder_counts_fn()
            if enc_counts and len(enc_counts) >= 4:
                wheel_spd = [enc_counts[i] / enc_scale[i] / GOTO_CTRL_DT
                             if enc_scale[i] != 0 else 0 for i in range(4)]
                enc_vx, enc_vy = get_chassis_speeds_from_raw(wheel_spd)
                curr_x, curr_y = fusion.update(raw_x, raw_y, enc_vx, enc_vy, yaw_deg, GOTO_CTRL_DT)
            else:
                curr_x, curr_y = raw_x, raw_y
        else:
            curr_x, curr_y = raw_x, raw_y
        
        error_x = target_x - curr_x
        error_y = target_y - curr_y
        dist = math.sqrt(error_x * error_x + error_y * error_y)

        # ── 进度回调（用于外部激活摄像头等外设）──
        if on_progress:
            on_progress(dist, curr_x, curr_y)

        # ── 跟随控制器: 机体变换 + P控制 + 减速曲线 + 限幅 + 死区 + LPF ──
        vx_cmd, vy_cmd = ctrl.compute(error_x, error_y, yaw_deg, dist)

        # ── 末端强制减速带：距目标 10cm 内限速，过冲后仍有动力微调修正 ──
        if dist < 10.0:
            spd = math.sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd)
            if spd > 0.05:
                scale = 0.05 / spd
                vx_cmd *= scale
                vy_cmd *= scale

        # ── 到达判定 (全局坐标单边区间 + 多帧对齐) ──
        if (-GOTO_ZONE_DX <= error_x <= 0) and (-GOTO_ZONE_DY <= error_y <= 0):
            arrival_count += 1
            if arrival_count >= GOTO_ARRIVAL_FRAMES:
                print("  [{}] 到达 ({:.1f},{:.1f}) err=({:.1f},{:.1f}) 连续{}帧".format(
                    label, curr_x, curr_y, error_x, error_y, arrival_count))
                stop_fn()
                if led_fn:
                    led_fn(False)
                return (True, 'arrived')
        else:
            arrival_count = 0  # 离开 zone 立即重置

        # ── 状态打印 ──
        now = time.ticks_ms()
        if verbose and time.ticks_diff(now, last_print_ms) >= GOTO_PRINT_INTERVAL_MS:
            last_print_ms = now
            print("  [{}] pos=({:.1f},{:.1f}) target=({:.1f},{:.1f}) dist={:.1f}cm yaw={:.2f}° vx={:.2f} vy={:.2f}".format(
                label, curr_x, curr_y, target_x, target_y, dist, yaw_deg, vx_cmd, vy_cmd))

        # ── GC ──
        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        # ── 航向修正 ──
        wz = calc_wz_fn(target_heading)

        # ── 驱动电机 ──
        try:
            if enc_available and drive_uses_speeds and wheel_spd is not None:
                drive_with_speeds_fn(vx_cmd, vy_cmd, wz, GOTO_CTRL_DT, wheel_spd)
            else:
                drive_fn(vx_cmd, vy_cmd, wz, GOTO_CTRL_DT)
        except Exception as e:
            print("  [{}] 驱动错误:".format(label), e)

        time.sleep_ms(int(GOTO_CTRL_DT * 1000))


