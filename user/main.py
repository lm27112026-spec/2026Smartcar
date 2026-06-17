"""
main.py — UWB跟随 + LED双灯视觉追踪集成（按键控制版）
【流程】
  1. 默认启动 UWB 跟随（uwb_tracker.UWBFollower）— 闭环航向纠偏
  2. 后台轮询摄像头 UART7 → 滑动窗口检测到双灯 → 切换视觉追踪
  3. 视觉追踪（Kalman + 级联 PID）逼近至目标像素距离 → 自动停车
  4. SW2 (D9) 拨码开关全程可强制退出
【摄像头协议】LED.py 双灯追踪（UART7, 115200）
  AA [EX_H EX_L] [EY_H EY_L] [DIST_H DIST_L] [ROLL_H ROLL_L] BB
  int16 大端序 ×10，丢失目标时全零帧
【按键控制】
  KEY1 (C8) → UWB 模式         LED 常亮
  KEY2 (C9) → 视觉追踪模式       LED 快闪 (~2.5Hz)
  KEY3 (C14)→ 停车状态          LED 慢闪 (~1Hz)
【依赖】uwb_tracker.py, kalman_filter.py, control.py, motor.py, imu_motion.py, key.py
【PIT 分配】PIT0=系统, PIT1=编码器(motor), PIT2=看门狗(key), PIT3=IMU
"""

import gc, time, math
from machine import UART, Pin
from motor import (stop_all, enc_ticker,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi)
import imu_motion
from imu_motion import update_angle, reset_ang_vel_pid
from key import capture, key_triggered, pet_watchdog

# ═══════════════════════════════════════════════════════════════
#  常量
# ═══════════════════════════════════════════════════════════════

LED_PIN  = 'C4'
SW2_PIN  = 'D9'

CAM_UART_ID     = 7       # 接收 OpenMV 的 UART
CAM_BAUDRATE    = 115200
CAM_FRAME_HEAD  = 0xAA
CAM_FRAME_TAIL  = 0xBB
CAM_FRAME_LEN   = 10
CAM_TIMEOUT_MS  = 500     # 视觉追踪数据超时（ms）

TARGET_DIST_PX  = 100     # 视觉追踪停车像素距离 — LED.py 协议用（越大=越近，需实车标定）
WINDOW_SIZE     = 8       # 摄像头中断判定滑动窗口大小
WINDOW_THRESHOLD = 5      # 窗口内有效帧数阈值

# 模式
STATE_UWB     = 0
STATE_VISUAL  = 1
STATE_STOPPED = 2

STATE_NAMES = ["UWB_FOLLOW", "VISUAL_TRACK", "STOPPED"]

# ═══════════════════════════════════════════════════════════════
#  硬件初始化
# ═══════════════════════════════════════════════════════════════

led = Pin(LED_PIN, Pin.OUT, value=False)
sw2 = Pin(SW2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)

# ── SW2 消抖状态（纯轮询，不依赖 IRQ / ticker）──
_sw2_last          = sw2.value()
_sw2_pending       = _sw2_last
_sw2_debounce_cnt  = 0
SW2_DEBOUNCE_FRAMES = 5    # 连续 N 帧一致才确认拨动

def check_sw2():
    """读取 SW2 并消抖。返回 True 表示确认发生了切换。"""
    global _sw2_last, _sw2_pending, _sw2_debounce_cnt
    val = sw2.value()
    if val != _sw2_pending:
        _sw2_pending = val
        _sw2_debounce_cnt = 0
    else:
        _sw2_debounce_cnt += 1
        if _sw2_debounce_cnt >= SW2_DEBOUNCE_FRAMES:
            _sw2_debounce_cnt = 0
            _sw2_pending = val
            if val != _sw2_last:
                _sw2_last = val
                return True
    return False


# ═══════════════════════════════════════════════════════════════
#  摄像头 UART7 环形缓冲区（处理粘包 / 断包）
# ═══════════════════════════════════════════════════════════════

class CamRingBuffer:
    """环形缓冲区，跨多次 UART.read() 累积数据并提取 AA..BB 帧。"""

    def __init__(self, size=256):
        self._buf  = bytearray(size)
        self._size = size
        self._head = 0   # 写入位置
        self._tail = 0   # 读取位置

    def feed(self, data):
        """追加原始字节到缓冲区。"""
        for b in data:
            self._buf[self._head] = b
            self._head = (self._head + 1) % self._size
            if self._head == self._tail:           # 满 → 丢弃最旧字节
                self._tail = (self._tail + 1) % self._size

    def get_frame(self):
        """提取第一个完整的 AA..BB 帧。返回 bytes(10) 或 None。"""
        # 收集可用字节
        available = []
        idx = self._tail
        while idx != self._head:
            available.append(self._buf[idx])
            idx = (idx + 1) % self._size

        if len(available) < CAM_FRAME_LEN:
            return None

        data = bytes(available)
        search_offset = 0

        while search_offset <= len(data) - CAM_FRAME_LEN:
            aa_pos = data.find(b'\xAA', search_offset)
            if aa_pos == -1:
                self._tail = self._head          # 无帧头，丢弃全部
                return None

            if len(data) < aa_pos + CAM_FRAME_LEN:
                self._tail = (self._tail + aa_pos) % self._size
                return None

            if data[aa_pos + 9] == CAM_FRAME_TAIL:
                frame = data[aa_pos:aa_pos + CAM_FRAME_LEN]
                self._tail = (self._tail + aa_pos + CAM_FRAME_LEN) % self._size
                return frame

            # AA 后第 9 字节不是 BB → 假帧头，跳过继续找下一个
            search_offset = aa_pos + 1

        self._tail = self._head
        return None

    def clear(self):
        self._head = 0
        self._tail = 0


# ═══════════════════════════════════════════════════════════════
#  摄像头协议解析
# ═══════════════════════════════════════════════════════════════

def parse_camera_frame(buf):
    """解析 AA..BB 10 字节帧（LED.py 双灯追踪协议）。
    返回 (ex, ey, dist, roll) 或 None。

    协议: AA [EX_H EX_L] [EY_H EY_L] [DIST_H DIST_L] [ROLL_H ROLL_L] BB
          int16 大端序，÷10 恢复实际值
          ex   = 双灯中心横向像素偏移
          ey   = 双灯中心纵向像素偏移
          dist = 双灯间像素距离（表征远近）
          roll = 面积比推算倾角（°）
    """
    if len(buf) < 10 or buf[0] != 0xAA or buf[9] != 0xBB:
        return None

    raw_ex   = (buf[1] << 8) | buf[2]
    raw_ey   = (buf[3] << 8) | buf[4]
    raw_dist = (buf[5] << 8) | buf[6]
    raw_roll = (buf[7] << 8) | buf[8]

    ex   = (raw_ex   if raw_ex   < 32768 else raw_ex   - 65536) / 10.0
    ey   = (raw_ey   if raw_ey   < 32768 else raw_ey   - 65536) / 10.0
    dist = (raw_dist if raw_dist < 32768 else raw_dist - 65536) / 10.0
    roll = (raw_roll if raw_roll < 32768 else raw_roll - 65536) / 10.0

    return (ex, ey, dist, roll)


def is_valid_target(ex, ey, dist, roll):
    """有效目标：数据不全为零（LED.py 丢失目标时发送全零帧）。"""
    return not (ex == 0.0 and ey == 0.0 and dist == 0.0 and roll == 0.0)


# ═══════════════════════════════════════════════════════════════
#  模式管理（闭包保持局部变量，避免全局污染）
# ═══════════════════════════════════════════════════════════════

def _create_mode_manager():
    """返回 (enter_uwb, exit_uwb, enter_visual, exit_visual, enter_stopped)
    以及共享资源引用字典。"""

    # ── 共享资源 ──
    res = {
        'uwb':       None,   # UWBFollower 实例
        'cam_uart':  None,   # UART(7) 实例
        'cam_kf':    None,   # CameraKalmanFilter
        'cam_ctrl':  None,   # CascadeController
        'cam_ring':  CamRingBuffer(),
        'window':    [],     # 滑动窗口 [1/0, ...]
        'last_data': 0,      # 摄像头最后收包时刻 (ms)
        'tracking':  False,  # 是否已确认跟踪目标
        'timeout_done': False,
    }

    # ── UWB 模式 ──────────────────────────────────────────

    def enter_uwb():
        print("\n>>> MODE: UWB_FOLLOW <<<")
        from uwb_tracker import UWBFollower
        res['uwb'] = UWBFollower(uart_id=0, baudrate=115200, target_anchor="8834")

        # 摄像头 UART7（后台轮询，惰性创建）
        if res['cam_uart'] is None:
            res['cam_uart'] = UART(CAM_UART_ID, baudrate=CAM_BAUDRATE,
                                   bits=8, parity=None, stop=1)
        res['cam_ring'].clear()
        res['window'] = []
        led.value(1)

    def exit_uwb():
        if res['uwb']:
            res['uwb'].stop()       # stop_all + enc_ticker.restart + deinit UART0
        res['uwb'] = None
        # 切换到视觉追踪：接管编码器（与 uwb_tracker 一致的手动管理）
        enc_ticker.stop()
        for _ in range(5):
            _ = get_encoder_counts()
            time.sleep_ms(10)
        reset_encoder_filter()
        reset_wheel_pi()
        reset_ang_vel_pid()

    # ── 视觉追踪模式 ──────────────────────────────────────

    def enter_visual():
        print("\n>>> MODE: VISUAL_TRACK <<<")
        from kalman_filter import CameraKalmanFilter
        from control import CascadeController

        # IMU 预热（确保 update_angle 已建立基准，供 control.py 的 imu.get() 使用）
        for _ in range(5):
            d = imu_motion.imu.read()
            update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
            time.sleep_ms(10)

        res['cam_kf']   = CameraKalmanFilter()
        res['cam_ctrl'] = CascadeController()
        res['cam_ring'].clear()
        res['window']       = []
        res['last_data']    = time.ticks_ms()
        res['tracking']     = False
        res['timeout_done'] = False

        led.value(0)

    def exit_visual():
        if res['cam_ctrl']:
            res['cam_ctrl'].emergency_stop()
        res['cam_ctrl'] = None
        res['cam_kf']   = None

    # ── 停车模式 ──────────────────────────────────────────

    def enter_stopped():
        print("\n>>> MODE: STOPPED (target reached) <<<")
        stop_all()
        led.value(1)

    def exit_stopped():
        """离开停车模式（当前无需清理，仅保持架构对称）。"""
        pass

    return enter_uwb, exit_uwb, enter_visual, exit_visual, enter_stopped, exit_stopped, res


# ═══════════════════════════════════════════════════════════════
#  主函数
# ═══════════════════════════════════════════════════════════════

def main():
    (enter_uwb, exit_uwb,
     enter_visual, exit_visual,
     enter_stopped, exit_stopped,
     res) = _create_mode_manager()

    state    = STATE_UWB
    loop_cnt = 0
    print_cnt = 0

    # ── 启动：进入 UWB 跟随 ──
    enter_uwb()

    print("=" * 50)
    print("RT1021 — UWB Follow + LED Visual Track")
    print("  UART0 : UWB 基站数据 (115200)")
    print("  UART7 : LED.py 双灯追踪 (115200)")
    print("  KEY1/2/3 : UWB / VISUAL / STOP")
    print("  SW2   : 强制退出")
    print("=" * 50)

    try:
        while True:
            now = time.ticks_ms()

            # ─── 看门狗喂狗 + 按键扫描 ───
            capture()
            pet_watchdog()

            # ─── 按键模式切换（集中处理，任意状态可触发）───
            if key_triggered(1):           # KEY1 (C8) → UWB
                if state != STATE_UWB:
                    print("\n[KEY1] → UWB_FOLLOW")
                    if state == STATE_VISUAL:
                        exit_visual()
                    enter_uwb()
                    state = STATE_UWB
                    loop_cnt = 0
                    continue
            if key_triggered(2):           # KEY2 (C9) → VISUAL
                if state != STATE_VISUAL:
                    print("\n[KEY2] → VISUAL_TRACK")
                    if state == STATE_UWB:
                        exit_uwb()
                    enter_visual()
                    state = STATE_VISUAL
                    loop_cnt = 0
                    continue
            if key_triggered(3):           # KEY3 (C14) → STOPPED
                if state != STATE_STOPPED:
                    print("\n[KEY3] → STOPPED")
                    if state == STATE_UWB:
                        exit_uwb()
                    elif state == STATE_VISUAL:
                        exit_visual()
                    enter_stopped()
                    state = STATE_STOPPED
                    loop_cnt = 0
                    continue

            # ─── SW2 检测（每次迭代）───
            if check_sw2():
                print("\n[SW2] Exit requested")
                if state == STATE_UWB:
                    exit_uwb()
                elif state == STATE_VISUAL:
                    exit_visual()
                break

            # ════════════════════════════════════════════════
            #  状态 ① — UWB 跟随
            # ════════════════════════════════════════════════
            if state == STATE_UWB:
                if res['uwb']:
                    res['uwb'].step()

                # 后台轮询摄像头（每 5 次迭代 ≈ 50ms）
                if loop_cnt % 5 == 0 and res['cam_uart']:
                    # 消费 UART 缓冲区（防积压）
                    for _ in range(4):
                        if res['cam_uart'].any() == 0:
                            break
                        raw = res['cam_uart'].read()
                        if raw:
                            res['cam_ring'].feed(raw)

                    # 提取并判定帧
                    frame = res['cam_ring'].get_frame()
                    if frame:
                        parsed = parse_camera_frame(frame)
                        if parsed:
                            ex, ey, dist, roll = parsed
                            hit = 1 if is_valid_target(ex, ey, dist, roll) else 0
                            res['window'].append(hit)
                            if len(res['window']) > WINDOW_SIZE:
                                res['window'].pop(0)

                            hits = sum(res['window'])
                            if (hits >= WINDOW_THRESHOLD
                                    and len(res['window']) >= WINDOW_SIZE):
                                print("[CAM] Target confirmed! {}/{} → VISUAL_TRACK".format(
                                    hits, len(res['window'])))
                                exit_uwb()
                                enter_visual()
                                state = STATE_VISUAL
                                loop_cnt = 0      # 重置计数器
                                continue

                # UWB 超时（UWBFollower 内部已处理停车，此处仅静默）
                if res['uwb'] and res['uwb'].is_timeout():
                    pass

            # ════════════════════════════════════════════════
            #  状态 ② — 视觉追踪
            # ════════════════════════════════════════════════
            elif state == STATE_VISUAL:
                if res['cam_uart'] is None:
                    exit_visual()
                    enter_uwb()
                    state = STATE_UWB
                    loop_cnt = 0
                    continue

                # 读取摄像头 UART7
                frame = None
                if res['cam_uart'].any() > 0:
                    raw = res['cam_uart'].read()
                    if raw:
                        res['cam_ring'].feed(raw)
                        frame = res['cam_ring'].get_frame()

                if frame:
                    parsed = parse_camera_frame(frame)
                    if parsed:
                        ex, ey, dist, roll = parsed
                        res['last_data']    = now
                        res['timeout_done'] = False

                        is_valid = is_valid_target(ex, ey, dist, roll)
                        res['window'].append(1 if is_valid else 0)
                        if len(res['window']) > WINDOW_SIZE:
                            res['window'].pop(0)

                        valid_count = sum(res['window'])
                        res['tracking'] = (valid_count >= WINDOW_THRESHOLD
                                           and len(res['window']) >= WINDOW_SIZE)

                        if res['tracking']:
                            # 卡尔曼滤波 → 级联 PID → 驱动电机
                            ex_f, dist_f, roll_f = res['cam_kf'].update(ex, dist, roll)
                            vx_out, vy_out, wz_out, speeds, dt = res['cam_ctrl'].step(
                                ex_f, dist_f, roll_f, True)

                            # 遥测打印
                            print_cnt += 1
                            if print_cnt >= 10:
                                print_cnt = 0
                                print("[VISUAL] ex={:.1f} dist={:.1f}px roll={:.1f}° | vx={:.3f} vy={:.3f} wz={:.3f} dt={:.0f}ms".format(
                                    ex_f, dist_f, roll_f, vx_out, vy_out, wz_out, dt * 1000))

                            # 到达目标 → 停车（LED.py 协议：dist 像素距离越大 = 越近）
                            # TODO: 需实车标定停车阈值，当前用 TARGET_DIST_PX 占位
                            if not math.isnan(dist_f) and dist_f >= TARGET_DIST_PX:
                                print("[VISUAL] Target reached! dist={:.1f}px → STOPPED".format(dist_f))
                                exit_visual()
                                enter_stopped()
                                state = STATE_STOPPED
                                loop_cnt = 0
                                continue
                        else:
                            # 无跟踪，输出零速度
                            if res['cam_ctrl']:
                                res['cam_ctrl'].step(0.0, 0.0, 0.0, False)
                else:
                    # 无帧，零速度
                    if res['cam_ctrl']:
                        res['cam_ctrl'].step(0.0, 0.0, 0.0, False)

                # 超时 500ms 无数据 → 退回 UWB（仅在已建立跟踪后生效，按键手动进入时不会误触发）
                if not res['timeout_done'] and res['tracking']:
                    if time.ticks_diff(now, res['last_data']) > CAM_TIMEOUT_MS:
                        res['timeout_done'] = True
                        stop_all()
                        if res['cam_ctrl']:
                            res['cam_ctrl'].emergency_stop()
                        if res['tracking']:
                            res['tracking'] = False
                            res['window'] = []
                        print("[VISUAL] Timeout {}ms → UWB_FOLLOW".format(CAM_TIMEOUT_MS))
                        exit_visual()
                        enter_uwb()
                        state = STATE_UWB
                        loop_cnt = 0
                        continue

                # LED 闪烁
                if loop_cnt % 20 == 0:
                    led.toggle()

            # ════════════════════════════════════════════════
            #  状态 ③ — 停车
            # ════════════════════════════════════════════════
            elif state == STATE_STOPPED:
                # LED 慢闪（~1Hz，区别于 UWB 常亮 / VISUAL 快闪）
                if loop_cnt % 50 == 0:
                    led.toggle()

            loop_cnt += 1
            time.sleep_ms(10)

            # 定期 GC
            if loop_cnt % 200 == 0:
                gc.collect()

    except Exception as e:
        print("[FATAL] Exception in main loop:")
        try:
            import sys
            sys.print_exception(e)
        except Exception:
            print("  ", e)

    finally:
        # ── 清理（等效于原 main.py 的 finally 逻辑）──
        stop_all()

        if res['uwb']:
            try:
                res['uwb'].stop()
            except Exception:
                pass

        if res['cam_uart']:
            try:
                res['cam_uart'].deinit()
            except Exception:
                pass

        # 停止编码器 ticker（PIT1），防止 ISR 干扰 REPL
        pause_encoder_ticker()

        # NOTE：key.py 看门狗(PIT2) 由 machine.reset() 回收，无需手动停止
        # imu_motion.py 的 PIT3 ticker 由 REPL 复位自动回收

        time.sleep_ms(50)
        print("\nRobot stopped. (you may now re-run or enter REPL)")


# ═══════════════════════════════════════════════════════════════
#  兼容原 motor.py 的 pause_encoder_ticker（main.py finally 调用）
# ═══════════════════════════════════════════════════════════════

def pause_encoder_ticker():
    """安全停止编码器 ticker（与 motor.py 的同名函数一致）。"""
    try:
        enc_ticker.stop()
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════

main()

