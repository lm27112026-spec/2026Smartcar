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
【PIT 分配】PIT0=系统, PIT1=编码器(motor), PIT3=IMU
【看门狗】machine.WDT() 硬件看门狗（key.py），3 秒超时硬复位
          不受 SPI 驱动 __disable_irq() 导致的全局中断关闭影响
"""

import gc, time, math
from machine import UART, Pin
from motor import (stop_all, enc_ticker,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi)
import imu_motion
from imu_motion import update_angle, get_angular_velocity, reset_ang_vel_pid, imu_get_safe
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

TARGET_DIST_PX  = 100     # 视觉追踪停车像素距离 — LED.py 协议用
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

# ── SW2 消抖状态 ──
_sw2_last           = sw2.value()
_sw2_changed        = False
_sw2_stable_start   = 0
SW2_DEBOUNCE_MS     = 50

def check_sw2():
    """读取 SW2 并消抖。返回 True 表示确认发生了切换。"""
    global _sw2_last, _sw2_changed, _sw2_stable_start
    val = sw2.value()
    if val != _sw2_last:
        _sw2_last = val
        _sw2_changed = True
        _sw2_stable_start = time.ticks_ms()
    if _sw2_changed and time.ticks_diff(time.ticks_ms(), _sw2_stable_start) >= SW2_DEBOUNCE_MS:
        _sw2_changed = False
        return True
    return False


# ═══════════════════════════════════════════════════════════════
#  摄像头 UART7 环形缓冲区
# ═══════════════════════════════════════════════════════════════

class CamRingBuffer:
    def __init__(self, size=256):
        self._buf  = bytearray(size)
        self._size = size
        self._head = 0
        self._tail = 0

    def feed(self, data):
        for b in data:
            self._buf[self._head] = b
            self._head = (self._head + 1) % self._size
            if self._head == self._tail:
                self._tail = (self._tail + 1) % self._size

    def get_frame(self):
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
                self._tail = self._head
                return None

            if len(data) < aa_pos + CAM_FRAME_LEN:
                self._tail = (self._tail + aa_pos) % self._size
                return None

            if data[aa_pos + 9] == CAM_FRAME_TAIL:
                frame = data[aa_pos:aa_pos + CAM_FRAME_LEN]
                self._tail = (self._tail + aa_pos + CAM_FRAME_LEN) % self._size
                return frame

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
    return not (ex == 0.0 and ey == 0.0 and dist == 0.0 and roll == 0.0)


# ═══════════════════════════════════════════════════════════════
#  模式管理
# ═══════════════════════════════════════════════════════════════

def _create_mode_manager():
    res = {
        'uwb':       None,
        'cam_uart':  None,
        'cam_kf':    None,
        'cam_ctrl':  None,
        'cam_ring':  CamRingBuffer(),
        'window':    [],
        'last_data': 0,
        'tracking':  False,
        'timeout_done': False,
    }

    # ── UWB 模式 ──────────────────────────────────────────

    def enter_uwb():
        print("\n>>> MODE: UWB_FOLLOW <<<")
        from uwb_tracker import UWBFollower
        res['uwb'] = UWBFollower(uart_id=0, baudrate=115200, target_anchor="8834")

        if res['cam_uart'] is None:
            res['cam_uart'] = UART(CAM_UART_ID, baudrate=CAM_BAUDRATE,
                                   bits=8, parity=None, stop=1)
        res['cam_ring'].clear()
        res['window'] = []
        led.value(1)

    def exit_uwb():
        if res['uwb']:
            res['uwb'].stop()
        res['uwb'] = None
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

        # ── 编码器接管 ──
        # exit_uwb() 已停止 PIT1 + 排空 + reset，此处无需重复。
        # 视觉控制循环通过 control.py 的 get_encoder_counts() 手动管理。

        # ── IMU 预热（从 PIT3 ticker 缓冲区读取，无 SPI 直读）──
        for _ in range(5):
            d = imu_get_safe()
            if d is not None:
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

            # ─── 按键模式切换 ───
            if key_triggered(1):           # KEY1 → UWB
                if state != STATE_UWB:
                    print("\n[KEY1] → UWB_FOLLOW")
                    if state == STATE_VISUAL:
                        exit_visual()
                    enter_uwb()
                    state = STATE_UWB
                    loop_cnt = 0
                    continue
            if key_triggered(2):           # KEY2 → VISUAL
                if state != STATE_VISUAL:
                    print("\n[KEY2] → VISUAL_TRACK")
                    if state == STATE_UWB:
                        exit_uwb()
                    elif state == STATE_STOPPED:
                        exit_uwb()
                    elif state == STATE_VISUAL:
                        exit_visual()
                    enter_visual()
                    state = STATE_VISUAL
                    loop_cnt = 0
                    continue
            if key_triggered(3):           # KEY3 → STOPPED
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

            # ─── SW2 检测 ───
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

                # 后台轮询摄像头
                if loop_cnt % 5 == 0 and res['cam_uart']:
                    for _ in range(4):
                        if res['cam_uart'].any() == 0:
                            break
                        raw = res['cam_uart'].read()
                        if raw:
                            res['cam_ring'].feed(raw)

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
                                loop_cnt = 0
                                continue

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
                        is_valid = is_valid_target(ex, ey, dist, roll)
                        if is_valid:
                            res['last_data']    = now
                            res['timeout_done'] = False

                        res['window'].append(1 if is_valid else 0)
                        if len(res['window']) > WINDOW_SIZE:
                            res['window'].pop(0)

                        valid_count = sum(res['window'])
                        res['tracking'] = (valid_count >= WINDOW_THRESHOLD
                                           and len(res['window']) >= WINDOW_SIZE)

                        if res['tracking']:
                            # ── IMU 读取（从 PIT3 ticker 缓冲区）──
                            imu_d = imu_get_safe()
                            if imu_d is not None:
                                update_angle(imu_d[0], imu_d[1], imu_d[2],
                                             imu_d[3], imu_d[4], imu_d[5])
                                actual_wz_dps = get_angular_velocity()
                                if actual_wz_dps != actual_wz_dps:
                                    actual_wz_dps = 0.0
                            else:
                                actual_wz_dps = 0.0

                            # 卡尔曼滤波 → 级联 PID → 驱动电机
                            ex_f, dist_f, roll_f = res['cam_kf'].update(ex, dist, roll)
                            vx_out, vy_out, wz_out, speeds, dt = res['cam_ctrl'].step(
                                ex_f, dist_f, roll_f, True, actual_wz_dps)

                            # 遥测打印
                            print_cnt += 1
                            if print_cnt >= 20:
                                print_cnt = 0
                                print("[VISUAL] ex={:.1f} dist={:.1f}px roll={:.1f}° "
                                      "| vx={:.3f} vy={:.3f} wz={:.3f} dt={:.0f}ms".format(
                                      ex_f, dist_f, roll_f, vx_out, vy_out, wz_out, dt * 1000))

                            # 到达目标 → 停车
                            if dist_f == dist_f and dist_f >= TARGET_DIST_PX:
                                print("[VISUAL] Target reached! dist={:.1f}px → STOPPED".format(dist_f))
                                exit_visual()
                                enter_stopped()
                                state = STATE_STOPPED
                                loop_cnt = 0
                                continue
                        else:
                            # 无跟踪 / 无帧 → 直接停车，不经过 encoder
                            if res['cam_ctrl']:
                                res['cam_ctrl'].emergency_stop()
                else:
                    # 无帧 → 直接停车，不经过 encoder
                    if res['cam_ctrl']:
                        res['cam_ctrl'].emergency_stop()

                # 超时 500ms 无有效目标 → 退回 UWB
                if not res['timeout_done']:
                    if time.ticks_diff(now, res['last_data']) > CAM_TIMEOUT_MS:
                        res['timeout_done'] = True
                        stop_all()
                        if res['cam_ctrl']:
                            res['cam_ctrl'].emergency_stop()
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
                if loop_cnt % 50 == 0:
                    led.toggle()

            loop_cnt += 1
            time.sleep_ms(10)

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

        pause_encoder_ticker()

        time.sleep_ms(50)
        print("\nRobot stopped. (you may now re-run or enter REPL)")


# ═══════════════════════════════════════════════════════════════
#  兼容 motor.py 的 pause_encoder_ticker
# ═══════════════════════════════════════════════════════════════

def pause_encoder_ticker():
    try:
        enc_ticker.stop()
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════════

main()
