"""
main.py — UWB跟随 + 视觉追踪集成（级联 PID 版）
【流程】
  1. 默认启动 UWB 跟随（uwb_tracker.UWBFollower）— 闭环航向纠偏
  2. 后台轮询摄像头 UART7（CamDataReceiver）→ 滑动窗口检测到目标 → 切换视觉追踪
  3. 视觉追踪（CascadePID 级联控制）逼近至目标距离 → 自动停车
  4. SW2 (D9) 拨码开关全程可强制退出
【摄像头协议】cam_data.py（UART7, 115200）
  AA [X_H X_L] [Y_H Y_L] [FLAG] [ID] [B6] [B7] BB
  int16 大端序 ×10，FLAG=0x02/0x03=检测到，0x00=丢失
【按键控制】
  KEY1 (C8) → UWB 模式         LED 常亮
  KEY2 (C9) → 视觉追踪模式       LED 快闪 (~2.5Hz)
  KEY3 (C14)→ 停车状态          LED 慢闪 (~1Hz)
【依赖】uwb_tracker.py, cam_data.py, motor.py, imu_motion.py, pid.py, key.py
【PIT 分配】PIT0=系统, PIT1=编码器(motor), PIT2=看门狗(key), PIT3=IMU
【看门狗】machine.WDT() 硬件看门狗（key.py），3 秒超时硬复位
"""
import gc, time, math
from machine import UART, Pin
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
                   enc_ticker, ENC_SCALE)
import imu_motion
from imu_motion import update_angle, get_angular_velocity, reset_ang_vel_pid, imu_get_safe
from key import capture, key_triggered, pet_watchdog
from pid import PID
from cam_data import CamDataReceiver, x_to_cm, y_to_distance

# ═══════════════════════════════════════════════════════════════
#  常量
# ═══════════════════════════════════════════════════════════════

LED_PIN  = 'C4'
SW2_PIN  = 'D9'

# ── 视觉追踪参数（对齐 cam_follow.py）──
TARGET_DIST_CM   = 30.0    # 目标跟随距离 (cm)
STOP_DIST_CM     = 5.0     # 到达判定容差 (cm)
LOST_TIMEOUT_MS  = 500     # 目标丢失超时 (ms)
MAX_SPEED_FWD    = 0.50    # 最大前进速度 (m/s)
MAX_SPEED_LAT    = 0.45    # 最大横向速度 (m/s)
ACCEL_LIMIT      = 3.0     # 加速度限制 (m/s²)
MIN_SPEED        = 0.10    # 最低速度（防电机死区）
DT               = 0.02    # 控制周期 (s)

# ── 速度滤波参数 ──
SPEED_FILTER_ALPHA = 0.8

# ── 摄像头判定 ──
WINDOW_SIZE       = 8     # 滑动窗口大小（帧数）
WINDOW_THRESHOLD  = 5     # 窗口内有效帧数阈值
CAM_TIMEOUT_MS    = 500   # 视觉追踪数据超时 → 退回 UWB

# ── 模式 ──
STATE_UWB     = 0
STATE_VISUAL  = 1
STATE_STOPPED = 2

# ═══════════════════════════════════════════════════════════════
#  硬件初始化
# ═══════════════════════════════════════════════════════════════

led = Pin(LED_PIN, Pin.OUT, value=False)
sw2 = Pin(SW2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)

# ── SW2 消抖状态（时间窗口 50ms）──
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
#  CascadePID 类（级联 PID：位置环 + 加速度限幅）
#  来源：cam_follow.py
# ═══════════════════════════════════════════════════════════════

class CascadePID:
    """级联 PID: 外环位置 PID → 加速度限幅 → 输出速度指令"""

    def __init__(self, kp, ki, kd, out_limit, accel_limit):
        self.pid = PID(kp=kp, ki=ki, kd=kd,
                       integral_limit=50, output_limit=out_limit)
        self.accel_limit = accel_limit
        self.prev_output = 0.0

    def compute(self, error, dt):
        """
        计算输出，带加速度限幅
        error: 位置误差 (cm)
        返回: 速度指令 (m/s)
        """
        # 位置 PID → 目标速度
        # compute(setpoint, measurement): error = setpoint - measurement
        # 当 error(位置误差)>0(太远) → PID输出>0 → 前进
        target_speed = self.pid.compute(error, 0, dt)

        # 加速度限幅（平滑速度变化）
        delta = target_speed - self.prev_output
        max_delta = self.accel_limit * dt
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        output = self.prev_output + delta
        self.prev_output = output

        return output

    def reset(self):
        self.pid.reset()
        self.prev_output = 0.0


# ═══════════════════════════════════════════════════════════════
#  速度低通滤波（方向变化时自动复位）
#  来源：cam_follow.py
# ═══════════════════════════════════════════════════════════════

_filt_fwd = 0.0
_filt_lat = 0.0
_filt_fwd_sign = 0
_filt_lat_sign = 0


def speed_filter(fwd, lat):
    """一阶低通滤波，方向变化时重置滤波器"""
    global _filt_fwd, _filt_lat, _filt_fwd_sign, _filt_lat_sign

    # 检测方向变化，变化时重置滤波器避免锁死
    new_sign = 1 if fwd > 0 else (-1 if fwd < 0 else 0)
    if new_sign != 0 and new_sign != _filt_fwd_sign:
        _filt_fwd = fwd
        _filt_fwd_sign = new_sign
    else:
        _filt_fwd = SPEED_FILTER_ALPHA * fwd + (1 - SPEED_FILTER_ALPHA) * _filt_fwd

    new_sign_lat = 1 if lat > 0 else (-1 if lat < 0 else 0)
    if new_sign_lat != 0 and new_sign_lat != _filt_lat_sign:
        _filt_lat = lat
        _filt_lat_sign = new_sign_lat
    else:
        _filt_lat = SPEED_FILTER_ALPHA * lat + (1 - SPEED_FILTER_ALPHA) * _filt_lat

    return _filt_fwd, _filt_lat


def reset_speed_filter():
    """重置速度滤波全局状态"""
    global _filt_fwd, _filt_lat, _filt_fwd_sign, _filt_lat_sign
    _filt_fwd = 0.0
    _filt_lat = 0.0
    _filt_fwd_sign = 0
    _filt_lat_sign = 0


# ═══════════════════════════════════════════════════════════════
#  模式管理
# ═══════════════════════════════════════════════════════════════

def _create_mode_manager():
    res = {
        'uwb':           None,
        'cam_recv':      None,   # CamDataReceiver (UART7)
        'cam_pid_fwd':   None,   # CascadePID — 前进/后退
        'cam_pid_lat':   None,   # CascadePID — 横向
        'cam_state':     None,   # 视觉追踪子状态 (FOLLOW/STOPPED/LOST)
        'last_target_ms': 0,
        'window':        [],
        'last_data':     0,
        'tracking':      False,
        'timeout_done':  False,
    }

    # 视觉追踪子状态
    STATE_FOLLOW  = 0
    STATE_STOP    = 1
    STATE_LOST    = 2

    # ── UWB 模式 ──────────────────────────────────────────

    def enter_uwb():
        print("\n>>> MODE: UWB_FOLLOW <<<")
        from uwb_tracker import UWBFollower
        res['uwb'] = UWBFollower(uart_id=0, baudrate=115200, target_anchor="8834")

        # 惰性创建 CamDataReceiver（UART7 摄像头数据通道）
        if res['cam_recv'] is None:
            res['cam_recv'] = CamDataReceiver(uart_id=7)

        res['cam_recv'].flush()
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

        # IMU 预热（从 PIT3 ticker 缓冲区读取，无 SPI 直读）
        for _ in range(5):
            d = imu_get_safe()
            if d is not None:
                update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
            time.sleep_ms(10)

        # 创建级联 PID 实例
        res['cam_pid_fwd'] = CascadePID(
            kp=0.012, ki=0.003, kd=0.005,
            out_limit=MAX_SPEED_FWD,
            accel_limit=ACCEL_LIMIT
        )
        res['cam_pid_lat'] = CascadePID(
            kp=0.010, ki=0.002, kd=0.006,
            out_limit=MAX_SPEED_LAT,
            accel_limit=ACCEL_LIMIT
        )
        reset_speed_filter()

        res['cam_state']      = STATE_LOST
        res['last_target_ms'] = time.ticks_ms()
        res['window']         = []
        res['last_data']      = time.ticks_ms()
        res['tracking']       = False
        res['timeout_done']   = False

        if res['cam_recv']:
            res['cam_recv'].flush()
        led.value(0)

    def exit_visual():
        if res['cam_pid_fwd']:
            res['cam_pid_fwd'].reset()
        if res['cam_pid_lat']:
            res['cam_pid_lat'].reset()
        res['cam_pid_fwd'] = None
        res['cam_pid_lat'] = None
        res['cam_state']   = None

    # ── 停车模式 ──────────────────────────────────────────

    def enter_stopped():
        print("\n>>> MODE: STOPPED (target reached) <<<")
        stop_all()
        led.value(1)

    def exit_stopped():
        pass

    return enter_uwb, exit_uwb, enter_visual, exit_visual, enter_stopped, exit_stopped, res, STATE_FOLLOW, STATE_STOP, STATE_LOST


# ═══════════════════════════════════════════════════════════════
#  主函数
# ═══════════════════════════════════════════════════════════════

def main():
    (enter_uwb, exit_uwb,
     enter_visual, exit_visual,
     enter_stopped, exit_stopped,
     res, STATE_FOLLOW, STATE_STOP, STATE_LOST) = _create_mode_manager()

    state    = STATE_UWB
    loop_cnt = 0
    print_cnt = 0

    enter_uwb()

    print("=" * 50)
    print("RT1021 — UWB Follow + Visual Track (Cascade PID)")
    print("  UART0 : UWB 基站数据 (115200)")
    print("  UART7 : cam_data.py 摄像头协议 (115200)")
    print("  Target: {:.0f}cm".format(TARGET_DIST_CM))
    print("  KEY1/2/3 : UWB / VISUAL / STOP")
    print("  SW2   : 强制退出")
    print("=" * 50)

    try:
        while True:
            now = time.ticks_ms()

            # ════════════════════════════════════════════════
            #  看门狗喂狗 + 按键扫描
            # ════════════════════════════════════════════════
            capture()
            pet_watchdog()

            # ════════════════════════════════════════════════
            #  按键模式切换
            # ════════════════════════════════════════════════
            if key_triggered(1):           # KEY1 (C8) → UWB
                if state != STATE_UWB:
                    print("\n[KEY1] → UWB_FOLLOW")
                    if state == STATE_VISUAL:
                        exit_visual()
                    elif state == STATE_STOPPED:
                        exit_stopped()
                    enter_uwb()
                    state = STATE_UWB
                    loop_cnt = 0
                    continue
            if key_triggered(2):           # KEY2 (C9) → VISUAL
                if state != STATE_VISUAL:
                    print("\n[KEY2] → VISUAL_TRACK")
                    if state == STATE_UWB:
                        exit_uwb()
                    elif state == STATE_STOPPED:
                        exit_stopped()
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

            # ════════════════════════════════════════════════
            #  SW2 检测
            # ════════════════════════════════════════════════
            if check_sw2():
                print("\n[SW2] Exit requested")
                if state == STATE_UWB:
                    exit_uwb()
                elif state == STATE_VISUAL:
                    exit_visual()
                break

            # ════════════════════════════════════════════════
            #  状态 ① — UWB 跟随（默认）
            # ════════════════════════════════════════════════
            if state == STATE_UWB:
                if res['uwb']:
                    res['uwb'].step()

                # 后台轮询摄像头（每 ~50ms）
                if loop_cnt % 5 == 0 and res['cam_recv']:
                    # 批量读取 UART7 帧
                    for _ in range(4):
                        data = res['cam_recv'].read()
                        if data is None:
                            break
                        hit = 1 if data['is_target'] else 0
                        res['window'].append(hit)
                        if len(res['window']) > WINDOW_SIZE:
                            res['window'].pop(0)

                    # 滑动窗口判定
                    if len(res['window']) >= WINDOW_SIZE:
                        hits = sum(res['window'])
                        if hits >= WINDOW_THRESHOLD:
                            print("[CAM] Target confirmed! {}/{} → VISUAL_TRACK".format(
                                hits, len(res['window'])))
                            exit_uwb()
                            enter_visual()
                            state = STATE_VISUAL
                            loop_cnt = 0
                            continue

            # ════════════════════════════════════════════════
            #  状态 ② — 视觉追踪（级联 PID）
            # ════════════════════════════════════════════════
            elif state == STATE_VISUAL:
                if res['cam_recv'] is None:
                    exit_visual()
                    enter_uwb()
                    state = STATE_UWB
                    loop_cnt = 0
                    continue

                # 读取摄像头数据
                data = res['cam_recv'].read()
                if data is None:
                    time.sleep_ms(1)
                    loop_cnt += 1
                    continue

                now = time.ticks_ms()

                # ── 子状态判定 ──
                if data['is_target']:
                    res['last_target_ms'] = now
                    if res['cam_state'] == STATE_LOST:
                        res['cam_state'] = STATE_FOLLOW
                        reset_wheel_pi()
                        if res['cam_pid_fwd']:
                            res['cam_pid_fwd'].reset()
                        if res['cam_pid_lat']:
                            res['cam_pid_lat'].reset()
                        reset_speed_filter()
                        print("[LOST -> FOLLOW] Target captured!")
                else:
                    if res['cam_state'] == STATE_FOLLOW:
                        if time.ticks_diff(now, res['last_target_ms']) > LOST_TIMEOUT_MS:
                            res['cam_state'] = STATE_LOST
                            stop_all()
                            print("[FOLLOW -> LOST] Target lost!")
                            loop_cnt += 1
                            continue

                # ── 更新滑动窗口（用于超时回退判定）──
                res['window'].append(1 if data['is_target'] else 0)
                if len(res['window']) > WINDOW_SIZE:
                    res['window'].pop(0)

                valid_count = sum(res['window'])
                res['tracking'] = (valid_count >= WINDOW_THRESHOLD
                                   and len(res['window']) >= WINDOW_SIZE)

                # ── 级联 PID 控制 ──
                if res['cam_state'] == STATE_FOLLOW:
                    # 更新最后有效数据时间
                    if data['is_target']:
                        res['last_data']    = now
                        res['timeout_done'] = False

                    # 误差计算
                    x_cm = x_to_cm(data['x'])
                    actual_dist = y_to_distance(data['y'])
                    x_error = x_cm
                    y_error = actual_dist - TARGET_DIST_CM

                    # 外环：位置 PID → 目标速度
                    cmd_fwd = res['cam_pid_fwd'].compute(y_error, DT)
                    cmd_lat = res['cam_pid_lat'].compute(x_error, DT)

                    # 中环：速度滤波
                    cmd_fwd, cmd_lat = speed_filter(cmd_fwd, cmd_lat)

                    # 最低速度防死区（仅有运动意图时生效）
                    if abs(cmd_fwd) > 0.01 and abs(cmd_fwd) < MIN_SPEED:
                        cmd_fwd = MIN_SPEED if cmd_fwd > 0 else -MIN_SPEED
                    if abs(cmd_lat) > 0.01 and abs(cmd_lat) < MIN_SPEED:
                        cmd_lat = MIN_SPEED if cmd_lat > 0 else -MIN_SPEED

                    # 到达判定 → 停车（同时退出视觉模式）
                    if abs(y_error) < STOP_DIST_CM and abs(x_cm) < 10:
                        res['cam_state'] = STATE_STOP
                        stop_all()
                        print("[FOLLOW -> STOPPED] dist={:.1f}cm".format(actual_dist))
                        print("[VISUAL] Target reached! dist={:.1f}cm → STOPPED".format(actual_dist))
                        exit_visual()
                        enter_stopped()
                        state = STATE_STOPPED
                        loop_cnt = 0
                        continue
                    else:
                        # 内环：编码器闭环驱动
                        rc = get_encoder_counts()
                        rs = [rc[i] / ENC_SCALE[i] / DT for i in range(4)]
                        omni_drive_closed_loop(cmd_fwd, cmd_lat, 0, rs, DT)

                    # 遥测打印
                    print_cnt += 1
                    if print_cnt >= 15:
                        print_cnt = 0
                        state_str = {0: "FOLLOW", 1: "STOP", 2: "LOST"}[res['cam_state']]
                        print("[#{:04d} {:s}] X:{:+5.1f}cm dist:{:5.1f}cm "
                              "fwd:{:+.3f} lat:{:+.3f}".format(
                              res['cam_recv'].frame_count, state_str,
                              x_cm, actual_dist, cmd_fwd, cmd_lat))

                elif res['cam_state'] == STATE_STOP:
                    # 等待目标移开
                    actual_dist = y_to_distance(data['y'])
                    if abs(actual_dist - TARGET_DIST_CM) > STOP_DIST_CM * 2:
                        res['cam_state'] = STATE_FOLLOW
                        reset_wheel_pi()
                        if res['cam_pid_fwd']:
                            res['cam_pid_fwd'].reset()
                        if res['cam_pid_lat']:
                            res['cam_pid_lat'].reset()
                        reset_speed_filter()
                        print("[STOPPED -> FOLLOW] Resuming.")

                elif res['cam_state'] == STATE_LOST:
                    stop_all()

                else:
                    # 无跟踪 → 停车
                    if not data['is_target']:
                        stop_all()

                # ── 超时 500ms 无有效目标 → 退回 UWB ──
                if not res['timeout_done']:
                    if data['is_target']:
                        res['last_data'] = now
                    if time.ticks_diff(now, res['last_data']) > CAM_TIMEOUT_MS:
                        res['timeout_done'] = True
                        stop_all()
                        res['tracking'] = False
                        res['window'] = []
                        print("[VISUAL] Timeout {}ms → UWB_FOLLOW".format(CAM_TIMEOUT_MS))
                        exit_visual()
                        enter_uwb()
                        state = STATE_UWB
                        loop_cnt = 0
                        continue

                # ── LED 快闪 (~2.5Hz) ──
                if loop_cnt % 20 == 0:
                    led.toggle()

            # ════════════════════════════════════════════════
            #  状态 ③ — 停车
            # ════════════════════════════════════════════════
            elif state == STATE_STOPPED:
                # LED 慢闪 (~1Hz)
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

        if res['cam_recv']:
            try:
                res['cam_recv'].flush()
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
