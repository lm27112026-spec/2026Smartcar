"""
main.py — UWB跟随 + 视觉追踪集成（调用 cam_follow.py 级联 PID）
【流程】
  1. 默认启动 UWB 跟随（uwb_tracker.UWBFollower）— 闭环航向纠偏
  2. 后台轮询摄像头 UART7（CamDataReceiver）→ 滑动窗口检测到目标 → 切换视觉追踪
  3. 视觉追踪通过 cam_follow.compute_control() 实现（级联 PID）
  4. SW2 (D9) 拨码开关全程可强制退出
【摄像头协议】cam_data.py（UART7, 115200）
  AA [X_H X_L] [Y_H Y_L] [FLAG] [ID] [B6] [B7] BB
  int16 大端序 ×10，FLAG=0x02/0x03=检测到，0x00=丢失
【按键控制】
  KEY1 (C8) → UWB 模式         LED 常亮
  KEY2 (C9) → 视觉追踪模式       LED 快闪 (~2.5Hz)
  KEY3 (C14)→ 停车状态          LED 慢闪 (~1Hz)
【依赖】uwb_tracker.py, cam_follow.py, cam_data.py, motor.py, imu_motion.py, key.py
【PIT 分配】PIT0=系统, PIT1=编码器(motor), PIT2=看门狗(key), PIT3=IMU
【看门狗】machine.WDT() 硬件看门狗（key.py），3 秒超时硬复位
"""
import gc, time, math
from machine import Pin
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
                   enc_ticker, ENC_SCALE)
import imu_motion
from imu_motion import update_angle, get_angular_velocity, reset_ang_vel_pid, imu_get_safe
from key import capture, key_triggered, pet_watchdog
from cam_data import CamDataReceiver, x_to_cm, y_to_distance

# ── 视觉追踪：全部委托给 cam_follow.py ──
from cam_follow import (
    compute_control, reset_control,
    STATE_FOLLOW, STATE_STOPPED as CAM_STOP, STATE_LOST,
    TARGET_DIST_CM, STOP_DIST_CM, DT as CAM_DT,
)

# ═══════════════════════════════════════════════════════════════
#  常量
# ═══════════════════════════════════════════════════════════════

LED_PIN  = 'C4'
SW2_PIN  = 'D9'

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
#  模式管理
# ═══════════════════════════════════════════════════════════════

def _create_mode_manager():
    res = {
        'uwb':           None,
        'cam_recv':      None,   # CamDataReceiver (UART7)
        'window':        [],
        'last_data':     0,
        'tracking':      False,
        'timeout_done':  False,
        'first_frames':  0,
    }

    # ── UWB 模式 ──────────────────────────────────────────

    def enter_uwb():
        print("\n>>> MODE: UWB_FOLLOW <<<")
        from uwb_tracker import UWBFollower
        res['uwb'] = UWBFollower(uart_id=0, baudrate=115200, target_anchor="8834")

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

        # 重置 cam_follow 控制状态（含状态机复位到 LOST）
        reset_control(reset_state=True)

        res['window']       = []
        res['last_data']    = time.ticks_ms()
        res['tracking']     = False
        res['timeout_done'] = False
        res['first_frames'] = 3   # 前N帧跳过电机驱动，给编码器缓冲

        if res['cam_recv']:
            res['cam_recv'].flush()
        led.value(0)

    def exit_visual():
        # cam_follow 状态在下次 enter_visual 时由 reset_control() 重置
        pass

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
    print("RT1021 — UWB Follow + Visual Track (cam_follow API)")
    print("  UART0 : UWB 基站数据 (115200)")
    print("  UART7 : cam_data.py 摄像头协议 (115200)")
    print("  Target: {:.0f}cm".format(TARGET_DIST_CM))
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

            # ─── SW2 检测 ───
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
                    for _ in range(4):
                        data = res['cam_recv'].read()
                        if data is None:
                            break
                        hit = 1 if data['is_target'] else 0
                        res['window'].append(hit)
                        if len(res['window']) > WINDOW_SIZE:
                            res['window'].pop(0)

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
            #  状态 ② — 视觉追踪（调用 cam_follow API）
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

                # ── 更新滑动窗口 ──
                res['window'].append(1 if data['is_target'] else 0)
                if len(res['window']) > WINDOW_SIZE:
                    res['window'].pop(0)
                valid_count = sum(res['window'])
                res['tracking'] = (valid_count >= WINDOW_THRESHOLD
                                   and len(res['window']) >= WINDOW_SIZE)

                # ── 调用 cam_follow 核心控制 ──
                x_cm = x_to_cm(data['x'])
                actual_dist = y_to_distance(data['y'])
                ctrl = compute_control(x_cm, actual_dist, data['is_target'], now)

                # ── 状态变化消息 ──
                if ctrl['state_msg']:
                    print(ctrl['state_msg'])

                # ── 到达目标 → 停车 ──
                if ctrl['arrived']:
                    exit_visual()
                    enter_stopped()
                    state = STATE_STOPPED
                    loop_cnt = 0
                    continue

                # ── 驱动电机（前N帧跳过 + 编码器合理性检查）──
                if ctrl['cmd_fwd'] is not None and res['first_frames'] <= 0:
                    try:
                        rc = get_encoder_counts()
                        if any(c != 0 for c in rc):
                            rs = [rc[i] / ENC_SCALE[i] / CAM_DT for i in range(4)]
                            omni_drive_closed_loop(
                                ctrl['cmd_fwd'], ctrl['cmd_lat'], 0, rs, CAM_DT)
                    except Exception as e:
                        print("[MOTOR] drive error:", e)
                elif res['first_frames'] > 0:
                    res['first_frames'] -= 1

                # ── 更新最后有效数据时间 ──
                if data['is_target']:
                    res['last_data']    = now
                    res['timeout_done'] = False

                # ── 遥测打印 ──
                print_cnt += 1
                if print_cnt >= 15:
                    print_cnt = 0
                    state_str = {0: "FOLLOW", 1: "STOP", 2: "LOST"}[ctrl['state']]
                    print("[#{:04d} {:s}] X:{:+5.1f}cm dist:{:5.1f}cm "
                          "fwd:{:+.3f} lat:{:+.3f}".format(
                        res['cam_recv'].frame_count, state_str,
                        x_cm, actual_dist,
                        ctrl['cmd_fwd_raw'], ctrl['cmd_lat_raw']))

                # ── 超时 500ms → 退回 UWB ──
                if not res['timeout_done']:
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


def pause_encoder_ticker():
    try:
        enc_ticker.stop()
    except Exception:
        pass


main()
