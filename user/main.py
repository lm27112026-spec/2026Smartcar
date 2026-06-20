"""
main.py — 物品搬运任务状态机
【流程】
  发车区启动 → ① UWB+视觉 找物品靠近10cm → ② 45°左前盲走找黄线
  → ③ 180°掉头+UWB回物品区 → ④ 扫描判定物品区是否清空
  → (有物品) 回到① / (无物品) → ⑤ 返回发车区 → ⑥ 停车结束
【摄像头协议】cam_data.py（UART7, 115200）
  AA [X_H X_L] [Y_H Y_L] [FLAG] [ID] [B6] [B7] BB
  int16 大端序 ×10，FLAG 区分场景
【依赖】uwb_tracker.py, cam_follow.py, cam_data.py, motor.py, imu_motion.py, key.py
【PIT 分配】PIT0=系统, PIT1=编码器(motor), PIT2=看门狗(key), PIT3=IMU
"""
import gc, time, math
from machine import Pin
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
                   enc_ticker, ENC_SCALE)
import imu_motion
from imu_motion import (update_angle, get_angular_velocity,
                        reset_ang_vel_pid, imu_get_safe, yaw)
from key import capture, key_triggered, pet_watchdog
from cam_data import CamDataReceiver, x_to_cm, y_to_distance

from cam_follow import (
    compute_control, reset_control,
    STATE_FOLLOW, STATE_STOPPED as CAM_STOP, STATE_LOST,
    TARGET_DIST_CM, STOP_DIST_CM, DT as CAM_DT,
    PID_FWD, PID_LAT,
)

# ═══════════════════════════════════════════════════════════════
#  常量
# ═══════════════════════════════════════════════════════════════

LED_PIN  = 'C4'
SW2_PIN  = 'D9'

# ── 摄像头判定 ──
WINDOW_SIZE       = 8
WINDOW_THRESHOLD  = 5
CAM_TIMEOUT_MS    = 500

# ── 任务状态机 ──
STATE_UWB           = 0   # UWB 跟随，后台轮询摄像头
STATE_VISUAL        = 1   # 视觉追踪，靠近物品到 10cm
STATE_LEFT_FWD      = 3   # 45°左前盲走，摄像头检测黄线
STATE_YELLOW_RETURN = 4   # 180°掉头 + UWB 辅助回物品区
STATE_SCAN_ITEMS    = 5   # 扫描物品区判定是否清空
STATE_RETURN_DEPART = 6   # 返回发车区（方案待定，占位）
STATE_DONE          = 7   # 任务完成，停车退出

# ── 摄像头 FLAG 占位值（待摄像头团队确认）──
# TODO: 确认黄线、发车区的实际 FLAG 值
FLAG_YELLOW    = 0xC0
FLAG_DEPARTURE = 0xC1

# ── 左前盲走参数 ──
LEFT_FWD_SPEED  = 0.0    # 合速度 (m/s)
LEFT_FWD_ANGLE  = 45.0    # 左前方角度 (deg)

# ── 物品区扫描参数 ──
SCAN_EMPTY_FRAMES = 15    # 连续 N 帧无目标 → 判定为空

# ═══════════════════════════════════════════════════════════════
#  硬件初始化
# ═══════════════════════════════════════════════════════════════

led = Pin(LED_PIN, Pin.OUT, value=False)
sw2 = Pin(SW2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)

_sw2_last           = sw2.value()
_sw2_changed        = False
_sw2_stable_start   = 0
SW2_DEBOUNCE_MS     = 50


def check_sw2():
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
#  过渡动作 — 辅助函数
# ═══════════════════════════════════════════════════════════════

def drive_left_forward(debug=False):
    """以 45° 左前固定速度 闭环驱动（编码器 PI 反馈）
    vy<0 = 左移（与视觉跟随 cmd_lat 同坐标系）
    运动学: vx=+0.212m/s vy=-0.212m/s → RF=+0.424 LB=-0.424 m/s"""
    vx = 0.212   # LEFT_FWD_SPEED * cos(45°) — 硬编码绕过 MP 浮点 bug
    vy = -0.212  # -LEFT_FWD_SPEED * sin(45°)
    try:
        rc = get_encoder_counts()
        rs = [rc[i] / ENC_SCALE[i] / CAM_DT for i in range(4)]
        omni_drive_closed_loop(vx, vy, 0, rs, CAM_DT)
        if debug:
            print("[DBG] LFTFWD vx={:.3f} vy={:.3f} enc={:d},{:d},{:d},{:d}".format(
                vx, vy, rc[0], rc[1], rc[2], rc[3]))
    except Exception as e:
        print("[LEFT_FWD] drive error:", e)


def do_180_turn():
    """
    原地 180° 掉头（IMU 陀螺仪闭环旋转）
    返回: True=完成, False=进行中
    """
    # 读取当前航向角
    d = imu_get_safe()
    if d is None:
        time.sleep_ms(5)
        return False
    update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

    current_yaw = yaw  # 当前航向角 (deg, 全局变量)
    target_yaw = current_yaw + 180.0
    if target_yaw > 180.0:
        target_yaw -= 360.0

    # 闭环旋转：P 控制 wz
    MAX_WZ = 1.5  # 最大旋转角速度 (rad/s)

    yaw_error = target_yaw - current_yaw
    if yaw_error > 180:
        yaw_error -= 360
    elif yaw_error < -180:
        yaw_error += 360

    if abs(yaw_error) < 5.0:
        stop_all()
        return True

    # P 控制 wz
    wz = 0.05 * yaw_error
    wz = max(-MAX_WZ, min(wz, MAX_WZ))
    if abs(wz) < 0.1:
        wz = 0.1 if wz > 0 else -0.1

    try:
        rc = get_encoder_counts()
        rs = [rc[i] / ENC_SCALE[i] / CAM_DT for i in range(4)]
        omni_drive_closed_loop(0, 0, wz, rs, CAM_DT)
    except Exception as e:
        print("[180_TURN] drive error:", e)

    return False


def enter_uwb_nav(target_anchor="8834"):
    """
    启动 UWB 导航（用于返回物品区等辅助导航场景）
    返回: UWBFollower 实例
    """
    from uwb_tracker import UWBFollower
    uwb = UWBFollower(uart_id=0, baudrate=115200, target_anchor=target_anchor)
    enc_ticker.start(10)           # UWB 需要编码器 ticker 运行
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)
    reset_encoder_filter()
    reset_wheel_pi()
    return uwb


def return_to_departure():
    """
    [占位] 从物品区返回发车区 — 方案待定
    当前只是停止电机，等待上层状态机通过摄像头 FLAG 判定到达
    """
    pass  # TODO: 实现发车区导航逻辑


# ═══════════════════════════════════════════════════════════════
#  模式管理
# ═══════════════════════════════════════════════════════════════

def _create_mode_manager():
    res = {
        'uwb':           None,
        'cam_recv':      None,
        'window':        [],
        'last_data':     0,
        'tracking':      False,
        'timeout_done':  False,
        'first_frames':  0,
        # ── 新增任务状态 ──
        'turn_done':     False,     # 180° 掉头完成标志
        'scan_empty_cnt': 0,        # 连续无目标帧计数
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
        stop_all()
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
        for _ in range(5):
            d = imu_get_safe()
            if d is not None:
                update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
            time.sleep_ms(10)
        reset_control(reset_state=True)
        res['window']       = []
        res['last_data']    = time.ticks_ms()
        res['tracking']     = False
        res['timeout_done'] = False
        res['first_frames'] = 3
        if res['cam_recv']:
            res['cam_recv'].flush()
        led.value(0)

    def exit_visual():
        stop_all()

    # ── 左前盲走（找黄线）────────────────────────────────

    def enter_left_fwd():
        print("\n>>> MODE: LEFT_FORWARD (finding yellow line) <<<")
        stop_all()
        enc_ticker.start(10)
        time.sleep_ms(50)              # 充裕时间等 ticker 首次捕获
        for _ in range(5):             # 5 次排空缓冲（与 cam_follow 一致）
            _ = get_encoder_counts()
            time.sleep_ms(10)
        reset_encoder_filter()
        reset_wheel_pi()
        res['cam_recv'].flush()
        res['window'] = []
        led.value(0)

    def exit_left_fwd():
        stop_all()
        enc_ticker.stop()     # 退出时停止 ticker，由后续模式自行管理

    # ── 黄线→掉头+UWB回物品区 ────────────────────────────

    def enter_yellow_return():
        print("\n>>> MODE: YELLOW_RETURN (180 turn + UWB nav) <<<")
        res['turn_done'] = False
        led.value(0)

    def exit_yellow_return():
        if res['uwb']:
            res['uwb'].stop()
        res['uwb'] = None
        stop_all()
        enc_ticker.stop()
        for _ in range(3):
            _ = get_encoder_counts()
            time.sleep_ms(10)
        reset_encoder_filter()
        reset_wheel_pi()

    # ── 扫描物品区 ────────────────────────────────────────

    def enter_scan_items():
        print("\n>>> MODE: SCAN_ITEMS <<<")
        stop_all()
        enc_ticker.stop()
        for _ in range(3):
            _ = get_encoder_counts()
            time.sleep_ms(10)
        reset_encoder_filter()
        res['scan_empty_cnt'] = 0
        res['window'] = []
        res['cam_recv'].flush()
        led.value(1)

    def exit_scan_items():
        stop_all()

    # ── 返回发车区（占位）─────────────────────────────────

    def enter_return_depart():
        print("\n>>> MODE: RETURN_DEPARTURE (placeholder) <<<")
        return_to_departure()
        res['cam_recv'].flush()
        led.value(0)

    def exit_return_depart():
        stop_all()

    return (enter_uwb, exit_uwb,
            enter_visual, exit_visual,
            enter_left_fwd, exit_left_fwd,
            enter_yellow_return, exit_yellow_return,
            enter_scan_items, exit_scan_items,
            enter_return_depart, exit_return_depart,
            res)


# ═══════════════════════════════════════════════════════════════
#  主函数
# ═══════════════════════════════════════════════════════════════

def main():
    (enter_uwb, exit_uwb,
     enter_visual, exit_visual,
     enter_left_fwd, exit_left_fwd,
     enter_yellow_return, exit_yellow_return,
     enter_scan_items, exit_scan_items,
     enter_return_depart, exit_return_depart,
     res) = _create_mode_manager()

    state    = STATE_UWB
    loop_cnt = 0
    print_cnt = 0

    enter_uwb()

    print("=" * 50)
    print("RT1021 — Item Delivery State Machine")
    print("  UART0 : UWB 基站 (115200)")
    print("  UART7 : Camera 视觉 (115200)")
    print("  Target dist: {:.0f}cm".format(TARGET_DIST_CM))
    print("  States: UWB→VISUAL→LFTFWD→YLWRET→SCAN→RETDEP→DONE")
    print("  SW2   : force exit")
    print("=" * 50)

    try:
        while True:
            now = time.ticks_ms()

            # ─── 看门狗 + 按键 ───
            capture()
            pet_watchdog()

            # ─── GC ───
            loop_cnt += 1
            if loop_cnt % 50 == 0:
                gc.collect()

            # ─── 按键模式切换 ───
            if key_triggered(1):           # KEY1 → UWB
                if state != STATE_UWB:
                    print("\n[KEY1] → UWB_FOLLOW")
                    _exit_state(state, exit_uwb, exit_visual,
                                exit_left_fwd, exit_yellow_return,
                                exit_scan_items, exit_return_depart)
                    enter_uwb()
                    state = STATE_UWB
                    loop_cnt = 0
                    continue
            if key_triggered(2):           # KEY2 → VISUAL
                if state != STATE_VISUAL:
                    print("\n[KEY2] → VISUAL_TRACK")
                    _exit_state(state, exit_uwb, exit_visual,
                                exit_left_fwd, exit_yellow_return,
                                exit_scan_items, exit_return_depart)
                    enter_visual()
                    state = STATE_VISUAL
                    loop_cnt = 0
                    continue
            # KEY3 已移除（原 STOPPED 模式）

            # ─── SW2 检测 ───
            if check_sw2():
                print("\n[SW2] Exit requested")
                _exit_state(state, exit_uwb, exit_visual,
                            exit_left_fwd, exit_yellow_return,
                            exit_scan_items, exit_return_depart)
                break

            # ════════════════════════════════════════════════
            #  状态 0 — UWB 跟随
            # ════════════════════════════════════════════════
            if state == STATE_UWB:
                if res['uwb']:
                    res['uwb'].step()

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
                            print("[CAM] Target confirmed! {}/{} → VISUAL".format(
                                hits, len(res['window'])))
                            exit_uwb()
                            enter_visual()
                            state = STATE_VISUAL
                            loop_cnt = 0
                            continue

            # ════════════════════════════════════════════════
            #  状态 1 — 视觉追踪
            # ════════════════════════════════════════════════
            elif state == STATE_VISUAL:
                if res['cam_recv'] is None:
                    print("[VISUAL] CamDataReceiver lost → DONE")
                    exit_visual()
                    state = STATE_DONE
                    break

                data = res['cam_recv'].read()
                if data is None:
                    time.sleep_ms(1)
                    continue

                now = time.ticks_ms()

                res['window'].append(1 if data['is_target'] else 0)
                if len(res['window']) > WINDOW_SIZE:
                    res['window'].pop(0)
                valid_count = sum(res['window'])
                res['tracking'] = (valid_count >= WINDOW_THRESHOLD
                                   and len(res['window']) >= WINDOW_SIZE)

                x_cm = x_to_cm(data['x'])
                actual_dist = y_to_distance(data['y'])
                ctrl = compute_control(x_cm, actual_dist, data['is_target'], now)

                if ctrl['state_msg']:
                    print(ctrl['state_msg'])

                # 到达目标 → 进入左前盲走
                if ctrl['arrived']:
                    exit_visual()
                    enter_left_fwd()
                    state = STATE_LEFT_FWD
                    loop_cnt = 0
                    print_cnt = 0
                    continue

                if ctrl['cmd_fwd'] is not None and res['first_frames'] <= 0:
                    try:
                        rc = get_encoder_counts()
                        rs = [rc[i] / ENC_SCALE[i] / CAM_DT for i in range(4)]
                        omni_drive_closed_loop(
                            ctrl['cmd_fwd'], ctrl['cmd_lat'], 0, rs, CAM_DT)
                    except Exception as e:
                        print("[MOTOR] drive error:", e)
                elif res['first_frames'] > 0:
                    res['first_frames'] -= 1

                if data['is_target']:
                    res['last_data']    = now
                    res['timeout_done'] = False

                print_cnt += 1
                if print_cnt >= 15:
                    print_cnt = 0
                    state_str = {0: "FOLLOW", 1: "STOP", 2: "LOST"}[ctrl['state']]
                    print("[#{:04d} {:s}] X:{:+5.1f}cm dist:{:5.1f}cm "
                          "fwd:{:+.3f} lat:{:+.3f}".format(
                        res['cam_recv'].frame_count, state_str,
                        x_cm, actual_dist,
                        PID_FWD.prev_output, PID_LAT.prev_output))

                if not res['timeout_done']:
                    if time.ticks_diff(now, res['last_data']) > CAM_TIMEOUT_MS:
                        res['timeout_done'] = True
                        stop_all()
                        res['tracking'] = False
                        res['window'] = []
                        print("[VISUAL] Timeout {}ms → DONE".format(CAM_TIMEOUT_MS))
                        exit_visual()
                        state = STATE_DONE
                        break

                if loop_cnt % 20 == 0:
                    led.toggle()

            # ════════════════════════════════════════════════
            #  状态 3 — 45°左前盲走（检测黄线）
            # ════════════════════════════════════════════════
            elif state == STATE_LEFT_FWD:
                data = res['cam_recv'].read()
                if data is None:
                    time.sleep_ms(1)
                    continue

                # 检查黄线 FLAG
                if data['flag'] == FLAG_YELLOW:
                    print("[LEFT_FWD] Yellow line detected! (FLAG={:02X})".format(data['flag']))
                    exit_left_fwd()
                    enter_yellow_return()
                    state = STATE_YELLOW_RETURN
                    loop_cnt = 0
                    print_cnt = 0
                    continue

                # 45°左前驱动（前5帧打印调试）
                drive_left_forward(debug=(print_cnt < 5))

                print_cnt += 1
                if print_cnt >= 15:
                    print_cnt = 0
                    print("[LEFT_FWD] X:{:+.1f}cm dist:{:5.1f}cm flag:{:02X}".format(
                        x_to_cm(data['x']), y_to_distance(data['y']), data['flag']))

                if loop_cnt % 20 == 0:
                    led.toggle()

            # ════════════════════════════════════════════════
            #  状态 4 — 180°掉头 + UWB 回物品区
            # ════════════════════════════════════════════════
            elif state == STATE_YELLOW_RETURN:
                # 阶段 A：先完成 180° 掉头
                if not res['turn_done']:
                    res['turn_done'] = do_180_turn()
                    if res['turn_done']:
                        print("[YELLOW_RET] 180 turn done, starting UWB nav")
                        # 启动 UWB 导航回物品区
                        res['uwb'] = enter_uwb_nav(target_anchor="8834")
                    continue

                # 阶段 B：UWB 导航步进
                if res['uwb']:
                    res['uwb'].step()

                # 检查是否回到物品区（摄像头检测到物品目标）
                data = res['cam_recv'].read()
                if data is not None and data['is_target']:
                    print("[YELLOW_RET] Item area detected → SCAN_ITEMS")
                    exit_yellow_return()
                    enter_scan_items()
                    state = STATE_SCAN_ITEMS
                    loop_cnt = 0
                    print_cnt = 0
                    continue

                time.sleep_ms(10)

            # ════════════════════════════════════════════════
            #  状态 5 — 扫描物品区
            # ════════════════════════════════════════════════
            elif state == STATE_SCAN_ITEMS:
                data = res['cam_recv'].read()
                if data is None:
                    time.sleep_ms(1)
                    continue

                if data['is_target']:
                    # 还有物品 → 重新开始物品跟随循环
                    print("[SCAN] Item found! Restarting UWB→VISUAL cycle")
                    exit_scan_items()
                    enter_uwb()
                    state = STATE_UWB
                    loop_cnt = 0
                    print_cnt = 0
                    continue
                else:
                    res['scan_empty_cnt'] += 1

                print_cnt += 1
                if print_cnt >= 10:
                    print_cnt = 0
                    print("[SCAN] empty frames: {}/{}".format(
                        res['scan_empty_cnt'], SCAN_EMPTY_FRAMES))

                # 连续 N 帧无目标 → 物品区清空 → 返回发车区
                if res['scan_empty_cnt'] >= SCAN_EMPTY_FRAMES:
                    print("[SCAN] Items area CLEAR → RETURN_DEPARTURE")
                    exit_scan_items()
                    enter_return_depart()
                    state = STATE_RETURN_DEPART
                    loop_cnt = 0
                    print_cnt = 0
                    continue

                if loop_cnt % 50 == 0:
                    led.toggle()

            # ════════════════════════════════════════════════
            #  状态 6 — 返回发车区（占位）
            # ════════════════════════════════════════════════
            elif state == STATE_RETURN_DEPART:
                data = res['cam_recv'].read()
                if data is None:
                    time.sleep_ms(1)
                    continue

                # 检查发车区 FLAG
                if data['flag'] == FLAG_DEPARTURE:
                    print("[RET_DEPART] Departure zone reached! (FLAG={:02X})".format(data['flag']))
                    exit_return_depart()
                    state = STATE_DONE
                    loop_cnt = 0
                    break

                # [占位] 发车区导航 — 当前不做任何移动
                print_cnt += 1
                if print_cnt >= 30:
                    print_cnt = 0
                    print("[RET_DEPART] waiting for departure FLAG... flag={:02X}".format(
                        data['flag']))

                if loop_cnt % 20 == 0:
                    led.toggle()

            time.sleep_ms(10)

            # ─── DONE 状态退出 ───
            if state == STATE_DONE:
                break

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


# ── 辅助：安全退出任意状态 ─────────────────────────────────

def _exit_state(state, exit_uwb, exit_visual,
                exit_left_fwd, exit_yellow_return,
                exit_scan_items, exit_return_depart):
    """根据当前状态调用对应的 exit 函数"""
    _exit_map = {
        STATE_UWB:           exit_uwb,
        STATE_VISUAL:        exit_visual,
        STATE_LEFT_FWD:      exit_left_fwd,
        STATE_YELLOW_RETURN: exit_yellow_return,
        STATE_SCAN_ITEMS:    exit_scan_items,
        STATE_RETURN_DEPART: exit_return_depart,
    }
    exiter = _exit_map.get(state)
    if exiter:
        exiter()


def pause_encoder_ticker():
    try:
        enc_ticker.stop()
    except Exception:
        pass


main()
