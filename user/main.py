#  RT1021 — 按键驱动控制 (1D S-Pattern & Unified Driver Refactored)

import gc, time, math
from machine import Pin
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
                   pause_encoder_ticker, resume_encoder_ticker, ENC_SCALE,
                   )
gc.collect()

from imu_motion import (update_angle, imu_read_safe,
                         reset_ang_vel_pid, stop_imu_ticker)

def _yaw():
    return __import__('imu_motion').yaw

from key import capture, key_triggered, pet_watchdog, stop as stop_key, start_watchdog
from uwb_position import UWBPosition
from IMU_hold import HeadingHold
import uwb_control


# ═══════════════════════════════════════════════════════════════
#  常量区域
# ═══════════════════════════════════════════════════════════════

LED_PIN  = 'C4'
SW2_PIN  = 'D9'

# ── C14: 前进 20cm ──
FORWARD_DIST_M   = 0.20    # 目标距离 (m)
FORWARD_SPEED    = 0.30    # 前进速度 (m/s)
FWD_TIMEOUT_S    = 10.0    # 超时 (s)
FWD_CTRL_DT      = 0.02    # 控制周期 (s)

# ── C14: UWB 平移 ──
UWB_LAT_SPEED    = 0.30    # 最大平移速度 (m/s)
UWB_X_DEADBAND   = 3.0     # X 方向死区 (cm)

UWB_LAT_P_GAIN   = 0.02    # X 误差 → 平移速度 P 增益
UWB_TIMEOUT_S    = 10.0    # 超时 (s)
UWB_CTRL_DT      = 0.02    # 控制周期 (s)

_heading_hold = None        # HeadingHold 实例（首次 _lock_yaw 时懒初始化）

# ── 蓝牙信号 ──
BT_WAIT_DEADLINE_S       = 30.0     # 蓝牙单次等待超时 (s)

# ── 启动: 收到 ok 后全速前进至 UWB X 距离 < -150cm ──
STARTUP_FULL_SPEED     = 1.00     # 全速前进速度（绝对值，m/s）
UWB_X_THRESHOLD_CM     = -150.0   # UWB X 轴距离阈值 (cm)，小于此值触发停车
UWB_X_SLOWDOWN_CM      = -80.0    # X < 此值时开始线性减速，防止冲出 UWB 覆盖
UWB_X_MIN_SPEED        = 0.25     # 接近阈值时的最低速度 (m/s)
UWB_X_TIMEOUT_S        = 15.0     # UWB X 距离检测超时 (s)
UWB_BACKUP_DIST_CM     = 20.0     # 触发后倒退距离 (cm)
UWB_BACKUP_SPEED       = 0.30     # 倒退速度 (m/s)
UWB_BACKUP_TIMEOUT_S   = 4.0      # 倒退超时 (s)
UWB_DEAD_TIMEOUT_S     = 2.0      # UWB 掉线容忍超时 (s)
UWB_DEAD_RECONNECT_MAX  = 4       # UWB 掉线最大重连次数

# ── 返航: 返回 origin 坐标 ──
ORIGIN_CTRL_DT      = 0.01    # 控制周期 (s)

# ── 倒车回到右移路径 ──
BACKUP_PATH_SPEED   = 0.50    # 倒车回到路径速度 (m/s)
BACKUP_PATH_TIMEOUT = 15.0    # 倒车回到路径超时 (s)

# ── move_toward_fixed_point ──
MFP_FORWARD_DIST_CM  = 55.0    # 目标50cm 
MFP_RIGHT_DIST_CM    = 60.0    # 目标110cm - 20cm惯性 = 90.0cm
MFP_SPEED            = 0.40    # 行进速度 (m/s)
MFP_TIMEOUT_S        = 15.0    # 超时 (s)
MFP_CTRL_DT          = 0.02    # 控制周期 (s)

# ── S形搜索参数 ──
RIGHTWARD_SPEED          = 0.30    # 平移速度 (m/s)
RIGHTWARD_TIMEOUT_S      = 20.0    # 单段平移超时 (s)
RIGHTWARD_CTRL_DT        = 0.02    # 控制周期 (s)

# ── SW2 ──
SW2_DEBOUNCE_MS  = 50

# ── 状态机与物理位置全局变量 ──
RUN_MODE              = "test"   # 运行模式: "loop" 或 "test"
origin                = None    # 起点坐标 (x, y)
supplies              = None    # 动态物资识别断点坐标 (x, y)，由扫描阶段动态写入
_uwb_shared           = None    # 共享 UWBPosition 实例
_cam_shared           = None    # 共享 CameraController 实例
_TARGET_HEADING       = None    # 全程锁定的目标航向
_approach_forward_dist = 0.0     # 取物流程净前进距离 (m)，由正解投影精确累加
_search_progress_cm   = 0.0     # 一维轴向S形总里程进度（0 ~ 220cm）
_origin_return_offset = 0.0     # 返航 X 坐标累计偏移 (cm)，每次返航 +10
_just_pushed          = False    # 避盲区保护标志：刚完成推出后置 True，用于起步避开二次触发

# ── 本地一阶低通滤波器（用于动力闭环速度输入）──
_SPD_FILTER_ALPHA = 0.75
_local_prev_speeds = [0.0, 0.0, 0.0, 0.0]
_local_speeds_initialized = False

# ── 延迟导入占位符 ──
CameraController = None
goto_location = None


def _get_local_filtered_speeds(raw_counts, dt):
    """基于外部传入 of counts 数组计算一阶低通滤波速度 [rf, lf, lb, rb] (m/s)。"""
    global _local_prev_speeds, _local_speeds_initialized
    if raw_counts is None or len(raw_counts) < 4:
        return _local_prev_speeds
    if dt <= 0.0001:
        dt = FWD_CTRL_DT
    if not _local_speeds_initialized:
        for i in range(4):
            _local_prev_speeds[i] = (raw_counts[i] / ENC_SCALE[i] / dt) if ENC_SCALE[i] != 0 else 0
        _local_speeds_initialized = True
        return _local_prev_speeds
    for i in range(4):
        raw_spd = (raw_counts[i] / ENC_SCALE[i] / dt) if ENC_SCALE[i] != 0 else 0
        _local_prev_speeds[i] = _SPD_FILTER_ALPHA * _local_prev_speeds[i] + (1 - _SPD_FILTER_ALPHA) * raw_spd
    return _local_prev_speeds


def _clean_globals_for_ide():
    """程序退出前清空命名空间中的自定义全局变量。"""
    print("  [IDE] 正在清理全局变量，防止 IDE 扫描冲突...")
    g = globals()
    keep = {'__name__', '__file__', 'gc', 'time', 'math', 'machine', 'sys', 'builtins'}
    for name in list(g.keys()):
        if name in keep:
            continue
        try:
            g[name] = None
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════
#  硬件初始化
# ═══════════════════════════════════════════════════════════════

led = Pin(LED_PIN, Pin.OUT, value=False)
sw2 = Pin(SW2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)

_sw2_last         = sw2.value()
_sw2_changed      = False
_sw2_stable_start = 0


def check_sw2():
    global _sw2_last, _sw2_changed, _sw2_stable_start
    val = sw2.value()
    if val != _sw2_last:
        _sw2_last = val
        _sw2_changed = True
        _sw2_stable_start = time.ticks_ms()
    if _sw2_changed and time.ticks_diff(time.ticks_ms(), _sw2_stable_start) >= SW2_DEBOUNCE_MS:
        _sw2_changed = False
        if val == 0:
            return True
    return False


# ═══════════════════════════════════════════════════════════════
#  统一的初始化与资源释放入口
# ═══════════════════════════════════════════════════════════════

def _system_init():
    """统一初始化：保证 loop / test 模式硬件与软件状态完全一致"""
    global origin, CameraController, goto_location, _TARGET_HEADING, _origin_return_offset
    print("\n[SYSTEM] 执行统一系统状态初始化...")

    pause_encoder_ticker()
    _encoder_reset()

    # 1. IMU 陀螺仪标定及目标偏航锁定
    _lock_yaw()

    # 2. 预先加载并初始化重型相机/控制模块
    gc.collect()
    import cam_control
    CameraController = cam_control.CameraController
    import uwb_control
    goto_location = uwb_control.goto_location
    gc.collect()
    print("  [INIT] 闭环控制模块加载成功")

    # 3. 正式开启 WDT
    start_watchdog()
    print("  [INIT] 独立硬件看门狗已启用 (3秒超时)")

    _origin_return_offset = 0.0

    # 4. 初始化 UWB（两个模式均统一初始化，保证测试/循环一致性）
    uwb = _ensure_uwb()
    if uwb is not None and uwb.get_frame_count() > 0:
        origin = uwb.get_position()
        print("  [INIT] UWB 起点坐标已记录: ({:.1f}, {:.1f})".format(origin[0], origin[1]))
    else:
        print("  [INIT] UWB 未就绪，使用默认坐标")
        origin = (130.0, 262.0)

    print("[SYSTEM] 初始化就绪，空闲内存: {} 字节\n".format(gc.mem_free()))


def _system_cleanup():
    """统一资源释放"""
    print("\n[SYSTEM] 执行统一系统资源回收...")
    stop_all()
    _encoder_reset()
    try:
        pause_encoder_ticker()
        stop_imu_ticker()
        stop_key()
        _clean_globals_for_ide()
    except Exception as e:
        print("  [CLEANUP] 异常:", e)
    print("[SYSTEM] 资源已安全释放。")


# ═══════════════════════════════════════════════════════════════
#  蓝牙从车通信
# ═══════════════════════════════════════════════════════════════

from uart_master import MasterBT
bt = MasterBT()


# ═══════════════════════════════════════════════════════════════
#  工具函数
# ═══════════════════════════════════════════════════════════════

_imu_ticker_stopped = False
_imu_ok_count = 0
_imu_fail_count = 0
_maintain_yaw_fail = 0


def _read_imu_update_yaw():
    global _imu_ticker_stopped, _imu_ok_count, _imu_fail_count
    if not _imu_ticker_stopped:
        print("  [IMU] 停止 PIT3 ticker，切换到直接 SPI 读取...")
        stop_imu_ticker()
        _imu_ticker_stopped = True

    d = imu_read_safe()
    if d is not None:
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        _imu_ok_count += 1
        if _imu_ok_count == 1:
            print("  [IMU] 首次成功 raw=(gx:{0:.1f} gy:{1:.1f} gz:{2:.1f}) yaw={3:.2f}°".format(
                d[3], d[4], d[5], _yaw()))
        if _imu_ok_count % 50 == 0:
            print("  [IMU] ok={}/fail={} yaw={:.2f}° gz_raw=({:+.1f},{:+.1f},{:+.1f})".format(
                _imu_ok_count, _imu_fail_count, _yaw(), d[3], d[4], d[5]))
        return True
    _imu_fail_count += 1
    if _imu_fail_count == 1:
        print("  [IMU] 首次读取失败")
    if _imu_fail_count % 50 == 0:
        print("  [IMU] fail count={} yaw={:.2f}°".format(_imu_fail_count, _yaw()))
    return False


def _get_heading_hold():
    global _heading_hold
    if _heading_hold is None:
        _heading_hold = HeadingHold()
    return _heading_hold


def _lock_yaw():
    global _TARGET_HEADING
    if _TARGET_HEADING is not None:
        return _TARGET_HEADING
    hold = _get_heading_hold()
    for _ in range(20):
        if _read_imu_update_yaw():
            _TARGET_HEADING = _yaw()
            hold.set_target(_TARGET_HEADING)
            print("  [HEADING] 全程航向锁定: {:.1f}°（偏差自动回正）".format(_TARGET_HEADING))
            return _TARGET_HEADING
        time.sleep_ms(10)
    _TARGET_HEADING = _yaw()
    hold.set_target(_TARGET_HEADING)
    print("  [HEADING] 全程航向锁定(兜底): {:.1f}°".format(_TARGET_HEADING))
    return _TARGET_HEADING


def _heading_correction(target_yaw, deadband=0.40, dt=None):
    if dt is None:
        dt = FWD_CTRL_DT
    if deadband is None:
        deadband = 0.40
    _read_imu_update_yaw()
    current_yaw = _yaw()
    diff = (target_yaw - current_yaw + 180) % 360 - 180
    if abs(diff) <= deadband:
        return 0.0
    hold = _get_heading_hold()
    if target_yaw != hold.target:
        hold.set_target(target_yaw)
    wz, _, _ = hold.update(current_yaw, dt, deadband=deadband)
    return wz


def _maintain_yaw(target_heading):
    """单次 yaw 保持迭代：应用层角度环回强校验 + 委托 HeadingHold → 驱动电机。"""
    global _maintain_yaw_fail
    _read_imu_update_yaw()
    current_yaw = _yaw()
    diff = (target_heading - current_yaw + 180) % 360 - 180
    if abs(diff) <= 1.0:
        stop_all()
        return True
    hold = _get_heading_hold()
    if target_heading != hold.target:
        hold.set_target(target_heading)
    wz, _, _ = hold.update(current_yaw, FWD_CTRL_DT)
    if abs(wz) < 0.005:
        stop_all()
        return True
    try:
        rc = get_encoder_counts()
        if rc is not None and len(rc) >= 4:
            speeds = _get_local_filtered_speeds(rc, 0.02)
            omni_drive_closed_loop(0, 0, wz, speeds, 0.02)
    except Exception as e:
        _maintain_yaw_fail += 1
        if _maintain_yaw_fail == 1:
            print("  [YAW] _maintain_yaw 驱动异常(首次):", e)
        return False
    return True


def _pause_with_yaw_hold(target_heading, duration_ms):
    deadline = time.ticks_ms() + duration_ms
    while time.ticks_diff(deadline, time.ticks_ms()) > 0:
        pet_watchdog() # 💡 强力喂狗
        _maintain_yaw(target_heading)
        time.sleep_ms(10)
    stop_all()


def _encoder_reset():
    global _local_prev_speeds, _local_speeds_initialized
    reset_encoder_filter()
    reset_wheel_pi()
    reset_ang_vel_pid()
    if _heading_hold is not None:
        _heading_hold.reset()
    _local_prev_speeds = [0.0, 0.0, 0.0, 0.0]
    _local_speeds_initialized = False
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)


def _abort_check():
    pet_watchdog()
    if check_sw2():
        print("  [SW2] Abort requested")
        return True
    return False


# ═══════════════════════════════════════════════════════════════
#  UWB 共享实例管理
# ═══════════════════════════════════════════════════════════════

def _ensure_uwb():
    global _uwb_shared
    _reset_uwb_if_needed()
    if _uwb_shared is None:
        try:
            _uwb_shared = UWBPosition(uart_id=0, baudrate=115200, target_anchor="8834")
            print("  [UWB] 初始化中，等待首帧...")
            wait_start = time.ticks_ms()
            while _uwb_shared.get_frame_count() == 0:
                _uwb_shared.step()
                pet_watchdog()
                if _TARGET_HEADING is not None:
                    _maintain_yaw(_TARGET_HEADING)
                if time.ticks_diff(time.ticks_ms(), wait_start) > 3000:
                    print("  [UWB] 首帧超时")
                    _uwb_shared.stop()
                    _uwb_shared = None
                    return None
                time.sleep_ms(10)
            if _uwb_shared.get_frame_count() > 0:
               print("  [UWB] 就绪 (frame={})".format(_uwb_shared.get_frame_count()))
        except Exception as e:
            print("  [UWB] 初始化失败:", e)
            _uwb_shared = None
    return _uwb_shared


def _reset_uwb_if_needed():
    global _uwb_shared
    if _uwb_shared is not None and _uwb_shared.is_timeout():
        if _uwb_shared.is_uart_alive():
            print("  [UWB] 帧超时但 UART 硬件正常，等待恢复...")
            return
        print("  [UWB] UART 硬件断连，重置连接...")
        _uwb_shared.stop()
        _uwb_shared = None


# ═══════════════════════════════════════════════════════════════
#  摄像头共享实例管理
# ═══════════════════════════════════════════════════════════════

def _ensure_cam():
    global _cam_shared
    if _cam_shared is None:
        _cam_shared = CameraController(uart_id=7)
        print("[CAM] CameraController 就绪")
    return _cam_shared


# ═══════════════════════════════════════════════════════════════
#  二、统一动力闭环驱动器 (Unified Closed-Loop Driver)
# ═══════════════════════════════════════════════════════════════

def _drive_closed_loop(vx, vy, target_dist_m, check_target=False, heading_deadband=1.0, timeout_s=15.0, label="DRIVE", ignore_dist_m=0.0):
    """
    统一的参数化物理闭环驱动器。
    代替原本分散重复的直行、平移测距运动，内部融合了：
    IMU 航向环校正、一阶低通滤波测速、硬件看门狗喂狗、SW2 毫秒级打断、高精度里程积分和视觉每帧拦截。

    返回:
        (has_spotted_target, actual_moved_dist_m)
    """
    global _local_prev_speeds, _local_speeds_initialized
    print("  [{}] 启动: vx={:.2f} vy={:.2f} 目标距离={:.2f}m".format(label, vx, vy, target_dist_m))
    target_heading = _lock_yaw()
    
    dist_integrated = 0.0
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    last_loop_ms = start_ms
    last_uwb_step_ms = start_ms
    loop_cnt = 0
    cam = _ensure_cam() if check_target else None
    led.value(1)

    while True:
        now_ms = time.ticks_ms()
        pet_watchdog()

        # ── UWB 串口防积压：每隔 50ms 步进一次，防止长期不读导致 uart_bytes=255 超时 ──
        if time.ticks_diff(now_ms, last_uwb_step_ms) >= 50:
            if _uwb_shared is not None:
                _uwb_shared.step()
            last_uwb_step_ms = now_ms

        if _abort_check():
            stop_all()
            led.value(0)
            print("  [{}] SW2 中途紧急打断！".format(label))
            return False, dist_integrated

        elapsed = time.ticks_diff(now_ms, start_ms) / 1000.0
        if elapsed > timeout_s:
            stop_all()
            led.value(0)
            print("  [{}] 运行超时 ({:.1f}s)".format(label, elapsed))
            return False, dist_integrated

        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001 or actual_dt > 3 * FWD_CTRL_DT:
            actual_dt = FWD_CTRL_DT
            _local_speeds_initialized = False
            for i in range(4):
                _local_prev_speeds[i] = 0.0
        last_loop_ms = now_ms

        # 统一闭环里程：绝对值累加
        step = sum(abs(counts[i] / ENC_SCALE[i]) for i in range(4)) / 4.0
        dist_integrated += step

        # 判断是否到达距离
        if dist_integrated >= target_dist_m:
            stop_all()
            led.value(0)
            print("  [{}] 完成指定目标距离: {:.2f}m".format(label, dist_integrated))
            return False, dist_integrated

        # 判断是否被视觉拦截（满足最小起步避盲区距离 ignore_dist_m 后才允许触发拦截）
        if check_target and cam is not None and dist_integrated >= ignore_dist_m:
            ctrl = cam.step()
            if ctrl and ctrl.get('has_target', False):
                stop_all()
                led.value(0)
                print("  [{}] 视觉捕获拦截! 当前进度已保存。".format(label))
                return True, dist_integrated

        if time.ticks_diff(now_ms, last_print_ms) >= 500:
            last_print_ms = now_ms
            print("  [{}] dist={:.2f}m / {:.2f}m  yaw={:.2f}°".format(
                label, dist_integrated, target_dist_m, _yaw()))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        wz = _heading_correction(target_heading, deadband=heading_deadband, dt=actual_dt)
        try:
            speeds = _get_local_filtered_speeds(counts, actual_dt)
            omni_drive_closed_loop(vx, vy, wz, speeds, actual_dt)
        except Exception as e:
            print("  [{}] 闭环控制驱动异常:".format(label), e)

        time.sleep_ms(int(FWD_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  UWB 坐标断点记录器
# ═══════════════════════════════════════════════════════════════

def _record_supplies_position():
    """读取当前 UWB 物理坐标，动态记录到 supplies 变量作为断点返回点。
    优化：前行扫描期间没有 step，此处通过短暂循环刷新串口缓冲区，
    确保获取到最新鲜、未过期的物理坐标。"""
    global supplies
    uwb = _ensure_uwb()
    if uwb is not None:
        print("  [UWB_RECORD] 正在刷新 UWB 接收缓冲区以获取最新坐标...")
        start_t = time.ticks_ms()
        old_frames = uwb.get_frame_count()
        # 持续 step 刷新，最多等待 300ms，或者直到串口接收到至少一个全新的完整数据帧
        while time.ticks_diff(time.ticks_ms(), start_t) < 300:
            uwb.step()
            time.sleep_ms(10)
            if uwb.get_frame_count() > old_frames:
                break

        try:
            supplies = uwb.get_position()
            print("  [UWB_RECORD] 成功记录当前最新坐标到 supplies 断点: ({:.1f}, {:.1f})".format(
                supplies[0], supplies[1]))
        except Exception as e:
            print("  [UWB_RECORD] 读取并保存 UWB 断点失败:", e)
    else:
        print("  [UWB_RECORD] 警告：UWB 未就绪或掉线，无法记录 supplies 坐标（将使用编码器倒车备用方案）")


# ═══════════════════════════════════════════════════════════════
#  一维一字型 S 曲线搜索算法 (1D S-Pattern State Machine)
# ═══════════════════════════════════════════════════════════════

def _action_s_pattern_search():
    """
    一维轴向一字型 S 曲线搜索路径规划。
    一维总进度通过全局变量 _search_progress_cm (0 ~ 220cm) 统一管理。
    """
    global _search_progress_cm, _just_pushed, supplies
    # 清空 move_toward_fixed_point 阶段积压 of 摄像头 UART 脏帧，保证视觉时效
    _ensure_cam().reset()
    target_heading = _lock_yaw()
    print("\n  [S_SEARCH] 开始/恢复 S 曲线搜索路径。当前全局总轴向进度: {:.1f}cm".format(_search_progress_cm))

    # 确定起步避空行程（防原地二次误触发刚推完的物品，测试/循环模式通用）
    ignore_d = 0.15 if _just_pushed else 0.0
    _just_pushed = False  # 清除标志位

    # ── 阶段 0: 右平移搜索区 (0 - 90cm) ──
    if _search_progress_cm < 90.0:
        rem_dist_m = (90.0 - _search_progress_cm) / 100.0
        print("  [S_SEARCH] 阶段 0 (向右平移搜索): 剩余待搜索距离 = {:.1f}cm".format(rem_dist_m * 100.0))
        found, moved_m = _drive_closed_loop(0, RIGHTWARD_SPEED, rem_dist_m, check_target=True, label="S_STAGE_0", ignore_dist_m=ignore_d)
        _search_progress_cm += moved_m * 100.0
        if found:
            _record_supplies_position()
            print("  [S_SEARCH] 阶段 0 捕获目标! 断点已锁定在: {:.1f}cm".format(_search_progress_cm))
            return True
        if check_sw2():
            return False

    # ── 阶段 1: 前行过渡区 (90 - 140cm) ──
    if 90.0 <= _search_progress_cm < 140.0:
        rem_dist_m = (140.0 - _search_progress_cm) / 100.0
        print("  [S_SEARCH] 阶段 1 (前行过渡): 剩余前行过渡距离 = {:.1f}cm".format(rem_dist_m * 100.0))
        bt.send_direction('L', 0.40)  # 前进
        # 前行段不进行视觉拦截
        _, moved_m = _drive_closed_loop(0.40, 0, rem_dist_m, check_target=False, label="S_STAGE_1")
        bt.send_direction('exit')
        _search_progress_cm += moved_m * 100.0
        if check_sw2():
            return False

    # ── 阶段 2: 左平移搜索区 (140 - 240cm) ──
    if 140.0 <= _search_progress_cm < 240.0:
        rem_dist_m = (220.0 - _search_progress_cm) / 100.0
        print("  [S_SEARCH] 阶段 2 (向左平移搜索): 剩余待搜索距离 = {:.1f}cm".format(rem_dist_m * 100.0))
        bt.send_direction('B', RIGHTWARD_SPEED)  # 左移
        # 向左平移使用负的 Y 速度分量
        found, moved_m = _drive_closed_loop(0, -RIGHTWARD_SPEED, rem_dist_m, check_target=True, label="S_STAGE_2", ignore_dist_m=ignore_d)
        bt.send_direction('exit')
        _search_progress_cm += moved_m * 100.0
        if found:
            _record_supplies_position()
            print("  [S_SEARCH] 阶段 2 捕获目标! 断点已锁定在: {:.1f}cm".format(_search_progress_cm))
            return True
        if check_sw2():
            return False

    # ── 阶段 3: 无果判定 ──
    if _search_progress_cm >= 240.0:
        print("  [S_SEARCH] 阶段 3 (轴向搜索结束，未发现任何物品)。")
        return False

    return False


# ═══════════════════════════════════════════════════════════════
#  辅助倒退执行闭环函数 (Compatible Wrapper)
# ═══════════════════════════════════════════════════════════════

def _execute_backup(target_heading):
    """倒退 20cm，使用统一闭环驱动器进行动作复用"""
    print("  [BACKUP] 开始倒退 {:.0f}cm...".format(UWB_BACKUP_DIST_CM))
    _drive_closed_loop(-UWB_BACKUP_SPEED, 0, UWB_BACKUP_DIST_CM / 100.0, check_target=False, label="BACKUP")
    time.sleep_ms(300)
    _encoder_reset()


# ═══════════════════════════════════════════════════════════════
#  倒退回到原本 S 搜索路径
# ═══════════════════════════════════════════════════════════════

def _reverse_to_rightward_path():
    """
    复位到搜索参考线。
    优先：通过 UWB 导航飞回动态记录的 supplies 断点坐标；
    备用：若 UWB 离线或坐标为空（如测试模式），采用物理打滑比例补偿倒退。
    """
    global _approach_forward_dist, supplies

    uwb = _ensure_uwb()

    # ── 优先方案：UWB 闭环导航返回 supplies 断点 ──
    if uwb is not None and not uwb.is_timeout() and supplies is not None:
        print("  [REV] 检测到有效 UWB 及 supplies 断点坐标，启动 UWB 导航返回: ({:.1f}, {:.1f})".format(
            supplies[0], supplies[1]))
        target_x, target_y = supplies

        # 声明 goto_location 的闭环控制回调依赖
        def _lock_fn(): return _lock_yaw()
        def _wz_fn(target, db): return _heading_correction(target, deadband=db, dt=ORIGIN_CTRL_DT)
        def _yaw_fn(): return _yaw()
        def _abort_fn():
            pet_watchdog()
            if check_sw2():
                print("  [SW2] Abort requested")
                return True
            return False
        def _drive_fn(vx, vy, wz, dt):
            rc = get_encoder_counts()
            if rc is not None and len(rc) >= 4:
                speeds = _get_local_filtered_speeds(rc, dt)
                omni_drive_closed_loop(vx, vy, wz, speeds, dt)
        def _stop_fn(): stop_all()
        def _led_fn(val): led.value(val)

        # 闭环导航返回断点起点：临时收紧到位死区防止多次循环累积横向漂移
        _save_db = uwb_control.GOTO_DB
        uwb_control.GOTO_DB = 5.0  # 缩小到位死区，提高轨道复位精度
        try:
            arrived, reason = goto_location(
                uwb, target_x, target_y,
                _lock_fn, _wz_fn, _yaw_fn,
                _abort_fn, _drive_fn, _stop_fn,
                _led_fn, label="REV_UWB"
            )
        finally:
            uwb_control.GOTO_DB = _save_db  # 恢复原值，不影响返航精度
        if arrived:
            print("  [REV] 成功通过 UWB 导航复位到 supplies 断点轨道！")
            _approach_forward_dist = 0.0
            _encoder_reset()
            return True
        else:
            print("  [REV] UWB 导航返回断点失败 (Reason: {})，自动切换到备用编码器倒车".format(reason))

    # ── 备用方案（Fallback）：编码器滑移补偿倒退（支持测试模式与断连保障） ──
    SLIP_COMPENSATION_FACTOR = 0.65  # 轮子阻力打滑补偿系数
    compensated_dist = _approach_forward_dist * SLIP_COMPENSATION_FACTOR

    if compensated_dist <= 0.001:
        print("  [REV] 积攒前进距离 ≈ 0，无需返回")
        return True

    print("  [REV] [Fallback] UWB 不可用，使用编码器补偿。原积攒: {:.2f}m, 补偿后倒退: {:.2f}m...".format(
        _approach_forward_dist, compensated_dist))

    # 使用统一闭环驱动器向后推进
    _, dist_m = _drive_closed_loop(-BACKUP_PATH_SPEED, 0, compensated_dist,
                                   check_target=False, timeout_s=BACKUP_PATH_TIMEOUT, label="REV_FALLBACK")

    _approach_forward_dist = 0.0
    _encoder_reset()
    return dist_m >= compensated_dist


# ═══════════════════════════════════════════════════════════════
#  摄像头跟随靠近 (see_and_push) - 竞速级高速对齐版
# ═══════════════════════════════════════════════════════════════

def see_and_push():
    """基于物理学逆运动学正解的高精度视觉对齐靠近算法 (竞速级高速对齐版)。
    1.35倍速度增益消除拖沓，0.08m/s收敛门限瞬间斩断蠕动长尾，提升比赛竞速净效率。"""
    global _approach_forward_dist, _local_prev_speeds, _local_speeds_initialized
    print("\n  [APPROACH] === 视觉对齐靠近 ➔ 蓝牙同步流程 ===")
    cam = _ensure_cam()
    # 清空 S 扫描残留帧 + 复位视觉追踪状态，确保与 test 模式一致的干净启动
    cam.reset()
    recv = cam._recv
    fc = cam._ctrl
    from cam_data import x_to_cm, y_to_distance

    target_heading = _lock_yaw()
    last_x_cm = 0.0
    last_dist_cm = 0.0
    last_has_tgt = False

    # ── 物理停靠判定变量 ──
    align_consecutive_count = 0
    arrived_success = False

    # ── 计时器与保护机制 ──
    start_time_ms = time.ticks_ms()
    last_target_seen_ms = start_time_ms
    MAX_APPROACH_TIME_S = 8.0     # 8秒整段最大靠近时限，防止意外卡住
    LOST_TARGET_TIMEOUT_MS = 1500  # 丢失目标 1.5秒 安全退出保护

    LOOP_MS = 10
    t_prev = time.ticks_ms()
    last_uwb_step_ms = t_prev
    loop_cnt = 0
    led.value(1)

    try:
        while True:
            t_now = time.ticks_ms()

            # ── UWB 串口防积压：每隔 50ms 步进一次，防止长期不读导致 uart_bytes=255 超时 ──
            if time.ticks_diff(t_now, last_uwb_step_ms) >= 50:
                if _uwb_shared is not None:
                    _uwb_shared.step()
                last_uwb_step_ms = t_now
            dt_act = time.ticks_diff(t_now, t_prev) * 0.001
            t_prev = t_now
            if dt_act <= 0 or dt_act > 0.1:
                dt_act = 0.01

            # ── 1. 整段最大时间硬超时拦截 ──
            if time.ticks_diff(t_now, start_time_ms) / 1000.0 > MAX_APPROACH_TIME_S:
                print("  [APPROACH] 达到单次靠近时限 ({:.1f}s)，物理停车并退出".format(MAX_APPROACH_TIME_S))
                stop_all()
                break

            cam_data = recv.read()
            wz = _heading_correction(target_heading, dt=dt_act)
            has_tgt = False
            x_cm = 0.0
            dist_cm = 0.0

            # 💡 核心修复：检查当前周期是否接收到真正的串口新数据包
            new_frame_arrived = (cam_data is not None)

            if new_frame_arrived:
                has_tgt = cam_data['is_target']
                if has_tgt:
                    x_cm = x_to_cm(cam_data['x'])
                    dist_cm = y_to_distance(cam_data['y'])
                    last_x_cm = x_cm
                    last_dist_cm = dist_cm
                    last_has_tgt = True
                    last_target_seen_ms = t_now  # 重置丢标安全时钟
                elif last_has_tgt:
                    x_cm = last_x_cm
                    dist_cm = last_dist_cm
            else:
                if last_has_tgt:
                    x_cm = last_x_cm
                    dist_cm = last_dist_cm

            # ── 2. 丢标安全超时拦截 ──
            if time.ticks_diff(t_now, last_target_seen_ms) > LOST_TARGET_TIMEOUT_MS:
                print("  [APPROACH] 目标丢失超时 ({:.1f}s)，安全断开退出".format(LOST_TARGET_TIMEOUT_MS / 1000.0))
                stop_all()
                break

            # 保持真实的 has_tgt 输入，让控制器内部 of 500ms 容忍时钟正常工作
            vx, vy, wz_out, is_aligned, dt_step = fc.step(
                x_cm, dist_cm, has_tgt, wz_in=wz, now_ms=t_now)

            # ── 3. 🔴 竞速优化：给控制器输出乘以 1.35 倍增益，并进行安全最高限幅 ──
            vx_boosted = vx * 1.35
            vy_boosted = vy * 1.35
            vx_boosted = max(-0.50, min(vx_boosted, 0.50))
            vy_boosted = max(-0.50, min(vy_boosted, 0.50))

            if abs(vx_boosted) > 0.001 or abs(vy_boosted) > 0.001:
                # 【纵向纯净位移积分】：累加实际使用的 vx_boosted 移动量
                step_forward = vx_boosted * dt_step
                _approach_forward_dist += step_forward
                if _approach_forward_dist < 0.0:
                    _approach_forward_dist = 0.0

                rc = get_encoder_counts()
                if rc is not None and len(rc) >= 4:
                    speeds = _get_local_filtered_speeds(rc, dt_step)
                    # 采用高增益后的 vx_boosted 和 vy_boosted 进行底盘控制，缩短过渡期
                    omni_drive_closed_loop(vx_boosted, vy_boosted, wz_out, speeds, dt_step)
                else:
                    stop_all()
            else:
                # 丢包或已对齐停靠：立即切断纵向与横向驱动，仅维持纯原地航向锁定
                if abs(wz) > 0.001:
                    rc = get_encoder_counts()
                    if rc is not None and len(rc) >= 4:
                        speeds = _get_local_filtered_speeds(rc, dt_step)
                        omni_drive_closed_loop(0, 0, wz, speeds, dt_step)
                    else:
                        stop_all()
                else:
                    stop_all()

            # ── 4. 优化后的精准停靠判定逻辑 ──
            if new_frame_arrived and has_tgt:
                if is_aligned:
                    align_consecutive_count += 1
                else:
                    align_consecutive_count = 0

            # 💡 双保险判据：
            # 保险 A：在相机新帧更新的前提下，连续 3 次确认对齐（稳定维持约 100ms）
            # 保险 B：将收敛阈值从原先的 0.03m/s 放宽到 0.08m/s (8cm/s)。
            # 这能瞬间切除末端极慢的"爬行尾巴"，至少能节省 3-4 秒的调整时间，且由于速度极低，惯性对角度精度无影响。
            speed_converged = is_aligned and abs(vx_boosted) < 0.08 and abs(vy_boosted) < 0.08

            if align_consecutive_count >= 3 or speed_converged:
                stop_all()
                time.sleep_ms(80)  # 原地物理刹车释能，消解余震
                _encoder_reset()

                print("\n  [APPROACH] 目标精准停靠确认 ({})！".format(
                    "连续真实帧对齐" if align_consecutive_count >= 3 else "输出速度收敛"))
                arrived_success = True
                break

            if _abort_check():
                print("\n  [APPROACH] SW2 手动跳过对齐，执行短距试探前进...")
                stop_all()
                time.sleep_ms(100)
                _drive_closed_loop(0.35, 0, 0.20, check_target=False, timeout_s=3.0, label="APPROACH_SKIP_FWD")
                arrived_success = True
                break

            elap = time.ticks_diff(time.ticks_ms(), t_now)
            if elap < LOOP_MS:
                time.sleep_ms(LOOP_MS - elap)
            loop_cnt += 1
            if loop_cnt % 50 == 0:
                gc.collect()
    except Exception as e:
        print("\n  [APPROACH] 视觉对齐靠近执行异常:", e)
        stop_all()
    finally:
        led.value(0)

    if not arrived_success:
        return False

    # ── 测试模式下，直接绕过后面的从车蓝牙通信部分 ──
    if RUN_MODE == "test":
        print("  [APPROACH] (TEST模式下) 绕过从车蓝牙信号发送与等待。")
        return True

    if check_sw2():
        print("  [APPROACH] 检测到 SW2 手动中断，强制跳过蓝牙等待")
        return True
    print("  [APPROACH] 向从车发送数字 0 (turn_right)...")
    retry_count = 0
    try:
        # 💡 强力清除方案：持续循环读取 300ms，把所有积压数据彻底读空并喂狗
        clear_start = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), clear_start) < 300:
            pet_watchdog()
            bt.read_response()
            time.sleep_ms(10)

        bt.turn_right()
        print("  [APPROACH] 指令已发，等待从车 ok（超时 3s 后自动继续）...")
        bt_wait_start = time.ticks_ms()
        while True:
            pet_watchdog()
            _maintain_yaw(target_heading)
            if time.ticks_diff(time.ticks_ms(), bt_wait_start) > 3000:
                print("  [APPROACH] 等待从车 ok 超时 (3s)，直接继续执行")
                break
            if check_sw2():
                print("  [APPROACH] SW2 中断，跳过蓝牙等待")
                break
            resp = bt.read_response()
            if resp == "ok":
                print("  [APPROACH] 从车确认完毕 (ok)")
                break
            time.sleep_ms(10)
    except Exception as e:
        print("  [APPROACH] 蓝牙通信发生异常:", e)
        return False
    return True

# ═══════════════════════════════════════════════════════════════
#  全速前进至 UWB X 距离 < -150cm
# ═══════════════════════════════════════════════════════════════

def _action_forward_until_uwb_x():
    """直行全速推进。推进期间临时倍增 P、I、D 参数并松绑纠偏上限，
    配合竞速级安全阈值抗打滑策略（TCS），在不损失竞速效率的前提下极力压制初始漂移峰值。"""
    global _approach_forward_dist, _local_prev_speeds, _local_speeds_initialized
    print("\n  [UWBX] === 开始过线推进 ===")
    
    bt.send_direction('B', STARTUP_FULL_SPEED)  # 前进
    
    uwb = _ensure_uwb()
    if uwb is None:
        print("  [UWBX] 警告：UWB 初始不可用，将在前行移动中建立连接...")

    target_heading = _lock_yaw()
    
    # ── 🔴 穿透 hold 对象，直接定位到内部的 pid 控制器实例 ──
    hold = _get_heading_hold()
    pid = hold._pid
    
    # 备份原始参数以便 finally 块还原
    orig_kp = pid.kp
    orig_ki = pid.ki
    orig_kd = pid.kd if hasattr(pid, 'kd') else 0.0
    orig_i_limit = pid.integral_limit
    orig_wz_max = pid.output_limit

    # ── 临时"极限竞速"参数调整 ──
    pid.kp = orig_kp * 1.8                  # 🔴 核心优化1：比例项放大1.8倍，极大缩短反应延迟，压制初始漂移峰值
    pid.ki = orig_ki * 4.0 if orig_ki > 0.0 else 0.02
    if hasattr(pid, 'kd') and orig_kd > 0.0:
        pid.kd = orig_kd * 1.5              # 🔴 核心优化2：微分项同步放大1.5倍，提供强力物理阻尼，防止纠偏过冲
        
    pid.integral_limit = orig_i_limit * 5.0
    pid.output_limit = 0.85                 # wz纠偏输出限幅松绑到 0.85（最大动用 85% 电机功率转向纠偏）

    print("  [UWBX] 竞速纠偏解限: Kp={:.2f}→{:.2f}, Ki={:.4f}→{:.4f}, I_LIMIT={:.3f}→{:.3f}, WZ_MAX={:.3f}→{:.3f}".format(
        orig_kp, pid.kp, orig_ki, pid.ki, orig_i_limit, pid.integral_limit, orig_wz_max, pid.output_limit))

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    last_uwb_ms = start_ms
    loop_cnt = 0
    last_loop_ms = start_ms
    uwb_dead_start = 0
    uwb_dead_reconnect = 0
    cam = _ensure_cam()
    cam.reset()
    armed_to_trigger = False
    yellow_lost_count = 0
    BLIND_TOLERANCE = 3
    last_cam_ms = start_ms
    ctrl = None
    led.value(1)

    # 安全里程积分器
    total_fwd_dist_m = 0.0

    # ── 使用 try...finally 安全闭环包裹，保障参数在任何出口无损恢复 ──
    try:
        while True:
            now_ms = time.ticks_ms()
            if _abort_check():
                led.value(0)
                return False
            elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            if elapsed > UWB_X_TIMEOUT_S:
                print("  [UWBX] 超时 ({:.1f}s)，停止推进。".format(elapsed))
                led.value(0)
                return False

            counts = get_encoder_counts()
            if counts is None or len(counts) < 4:
                time.sleep_ms(5)
                continue
            actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
            if actual_dt <= 0.001 or actual_dt > 3 * FWD_CTRL_DT:
                actual_dt = FWD_CTRL_DT
                for i in range(4):
                    _local_prev_speeds[i] = 0.0
                _local_speeds_initialized = False
            last_loop_ms = now_ms

            # 绝对值累加里程：由于是 vy=0 的纯直行，绝对值累加能高精度准确反映直行里程。
            step_forward = sum(abs(counts[i] / ENC_SCALE[i]) for i in range(4)) / 4.0
            if step_forward > 0:
                _approach_forward_dist += step_forward
                total_fwd_dist_m += step_forward

            # ── 通用安全物理最长防护距离 (1.5m) ──
            if total_fwd_dist_m >= 1.5:
                stop_all()
                led.value(0)
                print("  [UWBX] 已达到最长物理防护距离 (1.5m)，触发停车。")
                return True

            now = time.ticks_ms()
            if uwb is not None and time.ticks_diff(now, last_uwb_ms) >= 50:
                uwb.step()
                last_uwb_ms = now
            if time.ticks_diff(now, last_cam_ms) >= 50:
                ctrl = cam.step()
                last_cam_ms = now
            line_detected = ctrl.get('line_flag', 0) if ctrl else 0
            if line_detected:
                armed_to_trigger = True
                yellow_lost_count = 0
            elif armed_to_trigger:
                yellow_lost_count += 1
            if armed_to_trigger and yellow_lost_count >= BLIND_TOLERANCE:
                stop_all()
                print("  [UWBX] 黄线越界确认 (连续{}帧) → 触发停车！".format(BLIND_TOLERANCE))
                return True

            uwb_is_active = True
            if uwb is None or uwb.is_timeout():
                uwb_is_active = False
                if uwb_dead_start == 0:
                    uwb_dead_start = time.ticks_ms()
                    print("  [UWBX] UWB 数据中断，车身保持安全巡航速度并于移动中尝试重连...")
                elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > UWB_DEAD_TIMEOUT_S * 1000:
                    if uwb is not None and uwb.is_uart_alive():
                        print("  [UWBX] UART 仍活跃，延长移动等待 (帧过滤中)...")
                        uwb_dead_start = time.ticks_ms()
                    elif uwb_dead_reconnect < UWB_DEAD_RECONNECT_MAX:
                        uwb_dead_reconnect += 1
                        print("  [UWBX] UART 离线，移动中进行第 {}/{} 次硬件重连重建...".format(
                            uwb_dead_reconnect, UWB_DEAD_RECONNECT_MAX))
                        _reset_uwb_if_needed()
                        new_uwb = _ensure_uwb()
                        if new_uwb is not None:
                            uwb = new_uwb
                            uwb_dead_start = 0
                            last_uwb_ms = time.ticks_ms()
                            print("  [UWB] 移动中重连建立成功！")
                            uwb_is_active = True
                            continue
                        uwb_dead_start = time.ticks_ms()
                    else:
                        print("  [UWBX] UWB 彻底离线，已进入视觉托管状态，继续安全车速行进...")
                        uwb_dead_start = time.ticks_ms()
            else:
                if uwb_dead_start != 0:
                    print("  [UWBX] UWB 链路已自主恢复")
                uwb_dead_start = 0
                uwb_dead_reconnect = 0

            fwd_speed = UWB_X_MIN_SPEED
            if uwb_is_active:
                try:
                    x_cm, y_cm = uwb.get_position()
                    if x_cm < UWB_X_THRESHOLD_CM:
                        stop_all()
                        print("  [UWBX] UWB X={:.1f}cm < {:.0f}cm，坐标触发停车！".format(
                            x_cm, UWB_X_THRESHOLD_CM))
                        return True
                    if x_cm < UWB_X_SLOWDOWN_CM:
                        slowdown_range = UWB_X_SLOWDOWN_CM - UWB_X_THRESHOLD_CM
                        ratio = (UWB_X_SLOWDOWN_CM - x_cm) / slowdown_range
                        ratio = max(0.0, min(1.0, ratio))
                        fwd_speed = STARTUP_FULL_SPEED - ratio * (STARTUP_FULL_SPEED - UWB_X_MIN_SPEED)
                    else:
                        fwd_speed = STARTUP_FULL_SPEED
                except Exception as e:
                    print("  [UWBX] get_position() 读取异常:", e)
                    fwd_speed = UWB_X_MIN_SPEED
            else:
                fwd_speed = 0.35

            # ── 🔴 核心优化3：竞速级"安全阈值抗滑移"控滑策略 ──
            # 在跑偏 10° 以内时，100% 满功率竞速推进，绝不降速！
            # 仅在偏航角突破 10° 警戒线时，才启动极轻微的前进速度衰减（最低限制到 40% 蠕动），给 wz 释放物理抓地力
            # 一旦纠偏拉回到 10° 以内，底盘立刻重新爆发 100% 全速推进，完美保障竞速时间！
            _read_imu_update_yaw()
            yaw_err = abs((target_heading - _yaw() + 180) % 360 - 180)
            if yaw_err > 10.0:
                fwd_scale = max(0.40, min(1.0, 1.0 - ((yaw_err - 10.0) / 20.0)))
                fwd_speed = fwd_speed * fwd_scale

            if time.ticks_diff(now, last_print_ms) >= 500:
                last_print_ms = now
                st = "HEALTHY" if uwb_is_active else "DROPPED(RUNNING)"
                print("  [UWBX] UWB_Link={} v={:.2f}m/s yaw={:.2f}° t={:.1f}s dist={:.2f}m".format(
                    st, fwd_speed, _yaw(), elapsed, total_fwd_dist_m))
            loop_cnt += 1
            if loop_cnt % 50 == 0:
                gc.collect()
            wz = _heading_correction(target_heading, dt=actual_dt)
            try:
                speeds = _get_local_filtered_speeds(counts, actual_dt)
                omni_drive_closed_loop(fwd_speed, 0, wz, speeds, actual_dt)
            except Exception as e:
                print("  [UWBX] 驱动运行错误:", e)
            time.sleep_ms(int(FWD_CTRL_DT * 1000))
    finally:
        # ── 🔴 还原原始参数，并将推重物累积的饱和积分及微分缓存彻底复位 ──
        pid.kp = orig_kp
        pid.ki = orig_ki
        if hasattr(pid, 'kd'):
            pid.kd = orig_kd
        pid.integral_limit = orig_i_limit
        pid.output_limit = orig_wz_max
        pid.reset()  
        bt.send_direction('exit')
        print("  [UWBX] 纠偏参数已安全复原，积分及微分缓存已清空。")


# ═══════════════════════════════════════════════════════════════
#  返回原点: UWB 导航到 origin 坐标
# ═══════════════════════════════════════════════════════════════

def _action_return_to_origin():
    global origin, _origin_return_offset
    uwb = _ensure_uwb()
    if uwb is None:
        print("  [RTN] UWB 不可用")
        return False
    if origin is None:
        print("  [RTN] origin 坐标未记录")
        return False
    target_x = origin[0] + _origin_return_offset
    target_y = origin[1]
    print("  [RTN] 返航目标: ({:.1f}, {:.1f})  X偏移 +{:.0f}cm".format(target_x, target_y, _origin_return_offset))
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    def _lock_fn(): return _lock_yaw()
    def _wz_fn(target, db): return _heading_correction(target, deadband=db, dt=ORIGIN_CTRL_DT)
    def _yaw_fn(): return _yaw()
    def _abort_fn():
        pet_watchdog()
        if check_sw2():
            print("  [SW2] Abort requested")
            return True
        return False
    def _drive_fn(vx, vy, wz, dt):
        rc = get_encoder_counts()
        if rc is not None and len(rc) >= 4:
            speeds = _get_local_filtered_speeds(rc, dt)
            omni_drive_closed_loop(vx, vy, wz, speeds, dt)
    def _stop_fn(): stop_all()
    def _led_fn(val): led.value(val)

    arrived, reason = goto_location(
        uwb, target_x, target_y,
        _lock_fn, _wz_fn, _yaw_fn,
        _abort_fn, _drive_fn, _stop_fn,
        _led_fn, label="RTN"
    )
    if arrived:
        _origin_return_offset += 10.0
        print("  [RTN] 到达，下次返航 X 偏移累计: +{:.0f}cm".format(_origin_return_offset))
    return arrived


# ═══════════════════════════════════════════════════════════════
#  C14 步骤 2: UWB 平移（航向保持）
# ═══════════════════════════════════════════════════════════════

def _action_uwb_translate():
    print("  [UWB] Starting translate toward anchor...")
    target_heading = _lock_yaw()
    print("  [UWB] Heading locked: {:.1f}°".format(target_heading))
    uwb = _ensure_uwb()
    if uwb is None:
        print("  [UWB] 警告：UWB 初始不可用，将在前行移动中建立连接...")
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    last_loop_ms = start_ms
    uwb_dead_start = 0
    uwb_dead_reconnect = 0
    led.value(1)
    last_fwd_speed = 0.0

    while True:
        now_ms = time.ticks_ms()
        if _abort_check():
            led.value(0)
            return False
        elapsed = time.ticks_diff(now_ms, start_ms) / 1000.0
        if elapsed > UWB_TIMEOUT_S:
            print("  [UWB] 平移超时 ({:.1f}s)，结束本次平移动作".format(elapsed))
            led.value(0)
            return True
        if uwb is not None:
            uwb.step()
            try:
                x_cm, y_cm = uwb.get_position()
            except Exception as e:
                print("  [UWB] get_position() 异常:", e)
                _maintain_yaw(target_heading)
                time.sleep_ms(50)
                continue
        uwb_is_active = True
        if uwb is None or uwb.is_timeout():
            uwb_is_active = False
            if uwb_dead_start == 0:
                uwb_dead_start = time.ticks_ms()
                print("  [UWB] 数据中断，进行姿态锁定重连...")
            elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > UWB_DEAD_TIMEOUT_S * 1000:
                if uwb is not None and uwb.is_uart_alive():
                    print("  [UWB] UART 活跃，延长等待中...")
                    uwb_dead_start = time.ticks_ms()
                elif uwb_dead_reconnect < UWB_DEAD_RECONNECT_MAX:
                    uwb_dead_reconnect += 1
                    print("  [UWB] UART 掉线，第 {}/{} 次移动中硬件复位重建...".format(
                        uwb_dead_reconnect, UWB_DEAD_RECONNECT_MAX))
                    _reset_uwb_if_needed()
                    new_uwb = _ensure_uwb()
                    if new_uwb is not None:
                        uwb = new_uwb
                        uwb_dead_start = 0
                        print("  [UWB] UWB 移动中重连成功！")
                        uwb_is_active = True
                    else:
                        uwb_dead_start = time.ticks_ms()
                else:
                    print("  [UWB] 硬件重连均失败，以极低平移惯性盲走靠拢...")
                    uwb_dead_start = time.ticks_ms()
        else:
            uwb_dead_start = 0
            uwb_dead_reconnect = 0
        if uwb_is_active:
            if abs(x_cm) < UWB_X_DEADBAND:
                print("  [UWB] Centered! X={:.1f}cm (deadband={:.1f}cm)".format(
                     x_cm, UWB_X_DEADBAND))
                led.value(0)
                return True
            fwd_speed = x_cm * UWB_LAT_P_GAIN
            fwd_speed = max(-UWB_LAT_SPEED, min(fwd_speed, UWB_LAT_SPEED))
            last_fwd_speed = fwd_speed
        else:
            fwd_speed = last_fwd_speed * 0.90
        if time.ticks_diff(time.ticks_ms(), last_print_ms) >= 500:
            last_print_ms = time.ticks_ms()
            if uwb_is_active:
                try:
                    print("  [UWB] X={:+.1f}cm Y={:.1f}cm yaw={:.2f}° spd={:.2f}".format(
                        x_cm, y_cm, _yaw(), fwd_speed))
                except Exception:
                    pass
        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001 or actual_dt > 3 * UWB_CTRL_DT:
            actual_dt = UWB_CTRL_DT
            global _local_prev_speeds, _local_speeds_initialized
            _local_prev_speeds = [0.0, 0.0, 0.0, 0.0]
            _local_speeds_initialized = False
        last_loop_ms = now_ms
        wz = _heading_correction(target_heading, dt=actual_dt)
        try:
            speeds = _get_local_filtered_speeds(counts, actual_dt)
            omni_drive_closed_loop(fwd_speed, 0, wz, speeds, actual_dt)
        except Exception as e:
            print("  [UWB] 驱动错误:", e)
        time.sleep_ms(int(UWB_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  move_toward_fixed_point: 直线到达目标点，IMU 航向锁定
# ═══════════════════════════════════════════════════════════════

def move_toward_fixed_point():
    """直线斜向到达目标点 (前55cm, 右60cm)，全程 IMU 航向角锁定"""
    import math
    fwd_m  = MFP_FORWARD_DIST_CM / 100.0   # 0.55
    right_m = MFP_RIGHT_DIST_CM / 100.0    # 0.60
    diag_m = math.sqrt(fwd_m**2 + right_m**2)

    # 按位移比例分配速度分量，总速率 = MFP_SPEED
    vx = MFP_SPEED * fwd_m  / diag_m
    vy = MFP_SPEED * right_m / diag_m

    print("\n" + "=" * 50)
    print("[MFP] 斜线直达: 前{:.0f}cm + 右{:.0f}cm  对角{:.1f}cm（航向锁定）"
          .format(MFP_FORWARD_DIST_CM, MFP_RIGHT_DIST_CM, diag_m * 100))
    print("=" * 50)

    pause_encoder_ticker()
    _encoder_reset()

    _drive_closed_loop(vx, vy, diag_m,
                       check_target=False,
                       timeout_s=MFP_TIMEOUT_S,
                       label="MFP")

    print("[MFP] ✓ 完成")
    print("=" * 50 + "\n")
    return True




def _action_forward_20cm():
    _, dist = _drive_closed_loop(FORWARD_SPEED, 0, FORWARD_DIST_M, check_target=False, label="FWD")
    return dist >= FORWARD_DIST_M


# ═══════════════════════════════════════════════════════════════
#  按键测试模式 main
# ═══════════════════════════════════════════════════════════════

def main():
    print("\n  [TEST_MODE] 启动键盘调度测试，等待按键按下...")

    _system_init()

    while True:
        capture()
        pet_watchdog()

        # ──────────────────────────────────────────────────────────
        # KEY1 (C8) 触发：S形扫描物资并推离 (全模式统一 UWB 闭环 + 蓝牙由 see_and_push 内自动过滤)
        # ──────────────────────────────────────────────────────────
        if key_triggered(1):
            print("\n  [TEST_MODE] KEY1 (C8) 被按下。开始执行 S形搜索 ➔ 推出 ➔ 倒车复位闭环流程...")
            try:
                global _search_progress_cm, _approach_forward_dist, _just_pushed, supplies
                _search_progress_cm = 0.0
                _approach_forward_dist = 0.0
                _just_pushed = False
                supplies = None

                while True:
                    pause_encoder_ticker()
                    _pause_with_yaw_hold(_TARGET_HEADING, 300)
                    _encoder_reset()

                    # 1. 启动 S 曲线扫描，寻找物资
                    item_found = _action_s_pattern_search()

                    if check_sw2():
                        print("  [TEST_MODE] SW2 介入打断，正在返航...")
                        _action_return_to_origin()
                        break

                    # 扫描无果代表全部区域搜索完毕，UWB 归巢返航
                    if not item_found:
                        print("  [TEST_MODE] S型横向搜索全部完成且未发现更多物资，正在返航...")
                        _action_return_to_origin()
                        break

                    _approach_forward_dist = 0.0
                    _encoder_reset()

                    # 2. 视觉对准靠近 (see_and_push 内自动由于 RUN_MODE="test" 过滤掉蓝牙动作)
                    if not see_and_push():
                        print("  [TEST_MODE] 视觉靠近中断，正在退回到搜索轨道线...")
                        _reverse_to_rightward_path()
                        continue

                    _pause_with_yaw_hold(_TARGET_HEADING, 300)
                    _encoder_reset()

                    # 3. 越黄线推进 (全模式统一 UWB 坐标 + 视觉黄线复合守护)
                    if not _action_forward_until_uwb_x():
                        print("  [TEST_MODE] 推进中断，正在退回到搜索轨道线...")
                        _reverse_to_rightward_path()
                        continue

                    _pause_with_yaw_hold(_TARGET_HEADING, 300)
                    _encoder_reset()

                    # 4. 高精度倒车退回到 S 轴向扫描线上
                    _reverse_to_rightward_path()
                    
                    # 激活起步避盲区保护
                    _just_pushed = True

                    _pause_with_yaw_hold(_TARGET_HEADING, 300)
                    _encoder_reset()
                    print("  [TEST_MODE] 已返回扫描轨，断点：{:.1f}cm。继续执行下一次扫描...".format(_search_progress_cm))

            except Exception as e:
                print("  [TEST_MODE] 执行 S形取物工作流发生异常:", e)
            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()
            continue

        if key_triggered(2):
            print("\n  [TEST_MODE] KEY2 (C9) 触发: 原始带通信的 S 曲线测试")
            try:
                global _search_progress_cm, _approach_forward_dist, _just_pushed, supplies
                _search_progress_cm = 0.0
                _approach_forward_dist = 0.0
                _just_pushed = False
                supplies = None
                pause_encoder_ticker()
                _encoder_reset()
                found = _action_s_pattern_search()
                if found:
                    print("  [TEST_MODE] 发现目标，开始对齐靠近...")
                    _encoder_reset()
                    arrived = see_and_push()
                    if arrived:
                        _pause_with_yaw_hold(_TARGET_HEADING, 300)
                        _encoder_reset()
                        _action_forward_until_uwb_x()
                        _reverse_to_rightward_path()
                        _just_pushed = True
                    print("  [TEST_MODE] 靠近并传输完成:", arrived)
                else:
                    print("  [TEST_MODE] 未搜索到目标")
            except Exception as e:
                print("  [TEST_MODE] 发生异常:", e)
            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()
            continue

        if check_sw2():
            print("  [TEST_MODE] SW2 被按下，即将退出测试流程...")
            break

        time.sleep_ms(10)

    _system_cleanup()


# ═══════════════════════════════════════════════════════════════
#  安全返航停机
# ═══════════════════════════════════════════════════════════════

def _safe_return_and_exit():
    print("\n  [RTN] 安全回原点...")
    if _TARGET_HEADING is None:
        _lock_yaw()
    _pause_with_yaw_hold(_TARGET_HEADING, 300)
    _encoder_reset()
    _action_return_to_origin()
    _system_cleanup()
    print("\n[INFO] 已安全返航。")


# ═══════════════════════════════════════════════════════════════
#  系统入口
# ═══════════════════════════════════════════════════════════════

RUN_MODE = "loop"

if RUN_MODE == "loop":
    _system_init()

    try:
        # 第一阶段：推进固定路径
        move_toward_fixed_point()

        while True:
            pause_encoder_ticker()
            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()

            # 第二阶段：一轴 1D S 曲线横向扫描 (根据断点 _search_progress_cm 增量运行)
            item_found = _action_s_pattern_search()

            if check_sw2():
                print("  [MAIN_LOOP] SW2 介入打断，立即退出...")
                break

            if not item_found:
                print("\n  [MAIN_LOOP] S型扫描结束且未搜索到更多目标，UWB返航...")
                _action_return_to_origin()
                break

            _approach_forward_dist = 0.0
            _encoder_reset()

            # 第三阶段：视觉靠近，对准目标物
            if not see_and_push():
                print("  [MAIN_LOOP] 视觉靠近中断，正在无损退回到搜索轨道线...")
                _reverse_to_rightward_path()
                continue

            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()

            # 第四阶段：物理/坐标双重防护推进黄线过线
            if not _action_forward_until_uwb_x():
                print("  [MAIN_LOOP] 过线推进中断，正在无损退回到搜索轨道线...")
                _reverse_to_rightward_path()
                continue

            if check_sw2():
                print("  [MAIN_LOOP] SW2 手动旁路跳过蓝牙...")
            else:
                target_heading = _lock_yaw()
                retry_count = 0
                try:
                    # 等待 exit 指令的 "ok" 经蓝牙链路传回（50-120ms 物理延迟）
                    # 确保后续清空缓冲区时能一次性扫干净，防止误读为 turn_left 的 ok
                    time.sleep_ms(200)

                    print("  [MAIN_LOOP] 正在清空主车接收缓冲区...")
                    while bt.read_response() is not None:
                        time.sleep_ms(5)

                    while True:
                        bt.turn_left()
                        retry_count += 1
                        bt_wait_start = time.ticks_ms()
                        received_ok = False
                        sw2_skip = False
                        while True:
                            pet_watchdog()
                            _maintain_yaw(target_heading)
                            if time.ticks_diff(time.ticks_ms(), bt_wait_start) > BT_WAIT_DEADLINE_S * 1000:
                                break
                            if check_sw2():
                                sw2_skip = True
                                break
                            resp = bt.read_response()
                            if resp == "ok":
                                received_ok = True
                                break
                            time.sleep_ms(10)
                        if received_ok or sw2_skip:
                            break
                except Exception as e:
                    print("  [MAIN_LOOP] 蓝牙通信异常:", e)

            # 💡 收到从车 ok 后停车 2 秒，等待从车完成推物动作
            stop_all()
            time.sleep_ms(3000)

            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()

            # 第五阶段：按照整个过程的高精度累计前进偏移进行回退复位
            _reverse_to_rightward_path()
            
            # 激活起步避盲区保护
            _just_pushed = True

            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()
            print("  [MAIN_LOOP] 成功无损复位至 S 型轨道断点，继续执行下一次搜索...")

    except KeyboardInterrupt:
        print("  [MAIN_LOOP] 用户手动终止 (Ctrl+C)")
    finally:
        _system_cleanup()

elif RUN_MODE == "test":
    main()
    import sys; sys.exit()
    