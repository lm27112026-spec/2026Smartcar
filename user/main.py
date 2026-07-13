#  RT1021 — 按键驱动控制

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


# ═══════════════════════════════════════════════════════════════
#  常量区域
# ═══════════════════════════════════════════════════════════════

LED_PIN  = 'C4'
SW2_PIN  = 'D9'

# ── C14: 前进 20cm ──
FORWARD_DIST_M   = 0.20    # 目标距离 (m)
FORWARD_SPEED    = 0.50    # 前进速度 (m/s)
FWD_TIMEOUT_S    = 10.0    # 超时 (s)
FWD_CTRL_DT      = 0.02    # 控制周期 (s)

# ── C14: UWB 平移 ──
UWB_LAT_SPEED    = 0.50    # 最大平移速度 (m/s)
UWB_X_DEADBAND   = 3.0     # X 方向死区 (cm)

UWB_LAT_P_GAIN   = 0.02    # X 误差 → 平移速度 P 增益
UWB_TIMEOUT_S    = 10.0    # 超时 (s)
UWB_CTRL_DT      = 0.02    # 控制周期 (s)

_heading_hold = None        # HeadingHold 实例（首次 _lock_yaw 时懒初始化）

# ── 蓝牙信号 ──
BT_WAIT_DEADLINE_S       = 10.0     # 蓝牙单次等待超时 (s)，超时后重发消息重试，永不因蓝牙触发返航

# ── 启动: 收到 ok 后全速前进至 UWB X 距离 < -130cm ──
STARTUP_FULL_SPEED     = 1.00     # 全速前进速度（绝对值，m/s）
UWB_X_THRESHOLD_CM     = -130.0   # UWB X 轴距离阈值 (cm)，小于此值触发停车
UWB_X_SLOWDOWN_CM      = -80.0    # X < 此值时开始线性减速，防止冲出 UWB 覆盖
UWB_X_MIN_SPEED        = 0.25     # 接近阈值时的最低速度 (m/s)
UWB_X_TIMEOUT_S        = 15.0     # UWB X 距离检测超时 (s)
UWB_BACKUP_DIST_CM     = 20.0     # 触发后倒退距离 (cm)
UWB_BACKUP_SPEED       = 0.50     # 倒退速度 (m/s)
UWB_BACKUP_TIMEOUT_S   = 8.0      # 倒退超时 (s)
UWB_DEAD_TIMEOUT_S     = 2.0      # UWB 掉线容忍超时 (s)
UWB_DEAD_RECONNECT_MAX  = 4       # UWB 掉线最大重连次数

# ── 返航: 返回 origin 坐标 ──
ORIGIN_CTRL_DT      = 0.01    # 控制周期 (s)

# ── 倒车回到右移路径 ──
BACKUP_PATH_SPEED   = 0.50    # 倒车回到路径速度 (m/s)
BACKUP_PATH_TIMEOUT = 15.0    # 倒车回到路径超时 (s)

# ── move_toward_fixed_point: 前进 50cm(补偿后) → 右平移 110cm(补偿后) ──
MFP_FORWARD_DIST_CM  = 55.0    # 目标50cm 
MFP_RIGHT_DIST_CM    = 90.0   # 目标110cm - 20cm惯性 = 90.0cm
MFP_SPEED            = 0.50    # 行进速度 (m/s)
MFP_TIMEOUT_S        = 15.0    # 超时 (s)
MFP_CTRL_DT          = 0.02    # 控制周期 (s)

# ── _action_rightward_search: 右移 1.0m 搜索(补偿后) ──
RIGHTWARD_SEARCH_DIST_CM = 90.0   # 目标100cm - 10cm侧向惯性 = 90.0cm
RIGHTWARD_SPEED          = 0.40    # 右移速度 (m/s)
RIGHTWARD_TIMEOUT_S      = 20.0    # 单次右移超时 (s)
RIGHTWARD_CTRL_DT        = 0.02    # 控制周期 (s)

# ── SW2 ──
SW2_DEBOUNCE_MS  = 50

# ── UWB 坐标记录 ──
origin   = None    # 起点坐标 (x, y)
_uwb_shared = None  # 共享 UWBPosition 实例
_cam_shared = None  # 共享 CameraController 实例
_TARGET_HEADING = None  # 全程锁定的目标航向
_approach_forward_dist = 0.0   # 取物流程净前进距离 (m)
_rightward_cumulative  = 0.0   # 累计右移距离 (m)，跨多次调用持久化

# ── 本地一阶低通滤波器（单次读取编码器 + 滤波，杜绝双读隐患）──
_SPD_FILTER_ALPHA = 0.75
_local_prev_speeds = [0.0, 0.0, 0.0, 0.0]
_local_speeds_initialized = False

# ── 延迟导入占位符：消除模块级 Pylance 报错，实际值由 _system_init() 注入 ──
CameraController = None
goto_location = None


def _get_local_filtered_speeds(raw_counts, dt):
    """基于外部传入的 counts 数组计算一阶低通滤波速度 [rf, lf, lb, rb] (m/s)。
    不调用 get_encoder_counts()，杜绝二次读取导致增量清零。
    🟢 内存零分配：原地修改 _local_prev_speeds，不创建新 list"""
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
    """
    🟢 程序退出前将全局命名空间中所有自定义值置 None。
    只保留 Thonny 必需的 __name__ / __file__ 和 stdlib 模块，
    其余全部清空——杜绝任何 repr() 触发 C 扩展序列化报错。
    """
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
    global origin, CameraController, goto_location, _TARGET_HEADING
    print("\n[SYSTEM] 执行统一系统状态初始化...")

    pause_encoder_ticker()
    _encoder_reset()

    start_watchdog()
    print("  [INIT] 独立硬件看门狗已启用 (3秒超时)")

    _lock_yaw()

    gc.collect()
    import cam_control
    CameraController = cam_control.CameraController
    import uwb_control
    goto_location = uwb_control.goto_location
    gc.collect()
    print("  [INIT] 闭环控制模块加载成功")

    uwb = _ensure_uwb()
    if uwb is not None and uwb.get_frame_count() > 0:
        origin = uwb.get_position()
        print("  [INIT] UWB 起点坐标已记录: ({:.1f}, {:.1f})".format(origin[0], origin[1]))
    else:
        print("  [INIT] UWB 未就绪，使用默认坐标")
        origin = (130.0, 262.0)

    print("[SYSTEM] 初始化就绪，空闲内存: {} 字节\n".format(gc.mem_free()))


def _system_cleanup():
    """统一资源释放：无论何种因由退出，执行一致的物理归档"""
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


def _heading_correction(target_yaw, deadband=1.0, dt=None):
    if dt is None:
        dt = FWD_CTRL_DT
    if deadband is None:
        deadband = 1.0
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
    """单次 yaw 保持迭代：应用层角度环回强校验 + 委托 HeadingHold → 驱动电机。
    返回: True=已驱动/已在死区, False=驱动异常
    """
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
#  通用前进函数
# ═══════════════════════════════════════════════════════════════

def _forward_distance(dist_m, speed, timeout_s, heading_deadband=None, label="FWD"):
    print("  [{}] 前进 {:.0f}cm 开始...".format(label, dist_m * 100))
    target_heading = _lock_yaw()
    db_str = "  死区 {:.0f}°".format(heading_deadband) if heading_deadband is not None else ""
    print("  [{}] 航向锁定: {:.1f}°{}".format(label, target_heading, db_str))

    total_wheel_dists = [0.0, 0.0, 0.0, 0.0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    last_loop_ms = start_ms
    led.value(1)

    while True:
        now_ms = time.ticks_ms()
        if _abort_check():
            led.value(0)
            return False
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > timeout_s:
            print("  [{}] 超时 ({:.1f}s)".format(label, elapsed))
            led.value(0)
            return False
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001 or actual_dt > 3 * FWD_CTRL_DT:
            actual_dt = FWD_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms
        for i in range(4):
            if ENC_SCALE[i] != 0:
                total_wheel_dists[i] += abs(counts[i]) / abs(ENC_SCALE[i])
        avg_dist = sum(total_wheel_dists) / len(total_wheel_dists)
        if avg_dist >= dist_m:
            print("  [{}] 到达目标！dist={:.2f}m".format(label, avg_dist))
            led.value(0)
            return True
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [{}] dist={:.2f}m / {:.2f}m  yaw={:.2f}°".format(
                label, avg_dist, dist_m, _yaw()))
        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()
        wz = _heading_correction(target_heading, deadband=heading_deadband, dt=actual_dt)
        try:
            speeds = _get_local_filtered_speeds(counts, actual_dt)
            omni_drive_closed_loop(speed, 0, wz, speeds, actual_dt)
        except Exception as e:
            print("  [{}] 驱动错误:".format(label), e)
        time.sleep_ms(int(FWD_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  辅助倒退执行闭环函数
# ═══════════════════════════════════════════════════════════════

def _execute_backup(target_heading):
    """执行倒退 UWB_BACKUP_DIST_CM 厘米的闭环辅助函数"""
    print("  [BACKUP] 开始倒退 {:.0f}cm...".format(UWB_BACKUP_DIST_CM))
    backup_dist_m = UWB_BACKUP_DIST_CM / 100.0
    backup_dists = [0.0, 0.0, 0.0, 0.0]
    backup_start_ms = time.ticks_ms()
    last_loop_ms = backup_start_ms

    while True:
        now_ms = time.ticks_ms()
        if _abort_check():
            return
        b_elapsed = time.ticks_diff(time.ticks_ms(), backup_start_ms) / 1000.0
        if b_elapsed > UWB_BACKUP_TIMEOUT_S:
            print("  [BACKUP] 倒退超时 ({:.1f}s)，放弃倒退".format(b_elapsed))
            break
        bcounts = get_encoder_counts()
        if bcounts is None or len(bcounts) < 4:
            time.sleep_ms(5)
            continue
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001 or actual_dt > 3 * FWD_CTRL_DT:
            actual_dt = FWD_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms
        for i in range(4):
            if ENC_SCALE[i] != 0:
                backup_dists[i] += abs(bcounts[i]) / abs(ENC_SCALE[i])
        avg_backup = sum(backup_dists) / len(backup_dists)
        if abs(avg_backup) >= backup_dist_m:
            print("  [BACKUP] 倒退完成 dist={:.2f}m".format(abs(avg_backup)))
            break
        wz_b = _heading_correction(target_heading, dt=actual_dt)
        try:
            speeds = _get_local_filtered_speeds(bcounts, actual_dt)
            omni_drive_closed_loop(-UWB_BACKUP_SPEED, 0, wz_b, speeds, actual_dt)
        except Exception as e:
            print("  [BACKUP] 驱动错误:", e)
        time.sleep_ms(int(FWD_CTRL_DT * 1000))

    stop_all()
    time.sleep_ms(300)
    _encoder_reset()


# ═══════════════════════════════════════════════════════════════
#  _action_rightward_search: 向右平移搜索，累计1.2m内逐帧摄像头检测
# ═══════════════════════════════════════════════════════════════

def _action_rightward_search():
    """向右平移搜索，累计1m内逐帧摄像头检测物品。

    使用全局 _rightward_cumulative 跟踪跨轮次累计右移距离。

    返回:
        True  = 发现物品（已 stop_all，_rightward_cumulative 已保存断点）
        False = 1m 完成无发现 / SW2 中断 / 超时
    """
    global _rightward_cumulative
    target_heading = _lock_yaw()
    print("  [RIGHT] 航向锁定: {:.1f}°".format(target_heading))
    print("  [RIGHT] 当前累计进度: {:.2f}m / {:.1f}cm".format(
        _rightward_cumulative, RIGHTWARD_SEARCH_DIST_CM))

    target_dist_m = RIGHTWARD_SEARCH_DIST_CM / 100.0
    if _rightward_cumulative >= target_dist_m:
        print("  [RIGHT] 警告：累计右移已达目标值")
        return False

    session_dists = [0.0, 0.0, 0.0, 0.0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    last_loop_ms = start_ms
    cam = _ensure_cam()
    led.value(1)

    while True:
        now_ms = time.ticks_ms()
        if _abort_check():
            stop_all()
            led.value(0)
            print("  [RIGHT] 检测到 SW2 手动中断")
            return False
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > RIGHTWARD_TIMEOUT_S:
            stop_all()
            led.value(0)
            print("  [RIGHT] 超时 ({:.1f}s)".format(elapsed))
            return False
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001 or actual_dt > 3 * RIGHTWARD_CTRL_DT:
            actual_dt = RIGHTWARD_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms
        for i in range(4):
            if ENC_SCALE[i] != 0:
                session_dists[i] += abs(counts[i]) / abs(ENC_SCALE[i])
        session_avg = sum(session_dists) / 4.0
        total_cumulative = _rightward_cumulative + session_avg
        if total_cumulative >= target_dist_m:
            _rightward_cumulative = total_cumulative
            stop_all()
            led.value(0)
            print("  [RIGHT] 达到 1.2m 搜索边界 (实际 {:.2f}m)，未检测到目标".format(
                total_cumulative))
            return False
        ctrl = cam.step()
        if ctrl and ctrl.get('has_target', False):
            _rightward_cumulative = total_cumulative
            stop_all()
            led.value(0)
            print("  [RIGHT] 检测到目标！保存断点：{:.2f}m".format(_rightward_cumulative))
            return True
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [RIGHT] cum={:.2f}m remain={:.1f}cm yaw={:.2f}°".format(
                total_cumulative,
                (RIGHTWARD_SEARCH_DIST_CM - total_cumulative * 100.0),
                _yaw()))
        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()
        wz = _heading_correction(target_heading, dt=actual_dt)
        try:
            speeds = _get_local_filtered_speeds(counts, actual_dt)
            omni_drive_closed_loop(0, RIGHTWARD_SPEED, wz, speeds, actual_dt)
        except Exception as e:
            print("  [RIGHT] 驱动异常:", e)
        time.sleep_ms(int(RIGHTWARD_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  _reverse_to_rightward_path: 倒车回到右移搜索路径线
# ═══════════════════════════════════════════════════════════════

def _reverse_to_rightward_path():
    """取物流程完成后，沿原路向后倒车回到右移搜索路径线上。

    读取 _approach_forward_dist（取物流程净前进距离，已扣除 _execute_backup 的 20cm），
    编码器里程计闭环倒车该距离，全程 IMU 航向保持。
    完成后重置 _approach_forward_dist = 0。
    """
    global _approach_forward_dist
    if _approach_forward_dist <= 0.001:
        print("  [REV] 前进距离 ≈ 0，无需倒车")
        return True
    target_heading = _lock_yaw()
    print("  [REV] 倒车回到右移路径，目标 {:.2f}m...".format(_approach_forward_dist))
    backup_dists = [0.0, 0.0, 0.0, 0.0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    last_loop_ms = start_ms

    while True:
        now_ms = time.ticks_ms()
        if _abort_check():
            stop_all()
            _approach_forward_dist = 0
            return False
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > BACKUP_PATH_TIMEOUT:
            print("  [REV] 倒车超时 ({:.1f}s)".format(elapsed))
            stop_all()
            _approach_forward_dist = 0
            return False
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001 or actual_dt > 3 * FWD_CTRL_DT:
            actual_dt = FWD_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms
        for i in range(4):
            if ENC_SCALE[i] != 0:
                backup_dists[i] += abs(counts[i]) / abs(ENC_SCALE[i])
        avg_backup = sum(backup_dists) / len(backup_dists)
        if avg_backup >= _approach_forward_dist:
            print("  [REV] 倒车完成 dist={:.2f}m".format(avg_backup))
            stop_all()
            break
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [REV] dist={:.2f}m / {:.2f}m yaw={:.2f}°".format(
                avg_backup, _approach_forward_dist, _yaw()))
        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()
        wz = _heading_correction(target_heading, dt=actual_dt)
        try:
            speeds = _get_local_filtered_speeds(counts, actual_dt)
            omni_drive_closed_loop(-BACKUP_PATH_SPEED, 0, wz, speeds, actual_dt)
        except Exception as e:
            print("  [REV] 驱动错误:", e)
        time.sleep_ms(int(FWD_CTRL_DT * 1000))

    _approach_forward_dist = 0
    _encoder_reset()
    return True


# ═══════════════════════════════════════════════════════════════
#  摄像头跟随靠近
# ═══════════════════════════════════════════════════════════════

def see_and_push():
    """
    完全复用 test_vision_track.py 的核心跟随控制机理（CamDataReceiver + FollowController），
    结合本地单次读取一阶低通滤波、动态时间步长与 8 帧对齐精准停靠判定。
    """
    print("\n  [APPROACH] === 视觉对齐靠近 ➔ 蓝牙同步流程 ===")
    cam = _ensure_cam()
    from cam_data import x_to_cm, y_to_distance
    from cam_control import FollowController
    recv = cam._recv
    fc = FollowController()

    target_heading = _lock_yaw()
    last_x_cm = 0.0
    last_dist_cm = 0.0
    last_has_tgt = False
    prev_vy = 0.0
    align_consecutive_count = 0
    arrived_success = False
    LOOP_MS = 10
    t_prev = time.ticks_ms()
    loop_cnt = 0
    led.value(1)

    try:
        while True:
            t_now = time.ticks_ms()
            dt_act = time.ticks_diff(t_now, t_prev) * 0.001
            t_prev = t_now
            if dt_act <= 0 or dt_act > 0.1:
                dt_act = 0.01
            cam_data = recv.read()
            wz = _heading_correction(target_heading, dt=dt_act)
            has_tgt = False
            x_cm = 0.0
            dist_cm = 0.0
            if cam_data is not None:
                has_tgt = cam_data['is_target']
                if has_tgt:
                    x_cm = x_to_cm(cam_data['x'])
                    dist_cm = y_to_distance(cam_data['y'])
                    last_x_cm = x_cm
                    last_dist_cm = dist_cm
                    last_has_tgt = True
                elif last_has_tgt:
                    x_cm = last_x_cm
                    dist_cm = last_dist_cm
            else:
                if last_has_tgt:
                    x_cm = last_x_cm
                    dist_cm = last_dist_cm
            vx, vy, wz_out, is_aligned, dt_step = fc.step(
                x_cm, dist_cm, has_tgt, wz_in=wz, now_ms=t_now)
            boost_vx = max(0.0, min(vx * 1.5, 0.80))
            smooth_vy = 0.6 * prev_vy + 0.4 * vy
            prev_vy = smooth_vy
            if boost_vx < 0.12:
                smooth_vy = smooth_vy * 0.5
                if abs(smooth_vy) < 0.03:
                    smooth_vy = 0.0
            rc = get_encoder_counts()
            if rc is not None and len(rc) >= 4:
                global _approach_forward_dist
                # 麦轮正解：带符号位移均值 = 纯纵向净前进，平移/自转分量自动抵消归零
                dist_rf = rc[0] / ENC_SCALE[0]
                dist_lf = rc[1] / ENC_SCALE[1]
                dist_lb = rc[2] / ENC_SCALE[2]
                dist_rb = rc[3] / ENC_SCALE[3]
                step_forward = (dist_rf + dist_lf + dist_lb + dist_rb) / 4.0
                if step_forward > 0:
                    _approach_forward_dist += step_forward
                speeds = _get_local_filtered_speeds(rc, dt_step)
                omni_drive_closed_loop(boost_vx, smooth_vy, wz_out, speeds, dt_step)
            else:
                stop_all()
            if is_aligned:
                align_consecutive_count += 1
            else:
                align_consecutive_count = 0
            if align_consecutive_count >= 8:
                stop_all()
                print("\n  [APPROACH] 目标精准对齐停靠完成 (连续 8 帧对齐确认)！")
                arrived_success = True
                break
            if _abort_check():
                print("\n  [APPROACH] SW2 手动跳过 → 直接全速前进。")
                stop_all()
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
    if check_sw2():
        print("  [APPROACH] 检测到 SW2 手动中断，强制跳过蓝牙等待")
        return True
    print("  [APPROACH] 向从车发送数字 0 (turn_right)...")
    retry_count = 0
    try:
        while True:
            bt.turn_right()
            retry_count += 1
            suffix = " (第{}次发送)".format(retry_count) if retry_count > 1 else ""
            print("  [APPROACH] 指令已发{}，等待从车 ok（单次超时 {}s）...".format(suffix, BT_WAIT_DEADLINE_S))
            bt_wait_start = time.ticks_ms()
            while True:
                pet_watchdog()
                _maintain_yaw(target_heading)
                if time.ticks_diff(time.ticks_ms(), bt_wait_start) > BT_WAIT_DEADLINE_S * 1000:
                    print("  [APPROACH] 等待从车 ok 超时 ({}s)，重新发送...".format(BT_WAIT_DEADLINE_S))
                    break
                if check_sw2():
                    print("  [APPROACH] SW2 中断，跳过蓝牙等待")
                    return True
                resp = bt.read_response()
                if resp == "ok":
                    print("  [APPROACH] 从车确认完毕 (ok)")
                    return True
                time.sleep_ms(10)
    except Exception as e:
        print("  [APPROACH] 蓝牙通信发生异常:", e)
        return False


# ═══════════════════════════════════════════════════════════════
#  全速前进至 UWB X 距离 < -130cm
# ═══════════════════════════════════════════════════════════════

def _action_forward_until_uwb_x():
    """
    全速前进，同时 UWB 判断 X 轴距离 或 摄像头识别黄线越界。
    🟢 核心修复：即使 UWB 掉线，小车也绝不停车或返回起点，而是降低到安全速度继续向前推进并尝试在移动中重连，
    同时完全依赖摄像头黄线检测作为绝对物理安全边界进行兜底拦截。
    """
    print("\n  [UWBX] === 全速前进，等待 UWB X < {:.0f}cm ===".format(UWB_X_THRESHOLD_CM))
    uwb = _ensure_uwb()
    if uwb is None:
        print("  [UWBX] 警告：UWB 初始不可用，将在前行移动中建立连接...")
    target_heading = _lock_yaw()
    print("  [UWBX] 航向锁定: {:.1f}°  开始直行推进".format(target_heading))
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

    while True:
        now_ms = time.ticks_ms()
        if _abort_check():
            led.value(0)
            return False
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > UWB_X_TIMEOUT_S:
            print("  [UWBX] 超时 ({:.1f}s)，UWB 未达标，仅依靠视觉黄线推进兜底".format(elapsed))
            pass
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001 or actual_dt > 3 * FWD_CTRL_DT:
            actual_dt = FWD_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms
        global _approach_forward_dist
        # 麦轮正解：带符号位移均值 = 纯纵向净前进，平移/自转分量自动抵消归零
        dist_rf = counts[0] / ENC_SCALE[0]
        dist_lf = counts[1] / ENC_SCALE[1]
        dist_lb = counts[2] / ENC_SCALE[2]
        dist_rb = counts[3] / ENC_SCALE[3]
        step_forward = (dist_rf + dist_lf + dist_lb + dist_rb) / 4.0
        if step_forward > 0:
            _approach_forward_dist += step_forward
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
                        print("  [UWBX] UWB 移动中重连建立成功！")
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
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            st = "HEALTHY" if uwb_is_active else "DROPPED(RUNNING)"
            print("  [UWBX] UWB_Link={} v={:.2f}m/s yaw={:.2f}° t={:.1f}s".format(
                st, fwd_speed, _yaw(), elapsed))
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


# ═══════════════════════════════════════════════════════════════
#  返回原点: UWB 导航到 origin 坐标
# ═══════════════════════════════════════════════════════════════

def _action_return_to_origin():
    global origin
    uwb = _ensure_uwb()
    if uwb is None:
        print("  [RTN] UWB 不可用")
        return False
    if origin is None:
        print("  [RTN] origin 坐标未记录")
        return False
    target_x, target_y = origin
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
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
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
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
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
#  move_toward_fixed_point: 前进 50cm → 右平移 110cm
# ═══════════════════════════════════════════════════════════════

def move_toward_fixed_point():
    """前进 50cm 再向右平移 110cm，全程 IMU 航向锁定不漂移。

    参照 _forward_distance 的编码器里程计闭环 + IMU 航向保持模式。
    """
    print("\n" + "=" * 50)
    print("[MFP] 前进 50cm → 右平移 110cm（全程航向锁定）")
    print("=" * 50)

    pause_encoder_ticker()
    _encoder_reset()

    target_heading = _lock_yaw()
    print("  [MFP] 航向锁定: {:.1f}°".format(target_heading))
    led.value(1)

    # ── 阶段 1: 前进 50cm ──
    print("  [MFP] 阶段 1/2: 前进 {:.0f}cm...".format(MFP_FORWARD_DIST_CM))
    fwd_dist_m = MFP_FORWARD_DIST_CM / 100.0
    total_dists = [0.0, 0.0, 0.0, 0.0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    last_loop_ms = start_ms

    while True:
        now_ms = time.ticks_ms()
        if _abort_check():
            stop_all()
            led.value(0)
            return False
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > MFP_TIMEOUT_S:
            print("  [MFP] 前进超时 ({:.1f}s)".format(elapsed))
            stop_all()
            led.value(0)
            return False
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001 or actual_dt > 3 * MFP_CTRL_DT:
            actual_dt = MFP_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms
        for i in range(4):
            if ENC_SCALE[i] != 0:
                total_dists[i] += abs(counts[i]) / abs(ENC_SCALE[i])
        avg_dist = sum(total_dists) / len(total_dists)
        if avg_dist >= fwd_dist_m:
            print("  [MFP] 前进完成 dist={:.2f}m".format(avg_dist))
            break
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [MFP] FWD dist={:.2f}m / {:.0f}cm yaw={:.2f}°".format(
                avg_dist, MFP_FORWARD_DIST_CM, _yaw()))
        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()
        wz = _heading_correction(target_heading, dt=actual_dt)
        try:
            speeds = _get_local_filtered_speeds(counts, actual_dt)
            omni_drive_closed_loop(MFP_SPEED, 0, wz, speeds, actual_dt)
        except Exception as e:
            print("  [MFP] 驱动错误:", e)
        time.sleep_ms(int(MFP_CTRL_DT * 1000))

    _pause_with_yaw_hold(target_heading, 300)
    _encoder_reset()

    # ── 阶段 2: 右平移 90cm ──
    print("  [MFP] 阶段 2/2: 右平移 {:.0f}cm...".format(MFP_RIGHT_DIST_CM))
    right_dist_m = MFP_RIGHT_DIST_CM / 100.0
    total_dists = [0.0, 0.0, 0.0, 0.0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    last_loop_ms = start_ms

    while True:
        now_ms = time.ticks_ms()
        if _abort_check():
            stop_all()
            led.value(0)
            return False
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > MFP_TIMEOUT_S:
            stop_all()
            led.value(0)
            return False
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001 or actual_dt > 3 * MFP_CTRL_DT:
            actual_dt = MFP_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms
        for i in range(4):
            if ENC_SCALE[i] != 0:
                total_dists[i] += abs(counts[i]) / abs(ENC_SCALE[i])
        avg_dist = sum(total_dists) / len(total_dists)
        if avg_dist >= right_dist_m:
            print("  [MFP] 右平移完成 dist={:.2f}m".format(avg_dist))
            break
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [MFP] RIGHT dist={:.2f}m / {:.0f}cm yaw={:.2f}°".format(
                avg_dist, MFP_RIGHT_DIST_CM, _yaw()))
        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()
        wz = _heading_correction(target_heading, dt=actual_dt)
        try:
            speeds = _get_local_filtered_speeds(counts, actual_dt)
            omni_drive_closed_loop(0, MFP_SPEED, wz, speeds, actual_dt)
        except Exception as e:
            print("  [MFP] 驱动错误:", e)
        time.sleep_ms(int(MFP_CTRL_DT * 1000))

    stop_all()
    led.value(0)
    print("[MFP] ✓ 完成")
    print("=" * 50 + "\n")
    return True


# ═══════════════════════════════════════════════════════════════
#  C14 组合动作: 前进 20cm → UWB 平移
# ═══════════════════════════════════════════════════════════════

def action_c14():
    print("\n" + "=" * 50)
    print("[C14] 前进 20cm → UWB 平移（航向保持）")
    print("=" * 50)
    pause_encoder_ticker()
    _encoder_reset()
    result = True
    if not _action_forward_20cm():
        result = False
    else:
        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()
        if not _action_uwb_translate():
            result = False
    stop_all()
    led.value(0)
    if result:
        print("[C14] ✓ 完成")
    else:
        print("[C14] ✗ 中断")
    print("=" * 50 + "\n")


def _action_forward_20cm():
    return _forward_distance(FORWARD_DIST_M, FORWARD_SPEED, FWD_TIMEOUT_S, label="FWD")


# ═══════════════════════════════════════════════════════════════
#  按键测试模式 main
# ═══════════════════════════════════════════════════════════════

def main():
    print("\n  [TEST_MODE] 启动键盘调度测试，等待按键按下...")

    _system_init()

    while True:
        capture()
        pet_watchdog()

        if key_triggered(1):
            print("\n  [TEST_MODE] KEY1 (C8) 触发: move_toward_fixed_point()")
            try:
                move_toward_fixed_point()
            except Exception as e:
                print("  [TEST_MODE] 发生异常:", e)
            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()
            continue

        if key_triggered(2):
            print("\n  [TEST_MODE] KEY2 (C9) 触发: 右移搜索 ➔ 取物流程")
            try:
                global _rightward_cumulative, _approach_forward_dist
                _rightward_cumulative = 0.0
                _approach_forward_dist = 0.0
                pause_encoder_ticker()
                _encoder_reset()
                found = _action_rightward_search()
                if found:
                    print("  [TEST_MODE] 发现目标，开始对齐靠近...")
                    _encoder_reset()
                    arrived = see_and_push()
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
        move_toward_fixed_point()

        while True:
            pause_encoder_ticker()
            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()

            item_found = _action_rightward_search()

            if check_sw2():
                print("  [MAIN_LOOP] SW2 介入打断，立即退出...")
                break

            if not item_found:
                print("\n  [MAIN_LOOP] 右移搜索完成未找到，UWB 返航至初始坐标...")
                _action_return_to_origin()
                break

            _approach_forward_dist = 0.0
            _encoder_reset()

            if not see_and_push():
                print("  [MAIN_LOOP] 对齐靠近中途中断，倒退回巡航轨道...")
                _reverse_to_rightward_path()
                continue

            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()

            if not _action_forward_until_uwb_x():
                print("  [MAIN_LOOP] 前行过线中途中断，倒退回巡航轨道...")
                _reverse_to_rightward_path()
                continue

            if check_sw2():
                print("  [MAIN_LOOP] SW2 手动旁路跳过蓝牙...")
            else:
                target_heading = _lock_yaw()
                retry_count = 0
                try:
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

            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()

            _reverse_to_rightward_path()

            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()
            print("  [MAIN_LOOP] 已顺利复位回到巡航主线，继续搜寻下个目标...")

    except KeyboardInterrupt:
        print("  [MAIN_LOOP] 用户手动终止 (Ctrl+C)")
    finally:
        _system_cleanup()

elif RUN_MODE == "test":
    main()
    import sys; sys.exit()
