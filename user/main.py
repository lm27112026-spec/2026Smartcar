#  RT1021 — 按键驱动控制

import gc, time, math
from machine import Pin
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
                   pause_encoder_ticker, resume_encoder_ticker, ENC_SCALE,
                   )
gc.collect()  # 导入关键模块前回收内存

from imu_motion import (update_angle, imu_read_safe,
                         reset_ang_vel_pid, stop_imu_ticker)

def _yaw():
    """动态获取 imu_motion.yaw（避免 from-import 值拷贝）"""
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
MFP_FORWARD_DIST_CM  = 50.0    # 目标50cm 
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


def _get_local_filtered_speeds(raw_counts, dt):
    """基于外部传入的 counts 数组计算一阶低通滤波速度 [rf, lf, lb, rb] (m/s)。
    不调用 get_encoder_counts()，杜绝二次读取导致增量清零。
    🟢 内存零分配：原地修改 _local_prev_speeds，不创建新 list"""
    global _local_prev_speeds, _local_speeds_initialized

    if raw_counts is None or len(raw_counts) < 4:
        return _local_prev_speeds

    # ── 防御性阈值守护：防止外部库传入 dt=0 或时钟抖动导致的除零崩溃 ──
    if dt <= 0.0001:
        dt = FWD_CTRL_DT  # 容错回退到默认控制周期 0.02s

    if not _local_speeds_initialized:
        for i in range(4):
            _local_prev_speeds[i] = (raw_counts[i] / ENC_SCALE[i] / dt) if ENC_SCALE[i] != 0 else 0
        _local_speeds_initialized = True
        return _local_prev_speeds

    # 原地逐元素更新：零堆分配
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
        if val == 0:             # 仅按下（低电平）触发，松开不触发
            return True
    return False


# ═══════════════════════════════════════════════════════════════
#  蓝牙从车通信
# ═══════════════════════════════════════════════════════════════

from uart_master import MasterBT
bt = MasterBT()


# ═══════════════════════════════════════════════════════════════
#  工具函数与蓝牙安全接收
# ═══════════════════════════════════════════════════════════════

_imu_ticker_stopped = False
_imu_ok_count = 0     
_imu_fail_count = 0   
_maintain_yaw_fail = 0   # _maintain_yaw() 驱动异常计数（SPI 总线故障等）


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
        print("  [IMU] 首次读取失败（imu_read_safe 返回 None）")
    if _imu_fail_count % 50 == 0:
        print("  [IMU] fail count={} yaw={:.2f}°".format(_imu_fail_count, _yaw()))
    return False


def _get_heading_hold():
    """懒初始化 HeadingHold 实例（无 IMU 模式，由调用方外部喂 yaw）"""
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
    """PI 航向修正：应用层角度环回强校验 + 委托 HeadingHold。
    
    双重保险:
      1. 本层先做 wrap-around 角度差检查，|diff|≤deadband 直接返回 0
      2. HeadingHold.update() 内部同样有死区兜底
    """
    if dt is None:
        dt = FWD_CTRL_DT
    if deadband is None:
        deadband = 1.0
    _read_imu_update_yaw()
    current_yaw = _yaw()

    # ── 应用层物理强锁：角度环回差值 ≤ deadband 时严格释放 ──
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

    # ── 应用层物理强锁：角度环回差值 ≤ 1.0° 时释放电机，防高频微震 ──
    diff = (target_heading - current_yaw + 180) % 360 - 180
    if abs(diff) <= 1.0:
        stop_all()
        return True

    hold = _get_heading_hold()
    if target_heading != hold.target:
        hold.set_target(target_heading)

    wz, _, _ = hold.update(current_yaw, FWD_CTRL_DT)

    # 兜底死区检查（HeadingHold.update 内部也会做，此处为安全网）
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
        elif _maintain_yaw_fail % 50 == 0:
            print("  [YAW] _maintain_yaw 驱动异常累计={} 最近:".format(_maintain_yaw_fail), e)
        return False

    return True


def _pause_with_yaw_hold(target_heading, duration_ms):
    """暂停 duration_ms 毫秒，期间持续保持航向修正。
    确保暂停期间 yaw 不漂移。"""
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
    # 每步结束清零 HeadingHold PID 积分，防止碰撞后积分饱和扭动
    if _heading_hold is not None:
        _heading_hold.reset()
    # 重置本地滤波器状态机，防止阶段切换时残留速度产生"推背感"
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
                    _maintain_yaw(_TARGET_HEADING)  # 等待期间持续修正航向
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
    """如果 UWB 硬件断连（UART 死），重置以便下次 _ensure_uwb() 重建。"""
    global _uwb_shared
    if _uwb_shared is not None and _uwb_shared.is_timeout():
        if _uwb_shared.is_uart_alive():
            print("  [UWB] 帧超时但 UART 硬件正常，等待恢复...")
            return  # 不销毁实例，等帧自然恢复
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
#  通用前进函数（编码器里程计 + IMU 航向闭环）
# ═══════════════════════════════════════════════════════════════

def _forward_distance(dist_m, speed, timeout_s, heading_deadband=None, label="FWD"):
    """通用前进：编码器距离闭环 + IMU 航向保持。"""
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

        # ── 🟢 核心修改：先获取并校验编码器 counts，保证硬件计数周期不被 continue 割裂 ──
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        # ── 🟢 校验通过后，再刷新时间基准，计算真实的 actual_dt，防止突变 ──
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001:
            actual_dt = FWD_CTRL_DT
        if actual_dt > 3 * FWD_CTRL_DT:
            actual_dt = FWD_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms

        # 对单次增量脉冲取绝对值累加，彻底规避左右轮极性抵消的致命 Bug
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
#  C14 步骤 1: 前进 20cm
# ═══════════════════════════════════════════════════════════════

def _action_forward_20cm():
    return _forward_distance(FORWARD_DIST_M, FORWARD_SPEED, FWD_TIMEOUT_S, label="FWD")


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

        # ── 🟢 核心修改二：校验通过后，在时序计算前统一使用新时间戳，并加入零分配防暴冲 ──
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001:
            actual_dt = FWD_CTRL_DT
        if actual_dt > 3 * FWD_CTRL_DT:
            actual_dt = FWD_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms

        # 倒退时同样采用绝对值累加，防止倒退距离计算抵消
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
            print("  [BACKUP] 倒退驱动错误:", e)

        time.sleep_ms(int(FWD_CTRL_DT * 1000))

    stop_all()
    time.sleep_ms(300)
    _encoder_reset()


# ═══════════════════════════════════════════════════════════════
#  _action_rightward_search: 向右平移搜索，累计1.2m内逐帧摄像头检测
# ═══════════════════════════════════════════════════════════════

def _action_rightward_search():
    """向右平移搜索，累计1.2m内逐帧摄像头检测物品。

    使用全局 _rightward_cumulative 跟踪跨轮次累计右移距离。

    返回:
        True  = 发现物品（已 stop_all，_rightward_cumulative 已保存断点）
        False = 1.2m 完成无发现 / SW2 中断 / 超时
    """
    global _rightward_cumulative

    # 1. 锁定当前航向并打印初态
    target_heading = _lock_yaw()
    print("  [RIGHT] 航向锁定: {:.1f}°".format(target_heading))
    print("  [RIGHT] 当前累计进度: {:.2f}m / {:.1f}cm".format(
        _rightward_cumulative, RIGHTWARD_SEARCH_DIST_CM))

    # 2. 边界检查
    target_dist_m = RIGHTWARD_SEARCH_DIST_CM / 100.0
    if _rightward_cumulative >= target_dist_m:
        print("  [RIGHT] 警告：累计右移已达目标值 ({:.2f}m >= {:.2f}m)，直接退出".format(
            _rightward_cumulative, target_dist_m))
        return False

    # 3. 初始化会话内变量与摄像头
    session_dists = [0.0, 0.0, 0.0, 0.0]
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    last_loop_ms = start_ms
    cam = _ensure_cam()

    # 指示灯开启
    led.value(1)

    while True:
        now_ms = time.ticks_ms()

        # A. SW2 中断检查
        if _abort_check():
            stop_all()
            led.value(0)
            print("  [RIGHT] 检测到 SW2 手动中断，放弃保存本轮移动量")
            return False

        # B. 超时检查
        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > RIGHTWARD_TIMEOUT_S:
            stop_all()
            led.value(0)
            print("  [RIGHT] 超时 ({:.1f}s)，放弃保存本轮移动量".format(elapsed))
            return False

        # C. 读取编码器增量
        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        # 🟢 时序对齐：编码器读取成功后才计算 actual_dt 并更新时间戳
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001:
            actual_dt = RIGHTWARD_CTRL_DT
        if actual_dt > 3 * RIGHTWARD_CTRL_DT:
            actual_dt = RIGHTWARD_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms

        # D. 编码器绝对值累加（本会话平均距离）
        for i in range(4):
            if ENC_SCALE[i] != 0:
                session_dists[i] += abs(counts[i]) / abs(ENC_SCALE[i])
        session_avg = sum(session_dists) / 4.0

        # E. 计算跨轮次累计距离
        total_cumulative = _rightward_cumulative + session_avg

        # F. 判断是否达到 1.2m 目标
        if total_cumulative >= target_dist_m:
            _rightward_cumulative = total_cumulative  # 保存最终位置（接近 1.2m）
            stop_all()
            led.value(0)
            print("  [RIGHT] 达到 1.2m 搜索边界 (实际 {:.2f}m)，未检测到目标".format(
                total_cumulative))
            return False

        # G. 摄像头物品检测
        ctrl = cam.step()
        if ctrl and ctrl.get('has_target', False):
            _rightward_cumulative = total_cumulative  # 保存检测到物品时的断点位置
            stop_all()
            led.value(0)
            print("  [RIGHT] 检测到目标！保存断点：{:.2f}m".format(_rightward_cumulative))
            return True

        # H. 每 500ms 打印一次搜寻进度
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            print("  [RIGHT] cum={:.2f}m remain={:.1f}cm yaw={:.2f}°".format(
                total_cumulative,
                (RIGHTWARD_SEARCH_DIST_CM - total_cumulative * 100.0),
                _yaw()
            ))

        # I. 循环计数及内存垃圾回收
        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        # J. 航向修正
        wz = _heading_correction(target_heading, dt=actual_dt)

        # K. 驱动：执行闭环右平移 (vx=0, vy=+RIGHTWARD_SPEED)
        try:
            speeds = _get_local_filtered_speeds(counts, actual_dt)
            omni_drive_closed_loop(0, RIGHTWARD_SPEED, wz, speeds, actual_dt)
        except Exception as e:
            print("  [RIGHT] 驱动异常:", e)

        # L. 控制周期延时
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

        # ── 🟢 核心修改二：校验通过后，在时序计算前统一使用新时间戳，并加入零分配防暴冲 ──
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001:
            actual_dt = FWD_CTRL_DT
        if actual_dt > 3 * FWD_CTRL_DT:
            actual_dt = FWD_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms

        # 绝对值累加，规避镜像接线极性抵消
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
#  修改重构: 摄像头跟随靠近
# ═══════════════════════════════════════════════════════════════

def see_and_push():
    """
    完全复用 test_vision_track.py 的核心跟随控制机理（CamDataReceiver + FollowController），
    结合本地单次读取一阶低通滤波、动态时间步长与 8 帧对齐精准停靠判定。
    """
    print("\n  [APPROACH] === 视觉对齐靠近 ➔ 蓝牙同步流程 ===")

    # ── 🟢 获取全局共享的 Camera 实例 ──
    cam = _ensure_cam()
    from cam_data import x_to_cm, y_to_distance
    from cam_control import FollowController

    # ── 🟢 核心修改一：采用无侵入的方案 A，直接读取私有属性 _recv ──
    recv = cam._recv
    fc = FollowController()

    # ── 状态及历史数据初始化 ──
    target_heading = _lock_yaw()
    last_x_cm = 0.0
    last_dist_cm = 0.0
    last_has_tgt = False
    prev_vy = 0.0

    align_consecutive_count = 0  # 连续稳定对齐判定计数
    arrived_success = False      # 到达停靠点标志位

    LOOP_MS = 10                 # 100Hz 严苛控制节拍
    t_prev = time.ticks_ms()
    loop_cnt = 0

    # 开启指示灯
    led.value(1)

    try:
        while True:
            t_now = time.ticks_ms()
            dt_act = time.ticks_diff(t_now, t_prev) * 0.001
            t_prev = t_now
            
            # 防御性时间异常钳位
            if dt_act <= 0 or dt_act > 0.1:
                dt_act = 0.01

            # ── 1. 获取传感器原始数据 ──
            cam_data = recv.read()
            
            # ── 2. 计算 IMU 航向补偿 wz ──
            wz = _heading_correction(target_heading, dt=dt_act)

            # ── 3. 视觉跟随状态解算 ──
            has_tgt = False
            x_cm = 0.0
            dist_cm = 0.0

            if cam_data is not None:
                has_tgt = cam_data['is_target']
                if has_tgt:
                    x_cm = x_to_cm(cam_data['x'])
                    dist_cm = y_to_distance(cam_data['y'])
                    # 缓存有效坐标 — 短暂丢失时避免跳变到(0,0)产生剧烈摆动
                    last_x_cm = x_cm
                    last_dist_cm = dist_cm
                    last_has_tgt = True
                elif last_has_tgt:
                    # 摄像头帧存在但目标短暂丢失 → 使用上一帧有效值平滑过渡
                    x_cm = last_x_cm
                    dist_cm = last_dist_cm
            else:
                if last_has_tgt:
                    # UART 通信瞬时卡顿无数据 → 使用上一帧有效值过渡
                    x_cm = last_x_cm
                    dist_cm = last_dist_cm

            # ── 4. 输入 FollowController 进行 3 轴闭环速度及对齐状态解算 ──
            vx, vy, wz_out, is_aligned, dt_step = fc.step(
                x_cm, dist_cm, has_tgt, wz_in=wz, now_ms=t_now
            )

            # ── 5. 驱动增益控制增强：速度倍增 + 一阶阻尼 + 临停死区 ──
            
            # 前进速度放大 1.5 倍，物理上限限幅在 0.8m/s
            boost_vx = vx * 1.5
            boost_vx = max(0.0, min(boost_vx, 0.80))

            # 横向平移一阶阻尼滤波（60% 历史速度 + 40% 当前指令），阻尼横向共振摆动
            smooth_vy = 0.6 * prev_vy + 0.4 * vy
            prev_vy = smooth_vy

            # 临近精细对齐判定（改用 vx 判断接近度，避免远距离因偏差收敛提前降速）
            if boost_vx < 0.12:
                smooth_vy = smooth_vy * 0.5
                if abs(smooth_vy) < 0.03:
                    smooth_vy = 0.0

            # ── 6. 读取编码器并输出执行（严格单次读取，规避双读 bug） ──
            rc = get_encoder_counts()
            if rc is not None and len(rc) >= 4:
                # 实时累加平均位移到全局前进位移变量（倒车回线重要参数）
                global _approach_forward_dist
                wheel_sum = 0.0
                valid = 0
                for i in range(4):
                    if ENC_SCALE[i] != 0:
                        wheel_sum += abs(rc[i]) / abs(ENC_SCALE[i])
                        valid += 1
                if valid > 0:
                    _approach_forward_dist += (wheel_sum / valid)

                # 底层闭环轮速驱动
                speeds = _get_local_filtered_speeds(rc, dt_step)
                omni_drive_closed_loop(boost_vx, smooth_vy, wz_out, speeds, dt_step)
            else:
                # 编码器空读时仅做原地姿态维持
                stop_all()

            # ── 7. 精准到达对齐判定（引入 8 帧防抖动去噪滤波） ──
            if is_aligned:
                align_consecutive_count += 1
            else:
                align_consecutive_count = 0

            if align_consecutive_count >= 8:
                stop_all()
                print("\n  [APPROACH] 目标精准对齐停靠完成 (连续 8 帧对齐确认)！")
                arrived_success = True
                break

            # ── 8. 中断退出检测与 100Hz 严格节拍维持 ──
            if _abort_check():
                print("\n  [APPROACH] SW2 手动中断退出。")
                stop_all()
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

    # ── 如果对齐跟随过程未最终到达，不执行后续协同，直接退回 ──
    if not arrived_success:
        return False

    # ── 9. 对齐停靠成功，启动蓝牙协同发送与重试机制（生产 main 逻辑） ──
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
                _maintain_yaw(target_heading)  # 等待期间持续修正姿态
                if time.ticks_diff(time.ticks_ms(), bt_wait_start) > BT_WAIT_DEADLINE_S * 1000:
                    print("  [APPROACH] 等待从车 ok 超时 ({}s)，重新发送...".format(BT_WAIT_DEADLINE_S))
                    break  # 退出内循环，重新发送
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
#  核心修复: 全速前进至 UWB X 距离 < -130cm（支持移动中重连，依靠视觉兜底不停车）
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

    # ── 摄像头黄线检测初始化 ──
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
            # 即使掉线导致 UWB X 超时，由于已处于运行中，绝不返回原点，让其继续推进依靠黄线拦截
            print("  [UWBX] 超时 ({:.1f}s)，UWB 未达标，仅依靠视觉黄线推进兜底".format(elapsed))
            pass

        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        # ── 🟢 核心修改二：校验通过后，在时序计算前统一使用新时间戳，并加入零分配防暴冲 ──
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001:
            actual_dt = FWD_CTRL_DT
        if actual_dt > 3 * FWD_CTRL_DT:
            actual_dt = FWD_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms

        # 🟢 实时累加平均前进位移到 _approach_forward_dist
        global _approach_forward_dist
        wheel_sum = 0.0
        valid = 0
        for i in range(4):
            if ENC_SCALE[i] != 0:
                wheel_sum += abs(counts[i]) / abs(ENC_SCALE[i])
                valid += 1
        if valid > 0:
            _approach_forward_dist += (wheel_sum / valid)

        now = time.ticks_ms()
        
        # ── 1. UWB 与 摄像头步进（均在移动中高频执行） ──
        if uwb is not None and time.ticks_diff(now, last_uwb_ms) >= 50:
            uwb.step()
            last_uwb_ms = now

        if time.ticks_diff(now, last_cam_ms) >= 50:
            ctrl = cam.step()
            last_cam_ms = now

        # ── 2. 优先保障黄线越界视觉安全（核心安全兜底锁，即使 UWB 掉线，只要视觉检测到越界立刻停车倒退） ──
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

        # ── 3. UWB 掉线判定与移动中自动重建（🟢 核心：不调 stop_all，车不熄火，不重置主流程） ──
        uwb_is_active = True
        if uwb is None or uwb.is_timeout():
            uwb_is_active = False
            if uwb_dead_start == 0:
                uwb_dead_start = time.ticks_ms()
                print("  [UWBX] UWB 数据中断，车身保持安全巡航速度并于移动中尝试重连...")
            elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > UWB_DEAD_TIMEOUT_S * 1000:
                if uwb is not None and uwb.is_uart_alive():
                    print("  [UWBX] UART 仍活跃，延长移动等待 (帧过滤中)...")
                    uwb_dead_start = time.ticks_ms()  # 延长等待时间
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
                    # 重连用尽后依旧不熄火，保持匀速直行推进，直至触发黄线越界
                    print("  [UWBX] UWB 彻底离线，已进入视觉托管状态，继续安全车速行进...")
                    uwb_dead_start = time.ticks_ms()
        else:
            if uwb_dead_start != 0:
                print("  [UWBX] UWB 链路已自主恢复")
            uwb_dead_start = 0
            uwb_dead_reconnect = 0

        # ── 4. 根据 UWB 链路健康度动态决策车速 ──
        fwd_speed = UWB_X_MIN_SPEED
        if uwb_is_active:
            try:
                x_cm, y_cm = uwb.get_position()
                
                # UWB 正常到达
                if x_cm < UWB_X_THRESHOLD_CM:
                    stop_all()
                    print("  [UWBX] UWB X={:.1f}cm < {:.0f}cm，坐标触发停车！".format(
                        x_cm, UWB_X_THRESHOLD_CM))
                    return True

                # 线性减速
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
            # 🟢 掉线期间的安全匀速直行速度：继续推进
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

    def _lock_fn():
        return _lock_yaw()

    def _wz_fn(target, db):
        return _heading_correction(target, deadband=db, dt=ORIGIN_CTRL_DT)

    def _yaw_fn():
        return _yaw()

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

    def _stop_fn():
        stop_all()

    def _led_fn(val):
        led.value(val)

    arrived, reason = goto_location(
        uwb, target_x, target_y,
        _lock_fn, _wz_fn, _yaw_fn,
        _abort_fn, _drive_fn, _stop_fn,
        _led_fn, label="RTN"
    )
    return arrived


# ═══════════════════════════════════════════════════════════════
#  C14 步骤 2: UWB 平移（航向保持）- 支持掉线惯性滑动与重连
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
            return True # 平移作为辅助对齐动作，超时也不报错返回，继续后续逻辑

        if uwb is not None:
            uwb.step()
            try:
                x_cm, y_cm = uwb.get_position()
            except Exception as e:
                print("  [UWB] get_position() 异常:", e)
                _maintain_yaw(target_heading)
                time.sleep_ms(50)
                continue

        # UWB 平移掉线保护（惯性前移并后台重建）
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

        # 平移策略：掉线时使用衰减的最后已知平移速度，配合姿态锁定推进
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
            fwd_speed = last_fwd_speed * 0.90 # 掉线平移速度指数级衰减，防过度冲刺

        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 500:
            last_print_ms = now
            if uwb_is_active:
                dist_cm, angle_deg = uwb.get_distance_angle()
                print("  [UWB] X={0:+.1f}cm Y={1:.1f}cm D={2:.1f}cm A={3:.1f}°  yaw={4:.2f}°".format(
                    x_cm, y_cm, dist_cm, angle_deg, _yaw()))
            else:
                print("  [UWB] LOST v_lat={:.2f}m/s yaw={:.2f}°".format(fwd_speed, _yaw()))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        # ── 🟢 先校验编码器 ──
        rc = get_encoder_counts()
        if rc is None or len(rc) < 4:
            time.sleep_ms(5)
            continue

        # ── 🟢 校验通过后再算 dt 并刷新 ──
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001:
            actual_dt = UWB_CTRL_DT
        if actual_dt > 3 * UWB_CTRL_DT:
            actual_dt = UWB_CTRL_DT
            global _local_speeds_initialized
            for i in range(4):
                _local_prev_speeds[i] = 0.0
            _local_speeds_initialized = False
        last_loop_ms = now_ms

        wz = _heading_correction(target_heading, dt=actual_dt)

        try:
            speeds = _get_local_filtered_speeds(rc, actual_dt)
            omni_drive_closed_loop(fwd_speed, 0, wz, speeds, actual_dt)
        except Exception as e:
            print("  [UWB] 驱动错误:", e)

        time.sleep_ms(int(UWB_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  move_toward_fixed_point: 前进 50cm → 右平移 110cm（全程航向锁定）
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
            resume_encoder_ticker()
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > MFP_TIMEOUT_S:
            print("  [MFP] 前进超时 ({:.1f}s)".format(elapsed))
            stop_all()
            resume_encoder_ticker()
            led.value(0)
            return False

        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        # ── 🟢 核心修改二：校验通过后，在时序计算前统一使用新时间戳，并加入零分配防暴冲 ──
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001:
            actual_dt = MFP_CTRL_DT
        if actual_dt > 3 * MFP_CTRL_DT:
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

    # 阶段间停顿，保持航向
    _pause_with_yaw_hold(target_heading, 300)
    _encoder_reset()

    # ── 阶段 2: 右平移 110cm ──
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
            resume_encoder_ticker()
            led.value(0)
            return False

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > MFP_TIMEOUT_S:
            print("  [MFP] 右平移超时 ({:.1f}s)".format(elapsed))
            stop_all()
            resume_encoder_ticker()
            led.value(0)
            return False

        counts = get_encoder_counts()
        if counts is None or len(counts) < 4:
            time.sleep_ms(5)
            continue

        # ── 🟢 核心修改二：校验通过后，在时序计算前统一使用新时间戳，并加入零分配防暴冲 ──
        actual_dt = time.ticks_diff(now_ms, last_loop_ms) / 1000.0
        if actual_dt <= 0.001:
            actual_dt = MFP_CTRL_DT
        if actual_dt > 3 * MFP_CTRL_DT:
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
    resume_encoder_ticker()
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
    resume_encoder_ticker()
    led.value(0)

    if result:
        print("[C14] ✓ 完成")
    else:
        print("[C14] ✗ 中断")
    print("=" * 50 + "\n")


# ═══════════════════════════════════════════════════════════════
#  C8: 蓝牙向从车发送消息
# ═══════════════════════════════════════════════════════════════

def action_c8():
    print("\n" + "=" * 50)
    print("[C8] move_toward_fixed_point 前进50cm → 右移110cm")
    print("=" * 50)
    move_toward_fixed_point()
    print("=" * 50 + "\n")


# ═══════════════════════════════════════════════════════════════
#  C9: 摄像头跟随靠近
# ═══════════════════════════════════════════════════════════════

def action_c9():
    print("\n" + "=" * 50)
    print("[C9] 摄像头跟随靠近")
    print("=" * 50)

 

# ═══════════════════════════════════════════════════════════════
#  主程序流程控制（main）
# ═══════════════════════════════════════════════════════════════

def main():
    print("")
    print("=" * 50)
    print("  RT1021 — 按键驱动控制（右移搜索+取物返航版）")
    print("=" * 50)
    print("  C14 (KEY3): 前进 20cm → UWB 平移")
    print("  C8  (KEY1): 前进 50cm → 右移 110cm")
    print("  C9  (KEY2): ")
    print("  SW2 (D9)  : 强制退出")
    print("=" * 50)
    print("")

    # ── 🟢 延迟导入：此时 main.py 已完成编译，内存处于空闲状态，绝对不会溢出 ──
    global CameraController, goto_location
    gc.collect()
    import cam_control
    CameraController = cam_control.CameraController
    import uwb_control
    goto_location = uwb_control.goto_location
    gc.collect()
    print("  [MAIN] 控制模块加载就绪，当前可用内存: {} 字节".format(gc.mem_free()))

    # ── ⚠ 在进行任何闭环控制前，必须先暂停后台编码器定时器，避免底层资源竞争 [1] ──
    pause_encoder_ticker()
    _encoder_reset()

    # ── 🔒 提前锁定航向 ──
    _lock_yaw()

    # ── UWB 初始化 & 记录起点坐标 ──
    global origin
    uwb = _ensure_uwb()
    if uwb is not None and uwb.get_frame_count() > 0:
        origin = uwb.get_position()  
        print("  [ORIGIN] 起点坐标已记录: ({:.1f}, {:.1f})".format(
            origin[0], origin[1]))
    else:
        print("  [ORIGIN] UWB 未就绪，使用默认坐标")
        origin = (130.0, 262.0)

    # ──── 注释掉完整流程，启用测试模式 ────
    # print("  [MAIN] 航向已锁定，执行初始路径移动...")
    # move_toward_fixed_point()
    
    # ═══════════════════════════════════════════════════════
    #  主循环：右移搜索（累计1.2m）+ 取物流程
    # ═══════════════════════════════════════════════════════

    # ──── 测试模式：C8/C9 按键调度 ────
    print("  [MAIN] 航向已锁定（测试模式）")
    print("  [MAIN] C8=move_toward_fixed_point  C9=rightward_search  SW2=退出")
    print("")

    start_watchdog()
    print("  [WWDG] 独立硬件看门狗已启用 (3秒超时)")

    while True:
        capture()
        pet_watchdog()

        if key_triggered(1):  # KEY1 = C8
            print("\n  [MAIN] C8 触发: move_toward_fixed_point()")
            try:
                move_toward_fixed_point()
            except Exception as e:
                print("  [MAIN] move_toward_fixed_point 异常:", e)
            print("  [MAIN] 完成，等待下一次按键...\n")
            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()
            continue

        if key_triggered(2):  # KEY2 = C9
            print("\n  [MAIN] C9 触发: 右移搜索 → 发现目标后靠近")
            try:
                # ── 🟢 补全调试细节：每次测试搜寻时，均重置累计进度，保证多次按键调试皆能跑满 ──
                global _rightward_cumulative
                _rightward_cumulative = 0.0

                # 🟢 暂停后台定时器，防止与主循环抢编码器数据
                pause_encoder_ticker()
                _encoder_reset()

                found = _action_rightward_search()
                if found:
                    print("  [MAIN] 检测到目标，开始视觉靠近...")
                    global _approach_forward_dist
                    _approach_forward_dist = 0.0
                    _encoder_reset()

                    arrived = see_and_push()
                    print("  [MAIN] 视觉靠近结果: {}".format("到达" if arrived else "中断"))
                else:
                    print("  [MAIN] 未发现目标")

                resume_encoder_ticker()
            except Exception as e:
                print("  [MAIN] C9 流程异常:", e)
            print("  [MAIN] 完成，等待下一次按键...\n")
            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()
            continue

        if check_sw2():
            print("  [MAIN] SW2 退出测试模式")
            break

        time.sleep_ms(10)

    stop_all()
    _encoder_reset()
    try:
        pause_encoder_ticker()
        stop_imu_ticker()
        stop_key()  # 🟢 程序退出时，安全关闭看门狗定时器，防止其复位 MCU
        # ── 纯引用抹除：置 None + 从 globals() 删除，不触发任何物理外设关断 ──
        _clean_globals_for_ide()
    except Exception:
        pass
    
    print("\n[INFO] 程序运行结束。")
    return


# ═══════════════════════════════════════════════════════════════
#  安全返航并彻底停机退出 REPL 辅助函数
# ═══════════════════════════════════════════════════════════════

def _safe_return_and_exit():
    """
    导航回到 origin 点，清空硬件资源，关闭所有后台定时器，安全退回到 REPL。
    """
    print("\n  [RTN] ➔ 正在返航...")
    if _TARGET_HEADING is None:
        _lock_yaw()
    _pause_with_yaw_hold(_TARGET_HEADING, 300)
    _encoder_reset()
    
    _action_return_to_origin()

    stop_all()
    _encoder_reset()
    
    global _uwb_shared
    if _uwb_shared is not None:
        _uwb_shared.stop()
        _uwb_shared = None
        
    try:
        pause_encoder_ticker()
        stop_imu_ticker()
    except Exception:
        pass
        
    time.sleep_ms(100)
    print("\n[INFO] 已安全返航至起点。程序运行结束，已安全退回 REPL 命令行。")
    return


# ═══════════════════════════════════════════════════════════════
#  入口 — 改 RUN_MODE 切换模式
#    "loop" = 正式流程（MFP → 右移搜索 → 取物返航）
#    "test" = 按键测试模式
# ═══════════════════════════════════════════════════════════════
RUN_MODE = "loop"

if RUN_MODE == "loop":
    if _TARGET_HEADING is None:
        _TARGET_HEADING = _lock_yaw()
        gc.collect()
        import cam_control
        CameraController = cam_control.CameraController
        import uwb_control
        goto_location = uwb_control.goto_location
        gc.collect()
        move_toward_fixed_point()

    while True:
        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()

        # ① 向右平移搜索（累计1.2m，逐帧摄像头检测物品）
        item_found = _action_rightward_search()

        if not item_found:
            # 1.2m 右移完成，未发现物品 → 返航停机
            print("\n  [MAIN] 1.2m 右移搜索完成，未检测到物品")
            break

        # ── 物品被发现 → 执行取物流程 ②~⑥ ──
        _approach_forward_dist = 0.0  # 重置净前进距离追踪

        # 清零此前横移搜索产生的编码器数据
        _encoder_reset()

        # ② 离开平移路径，向前视觉靠近（移动期间已通过 _drive_fn 实时累加前进距离）
        if not see_and_push():
            print("  [MAIN] 视觉靠近中断，沿原路倒退回到搜索路径...")
            _reverse_to_rightward_path()
            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()
            continue

        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()

        # ③ 全速推送过 UWB 边界（移动期间已在内部实时累加前进距离）
        if not _action_forward_until_uwb_x():
            print("  [MAIN] UWB 推送中断，沿原路倒退回到搜索路径...")
            _reverse_to_rightward_path()
            _pause_with_yaw_hold(_TARGET_HEADING, 300)
            _encoder_reset()
            continue

        # 执行 20cm 倒退（避开障碍物/边界线）
        _execute_backup(_TARGET_HEADING)

        # 手动扣除 20cm 倒退距离，得到净前进距离
        _approach_forward_dist -= (UWB_BACKUP_DIST_CM / 100.0)
        if _approach_forward_dist < 0:
            _approach_forward_dist = 0.0

        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()
        # ④ 蓝牙通知从车（永不因蓝牙超时触发返航）
        if check_sw2():
            print("  [MAIN] SW2 手动旁路，跳过蓝牙")
        else:
            target_heading = _lock_yaw()
            retry_count = 0
            try:
                while True:
                    bt.turn_left()
                    retry_count += 1
                    suffix = " (第{}次发送)".format(retry_count) if retry_count > 1 else ""
                    print("  [MAIN] turn_left 已发出{}，等待从车 ok（单次超时 {}s）...".format(suffix, BT_WAIT_DEADLINE_S))
                    bt_wait_start = time.ticks_ms()
                    received_ok = False
                    while True:
                        pet_watchdog()
                        _maintain_yaw(target_heading)
                        if time.ticks_diff(time.ticks_ms(), bt_wait_start) > BT_WAIT_DEADLINE_S * 1000:
                            print("  [MAIN] 等待从车 ok 超时 ({}s)，重新发送...".format(BT_WAIT_DEADLINE_S))
                            break  # 退出内层等待循环，重新发送
                        if check_sw2():
                            print("  [MAIN] SW2 中断等待")
                            break
                        resp = bt.read_response()
                        if resp == "ok":
                            print("  [MAIN] 从车确认完毕")
                            received_ok = True
                            break
                        time.sleep_ms(10)
                    if received_ok or check_sw2():
                        break
            except Exception as e:
                print("  [MAIN] 蓝牙通信异常:", e)
                # 蓝牙硬件异常也直接继续，不触发返航

        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()

        # ⑤ 向后倒车，回到右移路径线
        _reverse_to_rightward_path()

        _pause_with_yaw_hold(_TARGET_HEADING, 300)
        _encoder_reset()

        # ⑥ 循环回到 ①，从断点处继续右移搜索
        print("  [MAIN] 已回到右移路径，继续搜索...")

elif RUN_MODE == "test":
    main()
    import sys; sys.exit()

