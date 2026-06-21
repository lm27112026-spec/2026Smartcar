"""
main_uwb_test.py — UWB 多点导航测试
【功能】
  从位置1出发，记录起始坐标 → 依次前往定点2/3/4/5 → 返回位置1
  每个点位到达后停车确认，再前往下一个点位
【运动学映射（实测）】
  - vx 对应 UWB Y 轴（负方向）：vx>0 → Y 减小
  - vy 对应 UWB X 轴（正方向）：vy>0 → X 增大
【依赖】uwb_position.py, motor.py, imu_motion.py, pid.py
【安全】SW2 随时终止，每段有独立超时保护
"""
import gc, time, math
from machine import Pin
from pid import PID
from motor import (
    omni_drive_closed_loop, stop_all, get_encoder_counts,
    enc_ticker, ENC_SCALE, LED_PIN, SWITCH2_PIN,
)
import imu_motion
from imu_motion import (
    update_angle, get_angular_velocity, angular_velocity_control,
    reset_ang_vel_pid, stop_imu_ticker,
)
from uwb_position import UWBPosition

# ── 硬件 ──
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ═══════════════════════════════════════════════════════════════
#  定点坐标 (cm，相对于 UWB 锚点) — 可根据实际场地修改
#  位置1：运行时自动记录，无需在此定义
# ═══════════════════════════════════════════════════════════════
WP2_X = 80.0
WP2_Y = 120.0

WP3_X = 180.0
WP3_Y = 80.0

WP4_X = 120.0
WP4_Y = -60.0

WP5_X = 0.0
WP5_Y = -120.0

# ── 参数 ──
DT = 0.01
MISSION_TIMEOUT_S = 300      # 总任务超时 (5分钟)
PHASE_TIMEOUT_S = 40         # 单段导航超时
MAX_SEGMENT_RETRIES = 2      # 每段最大重试次数（UWB 噪声/超调时重试）
RETRY_COOLDOWN_MS = 500      # 重试前等待 UWB 稳定的冷却时间
PRINT_MS = 300               # 打印间隔

# ── 位置 PID ──
POS_KP = 0.012
POS_KI = 0.0
POS_DB = 8.0                # 到位死区 (cm)
POS_HYSTERESIS = 30.0       # 迟滞阈值 (cm) — 到达后需大幅偏离才重新移动
ARRIVAL_CONFIRM_FRAMES = 8  # 连续 N 帧在死区内才算真正到达
SLOW_DIST = 20.0            # 距离目标 <20cm 开始线性减速
MAX_SPEED = 0.3             # 最大速度 (m/s)
STABLE_TIME_MS = 500        # 在死区内保持500ms才算稳定停车

# ── 航向 PID ──
HDG_KP = 1.5                # 航向偏差(°) → 目标 dps
HDG_DB = 1.0                # 航向死区 (度)

# ── UWB ──
UWB_TARGET_ANCHOR = "8834"

# ═══════════════════════════════════════════════════════════════
#  辅助函数
# ═══════════════════════════════════════════════════════════════

def normalize_angle(angle):
    """角度归一化到 [-180, 180]"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def calc_target_angle(from_x, from_y, to_x, to_y):
    """计算从 (from_x, from_y) 到 (to_x, to_y) 的目标角度（UWB 坐标系）"""
    dx = to_x - from_x
    dy = to_y - from_y
    return math.atan2(-dx, dy) * 180.0 / math.pi


def calc_distance(from_x, from_y, to_x, to_y):
    """计算两点间欧几里得距离"""
    dx = to_x - from_x
    dy = to_y - from_y
    return math.sqrt(dx * dx + dy * dy)


# ═══════════════════════════════════════════════════════════════
#  直接 XY 控制：根据 UWB 坐标移动到指定点
#  【来源】提取并精简自 main_uwb.py
# ═══════════════════════════════════════════════════════════════

def move_to_target_xy(uwb, target_x, target_y, label):
    """
    直接根据 UWB 坐标计算运动方向并移动到目标。
    
    参数:
        uwb:       UWBPosition 实例
        target_x:  目标 X 坐标 (cm)
        target_y:  目标 Y 坐标 (cm)
        label:     段标签（用于日志输出）
    
    返回:
        bool: True=成功到达, False=超时或 SW2 打断
    """
    # 清空编码器残余值
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    reset_ang_vel_pid()

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    last_uwb_update_ms = start_ms
    arrival_count = 0
    stable_start_ms = None
    stable_x, stable_y = 0.0, 0.0
    is_stopped = False

    # 锁定当前航向
    target_heading = imu_motion.yaw

    print("\n  ── [{:s}] move to ({:.0f}, {:.0f}) ──".format(label, target_x, target_y))

    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0

        # ── 超时保护 ──
        if elapsed_s > PHASE_TIMEOUT_S:
            print("  TIMEOUT ({:.1f}s)".format(elapsed_s))
            return False

        # ── SW2 紧急停止 ──
        if switch2.value() != state2:
            print("  SW2 stop")
            return False

        # ── UWB 数据轮询 (50ms 周期) ──
        if time.ticks_diff(now_ms, last_uwb_update_ms) >= 50:
            uwb.step()
            last_uwb_update_ms = now_ms

        # 获取当前 UWB 坐标
        curr_x, curr_y = uwb.get_position()

        # 计算到目标的误差
        error_x = target_x - curr_x
        error_y = target_y - curr_y
        dist = math.sqrt(error_x * error_x + error_y * error_y)

        # ── 稳定停车逻辑 ──
        if dist < POS_DB:
            arrival_count += 1
            if stable_start_ms is None:
                stable_start_ms = now_ms
                stable_x, stable_y = curr_x, curr_y
                print("  >>> entering stable zone, timing...")

            stable_elapsed = time.ticks_diff(now_ms, stable_start_ms)
            if arrival_count >= ARRIVAL_CONFIRM_FRAMES and stable_elapsed >= STABLE_TIME_MS and not is_stopped:
                is_stopped = True
                omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
                final_dist = calc_distance(stable_x, stable_y, target_x, target_y)
                print("  >>> STOPPED! pos=({:.1f},{:.1f}) err={:.1f}cm".format(
                    stable_x, stable_y, final_dist))

                if final_dist < POS_HYSTERESIS:
                    print("  >>> [{:s}] done: ({:.1f}, {:.1f}) err={:.1f}cm <<<".format(
                        label, stable_x, stable_y, final_dist))
                    return True
                else:
                    # 超出迟滞，继续移动
                    is_stopped = False
                    stable_start_ms = None
                    arrival_count = 0
                    print("  >>> final check failed: err={:.1f}cm, continuing".format(final_dist))
        else:
            arrival_count = 0
            stable_start_ms = None
            is_stopped = False

        # ── 已完全停止时只做航向保持 ──
        if is_stopped:
            wz = 0.0
            d = imu_motion.imu.read()
            if d is not None:
                update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
                hdg_err = normalize_angle(target_heading - imu_motion.yaw)
                if abs(hdg_err) > HDG_DB:
                    target_dps = hdg_err * HDG_KP
                    target_dps = max(-180, min(target_dps, 180))
                    wz = angular_velocity_control(target_dps, get_angular_velocity(), DT)
            omni_drive_closed_loop(0, 0, wz, [0, 0, 0, 0], DT)
            time.sleep_ms(10)
            continue

        # ── 正常运动控制 ──
        wz = 0.0
        d = imu_motion.imu.read()
        if d is not None:
            update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

            hdg_err = normalize_angle(target_heading - imu_motion.yaw)
            if abs(hdg_err) > HDG_DB:
                target_dps = hdg_err * HDG_KP
                target_dps = max(-180, min(target_dps, 180))
                wz = angular_velocity_control(target_dps, get_angular_velocity(), DT)

        # ── P 控制：误差 → 速度 ──
        # 世界坐标系误差 → 旋转到车体坐标系
        world_err_x = error_x
        world_err_y = error_y

        yaw_rad = math.radians(imu_motion.yaw)
        rot_x = world_err_x * math.cos(yaw_rad) + world_err_y * math.sin(yaw_rad)
        rot_y = -world_err_x * math.sin(yaw_rad) + world_err_y * math.cos(yaw_rad)

        # 映射到车体运动方向（UWB: 前进=-Y, 右移=+X → 车体: 前进=+vx, 右移=+vy）
        body_fwd = -rot_y
        body_right = rot_x

        vx_cmd = body_fwd * POS_KP
        vy_cmd = body_right * POS_KP

        # 距离 < SLOW_DIST 时线性减速
        if dist < SLOW_DIST:
            decay = (dist - POS_DB) / (SLOW_DIST - POS_DB)
            decay = max(0.0, min(1.0, decay))
            vx_cmd *= decay
            vy_cmd *= decay

        # 速度限幅
        speed = math.sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd)
        if speed > MAX_SPEED:
            vx_cmd = vx_cmd / speed * MAX_SPEED
            vy_cmd = vy_cmd / speed * MAX_SPEED

        vx_cmd = max(-MAX_SPEED, min(vx_cmd, MAX_SPEED))
        vy_cmd = max(-MAX_SPEED, min(vy_cmd, MAX_SPEED))

        # 编码器闭环
        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / DT for i in range(4)]
        omni_drive_closed_loop(vx_cmd, vy_cmd, wz, raw_speeds, DT)

        if time.ticks_diff(now_ms, last_print_ms) >= PRINT_MS:
            last_print_ms = now_ms
            print("  {:4.1f}s  pos=({:.1f},{:.1f})  err=({:.1f},{:.1f})  dist={:.1f}  yaw={:.0f}".format(
                elapsed_s, curr_x, curr_y, error_x, error_y, dist, imu_motion.yaw))

        time.sleep_ms(10)

    # 不应到达此处
    omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
    return False


# ═══════════════════════════════════════════════════════════════
#  初始化
# ═══════════════════════════════════════════════════════════════

print("\n" + "=" * 50)
print("  UWB Multi-Waypoint Navigation Test")
print("  Path: P1(start) → P2 → P3 → P4 → P5 → P1(return)")
print("  WP2: ({:.0f}, {:.0f}) cm".format(WP2_X, WP2_Y))
print("  WP3: ({:.0f}, {:.0f}) cm".format(WP3_X, WP3_Y))
print("  WP4: ({:.0f}, {:.0f}) cm".format(WP4_X, WP4_Y))
print("  WP5: ({:.0f}, {:.0f}) cm".format(WP5_X, WP5_Y))
print("=" * 50)

stop_all()
time.sleep_ms(50)

# 启动编码器 ticker
enc_ticker.start(10)
time.sleep_ms(50)

# 清空编码器残余值
for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)

# IMU 热身
stop_imu_ticker()
time.sleep_ms(10)

for _ in range(10):
    d = imu_motion.imu.read()
    if d is not None:
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

# UWB 初始化
uwb = UWBPosition(target_anchor=UWB_TARGET_ANCHOR)

# ═══════════════════════════════════════════════════════════════
#  等待 UWB 数据 & 记录位置1
# ═══════════════════════════════════════════════════════════════

print("\nWaiting for UWB data...")
print("Press SW2 to exit\n")

loop_count = 0
while uwb.get_frame_count() == 0:
    loop_count += 1
    if loop_count > 10 and switch2.value() != state2:
        print("\n=== SW2 STOP ===")
        break
    if not uwb.step():
        print("\n=== SW2 from UWB ===")
        break
    time.sleep_ms(10)

# ── 记录位置1（起始位置） ──
pos1_x, pos1_y = uwb.get_position()
print("\n" + "=" * 50)
print("  Position 1 (START) recorded!")
print("  P1: ({:.1f}, {:.1f}) cm".format(pos1_x, pos1_y))
print("=" * 50)

# ═══════════════════════════════════════════════════════════════
#  导航序列：P1 → P2 → P3 → P4 → P5 → P1
# ═══════════════════════════════════════════════════════════════

# 定义导航段列表
waypoints = [
    ("P1→P2", WP2_X, WP2_Y),
    ("P2→P3", WP3_X, WP3_Y),
    ("P3→P4", WP4_X, WP4_Y),
    ("P4→P5", WP5_X, WP5_Y),
    ("P5→P1", pos1_x, pos1_y),   # 最后一段回到位置1
]

mission_start = time.ticks_ms()
segment_results = []

print("\n" + "=" * 50)
print("  STARTING NAVIGATION")
print("  Mission timeout: {}s | Phase timeout: {}s".format(
    MISSION_TIMEOUT_S, PHASE_TIMEOUT_S))
print("=" * 50)

time.sleep(1)

for seg_label, seg_x, seg_y in waypoints:
    # 检查总任务超时
    elapsed_total = time.ticks_diff(time.ticks_ms(), mission_start) / 1000.0
    if elapsed_total > MISSION_TIMEOUT_S:
        print("\n=== MISSION TIMEOUT ({:.0f}s) ===".format(elapsed_total))
        break

    # 打印当前段信息
    curr_x, curr_y = uwb.get_position()
    seg_dist = calc_distance(curr_x, curr_y, seg_x, seg_y)
    seg_angle = calc_target_angle(curr_x, curr_y, seg_x, seg_y)
    print("\n┌{:─^48}┐".format(" SEGMENT: " + seg_label + " "))
    print("│  From: ({:7.1f}, {:7.1f}) cm                    │".format(curr_x, curr_y))
    print("│  To:   ({:7.1f}, {:7.1f}) cm                    │".format(seg_x, seg_y))
    print("│  Dist: {:6.1f} cm  Angle: {:+.0f}°                   │".format(seg_dist, seg_angle))
    print("└{:─^48}┘".format(""))

    # ── 段内重试循环（应对 UWB 噪声/超调） ──
    success = False
    seg_from_x, seg_from_y = curr_x, curr_y
    for attempt in range(MAX_SEGMENT_RETRIES + 1):
        if attempt > 0:
            print("  [{:s}] Retrying ({}/{}) in {}ms...".format(
                seg_label, attempt, MAX_SEGMENT_RETRIES, RETRY_COOLDOWN_MS))
            time.sleep_ms(RETRY_COOLDOWN_MS)

        success = move_to_target_xy(uwb, seg_x, seg_y,
                                    "{}({}/{})".format(seg_label, attempt + 1, MAX_SEGMENT_RETRIES + 1))
        if success:
            break

    # 记录结果
    final_x, final_y = uwb.get_position()
    final_dist = calc_distance(final_x, final_y, seg_x, seg_y)
    segment_results.append({
        'label': seg_label,
        'success': success,
        'from': (seg_from_x, seg_from_y),
        'target': (seg_x, seg_y),
        'final': (final_x, final_y),
        'error_cm': final_dist,
    })

    if success:
        print("  [{:s}] ✓ SUCCESS — final err={:.1f}cm".format(seg_label, final_dist))
    else:
        print("  [{:s}] ✗ FAILED after {} attempts — final err={:.1f}cm".format(
            seg_label, MAX_SEGMENT_RETRIES + 1, final_dist))
        break

    # 段间等待：停车稳定
    stop_all()
    time.sleep_ms(300)

    # 检查 SW2
    if switch2.value() != state2:
        print("\n=== SW2 STOP ===")
        break

# ═══════════════════════════════════════════════════════════════
#  任务汇总
# ═══════════════════════════════════════════════════════════════

print("\n" + "=" * 50)
print("  NAVIGATION COMPLETE")
print("=" * 50)

total_elapsed = time.ticks_diff(time.ticks_ms(), mission_start) / 1000.0
success_count = sum(1 for r in segment_results if r['success'])

print("  Start position (P1): ({:.1f}, {:.1f}) cm".format(pos1_x, pos1_y))
print("  Total time: {:.1f}s".format(total_elapsed))
print("  Segments completed: {}/{}".format(success_count, len(waypoints)))
print("")

for r in segment_results:
    status = "✓" if r['success'] else "✗"
    print("  {}  {:8s}  ({:.0f},{:.0f}) → ({:.0f},{:.0f})  err={:.1f}cm".format(
        status, r['label'],
        r['from'][0], r['from'][1],
        r['final'][0], r['final'][1],
        r['error_cm']))

final_x, final_y = uwb.get_position()
return_err = calc_distance(final_x, final_y, pos1_x, pos1_y)
print("\n  Final return error to P1: {:.1f} cm".format(return_err))

# ═══════════════════════════════════════════════════════════════
#  清理
# ═══════════════════════════════════════════════════════════════

stop_all()
enc_ticker.start(10)  # 恢复编码器默认运行状态（与 motor.py 导入时一致）
led.off()

print("\n=== Program Ended ===")
print("Stored positions: {}".format(uwb.get_location_count()))
uwb.stop()
