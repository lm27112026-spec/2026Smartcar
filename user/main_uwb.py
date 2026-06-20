"""
main_uwb.py — UWB 定位闭环行进
【功能】
  - 通过 UWB 获取当前坐标 (x, y)
  - 根据 UWB 坐标直接计算 vx, vy 运动分量
  - 编码器闭环驱动
【运动学映射（实测）】
  - vx 对应 UWB Y 轴（负方向）：vx>0 → Y 减小
  - vy 对应 UWB X 轴（正方向）：vy>0 → X 增大
【安全】SW2 随时终止，超时保护
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

# ── 目标坐标 (cm，相对于 UWB 锚点) ──
TARGET_X = 0.0
TARGET_Y = 40.0

# ── 参数 ──
DT = 0.01
MISSION_TIMEOUT_S = 60
PHASE_TIMEOUT_S = 20
PRINT_MS = 300

# ── 位置 PID ──
POS_KP = 0.015      # 位置增益
POS_KI = 0.0        # 积分增益（UWB 有噪声，不用积分）
POS_DB = 0.5        # 到位死区 (cm)
MAX_SPEED = 0.3    # 最大速度 (m/s)

# ── 航向 PID ──
HDG_KP = 1.5        # 航向偏差(°) → 目标 dps
HDG_DB = 1.0        # 航向死区 (度)

# ── UWB ──
UWB_TARGET_ANCHOR = "8834"


# ============================================================
#  初始化
# ============================================================
print("\n" + "=" * 50)
print("  UWB Position Navigation (Direct XY Control)")
print("  Target: ({:.1f}, {:.1f}) cm".format(TARGET_X, TARGET_Y))
print("=" * 50)

stop_all()
time.sleep_ms(50)

# 启动编码器 ticker（闭环控制需要）
enc_ticker.start(10)
time.sleep_ms(50)

# 清空编码器残余值
for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)

stop_imu_ticker()
time.sleep_ms(10)

for _ in range(10):
    d = imu_motion.imu.read()
    if d is not None:
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

uwb = UWBPosition(target_anchor=UWB_TARGET_ANCHOR)


# ============================================================
#  辅助函数
# ============================================================
def normalize_angle(angle):
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle


def calc_target_angle(from_x, from_y, to_x, to_y):
    """计算从 (from_x, from_y) 到 (to_x, to_y) 的目标角度（UWB 坐标系）"""
    dx = to_x - from_x
    dy = to_y - from_y
    # atan2(-dx, dy)：与 uwb_following 一致
    return math.atan2(-dx, dy) * 180.0 / math.pi


def calc_distance(from_x, from_y, to_x, to_y):
    """计算两点间距离"""
    dx = to_x - from_x
    dy = to_y - from_y
    return math.sqrt(dx * dx + dy * dy)


# ============================================================
#  直接 XY 控制（无旋转阶段，直接根据 UWB 坐标计算运动）
# ============================================================
def move_to_target_xy(target_x, target_y, label):
    """
    直接根据 UWB 坐标计算运动方向并移动到目标
    返回 True=成功
    """
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)
    
    # P 控制器（简单比例控制，无积分）
    reset_ang_vel_pid()
    
    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    last_uwb_update_ms = start_ms
    
    # 锁定当前航向
    target_heading = imu_motion.yaw
    
    print("\n  ── [{:s}] move to ({:.0f}, {:.0f}) ──".format(label, target_x, target_y))
    
    while True:
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, start_ms) / 1000.0
        
        if elapsed_s > PHASE_TIMEOUT_S:
            print("  TIMEOUT")
            return False
        
        if switch2.value() != state2:
            print("  SW2 stop")
            return False
        
        # 每 50ms 接收一次 UWB 数据
        if time.ticks_diff(now_ms, last_uwb_update_ms) >= 50:
            uwb.step()
            last_uwb_update_ms = now_ms
        
        # 获取当前 UWB 坐标
        curr_x, curr_y = uwb.get_position()
        
        # 计算到目标的误差
        error_x = target_x - curr_x
        error_y = target_y - curr_y
        dist = math.sqrt(error_x * error_x + error_y * error_y)
        
        # 到位判定
        if dist < POS_DB:
            break
        
        # IMU 读取 + 航向保持
        wz = 0.0
        d = imu_motion.imu.read()
        if d is not None:
            update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
            
            hdg_err = normalize_angle(target_heading - imu_motion.yaw)
            if abs(hdg_err) > HDG_DB:
                target_dps = hdg_err * HDG_KP
                target_dps = max(-180, min(target_dps, 180))
                wz = angular_velocity_control(target_dps, get_angular_velocity(), DT)
        
        # 简单 P 控制：速度 = KP × 误差
        # 误差在 UWB 世界坐标系，需要转换到车体坐标系
        # UWB 坐标系：前进 = -Y，右移 = +X
        # 车体坐标系：前进 = +vx，右移 = +vy
        
        # 世界坐标系误差
        world_err_x = error_x  # UWB X 方向误差
        world_err_y = error_y  # UWB Y 方向误差
        
        # 先旋转到车体坐标系（减去 yaw 角）
        yaw_rad = math.radians(imu_motion.yaw)
        rot_x = world_err_x * math.cos(yaw_rad) + world_err_y * math.sin(yaw_rad)
        rot_y = -world_err_x * math.sin(yaw_rad) + world_err_y * math.cos(yaw_rad)
        
        # 再映射到车体运动方向（UWB: 前进=-Y, 右移=+X → 车体: 前进=+vx, 右移=+vy）
        body_fwd = -rot_y   # UWB -Y 方向 = 车体前进 (+vx)
        body_right = rot_x  # UWB +X 方向 = 车体右移 (+vy)
        
        # 车体坐标系速度
        vx_cmd = body_fwd * POS_KP
        vy_cmd = body_right * POS_KP
        
        # 限幅
        speed = math.sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd)
        if speed > MAX_SPEED:
            vx_cmd = vx_cmd / speed * MAX_SPEED
            vy_cmd = vy_cmd / speed * MAX_SPEED
        
        # 限幅
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
    
    # 到位后停止并重新确认
    omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
    time.sleep_ms(200)  # 停止 200ms 等待 UWB 稳定
    
    # 重新读取坐标确认
    uwb.step()
    curr_x, curr_y = uwb.get_position()
    final_dist = calc_distance(curr_x, curr_y, target_x, target_y)
    
    if final_dist < POS_DB * 2:  # 允许 2 倍死区
        print("  >>> [{:s}] done: ({:.1f}, {:.1f}) err={:.1f}cm <<<".format(
            label, curr_x, curr_y, final_dist))
        return True
    else:
        print("  >>> [{:s}] overshoot: ({:.1f}, {:.1f}) err={:.1f}cm, retrying... <<<".format(
            label, curr_x, curr_y, final_dist))
        return False


# ============================================================
#  主循环
# ============================================================
print("\nWaiting for UWB data...")
print("Press SW2 to exit\n")

state = 0  # 0=等待数据, 1=移动中, 2=到达
retry_count = 0
MAX_RETRIES = 3

while True:
    if switch2.value() != state2:
        print("\n=== SW2 STOP ===")
        break
    
    if not uwb.step():
        print("\n=== SW2 from UWB ===")
        break
    
    curr_x, curr_y = uwb.get_position()
    
    if state == 0:
        # 等待 UWB 数据
        if uwb.get_frame_count() > 0:
            dist = calc_distance(curr_x, curr_y, TARGET_X, TARGET_Y)
            target_angle = calc_target_angle(curr_x, curr_y, TARGET_X, TARGET_Y)
            
            print("\nCurrent: ({:.1f}, {:.1f})".format(curr_x, curr_y))
            print("Target:  ({:.1f}, {:.1f})".format(TARGET_X, TARGET_Y))
            print("Distance: {:.1f} cm, Angle: {:+.1f}°".format(dist, target_angle))
            
            if dist < POS_DB:
                print("\n=== ALREADY AT TARGET ===")
                break
            
            state = 1
    
    elif state == 1:
        # 移动中（允许重试）
        if move_to_target_xy(TARGET_X, TARGET_Y, "MOVE"):
            print("\n=== TARGET REACHED ===")
            break
        else:
            # 超调了，等待 UWB 更新后重试
            retry_count += 1
            if retry_count >= MAX_RETRIES:
                print("\n=== MAX RETRIES REACHED ===")
                break
            print("  Retrying ({}/{}) in 500ms...".format(retry_count, MAX_RETRIES))
            time.sleep_ms(500)
            continue
    
    time.sleep_ms(10)


# ============================================================
#  清理
# ============================================================
stop_all()
enc_ticker.start(10)
led.off()

print("\n=== Program Ended ===")
print("Final position: ({:.1f}, {:.1f})".format(*uwb.get_position()))
print("Stored positions: {}".format(uwb.get_location_count()))
uwb.stop()
