import gc, time, json, math
from machine import UART, Pin
from motor import (omni_move_by_angle, stop_all,
                   omni_drive_closed_loop,
                   get_encoder_counts, get_encoder_speeds_filtered,
                   reset_encoder_filter, reset_wheel_pi,
                   enc_ticker, ENC_SCALE)
import imu_motion
from imu_motion import (
    update_angle, get_angular_velocity, angular_velocity_control,
    reset_ang_vel_pid, MAX_WZ_DPS,
)

time.sleep_ms(100)

# ── 初始化闭环控制器（参考 uart_move.py）──
stop_all()
enc_ticker.stop()
for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)
reset_encoder_filter()
reset_wheel_pi()

# IMU 航向初始化（参考 uart_move.py）
for _ in range(10):
    d = imu_motion.imu.read()
    update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

last_control_ms = time.ticks_ms()

LED_PIN = 'C4'
SWITCH2_PIN = 'D9'

led     = Pin(LED_PIN, Pin.OUT, value=True)



switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

APPROACH_SPEED    = 0.60
STOP_DIST_M       = 0.20
RESTART_DIST_M    = 0.25
FULL_SPEED_DIST_M = 0.35   # ≥此距离时全速
MIN_APPROACH_SPEED = 0.3  # 最低逼近速度
TARGET_ANCHOR     = "8834"
ANGLE_TIMEOUT_MS  = 800
last_data_ticks   = time.ticks_ms()

ANGLE_DEADBAND = 15    # 方向死区（度），防止 Xcm ≈ 0 时抖动

D_FILT_ALPHA  = 0.15   # 距离低通系数
XY_FILT_ALPHA = 0.10  # Xcm/Ycm 低通系数（防近距离跳变）

# 航向保持参数（参考 uart_move.py move_straight）
DT = 0.02                     # 控制周期 20ms（匹配 UWB 接收帧率）
USE_HEADING_HOLD = True
HDG_KP = 0.02                 # 航向偏差 → 目标 dps 增益（从 0.08 降低，减少抖动）
HDG_DB = 2.0                  # 航向死区（度）（从 0.5 放宽，小偏差不纠）


def _ramp_speed(dist_m):
    """距离越近速度越慢：STOP_DIST(0.20)→0 线性到 FULL_SPEED_DIST(0.50)→APPROACH_SPEED。
    最低不低于 MIN_APPROACH_SPEED，防止 PWM 低于电机死区导致卡住。"""
    if dist_m >= FULL_SPEED_DIST_M:
        return APPROACH_SPEED
    if dist_m <= STOP_DIST_M:
        return 0.0
    t = (dist_m - STOP_DIST_M) / (FULL_SPEED_DIST_M - STOP_DIST_M)
    speed = APPROACH_SPEED * t
    if speed < MIN_APPROACH_SPEED and dist_m > STOP_DIST_M:
        return MIN_APPROACH_SPEED
    return speed


_target_heading = None  # 航向保持目标（None=首次运行捕获）

def _closed_loop_move(speed, angle_deg):
    """
    编码器PID闭环驱动 + IMU航向保持（参考 uart_move.py move_straight）
    - 原始编码器速度（不滤波，避免滞后）
    - IMU yaw → angular_velocity_control → wz 航向保持
    """
    global last_control_ms, _target_heading
    rad = math.radians(angle_deg)
    vx = speed * math.sin(rad)
    vy = speed * math.cos(rad)

    now = time.ticks_ms()
    dt_raw = time.ticks_diff(now, last_control_ms) / 1000.0

    # 断帧 >100ms：编码器脉冲已过时 → 刹车 + 重置定时器，等下一帧
    if dt_raw > 0.1:
        last_control_ms = now
        omni_drive_closed_loop(0, 0, 0, [0, 0, 0, 0], DT)
        return

    # 使用实测 dt 算速度（不 clamp，避免速度算高导致 PID 来回纠）
    dt = max(dt_raw, 0.005)  # 仅防除 0
    last_control_ms = now

    # ── 航向保持：IMU yaw → angular_velocity_control → wz ──
    wz = 0.0
    if USE_HEADING_HOLD:
        d = imu_motion.imu.read()
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
        if _target_heading is None:
            _target_heading = imu_motion.yaw  # 首次运行锁定当前航向
        hdg_err = _target_heading - imu_motion.yaw
        while hdg_err > 180:
            hdg_err -= 360
        while hdg_err < -180:
            hdg_err += 360
        if abs(hdg_err) > HDG_DB:
            target_dps = hdg_err * HDG_KP * MAX_WZ_DPS
            target_dps = max(-180, min(target_dps, 180))
            wz = angular_velocity_control(target_dps, get_angular_velocity(), dt)
        else:
            reset_ang_vel_pid()

    # ── 原始编码器速度（不滤波，匹配 uart_move.py）──
    raw_counts = get_encoder_counts()
    raw_speeds = [raw_counts[i] / ENC_SCALE[i] / dt for i in range(4)]

    omni_drive_closed_loop(vx, vy, wz, raw_speeds, dt)


d_filt = None      # None = 未收到首帧，首帧直接赋实测值
x_filt = None      # Xcm 滤波状态
y_filt = None      # Ycm 滤波状态
is_stopped = False

uart = UART(0)
uart.init(baudrate=115200, bits=8, parity=None, stop=1)

rx_line = bytearray()

def parse_json_line(line_str):
    try:
        idx = line_str.find('{')
        if idx < 0:
            return None
        return json.loads(line_str[idx:])
    except:
        return None

print("=== UWB Receive Test (UART7 115200) ===")
print("Waiting for UWB data...")

frame_count = 0
timeout_stopped = False
loop_count = 0

while True:
    if time.ticks_diff(time.ticks_ms(), last_data_ticks) > ANGLE_TIMEOUT_MS:
        if not timeout_stopped:
            stop_all()
            timeout_stopped = True

    if switch2.value() != state2:
        stop_all()
        enc_ticker.start(10)  # 恢复编码器自动采集（参考 uart_move.py）
        print("Test program stop.")
        break

    if uart.any():
        raw = uart.read(uart.any())
        if raw:
            for i in range(len(raw)):
                b = raw[i]

                if b == 0x0D or b == 0x0A:
                    if len(rx_line) > 0:
                        line_str = ''
                        for c in rx_line:
                            line_str += chr(c)

                        data = parse_json_line(line_str)
                        if data and 'TWR' in data:
                            frame_count += 1
                            twr = data['TWR']
                            anchor = twr.get('a16', '?')
                            d_cm = twr.get('D', 0)
                            x_cm = twr.get('Xcm', 0)
                            y_cm = twr.get('Ycm', 0)
                            last_data_ticks = time.ticks_ms()
                            timeout_stopped = False

                            # --- Xcm/Ycm 低通滤波（防近距离单帧跳变） ---
                            if x_filt is None:
                                x_filt = float(x_cm)
                                y_filt = float(y_cm)
                            else:
                                x_filt = XY_FILT_ALPHA * x_cm + (1 - XY_FILT_ALPHA) * x_filt
                                y_filt = XY_FILT_ALPHA * y_cm + (1 - XY_FILT_ALPHA) * y_filt

                            # 方向角 = atan2(滤波后坐标)，正=右侧，负=左侧
                            angle_to_target = math.atan2(-x_filt, y_filt) * 180.0 / math.pi
                            # 转成电机坐标系（0°=右、90°=前）
                            angle_for_motor = 90 - angle_to_target
                            if abs(angle_for_motor) < ANGLE_DEADBAND:
                                angle_for_motor = 0

                            # 距离用 D（飞行时间直测，近距离更可靠），hypot 仅诊断
                            coord_d = math.sqrt(x_cm * x_cm + y_cm * y_cm)
                            if d_filt is None:
                                d_filt = float(d_cm)
                            else:
                                d_filt = D_FILT_ALPHA * d_cm + (1 - D_FILT_ALPHA) * d_filt
                            dist_m_filt = d_filt / 100.0

                            print("[{}] a={} D={} hypot={:.0f} Df={:.1f} X={} Y={} xf={:.0f} yf={:.0f} ang={:+.0f}° → mot={:+.0f}° speed={:.2f}".format(
                                frame_count, anchor, d_cm, coord_d, d_filt,
                                x_cm, y_cm, x_filt, y_filt, angle_to_target, angle_for_motor,
                                _ramp_speed(dist_m_filt)))

                            if TARGET_ANCHOR is not None and str(anchor) != TARGET_ANCHOR:
                                pass
                            elif is_stopped:
                                if dist_m_filt > RESTART_DIST_M:
                                    is_stopped = False
                                    _target_heading = None  # 重新捕获航向
                                    reset_wheel_pi()
                                    reset_ang_vel_pid()
                                    speed = _ramp_speed(dist_m_filt)
                                    _closed_loop_move(speed, angle_for_motor)
                            elif dist_m_filt <= STOP_DIST_M:
                                is_stopped = True
                                _target_heading = None
                                reset_ang_vel_pid()
                                reset_wheel_pi()
                                stop_all()
                            else:
                                speed = _ramp_speed(dist_m_filt)
                                _closed_loop_move(speed, angle_for_motor)

                            led.toggle()

                        rx_line = bytearray()
                    continue

                rx_line.append(b)
                if len(rx_line) > 200:
                    rx_line = bytearray()

    loop_count += 1
    if loop_count % 50 == 0:
        gc.collect()
    time.sleep_ms(1)
