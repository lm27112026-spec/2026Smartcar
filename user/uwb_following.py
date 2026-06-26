"""
uwb_following.py — UWB 锚点跟随（全向平移逼近）- 右侧安装适配版
【流程】
   1. FOLLOW：根据锚点实时位置分解为前向/侧向速度，全向平移靠近
   2. STOPPED：到达 20cm → 等移开 >25cm 重新跟随
【安全】SWITCH2 随时终止
【依赖】motor.py, imu_motion.py
"""

import gc, time, json, math
from machine import UART, Pin
from motor import (stop_all,
                   omni_drive_closed_loop,
                   get_encoder_counts,
                   reset_encoder_filter, reset_wheel_pi,
                   enc_ticker, ENC_SCALE)
import imu_motion
from imu_motion import (
    update_angle, get_angular_velocity, angular_velocity_control,
    reset_ang_vel_pid,
)

# ── 状态机 ──
STATE_FOLLOW   = 0
STATE_STOPPED  = 1

# ── 硬件 ──
LED_PIN = 'C4'
SWITCH2_PIN = 'D9'
led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ── 运动参数 ──
APPROACH_SPEED    = 0.60
STOP_DIST_M       = 0.20
RESTART_DIST_M    = 0.25
FULL_SPEED_DIST_M = 0.35
MIN_APPROACH_SPEED = 0.30
TARGET_ANCHOR     = "8834"
ANGLE_TIMEOUT_MS  = 800
DT                = 0.02

# ── 右侧安装 UWB 物理偏移参数（单位：cm） ──
UWB_OFFSET_X      = 15.0
UWB_OFFSET_Y      = 0.0

# ── 滤波 ──
D_FILT_ALPHA     = 0.15
XY_FILT_ALPHA    = 0.10
ANGLE_FILT_ALPHA = 0.10

# ── 航向纠偏参数 ──
ROT_KP       = 2.0
ROT_DEADBAND = 3.0
ROT_MAX_RATE = 200
ROT_MIN_RATE = 25

# ── 全局状态 ──
_state = STATE_FOLLOW
d_filt = None
x_filt = None
y_filt = None
angle_filt = None
last_data_ticks = time.ticks_ms()
last_control_ms = time.ticks_ms()
timeout_stopped = False


# ============================================================
#  初始化
# ============================================================
time.sleep_ms(100)
stop_all()
enc_ticker.stop()
for _ in range(5):
    _ = get_encoder_counts()
    time.sleep_ms(10)
reset_encoder_filter()
reset_wheel_pi()

for _ in range(10):
    d = imu_motion.imu.read()
    update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(10)

# ============================================================
#  速度斜坡
# ============================================================
def _ramp_speed(dist_m):
    if dist_m >= FULL_SPEED_DIST_M:
        return APPROACH_SPEED
    if dist_m <= STOP_DIST_M:
        return 0.0
    t = (dist_m - STOP_DIST_M) / (FULL_SPEED_DIST_M - STOP_DIST_M)
    speed = APPROACH_SPEED * t
    if speed < MIN_APPROACH_SPEED and dist_m > STOP_DIST_M:
        return MIN_APPROACH_SPEED
    return speed


# ============================================================
#  UART 接收
# ============================================================
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


# ============================================================
#  主循环
# ============================================================
print("=== UWB Following (Continuous - Right Mounted) ===")
print("Waiting for UWB data...")
frame_count = 0
loop_count = 0

while True:
    # ── 超时保护 ──
    if time.ticks_diff(time.ticks_ms(), last_data_ticks) > ANGLE_TIMEOUT_MS:
        if not timeout_stopped:
            stop_all()
            timeout_stopped = True

    if switch2.value() != state2:
        stop_all()
        enc_ticker.start(10)
        print("Test program stop.")
        break

    if uart.any():
        raw = uart.read(uart.any())
        if raw:
            for i in range(len(raw)):
                b = raw[i]

                if b == 0x0D or b == 0x0A:
                    if len(rx_line) > 0:
                        # ── 高效字节转字符串（避免逐字拼接吃内存） ──
                        try:
                            line_str = bytes(rx_line).decode()
                        except:
                            rx_line = bytearray()
                            continue

                        data = parse_json_line(line_str)
                        if data and 'TWR' in data:
                            frame_count += 1
                            twr = data['TWR']
                            anchor = twr.get('a16', '?')

                            # 获取原始 UWB 模块坐标
                            x_uwb = twr.get('Xcm', 0)
                            y_uwb = twr.get('Ycm', 0)

                            # ── 坐标系转换：UWB 右视 → 小车中心坐标系 ──
                            # 锚点正前(+Y_uwb) = 小车右侧(+X_robot)
                            x_cm = y_uwb + UWB_OFFSET_X
                            y_cm = -x_uwb + UWB_OFFSET_Y

                            d_cm = math.sqrt(x_cm**2 + y_cm**2)

                            last_data_ticks = time.ticks_ms()
                            timeout_stopped = False

                            # ── 滤波 ──
                            if x_filt is None:
                                x_filt = float(x_cm)
                                y_filt = float(y_cm)
                            else:
                                x_filt = XY_FILT_ALPHA * x_cm + (1 - XY_FILT_ALPHA) * x_filt
                                y_filt = XY_FILT_ALPHA * y_cm + (1 - XY_FILT_ALPHA) * y_filt

                            # ── 角度：锚点相对角（0°=锚点正前方，匹配 uwb_position.py） ──
                            angle_to_target = math.atan2(x_uwb, y_uwb) * 180.0 / math.pi
                            if angle_filt is None:
                                angle_filt = angle_to_target
                            else:
                                angle_filt = ANGLE_FILT_ALPHA * angle_to_target + (1 - ANGLE_FILT_ALPHA) * angle_filt

                            if d_filt is None:
                                d_filt = float(d_cm)
                            else:
                                d_filt = D_FILT_ALPHA * d_cm + (1 - D_FILT_ALPHA) * d_filt
                            dist_m_filt = d_filt / 100.0

                            print("[{}] a={} D_f={:.1f} X_f={:.1f} Y_f={:.1f} ang_f={:+.0f}° spd={:.2f} state={}".format(
                                frame_count, anchor, d_filt,
                                x_filt, y_filt, angle_filt, _ramp_speed(dist_m_filt),
                                _state))

                            # ── 跳过其它锚点 ──
                            if TARGET_ANCHOR is not None and str(anchor) != TARGET_ANCHOR:
                                continue

                            # ── 状态机 ──
                            if _state == STATE_FOLLOW:
                                if dist_m_filt <= STOP_DIST_M:
                                    _state = STATE_STOPPED
                                    reset_ang_vel_pid()
                                    reset_wheel_pi()
                                    stop_all()
                                    continue

                                # ── 控制周期 ──
                                now = time.ticks_ms()
                                dt_raw = time.ticks_diff(now, last_control_ms) / 1000.0
                                if dt_raw > 0.2:
                                    last_control_ms = now
                                    continue
                                dt = max(dt_raw, 0.005)
                                last_control_ms = now

                                # ── IMU 读取 ──
                                d = imu_motion.imu.read()
                                update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

                                # ── 航向保持（不主动面向 tag） ──
                                target_yaw = imu_motion.yaw - angle_filt
                                while target_yaw > 180:
                                    target_yaw -= 360
                                while target_yaw < -180:
                                    target_yaw += 360
                                yaw_err = target_yaw - imu_motion.yaw
                                while yaw_err > 180:
                                    yaw_err -= 360
                                while yaw_err < -180:
                                    yaw_err += 360

                                if abs(yaw_err) > ROT_DEADBAND:
                                    target_dps = yaw_err * ROT_KP
                                    if abs(target_dps) < ROT_MIN_RATE:
                                        target_dps = ROT_MIN_RATE if target_dps >= 0 else -ROT_MIN_RATE
                                    target_dps = max(-ROT_MAX_RATE, min(target_dps, ROT_MAX_RATE))
                                    wz = angular_velocity_control(target_dps, get_angular_velocity(), dt)
                                else:
                                    reset_ang_vel_pid()
                                    wz = 0.0

                                # ── 速度分解：锚点相对角 → 小车坐标系速度分量 ──
                                # 锚点正前(+Y_uwb)=小车右侧，故偏移 +90°
                                approach_spd = _ramp_speed(dist_m_filt)
                                robot_angle_rad = math.radians(angle_filt + 90.0)
                                fw_spd = -approach_spd * math.cos(robot_angle_rad)
                                lat_spd =  approach_spd * math.sin(robot_angle_rad)

                                # ── 编码器PID闭环驱动 ──
                                rc = get_encoder_counts()
                                rs = [rc[i] / ENC_SCALE[i] / dt for i in range(4)]
                                omni_drive_closed_loop(fw_spd, lat_spd, wz, rs, dt)

                            elif _state == STATE_STOPPED:
                                if dist_m_filt > RESTART_DIST_M:
                                    _state = STATE_FOLLOW
                                    reset_wheel_pi()
                                    reset_ang_vel_pid()

                            led.toggle()

                        rx_line = bytearray()
                    continue

                rx_line.append(b)
                if len(rx_line) > 200:
                    rx_line = bytearray()

    loop_count += 1
    if loop_count % 10 == 0:
        gc.collect()
    time.sleep_ms(1)
