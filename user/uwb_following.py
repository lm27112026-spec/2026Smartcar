"""
uwb_following.py — UWB 锚点跟随（实时调向+逼近）
【流程】
   1. FOLLOW：每帧根据锚点实时位置计算航向偏差，边走边调向
      速度按 cos(yaw_err) 缩放：正对全速、侧向减速、背对原地旋转
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
STATE_FOLLOW   = 0  # 跟随：边走向锚点边实时调整航向
STATE_STOPPED  = 1  # 到达

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
FULL_SPEED_DIST_M = 0.35   # ≥35cm 全速
MIN_APPROACH_SPEED = 0.30  # 最低逼近速度（防电机死区）
TARGET_ANCHOR     = "8834"
ANGLE_TIMEOUT_MS  = 800
DT                = 0.02   # 控制周期 20ms

# ── 滤波 ──
D_FILT_ALPHA    = 0.15      # 距离低通
XY_FILT_ALPHA   = 0.10      # 坐标低通
ANGLE_FILT_ALPHA = 0.10     # 角度低通（独立于XY，抑制UWB角度跳变）

# ── 航向纠偏参数（跟随阶段用，兼顾旋转+直行） ──
ROT_KP       = 2.0         # 偏差(°) → 目标 dps 增益（边开边转不宜过大）
ROT_DEADBAND = 3.0         # 到位判定（度）
ROT_MAX_RATE = 200         # 最大旋转速度 (dps)
ROT_MIN_RATE = 25          # 最低旋转速度 (dps)，仅死区外生效

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
print("=== UWB Following (Continuous) ===")
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
                        line_str = ''
                        for c in rx_line:
                            line_str += chr(c)

                        data = parse_json_line(line_str)
                        if data and 'TWR' in data:
                            frame_count += 1
                            twr = data['TWR']
                            anchor = twr.get('a16', '?')
                            d_cm  = twr.get('D', 0)
                            x_cm  = twr.get('Xcm', 0)
                            y_cm  = twr.get('Ycm', 0)
                            last_data_ticks = time.ticks_ms()
                            timeout_stopped = False

                            # ── 滤波 ──
                            if x_filt is None:
                                x_filt = float(x_cm)
                                y_filt = float(y_cm)
                            else:
                                x_filt = XY_FILT_ALPHA * x_cm + (1 - XY_FILT_ALPHA) * x_filt
                                y_filt = XY_FILT_ALPHA * y_cm + (1 - XY_FILT_ALPHA) * y_filt

                            angle_to_target = math.atan2(-x_filt, y_filt) * 180.0 / math.pi
                            if angle_filt is None:
                                angle_filt = angle_to_target
                            else:
                                angle_filt = ANGLE_FILT_ALPHA * angle_to_target + (1 - ANGLE_FILT_ALPHA) * angle_filt

                            if d_filt is None:
                                d_filt = float(d_cm)
                            else:
                                d_filt = D_FILT_ALPHA * d_cm + (1 - D_FILT_ALPHA) * d_filt
                            dist_m_filt = d_filt / 100.0

                            print("[{}] a={} D={} Df={:.1f} X={} Y={} ang={:+.0f}° ang_f={:+.0f}° spd={:.2f} state={}".format(
                                frame_count, anchor, d_cm, d_filt,
                                x_cm, y_cm, angle_to_target, angle_filt, _ramp_speed(dist_m_filt),
                                _state))

                            # ── 跳过其它锚点 ──
                            if TARGET_ANCHOR is not None and str(anchor) != TARGET_ANCHOR:
                                continue

                            # ── 状态机 ──
                            if _state == STATE_FOLLOW:
                                # == 跟随：边走向锚点边实时调向 ==
                                if dist_m_filt <= STOP_DIST_M:
                                    _state = STATE_STOPPED
                                    reset_ang_vel_pid()
                                    reset_wheel_pi()
                                    stop_all()
                                    continue

                                # ── 控制周期 ──
                                now = time.ticks_ms()
                                dt_raw = time.ticks_diff(now, last_control_ms) / 1000.0
                                if dt_raw > 0.2:          # 首帧 / 跳帧免执行
                                    last_control_ms = now
                                    continue
                                dt = max(dt_raw, 0.005)
                                last_control_ms = now

                                # ── IMU 读取 ──
                                d = imu_motion.imu.read()
                                update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

                                # ── 目标航向（锚点方向） ──
                                target_yaw = imu_motion.yaw - angle_filt
                                while target_yaw > 180:  target_yaw -= 360
                                while target_yaw < -180: target_yaw += 360
                                yaw_err = target_yaw - imu_motion.yaw
                                while yaw_err > 180:  yaw_err -= 360
                                while yaw_err < -180: yaw_err += 360

                                # ── 航向纠偏（角速度闭环） ──
                                if abs(yaw_err) > ROT_DEADBAND:
                                    target_dps = yaw_err * ROT_KP
                                    if abs(target_dps) < ROT_MIN_RATE:
                                        target_dps = ROT_MIN_RATE if target_dps >= 0 else -ROT_MIN_RATE
                                    target_dps = max(-ROT_MAX_RATE, min(target_dps, ROT_MAX_RATE))
                                    wz = angular_velocity_control(target_dps, get_angular_velocity(), dt)
                                else:
                                    reset_ang_vel_pid()
                                    wz = 0.0

                                # ── 速度按航向对齐度缩放 ──
                                alignment = max(0.0, math.cos(math.radians(abs(yaw_err))))
                                spd = _ramp_speed(dist_m_filt) * alignment
                                if spd < MIN_APPROACH_SPEED and dist_m_filt > STOP_DIST_M and alignment > 0.5:
                                    spd = MIN_APPROACH_SPEED

                                # ── 编码器PID闭环驱动 ──
                                rc = get_encoder_counts()
                                rs = [rc[i] / ENC_SCALE[i] / dt for i in range(4)]
                                omni_drive_closed_loop(spd, 0, wz, rs, dt)

                            elif _state == STATE_STOPPED:
                                # == 到达：等移开才重新跟随 ==
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
    if loop_count % 50 == 0:
        gc.collect()
    time.sleep_ms(1)
