"""
uwb_tracker.py — UWB 跟随控制器（重写自 uwb_following.py）
【层】应用层
【功能】
  - 通过 UART0 (115200) 接收 TWR 基站数据
  - 解析 JSON 帧，提取距离和相对坐标
  - 互补滤波器平滑数据
  - 状态机：跟随 → 停止 → 重新跟随
  - IMU 航向纠偏 + 角速度闭环 + 编码器闭环驱动
【使用】
  follower = UWBFollower()
  while True:
      follower.step()          # 内部处理 UART + 滤波 + 状态机 + 驱动
      if follower.is_timeout():
          stop_all()
      time.sleep_ms(10)
【依赖】motor.py, imu_motion.py
"""

import gc, time, math, json
from machine import UART
from motor import (
    stop_all, omni_drive_closed_loop,
    get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
    enc_ticker, ENC_SCALE,
)
import imu_motion
from imu_motion import (
    update_angle, get_angular_velocity, angular_velocity_control,
    reset_ang_vel_pid,
)


class UWBFollower:
    """UWB 跟随控制器 — 连续闭环跟随，集成 IMU 航向纠偏 + 闭环驱动。"""

    # ── 状态常量 ──
    STATE_FOLLOW  = 0
    STATE_STOPPED = 1

    # ── 运动参数 ──
    APPROACH_SPEED     = 0.60
    STOP_DIST_M        = 0.20
    RESTART_DIST_M     = 0.25
    FULL_SPEED_DIST_M  = 0.35
    MIN_APPROACH_SPEED = 0.30

    # ── 滤波系数 ──
    D_FILT_ALPHA    = 0.15
    XY_FILT_ALPHA   = 0.10
    ANGLE_FILT_ALPHA = 0.10

    # ── 航向纠偏 ──
    ROT_KP       = 2.0
    ROT_DEADBAND = 3.0
    ROT_MAX_RATE = 200
    ROT_MIN_RATE = 25

    # ── 超时 ──
    TIMEOUT_MS = 800

    # ── 控制周期 ──
    CTRL_INTERVAL_S = 0.02  # 20ms

    def __init__(self, uart_id=0, baudrate=115200, target_anchor="8834"):
        # ── UART 初始化 ──
        self._uart = UART(uart_id)
        self._uart.init(baudrate=baudrate, bits=8, parity=None, stop=1)
        self._rx_line = bytearray()
        self._target_anchor = target_anchor

        # ── 编码器（停 ticker 后手动管理） ──
        enc_ticker.stop()
        for _ in range(5):
            _ = get_encoder_counts()
            time.sleep_ms(10)
        reset_encoder_filter()
        reset_wheel_pi()

        # ── IMU 热身（建立 yaw 初始基准） ──
        for _ in range(10):
            d = imu_motion.imu.read()
            update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
            time.sleep_ms(10)

        # ── 状态变量 ──
        self._state = self.STATE_FOLLOW
        self._d_filt = None
        self._x_filt = None
        self._y_filt = None
        self._angle_filt = None
        self._last_data_ticks = time.ticks_ms()
        self._last_control_ms = time.ticks_ms()
        self._timeout_stopped = False
        self._frame_count = 0

        print("=== UWBFollower ready (UART{} {} baud, anchor={}) ===".format(
            uart_id, baudrate, target_anchor))

    # ============================================================
    #  内部：速度斜坡
    # ============================================================
    def _ramp_speed(self, dist_m):
        """距离越近速度越慢，线性衰减。"""
        if dist_m >= self.FULL_SPEED_DIST_M:
            return self.APPROACH_SPEED
        if dist_m <= self.STOP_DIST_M:
            return 0.0
        t = (dist_m - self.STOP_DIST_M) / (self.FULL_SPEED_DIST_M - self.STOP_DIST_M)
        speed = self.APPROACH_SPEED * t
        if speed < self.MIN_APPROACH_SPEED and dist_m > self.STOP_DIST_M:
            return self.MIN_APPROACH_SPEED
        return speed

    # ============================================================
    #  内部：JSON 解析
    # ============================================================
    @staticmethod
    def _parse_json_line(line_str):
        try:
            idx = line_str.find('{')
            if idx < 0:
                return None
            return json.loads(line_str[idx:])
        except Exception:
            return None

    # ============================================================
    #  公共接口：单次步进（每次主循环调用一次）
    # ============================================================
    def step(self):
        """单次迭代：读 UART → 滤波 → 状态机 → 驱动电机。

        可在主循环中每 10ms 调用一次，内部按 20ms 周期执行闭环控制。
        """
        # ── 超时保护 ──
        if time.ticks_diff(time.ticks_ms(), self._last_data_ticks) > self.TIMEOUT_MS:
            if not self._timeout_stopped:
                stop_all()
                self._timeout_stopped = True
            return

        # ── 非阻塞读取 UART ──
        if self._uart is not None and self._uart.any():
            raw = self._uart.read(self._uart.any())
            if raw:
                for i in range(len(raw)):
                    b = raw[i]

                    # 行结束符
                    if b == 0x0D or b == 0x0A:
                        if len(self._rx_line) > 0:
                            line_str = ''
                            for c in self._rx_line:
                                line_str += chr(c)
                            self._process_line(line_str)
                            self._rx_line = bytearray()
                        continue

                    self._rx_line.append(b)
                    if len(self._rx_line) > 200:
                        self._rx_line = bytearray()

    # ============================================================
    #  内部：处理一行 UWB 数据（滤波 + 状态机 + 驱动）
    # ============================================================
    def _process_line(self, line_str):
        data = self._parse_json_line(line_str)
        if data is None or 'TWR' not in data:
            return

        twr = data['TWR']
        anchor = twr.get('a16', '?')
        d_cm  = twr.get('D', 0)
        x_cm  = twr.get('Xcm', 0)
        y_cm  = twr.get('Ycm', 0)

        # 跳过非目标锚点
        if self._target_anchor is not None and str(anchor) != self._target_anchor:
            return

        # ── 更新计时 ──
        self._frame_count += 1
        self._last_data_ticks = time.ticks_ms()
        self._timeout_stopped = False

        # ── 低通滤波 ──
        if self._x_filt is None:
            self._x_filt = float(x_cm)
            self._y_filt = float(y_cm)
        else:
            self._x_filt = self.XY_FILT_ALPHA * x_cm + (1 - self.XY_FILT_ALPHA) * self._x_filt
            self._y_filt = self.XY_FILT_ALPHA * y_cm + (1 - self.XY_FILT_ALPHA) * self._y_filt

        angle_to_target = math.atan2(-self._x_filt, self._y_filt) * 180.0 / math.pi
        if self._angle_filt is None:
            self._angle_filt = angle_to_target
        else:
            self._angle_filt = self.ANGLE_FILT_ALPHA * angle_to_target + (
                1 - self.ANGLE_FILT_ALPHA) * self._angle_filt

        if self._d_filt is None:
            self._d_filt = float(d_cm)
        else:
            self._d_filt = self.D_FILT_ALPHA * d_cm + (1 - self.D_FILT_ALPHA) * self._d_filt
        dist_m_filt = self._d_filt / 100.0

        # ── Debug 打印 ──
        print("[{}] a={} D={} Df={:.1f} X={} Y={} ang={:+.0f}° ang_f={:+.0f}° spd={:.2f} state={}".format(
            self._frame_count, anchor, d_cm, self._d_filt,
            x_cm, y_cm, angle_to_target, self._angle_filt,
            self._ramp_speed(dist_m_filt), self._state))

        # ── 状态机 ──
        if self._state == self.STATE_FOLLOW:
            # == 跟随态 ==
            if dist_m_filt <= self.STOP_DIST_M:
                # 到达 → 停止
                self._state = self.STATE_STOPPED
                reset_ang_vel_pid()
                reset_wheel_pi()
                stop_all()
                return

            # ── 控制周期门控（20ms） ──
            now = time.ticks_ms()
            dt_raw = time.ticks_diff(now, self._last_control_ms) / 1000.0
            if dt_raw > 0.2:  # 首帧/跳帧跳过
                self._last_control_ms = now
                return
            dt = max(dt_raw, 0.005)
            self._last_control_ms = now

            # ── IMU 更新 ──
            d = imu_motion.imu.read()
            update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

            # ── 航向偏差计算 ──
            target_yaw = imu_motion.yaw - self._angle_filt
            while target_yaw > 180:
                target_yaw -= 360
            while target_yaw < -180:
                target_yaw += 360
            yaw_err = target_yaw - imu_motion.yaw
            while yaw_err > 180:
                yaw_err -= 360
            while yaw_err < -180:
                yaw_err += 360

            # ── 航向纠偏（角速度闭环） ──
            if abs(yaw_err) > self.ROT_DEADBAND:
                target_dps = yaw_err * self.ROT_KP
                if abs(target_dps) < self.ROT_MIN_RATE:
                    target_dps = self.ROT_MIN_RATE if target_dps >= 0 else -self.ROT_MIN_RATE
                target_dps = max(-self.ROT_MAX_RATE, min(target_dps, self.ROT_MAX_RATE))
                wz = angular_velocity_control(target_dps, get_angular_velocity(), dt)
            else:
                reset_ang_vel_pid()
                wz = 0.0

            # ── 速度按航向对齐度 cos(yaw_err) 缩放 ──
            alignment = max(0.0, math.cos(math.radians(abs(yaw_err))))
            spd = self._ramp_speed(dist_m_filt) * alignment
            if spd < self.MIN_APPROACH_SPEED and dist_m_filt > self.STOP_DIST_M and alignment > 0.5:
                spd = self.MIN_APPROACH_SPEED

            # ── 编码器闭环驱动 ──
            rc = get_encoder_counts()
            rs = [rc[i] / ENC_SCALE[i] / dt for i in range(4)]
            omni_drive_closed_loop(spd, 0, wz, rs, dt)

        elif self._state == self.STATE_STOPPED:
            # == 停止态：等目标移开 > RESTART_DIST 才重新跟随 ==
            if dist_m_filt > self.RESTART_DIST_M:
                self._state = self.STATE_FOLLOW
                reset_wheel_pi()
                reset_ang_vel_pid()

    # ============================================================
    #  公共查询接口
    # ============================================================

    def is_timeout(self):
        """超过 TIMEOUT_MS 未收到有效帧返回 True。"""
        if time.ticks_diff(time.ticks_ms(), self._last_data_ticks) > self.TIMEOUT_MS:
            if not self._timeout_stopped:
                self._timeout_stopped = True
                print("UWBFollower: timeout — no data for {}ms".format(self.TIMEOUT_MS))
            return True
        return False

    def get_distance(self):
        """返回滤波后距离（米），未收到首帧返回 inf。"""
        if self._d_filt is None:
            return float('inf')
        return self._d_filt / 100.0

    def get_angle(self):
        """返回锚点相对车头的滤波后方向角（度），正=右侧。"""
        return self._angle_filt if self._angle_filt is not None else 0.0

    def is_near_target(self):
        """返回是否已进入停车距离。"""
        if self._frame_count == 0:
            return False
        return self._d_filt / 100.0 <= self.STOP_DIST_M

    # ============================================================
    #  清理
    # ============================================================
    def stop(self):
        """停止 UART + 停电机 + 恢复编码器 ticker。"""
        print("UWBFollower: stopping...")
        stop_all()
        enc_ticker.start(10)
        if self._uart is not None:
            try:
                self._uart.deinit()
            except Exception:
                pass
            self._uart = None
        print("UWBFollower: stopped.")
