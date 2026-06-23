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
    reset_ang_vel_pid, imu_get_safe,
)


class UWBFollower:
    """UWB 跟随控制器 — 连续闭环跟随，集成 IMU 航向纠偏 + 闭环驱动。"""

    # ── 状态常量 ──
    STATE_FOLLOW  = 0
    STATE_STOPPED = 1

    # ── 运动参数 ──
    APPROACH_SPEED     = 0.60
    STOP_DIST_M        = 0.18
    RESTART_DIST_M     = 0.30
    FULL_SPEED_DIST_M  = 0.35

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
        # 使用 imu_get_safe() 从 PIT3 ticker 缓冲区读取，避免 SPI 总线冲突挂死
        for _ in range(10):
            d = imu_get_safe()
            if d is not None:
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
        """距离越近速度越慢，线性衰减（无硬跳变，motor.py SPD_DEADBAND 处理死区）。"""
        if dist_m >= self.FULL_SPEED_DIST_M:
            return self.APPROACH_SPEED
        if dist_m <= self.STOP_DIST_M:
            return 0.0
        denom = max(self.FULL_SPEED_DIST_M - self.STOP_DIST_M, 0.001)
        t = (dist_m - self.STOP_DIST_M) / denom
        return self.APPROACH_SPEED * t

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
        # ── 超时保护（仅首次超时停车，不 return — UART 始终被轮询以支持恢复） ──
        if time.ticks_diff(time.ticks_ms(), self._last_data_ticks) > self.TIMEOUT_MS:
            if not self._timeout_stopped:
                stop_all()
                self._timeout_stopped = True

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

        # ── 跳过非目标锚点（不计时、不滤波、不控制，避免超时屏蔽）──
        if self._target_anchor is not None and str(anchor) != self._target_anchor:
            return

        # ── 目标锚点：更新计时 ──
        self._frame_count += 1
        if self._frame_count % 50 == 0:
            gc.collect()
        self._last_data_ticks = time.ticks_ms()
        self._timeout_stopped = False

        # ── 低通滤波（仅目标锚点更新，避免非目标锚点污染）──
        if self._x_filt is None:
            self._x_filt = float(x_cm)
            self._y_filt = float(y_cm)
        else:
            self._x_filt = self.XY_FILT_ALPHA * x_cm + (1 - self.XY_FILT_ALPHA) * self._x_filt
            self._y_filt = self.XY_FILT_ALPHA * y_cm + (1 - self.XY_FILT_ALPHA) * self._y_filt

        angle_to_target = math.atan2(self._y_filt, -self._x_filt) * 180.0 / math.pi
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
        spd_raw = self._ramp_speed(dist_m_filt)
        print("[{}] a={} D={} Df={:.1f} X={} Y={} ang={:+.0f}° ang_f={:+.0f}° spd={:.2f} state={}".format(
            self._frame_count, anchor, d_cm, self._d_filt,
            x_cm, y_cm, angle_to_target, self._angle_filt,
            spd_raw, self._state))

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

            # ── IMU 更新（从 PIT3 ticker 缓冲区读取，非阻塞，无 I2C 竞态） ──
            d = imu_get_safe()
            if d is None:
                return  # IMU 读取失败，跳过本次控制周期
            update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

            # ── 航向偏差 ──
            # _angle_filt = atan2(y, -x): 正=目标在左，负=目标在右
            # 航向纠偏需要取反：目标在左(yaw_err>0) → 向左转(wz>0)
            yaw_err = -self._angle_filt

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
            spd = spd_raw * alignment
            # 注：motor.py 的 SPD_DEADBAND (0.005 m/s) 处理低速死区，无需硬兜底

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
        """返回锚点相对车头的滤波后方向角（度）。
        atan2(y, -x) 约定：负=右侧，正=左侧。"""
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
        """停止 UART + 停电机（编码器 ticker 由调用者管理）。"""
        print("UWBFollower: stopping...")
        stop_all()
        if self._uart is not None:
            try:
                self._uart.deinit()
            except Exception:
                pass
            self._uart = None
        print("UWBFollower: stopped.")


