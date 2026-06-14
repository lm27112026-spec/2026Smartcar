"""
robot.py — 机器人集成框架
【层】应用框架层
【功能】
  - 统一状态机管理（5 种运行模式）
  - 安全模式切换（退出时 stop_all + PID 重置）
  - SW2 按键控制（短按循环切换模式，长按急停）
  - LED 状态指示（不同模式不同闪烁频率）
  - 集成看门狗（摄像头数据超时保护）
  - 单次迭代模式调度（非阻塞，Main Loop 统一调度）
【设计原则】
  - 不修改任何已有模块（motor.py / pid.py / imu_motion.py / control.py 等保持原样）
  - 所有独立脚本（uwb_following.py / uart_move.py 等）仍可独立运行
  - 硬件初始化在 import 时完成（利用已有的模块级副作用）
【使用】
  from robot import Robot
  robot = Robot()
  robot.run()
"""

import gc, time, math, json
from machine import UART, Pin
from motor import (
    omni_drive_closed_loop, omni_move_by_angle, stop_all,
    get_encoder_counts, get_encoder_speeds_filtered,
    ENC_SCALE,
    LED_PIN, SWITCH2_PIN, reset_encoder_filter,
)
import imu_motion
from imu_motion import (
    update_angle, get_angular_velocity, angular_velocity_control,
    reset_ang_vel_pid, MAX_WZ_DPS,
)
from pid import PID
from kalman_filter import CameraKalmanFilter
from control import CascadeController
from utils import normalize_angle, limit_value
from uwb_tracker import UWBFollower
from uart_master import MasterBT
_key_ticker = None  # 仅在 fallback 路径创建；key.py 路径下为 None

try:
    from key import key as _key_matrix
except ImportError:
    # key.py 不在设备上 → 从内置 seekfree 创建 KEY_HANDLER，使用独立 ticker
    # （enc_ticker 保持运行，KEY3 扫描不受模式切换影响）
    from smartcar import ticker as _ticker_cls
    from seekfree import KEY_HANDLER
    _key_matrix = KEY_HANDLER(10)
    _key_ticker = _ticker_cls(0)          # PIT0：独立于 enc_ticker(PIT1)
    _key_ticker.capture_list(_key_matrix)
    _key_ticker.start(10)                 # 10ms 自动采集，永不停机


def stop_key_ticker():
    """停止 KEY3 扫描的独立 ticker（仅 fallback 路径有效）。
    程序退出时由 main.py 调用，防止 PIT ISR 干扰 REPL 握手。"""
    if _key_ticker is not None:
        _key_ticker.stop()


# ============================================================
#  非阻塞路线导航状态机
# ============================================================

class _RouteStateMachine:
    """
    非阻塞路线导航器。每次调用 iterate() 只执行一步（不阻塞 Main Loop）。

    路线步骤：
      1) 右移 30cm（vy=1）
      2) 前进 60cm（vx=1）
      3) 原地旋转 180°
      4) 右移 30cm（vy=1）
    """

    # --- 控制参数（来自 uart_move.py 的测试标定值）---
    DT              = 0.01      # 控制周期 10ms
    TIMEOUT_S       = 30        # 每步超时
    PRINT_MS        = 200       # 打印间隔

    # 距离 PID
    DIST_KP         = 1.0
    DIST_KI         = 0.5
    DIST_OUT_LIMIT  = 0.30
    MIN_SPEED       = 0.08

    # 航向保持 PID
    HDG_KP          = 0.08      # 航向偏差(°) → 目标 dps
    HDG_DB          = 0.5       # 死区(°)

    # 旋转参数（梯形速度曲线）
    ROT_MAX_RATE    = 150       # dps
    ROT_MAX_ACCEL   = 90        # dps/s²
    ROT_DEADBAND    = 3.0       # 到位判定(°)

    # --- 返回值 ---
    RESULT_CONTINUE = 0
    RESULT_DONE     = 1
    RESULT_ABORT    = 2

    # --- 路线步骤定义 ---
    # (type, vx_dir, vy_dir, target_m, label) for straight
    # (type, target_delta, label) for rotate
    STEPS = [
        ('straight', 0,  1,  0.30, "RIGHT 30cm"),
        ('straight', 1,  0,  0.60, "FWD 60cm"),
        ('rotate',   180,        "ROT 180°"),
        ('straight', 0,  1,  0.30, "RIGHT 30cm"),
    ]

    def __init__(self):
        self.step_idx    = -1
        self.total_steps = len(self.STEPS)
        self.done        = False
        self.aborted     = False
        self._label      = ""

        # 每个 step 的运行时状态
        self.pid_dist      = None
        self.total_counts  = [0, 0, 0, 0]
        self.start_ms      = 0
        self.target_heading = None
        self.target_yaw    = None
        self.start_yaw     = None

    # --------------------------------------------------------
    #  公共接口
    # --------------------------------------------------------

    def start(self):
        """开始执行路线（在 Mode Enter 时调用一次）。"""
        self.step_idx = 0
        self.done     = False
        self.aborted  = False
        self._init_step()

    def iterate(self):
        """
        执行当前步骤的一次迭代。
        返回 RESULT_CONTINUE / RESULT_DONE / RESULT_ABORT。
        """
        if self.done or self.aborted or self.step_idx < 0:
            return self.RESULT_DONE if self.done else self.RESULT_ABORT

        step = self.STEPS[self.step_idx]
        typ = step[0]

        if typ == 'straight':
            return self._iter_straight(step)
        elif typ == 'rotate':
            return self._iter_rotate(step)

        return self.RESULT_ABORT

    # --------------------------------------------------------
    #  内部：步骤初始化
    # --------------------------------------------------------

    def _init_step(self):
        """初始化当前步骤的状态。"""
        step = self.STEPS[self.step_idx]
        self._label = step[-1]

        # 清空编码器残余值
        for _ in range(5):
            get_encoder_counts()
            time.sleep_ms(10)

        if step[0] == 'straight':
            _, vx_dir, vy_dir, target_m, label = step
            self.pid_dist = PID(
                kp=self.DIST_KP, ki=self.DIST_KI, kd=0.0,
                integral_limit=0.5, output_limit=self.DIST_OUT_LIMIT,
            )
            self.total_counts = [0, 0, 0, 0]

            # 锁定起始航向
            reset_ang_vel_pid()
            for _ in range(5):
                d = imu_motion.imu.read()
                update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
                time.sleep_ms(10)
            self.target_heading = imu_motion.yaw

        elif step[0] == 'rotate':
            target_delta = step[1]
            reset_ang_vel_pid()
            for _ in range(5):
                d = imu_motion.imu.read()
                update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
                time.sleep_ms(10)

            self.start_yaw = imu_motion.yaw
            self.target_yaw = self.start_yaw + target_delta
            while self.target_yaw > 180:
                self.target_yaw -= 360
            while self.target_yaw < -180:
                self.target_yaw += 360

        self.start_ms = time.ticks_ms()
        print("\n  ── [{:s}] step {:d}/{:d} ──".format(
            self._label, self.step_idx + 1, self.total_steps))

    # --------------------------------------------------------
    #  内部：直线移动迭代
    # --------------------------------------------------------

    def _iter_straight(self, step):
        """直线移动的单次迭代（编码器里程计 + PID + 航向保持）。"""
        _, vx_dir, vy_dir, target_m, label = step
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, self.start_ms) / 1000.0

        if elapsed_s > self.TIMEOUT_S:
            print("  [{:s}] TIMEOUT".format(label))
            self.aborted = True
            return self.RESULT_ABORT

        # 1) 航向保持（角速度闭环）
        wz = 0.0
        if self.target_heading is not None:
            d = imu_motion.imu.read()
            update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
            hdg_err = self.target_heading - imu_motion.yaw
            while hdg_err > 180:
                hdg_err -= 360
            while hdg_err < -180:
                hdg_err += 360
            if abs(hdg_err) > self.HDG_DB:
                tgt_dps = hdg_err * self.HDG_KP * MAX_WZ_DPS
                tgt_dps = max(-180, min(tgt_dps, 180))
                wz = angular_velocity_control(tgt_dps, get_angular_velocity(), self.DT)

        # 2) 编码器里程计
        raw_counts = get_encoder_counts()
        raw_speeds = [raw_counts[i] / ENC_SCALE[i] / self.DT for i in range(4)]
        for i in range(4):
            self.total_counts[i] += raw_counts[i]

        wd = [abs(self.total_counts[i]) / ENC_SCALE[i] for i in range(4)]
        dist = sum(wd) / 4

        # 3) 距离 PID
        speed_cmd = self.pid_dist.compute(
            setpoint=target_m, measurement=dist, dt=self.DT)
        if dist < target_m and speed_cmd < self.MIN_SPEED:
            speed_cmd = self.MIN_SPEED

        # 4) 驱动
        vx = vx_dir * speed_cmd
        vy = vy_dir * speed_cmd
        omni_drive_closed_loop(vx, vy, wz, raw_speeds, self.DT)

        # 5) 到位判定
        if dist >= target_m:
            print("  >>> [{:s}] done: {:.1f}cm <<<".format(label, dist * 100))
            return self.RESULT_DONE

        return self.RESULT_CONTINUE

    # --------------------------------------------------------
    #  内部：旋转迭代
    # --------------------------------------------------------

    def _iter_rotate(self, step):
        """旋转的单次迭代（梯形速度曲线 + 角速度闭环）。"""
        _, target_delta, label = step
        now_ms = time.ticks_ms()
        elapsed_s = time.ticks_diff(now_ms, self.start_ms) / 1000.0

        if elapsed_s > self.TIMEOUT_S:
            print("  [{:s}] TIMEOUT".format(label))
            self.aborted = True
            return self.RESULT_ABORT

        # 1) IMU 更新
        d = imu_motion.imu.read()
        update_angle(d[0], d[1], d[2], d[3], d[4], d[5])

        # 2) 偏差计算
        err = self.target_yaw - imu_motion.yaw
        while err > 180:
            err -= 360
        while err < -180:
            err += 360

        if abs(err) <= self.ROT_DEADBAND:
            dlt = imu_motion.yaw - self.start_yaw
            while dlt > 180: dlt -= 360
            while dlt < -180: dlt += 360
            print("  >>> [{:s}] done: {:.1f}°  err={:+.1f}° <<<".format(
                label, dlt, err))
            return self.RESULT_DONE

        # 3) 梯形速度曲线
        ideal = (2 * self.ROT_MAX_ACCEL * abs(err)) ** 0.5
        tgt_rate = min(ideal, self.ROT_MAX_RATE)
        if err < 0:
            tgt_rate = -tgt_rate

        # 4) 角速度闭环
        actual_dps = get_angular_velocity()
        wz = angular_velocity_control(tgt_rate, actual_dps, self.DT)

        # 5) 驱动
        rc = get_encoder_counts()
        rs = [rc[i] / ENC_SCALE[i] / self.DT for i in range(4)]
        omni_drive_closed_loop(0, 0, wz, rs, self.DT)

        return self.RESULT_CONTINUE


# ============================================================
#  机器人集成框架
# ============================================================

class Robot:
    """机器人状态机与模式调度器。"""

    # ── 模式常量 ──
    MODE_IDLE          = 0
    MODE_CAMERA_TRACK  = 1
    MODE_UWB_FOLLOW    = 2
    MODE_UART_REMOTE   = 3
    MODE_ROUTE_NAV     = 4
    MODE_VISUAL_APPROACH = 5
    MODE_SLAVE_CMD       = 6
    MODE_SYNC_DRIVE      = 7

    MODE_NAMES  = ["IDLE", "CAMERA_TRACK", "UWB_FOLLOW", "UART_REMOTE", "ROUTE_NAV",
                   "VISUAL_APPROACH", "SLAVE_CMD", "SYNC_DRIVE"]
    MODE_COUNT  = 8

    # ── LED 闪烁周期（ms），0 = 常亮 ──
    LED_PERIODS = [500, 0, 100, 250, 300, 200, 150, 100]

    # ── 按键参数 ──
    KEY3_DEBOUNCE_MS    = 30      # KEY3 消抖窗口

    # ============================================================
    #  初始化
    # ============================================================

    def __init__(self):
        """初始化机器人状态。

        注意：硬件初始化在 import stage（motor.py, imu_motion.py）时已完成。
        此处仅初始化逻辑状态。
        """
        # ── LED ──
        self._led            = Pin(LED_PIN, Pin.OUT, value=False)
        self._led_state      = False
        self._led_last_toggle = time.ticks_ms()

        # ── SW2（拨码开关）—— 纯轮询检测（不依赖 IRQ/ticker，该端口不可靠） ──
        self._sw2      = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
        self._sw2_last = self._sw2.value()   # 初始电平，用于轮询比较
        self._sw2_exit = False                # 主循环退出标志

        # ── KEY3（通过 key.py 矩阵扫描检测，不直接读 Pin——C14 GPIO 不可靠） ──
        self._key3_press_start = 0
        self._key3_pressed   = False

        # ── 模式状态 ──
        self._mode     = self.MODE_IDLE
        self._running  = False
        self._loop_cnt = 0

        # ── 模式资源（惰性创建）──

        # CAMERA_TRACK / VISUAL_APPROACH
        self._cam_uart      = None
        self._cam_kf        = None
        self._cam_ctrl      = None
        self._cam_last_data = 0
        self._cam_tracking  = False
        self._cam_confirm_count = 0
        self._cam_confirm_threshold = 5   # 连续 N 帧确认才信任（防 YOLO 误检）

        # UWB_FOLLOW
        self._uwb  = None

        # UART_REMOTE
        self._remote_uart = None

        # ROUTE_NAV
        self._route = None

        # Master-Slave 通信
        self._master_bt = None

        # ── 安全保障 ──
        stop_all()
        print("Robot initialized. Mode: IDLE  |  KEY3=cycle mode  |  SW2 toggle=exit")

    # ============================================================
    #  主循环
    # ============================================================

    def run(self):
        """主循环，永不返回。

        每次迭代：KEY3 → LED → 模式调度 → sleep → GC
        """
        self._running = True
        print("=" * 50)
        print("Robot.run() started")
        print("  KEY3 short press : cycle mode")
        print("  KEY3 long  press : exit program (>=2s)")
        print("=" * 50)

        while self._running:
            now = time.ticks_ms()

            # ── SW2 检测（D9 在此移植版不可靠，保留作兜底） ──
            self._check_sw2()
            if self._sw2_exit:
                break

            # ── KEY3 短按 → 循环切换模式 ──
            self._handle_key3(now)

            # ── LED ──
            self._update_led(now)

            # ── 模式调度（内部也有 SW2 检测兜底） ──
            self._dispatch_mode()
            if self._sw2_exit:   # dispatch 内也可能触发
                break

            time.sleep_ms(10)

            self._loop_cnt += 1
            if self._loop_cnt % 50 == 0:
                gc.collect()

        self._running = False
        print("[Robot] run() ended")

    # ============================================================
    #  公共 API
    # ============================================================

    def get_mode(self):
        """返回当前模式 ID。"""
        return self._mode

    def get_mode_name(self):
        """返回当前模式名称。"""
        return self.MODE_NAMES[self._mode]

    def set_mode(self, mode):
        """安全切换到目标模式。"""
        if not (0 <= mode < self.MODE_COUNT):
            return
        if mode == self._mode:
            return

        # 退出当前模式
        self._exit_mode()

        # 切换
        old_name = self.MODE_NAMES[self._mode]
        self._mode = mode

        # 进入新模式
        self._enter_mode()
        print(">>> MODE: {:s} ({:s} → {:s}) <<<".format(
            self.MODE_NAMES[mode], old_name, self.MODE_NAMES[mode]))

    def emergency_stop(self):
        """急停：所有电机停止 + 资源释放 + PID 重置 + 回到 IDLE。"""
        old_mode = self._mode
        print("[EMERGENCY STOP]")
        stop_all()

        # 释放当前模式资源
        if old_mode == self.MODE_CAMERA_TRACK:
            if self._cam_ctrl:
                self._cam_ctrl.emergency_stop()   # stop_all + pid reset
            try:
                self._cam_uart.deinit()
            except AttributeError:
                pass
            self._cam_uart = None
            self._cam_kf   = None
            self._cam_ctrl = None
        elif old_mode == self.MODE_UWB_FOLLOW:
            if self._uwb:
                self._uwb.stop()                  # 内部已关闭 UART
            try:
                self._cam_uart.deinit()
            except AttributeError:
                pass
            self._cam_uart = None
            self._uwb = None
        elif old_mode == self.MODE_VISUAL_APPROACH:
            if self._cam_ctrl:
                self._cam_ctrl.emergency_stop()
            try:
                self._cam_uart.deinit()
            except AttributeError:
                pass
            self._cam_uart = None
            self._cam_kf   = None
            self._cam_ctrl = None
        elif old_mode == self.MODE_UART_REMOTE:
            self._remote_uart = None
        elif old_mode == self.MODE_ROUTE_NAV:
            if self._route:
                self._route.aborted = True
            self._route = None
        elif old_mode == self.MODE_SLAVE_CMD or old_mode == self.MODE_SYNC_DRIVE:
            if self._master_bt:
                self._master_bt.send_emergency_stop()
            self._master_bt = None

        reset_encoder_filter()
        reset_ang_vel_pid()
        self._mode = self.MODE_IDLE
        print(">>> MODE: IDLE (after EMERGENCY STOP) <<<")

    # ============================================================
    #  SW2 检测（纯轮询，不依赖 IRQ/ticker）
    # ============================================================

    def _check_sw2(self):
        """读 Pin 电平，与上次值比较。通知检测到变化即退出。
        
        该端口（D9）的 IRQ 和 ticker 回调在此移植版上不可靠，
        因此采用纯主线程轮询——每次主循环 + dispatch 前/后调用。
        """
        sw2_val = self._sw2.value()
        if sw2_val != self._sw2_last:
            self._sw2_last = sw2_val
            print("[SW2] {} → emergency stop + exit".format(sw2_val))
            self.emergency_stop()
            self._sw2_exit = True
            return

    # ============================================================
    #  KEY3 处理（消抖 + 短按循环切换模式）
    # ============================================================

    def _handle_key3(self, now):
        """KEY3 检测：通过 seekfree KEY_HANDLER 矩阵扫描（不直接读 C14 GPIO）。
        
        矩阵扫描由独立 ticker（key.py 的 PIT1 或 fallback 的 PIT0）自动驱动，
        此处手动 capture() 为兜底刷新。
        """
        try:
            _key_matrix.capture()
            state = _key_matrix.get()
            key3_down = bool(state[2]) if len(state) >= 3 else False
        except Exception as e:
            # 不要静默吞 — 至少打印一次，方便排查硬件/驱动问题
            print("[KEY3] capture error:", e)
            key3_down = False

        if key3_down and not self._key3_pressed:
            # 首次按下
            self._key3_press_start = now
            self._key3_pressed = True
        elif not key3_down and self._key3_pressed:
            # 释放
            hold_ms = time.ticks_diff(now, self._key3_press_start)
            if hold_ms >= 2000:
                print("[KEY3] Long press {:d}ms → emergency exit".format(hold_ms))
                self.emergency_stop()
                self._sw2_exit = True
            else:
                next_mode = (self._mode + 1) % self.MODE_COUNT
                self.set_mode(next_mode)
            self._key3_pressed = False
        elif key3_down and self._key3_pressed:
            # 持续按下中
            hold_ms = time.ticks_diff(now, self._key3_press_start)
            if hold_ms >= 2000:
                print("[KEY3] Long press {:d}ms → emergency exit".format(hold_ms))
                self.emergency_stop()
                self._sw2_exit = True
                self._key3_pressed = False

    # ============================================================
    #  LED 状态指示
    # ============================================================

    def _update_led(self, now):
        period = self.LED_PERIODS[self._mode]
        if period == 0:
            # 常亮
            if not self._led_state:
                self._led.value(1)
                self._led_state = True
        else:
            # 闪烁
            if time.ticks_diff(now, self._led_last_toggle) >= period:
                self._led_state = not self._led_state
                self._led.value(1 if self._led_state else 0)
                self._led_last_toggle = now

    # ============================================================
    #  模式调度
    # ============================================================

    def _dispatch_mode(self):
        # 每次派发前检测 SW2（兜底：某些模式下主循环轮询可能延迟）
        self._check_sw2()
        if self._sw2_exit:
            return

        if self._mode == self.MODE_IDLE:
            self._mode_idle()
        elif self._mode == self.MODE_CAMERA_TRACK:
            self._mode_camera_track()
        elif self._mode == self.MODE_UWB_FOLLOW:
            self._mode_uwb_follow()
        elif self._mode == self.MODE_UART_REMOTE:
            self._mode_uart_remote()
        elif self._mode == self.MODE_ROUTE_NAV:
            self._mode_route_nav()
        elif self._mode == self.MODE_VISUAL_APPROACH:
            self._mode_visual_approach()
        elif self._mode == self.MODE_SLAVE_CMD:
            self._mode_slave_cmd()
        elif self._mode == self.MODE_SYNC_DRIVE:
            self._mode_sync_drive()

        # 模式返回后再检测一次（捕获在 mode 内发生的拨动）
        self._check_sw2()

    # ============================================================
    #  模式切换钩子
    # ============================================================

    def _exit_mode(self):
        """退出当前模式：停电机 + 释放资源。"""
        stop_all()
        reset_encoder_filter()

        if self._mode == self.MODE_CAMERA_TRACK:
            self._exit_camera_track()
        elif self._mode == self.MODE_UWB_FOLLOW:
            self._exit_uwb_follow()
        elif self._mode == self.MODE_UART_REMOTE:
            self._exit_uart_remote()
        elif self._mode == self.MODE_ROUTE_NAV:
            self._exit_route_nav()
        elif self._mode == self.MODE_VISUAL_APPROACH:
            self._exit_visual_approach()
        elif self._mode == self.MODE_SLAVE_CMD:
            self._exit_slave_cmd()
        elif self._mode == self.MODE_SYNC_DRIVE:
            self._exit_sync_drive()

    def _enter_mode(self):
        """进入新模式：初始化资源。"""
        if self._mode == self.MODE_IDLE:
            self._enter_idle()
        elif self._mode == self.MODE_CAMERA_TRACK:
            self._enter_camera_track()
        elif self._mode == self.MODE_UWB_FOLLOW:
            self._enter_uwb_follow()
        elif self._mode == self.MODE_UART_REMOTE:
            self._enter_uart_remote()
        elif self._mode == self.MODE_ROUTE_NAV:
            self._enter_route_nav()
        elif self._mode == self.MODE_VISUAL_APPROACH:
            self._enter_visual_approach()
        elif self._mode == self.MODE_SLAVE_CMD:
            self._enter_slave_cmd()
        elif self._mode == self.MODE_SYNC_DRIVE:
            self._enter_sync_drive()

        # 重置 LED 定时器，让新模式立即显示正确的 LED 状态
        self._led_last_toggle = time.ticks_ms()

    # ============================================================
    #  MODE IDLE
    # ============================================================

    def _enter_idle(self):
        """IDLE 模式初始化。"""
        pass

    def _mode_idle(self):
        """IDLE 模式：无动作，LED 慢闪。"""
        pass

    # ============================================================
    #  MODE VISUAL_APPROACH（视觉逼近）
    # ============================================================

    def _enter_visual_approach(self):
        """视觉逼近初始化。使用 Kalman + Cascade PID 逼近目标至 10cm。"""
        if not self._cam_uart:
            self._cam_uart = UART(7, baudrate=115200, bits=8, parity=None, stop=1)
        self._cam_kf   = CameraKalmanFilter()
        self._cam_ctrl = CascadeController()
        self._cam_last_data = time.ticks_ms()
        self._cam_tracking  = False
        self._cam_confirm_count = 0   # 每次进入模式时重置
        print("VISUAL_APPROACH: approaching target to 10cm")

    def _mode_visual_approach(self):
        """视觉逼近单次迭代。目标距离 ≤10cm → SLAVE_CMD，丢失 → UWB_FOLLOW。"""
        if not self._cam_uart:
            return

        if self._cam_uart.any() >= 10:
            buf = self._cam_uart.read()
            if buf:
                idx = buf.find(b'\xAA')
                if idx != -1 and len(buf) >= idx + 10 and buf[idx + 9] == 0xBB:
                    # 协议: AA [X_H X_L] [Y_H Y_L] [LABEL_H LABEL_L] [STATUS_H STATUS_L] BB
                    raw_x      = (buf[idx + 1] << 8) | buf[idx + 2]
                    raw_y      = (buf[idx + 3] << 8) | buf[idx + 4]
                    raw_label  = (buf[idx + 5] << 8) | buf[idx + 6]
                    raw_status = (buf[idx + 7] << 8) | buf[idx + 8]

                    x_cm = (raw_x if raw_x < 32768 else raw_x - 65536) / 10.0
                    y_cm = (raw_y if raw_y < 32768 else raw_y - 65536) / 10.0
                    label = raw_label
                    target_detected = (raw_status == 1)

                    self._cam_last_data = time.ticks_ms()

                    # ── 目标确认：连续 N 帧检测到才信任（防 YOLO 误检）──
                    if target_detected and not (x_cm == 0 and y_cm == 0):
                        self._cam_confirm_count += 1
                    else:
                        self._cam_confirm_count = 0

                    self._cam_tracking = (self._cam_confirm_count >= self._cam_confirm_threshold)

                    if self._cam_tracking:
                        # x_cm → 横向偏差(ex), y_cm → 纵向距离(dist), roll=0(协议无此字段)
                        ex_f, dist_f, roll_f = self._cam_kf.update(x_cm, y_cm, 0.0)
                        vx_out, vy_out, wz_out, actual_speeds, dt = self._cam_ctrl.step(
                            ex_f, dist_f, roll_f, True)
                        print("[VISUAL] id:{:d} ex_f={:.1f} dist_f={:.1f} | vx={:.3f} vy={:.3f}".format(
                            label, ex_f, dist_f, vx_out, vy_out))

                        # 到达 10cm → 切换到 SLAVE_CMD
                        if dist_f <= 10:
                            print("[VISUAL] Target reached! dist_f={:.1f} → SLAVE_CMD".format(dist_f))
                            if self._cam_ctrl:
                                self._cam_ctrl.emergency_stop()
                            self.set_mode(self.MODE_SLAVE_CMD)
                            return
                    else:
                        if self._cam_confirm_count > 0:
                            print("[VISUAL] Confirming... {:d}/{:d}".format(
                                self._cam_confirm_count, self._cam_confirm_threshold))

        # 看门狗
        if time.ticks_diff(time.ticks_ms(), self._cam_last_data) > 500:
            if self._cam_ctrl:
                self._cam_ctrl.emergency_stop()
            if self._cam_tracking:
                self._cam_tracking = False
                self._cam_confirm_count = 0
                print("[VISUAL] Timeout → UWB_FOLLOW")
                self.set_mode(self.MODE_UWB_FOLLOW)

    def _exit_visual_approach(self):
        """视觉逼近清理。"""
        if self._cam_ctrl:
            self._cam_ctrl.emergency_stop()
        self._cam_kf   = None
        self._cam_ctrl = None

    # ============================================================
    #  MODE CAMERA_TRACK
    # ============================================================

    def _enter_camera_track(self):
        """摄像头追踪初始化。

        创建 UART7（115200）、卡尔曼滤波器、级联 PID 控制器。
        """
        self._cam_uart = UART(7, baudrate=115200, bits=8, parity=None, stop=1)
        self._cam_kf   = CameraKalmanFilter()
        self._cam_ctrl = CascadeController()
        self._cam_last_data = time.ticks_ms()
        self._cam_tracking  = False
        self._cam_confirm_count = 0   # 每次进入模式时重置

        print("CAMERA_TRACK: UART7 115200 | KF + CascadeController ready")

    def _exit_camera_track(self):
        """摄像头追踪清理。"""
        if self._cam_ctrl:
            self._cam_ctrl.emergency_stop()
        self._cam_uart = None
        self._cam_kf   = None
        self._cam_ctrl = None

    def _mode_camera_track(self):
        """摄像头追踪单次迭代。

        从 UART7 读取 10 字节 AA..BB 协议帧 → 卡尔曼滤波 → 级联 PID → 驱动电机。
        协议: AA [X] [Y] [LABEL] [STATUS] BB (int16 大端序, ÷10)
        """
        if not self._cam_uart:
            return

        # ── 读取并解析 UART 数据 ──
        if self._cam_uart.any() >= 10:
            buf = self._cam_uart.read()
            if buf:
                idx = buf.find(b'\xAA')
                if idx != -1 and len(buf) >= idx + 10 and buf[idx + 9] == 0xBB:
                    # 协议: AA [X_H X_L] [Y_H Y_L] [LABEL_H LABEL_L] [STATUS_H STATUS_L] BB
                    raw_x      = (buf[idx + 1] << 8) | buf[idx + 2]
                    raw_y      = (buf[idx + 3] << 8) | buf[idx + 4]
                    raw_label  = (buf[idx + 5] << 8) | buf[idx + 6]
                    raw_status = (buf[idx + 7] << 8) | buf[idx + 8]

                    x_cm = (raw_x if raw_x < 32768 else raw_x - 65536) / 10.0
                    y_cm = (raw_y if raw_y < 32768 else raw_y - 65536) / 10.0
                    label = raw_label
                    target_detected = (raw_status == 1)

                    self._cam_last_data = time.ticks_ms()

                    # ── 目标确认：连续 N 帧检测到才信任（防 YOLO 误检）──
                    if target_detected and not (x_cm == 0 and y_cm == 0):
                        self._cam_confirm_count += 1
                    else:
                        self._cam_confirm_count = 0

                    self._cam_tracking = (self._cam_confirm_count >= self._cam_confirm_threshold)

                    # 卡尔曼滤波: x_cm → ex, y_cm → dist, roll=0（协议无此字段）
                    ex_f, dist_f, roll_f = self._cam_kf.update(x_cm, y_cm, 0.0)

                    # 级联 PID 控制
                    vx_out, vy_out, wz_out, actual_speeds, dt = self._cam_ctrl.step(
                        ex_f, dist_f, roll_f, self._cam_tracking)

                    # 打印遥测
                    state = "TRACK" if self._cam_tracking else "WAIT"
                    spd_lf, spd_rf, spd_lb, spd_rb = actual_speeds
                    print("[{:s}] id:{:d} X:{:+6.1f} Y:{:5.1f} | "
                          "filt ex:{:+5.1f} dist:{:5.1f} | "
                          "vx:{:.3f} vy:{:.3f} wz:{:.3f} | dt:{:.0f}ms".format(
                              state, label, x_cm, y_cm,
                              ex_f, dist_f,
                              vx_out, vy_out, wz_out, dt * 1000))

        # ── 看门狗：数据超时 500ms 则紧急停车 ──
        if time.ticks_diff(time.ticks_ms(), self._cam_last_data) > 500:
            if self._cam_ctrl:
                self._cam_ctrl.emergency_stop()
            if self._cam_tracking:
                self._cam_tracking = False
                self._cam_confirm_count = 0
                print("[LOST] Connection timeout.")

    # ============================================================
    #  MODE UWB_FOLLOW
    # ============================================================

    def _enter_uwb_follow(self):
        """UWB 跟随初始化。UART0@115200 + 摄像头 UART7@115200。"""
        self._uwb = UWBFollower(uart_id=0, baudrate=115200, target_anchor="8834")
        # 同时初始化摄像头 UART7（用于中断检测）
        self._cam_uart = UART(7, baudrate=115200, bits=8, parity=None, stop=1)
        print("UWB_FOLLOW: UART0 115200 | Camera UART7 115200 (polling)")

    def _exit_uwb_follow(self):
        """UWB 跟随清理。"""
        stop_all()
        if self._uwb:
            self._uwb.stop()
        self._uwb = None
        self._cam_confirm_count = 0

    def _mode_uwb_follow(self):
        """UWB 跟随单次迭代。闭环控制 + 轮询摄像头中断。"""
        if not self._uwb:
            return

        # 闭环步进（内部处理 UART 读取 + 滤波 + IMU 航向纠偏 + 驱动）
        self._uwb.step()

        # 轮询摄像头中断检测
        self._poll_camera_interrupt()

    # ── 摄像头中断检测 ──

    def _poll_camera_interrupt(self):
        """非阻塞检查摄像头 UART7 是否检测到有效目标。
        连续 N 帧检测到才切换到 VISUAL_APPROACH（防单帧误检）。"""
        if self._mode != self.MODE_UWB_FOLLOW:
            return
        if not self._cam_uart:
            return
        if self._cam_uart.any() < 10:
            return

        buf = self._cam_uart.read()
        if not buf:
            return

        idx = buf.find(b'\xAA')
        if idx != -1 and len(buf) >= idx + 10 and buf[idx + 9] == 0xBB:
            raw_status = (buf[idx + 7] << 8) | buf[idx + 8]
            raw_x = (buf[idx + 1] << 8) | buf[idx + 2]
            raw_y = (buf[idx + 3] << 8) | buf[idx + 4]

            # 检查是否为有效目标（status=1 且坐标非零）
            if raw_status == 1 and not (raw_x == 0 and raw_y == 0):
                self._cam_confirm_count += 1
                if self._cam_confirm_count >= self._cam_confirm_threshold:
                    print("[CAM INTERRUPT] Confirmed {:d}/{:d}! → VISUAL_APPROACH".format(
                        self._cam_confirm_count, self._cam_confirm_threshold))
                    self._cam_confirm_count = 0
                    self.set_mode(self.MODE_VISUAL_APPROACH)
                else:
                    print("[CAM] Confirming... {:d}/{:d}".format(
                        self._cam_confirm_count, self._cam_confirm_threshold))
            else:
                self._cam_confirm_count = 0

    # ============================================================
    #  MODE UART_REMOTE（蓝牙遥控）
    # ============================================================

    def _enter_uart_remote(self):
        """蓝牙遥控初始化。

        UART5 初始化为 9600 波特率（HC-05 默认）。
        使用 motor.py 的正确电机映射（非 uart_slave.py）。
        """
        self._remote_uart = UART(5)
        self._remote_uart.init(baudrate=9600, bits=8, parity=None, stop=1)
        print("UART_REMOTE: HC-05 slave on UART5 9600")
        print("  Commands: run / stop / left / right / back")

    def _exit_uart_remote(self):
        """蓝牙遥控清理。"""
        stop_all()
        self._remote_uart = None

    def _mode_uart_remote(self):
        """蓝牙遥控单次迭代。

        读取 UART5 命令 → 执行全向移动。
        """
        if not self._remote_uart:
            return

        if self._remote_uart.any():
            data = self._remote_uart.readline()
            if data:
                cmd = data.strip()
                print("[UART_REMOTE] Received: {:s}".format(str(cmd)))

                if cmd in (b'run', b'run\r\n'):
                    omni_move_by_angle(0.5, 0)
                    self._remote_uart.write(b'MOTOR_RUN\r\n')
                elif cmd in (b'stop', b'stop\r\n'):
                    stop_all()
                    self._remote_uart.write(b'MOTOR_STOP\r\n')
                elif cmd in (b'left', b'left\r\n'):
                    omni_move_by_angle(0.3, -90)
                    self._remote_uart.write(b'LEFT\r\n')
                elif cmd in (b'right', b'right\r\n'):
                    omni_move_by_angle(0.3, 90)
                    self._remote_uart.write(b'RIGHT\r\n')
                elif cmd in (b'back', b'back\r\n'):
                    omni_move_by_angle(0.3, 180)
                    self._remote_uart.write(b'BACK\r\n')
                else:
                    self._remote_uart.write(b'ACK\r\n')

    # ============================================================
    #  MODE ROUTE_NAV（预设路线导航）
    # ============================================================

    def _enter_route_nav(self):
        """路线导航初始化。"""
        self._route = _RouteStateMachine()
        self._route.start()
        print("ROUTE_NAV: {:d} steps starting".format(self._route.total_steps))

    def _exit_route_nav(self):
        """路线导航清理。"""
        stop_all()
        self._route = None

    def _mode_route_nav(self):
        """路线导航单次迭代。

        非阻塞地执行路线中的每一步。
        """
        if not self._route:
            return

        result = self._route.iterate()

        if result == _RouteStateMachine.RESULT_DONE:
            # 当前步骤完成，进入下一步
            self._route.step_idx += 1
            if self._route.step_idx >= self._route.total_steps:
                print("\n=== Route complete! ===")
                self.set_mode(self.MODE_IDLE)
                return
            self._route._init_step()

        elif result == _RouteStateMachine.RESULT_ABORT:
            print("Route aborted.")
            self.set_mode(self.MODE_IDLE)

    # ============================================================
    #  MODE SLAVE_CMD（向从车发送指令）
    # ============================================================

    def _enter_slave_cmd(self):
        """初始化蓝牙通信。"""
        self._master_bt = MasterBT(uart_id=5, baudrate=9600)
        print("SLAVE_CMD: waiting for slave...")

    def _mode_slave_cmd(self):
        """发送 POS_ADJ 给从车，等待 POS_OK。"""
        if not self._master_bt:
            return

        ok = self._master_bt.send_pos_adjust(0.0, 0.0, 0.0)
        if ok:
            print("[SLAVE_CMD] POS_OK received → SYNC_DRIVE")
            self.set_mode(self.MODE_SYNC_DRIVE)
        else:
            print("[SLAVE_CMD] No response from slave")
            self.emergency_stop()

    def _exit_slave_cmd(self):
        """SLAVE_CMD 清理。"""
        self._master_bt = None

    # ============================================================
    #  MODE SYNC_DRIVE（两车同步行驶）
    # ============================================================

    def _enter_sync_drive(self):
        """同步驾驶初始化。"""
        print("SYNC_DRIVE: master + slave synchronized")

    def _mode_sync_drive(self):
        """同步驾驶：计算控制量 → 发送 SYNC_MOVE → 自身执行。"""
        vx = 0.5
        vy = 0.0
        wz = 0.0

        if self._master_bt:
            self._master_bt.send_sync_move(vx, vy, wz)

        speeds = get_encoder_speeds_filtered(0.01)
        omni_drive_closed_loop(vx, vy, wz, speeds, 0.01)

    def _exit_sync_drive(self):
        """同步驾驶清理。"""
        stop_all()
        if self._master_bt:
            self._master_bt.send_emergency_stop()
