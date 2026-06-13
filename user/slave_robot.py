"""
slave_robot.py - 从车状态机框架
====================================
模式:
  CAMERA_FOLLOW (0) - 摄像头跟随主车（外部提供的视觉代码）
  POS_ADJUST    (1) - 位置微调（接收主车指令）
  SYNC_DRIVE    (2) - 同步行驶
  EMERGENCY     (3) - 紧急停止

依赖:
  uart_slave.SlaveBT - 蓝牙通信
  slave_motor - 电机驱动 + 编码器
"""

import time
from machine import UART
from uart_slave import SlaveBT


class SlaveRobot:
    """从车状态机"""

    MODE_CAMERA_FOLLOW = 0
    MODE_POS_ADJUST    = 1
    MODE_SYNC_DRIVE    = 2
    MODE_EMERGENCY     = 3

    MODE_NAMES = ["CAMERA_FOLLOW", "POS_ADJUST", "SYNC_DRIVE", "EMERGENCY"]

    def __init__(self):
        from slave_motor import stop_all

        self._bt       = SlaveBT(uart_id=5, baudrate=9600)
        self._mode     = self.MODE_CAMERA_FOLLOW
        self._cmd_vx   = 0.0
        self._cmd_vy   = 0.0
        self._cmd_wz   = 0.0

        stop_all()
        print("SlaveRobot initialized. Mode: CAMERA_FOLLOW")

    def run(self):
        """主循环（永不返回）。"""
        print("=" * 50)
        print("SlaveRobot.run() started")
        print("  Listening on UART5 (HC-05)")
        print("=" * 50)

        while True:
            # 1. 优先处理主车命令
            cmd = self._bt.read_command()
            if cmd:
                self._handle_command(cmd)

            # 2. 执行当前模式
            self._dispatch_mode()

            time.sleep_ms(10)

    def _handle_command(self, cmd):
        """解析主车命令，切换模式。"""
        t = cmd.get("type", "")

        if t == "pos_adj":
            self._cmd_vx = cmd.get("vx", 0.0)
            self._cmd_vy = cmd.get("vy", 0.0)
            self._cmd_wz = cmd.get("wz", 0.0)
            self._mode = self.MODE_POS_ADJUST
            print("[CMD] POS_ADJ vx={:.3f} vy={:.3f} wz={:.3f}".format(
                self._cmd_vx, self._cmd_vy, self._cmd_wz))

        elif t == "sync_move":
            self._cmd_vx = cmd.get("vx", 0.0)
            self._cmd_vy = cmd.get("vy", 0.0)
            self._cmd_wz = cmd.get("wz", 0.0)
            old = self._mode
            self._mode = self.MODE_SYNC_DRIVE
            if old != self.MODE_SYNC_DRIVE:
                print("[CMD] SYNC_MOVE -> SYNC_DRIVE")

        elif t == "emergency_stop":
            from slave_motor import stop_all
            stop_all()
            self._mode = self.MODE_EMERGENCY
            print("[CMD] EMERGENCY STOP")

    def _dispatch_mode(self):
        """执行当前模式的单次迭代。"""
        if self._mode == self.MODE_CAMERA_FOLLOW:
            self._mode_camera_follow()

        elif self._mode == self.MODE_POS_ADJUST:
            self._mode_pos_adjust()

        elif self._mode == self.MODE_SYNC_DRIVE:
            self._mode_sync_drive()

        elif self._mode == self.MODE_EMERGENCY:
            pass  # already stopped

    # ================================================================
    #  MODE 0: CAMERA_FOLLOW - 摄像头跟随主车
    #  【预留接口】用户填入外部视觉跟踪代码
    # ================================================================

    def _mode_camera_follow(self):
        """摄像头视觉跟随主车。
        TODO: 用户填入外部摄像头跟踪代码。
        典型用法:
          vx, vy, wz = your_camera_tracking_function()
          omni_drive_closed_loop(vx, vy, wz, speeds, dt)
        """
        pass

    # ================================================================
    #  MODE 1: POS_ADJUST - 位置微调
    # ================================================================

    def _mode_pos_adjust(self):
        """执行位置微调，完成后回复 POS_OK，回到 CAMERA_FOLLOW。"""
        from slave_motor import (
            omni_drive_closed_loop, get_encoder_speeds,
            stop_all, reset_wheel_pi, reset_encoder_filter,
        )

        vx = self._cmd_vx
        vy = self._cmd_vy
        wz = self._cmd_wz

        print("[POS] Executing pos_adjust: vx={:.3f} vy={:.3f} wz={:.3f}".format(vx, vy, wz))

        # 重置滤波器和 PID
        reset_encoder_filter()
        reset_wheel_pi()

        # 执行调整：驱动 300ms
        start_ms = time.ticks_ms()
        last_ms  = start_ms
        while time.ticks_diff(time.ticks_ms(), start_ms) < 300:
            now_ms = time.ticks_ms()
            dt = time.ticks_diff(now_ms, last_ms) * 0.001
            if dt <= 0 or dt > 0.5:
                dt = 0.01
            last_ms = now_ms
            speeds = get_encoder_speeds(dt)
            omni_drive_closed_loop(vx, vy, wz, speeds, dt)
            time.sleep_ms(10)

        stop_all()
        self._bt.send_ok()
        self._mode = self.MODE_CAMERA_FOLLOW
        print("[POS] Done -> POS_OK sent -> CAMERA_FOLLOW")

    # ================================================================
    #  MODE 2: SYNC_DRIVE - 同步行驶
    # ================================================================

    def _mode_sync_drive(self):
        """按主车指令同步行驶。保持当前模式直到收到新命令。"""
        from slave_motor import omni_drive_closed_loop, get_encoder_speeds
        import time as t

        dt = 0.01
        speeds = get_encoder_speeds(dt)
        omni_drive_closed_loop(self._cmd_vx, self._cmd_vy, self._cmd_wz, speeds, dt)
