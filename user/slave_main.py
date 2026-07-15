"""
main.py — 全向轮视觉跟随主控 + 蓝牙多方向遥控与跟随状态切换
【层】编排层（Orchestration）
"""
from machine import UART, Pin
import time, gc
from motor import stop_all
from cam_uart import UartParser
from uart_slave import SlaveBT
from control import CascadeController, TARGET_DIST
from kalman_filter import CameraKalmanFilter
from imu import IMU
from IMU_hold import HeadingHold, Rotator

# ============================================================
# 硬件初始化
# ============================================================
uart_cam = UART(7, baudrate=115200, bits=8, parity=None, stop=1)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# ============================================================
# 模块实例化
# ============================================================

# 视觉 UART 协议解析器（OpenMV 摄像头）
SCALE = 10
parser = UartParser(uart_cam, scale=SCALE)

slave_bt = SlaveBT(uart_id=5, baudrate=38400)

# 卡尔曼滤波器 —— USE_KALMAN=False 切回原始数据
USE_KALMAN = False
kf = CameraKalmanFilter()
T_DELAYS = 0.04               # 前馈补偿时间 系统总延迟时间 (秒)

# 级联跟随控制器
ctrl = CascadeController()

# ── IMU + 偏航保持 ──
TARGET_YAW_DEG = 0.0          # 默认目标偏航角 = 上电存储航角（0°）
ROTATE_DEG = 90               # 蓝牙旋转角度 (°)
target_global_yaw = 0.0       # 全局绝对目标航向，以绝对上电航角（0°）为基准
imu = None                    # 模块级，供 finally 清理
heading_hold = None
rotator = None

# ── 丢失确认 ──
LOST_TIMEOUT_MS = 250

# ── 循环节拍 ──
TARGET_LOOP_MS = 20           # 控制周期 100Hz

def main():
    global imu, heading_hold, rotator, target_global_yaw

    stop_all()

    # ═══════════════════════════════════════════════════════
    # IMU 初始化（上电延时存储航角）
    # ═══════════════════════════════════════════════════════
    try:
        imu = IMU(calibrate_on_init=True, calib_samples=500, period_ms=10)
        imu.start()
        time.sleep_ms(200)                 # 等 ticker 稳定
        imu.set_zero_reference()           # 当前朝向 = 0°（上电存储航角）

        # 偏航保持控制器
        heading_hold = HeadingHold(imu, target_yaw_deg=TARGET_YAW_DEG)

        # 绕轴旋转控制器（共享同一 IMU）
        rotator = Rotator(imu, ctrl,
                          switch_check=lambda: switch2.value() != state2)
    except Exception as e:
        imu = None

    # ═══════════════════════════════════════════════════════
    # 状态变量初始化
    # ═══════════════════════════════════════════════════════
    ex_f, dist_f, roll_f = 0.0, 0.0, 0.0
    yaw_err = 0.0              # 全局偏航误差，跨层1/层2共享
    tracking_state = False

    # 手动遥控状态机
    manual_mode = False        # 是否处于手动遥控行进状态
    backward_mode = False      # 是否处于视觉辅助后退模式
    manual_vx = 0.0            # 手动 X 轴目标速度
    manual_vy = 0.0            # 手动 Y 轴目标速度

    target_lost_time = time.ticks_ms()
    last_data_time = time.ticks_ms()
    _hold_ex, _hold_dist = 0.0, 150.0

    pending_bt_ok = False

    while True:
        loop_start = time.ticks_ms()

        # ═══════════════════════════════════════════════════════
        # 蓝牙处理（协议指令解析与路由）
        # ═══════════════════════════════════════════════════════
        bt_cmd = slave_bt.read_command()
        if bt_cmd is not None:
            cmd_type = bt_cmd.get("type")

            # 1. 右转指令 '0'：累加绝对偏航旋转 -90°，旋转后等待视觉对齐再应答 OK
            if cmd_type == "turn_right":
                target_global_yaw -= ROTATE_DEG
                target_global_yaw = ((target_global_yaw + 180) % 360) - 180

                if rotator is not None:
                    rotator.rotate_to_abs(target_global_yaw)

                if heading_hold is not None:
                    heading_hold.set_target(target_global_yaw)

                manual_mode = False
                backward_mode = False
                pending_bt_ok = True
                tracking_state = False
                ctrl.reset_vision()
                if USE_KALMAN:
                    kf.reset()
                last_data_time = time.ticks_ms()
                target_lost_time = time.ticks_ms()

            # 2. 左转指令 '1'：累加绝对偏航旋转 +90°，旋转后等待视觉对齐再应答 OK
            elif cmd_type == "turn_left":
                target_global_yaw += ROTATE_DEG
                target_global_yaw = ((target_global_yaw + 180) % 360) - 180

                if rotator is not None:
                    rotator.rotate_to_abs(target_global_yaw)

                if heading_hold is not None:
                    heading_hold.set_target(target_global_yaw)

                manual_mode = False
                backward_mode = False
                pending_bt_ok = True
                tracking_state = False
                ctrl.reset_vision()
                if USE_KALMAN:
                    kf.reset()
                last_data_time = time.ticks_ms()
                target_lost_time = time.ticks_ms()

            # 3. 变速度多方向遥控指令
            elif cmd_type == "direction":
                direction = bt_cmd.get("direction")
                speed = max(0, bt_cmd.get("speed") - 0.05)

                if direction == 'B':
                    # 后退：视觉辅助修正横向 + 固定后退速度
                    slave_bt.send_ok()
                    manual_mode = False
                    backward_mode = True
                    manual_vx = -speed * 0.4
                else:
                    # 其他方向：纯手动遥控
                    slave_bt.send_ok()
                    manual_mode = True
                    backward_mode = False
                    tracking_state = False

                    if direction == 'F':
                        manual_vx = speed
                        manual_vy = 0.0
                    elif direction == 'L':
                        manual_vx = 0.0
                        manual_vy = -speed
                    elif direction == 'R':
                        manual_vx = 0.0
                        manual_vy = speed
                    else:
                        manual_vx = 0.0
                        manual_vy = 0.0

                # 遥控期间锁定当前角度，防止跑偏
                if heading_hold is not None:
                    imu.update()
                    _, _, yaw_now = imu.get_angles()
                    heading_hold.set_target(yaw_now)

                ctrl.reset_vision()
                last_data_time = time.ticks_ms()
                target_lost_time = time.ticks_ms()

            # 4. 运动结束指令：退出遥控行进，切回跟随模式（立即应答）
            elif cmd_type == "exit":
                slave_bt.send_ok()
                manual_mode = False
                backward_mode = False
                manual_vx, manual_vy = 0.0, 0.0
                stop_all()                 # 刹车

                # 重新使能并重置视觉跟随状态
                tracking_state = True
                ctrl.reset_vision()
                if USE_KALMAN:
                    kf.reset()

                last_data_time = time.ticks_ms()
                target_lost_time = time.ticks_ms()
                loop_start = time.ticks_ms()

        # ═══════════════════════════════════════════════════
        # 层 1：视觉数据刷新（UART 协议解析 + 卡尔曼滤波）
        # ═══════════════════════════════════════════════════
        parsed = parser.poll(loop_start)
        if parsed is not None:
            ex, ey, dist, roll, camera_sees_target = parsed
            last_data_time = loop_start

            # 旋转期间屏蔽视觉：偏航误差 >5° 时忽略摄像头数据，防止旧帧/刷卡误触发
            if heading_hold is not None and abs(yaw_err) > 5.0:
                camera_sees_target = False

            if USE_KALMAN:
                if camera_sees_target:
                    kf.update(ex, dist)
                else:
                    kf.predict_only()

                ex_pred, dist_pred = kf.get_feedforward(T_DELAYS)
                roll_pred = roll
            else:
                ex_pred, dist_pred, roll_pred = ex, dist, roll

            if not camera_sees_target and tracking_state:
                ex_pred, dist_pred = _hold_ex, _hold_dist
            else:
                _hold_ex, _hold_dist = ex_pred, dist_pred

            # 理顺并校准状态机（防止与 backward_mode 的控制发生冲突）
            if camera_sees_target:
                if not tracking_state and not manual_mode:
                    if USE_KALMAN:
                        kf.reset()
                if not manual_mode and not backward_mode:
                    tracking_state = True
                target_lost_time = loop_start
            else:
                if tracking_state and not manual_mode and not backward_mode:
                    if time.ticks_diff(loop_start, target_lost_time) >= LOST_TIMEOUT_MS:
                        tracking_state = False
                        if heading_hold is not None:
                            imu.update()
                            _, _, yaw_now = imu.get_angles()
                            heading_hold.set_target(yaw_now)

        # ═══════════════════════════════════════════════════
        # 层 2：控制计算（始终以 100Hz 运行）
        # ═══════════════════════════════════════════════════
        dt_fixed = TARGET_LOOP_MS / 1000.0

        # ── IMU 偏航保持 → 输出 wz ──
        wz, yaw = 0.0, 0.0
        if heading_hold is not None:
            wz, yaw, yaw_err = heading_hold.compute(dt_fixed)

        if backward_mode:
            # ── 视觉辅助后退：横向 PID 修正，纵向固定速度 ──
            ctrl_input_ex = ex_pred if tracking_state else _hold_ex
            vx_out, vy_out, wz_out, actual_speeds, dt, is_aligned = ctrl.step(
                ctrl_input_ex, TARGET_DIST, tracking_state, wz=wz, vx_extra=manual_vx)

            # 辅助后退过程中，如果开启了 pending_bt_ok，也需支持 OK 发送
            yaw_aligned = (abs(yaw_err) < 4.5) if heading_hold is not None else True
            if pending_bt_ok and tracking_state and is_aligned and yaw_aligned:
                slave_bt.send_ok()
                pending_bt_ok = False

        elif not manual_mode:
            # ── 正常跟随模式 ──
            ctrl_input_ex   = ex_pred   if tracking_state else _hold_ex
            ctrl_input_dist = dist_pred if tracking_state else _hold_dist

            vx_out, vy_out, wz_out, actual_speeds, dt, is_aligned = ctrl.step(
                ctrl_input_ex, ctrl_input_dist, tracking_state, wz=wz)

            # 双重校验：视觉对齐 + 物理角度到位
            yaw_aligned = (abs(yaw_err) < 4.5) if heading_hold is not None else True
            if pending_bt_ok and tracking_state and is_aligned and yaw_aligned:
                slave_bt.send_ok()
                pending_bt_ok = False
        else:
            # ── 蓝牙手动模式（正交平移速度下发 + IMU自稳） ──
            if hasattr(ctrl, "set_speeds"):
                ctrl.set_speeds(manual_vx, manual_vy, wz)
            elif hasattr(ctrl, "drive"):
                ctrl.drive(manual_vx, manual_vy, wz)
            else:
                pass

        # ═══════════════════════════════════════════════════
        # 层 3：安全 + 节拍控制
        # ═══════════════════════════════════════════════════
        if tracking_state and (time.ticks_diff(loop_start, target_lost_time) >= LOST_TIMEOUT_MS):
            tracking_state = False
            if heading_hold is not None:
                heading_hold.set_target(target_global_yaw)

        # 维持 100Hz 控制节拍
        elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
        remaining = TARGET_LOOP_MS - elapsed
        if remaining > 0:
            time.sleep_ms(remaining)

        # 拨码开关退出
        if switch2.value() != state2:
            break

        gc.collect()


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    if imu is not None:
        imu.stop()
    stop_all()
