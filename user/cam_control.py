"""
test_follow.py — 视觉跟随与对齐策略专用调试脚本

【测试目的】
  1. 验证 cam_follow.py 中 X 轴(横移)和 Y 轴(前后)的 PID 跟随效果。
  2. 测试 DX0/DY0 死区、BX/BY 刹车带参数的平滑度。
  3. 观察对齐阈值 (ALGN_X, ALGN_Y) 的判定触发时机。
  4. 验证 yaw_hold.py 在跟随过程中抑制底盘自旋的稳定性。
"""

import gc, time
from machine import Pin

from imu import IMU
from cam_data import CamDataReceiver, x_to_cm, y_to_distance
from cam_follow import compute_control, reset_control, DIST, ALGN_X, ALGN_Y, E_X
# E_X 可在此处覆盖: cam_follow.E_X = 5.0  # 偏右5cm跟随
from yaw_hold import YawHolder
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_speeds_filtered, get_encoder_counts,
                   reset_encoder_filter, reset_wheel_pi)


# ═══════════════════════════════════════════════════════════════
#  CameraController — 供 main.py 调用的摄像头跟随封装
# ═══════════════════════════════════════════════════════════════

class CameraController:
    """摄像头目标跟随控制器（不含航向保持，wz 由外部提供）"""

    def __init__(self, uart_id=7):
        self._recv = CamDataReceiver(uart_id)
        self._last_x_cm = 0.0
        self._last_dist_cm = 0.0
        self._last_has_tgt = False
        self._t_prev = 0
        self._first = True
        reset_control(reset_state=True)

    # ── 属性（只读统计） ──
    @property
    def frame_count(self):
        return self._recv.frame_count

    @property
    def target_count(self):
        return self._recv.target_count

    @property
    def error_count(self):
        return self._recv.error_count

    def flush(self):
        """清空摄像头串口缓冲区（循环读取直到无数据）"""
        for _ in range(20):
            if self._recv.read() is None:
                break

    def reset(self):
        """复位 PID + 状态机 + 清空坐标缓存"""
        reset_control(reset_state=True)
        self._last_x_cm = 0.0
        self._last_dist_cm = 0.0
        self._last_has_tgt = False
        self._first = True
        self.flush()

    def step(self, now_ms=None):
        """
        一帧控制计算（封装 cam_control.py L79-115 逻辑）。

        参数:
            now_ms: 当前时间戳 (ms)，用于 dt 计算；None 则取 time.ticks_ms()

        返回: dict {
            'vx': float|None,       # 前进速度 (m/s)，None=不应驱动
            'vy': float|None,       # 横向速度 (m/s)
            'has_target': bool,     # 当前是否检测到目标
            'x_cm': float,          # 目标横向偏移 (cm)
            'dist_cm': float,       # 目标距离 (cm)
            'obj_id': int,          # 目标 ID
            'line_flag': bool,      # 黄线标志（从 b7 解析，预留）
            'arrived': bool,        # 是否到达目标距离
            'state': int,           # 0=FOLLOW, 1=STOPPED, 2=LOST
            'state_msg': str|None,  # 状态切换消息（供打印）
        }
        """
        if now_ms is None:
            now_ms = time.ticks_ms()

        # ── dt 计算 ──
        if self._first:
            self._t_prev = now_ms
            self._first = False
            dt_act = 0.01  # 首帧用默认值
        else:
            dt_act = time.ticks_diff(now_ms, self._t_prev) * 0.001
            self._t_prev = now_ms
            if dt_act <= 0 or dt_act > 0.1:
                dt_act = 0.01

        # ── 读取摄像头 ──
        cam_data = self._recv.read()

        has_tgt = False
        x_cm = 0.0
        dist_cm = 0.0
        obj_id = 0
        line_flag = False

        if cam_data is not None:
            has_tgt = cam_data['is_target']
            obj_id = cam_data['id']
            # 【待确认】黄线标志 — 按实际协议修改此行
            line_flag = bool(cam_data.get('b7', 0) & 0x01)
            if has_tgt:
                x_cm = x_to_cm(cam_data['x'])
                dist_cm = y_to_distance(cam_data['y'])
                self._last_x_cm = x_cm
                self._last_dist_cm = dist_cm
                self._last_has_tgt = True
            elif self._last_has_tgt:
                x_cm = self._last_x_cm
                dist_cm = self._last_dist_cm
        else:
            if self._last_has_tgt:
                x_cm = self._last_x_cm
                dist_cm = self._last_dist_cm

        # ── 级联控制计算 ──
        ctrl = compute_control(x_cm, dist_cm, has_tgt, now_ms, dt_act)

        return {
            'vx':         ctrl['cmd_fwd'],
            'vy':         ctrl['cmd_lat'],
            'has_target': has_tgt,
            'x_cm':       x_cm,
            'dist_cm':    dist_cm,
            'obj_id':     obj_id,
            'line_flag':  line_flag,
            'arrived':    ctrl.get('arrived', False),
            'state':      ctrl['state'],
            'state_msg':  ctrl.get('state_msg'),
        }


# ═══════════════════════════════════════════════════════════════
#  cam_approach() — 摄像头驱动闭环靠近目标直到到达判定
#  提取自 main.py see_and_push() 的驱动闭环部分（不含蓝牙通信）
# ═══════════════════════════════════════════════════════════════

APPROACH_TIMEOUT_S       = 15.0   # 靠近超时 (s)
APPROACH_CTRL_DT         = 0.02   # 控制周期 (s)
APPROACH_DEADBAND_DEG    = 0.5    # 航向死区 (度)
APPROACH_THRESHOLD_CM    = 6.0    # 物理距离防撞兜底 (cm)
CLOSE_LOSS_THRESHOLD_CM  = 16.0   # 近距离丢锁判定阈值 (cm)
APPROACH_PRINT_INTERVAL_MS = 500  # 状态打印间隔 (ms)


def cam_approach(cam, lock_heading_fn, calc_wz_fn,
                 should_abort_fn, drive_fn, stop_fn, led_fn=None):
    """摄像头驱动闭环靠近目标直到到达判定。

    参数（均为回调函数，由调用方注入）:
        cam:              CameraController 实例
        lock_heading_fn:  () -> float  锁定并返回目标航向角 (度)
        calc_wz_fn:       (target_deg, deadband_deg) -> float  计算航向修正 wz
        should_abort_fn:  () -> bool   检查 SW2/看门狗，返回 True 则中断
        drive_fn:         (vx, vy, wz, dt)  驱动电机（须内部读取编码器）
        stop_fn:          ()  停止所有电机
        led_fn:           (bool) -> None  设置 LED，None 表示不控制

    返回:
        (arrived: bool, reason: str)
        - (True, 'aligned')         → PID 对齐到达
        - (True, 'safety_threshold')→ 物理防撞兜底
        - (True, 'blind_zone')      → 近距离盲区遮挡到位
        - (False, 'lost_far')       → 远距离丢锁，需重新搜索
        - (False, 'timeout')        → 超时
        - (False, 'aborted')        → SW2 中断
    """
    cam.reset()

    target_heading = lock_heading_fn()
    print("  [APPROACH] 航向锁定: {:.1f}°".format(target_heading))

    if led_fn:
        led_fn(True)

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    loop_cnt = 0
    has_found_once = False
    last_known_dist = 999.0

    while True:
        if should_abort_fn():
            if led_fn:
                led_fn(False)
            return (False, 'aborted')

        elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed > APPROACH_TIMEOUT_S:
            print("  [APPROACH] 靠近超时 ({:.1f}s)".format(elapsed))
            if led_fn:
                led_fn(False)
            return (False, 'timeout')

        ctrl = cam.step()

        if ctrl['state_msg']:
            print("\n" + ctrl['state_msg'])

        if ctrl['has_target']:
            has_found_once = True
            if 0 < ctrl['dist_cm']:
                last_known_dist = ctrl['dist_cm']

        # ── ① PID 精确对齐到达 ──
        if ctrl['arrived']:
            print("\n  [APPROACH] ➔ 触发内环 PID 精确对齐到位")
            stop_fn()
            break

        # ── ② 物理距离防撞兜底 ──
        if ctrl['has_target'] and 0 < ctrl['dist_cm'] <= APPROACH_THRESHOLD_CM:
            print("\n  [APPROACH] ➔ 物理距离 D:{:.1f}cm 已达到防撞界限 (<= {:.1f}cm)".format(
                ctrl['dist_cm'], APPROACH_THRESHOLD_CM))
            stop_fn()
            break

        # ── ③ 丢锁滤波中断判定 ──
        if has_found_once and ctrl['state'] == 2:
            if last_known_dist <= CLOSE_LOSS_THRESHOLD_CM:
                print("\n  [APPROACH] ➔ 目标在近距离 ({:.1f}cm <= {:.1f}cm) 连续丢失，判定盲区遮挡到位！".format(
                    last_known_dist, CLOSE_LOSS_THRESHOLD_CM))
                stop_fn()
                break
            else:
                print("\n  [APPROACH] ➔ 目标在远距离 ({:.1f}cm) 异常丢失！返回重新搜索。".format(
                    last_known_dist))
                stop_fn()
                cam.reset()
                return (False, 'lost_far')

        # ── 驱动电机 ──
        if ctrl['vx'] is not None and ctrl['vy'] is not None:
            wz = calc_wz_fn(target_heading, APPROACH_DEADBAND_DEG)
            try:
                drive_fn(ctrl['vx'], ctrl['vy'], wz, APPROACH_CTRL_DT)
            except Exception as e:
                print("  [APPROACH] 驱动错误:", e)
        else:
            wz = calc_wz_fn(target_heading, APPROACH_DEADBAND_DEG)
            if abs(wz) > 0.001:
                try:
                    drive_fn(0, 0, wz, APPROACH_CTRL_DT)
                except Exception:
                    pass
            else:
                stop_fn()

        # ── 状态打印 ──
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= APPROACH_PRINT_INTERVAL_MS:
            last_print_ms = now
            tgt_str = "T" if ctrl['has_target'] else "-"
            print("  [APPROACH] {0} X:{1:+5.1f}cm D:{2:5.1f}cm vx:{3:.2f} vy:{4:.2f}".format(
                tgt_str, ctrl['x_cm'], ctrl['dist_cm'],
                ctrl['vx'] if ctrl['vx'] else 0.0,
                ctrl['vy'] if ctrl['vy'] else 0.0))

        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        time.sleep_ms(int(APPROACH_CTRL_DT * 1000))

    if led_fn:
        led_fn(False)
    return (True, 'arrived')


# ── 调试配置 ──
LOOP_MS  = 10               # 控制周期 100Hz
DT       = 0.01
PRINT_MS = 200              # 终端打印间隔 (ms)
SW2_PIN  = 'D9'

def main():
    gc.collect()

    sw2 = Pin(SW2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
    sw2_start_state = sw2.value()

    # ── 2. IMU 与航向保持初始化 ──
    print("[INIT] 校准 IMU 陀螺仪...")
    imu = IMU(calibrate_on_init=True, calib_samples=300)
    imu.start()
    time.sleep_ms(200)
    imu.set_zero_reference()
    
    hold = YawHolder(imu)
    hold.set_target(0)
    print("[INIT] 航向保持就绪，锁定当前朝向(0°)")

    # ── 3. 视觉与电机初始化 ──
    recv = CamDataReceiver(uart_id=7)
    print("[INIT] 摄像头串口就绪")

    stop_all()
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)
    reset_encoder_filter()
    reset_wheel_pi()
    reset_control(reset_state=True)
    
    print("=" * 50)
    print(" 开始跟随测试 | 目标对齐距离: {:.0f}cm | 按 SW2 退出".format(DIST))
    print("=" * 50)

    t_prev = time.ticks_ms()
    t_print = time.ticks_ms()
    loop_cnt = 0

    # 目标短暂丢失时保留上一帧有效坐标，防止跳变到(0,0)误驱动
    last_x_cm = 0.0
    last_dist_cm = 0.0
    last_has_tgt = False

    try:
        while True:
            t_now = time.ticks_ms()
            dt_act = time.ticks_diff(t_now, t_prev) * 0.001
            t_prev = t_now
            if dt_act <= 0 or dt_act > 0.1:
                dt_act = DT

            # ── 获取传感数据 ──
            cam_data = recv.read()
            imu.update()
            
            # 计算航向补偿 WZ (独立闭环)
            wz = hold.compute(dt_act)

            # ── 视觉跟随解算 ──
            has_tgt = False
            x_cm = 0.0
            dist_cm = 0.0
            obj_id = 0
            line_flag = 0

            if cam_data is not None:
                has_tgt = cam_data['is_target']
                obj_id = cam_data['id']
                line_flag = cam_data['line_flag']
                if has_tgt:
                    x_cm = x_to_cm(cam_data['x'])
                    dist_cm = y_to_distance(cam_data['y'])
                    # 缓存有效坐标 — 短暂丢失时避免跳变到(0,0)
                    last_x_cm = x_cm
                    last_dist_cm = dist_cm
                    last_has_tgt = True
                elif last_has_tgt:
                    # 摄像头帧存在但目标丢失 → 用上一帧有效值过渡
                    x_cm = last_x_cm
                    dist_cm = last_dist_cm
            else:
                if last_has_tgt:
                    # 无 UART 数据 → 用上一帧有效值过渡
                    x_cm = last_x_cm
                    dist_cm = last_dist_cm

            # 将目标状态输入级联控制器
            ctrl = compute_control(x_cm, dist_cm, has_tgt, t_now, dt_act)

            # 状态切换日志 (独立换行)
            if ctrl['state_msg']:
                print("\n" + ctrl['state_msg'])

            # ── 速度合成与底层闭环 ──
            vx = ctrl['cmd_fwd']
            vy = ctrl['cmd_lat']

            # 黄线越界 → 立即停车 (测试跟随时注释)
            # if line_flag:
            #     print("\n[LINE] 黄线越界 → 停车!")
            #     stop_all()
            #     break

            if vx is not None and vy is not None:
                # 在跟随状态
                speeds = get_encoder_speeds_filtered(dt_act)
                omni_drive_closed_loop(vx, vy, wz, speeds, dt_act)
            else:
                # 丢失或达到停止条件，仅维持航向
                if abs(wz) > 0.001:
                    speeds = get_encoder_speeds_filtered(dt_act)
                    omni_drive_closed_loop(0, 0, wz, speeds, dt_act)
                else:
                    stop_all()

            # ── 状态打印 (供参数调优使用) ──
            if time.ticks_diff(t_now, t_print) >= PRINT_MS:
                state_str = "FOLLOW" if ctrl['state'] == 0 else ("STOP" if ctrl['state'] == 1 else "LOST")
                tgt_str = "T" if has_tgt else "-"
                arrived_str = "ARRIVED!" if ctrl.get('arrived') else ""

                # 读取状态原始值用于诊断
                raw_st = cam_data['status'] if cam_data else 0

                # 对齐判定标记 (观测 ALGN_X/ALGN_Y 阈值)
                algn_x = "✓" if abs(x_cm - E_X) < ALGN_X else " "
                algn_y = "✓" if abs(dist_cm - DIST) < ALGN_Y else " "

                # 修改为打印 vx 和 vy 的期望值
                print("[{:04d} {}] ID:{:d} | X:{:+5.1f} Y:{:5.1f} | vx:{:.2f} vy:{:.2f}".format(
                    loop_cnt, state_str, obj_id, x_cm, dist_cm, 
                    ctrl['cmd_fwd'] if ctrl['cmd_fwd'] else 0.0, 
                    ctrl['cmd_lat'] if ctrl['cmd_lat'] else 0.0
                ), end='\r')
                
                t_print = t_now

            # ── 退出检测与节拍维持 ──
            if sw2.value() != sw2_start_state:
                print("\n[EXIT] SW2 触发退出。")
                break

            elap = time.ticks_diff(time.ticks_ms(), t_now)
            if elap < LOOP_MS:
                time.sleep_ms(LOOP_MS - elap)

            loop_cnt += 1
            if loop_cnt % 50 == 0:
                gc.collect()

    except KeyboardInterrupt:
        print("\n[EXIT] 键盘中断。")
    finally:
        stop_all()
        imu.stop()
        print("测试结束，电机已锁定。")

if __name__ == '__main__':
    main()
