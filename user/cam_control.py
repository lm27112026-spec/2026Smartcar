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
