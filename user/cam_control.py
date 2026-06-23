"""
main.py — 全向轮视觉追踪主控

【状态机】
    INIT → RIGHT → FOLLOW → ALIGN → STOP
     上电   右移搜索  视觉跟随   对齐前行   停车(黄线/超时)

【控制层级】
    外环 cam_follow  (视觉→速度)  +  yaw_hold  (IMU→wz)
    内环 motor       (速度→PWM)  4轮闭环PI

【依赖】 imu, cam_data, cam_follow, yaw_hold, motor, pid
"""

import gc, time
from machine import Pin

from imu import IMU
from cam_data import CamDataReceiver, x_to_cm, y_to_distance
from cam_follow import compute_control, reset_control, ALGN_X, ALGN_Y, E_X, DIST
from yaw_hold import YawHolder
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_speeds_filtered, get_encoder_counts,
                   reset_encoder_filter, reset_wheel_pi)


# ═══════════════════════════════════════════════════════════════
#  状态机
# ═══════════════════════════════════════════════════════════════

ST_INIT   = 0               # 上电: 存IMU角度
ST_RIGHT  = 1               # 右移搜索目标
ST_FOLLOW = 2               # 视觉跟随
ST_ALIGN  = 3               # 对齐前行等黄线
ST_STOP   = 4               # 停车

S = {0: "INIT", 1: "RIGHT", 2: "FOLLOW", 3: "ALIGN", 4: "STOP"}


# ═══════════════════════════════════════════════════════════════
#  运动参数
# ═══════════════════════════════════════════════════════════════

V_RIGHT = 0.10              # 右移搜索速度 (m/s)
V_FWD   = 0.20              # 对齐后前行速度 (m/s)


# ═══════════════════════════════════════════════════════════════
#  航向保持参数
# ═══════════════════════════════════════════════════════════════

YAW_P  = 0.01               # 航向 P 增益
YAW_I  = 0.001              # 航向 I 增益
YAW_DZ = 2.0                # 航向死区 (°)
WZ_LIM = 0.20               # WZ 输出限幅


# ═══════════════════════════════════════════════════════════════
#  时序 & 硬件
# ═══════════════════════════════════════════════════════════════

LOOP_MS  = 10               # 主循环周期 (ms)  100Hz
DT       = 0.01             # 默认控制周期 (s)
PRINT_MS = 300              # 调试打印间隔 (ms)
UART_WDT = 500              # UART看门狗 (ms)  超时→STOP

LED = 'C4'
SW2 = 'D9'


# ═══════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════

def main():
    gc.collect()

    # ── 硬件 ──
    led    = Pin(LED, Pin.OUT, value=1)
    sw2    = Pin(SW2, Pin.IN, pull=Pin.PULL_UP_47K)
    sw2_st = sw2.value()

    # ── 模块: IMU ──
    print("[IMU] 校准中...")
    imu = IMU(calibrate_on_init=True, calib_samples=500)
    imu.start()
    time.sleep_ms(200)
    imu.set_zero_reference()
    print("[IMU] 就绪, 零参考已存")

    # ── 模块: 摄像头 ──
    recv = CamDataReceiver(uart_id=7)
    print("[CAM] 就绪")

    # ── 模块: 航向保持 ──
    hold = YawHolder(imu, kp=YAW_P, ki=YAW_I, deadband=YAW_DZ, wz_max=WZ_LIM)
    hold.set_target(0)
    print("[YAW] 就绪, 目标=0°")

    # ── 电机准备 ──
    time.sleep_ms(100)
    stop_all()
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)
    reset_encoder_filter()
    reset_wheel_pi()

    # ── 状态机 ──
    state  = ST_INIT
    t_last = time.ticks_ms()           # UART 看门狗
    t_prev = time.ticks_ms()           # dt 计算基准
    cnt    = 0
    t_pr   = time.ticks_ms()
    last_x = 0.0          # FOLLOW/ALIGN 丢失时保留上一帧有效坐标
    last_d = 0.0

    print("-" * 50)
    print("INIT → RIGHT → FOLLOW → ALIGN → STOP")
    print("-" * 50)

    try:
        while True:
            t0 = time.ticks_ms()

            # 实际控制周期 (s)
            dt_act = time.ticks_diff(t0, t_prev) * 0.001
            t_prev = t0
            if dt_act <= 0 or dt_act > 0.1:
                dt_act = DT              # 异常值回退默认

            # ═══════════════════════════════════════════════════
            #  数据刷新
            # ═══════════════════════════════════════════════════
            data = recv.read()
            if data is not None:
                t_last = t0

            # ═══════════════════════════════════════════════════
            #  line_flag 全局最高优先 → STOP
            # ═══════════════════════════════════════════════════
            if data is not None and data['line_flag'] and state != ST_STOP:
                state = ST_STOP
                stop_all()
                print(">>> [LINE] 黄线 → STOP <<<")

            # ═══════════════════════════════════════════════════
            #  IMU 更新
            # ═══════════════════════════════════════════════════
            imu.update()
            _, _, yaw = imu.get_angles()
            wz = hold.compute(dt_act)

            # ═══════════════════════════════════════════════════
            #  状态机
            # ═══════════════════════════════════════════════════

            # ── INIT → RIGHT ──
            if state == ST_INIT:
                state = ST_RIGHT
                print("[INIT → RIGHT] 开始右移搜索")

            # ── RIGHT: 右移搜索 ──
            elif state == ST_RIGHT:
                if data is not None and data['is_target']:
                    state = ST_FOLLOW
                    reset_control(reset_state=True)
                    print("[RIGHT → FOLLOW] 发现目标")
                else:
                    spd = get_encoder_speeds_filtered(dt_act)
                    omni_drive_closed_loop(0, V_RIGHT, wz, spd, dt_act)

            # ── FOLLOW: 视觉跟随 ──
            elif state == ST_FOLLOW:
                if data is not None:
                    x = x_to_cm(data['x'])
                    d = y_to_distance(data['y'])
                    ctrl = compute_control(x, d, data['is_target'], t0, dt_act)
                    if data['is_target']:
                        last_x = x; last_d = d       # 缓存有效坐标
                else:
                    # UART 丢失 → 用上一帧有效值过渡，防跳变到 (0,0)
                    ctrl = compute_control(last_x, last_d, False, t0, dt_act)
                    x = last_x; d = last_d

                if ctrl['state_msg']:
                    print(ctrl['state_msg'])

                if ctrl['cmd_fwd'] is not None:
                    spd = get_encoder_speeds_filtered(dt_act)
                    omni_drive_closed_loop(ctrl['cmd_fwd'], ctrl['cmd_lat'], wz, spd, dt_act)

                    # 对齐判定 → ALIGN
                    if data is not None and data['is_target']:
                        if abs(x - E_X) < ALGN_X and abs(d - DIST) < ALGN_Y:
                            state = ST_ALIGN
                            reset_control(reset_state=True)
                            print("[FOLLOW → ALIGN] X={:.1f} Y={:.1f}".format(x, d))
                else:
                    # cmd_fwd=None → 目标丢失, 重回搜索
                    state = ST_RIGHT
                    reset_control(reset_state=True)
                    stop_all()
                    print("[FOLLOW → RIGHT] 目标丢失")

            # ── ALIGN: 对齐前行, 等黄线 ──
            elif state == ST_ALIGN:
                if data is not None and data['is_target']:
                    x = x_to_cm(data['x'])
                    d = y_to_distance(data['y'])
                    ctrl = compute_control(x, d, True, t0, dt_act)
                    vy = ctrl['cmd_lat'] if ctrl['cmd_lat'] is not None else 0
                    vx = V_FWD
                    last_x = x; last_d = d            # 缓存有效坐标
                else:
                    vy = 0
                    vx = 0                            # 丢失时禁止盲开前行

                if vx != 0 or vy != 0:
                    spd = get_encoder_speeds_filtered(dt_act)
                    omni_drive_closed_loop(vx, vy, wz, spd, dt_act)
                elif abs(wz) > 0.001:
                    spd = get_encoder_speeds_filtered(dt_act)
                    omni_drive_closed_loop(0, 0, wz, spd, dt_act)
                else:
                    stop_all()

            # ── STOP: 停车 ──
            elif state == ST_STOP:
                stop_all()

            # ═══════════════════════════════════════════════════
            #  安全 + 打印 + 节拍
            # ═══════════════════════════════════════════════════

            # 看门狗: UART 超时
            if time.ticks_diff(t0, t_last) > UART_WDT:
                if state != ST_STOP:
                    state = ST_STOP
                    stop_all()
                    print("[WDT] UART 超时 → STOP")

            # 调试打印
            now = time.ticks_ms()
            if time.ticks_diff(now, t_pr) >= PRINT_MS:
                yerr = hold.get_yaw_error()

                if state == ST_RIGHT:
                    tgt = "T" if (data and data['is_target']) else "-"
                    print("[{:05d} {:s}] {:s} yaw={:+5.1f}° err={:+4.1f}° R={:.2f}".format(
                        cnt, S[state], tgt, yaw, yerr, V_RIGHT))

                elif state == ST_FOLLOW:
                    tgt = "T" if (data and data['is_target']) else "-"
                    x   = data['x'] if data else 0
                    y   = data['y'] if data else 0
                    print("[{:05d} {:s}] {:s} X={:+5.1f} Y={:5.1f} yaw={:+5.1f}°".format(
                        cnt, S[state], tgt, x, y, yaw))

                elif state == ST_ALIGN:
                    tgt = "T" if (data and data['is_target']) else "-"
                    x   = data['x'] if data else 0
                    print("[{:05d} {:s}] {:s} X={:+5.1f} FWD={:.2f} 等黄线".format(
                        cnt, S[state], tgt, x, V_FWD))

                elif state == ST_STOP:
                    print("[{:05d} {:s}] 停车 yaw={:+5.1f}°".format(cnt, S[state], yaw))

                else:
                    print("[{:05d} {:s}] yaw={:+5.1f}°".format(cnt, S[state], yaw))

                led.toggle()
                t_pr = now

            # 安全退出
            if sw2.value() != sw2_st:
                print("\n[EXIT] SW2")
                break

            # 节拍维持 (100Hz)
            elap = time.ticks_diff(time.ticks_ms(), t0)
            if elap < LOOP_MS:
                time.sleep_ms(LOOP_MS - elap)

            cnt += 1
            if cnt % 50 == 0:
                gc.collect()

    except KeyboardInterrupt:
        pass
    finally:
        stop_all()
        imu.stop()
        led.value(1)
        print("\n" + "=" * 50)
        print("最终状态: {:s} | 循环: {:d} | 帧: {:d}".format(
            S[state], cnt, recv.frame_count))
        print("=" * 50)


if __name__ == '__main__':
    main()

