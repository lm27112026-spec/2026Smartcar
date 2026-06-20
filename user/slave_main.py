from machine import UART, Pin
import time
import gc
from motor import (
    stop_all,
    omni_drive_closed_loop,
    get_encoder_speeds_filtered,
    reset_wheel_pi,
    reset_encoder_filter,
)
from control import CascadeController

# ============================================================
# 硬件初始化
# ============================================================
uart = UART(7, baudrate=115200, bits=8, parity=None, stop=1)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# 卡尔曼滤波器 — 关闭，直接用原始数据
# kf = CameraKalmanFilter()

# 级联 PID 控制器
ctrl = CascadeController()

LOST_TIMEOUT_MS = 250        # 丢失确认超时
LOST_ROTATE_WZ = 0.12        # 自转搜索速度（归一化）
LOST_ROTATE_PERIOD_MS = 2000 # 每个方向转2秒后换向
SCALE = 10  # 串口精度：与 OpenMV 一致
_UART_BUF = bytearray()  # UART 接收缓冲区（带帧拼装）

def main():
    print("System Starting: NO FILTER mode...")
    stop_all()

    last_data_time = time.ticks_ms()
    target_lost_time = 0
    tracking_state = False
    lost_rotate_start = 0      # 自转开始时间
    lost_rotate_dir = 1        # 自转方向: 1=CW, -1=CCW
    _loop_time = time.ticks_ms()

    while True:
        now_ms = time.ticks_ms()
        
        # 带缓冲的 UART 读取（兼容帧碎片到达，不被后台 Ticker 干扰）
        if uart.any() > 0:
            chunk = uart.read()
            if chunk:
                _UART_BUF.extend(chunk)
        
        # 防缓冲区无限增长（>1KB 时截断，只保留最后一个可能的帧头位置）
        if len(_UART_BUF) > 1024:
            tail = _UART_BUF.rfind(b'\xAA')
            _UART_BUF = _UART_BUF[tail:] if tail != -1 else bytearray()
        
        # 从缓冲区搜索完整帧（0xAA ... 0xBB）
        idx = _UART_BUF.find(b'\xAA')
        if idx == -1:
            # 没有帧头，清空缓冲区
            _UART_BUF = bytearray()
        elif len(_UART_BUF) >= idx + 10:
            if _UART_BUF[idx + 9] == 0xBB:
                frame = _UART_BUF[idx:idx + 10]
                _UART_BUF = _UART_BUF[idx + 10:]
                
                # 1. 数据解析（int16 大端序，×10 精度）
                raw_ex   = (frame[1] << 8) | frame[2]
                raw_ey   = (frame[3] << 8) | frame[4]
                raw_dist = (frame[5] << 8) | frame[6]
                raw_roll = (frame[7] << 8) | frame[8]

                ex   = (raw_ex if raw_ex < 32768 else raw_ex - 65536) / SCALE
                ey   = (raw_ey if raw_ey < 32768 else raw_ey - 65536) / SCALE
                dist = raw_dist / SCALE
                roll = (raw_roll if raw_roll < 32768 else raw_roll - 65536) / SCALE

                last_data_time = now_ms

                # 直接原始值，不滤波
                ex_f   = ex
                dist_f = dist
                roll_f = roll
                
                camera_sees_target = not (ex == 0 and ey == 0 and dist == 0 and roll == 0)

                # ==========================================
                # 跟踪/丢失 状态机
                # ==========================================
                if camera_sees_target:
                    if not tracking_state:
                        ctrl.emergency_stop()
                        lost_rotate_dir = 1
                    tracking_state = True
                    target_lost_time = 0
                else:
                    if tracking_state:
                        if target_lost_time == 0:
                            target_lost_time = now_ms
                            lost_rotate_start = now_ms
                        if time.ticks_diff(now_ms, target_lost_time) >= LOST_TIMEOUT_MS:
                            tracking_state = False
                            ex_f = dist_f = roll_f = 0.0

                # 2. 控制
                if tracking_state:
                    vx_out, vy_out, wz_out, actual_speeds, dt, is_aligned = ctrl.step(
                        ex_f, dist_f, roll_f, tracking_state)
                else:
                    # 丢失自转搜索：交替方向旋转扫描
                    elapsed = time.ticks_diff(now_ms, lost_rotate_start)
                    if elapsed >= LOST_ROTATE_PERIOD_MS:
                        lost_rotate_dir = -lost_rotate_dir
                        lost_rotate_start = now_ms
                        reset_wheel_pi()
                    dt = max(time.ticks_diff(now_ms, _loop_time) / 1000.0, 0.001)
                    actual_speeds = get_encoder_speeds_filtered(dt)
                    wz_cmd = LOST_ROTATE_WZ * lost_rotate_dir
                    omni_drive_closed_loop(0.0, 0.0, wz_cmd, actual_speeds, dt)
                    vx_out, vy_out, wz_out = 0.0, 0.0, wz_cmd
                    is_aligned = False

                _loop_time = now_ms

                # 3. 打印状态
                state_str = "ALIGN" if is_aligned else ("TRACK" if tracking_state else "SCAN")
                spd_lf, spd_rf, spd_lb, spd_rb = actual_speeds
                print("[{}] EX:{:+6.1f} | Roll:{:+5.1f} dist:{:5.1f} | vx:{:.3f} vy:{:.3f} wz:{:.3f} | dt:{:.0f}ms".format(
                    state_str, ex, roll_f, dist_f, vx_out, vy_out, wz_out, dt * 1000))

        # 安全看门狗
        if time.ticks_diff(now_ms, last_data_time) > 500:
            ctrl.emergency_stop()
            if tracking_state:
                tracking_state = False
                print("[LOST] Connection timeout.")

        time.sleep_ms(10)

        # 拨码开关退出
        if switch2.value() != state2:
            print("Switch triggered. Test program stopped.")
            break

        gc.collect()

try:
    main()
except KeyboardInterrupt:
    print("\nProgram stopped by user.")
finally:
    stop_all()
