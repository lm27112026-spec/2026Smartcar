from machine import UART, Pin
import time
import gc
from motor import stop_all
from kalman_filter import CameraKalmanFilter
from control import CascadeController

# ============================================================
# 硬件初始化
# ============================================================
uart = UART(7, baudrate=9600, bits=8, parity=None, stop=1)
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

# 卡尔曼滤波器 — 对摄像头原始数据 (ex, dist, roll) 降噪
kf = CameraKalmanFilter()

# 级联 PID 控制器 (外环位置环 + 内环速度环)
ctrl = CascadeController()

def main():
    print("System Starting: Cascade PID Control...")
    stop_all()

    last_data_time = time.ticks_ms()
    tracking = False

    while True:
        if uart.any() >= 6:
            buf = uart.read()
            idx = buf.find(b'\xAA')

            if idx != -1 and len(buf) >= idx + 6:
                if buf[idx + 5] == 0xBB:

                    # 1. 数据解析
                    raw_ex   = buf[idx + 1]
                    raw_ey   = buf[idx + 2]
                    dist     = buf[idx + 3]
                    raw_roll = buf[idx + 4]

                    ex   = raw_ex if raw_ex < 128 else raw_ex - 256
                    ey   = raw_ey if raw_ey < 128 else raw_ey - 256
                    roll = raw_roll if raw_roll < 128 else raw_roll - 256

                    last_data_time = time.ticks_ms()

                    # 2. 卡尔曼滤波: 对 ex / dist / roll 降噪
                    ex_f, dist_f, roll_f = kf.update(ex, dist, roll)

                    # 3. 目标检测 (原始值全零 = 摄像头未检测到目标)
                    tracking = not (ex == 0 and ey == 0 and dist == 0 and roll == 0)

                    # 4. 级联 PID 控制 (外环位置 + 内环速度)
                    vx_out, vy_out, wz_out, actual_speeds, dt = ctrl.step(
                        ex_f, dist_f, roll_f, tracking)

                    # 打印状态和数据
                    state = "TRACK" if tracking else "LOST"
                    spd_lf, spd_rf, spd_lb, spd_rb = actual_speeds
                    print("[{}] raw EX:{:4d} Dist:{:3d} Roll:{:4d} | filt EX:{:5.1f} Dist:{:5.1f} Roll:{:5.1f} | vx:{:.3f} vy:{:.3f} wz:{:.3f} | whl {:.3f} {:.3f} {:.3f} {:.3f} | dt:{:.0f}ms".format(
                        state, ex, dist, roll,
                        ex_f, dist_f, roll_f,
                        vx_out, vy_out, wz_out,
                        spd_lf, spd_rf, spd_lb, spd_rb, dt * 1000))

        # 安全看门狗
        if time.ticks_diff(time.ticks_ms(), last_data_time) > 500:
            ctrl.emergency_stop()
            if tracking:
                tracking = False
                print("[LOST] Connection timeout.")

        time.sleep_ms(10)

        # 拨码开关退出逻辑
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