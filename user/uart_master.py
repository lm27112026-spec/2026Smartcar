"""
MasterBT - 主控车蓝牙从机通信模块

通过 HC-05 蓝牙模块（UART5, 9600 波特）与从控车建立串口通信。
提供位置调整、同步运动、紧急停止等指令的发送与应答处理。

协议说明:
  - POS_ADJ:  发送位置调整，等待 POS_OK 应答（最多 3 次重试）
  - SYNC_MOVE:发送同步运动，不等待应答
  - EMERGENCY_STOP: 紧急停止
  - IMU telemetry (JSON): 发送 IMU 姿态与角速度数据，纯火抛写入，
    JSON 格式: {"roll":R,"pitch":P,"yaw":Y,"wx":X,"wy":Y,"wz":Z}\\r\\n
    所有值使用 {:.1f} 格式化，末尾 \\r\\n 终止。

作者: robot.py 依赖模块
"""

from machine import UART
import time


class MasterBT:
    """主控车蓝牙从机通信控制器"""

    def __init__(self, uart_id=5, baudrate=38400):
        self._uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1)


    def send_imu_data(self, roll, pitch, yaw, wx, wy, wz):
        """发送 IMU 姿态与角速度 JSON 数据，火抛不等待应答。

        参数:
            roll:  横滚角（度）
            pitch: 俯仰角（度）
            yaw:   偏航角（度）
            wx:    X 轴角速度
            wy:    Y 轴角速度
            wz:    Z 轴角速度

        JSON 格式: {"roll":R,"pitch":P,"yaw":Y,"wx":X,"wy":Y,"wz":Z}\\r\\n
        所有值使用 {:.1f} 格式化。
        """
        json_str = '{{"roll":{:.1f},"yaw":{:.1f},"wx":{:.1f},"wy":{:.1f},"wz":{:.1f}}}\r\n'.format(
            roll,yaw, wx, wy, wz
        )
        print("MasterBT: IMU ->", json_str)
        self._uart.write(json_str.encode())


    def turn_left(self):
        """发送数字 1 给从车，表示向左转。火抛不等待应答。"""
        print("MasterBT: turn_left -> 1")
        self._uart.write(b"1\r\n")

    def turn_right(self):
        """发送数字 0 给从车，表示向右转。火抛不等待应答。"""
        print("MasterBT: turn_right -> 0")
        self._uart.write(b"0\r\n")

    
    # ── 接收 ───────────────────────────────────────────

    def read_response(self):
        """非阻塞读取从车应答（一行，\\r\\n 终止）

        返回:
            str: 解码后的字符串（不含 \\r\\n）
            None: 无数据
        """
        if not self._uart.any():
            return None
        raw = self._uart.readline()
        if not raw:
            return None
        try:
            line = raw.decode().strip()
        except UnicodeError:
            return None
        return line if line else None

    def wait_ok(self, timeout_ms=1000):
        """阻塞等待从车发送 'ok' 应答

        参数:
            timeout_ms: 超时时间 (ms)

        返回:
            bool: True=收到 ok, False=超时
        """
        deadline = time.ticks_ms() + timeout_ms
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            resp = self.read_response()
            if resp == "ok":
                return True
            time.sleep_ms(5)
        return False


# ═══════════════════════════════════════════════════════════════
#  独立运行模式：实时 IMU 数据蓝牙发送
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    from imu import IMU as BtIMU
    from machine import Pin

    SWITCH2_PIN = 'D9'
    LED_PIN     = 'C4'

    print("MasterBT standalone — IMU telemetry via Bluetooth (UART5, 9600)")
    print("Toggle SWITCH2 to stop.")

    bt     = MasterBT()
    bt_imu = BtIMU(calibrate_on_init=True)

    led     = Pin(LED_PIN, Pin.OUT, value=True)
    switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
    state2  = switch2.value()

    last_send = 0

    try:
        while True:
            if switch2.value() != state2:
                print("MasterBT standalone: SWITCH2 exit.")
                break

            now = time.ticks_ms()
            if time.ticks_diff(now, last_send) >= 100:   # 10Hz
                d = bt_imu.get_safe()
                if d:
                    bt_imu.update(d)
                    r, p, y = bt_imu.get_angles()
                    wx, wy, wz = bt_imu.get_angular_velocity()
                    bt.send_imu_data(r, p, y, wx, wy, wz)
                last_send = now
                led.toggle()

            time.sleep_ms(10)
    except Exception as e:
        print("MasterBT standalone error:", e)
    finally:
        try:
            bt_imu.stop()
        except Exception:
            pass
        print("MasterBT standalone stopped.")
