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

    def __init__(self, uart_id=5, baudrate=9600):
        self._uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1)

    def send_pos_adjust(self, vx, vy, wz):
        """发送位置调整指令，等待从机应答 POS_OK

        参数:
            vx: X 轴速度/位置
            vy: Y 轴速度/位置
            wz: Z 轴（角速度）调整量

        返回:
            True   - 从机应答 POS_OK
            False  - 超时无应答
        """
        cmd = "POS_ADJ:{:.3f},{:.3f},{:.3f}\r\n".format(vx, vy, wz)
        cmd_bytes = cmd.encode()

        for attempt in range(3):
            print("MasterBT: send ->", cmd.strip())
            self._uart.write(cmd_bytes)

            deadline = time.ticks_ms() + 3000
            ok = False
            while time.ticks_diff(deadline, time.ticks_ms()) > 0:
                if self._uart.any():
                    line = self._uart.readline()
                    if line == b"POS_OK\r\n" or line == b"POS_OK":
                        ok = True
                        break
                time.sleep_ms(10)

            if ok:
                return True

            if attempt < 2:
                print("MasterBT: retry {}/2 for POS_ADJ".format(attempt + 1))

        print("MasterBT: POS_ADJ failed after 3 attempts")
        return False

    def read_response_ok(self):
        """非阻塞检查从机是否回复 POS_OK（使用 read() 避免 readline 阻塞）。

        返回:
            True  - 收到 POS_OK
            False - 尚无有效应答（调用方应稍后重试）
        """
        while self._uart.any():
            data = self._uart.read()
            if data and b"POS_OK" in data:
                return True
        return False

    def send_pos_adjust_async(self, vx, vy, wz):
        """发送 POS_ADJ 但不等待应答（测试用，发送后即忘）。"""
        cmd = "POS_ADJ:{:.3f},{:.3f},{:.3f}\r\n".format(vx, vy, wz)
        print("MasterBT: send ->", cmd.strip())
        self._uart.write(cmd.encode())

    def send_sync_move(self, vx, vy, wz):
        """发送同步运动指令，不等待应答（发送后即忘）

        参数:
            vx: X 轴速度
            vy: Y 轴速度
            wz: Z 轴（角速度）
        """
        cmd = "SYNC_MOVE:{:.3f},{:.3f},{:.3f}\r\n".format(vx, vy, wz)
        print("MasterBT: send ->", cmd.strip())
        self._uart.write(cmd.encode())

    def send_emergency_stop(self):
        """发送紧急停止指令（发送后即忘）"""
        cmd = b"EMERGENCY_STOP\r\n"
        print("MasterBT: send -> EMERGENCY_STOP")
        self._uart.write(cmd)

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
        json_str = '{{"roll":{:.1f},"pitch":{:.1f},"yaw":{:.1f},"wx":{:.1f},"wy":{:.1f},"wz":{:.1f}}}\r\n'.format(
            roll, pitch, yaw, wx, wy, wz
        )
        print("MasterBT: IMU ->", json_str)
        self._uart.write(json_str.encode())


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
