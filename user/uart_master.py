"""
MasterBT - 主控车蓝牙从机通信模块

通过 HC-05 蓝牙模块（UART5, 9600 波特）与从控车建立串口通信。
提供位置调整、同步运动、紧急停止等指令的发送与应答处理。

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
