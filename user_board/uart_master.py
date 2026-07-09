"""
MasterBT - 主控车蓝牙从机通信模块

【层】控制层 - 蓝牙主控通信
【职责】通过 HC-05 蓝牙模块与从控车建立 UART 串口通信，
        发送转向指令、IMU 姿态遥测数据，并接收从车应答。

【协议】HC-05 蓝牙串口通信协议
    物理层:  UART5, 38400 baud, 8N1 (8 数据位, 无校验, 1 停止位)
    帧格式:  每条消息以 \\r\\n 结尾
    主控 -> 从控:
        - "0\\r\\n"         向右转 (turn_right)
        - "1\\r\\n"         向左转 (turn_left)
        - IMU JSON\\r\\n   姿态遥测, 格式见 send_imu_data()
    从控 -> 主控:
        - "ok\\r\\n"        确认应答
    所有发送指令均为 fire-and-forget（不等待应答），
    仅 wait_ok() 会阻塞等待从车返回 "ok"。

作者: robot.py 依赖模块
"""

from machine import UART
import time

# ── 蓝牙通信常量 ──────────────────────────────────────────
BT_UART_ID = 5          # UART 通道编号 (RT1021 主控板 UART5 连接 HC-05)
BT_BAUDRATE = 38400     # HC-05 蓝牙波特率 (bps)
BT_TIMEOUT_MS = 1000    # wait_ok() 默认超时时间 (毫秒)


class MasterBT:
    """主控车蓝牙从机通信控制器

    参数:
        uart_id (int):  UART 通道编号, 默认 5 (UART5)
        baudrate (int): 串口波特率, 默认 38400
    """

    def __init__(self, uart_id=BT_UART_ID, baudrate=BT_BAUDRATE):
        # 初始化 UART: 8 数据位, 无校验, 1 停止位
        self._uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1)


    def send_imu_data(self, roll, pitch, yaw, wx, wy, wz):
        """发送 IMU 姿态与角速度 JSON 数据，火抛不等待应答。

        参数:
            roll  (float): 横滚角 (度)
            pitch (float): 俯仰角 (度)
            yaw   (float): 偏航角 (度)
            wx    (float): X 轴角速度 (rad/s)
            wy    (float): Y 轴角速度 (rad/s)
            wz    (float): Z 轴角速度 (rad/s)

        协议帧: {"roll":R,"pitch":P,"yaw":Y,"wx":X,"wy":Y,"wz":Z}\\r\\n
        所有值使用 {:.1f} 格式化。
        """
        # 构造 JSON 字符串, {:.1f} 保留一位小数, 末尾追加 \\r\\n
        json_str = '{{"roll":{:.1f},"pitch":{:.1f},"yaw":{:.1f},"wx":{:.1f},"wy":{:.1f},"wz":{:.1f}}}\r\n'.format(
            roll, pitch, yaw, wx, wy, wz
        )
        print("MasterBT: IMU ->", json_str)
        self._uart.write(json_str.encode())


    def turn_left(self):
        """发送左转指令给从车。火抛不等待应答。

        协议帧: "1\\r\\n"
        """
        # 发送字符 "1" + 换行, 从车收到后执行左转
        print("MasterBT: turn_left -> 1")
        self._uart.write(b"1\r\n")

    def turn_right(self):
        """发送右转指令给从车。火抛不等待应答。

        协议帧: "0\\r\\n"
        """
        # 发送字符 "0" + 换行, 从车收到后执行右转
        print("MasterBT: turn_right -> 0")
        self._uart.write(b"0\r\n")


    # ── 接收 ───────────────────────────────────────────

    def read_response(self):
        """非阻塞读取从车应答 (一行, \\r\\n 终止)。

        返回:
            str:  解码后的字符串 (已去除首尾空白)
            None: 无数据或解码失败
        """
        # 检查 UART 接收缓冲区是否有数据
        if not self._uart.any():
            return None
        # 读取一行 (以 \\r\\n 结尾)
        raw = self._uart.readline()
        if not raw:
            return None
        try:
            line = raw.decode().strip()
        except UnicodeError:
            # 解码失败时丢弃损坏数据
            return None
        return line if line else None

    def wait_ok(self, timeout_ms=BT_TIMEOUT_MS):
        """阻塞等待从车发送 'ok' 应答。

        参数:
            timeout_ms (int): 超时时间 (毫秒), 默认 1000

        返回:
            bool: True = 收到 "ok", False = 超时未收到

        实现: 轮询 read_response(), 每次间隔 5ms, 直到超时或匹配 "ok"。
        """
        deadline = time.ticks_ms() + timeout_ms
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            resp = self.read_response()
            if resp == "ok":
                return True
            time.sleep_ms(5)  # 轮询间隔 5ms, 避免 CPU 空转
        return False


