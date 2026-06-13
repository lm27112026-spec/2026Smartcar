"""
uart_slave.py - SlaveBT 蓝牙从机通信模块

通过 UART5 (HC-05 蓝牙模块) 接收来自主机的控制指令，并返回响应。
本模块仅处理通信协议解析，不包含任何电机驱动代码。

协议格式:
  POS_ADJ:<vx>,<vy>,<wz>   -- 位置微调
  SYNC_MOVE:<vx>,<vy>,<wz> -- 同步运动
  EMERGENCY_STOP            -- 紧急停止
  响应: POS_OK\r\n 或 ERROR:<code>\r\n

用法:
  bt = SlaveBT()
  cmd = bt.read_command()
  if cmd:
      print(cmd)
      bt.send_ok()
"""

from machine import UART


class SlaveBT:
    """蓝牙从机通信类，接收解析主机指令并发送响应。"""

    def __init__(self, uart_id=5, baudrate=9600):
        self._uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1)

    def read_command(self):
        """非阻塞读取并解析一条指令。
        
        返回:
          dict: {"type": "pos_adj"|"sync_move"|"emergency_stop", ...}
          None: 无数据或格式错误
        """
        if not self._uart.any():
            return None

        raw = self._uart.readline()
        if raw is None or raw == b'':
            return None

        try:
            line_str = raw.decode().strip()
        except UnicodeError:
            return None

        if not line_str:
            return None

        # EMERGENCY_STOP (no colon)
        if line_str == "EMERGENCY_STOP":
            return {"type": "emergency_stop"}

        # All other commands have "KEYWORD:value" format
        if ':' not in line_str:
            print("uart_slave: unknown format:", line_str)
            return None

        left, _, right = line_str.partition(':')

        if left == "POS_ADJ":
            parts = right.split(',')
            if len(parts) != 3:
                return None
            try:
                vx = float(parts[0])
                vy = float(parts[1])
                wz = float(parts[2])
            except ValueError:
                return None
            return {"type": "pos_adj", "vx": vx, "vy": vy, "wz": wz}

        elif left == "SYNC_MOVE":
            parts = right.split(',')
            if len(parts) != 3:
                return None
            try:
                vx = float(parts[0])
                vy = float(parts[1])
                wz = float(parts[2])
            except ValueError:
                return None
            return {"type": "sync_move", "vx": vx, "vy": vy, "wz": wz}

        else:
            print("uart_slave: unknown command:", left)
            return None

    def send_ok(self):
        """发送成功响应。"""
        self._uart.write(b'POS_OK\r\n')

    def send_error(self, code):
        """发送错误响应。"""
        self._uart.write("ERROR:{:d}\r\n".format(code).encode())
