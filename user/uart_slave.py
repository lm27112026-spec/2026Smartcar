"""
uart_slave.py - UART 数据接收解析模块

通过 UART5 (HC-05 蓝牙模块) 接收并解析主机指令。

协议格式:
  EMERGENCY_STOP            -- 紧急停止
  {"roll":...,"pitch":...,"yaw":...,"wx":...,"wy":...,"wz":...} -- IMU 数据 (JSON)

用法:
  slave = SlaveBT()
  cmd = slave.read_command()
  if cmd:
      print(cmd)
"""

from machine import UART

try:
    import ujson as json
except ImportError:
    import json


class SlaveBT:
    """UART 数据接收解析类"""

    def __init__(self, uart_id=5, baudrate=9600):
        self._uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1, timeout=10)

    def read_command(self):
        """非阻塞读取并解析一条指令。
        
        返回:
          dict: {"type": "emergency_stop"} 或 {"type": "imu_data", ...}
          None: 无数据或格式错误
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

        if not line:
            return None

        # JSON IMU 数据
        if line.startswith('{'):
            return self._parse_json(line)

        # EMERGENCY_STOP
        if line == "EMERGENCY_STOP":
            return {"type": "emergency_stop"}

        return None

    def _parse_json(self, line):
        """解析 JSON 格式 IMU 数据"""
        try:
            data = json.loads(line)
            required = ('roll', 'pitch', 'yaw', 'wx', 'wy', 'wz')
            if all(k in data for k in required):
                return {
                    "type": "imu_data",
                    "roll": float(data['roll']),
                    "pitch": float(data['pitch']),
                    "yaw": float(data['yaw']),
                    "wx": float(data['wx']),
                    "wy": float(data['wy']),
                    "wz": float(data['wz']),
                }
        except (ValueError, TypeError, KeyError):
            pass
        return None
