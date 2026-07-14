"""
uart_slave.py - UART 从机蓝牙接收解析模块

通过 UART5 (HC-05 蓝牙模块) 接收并解析主控车 MasterBT 指令。

协议格式（来自 uart_master.py MasterBT）:
  {"roll":R,"yaw":Y,"wx":X,"wy":Y,"wz":Z}\r\n   -- IMU 姿态角速度 (JSON, {:.1f} 格式化)
  "1\r\n"                                        -- 左转 (turn_left)
  "0\r\n"                                        -- 右转 (turn_right)

应答:
  收到指令后发送 "ok\r\n" 应答。
"""

from machine import UART

try:
    import ujson as json
except ImportError:
    import json

# ── 协议常量 ─────────────────────────────────────────

_IMU_FIELDS = ('roll', 'yaw', 'wx', 'wy', 'wz')       # 主控车发送的 IMU JSON 字段（不含 pitch）
_CMD_LEFT  = '1'        # 左转
_CMD_RIGHT = '0'        # 右转


class SlaveBT:
    """UART 从机蓝牙接收解析类"""

    def __init__(self, uart_id=5, baudrate=38400):
        self._uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1, timeout=10)

    # ── 发送 ─────────────────────────────────────────

    def send(self, data: str):
        """向主控车发送字符串（自动追加 \\r\\n）。"""
        self._uart.write((data + '\r\n').encode())

    def send_ok(self):
        """向主控车发送 'ok' 应答。"""
        self.send('ok')

    # ── 接收解析 ─────────────────────────────────────

    def read_command(self):
        """非阻塞读取并解析一条指令。

        返回:
            dict: {"type": "imu_data",       "roll":..., "yaw":..., "wx":..., "wy":..., "wz":...}
                  {"type": "turn_left"}
                  {"type": "turn_right"}
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
            result = self._parse_imu_json(line)
            return result

        # 左转
        if line == _CMD_LEFT:
            return {'type': 'turn_left'}

        # 右转
        if line == _CMD_RIGHT:
            return {'type': 'turn_right'}

        return None

    def _parse_imu_json(self, line: str):
        """解析主控车发送的 IMU JSON 数据。

        必须包含字段: roll, yaw, wx, wy, wz（不含 pitch）。
        """
        try:
            data = json.loads(line)
            if all(k in data for k in _IMU_FIELDS):
                return {
                    'type': 'imu_data',
                    'roll': float(data['roll']),
                    'yaw':  float(data['yaw']),
                    'wx':   float(data['wx']),
                    'wy':   float(data['wy']),
                    'wz':   float(data['wz']),
                }
        except (ValueError, TypeError, KeyError):
            pass
        return None



# # ═══════════════════════════════════════════════════════════════
# #  独立运行模式：实时接收并打印主控车指令
# # ═══════════════════════════════════════════════════════════════

# if __name__ == '__main__':
#     from machine import Pin
#     import time

#     SWITCH2_PIN = 'D9'
#     LED_PIN     = 'C4'

#     print('SlaveBT standalone — listening on UART5 (9600)')
#     print('Toggle SWITCH2 to stop.')

#     slave = SlaveBT()
#     led   = Pin(LED_PIN, Pin.OUT, value=True)
#     sw2   = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
#     sw2_last = sw2.value()

#     try:
#         while True:
#             if sw2.value() != sw2_last:
#                 print('SlaveBT standalone: SWITCH2 exit.')
#                 break

#             cmd = slave.read_command()
#             if cmd is not None:
#                 print('[RECV]', cmd)
#                 led.toggle()

#             time.sleep_ms(10)
#     except Exception as e:
#         print('SlaveBT standalone error:', e)
#     finally:
#         print('SlaveBT standalone stopped.')

