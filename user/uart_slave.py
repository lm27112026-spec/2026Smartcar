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


    def send_ok(self):
        """向主车发送 'ok' 应答"""
        self.send("ok")

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

        # rotate 指令
        if line == "1":
            return {"type": "rotate"}

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


# ═══════════════════════════════════════════════════════════════
#  独立运行模式：实时接收并打印主机指令
# ═══════════════════════════════════════════════════════════════

if __name__ == "__main__":
    from machine import Pin
    import time

    SWITCH2_PIN = 'D9'
    LED_PIN     = 'C4'

    print("SlaveBT standalone — listening on UART5 (9600)")
    print("Toggle SWITCH2 to stop.")

    slave = SlaveBT()
    led   = Pin(LED_PIN, Pin.OUT, value=True)
    sw2   = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
    sw2_last = sw2.value()

    try:
        while True:
            if sw2.value() != sw2_last:
                print("SlaveBT standalone: SWITCH2 exit.")
                break

            cmd = slave.read_command()
            if cmd is not None:
                print("[RECV]", cmd)
                led.toggle()

            time.sleep_ms(10)
    except Exception as e:
        print("SlaveBT standalone error:", e)
    finally:
        print("SlaveBT standalone stopped.")
