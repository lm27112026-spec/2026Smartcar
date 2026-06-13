"""
test_raw_json.py — 打印 UWB 模块的完整原始 JSON 帧
"""

import time, json
from machine import UART

uart = UART(0)
uart.init(baudrate=115200, bits=8, parity=None, stop=1)

rx_line = bytearray()

print("=" * 60)
print("UWB 原始 JSON 帧打印")
print("=" * 60)

for _ in range(50):
    if uart.any():
        raw = uart.read(uart.any())
        if raw:
            for i in range(len(raw)):
                b = raw[i]
                if b == 0x0D or b == 0x0A:
                    if len(rx_line) > 0:
                        line_str = ''.join(chr(c) for c in rx_line)
                        try:
                            idx = line_str.find('{')
                            if idx >= 0:
                                data = json.loads(line_str[idx:])
                                # 打印全部字段
                                if 'TWR' in data:
                                    twr = data['TWR']
                                    print("TWR keys:", list(twr.keys()), "  values:", twr)
                        except:
                            pass
                        rx_line = bytearray()
                    continue
                rx_line.append(b)
    time.sleep_ms(10)

print("=" * 60)
print("Done")
