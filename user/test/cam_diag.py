"""
cam_diag.py — UART7 原始字节诊断
【功能】不做任何协议解析，直接 dump 所有收到的原始字节
        用于排查：摄像头是否在发送数据、波特率是否正确、数据格式是什么
【使用】运行后观察 Thonny Shell 输出
        SW2 拨动退出
"""

import gc, time
from machine import UART, Pin

# ── 硬件初始化 ──────────────────────────────────────────────
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

uart_cam = UART(7)
uart_cam.init(baudrate=115200, bits=8, parity=None, stop=1)

print("=" * 60)
print("UART7 Raw Byte Diag — 115200 8N1")
print("Waiting for any bytes on UART7...")
print("SW2 toggle = exit")
print("=" * 60)

total_bytes = 0
last_print  = time.ticks_ms()

while True:
    if switch2.value() != state2:
        print("\n[EXIT] SW2 toggled. total_bytes={:d}".format(total_bytes))
        break

    n = uart_cam.any()
    if n > 0:
        buf = uart_cam.read(n)
        if buf:
            total_bytes += len(buf)
            hex_str = ' '.join('{:02X}'.format(b) for b in buf)
            ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in buf)
            print("[+{:4d}B total={:6d}] HEX: {:s}".format(len(buf), total_bytes, hex_str))
            print("                   ASCII: {:s}".format(ascii_str))
    else:
        time.sleep_ms(5)

    # 每 2 秒无数据打印一次心跳
    now = time.ticks_ms()
    if time.ticks_diff(now, last_print) >= 2000:
        if total_bytes == 0:
            print("[...] No bytes received yet (total=0)")
        last_print = now

    gc.collect()
