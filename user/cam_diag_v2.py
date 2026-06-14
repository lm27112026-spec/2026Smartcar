"""
cam_diag_v2.py — UART7 全波特率扫描诊断
【功能】自动扫描 6 种常见波特率，检测 UART7 是否有任何数据
        无论 OpenMV 发什么格式都能看到原始字节
【使用】运行后等待约 30 秒，自动扫描完成并给出结果
        SW2 拨动退出
"""

import gc, time
from machine import UART, Pin

# ── 硬件初始化 ──────────────────────────────────────────────
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

print("=" * 60)
print("UART7 Full Baudrate Scanner")
print("UART(7) = LPUART8 = D22(TX) / D23(RX)")
print("=" * 60)

BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400]

def scan_baudrate(baud, duration_ms=3000):
    """扫描指定波特率，返回收到的字节数和前几帧数据"""
    uart = UART(7)
    uart.init(baudrate=baud, bits=8, parity=None, stop=1)

    start = time.ticks_ms()
    total_bytes = 0
    frames = []
    frame_count = 0

    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        if switch2.value() != state2:
            return -1, []

        n = uart.any()
        if n > 0:
            raw = uart.read(n)
            if raw:
                total_bytes += len(raw)
                frame_count += 1
                if frame_count <= 5:
                    frames.append(raw)
                gc.collect()
        time.sleep_ms(1)

    return total_bytes, frames

# ── Phase 1: 快速扫描所有波特率 ──
print("\nPhase 1: Scanning all baud rates (3s each)...\n")

results = {}
for baud in BAUD_RATES:
    if switch2.value() != state2:
        print("\n[EXIT] Stopped by user.")
        break

    count, frames = scan_baudrate(baud, duration_ms=3000)
    results[baud] = (count, frames)

    if count == -1:
        print("\n[EXIT] Stopped by user.")
        break
    elif count > 0:
        print("  {:>7d} baud: {:>6d} bytes  <<< DATA FOUND".format(baud, count))
    else:
        print("  {:>7d} baud: no data".format(baud))

# ── Phase 2: 汇总 ──
print("\n" + "=" * 60)
print("Scan Results:")
print("=" * 60)

found_any = False
best_baud = 0
best_count = 0

for baud in BAUD_RATES:
    count, frames = results.get(baud, (0, []))
    if count > 0:
        print("  {} baud: {} bytes  <<<".format(baud, count))
        found_any = True
        if count > best_count:
            best_count = count
            best_baud = baud
    else:
        print("  {} baud: no data".format(baud))

if found_any:
    print("\n>>> Best baud rate: {} ({} bytes)".format(best_baud, best_count))
    print("\nPhase 2: Detailed capture at {} baud for 5s...\n".format(best_baud))

    uart = UART(7)
    uart.init(baudrate=best_baud, bits=8, parity=None, stop=1)

    start = time.ticks_ms()
    total = 0
    frame_count = 0

    while time.ticks_diff(time.ticks_ms(), start) < 5000:
        if switch2.value() != state2:
            break
        n = uart.any()
        if n > 0:
            raw = uart.read(n)
            if raw:
                total += len(raw)
                frame_count += 1
                if frame_count <= 50:
                    hex_str = ' '.join(['{:02X}'.format(b) for b in raw])
                    ascii_str = ''.join([chr(b) if 32 <= b < 127 else '.' for b in raw])
                    print("[Frame {:>3d}] {:>3d} bytes: {}".format(frame_count, len(raw), hex_str))
                gc.collect()
        time.sleep_ms(1)

    print("\nCapture: {} bytes, {} frames at {} baud".format(total, frame_count, best_baud))
else:
    print("\n>>> NO DATA on any baud rate!")
    print(">>> Check these:")
    print("    1. OpenMV power LED on?")
    print("    2. OpenMV running a UART send script?")
    print("    3. OpenMV TX -> Board D23 (RX)")
    print("    4. OpenMV RX -> Board D22 (TX)")
    print("    5. GND connected between both boards?")

print("\n" + "=" * 60)
print("Done.")
