import gc, time
from machine import UART, Pin

time.sleep_ms(100)

LED_PIN = 'C4'
SWITCH2_PIN = 'D9'

led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

BAUD_RATES = [115200, 9600, 19200, 38400, 57600, 230400, 460800]

def test_baudrate(baud, duration_ms=2000):
    uart = UART(7)
    uart.init(baudrate=baud, bits=8, parity=None, stop=1)

    print("\n--- Testing UART(7) @ {} baud ({}ms) ---".format(baud, duration_ms))
    start = time.ticks_ms()
    total_bytes = 0
    frames = 0

    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        if switch2.value() != state2:
            return -1

        n = uart.any()
        if n > 0:
            raw = uart.read(n)
            if raw:
                total_bytes += n
                frames += 1
                hex_str = ' '.join(['{:02X}'.format(b) for b in raw])
                ascii_str = ''.join([chr(b) if 32 <= b < 127 else '.' for b in raw])
                if frames <= 10:
                    print("  [{}] {} bytes: {}".format(frames, n, hex_str))
                    print("  ASCII: {}".format(ascii_str))
                gc.collect()

        time.sleep_ms(1)

    print("  Total: {} bytes in {} frames".format(total_bytes, frames))
    return total_bytes

print("========================================")
print("  UART Diagnostic for UWB Module")
print("  UART(7) = LPUART8 = D22(TX)/D23(RX)")
print("========================================")
print("Press SWITCH2 to stop.\n")

print("Phase 1: Quick scan all baud rates\n")
results = {}
for baud in BAUD_RATES:
    count = test_baudrate(baud, duration_ms=2000)
    results[baud] = count
    if count == -1:
        print("\nTest stopped by user.")
        break
    if count > 0:
        print("  >>> DATA FOUND at {} baud!".format(baud))

print("\n========================================")
print("  Scan Results Summary:")
print("========================================")
for baud in BAUD_RATES:
    count = results.get(baud, 0)
    if count > 0:
        print("  {} baud: {} bytes  <<<".format(baud, count))
    else:
        print("  {} baud: no data".format(baud))

best = max(results, key=results.get) if results else 0
if results.get(best, 0) > 0:
    print("\n>>> Recommended baud rate: {}".format(best))
    print("\nPhase 2: Detailed capture at {} baud for 5 seconds...".format(best))

    uart = UART(7)
    uart.init(baudrate=best, bits=8, parity=None, stop=1)

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
                total += n
                frame_count += 1
                hex_str = ' '.join(['{:02X}'.format(b) for b in raw])
                if frame_count <= 50:
                    print("[Frame {}] {} bytes: {}".format(frame_count, len(raw), hex_str))
                gc.collect()
        time.sleep_ms(1)
    print("\nCapture done: {} bytes, {} frames".format(total, frame_count))
else:
    print("\n>>> No data received on any baud rate!")
    print(">>> Check: 1) UWB module power  2) TX/RX wiring  3) UWB module output enable")

print("\nTest finished.")
gc.collect()
