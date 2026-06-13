import gc, time, json
from machine import UART, Pin

time.sleep_ms(100)

LED_PIN = 'C4'
SWITCH2_PIN = 'D9'

led     = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

uart = UART(0)
uart.init(baudrate=115200, bits=8, parity=None, stop=1)

rx_line = bytearray()

def parse_json_line(line_str):
    try:
        idx = line_str.find('{')
        if idx < 0:
            return None
        return json.loads(line_str[idx:])
    except:
        return None

print("=== UWB Receive Test (UART7 115200) ===")
print("Waiting for UWB data...")

frame_count = 0

while True:
    if switch2.value() != state2:
        print("Test program stop.")
        break

    if uart.any():
        raw = uart.read(uart.any())
        if raw:
            for i in range(len(raw)):
                b = raw[i]

                if b == 0x0D or b == 0x0A:
                    if len(rx_line) > 0:
                        line_str = ''
                        for c in rx_line:
                            line_str += chr(c)

                        data = parse_json_line(line_str)
                        if data and 'TWR' in data:
                            frame_count += 1
                            twr = data['TWR']
                            anchor = twr.get('a16', '?')
                            d_cm = twr.get('D', 0)
                            dist_m = d_cm / 100.0

                            print("[Frame {}] anchor={} dist={:.2f}m (D={})".format(
                                frame_count, anchor, dist_m, d_cm))
                            led.toggle()

                        rx_line = bytearray()
                    continue

                rx_line.append(b)
                if len(rx_line) > 200:
                    rx_line = bytearray()

    gc.collect()
    time.sleep_ms(1)
