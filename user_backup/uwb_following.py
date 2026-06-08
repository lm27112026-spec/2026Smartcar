import gc, time, json
from machine import UART, Pin
from motor import omni_move_by_angle, stop_all

time.sleep_ms(100)

LED_PIN = 'C4'
SWITCH2_PIN = 'D9'

led     = Pin(LED_PIN, Pin.OUT, value=True)



switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

APPROACH_SPEED  = 0.30
STOP_DIST_M     = 0.20
TARGET_ANCHOR   = "8834"
ANGLE_TIMEOUT_MS = 800
last_data_ticks = time.ticks_ms()

P_FILT_ALPHA   = 0.3
ANGLE_DEADBAND = 10
p_filt = 0.0

D_FILT_ALPHA   = 0.3
RESTART_DIST_M = 0.25
d_filt = 0.0
is_stopped = False

uart = UART(7)
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
timeout_stopped = False
loop_count = 0

while True:
    if time.ticks_diff(time.ticks_ms(), last_data_ticks) > ANGLE_TIMEOUT_MS:
        if not timeout_stopped:
            stop_all()
            timeout_stopped = True

    if switch2.value() != state2:
        stop_all()
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
                            angle_p = twr.get('P', 0)
                            last_data_ticks = time.ticks_ms()
                            timeout_stopped = False

                            p_filt = P_FILT_ALPHA * angle_p + (1 - P_FILT_ALPHA) * p_filt
                            angle_for_motor = -p_filt
                            if abs(angle_for_motor) < ANGLE_DEADBAND:
                                angle_for_motor = 0

                            d_filt = D_FILT_ALPHA * d_cm + (1 - D_FILT_ALPHA) * d_filt
                            dist_m_filt = d_filt / 100.0

                            print("[{}] a={} D={} D_filt={:.1f} P_raw={:+.0f} P_filt={:+.0f} → mot={:+.0f}".format(
                                frame_count, anchor, d_cm, d_filt, angle_p, p_filt, angle_for_motor))

                            if TARGET_ANCHOR is not None and str(anchor) != TARGET_ANCHOR:
                                pass
                            elif is_stopped:
                                if dist_m_filt > RESTART_DIST_M:
                                    is_stopped = False
                                    omni_move_by_angle(APPROACH_SPEED, angle_for_motor)
                            elif dist_m_filt <= STOP_DIST_M:
                                is_stopped = True
                                stop_all()
                            else:
                                omni_move_by_angle(APPROACH_SPEED, angle_for_motor)

                            led.toggle()

                        rx_line = bytearray()
                    continue

                rx_line.append(b)
                if len(rx_line) > 200:
                    rx_line = bytearray()

    loop_count += 1
    if loop_count % 50 == 0:
        gc.collect()
    time.sleep_ms(1)
