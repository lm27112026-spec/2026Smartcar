from machine import UART, Pin, PWM
import time
import math
import time

uart6 = UART(5)
uart6.init(baudrate=9600, bits=8, parity=None, stop=1)

print("Master HC-05 ready, sending...")

while True:
    cmd = b'run/r/n'
    uart6.write(cmd)
    print('Sent:', cmd)

    time.sleep_ms(100)
    if uart6.any():
        reply = uart6.readline()
        print('Reply from slave:', reply)

    time.sleep(1)