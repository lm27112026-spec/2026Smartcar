from machine import UART, Pin, PWM
import time
import math

uart6 = UART(5)
uart6.init(baudrate=9600, bits=8, parity=None, stop=1)

MAX_PWM = 50000

def omni_kinematics(vx, vy, wz):
    w_rf =  vx - vy - wz
    w_lf = -vx - vy - wz
    w_lb = -vx + vy - wz
    w_rb =  vx + vy - wz
    return [w_rf, w_lf, w_lb, w_rb]

def set_motor(motor, duty_val):
    pwm, dir_a, dir_b = motor
    duty = int(abs(duty_val))
    if duty_val > 0:
        dir_a.value(0)
        dir_b.value(1)
        pwm.duty_u16(duty)
    elif duty_val < 0:
        dir_a.value(1)
        dir_b.value(0)
        pwm.duty_u16(duty)
    else:
        dir_a.value(0)
        dir_b.value(0)
        pwm.duty_u16(0)

def omni_drive(vx, vy, wz, max_pwm=MAX_PWM):
    speeds = omni_kinematics(vx, vy, wz)
    max_speed = max(abs(s) for s in speeds)
    scale = 1.0
    if max_speed > 1.0:
        scale = 1.0 / max_speed
    pwm_vals = [int(s * scale * max_pwm) for s in speeds]
    set_motor(MOTOR_RF, pwm_vals[0])
    set_motor(MOTOR_LF, pwm_vals[1])
    set_motor(MOTOR_LB, pwm_vals[2])
    set_motor(MOTOR_RB, pwm_vals[3])

def omni_move_by_angle(speed, angle_deg, rotation=0):
    rad = math.radians(angle_deg)
    vy = speed * math.sin(rad)
    vx = speed * math.cos(rad)
    omni_drive(vx, vy, rotation)

time.sleep_ms(100)

pwm_1 = PWM("B26", 15 * 1000, 0)
pwm_2 = PWM("C20", 15 * 1000, 0)
pwm_3 = PWM("C24", 15 * 1000, 0)
pwm_4 = PWM("C26", 15 * 1000, 0)

pin_c28 = Pin("C28", Pin.OUT)
pin_c29 = Pin("C29", Pin.OUT)
pin_c30 = Pin("C30", Pin.OUT)
pin_c31 = Pin("C31", Pin.OUT)
pin_d4  = Pin("D4",  Pin.OUT)
pin_d5  = Pin("D5",  Pin.OUT)
pin_d6  = Pin("D6",  Pin.OUT)
pin_d7  = Pin("D7",  Pin.OUT)

MOTOR_LF = (pwm_2, pin_c30, pin_c31)
MOTOR_LB = (pwm_1, pin_c28, pin_c29)
MOTOR_RF = (pwm_3, pin_d4,  pin_d5)
MOTOR_RB = (pwm_4, pin_d6,  pin_d7)

print("Slave HC-05 ready, waiting for data...")

while True:
    if uart6.any():
        data = uart6.readline()
        print('Received:', data)
        if data == b'run\r\n' or data == b'run':
            omni_move_by_angle(0.5, 0)
            uart6.write(b'MOTOR_RUN\r\n')
        elif data == b'stop\r\n' or data == b'stop':
            omni_move_by_angle(0, 0, 0)
            uart6.write(b'MOTOR_STOP\r\n')
        else:
            uart6.write(b'ACK\r\n')
    time.sleep_ms(10)