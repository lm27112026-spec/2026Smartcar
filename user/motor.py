"""
motor.py - 电机与编码器硬件抽象层
【层】硬件抽象层
【功能】
  - 4 路 PWM + 方向引脚 → 驱动 4 个全向轮电机
  - 4 路编码器读取（smartcar.encoder）→ 获取各轮转速
【依赖】smartcar 库（encoder 类）、seekfree 库
【使用】
  from motor import set_motor, omni_drive, get_encoder_speeds
"""

import gc, time, math
from machine import *
from smartcar import *
from seekfree import *
from pid import PID

ANG_OFFSET = 0.0
MAX_PWM = 50000
LED_PIN = 'C4'
SWITCH2_PIN = 'D10'

# 编码器标定因子（脉冲数/米），4个轮子各自独立
# ⚠ 实测编码器在轮轴侧（减速后），PPR=7，无减速比倍乘
#   7 脉冲/轮圈 ÷ (π × 0.05m/轮圈) = 44.6 脉冲/米
#   各轮独立标定方法：让车轮空转 N 圈 → total_pulses / (N * 0.1571)
ENC_SCALE = [833, 840, 853, 817]  # [rf, lf, lb, rb] — 2026-06-08 去掉负号

# ============================================================
#  一、编码器引脚定义 & 初始化
#  接线表（编码器 A/B 相已对调以修正计数方向）:
#    电机位置   编码器 A 相   编码器 B 相  （驱动方向  → 计数符号）
# 右前 (RF)      C3           C2          正 PWM → 正计数
# 左前 (LF)      D14          D13         正 PWM → 正计数
# 左后 (LB)      D16          D15         正 PWM → 正计数
# 右后 (RB)      C1           C0          正 PWM → 正计数
# ============================================================

ENCODER_RF_A, ENCODER_RF_B = 'C3',  'C2'   # 右前编码器
ENCODER_LF_A, ENCODER_LF_B = 'D14', 'D13'  # 左前编码器
ENCODER_LB_A, ENCODER_LB_B = 'D16', 'D15'  # 左后编码器
ENCODER_RB_A, ENCODER_RB_B = 'C1',  'C0'   # 右后编码器

encoder_rf = encoder(ENCODER_RF_A, ENCODER_RF_B, capture_div=1)
encoder_lf = encoder(ENCODER_LF_A, ENCODER_LF_B, capture_div=1)
encoder_lb = encoder(ENCODER_LB_A, ENCODER_LB_B, capture_div=1)
encoder_rb = encoder(ENCODER_RB_A, ENCODER_RB_B, capture_div=1)

def get_encoder_counts():
    """
    返回 4 个编码器脉冲增量 [rf, lf, lb, rb]
    每次调用返回自上次 get() 以来的脉冲变化量，停止时为 0
    """
    return [encoder_rf.get(), encoder_lf.get(), encoder_lb.get(), encoder_rb.get()]


def get_encoder_speeds(dt):
    """
    返回 4 个编码器转速（米/秒）[rf, lf, lb, rb]
    dt: 采样间隔（秒），应与主控制循环周期一致
    """
    counts = get_encoder_counts()
    return [c / ENC_SCALE[i] / dt for i, c in enumerate(counts)]


def reset_encoders():
    """空函数（保留接口兼容，encoder.get() 自带增量特性无需清零）"""
    pass


# ============================================================
#  二-a、编码器速度滤波（一阶低通）
# ============================================================

_prev_spd = [0.0, 0.0, 0.0, 0.0]
_spd_first = True
SPD_FILTER_ALPHA = 0.4

def get_encoder_speeds_filtered(dt):
    """
    返回低通滤波后的 4 轮速度 [rf, lf, lb, rb]（米/秒）
    第一次调用直接返回原始值，后续做一阶低通
    """
    global _prev_spd, _spd_first
    raw = get_encoder_speeds(dt)
    if _spd_first:
        _prev_spd = raw[:]
        _spd_first = False
        return _prev_spd
    _prev_spd = [
        SPD_FILTER_ALPHA * p + (1 - SPD_FILTER_ALPHA) * r
        for p, r in zip(_prev_spd, raw)
    ]
    return _prev_spd


# ============================================================
#  二-b、闭环驱动（前馈 + PI 反馈）
# ============================================================

CTRL_DT = 0.02                 # 控制周期 20ms（与主循环一致）

WHEEL_PI = [
    PID(kp=8000, ki=1000, kd=0.0, integral_limit=5000, output_limit=MAX_PWM)
    for _ in range(4)
]

SPD_DEADBAND = 0.005           # 5mm/s 以下视为静止，清零积分

MAX_SPEED_MPS = [0.123, 0.123, 0.123, 0.123]  # [rf, lf, lb, rb] 由 encoder_check.py 标定
PWM_PER_MPS   = [405455, 405455, 405455, 405455]  # [rf, lf, lb, rb]


def omni_drive_closed_loop(vx, vy, wz, actual_speeds=None):
    """
    前馈 + PI 反馈闭环全向驱动（替换 omni_drive 用于直线行驶）
    vx, vy, wz: 归一化值（-1 ~ 1），同 omni_drive 语义
    actual_speeds: 外部传入的 4 轮速度 [rf,lf,lb,rb]（米/秒）
                   为 None 时内部自动读取编码器
    """
    norms = omni_kinematics(vx, vy, wz)
    if actual_speeds is None:
        actual = get_encoder_speeds_filtered(CTRL_DT)
    else:
        actual = actual_speeds
    motor_list = [MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB]

    for i in range(4):
        target_mps = norms[i] * MAX_SPEED_MPS[i]

        if abs(target_mps) < SPD_DEADBAND:
            WHEEL_PI[i].reset()
            set_motor(motor_list[i], 0)
            continue

        feedforward = target_mps * PWM_PER_MPS[i]
        correction = WHEEL_PI[i].compute(target_mps, actual[i])
        final_pwm = feedforward + correction
        final_pwm = max(-MAX_PWM, min(final_pwm, MAX_PWM))

        set_motor(motor_list[i], int(final_pwm))


# ============================================================
#  三、运动学模型 & 电机驱动
# ============================================================

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


def omni_move_by_angle(speed, angle_deg, rotation=0, max_pwm=MAX_PWM):
    rad = math.radians(angle_deg)
    vx = speed * math.sin(rad)
    vy = speed * math.cos(rad)
    omni_drive(vx, vy, rotation, max_pwm)

# ============================================================
#  四、硬件初始化（PWM + 方向引脚）
#  这些代码在 import motor 时自动执行一次
# ============================================================

time.sleep_ms(100)
print("REAL TYPE : " + BOARD_TYPE)
print("BOARD VERSION : " + BOARD_VERSION)

pwm_1 = PWM("B26", 15000, duty_u16=0)
pwm_2 = PWM("C20", 15000, duty_u16=0)
pwm_3 = PWM("C24", 15000, duty_u16=0)
pwm_4 = PWM("C26", 15000, duty_u16=0)

pin_c28 = Pin("C28", Pin.OUT, value=0)
pin_c29 = Pin("C29", Pin.OUT, value=0)
pin_c30 = Pin("C30", Pin.OUT, value=0)
pin_c31 = Pin("C31", Pin.OUT, value=0)
pin_d4  = Pin("D4",  Pin.OUT, value=0)
pin_d5  = Pin("D5",  Pin.OUT, value=0)
pin_d6  = Pin("D6",  Pin.OUT, value=0)
pin_d7  = Pin("D7",  Pin.OUT, value=0)

# 电机元组 (PWM, 方向A, 方向B)
# TB6612 驱动板映射 — 2026-06-08 实测
MOTOR_RF = (pwm_4, pin_d6,  pin_d7)   # C26 + D6/D7
MOTOR_LF = (pwm_3, pin_d4,  pin_d5)   # C24 + D4/D5
MOTOR_LB = (pwm_2, pin_c30, pin_c31)  # C20 + C30/C31
MOTOR_RB = (pwm_1, pin_c28, pin_c29)  # B26 + C28/C29

def stop_all():
    """急停：所有电机方向引脚置 0，PWM 置 0"""
    for pin in (pin_c28, pin_c29, pin_c30, pin_c31, pin_d4, pin_d5, pin_d6, pin_d7):
        pin.low()
    for pwm in (pwm_1, pwm_2, pwm_3, pwm_4):
        pwm.duty_u16(0)


# 导入完成后立即强制停机一次
stop_all()
