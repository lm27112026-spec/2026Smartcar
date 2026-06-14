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
from smartcar import ticker
from smartcar import encoder
from pid import PID

ANG_OFFSET = 0.0
MAX_PWM = 50000
LED_PIN = 'C4'
SWITCH2_PIN = 'D9'

# 编码器标定因子（脉冲数/米），4个轮子各自独立
# ⚠ 实测编码器在轮轴侧（减速后），PPR=7，无减速比倍乘
#   7 脉冲/轮圈 ÷ (π × 0.05m/轮圈) = 44.6 脉冲/米
#   各轮独立标定方法：让车轮空转 N 圈 → total_pulses / (N * 0.1571)
ENC_SCALE = [1621, 1519, 1624, 1535]  # [rf, lf, lb, rb] — 2026-06-11 绝对距离法标定

# ============================================================
#  一、编码器引脚定义 & 初始化
#  接线表（编码器 A/B 相已对调以修正计数方向）:
# 电机位置   	编码器 A 相     编码器 B 相  	  PWM脚     电机
# 左前 (LF)      C3             C2                C24       M3
# 右前 (RF)      C1             C0                C26       M4
# 左后 (LB)      D14            D13           	  C20       M2
# 右后 (RB)      D16            D15               B26       M1
# ============================================================

ENCODER_LF_A, ENCODER_LF_B = 'C3', 'C2'     # 左前编码器
ENCODER_RF_A, ENCODER_RF_B = 'C1', 'C0'     # 右前编码器
ENCODER_LB_A, ENCODER_LB_B = 'D14', 'D13'   # 左后编码器 
ENCODER_RB_A, ENCODER_RB_B = 'D16', 'D15'   # 右后编码器

encoder_lf = encoder(ENCODER_LF_A, ENCODER_LF_B)
encoder_rf = encoder(ENCODER_RF_A, ENCODER_RF_B)
encoder_lb = encoder(ENCODER_LB_A, ENCODER_LB_B)
encoder_rb = encoder(ENCODER_RB_A, ENCODER_RB_B)


enc_ticker = ticker(1) 
# 将四个编码器统统挂载到定时器的自动采集列表上
enc_ticker.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
# 启动定时器，底层开始以 10ms 为周期疯狂帮你采集数据
enc_ticker.start(10)

def get_encoder_counts():
    """
    返回 4 个编码器脉冲增量 [rf, lf, lb, rb]
    每次调用返回自上次 get() 以来的脉冲变化量，停止时为 0
    """
    encoder_rf.capture()
    encoder_lf.capture()
    encoder_lb.capture()
    encoder_rb.capture()

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


def reset_encoder_filter():
    """重置速度滤波器状态（用于主循环启动前的一次性初始化）"""
    global _spd_first
    _spd_first = True


# ============================================================
#  二-b、闭环驱动（前馈 + PI 反馈）
# ============================================================
WHEEL_PI = [
    PID(kp=40000, ki=0, kd=0.0, integral_limit=5000, output_limit=MAX_PWM)
    for _ in range(4)
]

SPD_DEADBAND = 0.005           # 5mm/s 以下视为静止，清零积分

MAX_SPEED_MPS = [0.425, 0.431, 0.437, 0.424]  # [rf, lf, lb, rb] — (MAX_PWM-MIN_PWM)/PWM_PER_MPS
PWM_PER_MPS   = [105985, 104300, 102874, 106076]  # [rf, lf, lb, rb] — 2026-06-11 04标定


def reset_wheel_pi():
    """重置所有轮子 PI 控制器积分（方向切换 / 段间调用）"""
    for pi in WHEEL_PI:
        pi.reset()

# 新增电机死区补偿参数（需要实测，填入电机刚开始转动的最小 PWM）
MIN_PWM_POS = 5000   # 正转起步 PWM（后轮需要更大死区补偿）
MIN_PWM_NEG = 5000   # 反转起步 PWM（前轮反转超速，降低死区补偿）

# 闭环控制函数：根据目标速度和实际速度计算 PWM 输出
def omni_drive_closed_loop(vx, vy, wz, actual_speeds, dt):
    # 运动学解算获取各轮目标速度 (米/秒)
    norms = omni_kinematics(vx, vy, wz)
    motor_list = [MOTOR_RF, MOTOR_LF, MOTOR_LB, MOTOR_RB]

    for i in range(4):
        target_mps = norms[i] * MAX_SPEED_MPS[i]

        # 速度死区：指令极小时直接刹车，清空 PID
        if abs(target_mps) < SPD_DEADBAND:
            WHEEL_PI[i].reset()
            set_motor(motor_list[i], 0)
            continue

        # 【核心改进】：前馈计算引入死区补偿
        if target_mps > 0:
            feedforward = MIN_PWM_POS + (target_mps * PWM_PER_MPS[i])
        else:
            feedforward = -MIN_PWM_NEG + (target_mps * PWM_PER_MPS[i])

        # PID 反馈修正 (基于重构后的 pid.py)
        correction = WHEEL_PI[i].compute(target_mps, actual_speeds[i], dt)
        
        # 总输出 = 基础前馈 + 反馈动态调整
        final_pwm = feedforward + correction
        final_pwm = max(-MAX_PWM, min(final_pwm, MAX_PWM))

        set_motor(motor_list[i], int(final_pwm))


# ============================================================
#  三、运动学模型 & 电机驱动
# ============================================================

def omni_kinematics(vx, vy, wz):
    w_rf =  vx - vy + wz
    w_lf = -vx - vy + wz
    w_lb = -vx + vy + wz
    w_rb =  vx + vy + wz
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

# 开环驱动函数：直接根据输入的 vx, vy, wz 计算 PWM 输出，无速度反馈
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
MOTOR_LF = (pwm_3, pin_d4,  pin_d5)   # C24 + D4/D5
MOTOR_RF = (pwm_4, pin_d6,  pin_d7)   # C26 + D6/D7
MOTOR_LB = (pwm_2, pin_c30, pin_c31)  # C20 + C30/C31
MOTOR_RB = (pwm_1, pin_c28, pin_c29)  # B26 + C28/C29

def pause_encoder_ticker():
    """暂停编码器自动采集 — 用于手动接管编码器读取的模块调用。
    调用者必须在完成后调用 resume_encoder_ticker() 恢复。"""
    enc_ticker.stop()

def resume_encoder_ticker():
    """恢复编码器自动采集（10ms 周期）。"""
    enc_ticker.start(10)


def stop_all():
    """急停：所有电机方向引脚置 0，PWM 置 0"""
    for pin in (pin_c28, pin_c29, pin_c30, pin_c31, pin_d4, pin_d5, pin_d6, pin_d7):
        pin.low()
    for pwm in (pwm_1, pwm_2, pwm_3, pwm_4):
        pwm.duty_u16(0)


# 导入完成后立即强制停机一次
stop_all()


