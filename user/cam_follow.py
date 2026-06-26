"""
cam_follow.py — 视觉跟随级联控制器

【架构】 外环位置PID → 死区+刹车带 → 动态速度预算 → 内环轮速PI
【依赖】 pid.py, motor.py
【API】
    from cam_follow import compute_control, reset_control
    ctrl = compute_control(x_cm, dist_cm, has_tgt, now_ms, dt)
    ctrl['cmd_fwd']  # vx (m/s)
    ctrl['cmd_lat']  # vy (m/s)
"""

import time
from pid import PID
from motor import stop_all, reset_wheel_pi


# ═══════════════════════════════════════════════════════════════
#  跟踪参数
# ═══════════════════════════════════════════════════════════════

DIST       = 8          # 期望距离 (cm)  纵向目标（太近摄像头丢失）
E_X        = 5           # 期望横向距离 (cm)  正=偏左目标

# ── 外环 PID 增益 ──
PX  = 0.05                # 横向 P  (x_err → vy)
IX  = 0.003                # 横向 I
DX  = 0.01                # 横向 D

PY  = 0.05                 # 纵向 P  (y_err → vx)
IY  = 0.003                # 纵向 I
DY  = 0.01                # 纵向 D

# ── 死区 + 刹车带 ──
DX0 = 0.5                  # 横向死区 (cm)  |x|<DX0 → 冻结PID/输出0
DY0 = 0.5                  # 纵向死区 (cm)  |y|<DY0 → 冻结PID/输出0
BX  = 1.0                  # 横向刹车带宽 (cm)  区间 DX0~DX0+BX → smoothstep
BY  = 1.0                  # 纵向刹车带宽 (cm)  区间 DY0~DY0+BY → smoothstep

# ── 输出限幅 ──
VX_MAX = 1.0              # 前后最大 (m/s)
VY_MAX = 1.0              # 横向最大 (m/s)

# ── 动态速度预算 ──
VBUD   = 0.80              # 总预算 (归一化) 预留0.25给WZ航向
VY_LO  = 0.25              # 横向占比下限 (y_err大→vx优先时)
VY_HI  = 0.70              # 横向占比上限 (x_err大→vy优先时)
#   分配策略: 根据 x_err/(x_err+y_err) 在 [VY_LO, VY_HI] 间线性插值

# ── 积分限幅 ──
IX_OUT = 0.20                   # X通道积分输出上限 (m/s)
IY_OUT = 0.20                   # Y通道积分输出上限 (m/s)
I_BUF  = 50                     # PID内部积分缓冲

# ── 对齐 & 到达 ──
ALGN_X = 1.0               # 横向对齐阈值 (cm)
ALGN_Y = 2.0               # 纵向对齐阈值 (cm)
ARRIV  = 3.0               # 到达判定 (cm) — 适配 DIST=8

# ── 丢失 & 周期 ──
LOST_T = 500               # 丢失超时 (ms)
DT     = 0.01              # 默认控制周期 (s)

# ═══════════════════════════════════════════════════════════════
#  状态枚举
# ═══════════════════════════════════════════════════════════════

S_FOLLOW = 0
S_STOP   = 1
S_LOST   = 2


_pid_x = PID(PX, IX, DX, IX_OUT / IX, VY_MAX)
_pid_y = PID(PY, IY, DY, IY_OUT / IY, VX_MAX)


# ═══════════════════════════════════════════════════════════════
#  模块状态
# ═══════════════════════════════════════════════════════════════

_st  = S_LOST
_tms = 0
_hot = True                 # 热启动: 首帧消D脉冲


# ═══════════════════════════════════════════════════════════════
#  工具函数
# ═══════════════════════════════════════════════════════════════

def _brake(val, err, dead, width):
    """smoothstep 刹车: [dead, dead+width] 内三次平滑衰减"""
    if err <= dead:
        return 0.0
    end = dead + width
    if err >= end:
        return val
    t = (err - dead) / width
    return val * t * t * (3.0 - 2.0 * t)   # smoothstep: 0→1 首尾导数=0


def _budget(vx, vy, x_err, y_err):
    """动态速度预算: 误差比决定 vy/vx 分配优先级"""
    total = abs(vx) + abs(vy)
    if total <= VBUD:
        return vx, vy

    # x_err权重↑ → vy多分; y_err权重↑ → vx多分
    w = abs(x_err) / (abs(x_err) + abs(y_err) + 1e-6)
    vy_frac = VY_LO + (VY_HI - VY_LO) * w

    vy_b = min(vy_frac * VBUD, abs(vy))
    vy_c = vy_b if vy >= 0 else -vy_b

    rem  = VBUD - vy_b
    vx_c = (1 if vx >= 0 else -1) * min(abs(vx), rem)
    return vx_c, vy_c



def reset_control(reset_state=False):
    """重置控制状态 (状态切换时调用)"""
    global _st, _tms, _hot
    if reset_state:
        _st  = S_LOST
        _tms = time.ticks_ms()
    _pid_x.reset()
    _pid_y.reset()
    _hot = True


def compute_control(x_cm, dist_cm, has_tgt, now_ms=None, dt=None):
    """单帧跟随控制 — 由主循环每 10ms 调用

    X/Y 期望由模块常量 E_X / DIST 定义, 默认 E_X=0 (居中跟踪)

    参数:
        x_cm:    横向偏移 (cm) 正=右, 相对于摄像头中心
        dist_cm: 纵向距离 (cm)
        has_tgt: 是否有目标
        now_ms:  时间戳 (默认自动取)
        dt:      实际控制周期 (s) 默认 DT

    返回 dict:
        cmd_fwd:  前后速度 vx (m/s)  None=未输出
        cmd_lat:  横向速度 vy (m/s)
        state:    当前子状态 S_*
        arrived:  是否已到达
        state_msg:状态切换日志  None=无变化
    """
    global _st, _tms, _hot

    if now_ms is None:
        now_ms = time.ticks_ms()
    if dt is None:
        dt = DT

    out = {
        'cmd_fwd': None, 'cmd_lat': None,
        'cmd_fwd_raw': 0.0, 'cmd_lat_raw': 0.0,
        'state': _st, 'arrived': False,
        'state_msg': None, 'just_switched': False,
    }

    # ── 状态判定 ──
    if has_tgt:
        _tms = now_ms
        if _st == S_LOST:
            _st = S_FOLLOW
            reset_wheel_pi()
            _pid_x.reset()
            _pid_y.reset()
            _hot = True
            out['state_msg']     = "[LOST → FOLLOW]"
            out['just_switched'] = True
    elif _st == S_FOLLOW:
        if time.ticks_diff(now_ms, _tms) > LOST_T:
            _st = S_LOST
            stop_all()
            _pid_x.reset()
            _pid_y.reset()
            out['state_msg'] = "[FOLLOW → LOST]"
            out['state']     = _st
            return out
        # 短暂丢失（<LOST_T） → 用缓存坐标判断是否已到达目标
        x_err = x_cm - E_X
        y_err = dist_cm - DIST
        if abs(y_err) < ARRIV and abs(x_err) < ALGN_X:
            _st = S_STOP
            stop_all()
            _pid_x.reset()
            _pid_y.reset()
            out['arrived']   = True
            out['state_msg'] = "[FOLLOW → STOPPED] d={:.1f}".format(dist_cm)
            out['state']     = _st
            return out

    out['state'] = _st

    # ── FOLLOW: 级联控制 ──
    if _st == S_FOLLOW:
        x_err = x_cm - E_X       # 补充横向误差定义
        y_err = dist_cm - DIST   # 补充纵向误差定义
 
        # ── 横向控制: x_err → vy ──
        if abs(x_err) < DX0:
            _pid_x._prev_error = x_err
            _pid_x._d_filtered = 0.0
            vy = 0.0
        else:
            vy = _pid_x.compute(x_err, 0.0, dt)
            vy = _brake(vy, abs(x_err), DX0, BX)
        
        # ── 纵向控制: y_err → vx ──
        if abs(y_err) < DY0:
            _pid_y._prev_error = -y_err
            _pid_y._d_filtered = 0.0
            vx = 0.0
        else:
            vx = -_pid_y.compute(DIST, dist_cm, dt)  # PID(setpoint=DIST, meas=dist_cm)→负反馈取反
            vx = _brake(vx, abs(y_err), DY0, BY)
 
        out['cmd_fwd_raw'] = vx
        out['cmd_lat_raw'] = vy
 
        # 动态速度分配
        vx, vy = _budget(vx, vy, x_err, y_err)
        out['cmd_fwd'] = vx
        out['cmd_lat'] = vy

        # ── 到达判定 ──
        if abs(y_err) < ARRIV and abs(x_err) < ALGN_X:
            _st = S_STOP
            stop_all()
            _pid_x.reset()
            _pid_y.reset()
            out['arrived']   = True
            out['state_msg'] = "[FOLLOW → STOPPED] d={:.1f}".format(dist_cm)
            out['state']     = _st
            return out


    # ── STOPPED: 保持静止, 距离或横向偏移恢复后自动重跟 ──
    elif _st == S_STOP:
        if abs(dist_cm - DIST) > ARRIV * 2 or abs(x_cm - E_X) > ALGN_X * 2:
            _st = S_FOLLOW
            reset_wheel_pi()
            _pid_x.reset()
            _pid_y.reset()
            _hot = True
            out['state_msg'] = "[STOPPED → FOLLOW]"
            out['state']     = _st
        else:
            stop_all()

    # ── LOST: 停车等待 ──
    elif _st == S_LOST:
        stop_all()

    return out

