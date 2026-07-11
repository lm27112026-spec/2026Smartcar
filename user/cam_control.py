# cam_control.py — 摄像头跟随控制模块 (参照 led CascadeController)
# 【层】控制层 — 摄像头数据接收封装 + 级联PID跟随控制 + 闭环靠近
# 【职责】
#   FollowController:  内部 PID 实例 + 输入死区 + 3轴速度预算
#   CameraController:  封装 CamDataReceiver + FollowController
#   cam_approach():    闭环靠近，PID对齐 / 防撞 / 盲区遮挡
# 【依赖】cam_data.py、motor.py
import gc, time
from pid import PID
from cam_data import CamDataReceiver, x_to_cm, y_to_distance
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_speeds_filtered, get_encoder_counts,
                   reset_encoder_filter, reset_wheel_pi)

# ═══════════════════════════════════════════════════════════════
#  全局参数
# ═══════════════════════════════════════════════════════════════

# ── 跟踪目标 ──
DIST  = 4            # 期望距离 (cm) — 纵向目标
E_X   = 5         # 期望横向偏移 (cm) — 正=偏左

# ── 外环 PID 增益 ──
PX  = 0.025           # 横向 P (x_err → vy)
IX  = 0.0           # 横向 I
DX  = 0.002           # 横向 D

PY  = 0.025           # 纵向 P (y_err → vx)
IY  = 0.00              # 纵向 I
DY  = 0.001           # 纵向 D

# ── 死区  ──
DEAD_X   = 0.3       # 横向死区 (cm) — |x_err|<DEAD_X → 输出 0
DEAD_DIST = 0.5      # 纵向死区 (cm) — |y_err|<DEAD_DIST → 输出 0

# ── 输出限幅 ──
MAX_VX   = 1.2       # 前后最大速度 (m/s)
MAX_VY   = 1.2       # 横向最大速度 (m/s)
MAX_WZ   = 1         # 旋转最大速度 (归一化)

# ── 积分限幅 ──
IX_OUT = 0.20        # X通道积分输出上限 (m/s)
IY_OUT = 0.20        # Y通道积分输出上限 (m/s)

# ── 动态速度预算 (3轴: vx+vy+wz) ──
SPEED_BUDGET   = 0.90  # 总速度预算 (归一化)
VY_BUDGET_MAX  = 0.70  # 横向占比上限 — x_err大→vy优先
VY_BUDGET_MIN  = 0.30  # 横向占比下限 — y_err大→vx优先

# ── 对齐 & 到达判定 ──
ALIGN_EX   = 0.5        # 横向对齐阈值 (cm)
ALIGN_DIST = 0.5        # 纵向对齐阈值 (cm)

# ── 丢失超时 & 控制周期 ──
LOST_T = 500            # 目标丢失超时 (ms)
DT     = 0.01           # 默认控制周期 (s)

# ═══════════════════════════════════════════════════════════════
#  状态枚举
# ═══════════════════════════════════════════════════════════════

S_FOLLOW = 0
S_STOP   = 1
S_LOST   = 2


def _apply_input_deadzone(error, dead):
    """输入级死区: 从误差中直接减去死区量"""
    if abs(error) <= dead:
        return 0.0
    if error > 0:
        return error - dead
    else:
        return error + dead


# ═══════════════════════════════════════════════════════════════
#  FollowController — 级联 PID 跟随控制器
# ═══════════════════════════════════════════════════════════════

class FollowController:
    """级联 PID 跟随控制器 — 输入死区 + 3轴速度预算"""

    def __init__(self):
        self.pid_ex = PID(kp=PX, ki=IX, kd=DX,
                          integral_limit=IX_OUT / IX if IX > 0 else 0,
                          output_limit=MAX_VY)
        self.pid_dist = PID(kp=PY, ki=IY, kd=DY,
                            integral_limit=IY_OUT / IY if IY > 0 else 0,
                            output_limit=MAX_VX)
        self._initialized = False
        self._first_valid_frame = True
        self._prev_time = 0
        self._lost_time = 0
        self._state = S_LOST

    def emergency_stop(self):
        self.pid_ex.reset(); self.pid_dist.reset()
        reset_wheel_pi(); reset_encoder_filter()
        self._initialized = False; self._first_valid_frame = True

    def reset_vision(self):
        self.pid_ex.reset(); self.pid_dist.reset()
        self._first_valid_frame = True

    def step(self, x_cm, dist_cm, has_tgt, wz_in=0.0, dt=None, now_ms=None, vy_extra=0.0):
        """单帧跟随控制。返回 (vx, vy, wz_out, is_aligned)

        对齐 led CascadeController 关键策略:
        - PID 不随状态切换重置（保持积分连续）
        - 非跟踪态注入零误差计算（保持 PID d_filter 存续）
        """
        if now_ms is None: now_ms = time.ticks_ms()
        if dt is None: dt = DT

        raw_ex_err   = -x_cm + E_X
        raw_dist_err = DIST - dist_cm

        # ── dt 计算（对齐 led: 无论跟踪状态都算 dt） ──
        if not self._initialized:
            self._prev_time = now_ms; self._initialized = True
        dt_act = max(time.ticks_diff(now_ms, self._prev_time) / 1000.0, 0.001)
        dt_act = min(dt_act, 0.050)
        self._prev_time = now_ms

        # ── 状态机 ──
        if has_tgt:
            self._lost_time = now_ms
            if self._state != S_FOLLOW:
                self._state = S_FOLLOW
                self._first_valid_frame = True  # 仅防 D 脉冲，不重置积分（对齐 led）
        elif self._state == S_FOLLOW:
            if time.ticks_diff(now_ms, self._lost_time) > LOST_T:
                self._state = S_LOST
                self._first_valid_frame = True

        # ── 非跟踪态：零误差计算保持 PID 存续（对齐 led） ──
        if self._state != S_FOLLOW:
            self.pid_ex.compute(0.0, 0.0, dt_act)
            self.pid_dist.compute(0.0, 0.0, dt_act)
            return 0.0, 0.0, 0.0, False, dt_act

        # ── 首帧有效: 初始化 prev_error 避免 D 脉冲 ──
        if self._first_valid_frame:
            self.pid_ex._prev_error = _apply_input_deadzone(raw_ex_err, DEAD_X)
            self.pid_dist._prev_error = _apply_input_deadzone(raw_dist_err, DEAD_DIST)
            self.pid_ex._d_filtered = 0.0
            self.pid_dist._d_filtered = 0.0
            self._first_valid_frame = False

        # 输入死区 → PID
        ex_err_dz   = _apply_input_deadzone(raw_ex_err, DEAD_X)
        dist_err_dz = _apply_input_deadzone(raw_dist_err, DEAD_DIST)
        vy = -self.pid_ex.compute(ex_err_dz, 0.0, dt_act)
        # vx 符号: 实测本车 vx > 0 → 前进 (校准测试确认)
        # raw_dist_err = DIST - dist_cm
        #   目标远(实际>DIST) → dist_err_dz < 0 → PID输出负 → 取反得正vx → 前进 ✓
        vx = -self.pid_dist.compute(dist_err_dz, 0.0, dt_act)

        vy += vy_extra

        # 3轴速度预算
        vx, vy, wz_out = self._enforce_speed_budget(vx, vy, wz_in, raw_dist_err)

        # 对齐判定
        is_aligned = (abs(raw_ex_err) < ALIGN_EX and abs(raw_dist_err) < ALIGN_DIST)
        return vx, vy, wz_out, is_aligned, dt_act

    def _enforce_speed_budget(self, vx, vy, wz, dist_error):
        total = abs(vx) + abs(vy) + abs(wz)
        if total <= SPEED_BUDGET:
            return vx, vy, wz
        norm = min(abs(dist_error) / 80.0, 1.0)
        dyn_vy_frac = VY_BUDGET_MAX - (VY_BUDGET_MAX - VY_BUDGET_MIN) * norm
        vy_b = min(dyn_vy_frac * SPEED_BUDGET, abs(vy))
        vy_c = (1.0 if vy >= 0 else -1.0) * vy_b
        rem = SPEED_BUDGET - vy_b
        vx_a, wz_a = abs(vx), abs(wz)
        s = vx_a + wz_a + 1e-6
        vx_c = (1.0 if vx >= 0 else -1.0) * (vx_a / s) * rem
        wz_c = (1.0 if wz >= 0 else -1.0) * (wz_a / s) * rem
        return vx_c, vy_c, wz_c


# ═══════════════════════════════════════════════════════════════
#  CameraController — 供 main.py 调用的摄像头跟随封装
# ═══════════════════════════════════════════════════════════════

class CameraController:
    """摄像头目标跟随控制器。不含航向保持，wz 由外部提供。"""

    def __init__(self, uart_id=7):
        self._recv = CamDataReceiver(uart_id)
        self._ctrl = FollowController()
        self._last_x_cm = 0.0; self._last_dist_cm = 0.0
        self._last_has_tgt = False
        self._t_prev = 0; self._first = True

    @property
    def frame_count(self): return self._recv.frame_count
    @property
    def target_count(self): return self._recv.target_count
    @property
    def error_count(self): return self._recv.error_count

    def flush(self):
        for _ in range(20):
            if self._recv.read() is None: break

    def reset(self):
        self._ctrl.reset_vision()
        self._last_x_cm = 0.0; self._last_dist_cm = 0.0
        self._last_has_tgt = False; self._first = True
        self.flush()

    def step(self, now_ms=None):
        """一帧控制计算。返回 dict {vx, vy, has_target, x_cm, dist_cm, obj_id, line_flag, arrived, state, state_msg}"""
        if now_ms is None: now_ms = time.ticks_ms()
        if self._first:
            self._t_prev = now_ms; self._first = False; dt_act = 0.01
        else:
            dt_act = time.ticks_diff(now_ms, self._t_prev) * 0.001
            self._t_prev = now_ms
            if dt_act <= 0 or dt_act > 0.1: dt_act = 0.01

        cam_data = self._recv.read()
        has_tgt = False; x_cm = 0.0; dist_cm = 0.0; obj_id = 0; line_flag = 0
        if cam_data is not None:
            has_tgt = cam_data['is_target']; obj_id = cam_data['id']
            line_flag = cam_data.get('line_flag', 0)
            if has_tgt:
                x_cm = x_to_cm(cam_data['x']); dist_cm = y_to_distance(cam_data['y'])
                self._last_x_cm = x_cm; self._last_dist_cm = dist_cm
                self._last_has_tgt = True
            elif self._last_has_tgt:
                x_cm = self._last_x_cm; dist_cm = self._last_dist_cm
        elif self._last_has_tgt:
            x_cm = self._last_x_cm; dist_cm = self._last_dist_cm

        vx, vy, wz, is_aligned, _dt = self._ctrl.step(x_cm, dist_cm, has_tgt, wz_in=0.0, dt=dt_act, now_ms=now_ms)
        arrived = self._ctrl._state == S_STOP or is_aligned
        return {
            'vx': vx if abs(vx) > 0.001 or abs(vy) > 0.001 else None,
            'vy': vy if abs(vx) > 0.001 or abs(vy) > 0.001 else None,
            'has_target': has_tgt, 'x_cm': x_cm, 'dist_cm': dist_cm,
            'obj_id': obj_id, 'line_flag': line_flag,
            'arrived': arrived, 'state': self._ctrl._state, 'state_msg': None,
        }


# ═══════════════════════════════════════════════════════════════
#  cam_approach() — 闭环靠近目标直到到达判定
# ═══════════════════════════════════════════════════════════════

def cam_approach(cam, lock_heading_fn, calc_wz_fn,
                 should_abort_fn, drive_fn, stop_fn, led_fn=None):
    """闭环靠近。返回 (arrived: bool, reason: str)。"""
    cam.reset()
    target = lock_heading_fn()
    if led_fn: led_fn(True)

    t0 = time.ticks_ms()
    found = False
    last_d = 999.0

    while True:
        if should_abort_fn():
            if led_fn: led_fn(False)
            return (False, 'aborted')
        if time.ticks_diff(time.ticks_ms(), t0) / 1000.0 > 15.0:
            return (False, 'timeout')

        ctrl = cam.step()
        if ctrl['has_target']:
            found = True
            if 0 < ctrl['dist_cm']: last_d = ctrl['dist_cm']

        # ── 纯跟随模式：仅 PID 对齐停车 ──
        # 防撞兜底(≤6cm) 已暂时关闭 — 恢复时改为：
        #   if ctrl['arrived'] or (ctrl['has_target'] and 0 < ctrl['dist_cm'] <= 6.0):
        if ctrl['arrived']:
            stop_fn(); break

        # ── 盲区遮挡(≤16cm) 与 lost_far 判定 已暂时关闭 — 纯跟随模式 ──
        # 原逻辑：found 且 S_LOST 时，last_d≤16 → 盲区到达；last_d>16 → lost_far 失败
        # 现仅依赖 PID 对齐 / 15s 超时 / SW2 中断 退出
        # 恢复时取消下方注释即可：
        # if found and ctrl['state'] == S_LOST:
        #     if last_d <= 16.0:
        #         stop_fn(); break
        #     else:
        #         stop_fn(); cam.reset()
        #         return (False, 'lost_far')

        if ctrl['vx'] is not None and ctrl['vy'] is not None:
            wz = calc_wz_fn(target)
            try: drive_fn(ctrl['vx'], ctrl['vy'], wz, DT)
            except Exception: pass
        else:
            wz = calc_wz_fn(target)
            if abs(wz) > 0.001:
                try: drive_fn(0, 0, wz, DT)
                except Exception: pass
            else: stop_fn()

        time.sleep_ms(int(DT * 1000))

    if led_fn: led_fn(False)
    return (True, 'arrived')
