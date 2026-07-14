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

DIST, E_X = 5, 5
PX, IX, DX = 0.03, 0.0, 0.002
PY, IY, DY = 0.03, 0.00, 0.001
DEAD_X, DEAD_DIST = 0.3, 0.5
MAX_VX, MAX_VY, MAX_WZ = 1.5, 1.5, 1
IX_OUT, IY_OUT = 0.20, 0.20
SPEED_BUDGET, VY_BUDGET_MAX, VY_BUDGET_MIN = 1.0, 0.70, 0.30
ALIGN_EX, ALIGN_DIST = 0.5, 0.5
LOST_T, DT = 500, 0.01

S_FOLLOW, S_STOP, S_LOST = 0, 1, 2

def _apply_input_deadzone(error, dead):
    if abs(error) <= dead: return 0.0
    return error - dead if error > 0 else error + dead

class FollowController:
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
        if now_ms is None: now_ms = time.ticks_ms()
        if dt is None: dt = DT
        raw_ex_err   = -x_cm + E_X
        raw_dist_err = DIST - dist_cm
        if not self._initialized:
            self._prev_time = now_ms; self._initialized = True
        dt_act = max(time.ticks_diff(now_ms, self._prev_time) / 1000.0, 0.001)
        dt_act = min(dt_act, 0.050)
        self._prev_time = now_ms
        if has_tgt:
            self._lost_time = now_ms
            if self._state != S_FOLLOW:
                self._state = S_FOLLOW
                self._first_valid_frame = True
        elif self._state == S_FOLLOW:
            if time.ticks_diff(now_ms, self._lost_time) > LOST_T:
                self._state = S_LOST
                self._first_valid_frame = True
        if self._state != S_FOLLOW:
            self.pid_ex.compute(0.0, 0.0, dt_act)
            self.pid_dist.compute(0.0, 0.0, dt_act)
            return 0.0, 0.0, 0.0, False, dt_act
        if self._first_valid_frame:
            self.pid_ex._prev_error = _apply_input_deadzone(raw_ex_err, DEAD_X)
            self.pid_dist._prev_error = _apply_input_deadzone(raw_dist_err, DEAD_DIST)
            self.pid_ex._d_filtered = 0.0
            self.pid_dist._d_filtered = 0.0
            self._first_valid_frame = False
        ex_err_dz   = _apply_input_deadzone(raw_ex_err, DEAD_X)
        dist_err_dz = _apply_input_deadzone(raw_dist_err, DEAD_DIST)
        vy = -self.pid_ex.compute(ex_err_dz, 0.0, dt_act)
        vx = -self.pid_dist.compute(dist_err_dz, 0.0, dt_act)
        vy += vy_extra
        vx, vy, wz_out = self._enforce_speed_budget(vx, vy, wz_in, raw_dist_err)
        is_aligned = (abs(raw_ex_err) < ALIGN_EX and abs(raw_dist_err) < ALIGN_DIST)
        return vx, vy, wz_out, is_aligned, dt_act

    def _enforce_speed_budget(self, vx, vy, wz, dist_error):
        total = abs(vx) + abs(vy) + abs(wz)
        if total <= SPEED_BUDGET: return vx, vy, wz
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

class CameraController:
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
        if ctrl['arrived']:
            stop_fn(); break
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
