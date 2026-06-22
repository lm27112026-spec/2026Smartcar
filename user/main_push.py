"""
main_push.py — 摄像头视觉追踪 + 阈值触发推送
================================================
【两阶段流程】
  Phase 1 — APPROACH（视觉追踪靠近）
    PID 闭环追踪，将目标拉近到阈值范围内
    过程中每帧检查阈值条件
  Phase 2 — PUSH（前推）
    阈值命中 → 立即切换为前向推动

【触发条件】
  X 在 2.0 ~ 4.0 (mm)  且  Y 在 3.0 ~ 7.0 (cm)
  → 进入 PUSH 模式，全向轮闭环前推

【数据来源】cam_data.py（UART7, 115200）
  data['x']: 横向偏移 (mm)，x>0=右  x<0=左
  data['y']: 纵向距离 (cm)
  data['is_target']: 是否检测到目标

【硬件】SW2(D9) 随时退出
【依赖】cam_data.py, motor.py, pid.py
【使用】
  import main_push
  main_push.main()
"""

import time, math
from machine import Pin
from cam_data import CamDataReceiver, x_to_cm
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts, reset_encoder_filter, reset_wheel_pi,
                   enc_ticker, ENC_SCALE)
from pid import PID

# ═══════════════════════════════════════════════════════════════
#  阈值参数（push 触发条件）
# ═══════════════════════════════════════════════════════════════

X_MIN = 2.0          # X 下限 (mm) — data['x'] 原始值
X_MAX = 4.0          # X 上限 (mm)
Y_MIN = 3.0          # Y 下限 (cm) — data['y'] 原始值
Y_MAX = 7.0          # Y 上限 (cm)

# ═══════════════════════════════════════════════════════════════
#  Approach 参数（PID 追踪靠近）
# ═══════════════════════════════════════════════════════════════

APPROACH_TARGET_Y = (Y_MIN + Y_MAX) / 2    # 5.0 cm — PID 追踪目标（阈值区间中点）

# 前向 PID: 将距离拉近到 APPROACH_TARGET_Y
APPROACH_FWD_KP = 0.04
APPROACH_FWD_KI = 0.0005
APPROACH_FWD_KD = 0.0

# 横向 PID: 将 X 偏移归零（居中目标）
APPROACH_LAT_KP = 0.02
APPROACH_LAT_KI = 0.0003
APPROACH_LAT_KD = 0.0

MAX_APPROACH_FWD = 0.40   # 前向最大速度 (m/s)
MAX_APPROACH_LAT = 0.30   # 横向最大速度 (m/s)
APPROACH_DT     = 0.02    # 控制周期 (s)

# ═══════════════════════════════════════════════════════════════
#  Push 参数（前推）
# ═══════════════════════════════════════════════════════════════

PUSH_SPEED      = 0.30    # 推动速度 (m/s)
PUSH_DURATION_S = 3.0     # 推动持续时间 (s)

# ═══════════════════════════════════════════════════════════════
#  硬件
# ═══════════════════════════════════════════════════════════════

SW2_PIN = 'D9'


# ═══════════════════════════════════════════════════════════════
#  阈值判断函数
# ═══════════════════════════════════════════════════════════════

def check_push_threshold(x_mm, y_cm):
    """
    判断摄像头数据是否满足 push 触发条件

    参数:
        x_mm: 摄像头 X 坐标 (mm) — data['x'] 原始值
        y_cm: 摄像头 Y 坐标 (cm) — data['y'] 原始值

    返回:
        bool: True=满足阈值，应进入 push 模式
    """
    return (X_MIN <= x_mm <= X_MAX) and (Y_MIN <= y_cm <= Y_MAX)


# ═══════════════════════════════════════════════════════════════
#  主函数
# ═══════════════════════════════════════════════════════════════

def main():
    # ── 硬件初始化 ──
    sw2 = Pin(SW2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
    sw2_initial = sw2.value()
    recv = CamDataReceiver(uart_id=7)
    stop_all()

    # ── 编码器初始化 ──
    enc_ticker.stop()
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)
    reset_encoder_filter()
    reset_wheel_pi()
    enc_ticker.start(10)

    # ── PID 初始化 ──
    pid_fwd = PID(kp=APPROACH_FWD_KP, ki=APPROACH_FWD_KI, kd=APPROACH_FWD_KD,
                  integral_limit=0.3, output_limit=MAX_APPROACH_FWD)
    pid_lat = PID(kp=APPROACH_LAT_KP, ki=APPROACH_LAT_KI, kd=APPROACH_LAT_KD,
                  integral_limit=0.2, output_limit=MAX_APPROACH_LAT)

    # ── 信息输出 ──
    print("=" * 50)
    print("Push Mode — Camera Approach + Threshold Trigger")
    print("  UART7 : Camera (115200)")
    print("  Phase 1: APPROACH — PID visual tracking")
    print("    Target Y : {:.1f} cm".format(APPROACH_TARGET_Y))
    print("  Phase 2: PUSH — forward drive")
    print("    Push speed: {:.2f} m/s".format(PUSH_SPEED))
    print("    Push dur. : {:.1f} s".format(PUSH_DURATION_S))
    print("  Trigger: X={:.1f}~{:.1f} mm  Y={:.1f}~{:.1f} cm".format(
        X_MIN, X_MAX, Y_MIN, Y_MAX))
    print("  SW2 toggle: exit")
    print("=" * 50)

    push_triggered = False
    last_print_ms  = time.ticks_ms()
    last_time_ms   = time.ticks_ms()
    last_diag_ms   = time.ticks_ms()   # 诊断心跳
    loop_count     = 0
    no_data_count  = 0                 # 连续无数据帧计数
    no_target_count = 0                # 连续无目标帧计数

    try:
        # ═══════════════════════════════════════════════════════
        #  Phase 1 — APPROACH（视觉追踪靠近，持续阈值判定）
        # ═══════════════════════════════════════════════════════
        print("[APPROACH] Waiting for target...")

        while True:
            loop_count += 1

            # ── SW2 退出检测 ──
            if sw2.value() != sw2_initial:
                print("\n[EXIT] SW2 toggled.")
                break

            # ── 读取摄像头 ──
            data = recv.read()
            if data is None:
                no_data_count += 1
                time.sleep_ms(5)
                # ── 每 2 秒输出诊断 ──
                if time.ticks_diff(time.ticks_ms(), last_diag_ms) >= 2000:
                    print("[DIAG] No camera data ({} loops, {} consecutive None)".format(
                        loop_count, no_data_count))
                    print("[DIAG]   UART7 status: any={}  frames={}  errors={}".format(
                        hasattr(recv, '_uart') and recv._uart.any(),
                        recv.frame_count, recv.error_count))
                    last_diag_ms = time.ticks_ms()
                continue

            no_data_count = 0  # 有数据，清零

            if not data['is_target']:
                no_target_count += 1
                # 目标丢失 → 停车 + 重置 PID
                stop_all()
                pid_fwd.reset()
                pid_lat.reset()
                # ── 每 2 秒输出一次丢失信息 ──
                if time.ticks_diff(time.ticks_ms(), last_diag_ms) >= 2000:
                    print("[DIAG] Cam active but no target ({} consecutive lost)".format(
                        no_target_count))
                    print("[DIAG]   X={:+.1f}mm Y={:.1f}cm flag={:02X} frames={}".format(
                        data['x'], data['y'], data['flag'], recv.frame_count))
                    last_diag_ms = time.ticks_ms()
                time.sleep_ms(5)
                continue

            no_target_count = 0  # 有目标，清零

            now_ms = time.ticks_ms()

            # ── 坐标转换 ──
            x_cm = x_to_cm(data['x'])   # X → cm
            y_cm = data['y']             # Y 已是 cm

            # ── 阈值判断（每帧检查）──
            if check_push_threshold(data['x'], data['y']):
                push_triggered = True
                print("\n" + "=" * 50)
                print("[APPROACH] Threshold met! Switching to PUSH...")
                print("  X = {:+.1f} mm (need {:.1f}~{:.1f})".format(
                    data['x'], X_MIN, X_MAX))
                print("  Y = {:.1f} cm (need {:.1f}~{:.1f})".format(
                    data['y'], Y_MIN, Y_MAX))
                print("=" * 50)
                stop_all()
                break

            # ── PID 计算 ──
            dt = time.ticks_diff(now_ms, last_time_ms) * 0.001
            last_time_ms = now_ms
            if dt > 0.1 or dt <= 0:
                dt = APPROACH_DT

            # 前向速度：将距离拉近到 APPROACH_TARGET_Y
            # y_error = y_cm - APPROACH_TARGET_Y  (>0 太远需前进)
            # PID error = setpoint - measurement; measurement=-y_error → error=y_error
            y_error = y_cm - APPROACH_TARGET_Y
            vy = pid_fwd.compute(setpoint=0, measurement=-y_error, dt=dt)
            # 只前进不停退（防后退 + 防积分饱和）
            if vy < 0:
                vy = 0.0
                pid_fwd.reset()

            # 横向速度：将 X 偏移归零
            # x_cm > 0 (目标偏右) → 需右移 (vx > 0)
            # PID error = setpoint - measurement; measurement=-x_cm → error=x_cm
            vx = pid_lat.compute(setpoint=0, measurement=-x_cm, dt=dt)

            # ── NaN 防护 ──
            if math.isnan(vx) or math.isnan(vy):
                vx = vy = 0.0
                pid_fwd.reset()
                pid_lat.reset()

            # ── 闭环驱动（用实测 dt 计算编码器速度，避免 DT 与实际周期失配）──
            try:
                rc = get_encoder_counts()
                rs = [rc[i] / ENC_SCALE[i] / dt for i in range(4)]
                omni_drive_closed_loop(vy, vx, 0, rs, dt)
            except Exception as e:
                print("[MOTOR] drive error:", e)

            # ── 定期打印 ──
            if time.ticks_diff(now_ms, last_print_ms) >= 300:
                print("[APPROACH] X:{:+5.1f}cm  Y:{:5.1f}cm  vx:{:+.3f}  vy:{:+.3f}".format(
                    x_cm, y_cm, vx, vy))
                last_print_ms = now_ms

            time.sleep_ms(10)

        # ═══════════════════════════════════════════════════════
        #  Phase 2 — PUSH（前向推动）
        # ═══════════════════════════════════════════════════════
        if push_triggered:
            print("[PUSH] Starting forward push...")
            push_start_ms = time.ticks_ms()

            # 清零 PID 积分（从追踪切换到推动）
            reset_wheel_pi()

            push_last_ms = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), push_start_ms) < PUSH_DURATION_S * 1000:
                # SW2 随时退出
                if sw2.value() != sw2_initial:
                    print("\n[EXIT] SW2 toggled during push.")
                    stop_all()
                    break

                now_push = time.ticks_ms()
                push_dt = time.ticks_diff(now_push, push_last_ms) * 0.001
                push_last_ms = now_push
                if push_dt > 0.1 or push_dt <= 0:
                    push_dt = APPROACH_DT

                try:
                    rc = get_encoder_counts()
                    rs = [rc[i] / ENC_SCALE[i] / push_dt for i in range(4)]
                    omni_drive_closed_loop(PUSH_SPEED, 0, 0, rs, push_dt)
                except Exception as e:
                    print("[MOTOR] push error:", e)

                time.sleep_ms(10)

            stop_all()
            print("[PUSH] Complete. Stopped.")

    except Exception as e:
        print("[ERROR] Exception in main loop:")
        import sys
        sys.print_exception(e)

    finally:
        stop_all()
        try:
            enc_ticker.stop()
        except Exception:
            pass
        print("\nProgram ended.")


# ── 直接运行入口 ──
main()
