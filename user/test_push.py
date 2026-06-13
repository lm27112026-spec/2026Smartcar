"""
test_push.py — 物资推送测试程序
====================================
【模拟流程】（替代摄像头识别 → UWB 距离触发）
  1. UWB 跟随目标锚点（UART0, 115200）
  2. 当 UWB 滤波距离 ≤ 20cm → 模拟"物资识别成功"
  3. 主车停止 → 蓝牙发送 POS_ADJ（右前 45°）给从车
  4. 等待从车 POS_OK → 两车同步推动物资

【运行方式】
  直接在主车上运行此文件：
    import test_push
    test_push.main()

  或替换 main.py 内容：
    from test_push import main
    main()
"""

import time, math
from uwb_tracker import UWBTracker
from uart_master import MasterBT
from motor import stop_all, omni_move_by_angle


# ── 参数 ──────────────────────────────────────────
TRIGGER_DIST_M   = 0.20       # UWB 距离阈值（米）
PUSH_SPEED       = 0.30       # 推动速度
PUSH_ANGLE_DEG   = 45         # 推动方向（右前）
PUSH_DURATION_S  = 5.0        # 持续推动时间（秒）
LOOP_DT_MS       = 10         # 主循环周期
SKIP_SLAVE_ACK   = True       # True=发完不等回复（测试用） False=等从车确认
# ──────────────────────────────────────────────────


def main():
    """主函数 — 永不返回。"""

    # 1. 初始化模块
    uwb = UWBTracker(uart_id=0, baudrate=115200, target_anchor="8834",
                      stop_dist_m=TRIGGER_DIST_M)
    bt  = MasterBT(uart_id=5, baudrate=9600)

    stop_all()
    print("=" * 50)
    print("Test Push Program")
    print("  Trigger: UWB distance <= {:.2f}m".format(TRIGGER_DIST_M))
    print("  Push:    {:.1f} m/s @ {:d} deg (right-front)".format(PUSH_SPEED, PUSH_ANGLE_DEG))
    print("=" * 50)

    triggered = False

    while True:
        # ── 超时保护（只停车，不跳过 UART 读取，否则会永久死锁） ──
        if uwb.is_timeout():
            stop_all()
            time.sleep_ms(LOOP_DT_MS)
            # 不 continue — 继续调用 get_command() 读 UART，新数据来了会清除超时

        # ── UWB 跟随 ──
        cmd = uwb.get_command()
        if cmd:
            spd, ang = cmd
            omni_move_by_angle(spd, ang)

        # ── 距离触发检测 ──
        if not triggered and uwb.is_near_target():
            triggered = True
            print("\n" + "=" * 50)
            print("[TEST] UWB distance <= {:.1f}m → Simulating cargo detect!".format(TRIGGER_DIST_M))
            print("[TEST] Distance: {:.2f}m  Angle: {:+.0f} deg".format(
                uwb.get_distance(), uwb.get_angle()))

            # 停车
            stop_all()
            time.sleep_ms(200)

            # 计算右前 45° 速度分量
            rad = math.radians(PUSH_ANGLE_DEG)
            vx = PUSH_SPEED * math.sin(rad)
            vy = PUSH_SPEED * math.cos(rad)
            print("[TEST] Push vector: vx={:.3f} vy={:.3f} (angle={:d} deg)".format(
                vx, vy, PUSH_ANGLE_DEG))

            # 发送 POS_ADJ 给从车
            print("[TEST] Sending POS_ADJ to slave...")
            if SKIP_SLAVE_ACK:
                bt.send_pos_adjust_async(vx, vy, 0.0)
                print("[TEST] POS_ADJ sent (skip ack mode)")
                time.sleep_ms(500)  # 给从车反应时间
            else:
                ok = bt.send_pos_adjust(vx, vy, 0.0)
                if not ok:
                    print("[TEST] ERROR: Slave no response after retries")
                    stop_all()
                    break
                print("[TEST] POS_OK received → starting sync push")

            # 同步推动阶段
            push_start_ms = time.ticks_ms()
            while time.ticks_diff(time.ticks_ms(), push_start_ms) < PUSH_DURATION_S * 1000:
                # 主车推动
                omni_move_by_angle(PUSH_SPEED, PUSH_ANGLE_DEG)

                # 从车同步（每个周期发一次）
                bt.send_sync_move(vx, vy, 0.0)

                time.sleep_ms(LOOP_DT_MS)

            # 推动结束
            stop_all()
            bt.send_emergency_stop()
            print("=" * 50)
            print("[TEST] Push complete. Stopping.")
            break

        time.sleep_ms(LOOP_DT_MS)

    # 清理
    uwb.stop()
    stop_all()
    print("Test program ended.")


# ── 直接运行入口 ──
main()
