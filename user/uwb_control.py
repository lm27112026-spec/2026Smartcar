"""
uwb_control.py — UWB 导航控制模块
提取自 main.py 的 _action_goto_supplies_startup() 和 _action_return_to_origin()
合并为通用 goto_location(x, y) 函数。

【功能】
  - 基于 UWB 坐标的 PID 闭环导航到目标点
  - 航向保持 + 机体坐标系转换
  - 到达死区判定 + 减速曲线 + 速度限幅
  - UWB 离线检测与自动恢复

【使用】
  from uwb_control import goto_location

  uwb = UWBPosition(...)
  arrived, reason = goto_location(
      uwb, target_x, target_y,
      lock_heading_fn, calc_wz_fn, get_yaw_fn,
      should_abort_fn, drive_fn, stop_fn,
      led_fn=led_fn, label="GOTO"
  )

【依赖】math, time, gc（标准库） + UWBPosition 实例（由调用方注入）
"""

import gc, time, math


# ═══════════════════════════════════════════════════════════════
#  导航参数（独立管理，调试时直接修改此处）
# ═══════════════════════════════════════════════════════════════

GOTO_KP              = 0.012   # 位置误差 → 速度 P 增益
GOTO_DB              = 8.0     # 到位死区 (cm)
GOTO_SLOW_DIST       = 20.0    # 减速起始距离 (cm)
GOTO_MAX_SPEED       = 0.50    # 最大速度 (m/s)
GOTO_MIN_SPEED       = 0.06    # 最小速度 (m/s)，克服静摩擦
GOTO_TIMEOUT_S       = 20.0    # 总超时 (s)
GOTO_CTRL_DT         = 0.01    # 控制周期 (s)
GOTO_ARRIVAL_FRAMES  = 5       # 连续 N 帧在死区内判定到达
GOTO_UWB_STEP_MS     = 50      # uwb.step() 调用间隔 (ms)
GOTO_UWB_DEAD_TIMEOUT_S = 5.0  # UWB 离线超时 (s)，超时后尝试重连
GOTO_UWB_MAX_RECONNECT  = 3     # UWB 最大重连次数
GOTO_UWB_RECONNECT_WAIT_MS = 500  # 重连后等待数据恢复 (ms)
GOTO_PRINT_INTERVAL_MS = 500   # 状态打印间隔 (ms)
GOTO_HEADING_DEADBAND = 2.0    # 航向死区 (度)


# ═══════════════════════════════════════════════════════════════
#  goto_location() — 通用 UWB 导航到目标坐标
# ═══════════════════════════════════════════════════════════════

def goto_location(uwb, target_x, target_y,
                  lock_heading_fn, calc_wz_fn, get_yaw_fn,
                  should_abort_fn, drive_fn, stop_fn,
                  led_fn=None, label="GOTO"):
    """UWB 闭环导航到目标坐标点。

    参数（回调函数，由调用方注入）:
        uwb:              UWBPosition 实例
        target_x:         float  目标 X 坐标 (cm)
        target_y:         float  目标 Y 坐标 (cm)
        lock_heading_fn:  () -> float              锁定并返回目标航向角 (度)
        calc_wz_fn:       (target_deg, deadband_deg) -> float  计算航向修正 wz
        get_yaw_fn:       () -> float              获取当前航向角 (度)
        should_abort_fn:  () -> bool               检查 SW2/看门狗中断
        drive_fn:         (vx, vy, wz, dt) -> None 驱动电机
        stop_fn:          () -> None               停止电机
        led_fn:           (bool) -> None           LED 控制，None 不控制
        label:            str                      打印前缀标签

    返回:
        (arrived: bool, reason: str)
        - (True,  'arrived')  → 已到达目标
        - (False, 'aborted')  → SW2 中断
        - (False, 'timeout')  → 总超时
        - (False, 'uwb_lost') → UWB 离线超时
    """
    print("\n  [{}] === 导航到目标 ({:.1f}, {:.1f}) ===".format(label, target_x, target_y))

    target_heading = lock_heading_fn()
    print("  [{}] 航向锁定: {:.1f}°".format(label, target_heading))

    if led_fn:
        led_fn(True)

    start_ms = time.ticks_ms()
    last_print_ms = start_ms
    last_uwb_ms = start_ms
    loop_cnt = 0
    near_target_count = 0
    uwb_dead_start = 0
    uwb_reconnect_count = 0

    while True:
        # ── 中断检测 ──
        if should_abort_fn():
            if led_fn:
                led_fn(False)
            return (False, 'aborted')

        # ── 总超时检测 ──
        elapsed_total = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
        if elapsed_total > GOTO_TIMEOUT_S:
            print("  [{}] 超时 ({:.1f}s)".format(label, elapsed_total))
            if led_fn:
                led_fn(False)
            return (False, 'timeout')

        # ── UWB 数据采集（按固定间隔） ──
        now_ms = time.ticks_ms()
        if time.ticks_diff(now_ms, last_uwb_ms) >= GOTO_UWB_STEP_MS:
            uwb.step()
            last_uwb_ms = now_ms

        # ── UWB 离线检测与自动重连 ──
        if uwb.is_timeout():
            if uwb_dead_start == 0:
                uwb_dead_start = time.ticks_ms()
                stop_fn()
                print("  [{}] UWB 掉线，等待恢复...".format(label))
            elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > GOTO_UWB_DEAD_TIMEOUT_S * 1000:
                if uwb_reconnect_count < GOTO_UWB_MAX_RECONNECT:
                    uwb_reconnect_count += 1
                    print("  [{}] UWB 离线超时 {:.0f}s，第 {}/{} 次尝试重连...".format(
                        label, GOTO_UWB_DEAD_TIMEOUT_S,
                        uwb_reconnect_count, GOTO_UWB_MAX_RECONNECT))
                    try:
                        uwb.reset_uart()
                        time.sleep_ms(GOTO_UWB_RECONNECT_WAIT_MS)
                        # 等待首帧数据
                        wait_start = time.ticks_ms()
                        while uwb.get_frame_count() == 0 or uwb.is_timeout():
                            uwb.step()
                            if time.ticks_diff(time.ticks_ms(), wait_start) > 3000:
                                break
                            time.sleep_ms(10)
                        if not uwb.is_timeout() and uwb.get_frame_count() > 0:
                            print("  [{}] UWB 重连成功，继续导航".format(label))
                            uwb_dead_start = 0
                            last_uwb_ms = time.ticks_ms()
                            continue
                    except Exception as e:
                        print("  [{}] UWB 重连异常:".format(label), e)
                    uwb_dead_start = time.ticks_ms()  # 重置计时，准备下次重连
                else:
                    print("  [{}] UWB 全部 {} 次重连均失败，放弃导航".format(
                        label, GOTO_UWB_MAX_RECONNECT))
                    if led_fn:
                        led_fn(False)
                    return (False, 'uwb_lost')
            # 离线期间停车等待恢复
            time.sleep_ms(int(GOTO_CTRL_DT * 1000))
            continue
        else:
            uwb_dead_start = 0
            uwb_reconnect_count = 0  # 恢复后重置重连计数

        # ── 获取当前位置与误差 ──
        curr_x, curr_y = uwb.get_position()
        error_x = target_x - curr_x
        error_y = target_y - curr_y
        dist = math.sqrt(error_x * error_x + error_y * error_y)

        # ── 到达判定（连续 N 帧在死区内） ──
        if dist < GOTO_DB:
            near_target_count += 1
            if near_target_count >= GOTO_ARRIVAL_FRAMES:
                print("  [{}] 已到达目标 ({:.1f}, {:.1f})  dist={:.1f}cm".format(
                    label, target_x, target_y, dist))
                stop_fn()
                if led_fn:
                    led_fn(False)
                return (True, 'arrived')
        else:
            near_target_count = 0

        # ── 状态打印 ──
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= GOTO_PRINT_INTERVAL_MS:
            last_print_ms = now
            print("  [{}] pos=({:.1f},{:.1f}) target=({:.1f},{:.1f}) dist={:.1f}cm yaw={:.2f}°".format(
                label, curr_x, curr_y, target_x, target_y, dist, get_yaw_fn()))

        # ── GC ──
        loop_cnt += 1
        if loop_cnt % 50 == 0:
            gc.collect()

        # ── 航向修正 ──
        wz = calc_wz_fn(target_heading, GOTO_HEADING_DEADBAND)

        # ── 机体坐标系变换：全局误差 → 前后/横向速度 ──
        yaw_rad = math.radians(get_yaw_fn())
        body_fwd  = -math.cos(yaw_rad) * error_x - math.sin(yaw_rad) * error_y
        body_right =  math.sin(yaw_rad) * error_x - math.cos(yaw_rad) * error_y

        vx_cmd = body_fwd * GOTO_KP
        vy_cmd = body_right * GOTO_KP

        # ── 减速曲线（接近目标时线性衰减） ──
        if dist < GOTO_SLOW_DIST and dist > 0:
            decay = (dist - GOTO_DB) / (GOTO_SLOW_DIST - GOTO_DB)
            decay = max(0.0, min(1.0, decay))
            vx_cmd *= decay
            vy_cmd *= decay

        # ── 速度限幅 ──
        speed = math.sqrt(vx_cmd * vx_cmd + vy_cmd * vy_cmd)
        if speed > GOTO_MAX_SPEED:
            vx_cmd = vx_cmd / speed * GOTO_MAX_SPEED
            vy_cmd = vy_cmd / speed * GOTO_MAX_SPEED

        # ── 最小速度保持（克服静摩擦） ──
        if 0 < speed < GOTO_MIN_SPEED and dist > GOTO_DB:
            vx_cmd = vx_cmd / speed * GOTO_MIN_SPEED
            vy_cmd = vy_cmd / speed * GOTO_MIN_SPEED

        # ── 驱动电机 ──
        try:
            drive_fn(vx_cmd, vy_cmd, wz, GOTO_CTRL_DT)
        except Exception as e:
            print("  [{}] 驱动错误:".format(label), e)

        time.sleep_ms(int(GOTO_CTRL_DT * 1000))


# ═══════════════════════════════════════════════════════════════
#  独立调试入口
# ═══════════════════════════════════════════════════════════════

if __name__ == '__main__':
    """
    独立调试：初始化 UWB + 虚拟回调 → 导航到指定坐标。
    实际部署时需替换为真实的 IMU / 电机 / SW2 回调。
    """
    from uwb_position import UWBPosition

    print("=" * 50)
    print("  uwb_control 独立调试模式")
    print("=" * 50)

    # ── 初始化 UWB ──
    uwb = UWBPosition(uart_id=0, baudrate=115200, target_anchor="8834")
    print("等待 UWB 首帧...")
    wait_start = time.ticks_ms()
    while uwb.get_frame_count() == 0:
        uwb.step()
        if time.ticks_diff(time.ticks_ms(), wait_start) > 3000:
            print("UWB 首帧超时，退出")
            uwb.stop()
            raise SystemExit
        time.sleep_ms(10)
    print("UWB 就绪 (frame={})".format(uwb.get_frame_count()))

    # ── 虚拟回调（调试时用，实际部署替换为真实函数） ──
    def _lock_fn():
        print("  [DEBUG] 航向锁定: 0.0°（虚拟）")
        return 0.0

    def _wz_fn(target, db):
        return 0.0  # 无航向修正

    def _yaw_fn():
        return 0.0

    def _abort_fn():
        return False  # 不中断

    def _drive_fn(vx, vy, wz, dt):
        print("  [DEBUG] drive vx={:.3f} vy={:.3f} wz={:.3f}".format(vx, vy, wz))

    def _stop_fn():
        print("  [DEBUG] stop")

    def _led_fn(val):
        pass

    # ── 导航到默认测试坐标 ──
    TEST_X = 40.0
    TEST_Y = 50.0
    arrived, reason = goto_location(
        uwb, TEST_X, TEST_Y,
        _lock_fn, _wz_fn, _yaw_fn,
        _abort_fn, _drive_fn, _stop_fn,
        led_fn=None, label="TEST"
    )

    print("\n结果: arrived={} reason={}".format(arrived, reason))
    uwb.stop()
