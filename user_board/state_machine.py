"""
state_machine.py — 补给任务状态机（SupplyMissionSM）

【层】控制层 — 8 状态有限状态机
【职责】通过回调注入编排 RT1021 全向轮小车在补给任务中的全部控制逻辑，
       零直接硬件访问。所有传感/驱动/通信均通过 callbacks dict 注入。

【依赖】
  - uwb_control.goto_location()  : UWB 闭环导航
  - cam_follow.cam_approach()    : 摄像头闭环靠近

【状态转移图】
  State 1 (goto_supplies)    ──到达──▶ State 2
                              ──中断──▶ State 8
  State 2 (precise_position) ──到达──▶ State 3
  State 3 (search_and_follow)──发现──▶ (cam_approach) ──到达──▶ State 4
                              ──未发现─▶ State 7
  State 4 (bt_interaction_1) ──完成──▶ State 5
  State 5 (sprint_forward)   ──触发──▶ State 6
  State 6 (bt_interaction_2) ──完成──▶ State 7
  State 7 (return_and_branch)──有待搜──▶ State 3
                              ──均已搜─▶ State 8
                              ──已找到─▶ (goto_supplies) ──▶ State 3
  State 8 (return_to_origin) ──到达──▶ 结束
"""

import gc, time, math

# ═══════════════════════════════════════════════════════════════
#  物资区搜索参数
# ═══════════════════════════════════════════════════════════════
SEARCH_AREA_SIZE_CM      = 100.0   # 搜索区域边长 (cm)
SEARCH_ROW_STEP_CM       = 40.0    # 行间步进距离 (cm)
SEARCH_SPEED             = 0.40    # 搜索速度 (m/s)
SEARCH_ROW_TIMEOUT_S     = 30.0    # 单行搜索超时 (s)
SEARCH_STEP_TIMEOUT_S    = 10.0    # 步进超时 (s)
SEARCH_CTRL_DT           = 0.02    # 控制周期 (s)

# ═══════════════════════════════════════════════════════════════
#  冲刺与边界参数
# ═══════════════════════════════════════════════════════════════
STARTUP_FULL_SPEED     = 1.00     # 全速前进速度 (m/s)
UWB_X_THRESHOLD_CM     = -130.0   # UWB X 轴距离阈值: 小于此值触发停车 (cm)
UWB_X_SLOWDOWN_CM      = -80.0    # 减速起始 X 坐标 (cm)
UWB_X_MIN_SPEED        = 0.25     # 接近阈值时的最低速度 (m/s)
UWB_X_TIMEOUT_S        = 15.0     # 冲刺阶段超时 (s)
UWB_BACKUP_DIST_CM     = 20.0     # 触发后倒退距离 (cm)
UWB_BACKUP_SPEED       = 0.50     # 倒退速度 (m/s)
UWB_BACKUP_TIMEOUT_S   = 8.0      # 倒退超时 (s)
UWB_DEAD_TIMEOUT_S     = 2.0      # UWB 掉线容忍超时 (s)

# ═══════════════════════════════════════════════════════════════
#  非线性刹车曲线参数 — 弹射起步 + 末期急刹停稳
# ═══════════════════════════════════════════════════════════════
# 刹车曲线: speed = END + (MAX - END) * (1-ratio)^POWER
#   ratio=0 (远) → speed=MAX (全速)    ratio=1 (近) → speed=END (近零)
#   POWER>1 使曲线下凹: 远端保持高速更久, 末期急刹 — 对比线性减速动量更小
BRAKE_CURVE_POWER    = 2.0     # 刹车幂次: 1.0=线性, 2.0=二次, 3.0=三次(更激进)
BRAKE_END_SPEED      = 0.05    # 刹车终点速度 (m/s) — 近零动量, stop_all() 无冲击
BACKUP_BRAKE_RATIO   = 0.70    # 倒退刹车起点 (占目标距离比例) — 后 30% 开始减速
BRAKE_WZ_FLOOR_SPEED = 0.30    # 低于此速度按比例缩减 wz — 防低速旋转主导"回转"

# ═══════════════════════════════════════════════════════════════
#  蓝牙通信参数
# ═══════════════════════════════════════════════════════════════
BT_WAIT_DEADLINE_S       = 15.0    # 蓝牙阻塞等待超时 (s)

# ═══════════════════════════════════════════════════════════════
#  任务
# ═══════════════════════════════════════════════════════════════
BLIND_TOLERANCE      = 3       # 黄线确认连续帧数 — 丢失后需连续 N 帧无检测才复位


class SupplyMissionSM:
    """补给任务状态机 — 8 状态全闭环自动控制。

    通过回调注入实现零直接硬件访问。所有传感、驱动、通信均通过
    callbacks dict 注入，状态机仅负责流程编排与控制逻辑。
    """

    def __init__(self, callbacks: dict):
        """初始化状态机。

        callbacks dict 键值与类型:
            'origin'          : (float, float) — 起点坐标 (x_cm, y_cm)
            'supplies'        : (float, float) — 物资区坐标 (x_cm, y_cm)
            'lock_heading'    : () -> float      — 锁定并返回目标航向角(度)
            'heading_correct' : (target_deg) -> wz — 计算航向修正（死区由 IMU_hold.HOLD_DEADBAND 全局控制）
            'maintain_yaw'    : (target_deg) -> bool — 单次维持航向+驱动
            'get_yaw'         : () -> float       — 获取当前航向角(度)
            'encoder_reset'   : () -> None        — 重置编码器/PID积分
            'abort_check'     : () -> bool        — SW2+看门狗中断检测
            'drive'           : (vx, vy, wz, dt)  — 闭环驱动电机
            'stop'            : () -> None        — 急停所有电机
            'led'             : (bool) -> None    — LED控制
            'get_encoder'     : () -> [float]*4   — 获取4轮编码器脉冲增量
            'encoder_scale'   : [float]*4         — 4轮编码器比例因子
            'ctrl_dt'         : float             — 控制周期 (s), 供 encoder speed 计算
            'uwb'             : UWBPosition       — UWB实例 (含step/position/is_timeout)
            'cam'             : CameraController  — 摄像头实例 (含step/reset)
            'bt_send_0'       : () -> None        — 蓝牙发送"0"(turn_right)
            'bt_send_1'       : () -> None        — 蓝牙发送"1"(turn_left)
            'bt_wait_ok'      : (timeout_ms) -> bool — 蓝牙等待ok应答
            'pause_yaw_hold'  : (target_deg, duration_ms) -> None — 暂停并保持航向
        """
        self.cb = callbacks
        self.state = 1  # 当前状态

        # ── 全程锁定航向 ──
        self._target_heading = None

        # ── 搜索方向与状态追踪 ──
        self._search_direction = "RIGHT"   # 当前搜索方向: "RIGHT" / "LEFT"
        self._right_searched = False       # 右侧区域是否已搜索完毕
        self._left_searched = False        # 左侧区域是否已搜索完毕
        self._found_target = False         # 本轮是否已捕获目标（驱动 cam_approach 成功）

        # ── 搜索行计数器 ──
        self._search_rows_per_side = max(1, int(math.ceil(
            SEARCH_AREA_SIZE_CM / SEARCH_ROW_STEP_CM)))
        self._search_rows_done = 0

        # ── 共享编码器融合滤波器（全状态复用，不重建）──
        self._fusion = None
        if callbacks.get('get_encoder') and callbacks.get('encoder_scale'):
            from uwb_fusion import UWBComplementaryFilter
            self._fusion = UWBComplementaryFilter(uwb_gain=0.08)

    # ═══════════════════════════════════════════════════════════
    #  内部辅助方法
    # ═══════════════════════════════════════════════════════════

    def _maintain_heading(self):
        """单次航向保持迭代：委托给 maintain_yaw 回调驱动电机。"""
        self.cb['maintain_yaw'](self._target_heading)

    def _heading_correction(self):
        """计算航向修正 wz（不驱动，仅返回 wz 值）。死区由 IMU_hold.HOLD_DEADBAND 全局控制。"""
        return self.cb['heading_correct'](self._target_heading)

    def _goto(self, target_x, target_y, label="GOTO", on_progress=None):
        """通用 UWB 导航到目标点（封装 goto_location 样板回调）。"""
        from uwb_control import goto_location

        def _lock_fn(): return self._target_heading
        def _wz_fn(t): return self.cb['heading_correct'](t)
        def _yaw_fn(): return self.cb['get_yaw']()
        def _abort_fn(): return self.cb['abort_check']()
        def _drive_fn(vx, vy, wz, dt): self.cb['drive'](vx, vy, wz, dt)
        def _stop_fn(): self.cb['stop']()
        def _led_fn(val): self.cb['led'](val)
        def _enc_fn(): return self.cb['get_encoder']()
        _enc_scale = self.cb['encoder_scale']
        def _drive_with_spd(vx, vy, wz, dt, spd):
            self.cb['drive_with_speeds'](vx, vy, wz, dt, spd)

        return goto_location(
            self.cb['uwb'], target_x, target_y,
            _lock_fn, _wz_fn, _yaw_fn,
            _abort_fn, _drive_fn, _stop_fn,
            _enc_fn, _enc_scale, _drive_with_spd,
            self._fusion,
            _led_fn, label=label, on_progress=on_progress
        )

    def _state_transition(self, from_state, to_state, reason=""):
        """打印状态转移消息并切换状态。"""
        state_names = {
            1: "goto_supplies", 2: "precise_position",
            3: "search_and_follow", 4: "bt_interaction_1",
            5: "sprint_forward", 6: "bt_interaction_2",
            7: "return_and_branch", 8: "return_to_origin",
        }
        fname = state_names.get(from_state, str(from_state))
        tname = state_names.get(to_state, str(to_state))
        extra = "  — " + reason if reason else ""
        print("[SM] State {}→{}: {}→{}{}".format(
            from_state, to_state, fname, tname, extra))
        self.state = to_state

    # ═══════════════════════════════════════════════════════════
    #  通用前进函数（编码器里程计 + 航向保持）
    # ═══════════════════════════════════════════════════════════

    def _forward_distance(self, dist_m, speed, timeout_s, label="FWD"):
        """通用前进：编码器距离闭环 + IMU 航向保持。

        参数:
            dist_m   : 目标距离 (m)
            speed    : 前进速度 (m/s)
            timeout_s: 超时 (s)
            label    : 日志前缀

        返回: True=到达, False=超时或中断
        """
        print("  [SM:{}] 前进 {:.0f}cm 开始...".format(label, dist_m * 100))

        enc_scale = self.cb['encoder_scale']
        ctrl_dt = self.cb.get('ctrl_dt', SEARCH_CTRL_DT)

        total_dists = [0.0, 0.0, 0.0, 0.0]
        start_ms = time.ticks_ms()
        last_print_ms = start_ms
        loop_cnt = 0

        while True:
            if self.cb['abort_check']():
                return False

            elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            if elapsed > timeout_s:
                print("  [SM:{}] 超时 ({:.1f}s)".format(label, elapsed))
                return False

            counts = self.cb['get_encoder']()
            if counts is None or len(counts) < 4:
                time.sleep_ms(5)
                continue

            # 对单次增量脉冲取绝对值累加，规避镜像接线极性抵消
            for i in range(4):
                if enc_scale[i] != 0:
                    total_dists[i] += abs(counts[i]) / abs(enc_scale[i])
            avg_dist = sum(total_dists) / len(total_dists)

            if avg_dist >= dist_m:
                print("  [SM:{}] 到达目标！dist={:.2f}m".format(label, avg_dist))
                return True

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print_ms) >= 500:
                last_print_ms = now
                print("  [SM:{}] dist={:.2f}m / {:.2f}m  yaw={:.2f}°".format(
                    label, avg_dist, dist_m, self.cb['get_yaw']()))

            loop_cnt += 1
            if loop_cnt % 50 == 0:
                gc.collect()

            wz = self._heading_correction()

            try:
                rs = [counts[i] / enc_scale[i] / ctrl_dt if enc_scale[i] != 0 else 0
                      for i in range(4)]
                self.cb['drive'](speed, 0, wz, ctrl_dt)
            except Exception as e:
                print("  [SM:{}] 驱动错误:".format(label), e)

            time.sleep_ms(int(ctrl_dt * 1000))

    def _execute_backup(self):
        """执行倒退 UWB_BACKUP_DIST_CM 厘米的闭环辅助函数。

        倒退时保持航向修正，使用编码器绝对值累加计量距离。
        """
        backup_dist_m = UWB_BACKUP_DIST_CM / 100.0
        print("  [SM:BACKUP] 开始倒退 {:.0f}cm...".format(UWB_BACKUP_DIST_CM))

        enc_scale = self.cb['encoder_scale']
        ctrl_dt = self.cb.get('ctrl_dt', SEARCH_CTRL_DT)
        backup_dists = [0.0, 0.0, 0.0, 0.0]
        start_ms = time.ticks_ms()
        _loop_bk = 0

        while True:
            _loop_bk += 1
            if _loop_bk % 100 == 0:
                gc.collect()
            
            if self.cb['abort_check']():
                return

            elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            if elapsed > UWB_BACKUP_TIMEOUT_S:
                print("  [SM:BACKUP] 倒退超时 ({:.1f}s)，放弃倒退".format(elapsed))
                break

            counts = self.cb['get_encoder']()
            if counts is None or len(counts) < 4:
                time.sleep_ms(5)
                continue

            for i in range(4):
                if enc_scale[i] != 0:
                    backup_dists[i] += abs(counts[i]) / abs(enc_scale[i])
            avg_backup = sum(backup_dists) / len(backup_dists)

            if abs(avg_backup) >= backup_dist_m:
                print("  [SM:BACKUP] 倒退完成 dist={:.2f}m".format(abs(avg_backup)))
                break

            # 非线性刹车曲线: 后 30% 距离幂次减速至近零
            progress = abs(avg_backup) / backup_dist_m
            if progress > BACKUP_BRAKE_RATIO:
                brake_ratio = (progress - BACKUP_BRAKE_RATIO) / (1.0 - BACKUP_BRAKE_RATIO)
                brake_ratio = max(0.0, min(1.0, brake_ratio))
                brake_factor = (1.0 - brake_ratio) ** BRAKE_CURVE_POWER
                backup_v = BRAKE_END_SPEED + (UWB_BACKUP_SPEED - BRAKE_END_SPEED) * brake_factor
            else:
                backup_v = UWB_BACKUP_SPEED

            wz = self._heading_correction()
            # 低速段按比例缩减 wz — 防止刹车末期旋转主导
            if backup_v < BRAKE_WZ_FLOOR_SPEED:
                wz = wz * (backup_v / BRAKE_WZ_FLOOR_SPEED)

            try:
                rs = [counts[i] / enc_scale[i] / ctrl_dt
                      if enc_scale[i] != 0 else 0 for i in range(4)]
                self.cb['drive'](-backup_v, 0, wz, ctrl_dt)
            except Exception as e:
                print("  [SM:BACKUP] 倒退驱动错误:", e)

            time.sleep_ms(int(ctrl_dt * 1000))

        self.cb['stop']()
        time.sleep_ms(300)
        self.cb['encoder_reset']()

    # ═══════════════════════════════════════════════════════════
    #  State 1 — 导航到物资区
    # ═══════════════════════════════════════════════════════════

    def _state_goto_supplies(self):
        """State 1: UWB 导航到物资区。摄像头随 S1 启动立即激活。"""
        target_x, target_y = self.cb['supplies']
        print("\n  [SM:S1] === 导航到物资区 ({:.1f}, {:.1f}) ===".format(target_x, target_y))

        # S1 启动即激活摄像头，为后续 S3 搜索预加载
        self.cb['cam'].reset()

        arrived, reason = self._goto(target_x, target_y, label="S1")

        if reason == 'aborted':
            self._state_transition(1, 8, "SW2中断→返航")
        elif reason == 'timeout':
            print("  [SM:S1] 超时, 继续精确定位")
            self._state_transition(1, 2, "导航超时但继续")
        else:
            self._state_transition(1, 2, "到达物资区")

    # ═══════════════════════════════════════════════════════════
    #  State 2 — UWB 精细定位
    # ═══════════════════════════════════════════════════════════

    def _state_precise_position(self):
        """State 2: UWB 精细定位，使用更小的死区微调到位。"""
        target_x, target_y = self.cb['supplies']

        print("\n  [SM:S2] === 精细定位到物资区 ({:.1f}, {:.1f}) ===".format(
            target_x, target_y))

        # 精细定位：减半死区，降低最大速度防止过冲
        # 注: 控制参数已移至 uwb_follow.py，动态覆写需写 uwb_follow 模块
        import uwb_follow
        save_zx = uwb_follow.GOTO_ZONE_DX
        save_zy = uwb_follow.GOTO_ZONE_DY
        save_max = uwb_follow.GOTO_MAX_SPEED
        uwb_follow.GOTO_ZONE_DX = save_zx * 0.5  # 精细定位使用标准阈值的一半
        uwb_follow.GOTO_ZONE_DY = save_zy * 0.5
        uwb_follow.GOTO_MAX_SPEED = 0.30

        try:
            arrived, reason = self._goto(target_x, target_y, label="PRECISE")
        finally:
            uwb_follow.GOTO_ZONE_DX = save_zx
            uwb_follow.GOTO_ZONE_DY = save_zy
            uwb_follow.GOTO_MAX_SPEED = save_max

        if arrived:
            self._state_transition(2, 3, "精确定位完成→开始搜索")
        elif reason == 'aborted':
            self._state_transition(2, 8, "SW2中断→返航")
        else:
            print("  [SM:S2] 精细定位异常 ({}), 仍进入搜索".format(reason))
            self._state_transition(2, 3, "定位异常但继续搜索")

    # ═══════════════════════════════════════════════════════════
    #  State 3 — 搜索与跟随
    # ═══════════════════════════════════════════════════════════

    def _state_search_and_follow(self):
        """State 3: 横向搜索单行，发现目标后调用 cam_approach 闭环靠近。

        根据 self._search_direction 决定搜索方向 ("RIGHT" 或 "LEFT")。
        每周期检查 cam.step() 是否有 has_target。
        搜索完毕未发现 → 记录结果，转移至 State 7。
        """
        direction = self._search_direction
        vy_sign = 1.0 if direction == "RIGHT" else -1.0
        row_dist_m = SEARCH_AREA_SIZE_CM / 100.0

        print("\n  [SM:S3] === 横向搜索 {} {:.0f}cm ===".format(
            direction, SEARCH_AREA_SIZE_CM))
        print("  [SM:S3] 航向锁定: {:.1f}°".format(self._target_heading))

        enc_scale = self.cb['encoder_scale']
        ctrl_dt = SEARCH_CTRL_DT

        total_dists = [0.0, 0.0, 0.0, 0.0]
        start_ms = time.ticks_ms()
        last_print_ms = start_ms
        loop_cnt = 0

        self.cb['led'](True)
        self.cb['encoder_reset']()

        while True:
            if self.cb['abort_check']():
                self.cb['led'](False)
                self._state_transition(3, 8, "SW2中断→返航")
                return

            # ── 超时检测 ──
            elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            if elapsed > SEARCH_ROW_TIMEOUT_S:
                print("  [SM:S3] 行搜索超时 ({:.1f}s)".format(elapsed))
                self.cb['led'](False)
                self._search_rows_done += 1
                self._state_transition(3, 7, "搜索超时→分支")
                return

            # ── 编码器读取 ──
            counts = self.cb['get_encoder']()
            if counts is None or len(counts) < 4:
                time.sleep_ms(5)
                continue

            for i in range(4):
                if enc_scale[i] != 0:
                    total_dists[i] += abs(counts[i]) / abs(enc_scale[i])
            avg_dist = sum(total_dists) / len(total_dists)

            # ── 摄像头检测 ──
            ctrl = self.cb['cam'].step()
            if ctrl.get('has_target'):
                # 发现目标 → 停止当前搜索，进入 cam_approach 闭环靠近
                self.cb['stop']()
                print("  [SM:S3] 摄像头识别到物品！进入闭环靠近...")
                self.cb['led'](False)

                arrived = self._cam_approach_target()
                if arrived:
                    self._found_target = True
                    self.cb['cam'].reset()  # 停车确定 → 放弃 XY 误差处理
                    self._state_transition(3, 4, "靠近成功→蓝牙交互1")
                else:
                    # 靠近失败（丢锁/超时）→ 返回 State 7 分支
                    self._state_transition(3, 7, "靠近失败→分支")
                return

            # ── 行搜索完成判定 ──
            if avg_dist >= row_dist_m:
                print("  [SM:S3] {} 方向搜索完成 dist={:.2f}m".format(direction, avg_dist))
                self.cb['stop']()
                self.cb['led'](False)
                self._search_rows_done += 1
                # 标记当前方向搜索完毕
                if direction == "RIGHT":
                    self._right_searched = True
                else:
                    self._left_searched = True
                self._state_transition(3, 7, "搜索完成未发现→分支")
                return

            # ── 状态打印 ──
            now = time.ticks_ms()
            if time.ticks_diff(now, last_print_ms) >= 500:
                last_print_ms = now
                print("  [SM:S3] {} dist={:.2f}m / {:.0f}cm yaw={:.2f}°".format(
                    direction, avg_dist, SEARCH_AREA_SIZE_CM, self.cb['get_yaw']()))

            loop_cnt += 1
            if loop_cnt % 20 == 0:
                gc.collect()

            # ── 航向修正 + 驱动 ──
            wz = self._heading_correction()

            try:
                rs = [counts[i] / enc_scale[i] / ctrl_dt if enc_scale[i] != 0 else 0
                      for i in range(4)]
                self.cb['drive'](0, vy_sign * SEARCH_SPEED, wz, ctrl_dt)
            except Exception as e:
                print("  [SM:S3] 驱动错误:", e)

            time.sleep_ms(int(ctrl_dt * 1000))

    def _cam_approach_target(self):
        """调用 cam_approach() 闭环靠近目标直到到达判定。

        返回: True=到达, False=丢失/超时/中断
        """
        from cam_follow import cam_approach

        def _lock_fn():
            return self._target_heading

        def _wz_fn(target):
            return self.cb['heading_correct'](target)

        def _abort_fn():
            return self.cb['abort_check']()

        def _drive_fn(vx, vy, wz, dt):
            self.cb['drive'](vx, vy, wz, dt)

        def _stop_fn():
            self.cb['stop']()

        def _led_fn(val):
            self.cb['led'](val)

        arrived, reason = cam_approach(
            self.cb['cam'], _lock_fn, _wz_fn,
            _abort_fn, _drive_fn, _stop_fn, _led_fn
        )

        if arrived:
            print("  [SM:S3] cam_approach 到达: {}".format(reason))
            return True

        print("  [SM:S3] cam_approach 未到达: {}".format(reason))
        return False

    # ═══════════════════════════════════════════════════════════
    #  State 4 — 蓝牙交互 1（发送 "0"）
    # ═══════════════════════════════════════════════════════════

    def _state_bt_interaction_1(self):
        """State 4: 蓝牙发送 "0" (turn_right) 并阻塞等待 ok 应答。

        等待期间持续保持航向。收到 ok 或超时后转移至 State 5。
        """
        print("\n  [SM:S4] === 蓝牙交互 1: 发送 '0' (turn_right) ===")

        self.cb['bt_send_0']()
        print("  [SM:S4] 指令已发出，等待从车 ok（兜底 {}s）...".format(BT_WAIT_DEADLINE_S))

        deadline = time.ticks_ms() + int(BT_WAIT_DEADLINE_S * 1000)

        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            if self.cb['abort_check']():
                print("  [SM:S4] SW2 中断，强制跳过")
                self._state_transition(4, 5, "SW2跳过→冲刺")
                return

            # 等待期间持续保持航向
            self.cb['maintain_yaw'](self._target_heading)

            if self.cb['bt_wait_ok'](10):  # 短超时轮询，高频维持航向
                print("  [SM:S4] 从车确认完毕 (ok)")
                self._state_transition(4, 5, "ok确认→冲刺")
                return

            time.sleep_ms(10)

        # 兜底超时
        print("  [SM:S4] 等待从车 ok 兜底超时 ({}s)，继续执行".format(BT_WAIT_DEADLINE_S))
        self._state_transition(4, 5, "超时→冲刺")

    # ═══════════════════════════════════════════════════════════
    #  State 5 — 全速冲刺前进
    # ═══════════════════════════════════════════════════════════

    def _state_sprint_forward(self):
        """State 5: 全速前进至触发停车条件。

        停止条件（任一触发即停）：
          1. cam.step() 检测到 line_flag (黄线)
          2. uwb.get_position()[0] < UWB_X_THRESHOLD_CM (-130cm)

        UWB 掉线保护：不熄火，降速至 0.35 m/s，移动中重连。
        线性减速：x < UWB_X_SLOWDOWN_CM (-80cm) 时比例减速。
        停车后 → 倒退 UWB_BACKUP_DIST_CM → State 6。
        """
        print("\n  [SM:S5] === 全速前进，等待 UWB X < {:.0f}cm ===".format(
            UWB_X_THRESHOLD_CM))
        print("  [SM:S5] 航向锁定: {:.1f}°  开始直行推进".format(self._target_heading))

        enc_scale = self.cb['encoder_scale']
        ctrl_dt = self.cb.get('ctrl_dt', SEARCH_CTRL_DT)

        start_ms = time.ticks_ms()
        last_print_ms = start_ms
        last_uwb_ms = start_ms
        last_cam_ms = start_ms
        loop_cnt = 0
        uwb_dead_start = 0

        # ── 摄像头黄线检测初始化 ──
        self.cb['cam'].reset()
        armed_to_trigger = False
        yellow_lost_count = 0
        ctrl = None

        self.cb['led'](True)

        # ── 超时后摄像头兜底模式 ──
        _timeout_fallback = False          # UWB 超时后启用
        _timeout_fallback_start = 0        # 兜底模式开始时间
        CAM_FALLBACK_TIMEOUT_S = 8.0       # 摄像头兜底最大时长

        while True:
            if self.cb['abort_check']():
                self.cb['led'](False)
                self._state_transition(5, 8, "SW2中断→返航")
                return

            # ── 总超时 ──
            elapsed = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            if elapsed > UWB_X_TIMEOUT_S and not _timeout_fallback:
                print("  [SM:S5] 超时 ({:.1f}s)，UWB 未达标 → 摄像头兜底模式".format(elapsed))
                _timeout_fallback = True
                _timeout_fallback_start = time.ticks_ms()

            # ── 编码器 ──
            counts = self.cb['get_encoder']()
            if counts is None or len(counts) < 4:
                time.sleep_ms(5)
                continue

            now = time.ticks_ms()

            # ── UWB & 摄像头步进（移动中高频执行） ──
            if time.ticks_diff(now, last_uwb_ms) >= 50:
                self.cb['uwb'].step()
                last_uwb_ms = now

            if time.ticks_diff(now, last_cam_ms) >= 50:
                ctrl = self.cb['cam'].step()
                last_cam_ms = now

            # ── 黄线越界视觉安全检测（核心安全兜底） ──
            line_detected = ctrl.get('line_flag', 0) if ctrl else 0
            if line_detected:
                armed_to_trigger = True
                yellow_lost_count = 0
            elif armed_to_trigger:
                yellow_lost_count += 1

            if armed_to_trigger and yellow_lost_count >= BLIND_TOLERANCE:
                self.cb['stop']()
                print("  [SM:S5] 黄线越界确认 (连续{}帧) → 触发停车！".format(
                    BLIND_TOLERANCE))
                self._execute_backup()
                self._state_transition(5, 6, "黄线停车+倒退→蓝牙交互2")
                return

            # ── 摄像头兜底超时：超时后完全依赖视觉，设硬 deadline ──
            if _timeout_fallback:
                fb_elapsed = time.ticks_diff(time.ticks_ms(), _timeout_fallback_start) / 1000.0
                if fb_elapsed > CAM_FALLBACK_TIMEOUT_S:
                    self.cb['stop']()
                    print("  [SM:S5] 摄像头兜底超时 ({:.1f}s)，强制停车".format(fb_elapsed))
                    self._execute_backup()
                    self._state_transition(5, 6, "兜底超时+倒退→蓝牙交互2")
                    return

            # ── UWB 掉线判定与移动中自动重建 ──
            uwb_is_active = True
            if self.cb['uwb'].is_timeout():
                uwb_is_active = False
                if uwb_dead_start == 0:
                    uwb_dead_start = time.ticks_ms()
                    print("  [SM:S5] UWB 数据中断，车身保持安全巡航速度并于移动中尝试重连...")
                elif time.ticks_diff(time.ticks_ms(), uwb_dead_start) > UWB_DEAD_TIMEOUT_S * 1000:
                    # UWB 掉线超时后仍不熄火，维持安全速度推进
                    if self.cb['uwb'].is_uart_alive():
                        print("  [SM:S5] UART 仍活跃，延长移动等待 (帧过滤中)...")
                        uwb_dead_start = time.ticks_ms()
                    else:
                        print("  [SM:S5] UART 离线，维持安全车速继续推进...")
                        uwb_dead_start = time.ticks_ms()
            else:
                if uwb_dead_start != 0:
                    print("  [SM:S5] UWB 链路已自主恢复")
                uwb_dead_start = 0

            # ── 根据 UWB 链路健康度动态决策车速 ──
            fwd_speed = UWB_X_MIN_SPEED
            if uwb_is_active:
                try:
                    x_cm, y_cm = self.cb['uwb'].get_position()

                    # UWB 正常到达
                    if x_cm < UWB_X_THRESHOLD_CM:
                        self.cb['stop']()
                        print("  [SM:S5] UWB X={:.1f}cm < {:.0f}cm，坐标触发停车！".format(
                            x_cm, UWB_X_THRESHOLD_CM))
                        self._execute_backup()
                        self._state_transition(5, 6, "UWB坐标触发+倒退→蓝牙交互2")
                        return

                    # 非线性刹车曲线: (1-ratio)^n — 远端全速保持, 末期急刹至近零
                    if x_cm < UWB_X_SLOWDOWN_CM:
                        slowdown_range = UWB_X_SLOWDOWN_CM - UWB_X_THRESHOLD_CM
                        if slowdown_range > 0:
                            ratio = (UWB_X_SLOWDOWN_CM - x_cm) / slowdown_range
                            ratio = max(0.0, min(1.0, ratio))
                            brake_factor = (1.0 - ratio) ** BRAKE_CURVE_POWER
                            fwd_speed = BRAKE_END_SPEED + (STARTUP_FULL_SPEED - BRAKE_END_SPEED) * brake_factor
                    else:
                        fwd_speed = STARTUP_FULL_SPEED
                except Exception as e:
                    print("  [SM:S5] get_position() 读取异常:", e)
                    fwd_speed = UWB_X_MIN_SPEED
            else:
                # 掉线期间的安全匀速直行速度
                fwd_speed = 0.35

            # ── UWB 超时兜底：强制安全速度，仅靠摄像头黄线 ──
            if _timeout_fallback:
                fwd_speed = UWB_X_MIN_SPEED

            # ── 状态打印 ──
            if time.ticks_diff(now, last_print_ms) >= 500:
                last_print_ms = now
                st = "HEALTHY" if uwb_is_active else "DROPPED(RUNNING)"
                print("  [SM:S5] UWB_Link={} v={:.2f}m/s yaw={:.2f}° t={:.1f}s".format(
                    st, fwd_speed, self.cb['get_yaw'](), elapsed))

            loop_cnt += 1
            if loop_cnt % 50 == 0:
                gc.collect()

            # ── 航向修正 + 驱动 ──
            wz = self._heading_correction()
            # 低速段按比例缩减 wz — 防止刹车末期旋转主导导致"回转"振荡
            if fwd_speed < BRAKE_WZ_FLOOR_SPEED:
                wz = wz * (fwd_speed / BRAKE_WZ_FLOOR_SPEED)

            try:
                rs = [counts[i] / enc_scale[i] / ctrl_dt if enc_scale[i] != 0 else 0
                      for i in range(4)]
                self.cb['drive'](fwd_speed, 0, wz, ctrl_dt)
            except Exception as e:
                print("  [SM:S5] 驱动运行错误:", e)

            time.sleep_ms(int(ctrl_dt * 1000))

    # ═══════════════════════════════════════════════════════════
    #  State 6 — 蓝牙交互 2（发送 "1"）
    # ═══════════════════════════════════════════════════════════

    def _state_bt_interaction_2(self):
        """State 6: 蓝牙发送 "1" (turn_left) 并阻塞等待 ok 应答。

        等待期间持续保持航向。收到 ok 或超时后转移至 State 7。
        """
        print("\n  [SM:S6] === 蓝牙交互 2: 发送 '1' (turn_left) ===")

        self.cb['bt_send_1']()
        print("  [SM:S6] 指令已发出，等待从车 ok（兜底 {}s）...".format(BT_WAIT_DEADLINE_S))

        deadline = time.ticks_ms() + int(BT_WAIT_DEADLINE_S * 1000)

        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            if self.cb['abort_check']():
                print("  [SM:S6] SW2 中断，强制跳过")
                self._state_transition(6, 7, "SW2跳过→分支")
                return

            # 等待期间持续保持航向
            self.cb['maintain_yaw'](self._target_heading)

            if self.cb['bt_wait_ok'](10):  # 短超时轮询，高频维持航向
                print("  [SM:S6] 从车确认完毕 (ok)")
                self._state_transition(6, 7, "ok确认→分支")
                return

            time.sleep_ms(10)

        # 兜底超时
        print("  [SM:S6] 等待从车 ok 兜底超时 ({}s)，继续执行".format(BT_WAIT_DEADLINE_S))
        self._state_transition(6, 7, "超时→分支")

    # ═══════════════════════════════════════════════════════════
    #  State 7 — 返回与分支
    # ═══════════════════════════════════════════════════════════

    def _state_return_and_branch(self):
        """State 7: 复杂分支逻辑，决定下一步搜索策略。

        分支：
          1. 已找到目标 → 导航回 supplies → 继续搜索
          2. 右侧未搜完 → 步进 → 继续搜右
          3. 右侧已搜完、左侧未搜 → 步进 → 搜左
          4. 左右均已搜完 → 返航
        """
        print("\n  [SM:S7] === 返回与分支判定 ===")
        print("  [SM:S7] found_target={} right_searched={} left_searched={} rows_done={}".format(
            self._found_target, self._right_searched, self._left_searched,
            self._search_rows_done))

        # S6→S7: 重启摄像头跟随（从蓝牙交互恢复，准备下轮搜索）
        self.cb['cam'].reset()
        print("  [SM:S7] 摄像头跟随已重启")

        if self._found_target:
            # 已找到目标 → 导航回物资区开始下一轮搜索
            print("  [SM:S7] 已找到目标，返回物资区继续搜索")
            self._found_target = False  # 重置目标标记
            self._search_rows_done = 0
            self._right_searched = False
            self._left_searched = False
            self._search_direction = "RIGHT"

            target_x, target_y = self.cb['supplies']
            arrived, reason = self._goto(target_x, target_y, label="S7_GOTO")

            if reason == 'aborted':
                self._state_transition(7, 8, "导航中断→返航")
            else:
                self.cb['encoder_reset']()
                self._state_transition(7, 3, "回到物资区→继续搜索")
            return

        if not self._right_searched:
            # 右侧还没搜完 → 步进后继续搜右
            print("  [SM:S7] 右侧未搜完，行间步进 {:.0f}cm 后继续搜右".format(
                SEARCH_ROW_STEP_CM))
            self.cb['encoder_reset']()

            if self._forward_distance(
                SEARCH_ROW_STEP_CM / 100.0, SEARCH_SPEED,
                SEARCH_STEP_TIMEOUT_S, label="STEP_R"
            ):
                self._search_direction = "RIGHT"
                self._right_searched = True  # 标记已尝试，防止超时后无限重试
                self._state_transition(7, 3, "步进完成→搜右")
            else:
                self._state_transition(7, 8, "步进中断→返航")
            return

        if not self._left_searched:
            # 右侧搜完没找到 → 步进后搜左
            print("  [SM:S7] 右侧已搜完，步进 {:.0f}cm 后搜左".format(
                SEARCH_ROW_STEP_CM))
            self.cb['encoder_reset']()

            if self._forward_distance(
                SEARCH_ROW_STEP_CM / 100.0, SEARCH_SPEED,
                SEARCH_STEP_TIMEOUT_S, label="STEP_L"
            ):
                self._search_direction = "LEFT"
                self._left_searched = True   # 标记已尝试，防止超时后无限重试
                self._state_transition(7, 3, "步进完成→搜左")
            else:
                self._state_transition(7, 8, "步进中断→返航")
            return

        # 左右均已搜完 → 返航
        print("  [SM:S7] 左右侧均已搜索完毕，未发现目标 → 返航")
        self._state_transition(7, 8, "搜索枯竭→返航")

    # ═══════════════════════════════════════════════════════════
    #  State 8 — 返回原点
    # ═══════════════════════════════════════════════════════════

    def _state_return_to_origin(self):
        """State 8: UWB 导航回 origin 坐标，到达后结束 run()。

        这是最终状态，导航完成后 run() 主循环通过 break 退出。
        """
        target_x, target_y = self.cb['origin']

        print("\n  [SM:S8] === 返航到原点 ({:.1f}, {:.1f}) ===".format(
            target_x, target_y))

        arrived, reason = self._goto(target_x, target_y, label="RTN")

        if arrived:
            print("  [SM:S8] 已到达原点，任务完成")
        else:
            print("  [SM:S8] 返航结束 (reason={})".format(reason))

    # ═══════════════════════════════════════════════════════════
    #  run() — 主循环
    # ═══════════════════════════════════════════════════════════

    def run(self):
        """主控制循环：8 状态机调度器。

        在 __init__ 后调用，阻塞运行直到任务完成或 SW2 中断。
        """
        # ── 锁定全程航向 ──
        self._target_heading = self.cb['lock_heading']()
        print("[SM] 全程航向锁定: {:.1f}°".format(self._target_heading))

        while True:
            # ── 全局中断检测 ──
            if self.cb['abort_check']():
                print("[SM] SW2 中断 → 退出")
                break

            # ── 状态调度 ──
            if self.state == 1:
                self._state_goto_supplies()
            elif self.state == 2:
                self._state_precise_position()
            elif self.state == 3:
                self._state_search_and_follow()
            elif self.state == 4:
                self._state_bt_interaction_1()
            elif self.state == 5:
                self._state_sprint_forward()
            elif self.state == 6:
                self._state_bt_interaction_2()
            elif self.state == 7:
                self._state_return_and_branch()
            elif self.state == 8:
                self._state_return_to_origin()
                break
            else:
                print("[SM] 未知状态 {} → 退出".format(self.state))
                break

        # ── 清理 ──
        self.cb['stop']()
        self.cb['led'](False)
        print("[SM] 状态机运行结束")

    # ═══════════════════════════════════════════════════════════
    #  获取状态机摘要（调试用）
    # ═══════════════════════════════════════════════════════════

    def summary(self):
        """返回状态机当前状态的摘要字符串。"""
        state_names = {
            1: "goto_supplies", 2: "precise_position",
            3: "search_and_follow", 4: "bt_interaction_1",
            5: "sprint_forward", 6: "bt_interaction_2",
            7: "return_and_branch", 8: "return_to_origin",
        }
        return (
            "SupplyMissionSM(state={} {}) heading={:.1f}° "
            "dir={} found={} right_done={} left_done={} rows={}"
        ).format(
            self.state, state_names.get(self.state, "?"),
            self._target_heading if self._target_heading is not None else -999,
            self._search_direction, self._found_target,
            self._right_searched, self._left_searched,
            self._search_rows_done
        )
