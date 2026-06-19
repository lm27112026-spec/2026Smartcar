# RT1021 UWB 跟随 + 视觉追踪系统技术文档

> **版本**：v2.3  
> **平台**：RT1021 (NXP i.MX RT1021) + MicroPython  
> **日期**：2026-06-19  
> **修复**：回退 PID 符号和 KD 值，对齐 cam_follow_backup.py 已验证参数

---

## 目录

1. [系统概述](#1-系统概述)
2. [硬件拓扑](#2-硬件拓扑)
3. [软件架构](#3-软件架构)
4. [运行流程详解](#4-运行流程详解)
5. [UWB 跟随子系统](#5-uwb-跟随子系统)
6. [摄像头检测与切换子系统](#6-摄像头检测与切换子系统)
7. [视觉追踪子系统](#7-视觉追踪子系统)
8. [按键控制与 SW2 退出](#8-按键控制与-sw2-退出)
9. [状态机完整定义](#9-状态机完整定义)
10. [帧协议规范](#10-帧协议规范)
11. [关键参数速查表](#11-关键参数速查表)
12. [部署与调试](#12-部署与调试)

---

## 1. 系统概述

### 1.1 功能描述

运行 `main.py` 后，小车自动启动以下复合功能：

| 功能 | 描述 |
|---|---|
| **UWB 跟随** | 开机默认启动。通过 UART0 接收基站 TWR 数据，跟随指定锚点（ID="8834"）移动，边走边实时航向纠偏 |
| **摄像头后台轮询** | 在 UWB 跟随期间，后台持续读取 UART7 的 OpenMV 摄像头数据，滑动窗口判定是否检测到目标物品 |
| **模式切换** | 摄像头连续确认检测到物品后，立即中断 UWB 跟随，切换为视觉追踪模式（亦可按键手动切换） |
| **按键控制** | KEY1(C8)→UWB 模式(LED 常亮)，KEY2(C9)→视觉追踪(LED 快闪 ~2.5Hz)，KEY3(C14)→停车(LED 慢闪 ~1Hz) |
| **视觉逼近停车** | 视觉追踪通过级联 PID 控制（CascadePID），逼近物品至 **30cm** 时自动停车 |
| **SW2 安全退出** | 全程通过拨码开关 SW2 (D9) 可实现强制退出，所有电机停转、外设释放 |

### 1.2 核心文件依赖关系

```
main.py (主控，状态机 + 按键)
├── uwb_tracker.py     ← UWB 跟随控制器（类 UWBFollower）
├── cam_data.py        ← ★ 摄像头数据接收与协议解析（CamDataReceiver）
├── cam_follow.py      ← ★ 视觉追踪独立实现（级联 PID 控制）
├── motor.py           ← 电机/编码器硬件抽象层
├── imu_motion.py      ← IMU 姿态解算 + 角速度闭环
├── pid.py             ← PID 控制器基类
├── key.py             ← 按键驱动 + 硬件看门狗
├── ticker.py          ← 定时器封装
└── utils.py           ← 工具函数（角度归一化、限幅）
```

---

## 2. 硬件拓扑

### 2.1 引脚分配

| 功能 | 引脚 | 说明 |
|---|---|---|
| **SW2 拨码开关** | D9 | 上拉输入，拨动触发强制退出（50ms 消抖） |
| **LED 状态灯** | C4 | 输出，模式指示（常亮/快闪/慢闪） |
| **KEY1 按键** | C8 | 上拉输入，切换 UWB 模式 |
| **KEY2 按键** | C9 | 上拉输入，切换视觉追踪模式 |
| **KEY3 按键** | C14 | 上拉输入，切换停车状态 |
| **UART0 (UWB)** | 默认引脚 | 115200 bps，接收 TWR 基站 JSON 数据 |
| **UART7 (摄像头)** | 默认引脚 | 115200 bps，接收 OpenMV 的 AA..BB 帧 |
| **IMU (IMU660RX)** | SPI 引脚 | PIT3 ticker 10ms 自动采集 |
| **电机 PWM×4** | B26/C20/C24/C26 | 15000Hz，4 路全向轮驱动 |
| **方向引脚×8** | C28-C31, D4-D7 | TB6612 电机驱动方向控制 |
| **编码器×4** | C0-C3, D13-D16 | PIT1 ticker 10ms 自动采集 |

### 2.2 定时器分配

| 定时器 | 用途 | 周期 |
|---|---|---|
| **PIT0** | MicroPython 系统定时器 | 系统保留 |
| **PIT1** | 编码器自动采集 (`enc_ticker`) | 10ms |
| **PIT2** | 硬件看门狗 (`key.py`，独占) | 10ms |
| **PIT3** | IMU 自动采集 (`imu_motion.py`) | 10ms |

### 2.3 系统拓扑图

```
┌─────────────────────────────────────────────────────────┐
│                    RT1021 主控                           │
│                                                         │
│  UART0 ←── TWR 基站 (JSON)     UWB 数据                 │
│  UART7 ←── OpenMV 摄像头 (AA..BB 帧)  视觉检测数据       │
│  D9   ←── SW2 拨码开关          紧急退出                 │
│  C8   ←── KEY1                  切换 UWB 模式            │
│  C9   ←── KEY2                  切换视觉追踪             │
│  C14  ←── KEY3                  切换停车                 │
│  C4   ──→ LED                   状态指示                 │
│                                                         │
│  SPI  ←── IMU660RX              姿态/角速度             │
│  PWM×4 ──→ TB6612 ×4           电机驱动                 │
│  ENC×4 ←── 编码器 ×4            轮速反馈                 │
└─────────────────────────────────────────────────────────┘
```

---

## 3. 软件架构

### 3.1 分层架构

```
┌──────────────────────────────────────────┐
│  应用层 (main.py)                         │
│  - 状态机 (UWB → VISUAL → STOPPED)       │
│  - 模式管理 (enter/exit 闭包)             │
│  - 按键扫描 + SW2 消抖轮询                │
│  - 摄像头环形缓冲区 + 滑动窗口判定         │
├──────────────────────────────────────────┤
│  控制层                                   │
│  - uwb_tracker.py (UWBFollower 类)       │
│  - cam_follow.py (级联 PID + 状态机)      │
├──────────────────────────────────────────┤
│  信号处理层                               │
│  - imu_motion.py (互补滤波 + 角速度闭环)   │
│  - pid.py (PID 基类含积分抗饱和)          │
│  - cam_data.py (摄像头数据接收与解析)     │
├──────────────────────────────────────────┤
│  硬件抽象层                               │
│  - motor.py (PWM/编码器/闭环驱动)         │
│  - key.py (按键消抖 + 独立看门狗)          │
│  - ticker.py (PIT 定时器封装)             │
│  - utils.py (角度归一化/限幅)             │
└──────────────────────────────────────────┘
```

### 3.2 模块职责表

| 文件 | 类/函数 | 职责 |
|---|---|---|
| `main.py` | `main()`, `_create_mode_manager()` | 主入口、状态机、模式切换、按键分发 |
| `main.py` | `CamRingBuffer` | 环形缓冲区（256 字节），处理 UART 粘包/断包 |
| `main.py` | `parse_camera_frame()` | 解析 AA..BB 10 字节帧（旧 LED 协议） |
| `main.py` | `check_sw2()` | SW2 消抖检测（50ms 时间窗口） |
| `uwb_tracker.py` | `UWBFollower` | UWB 跟随完整控制器（JSON 解析 + 滤波 + 航向纠偏 + 速度斜坡） |
| `cam_data.py` | `CamDataReceiver` | 摄像头数据接收器（字节缓冲 + AA..BB 帧解析） |
| `cam_data.py` | `x_to_cm()` | 横向偏移 mm → cm 转换（÷10） |
| `cam_data.py` | `y_to_distance()` | 纵向坐标 → 实际距离转换（29.0 − y） |
| `cam_follow.py` | `CascadePID` | 级联 PID 控制器（位置 PID + 加速度限幅 + 速度滤波） |
| `cam_follow.py` | `speed_filter()` | 一阶低通速度滤波（方向变化时自动复位） |
| `motor.py` | `omni_drive_closed_loop()` | 四轮全向闭环驱动（前馈+PI 反馈） |
| `motor.py` | `stop_all()` | 急停 |
| `imu_motion.py` | `update_angle()` | 互补滤波姿态解算 |
| `imu_motion.py` | `angular_velocity_control()` | 角速度 PID 闭环 |
| `key.py` | `capture()` | 驱动 KEY_HANDLER 消抖状态机 |
| `key.py` | `key_triggered(index)` | 检测按键触发（短按松发，自动清零） |
| `key.py` | `pet_watchdog()` | 喂狗（重置看门狗计数器） |
| `pid.py` | `PID` | 增量式 PID（含积分抗饱和、输出限幅、微分低通） |
| `ticker.py` | `ticker` | PIT 定时器封装 |

---

## 4. 运行流程详解

### 4.1 启动序列

```
main() 启动
  │
  ├─ 1. 硬件初始化
  │     ├─ LED = C4 (输出, 灭)
  │     ├─ SW2 = D9 (上拉输入)
  │     ├─ KEY1(C8), KEY2(C9), KEY3(C14) 由 key.py 管理
  │     └─ SW2 消抖状态初始化 (SW2_DEBOUNCE_MS=50ms)
  │
  ├─ 2. 创建模式管理器（闭包）
  │     └─ _create_mode_manager()
  │         ├─ 创建 CamRingBuffer (256 字节)
  │         ├─ 定义 enter_uwb()  / exit_uwb()
  │         ├─ 定义 enter_visual() / exit_visual()
  │         └─ 定义 enter_stopped()
  │
  ├─ 3. enter_uwb() — 默认 UWB 跟随
  │     ├─ 创建 UWBFollower(UART0, 115200, target="8834")
  │     │   ├─ UART0 初始化
  │     │   ├─ 停止 enc_ticker → 手动接管编码器
  │     │   ├─ IMU 热身 10 帧
  │     │   └─ 滤波状态重置
  │     │
  │     └─ UART7 惰性创建 (摄像头数据通道)
  │
  ├─ 4. 进入主循环 (10ms 周期)
  │
  └─ ...
```

### 4.2 主循环伪代码

```python
while True:
    # ─── ① 看门狗 + 按键扫描 ───
    capture()             # 驱动按键消抖状态机
    pet_watchdog()        # 喂狗（3s 超时 → 硬复位）

    # ─── ② 按键模式切换 ───
    if key_triggered(1):  # KEY1(C8) → UWB (LED 常亮)
        exit_current(); enter_uwb(); state = STATE_UWB
    if key_triggered(2):  # KEY2(C9) → VISUAL (LED 快闪)
        exit_current(); enter_visual(); state = STATE_VISUAL
    if key_triggered(3):  # KEY3(C14) → STOPPED (LED 慢闪)
        exit_current(); enter_stopped(); state = STATE_STOPPED

    # ─── ③ SW2 检测（每次迭代）───
    if check_sw2():
        cleanup(); break  # 强制退出

    # ═══════════════════════════════════
    #  状态 ① — UWB 跟随（默认）
    # ═══════════════════════════════════
    if state == STATE_UWB:
        uwb.step()              # UWB 跟随一步
        if loop_cnt % 5 == 0:   # 每 ~50ms 轮询一次摄像头
            poll_camera()        # 滑动窗口判定
            if hits >= WINDOW_THRESHOLD:
                exit_uwb(); enter_visual()
                state = STATE_VISUAL

    # ═══════════════════════════════════
    #  状态 ② — 视觉追踪
    # ═══════════════════════════════════
    elif state == STATE_VISUAL:
        data = recv.read()             # CamDataReceiver 非阻塞读取
        if data and data['is_target']:
            # 坐标转换 → 误差计算
            x_cm = x_to_cm(data['x'])
            dist = y_to_distance(data['y'])
            x_error = x_cm
            y_error = dist - TARGET_DIST_CM
            # 级联 PID → 滤波 → 驱动
            cmd_fwd = PID_FWD.compute(y_error, DT)  # 外环位置 PID + 加速度限幅
            cmd_lat = PID_LAT.compute(x_error, DT)
            cmd_fwd, cmd_lat = speed_filter(cmd_fwd, cmd_lat)  # 速度低通滤波
            # 到达判定 → 停车
            if abs(y_error) < STOP_DIST_CM and abs(x_cm) < 10:
                exit_visual(); enter_stopped()
                state = STATE_STOPPED
            else:
                omni_drive_closed_loop(cmd_fwd, cmd_lat, 0, actual_speeds, DT)
        elif timeout > 500ms:
            exit_visual(); enter_uwb(); state = STATE_UWB  # 退回 UWB

    # ═══════════════════════════════════
    #  状态 ③ — 停车
    # ═══════════════════════════════════
    elif state == STATE_STOPPED:
        led.toggle()  # 慢闪 ~1Hz

    time.sleep_ms(10)
```

### 4.3 完整状态流转图

```
                    ┌──────────┐
         启动 ──────→│ UWB 跟随  │←──────────────┐
                    │ (默认)    │                │
                    └─────┬─────┘                │
                          │                      │
         KEY2 或 摄像头检测到物品                   │
                          │                      │
                          ↓                      │
                    ┌──────────┐   超时 500ms     │
                    │ 视觉追踪  │────────────┘
                    │ (级联PID) │   或丢失目标
                    └─────┬─────┘
                          │
             距离 30cm±5cm 且 居中
                          │
                          ↓
                    ┌──────────┐
                    │  停车     │
                    │ (STOPPED) │
                    └──────────┘
                          │
     KEY1 → UWB            KEY3 → STOPPED          SW2 拨动
          ↓                     ↓                     ↓
    回到 UWB 跟随         手动停车              程序结束
```

---

## 5. UWB 跟随子系统

### 5.1 入口文件

实际入口为 `main.py`，内部通过 `UWBFollower` 类（`uwb_tracker.py`）实现。

### 5.2 UWBFollower 类结构

```python
class UWBFollower:
    # ── 状态常量 ──
    STATE_FOLLOW  = 0   # 跟随态：边走边纠偏
    STATE_STOPPED = 1   # 停止态：到达停车距离

    # ── 运动参数 ──
    APPROACH_SPEED     = 0.60   # 全速逼近 (m/s)
    STOP_DIST_M        = 0.18   # 停车距离 (m)
    RESTART_DIST_M     = 0.30   # 重新跟随距离 (m)
    FULL_SPEED_DIST_M  = 0.35   # 全速区间起点 (m)

    # ── 滤波系数 ──
    D_FILT_ALPHA    = 0.15   # 距离低通
    XY_FILT_ALPHA   = 0.10   # 坐标低通
    ANGLE_FILT_ALPHA = 0.10  # 角度低通

    # ── 航向纠偏 ──
    ROT_KP       = 2.0     # 偏角→目标角速度增益
    ROT_DEADBAND = 3.0     # 到位死区 (度)
    ROT_MAX_RATE = 200     # 最大旋转速度 (dps)
    ROT_MIN_RATE = 25      # 最小旋转速度 (dps)

    # ── 超时 ──
    TIMEOUT_MS = 800       # UWB 数据超时
```

### 5.3 UWB 跟随控制流程

```
UART0 接收 TWR 数据
      │
      ▼
JSON 解析 → 提取 (anchor, D, Xcm, Ycm)
      │
      ▼
低通滤波 → 坐标 (x_filt, y_filt) + 距离 (d_filt) + 角度 (angle_filt)
      │
      ▼
速度斜坡 → _ramp_speed(dist) — 线性衰减：FULL_SPEED_DIST→APPROACH_SPEED, STOP→0
      │
      ▼
航向偏差 → yaw_err = target_yaw − imu.yaw（提取 cos 对齐分量）
      │
      ▼
角速度闭环 → angular_velocity_control(target_dps, actual_dps)
      │
      ▼
编码器闭环 → omni_drive_closed_loop(speed, 0, wz, actual_speeds)
```

### 5.4 速度斜坡策略

```
speed
0.60 │████████████████████████
     │                    ╲
     │                     ╲
     │                      ╲
     │                       ╲
0.00 │                        ╳══════
     └──────┬────────┬────────┬────────→ dist
          STOP     RESTART  FULL
         0.18m     0.30m    0.35m
```

---

## 6. 摄像头检测与切换子系统

### 6.1 摄像头帧协议（cam_data.py）

OpenMV 摄像头通过 UART7 发送固定的 10 字节数据帧，由 `cam_data.py` 的 `CamDataReceiver` 负责接收与解析：

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│  AA  │ X_H  │ X_L  │ Y_H  │ Y_L  │ FLAG │  ID  │  B6  │  B7  │  BB  │
│(0xAA)│      │      │      │      │      │      │      │      │(0xBB)│
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
│      │← int16 大端序 →│← int16 大端序 →│ 标志 │ 标识 │← 附加 →│      │
│      帧头   X (mm)×10    Y (cm)×10     FLAG   ID    B6 B7   帧尾  │
│                                                                     │
│  X 字段：横向偏移，单位 mm，÷10 恢复 mm，再 ×0.1 → cm              │
│  Y 字段：纵向坐标，单位 cm，Y=0 对应实际距离 29cm（参考原点）        │
│           Y>0 表示更近，Y<0 表示更远                                 │
│  FLAG：0x02 或 0x03 = 检测到目标，0x00 = 目标丢失                   │
│  ID：  目标标识符（单字节）                                          │
│  B6/B7：附加数据字段（待定）                                         │
│                                                                     │
│  接收端转换：                                                        │
│    x_cm = x_to_cm(x)           # mm → cm (÷10)                     │
│    dist = y_to_distance(y)     # y → 实际距离 = 29.0 − y            │
│    is_target = (flag != 0) and not (x==0 and y==0)                  │
└─────────────────────────────────────────────────────────────────────┘
```

### 6.2 数据接收器 (CamDataReceiver)

`cam_data.py` 中的 `CamDataReceiver` 类负责 UART7 数据的接收与帧解析：

```python
class CamDataReceiver:
    # 初始化 UART7 (115200 bps)
    recv = CamDataReceiver(uart_id=7)

    # 内部使用动态 bytearray 缓冲 → find(0xAA) → 校验 0xBB 帧尾
    # 非阻塞读取返回完整帧 dict 或 None
    data = recv.read()
    # data = {'x': -12.5, 'y': 15.0, 'flag': 0x02, 'id': 1, 'is_target': True}

    # 阻塞读取（含超时）
    data = recv.read_block(timeout_ms=100)
```

**工作原理**：
1. 调用 `read()` 时，先从 `UART.read()` 读取原始字节追加到内部 `bytearray` 缓冲区
2. 在缓冲区中查找 `0xAA` 帧头，校验第 9 字节是否为 `0xBB`
3. 假帧头（第 9 字节 ≠ 0xBB）自动跳过，继续搜索
4. 解析通过 `_parse_frame()` 完成：提取 X/Y int16（大端序，有符号）、FLAG、ID、B6、B7
5. 返回标准字典 `{x, y, flag, id, b6, b7, is_target}`，或 `None`（无完整帧）

**内置统计**：
- `frame_count`：总接收帧数
- `target_count`：检测到目标的帧数
- `lost_count`：目标丢失的帧数
- `error_count`：无效帧尾的假帧数

### 6.3 滑动窗口判定机制

为避免摄像头单帧误检导致错误切换，`main.py` 采用滑动窗口确认：

```python
WINDOW_SIZE      = 8    # 窗口大小（帧数）
WINDOW_THRESHOLD = 5    # 确认阈值（≥5 帧有效才切换）
```

**判定逻辑**：
```
窗口帧序列:  [0, 1, 0, 1, 1, 1, 1, 1]
              ↑               ↑
            历史帧          当前帧
                     有效帧数 = 5 ≥ 阈值5 → 触发切换
```

**有效帧条件**（`is_target` 字段，源自 `cam_data.py`）：
- `FLAG != 0x00`（0x02 或 0x03，表示摄像头检测到目标）
- 且非全零帧（`x ≠ 0` 或 `y ≠ 0`，排除无效数据）

### 6.4 模式切换时序

```
UWB 跟随运行中...
  │
  ├─ 每 50ms 读取 UART7
  │      │
  │      ▼
  │   解析帧 + 滑动窗口判定
  │      │
  │      ├─ hits < 5 → 继续 UWB 跟随
  │      │
  │      └─ hits ≥ 5 → [CAM] Target confirmed! 5/8
  │                         │
  │              ┌──────────┘
  │              ▼
  │         exit_uwb()
  │           ├─ uwb.stop() — 停 UART0、停电机
  │           ├─ enc_ticker.stop() — 暂停编码器自动采集
  │           ├─ 清空编码器缓冲区 (×5)
  │           ├─ reset_encoder_filter()
  │           ├─ reset_wheel_pi()
  │           └─ reset_ang_vel_pid()
  │              │
  │              ▼
  │         enter_visual()
  │           ├─ IMU 预热 (×5 帧，从 PIT3 ticker 缓冲区读取)
  │           ├─ CamDataReceiver 已由 enter_uwb() 惰性创建
  │           ├─ 编码器接管（手动管理，enc_ticker 已停止）
  │           └─ 重置滑动窗口 + 计时器 + 状态标志
  │              │
  │              ▼
  │         state = STATE_VISUAL
  │         LED 快闪 (~2.5Hz)
  │
  └─ 进入视觉追踪主循环
```

---

## 7. 视觉追踪子系统

视觉追踪由 `cam_follow.py` 实现，采用**级联 PID 控制 + 三态状态机**架构。

### 7.1 级联控制架构

控制系统分为三层：

```
┌─────────────────────────────────────┐
│  外环：位置 PID (CascadePID.compute)  │
│  PID_FWD (前进/后退)                 │
│    kp=0.012, ki=0.003, kd=0.005     │
│  PID_LAT (横向)                      │
│    kp=0.010, ki=0.002, kd=0.006     │
│  输入: 位置误差 (cm)                  │
│  输出: 目标速度 (m/s)                 │
│  ← 积分抗饱和 + 微分低通              │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  中环：速度限幅 + 低通滤波             │
│  - 加速度限幅 (ACCEL_LIMIT=3.0 m/s²) │
│  - speed_filter() 一阶低通 (α=0.8)   │
│  - 方向变化检测 → 滤波器自动复位       │
│  - 最低速度防死区 (MIN_SPEED=0.10)    │
│  输出: 平滑速度指令 (m/s)             │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  内环：编码器速度闭环 (motor.py)       │
│  omni_drive_closed_loop()            │
│  前馈 + PI 反馈 → PWM 输出           │
│  输出: 4 路电机 PWM                   │
└─────────────────────────────────────┘
```

### 7.2 CascadePID 类

`cam_follow.py` 定义了 `CascadePID` 类，在 `pid.PID` 基础上封装加速度限幅：

```python
class CascadePID:
    def __init__(self, kp, ki, kd, out_limit, accel_limit):
        self.pid = PID(kp=kp, ki=ki, kd=kd,
                       integral_limit=50, output_limit=out_limit)
        self.accel_limit = accel_limit
        self.prev_output = 0.0

    def compute(self, error, dt):
        # 1. 位置 PID → 目标速度
        target_speed = self.pid.compute(0, error, dt)
        # 2. 加速度限幅（平滑速度变化）
        delta = target_speed - self.prev_output
        max_delta = self.accel_limit * dt
        if abs(delta) > max_delta:
            delta = max_delta if delta > 0 else -max_delta
        output = self.prev_output + delta
        self.prev_output = output
        return output
```

**设计要点**：
- `setpoint=0`，`measurement=error`：PID 内部 `error = setpoint − measurement = −error`，与 `y_to_distance()` 坐标映射形成双重反转恰好抵消。此符号已配合摄像头坐标系验证，不可随意修改
- 加速度限幅保证速度变化平滑，避免急加速/急减速
- 积分抗饱和（`integral_limit=50`）+ 微分低通（`d_filter_alpha=0.6`）继承自 `pid.PID`

### 7.3 实例化参数

```python
# 前进/后退方向
PID_FWD = CascadePID(
    kp=0.012, ki=0.003, kd=0.005,
    out_limit=MAX_SPEED_FWD,   # 0.50 m/s
    accel_limit=ACCEL_LIMIT    # 3.0 m/s²
)

# 横向方向
PID_LAT = CascadePID(
    kp=0.010, ki=0.002, kd=0.006,
    out_limit=MAX_SPEED_LAT,   # 0.45 m/s
    accel_limit=ACCEL_LIMIT    # 3.0 m/s²
)
```

### 7.4 速度滤波

`speed_filter()` 对 PID 输出的原始速度进行一阶低通滤波，并检测方向变化时自动复位：

```python
SPEED_FILTER_ALPHA = 0.8

def speed_filter(fwd, lat):
    global filt_fwd, filt_lat, filt_fwd_sign, filt_lat_sign

    # 检测方向变化 → 复位滤波器，避免锁死
    new_sign = 1 if fwd > 0 else (-1 if fwd < 0 else 0)
    if new_sign != 0 and new_sign != filt_fwd_sign:
        filt_fwd = fwd       # 复位：不继承旧值
        filt_fwd_sign = new_sign
    else:
        filt_fwd = 0.8 * fwd + 0.2 * filt_fwd  # 一阶低通

    # 同理处理横向
    ...

    return filt_fwd, filt_lat
```

**为什么需要方向检测**：当小车从前进变为后退时，如果滤波值仍保留正向速度，会导致短时间的"反向拖拽"，方向变化检测可立即清除此残留。

### 7.5 三态状态机

```
                    ┌──────────┐
         启动 ──────→│  LOST    │←─────────────────────┐
                    │ (停车等待) │                      │
                    └─────┬─────┘                      │
                          │ 检测到目标 (is_target)       │
                          ▼                             │
                    ┌──────────┐   到达 30cm±5cm        │
                    │ FOLLOW   │─────────────┐          │
                    │ (级联PID) │              │          │
                    └─────┬─────┘              ▼          │
                          │ 目标丢失      ┌──────────┐    │
                          │ > 500ms       │ STOPPED  │    │
                          └─────────────→│ (停车等待) │    │
                                         └─────┬─────┘    │
                                               │ 目标移开  │
                                               │ > 10cm    │
                                               └───────────┘
```

| 状态 | 常量 | 行为 | 触发条件 |
|---|---|---|---|
| **FOLLOW** | `STATE_FOLLOW` | 级联 PID 控制跟随 | 检测到目标（`is_target`） |
| **STOPPED** | `STATE_STOPPED` | 停车等待 | `abs(y_error) < 5cm` 且 `abs(x_cm) < 10cm` |
| **LOST** | `STATE_LOST` | 停车等待重新捕获 | `is_target == False` 持续 ≥ 500ms |

**状态切换动作**：
- `LOST → FOLLOW`：重置 `reset_wheel_pi()`、`PID_FWD.reset()`、`PID_LAT.reset()`、速度滤波状态
- `FOLLOW → STOPPED`：`stop_all()`
- `STOPPED → FOLLOW`：目标移开超 2× 容差时恢复，同样重置所有状态
- `FOLLOW → LOST`：`stop_all()`

### 7.6 坐标转换与误差计算

```
CamDataReceiver.read()
        │
        ▼
  data = {'x': mm, 'y': cm, 'is_target': bool, ...}
        │
        ▼
  ┌──────────────────────────────────┐
  │ x_cm = x_to_cm(data['x'])       │  ← 横向偏移 mm → cm (÷10)
  │ actual_dist = y_to_distance(y)   │  ← 纵向 → 实际距离 = 29.0 − y
  │                                  │
  │ x_error = x_cm                    │  ← 横向误差（左负右正）
  │ y_error = actual_dist − TARGET   │  ← 距离误差（正=太远，负=太近）
  └──────────────────────────────────┘
```

**坐标系统约定**：
- X > 0 → 目标在右侧，需向右平移（横向速度 +lat）
- Y 坐标：Y=0 → 实际距离 29cm；Y>0 → 更近
- 目标距离 30cm 时：`y = 29.0 − 30.0 = −1.0`

### 7.7 停车判定与自动恢复

```python
TARGET_DIST_CM  = 30.0   # 目标跟随距离 (cm)
STOP_DIST_CM    = 5.0    # 到达判定容差 (cm)

# FOLLOW → STOPPED: 距离误差在容差内 且 横向居中
if abs(y_error) < STOP_DIST_CM and abs(x_cm) < 10:
    state = STATE_STOPPED
    stop_all()

# STOPPED → FOLLOW: 目标移开超过 2 倍容差 (10cm)
if abs(actual_dist - TARGET_DIST_CM) > STOP_DIST_CM * 2:
    state = STATE_FOLLOW
    reset_all_states()
```

### 7.8 目标丢失与超时

```python
LOST_TIMEOUT_MS = 500  # 丢失超时 (ms)

# FOLLOW → LOST: 连续 500ms 无有效目标
if not is_target:
    if time.ticks_diff(now, last_target_ms) > LOST_TIMEOUT_MS:
        state = STATE_LOST
        stop_all()

# LOST → FOLLOW: 重新检测到目标 → 恢复跟随
if data['is_target']:
    state = STATE_FOLLOW
    reset_all_states()
```

在 `main.py` 的上下文中，视觉追踪丢失还会触发 500ms 超时回退到 UWB 模式。

### 7.9 调试输出

每 300ms 打印一行控制状态（`cam_follow.py` 独立运行时）：

```
[#0123 FOLLOW] X: +5.2cm dist:32.3cm fwd:+0.123 lat:+0.045
[#0456 STOP]   X: +1.1cm dist:29.8cm fwd:+0.000 lat:+0.000
[#0789 LOST]   X: +0.0cm dist:29.0cm fwd:+0.000 lat:+0.000
```

输出字段：帧计数器、当前状态、横向偏移(cm)、实际距离(cm)、前进速度(m/s)、横向速度(m/s)。

---

## 8. 按键控制与 SW2 退出

### 8.1 按键定义

| 按键 | 引脚 | 功能 | LED 状态 |
|---|---|---|---|
| **KEY1** | C8 | 切换到 UWB 跟随模式 | 常亮 |
| **KEY2** | C9 | 切换到视觉追踪模式 | 快闪 (~2.5Hz) |
| **KEY3** | C14 | 切换到停车状态 | 慢闪 (~1Hz) |
| **SW2** | D9 | 强制退出程序 | — |

### 8.2 按键驱动（key.py）

`key.py` 基于 seekfree 库的 `KEY_HANDLER`，由主循环每 10ms 调用 `capture()` 驱动消抖状态机：

```python
from key import capture, key_triggered, pet_watchdog

# 主循环中每帧调用：
capture()              # 驱动消抖状态机
pet_watchdog()         # 喂狗

# 检测按键触发（短按松发，自动清零）：
if key_triggered(1):   # KEY1(C8) 被按下
    ...
```

**key_triggered(index)** 内部逻辑：
1. 读取 `KEY_HANDLER.get()` 获取按键状态
2. 若 `state[index−1]` 为 True → 调用 `key.clear(index)` 清除 → 返回 True
3. 否则返回 False

### 8.3 硬件看门狗

```python
# PIT2 独占，10ms 周期
_WWDG_THRESHOLD = 300   # 300 × 10ms = 3 秒

# 若主循环 3 秒内未调用 pet_watchdog() → machine.reset() 硬复位
```

### 8.4 SW2 消抖退出

`main.py` 使用时间窗口消抖（50ms），不依赖 `key.py`：

```python
SW2_DEBOUNCE_MS = 50

def check_sw2():
    val = sw2.value()
    if val != _sw2_last:
        _sw2_last = val
        _sw2_changed = True
        _sw2_stable_start = time.ticks_ms()
    if _sw2_changed and time.ticks_diff(now, _sw2_stable_start) >= SW2_DEBOUNCE_MS:
        _sw2_changed = False
        return True  # 确认切换
    return False
```

**退出流程**：
```
SW2 拨动触发 → check_sw2() → True → break 主循环
    │
    ▼
finally 清理块:
  ├─ stop_all()                 — 所有电机急停
  ├─ uwb.stop()                 — UART0 deinit + enc_ticker 重启
  ├─ cam_uart.deinit()          — UART7 释放
  ├─ pause_encoder_ticker()     — PIT1 停止 (防 ISR 干扰 REPL)
  └─ time.sleep_ms(50)          — 等待外设稳定
```

---

## 9. 状态机完整定义

### 9.1 状态枚举

| 状态常量 | 值 | 含义 | LED 状态 |
|---|---|---|---|
| `STATE_UWB` | 0 | UWB 锚点跟随 | 常亮 |
| `STATE_VISUAL` | 1 | 摄像头视觉追踪（级联 PID） | 快闪 (~2.5Hz) |
| `STATE_STOPPED` | 2 | 到达目标停车 | 慢闪 (~1Hz) |

### 9.2 状态转换条件

| 当前状态 | 条件 | 目标状态 | 触发动作 |
|---|---|---|---|
| UWB | 摄像头滑动窗口 ≥5/8 确认 | VISUAL | `exit_uwb()` → `enter_visual()` |
| UWB | KEY2 按下 | VISUAL | `exit_uwb()` → `enter_visual()` |
| VISUAL | 距离误差 < 5cm 且 横向偏移 < 10cm | STOPPED | `exit_visual()` → `enter_stopped()` |
| VISUAL | KEY3 按下 | STOPPED | `exit_visual()` → `enter_stopped()` |
| VISUAL | 连续 500ms 无数据 | UWB | `exit_visual()` → `enter_uwb()` |
| VISUAL | KEY1 按下 | UWB | `exit_visual()` → `enter_uwb()` |
| STOPPED | KEY1 按下 | UWB | `enter_uwb()` |
| STOPPED | KEY2 按下 | VISUAL | `enter_visual()` |
| 任意 | SW2 拨动 | 程序退出 | `finally` 清理块 |

### 9.3 模式切换资源管理

| 操作 | UWB→VISUAL | VISUAL→UWB | VISUAL→STOPPED |
|---|---|---|---|
| UART0 释放 | ✓ | 重新初始化 | — |
| 编码器接管 | 停 ticker → 手动管理 | 停 ticker → 重启 ticker | — |
| 编码器清空 | ✓ (×5 次空读) | — | — |
| 滤波器重置 | ✓ | — | — |
| PID 重置 | ✓ (wheel PI + ang vel) | — | ✓ (stop_all) |
| IMU 预热 | ✓ (×5 帧) | — | — |
| 滑动窗口重置 | ✓ | ✓ (超时时) | — |
| 环形缓冲区清空 | ✓ | — | — |

---

## 10. 帧协议规范

### 10.1 OpenMV → RT1021 帧协议（cam_data.py）

```
字节偏移  | 0    | 1    | 2    | 3    | 4    | 5    | 6    | 7    | 8    | 9
内容     | 0xAA | X_H  | X_L  | Y_H  | Y_L  | FLAG |  ID  |  B6  |  B7  | 0xBB
数据类型  | 帧头  |      int16      |      int16      | byte | byte | byte | byte | 帧尾
字段含义  |       X 横向偏移(mm)×10  | Y 纵向坐标(cm)×10  | 检测标志 | 目标ID | 附加1 | 附加2 |
```

**解码规则**：
- 所有数值为大端序 int16（X、Y 字段）
- 接收端做有符号转换：`val if val < 32768 else val − 65536`
- 除以 10 恢复实际值（精度 0.1 mm / 0.1 cm）
- **FLAG**：`0x02` / `0x03` = 检测到目标，`0x00` = 目标丢失
- **ID**：目标标识符（单字节）
- **B6 / B7**：附加数据字段（待定用途）

**坐标转换**（`cam_data.py`）：

| 函数 | 输入 | 输出 | 公式 |
|---|---|---|---|
| `x_to_cm(x)` | X 原始值（mm） | 横向偏移（cm） | `x / 10.0` |
| `y_to_distance(y)` | Y 原始值（cm） | 实际距离（cm） | `29.0 − y` |

**有效目标判定**（`is_target`）：
- `FLAG != 0x00` 且 非全零帧（x ≠ 0 或 y ≠ 0）

### 10.2 UWB TWR → RT1021 协议

```
UART0, 115200 bps, JSON 行协议

示例帧：
{"TWR":{"a16":"8834","D":150,"Xcm":20,"Ycm":148}}
         ↑        ↑      ↑      ↑       ↑
      对象类型  锚点ID  距离cm  X坐标cm  Y坐标cm
```

| 字段 | 含义 | 单位 |
|---|---|---|
| `a16` | 锚点 ID（目标="8834"） | 字符串 |
| `D` | 直线距离（一维） | cm |
| `Xcm` | 横向相对坐标 | cm |
| `Ycm` | 纵向相对坐标 | cm |

---

## 11. 关键参数速查表

### 11.1 UWB 跟随参数

| 参数 | 值 | 说明 |
|---|---|---|
| `APPROACH_SPEED` | 0.60 m/s | 全速逼近速度 |
| `STOP_DIST_M` | 0.18 m | 停车触发距离 |
| `RESTART_DIST_M` | 0.30 m | 重新跟随触发距离 |
| `FULL_SPEED_DIST_M` | 0.35 m | 全速区间起点 |
| `D_FILT_ALPHA` | 0.15 | 距离低通滤波系数 |
| `XY_FILT_ALPHA` | 0.10 | 坐标低通滤波系数 |
| `ANGLE_FILT_ALPHA` | 0.10 | 角度低通滤波系数 |
| `ROT_KP` | 2.0 | 航向偏差→角速度增益 |
| `ROT_DEADBAND` | 3.0° | 航向到位死区 |
| `ROT_MAX_RATE` | 200 dps | 最大旋转速度 |
| `ROT_MIN_RATE` | 25 dps | 最小旋转速度（防死区） |
| `TIMEOUT_MS` | 800 ms | UWB 数据超时 |

### 11.2 视觉追踪参数

| 参数 | 值 | 说明 |
|---|---|---|
| `TARGET_DIST_CM` | 30.0 cm | 目标跟随距离 |
| `STOP_DIST_CM` | 5.0 cm | 到达判定容差（距离误差 < 此值 + 横向偏移 < 10cm → 停车） |
| `LOST_TIMEOUT_MS` | 500 ms | 目标丢失超时（连续无有效帧 → LOST 状态） |
| `CAM_TIMEOUT_MS` | 500 ms | 视觉追踪数据超时（main.py 层，退回 UWB 模式） |
| `WINDOW_SIZE` | 8 | 滑动窗口大小（UWB→VISUAL 切换判定） |
| `WINDOW_THRESHOLD` | 5 | 窗口确认阈值（≥5 帧有效才切换） |
| `MAX_SPEED_FWD` | 0.50 m/s | 最大前进速度（CascadePID 输出限幅） |
| `MAX_SPEED_LAT` | 0.45 m/s | 最大横向速度（CascadePID 输出限幅） |
| `MIN_SPEED` | 0.10 m/s | 最低速度（防电机死区） |
| `ACCEL_LIMIT` | 3.0 m/s² | 加速度限制（速度变化斜率上限） |
| `SPEED_FILTER_ALPHA` | 0.8 | 速度一阶低通滤波系数 |
| `DT` | 0.02 s | 控制周期（20ms） |
| `Y_REF_DISTANCE` | 29.0 cm | Y 坐标参考原点距离（Y=0 时对应实际距离） |

### 11.3 PID 增益

**视觉追踪级联 PID**（`cam_follow.py`，基于 `pid.PID` + `CascadePID`）：

| PID | 用途 | KP | KI | KD | 积分限幅 | 输出限幅 | 加速度限幅 |
|---|---|---|---|---|---|---|---|
| **PID_FWD** | 前进/后退 | 0.012 | 0.003 | 0.004 | ±50 | 0.50 m/s | 3.0 m/s² |
| **PID_LAT** | 横向平移 | 0.010 | 0.002 | 0.003 | ±50 | 0.45 m/s | 3.0 m/s² |

**UWB 跟随 PID / PI**（`uwb_tracker.py` / `imu_motion.py` / `motor.py`）：

| PID | KP | KI | KD | 输出限幅 | 积分限幅 |
|---|---|---|---|---|---|
| 角速度 PID (imu_motion) | 0.6 | 0.6 | 0 | ±300 dps | ±150 |
| 轮子 PI ×4 (motor) | 40000 | 0 | 0 | ±50000 PWM | ±5000 |

---

## 12. 部署与调试

### 12.1 部署步骤

1. **烧录 MicroPython 固件** 到 RT1021 开发板
2. **将 `user/` 目录下所有 `.py` 文件** 复制到板载文件系统
3. **连接硬件**：
   - OpenMV 摄像头 → UART7
   - UWB TWR 基站 → UART0
   - 确认 SW2 拨码开关连接 D9
   - 确认 KEY1(C8)、KEY2(C9)、KEY3(C14) 连接
4. **上电启动**，RT1021 自动执行 `main.py`，默认进入 UWB 跟随模式（LED 常亮）

### 12.2 日志输出解读

```
=== UWBFollower ready (UART0 115200 baud, anchor=8834) ===
>>> MODE: UWB_FOLLOW <<<
RT1021 — UWB Follow + Visual Track
  UART0 : UWB 基站数据 (115200)
  UART7 : 摄像头检测数据 (115200)
  KEY1/2/3 : UWB / VISUAL / STOP
  SW2   : 强制退出

[1] a=8834 D=150 Df=148.3 X=20 Y=148 ang=+8° ang_f=+7° spd=0.60 state=0
          ↑              ↑                ↑        ↑            ↑       ↑
       帧计数器     滤波后距离       滤波后角度    速度          状态:0=跟随

[KEY2] → VISUAL_TRACK                           ← 按键切换或摄像头检测到物品
>>> MODE: VISUAL_TRACK <<<
[#0123 FOLLOW] X:+5.2cm dist:32.3cm fwd:+0.123 lat:+0.045  ← 级联 PID 控制中
[#0456 STOP]   X:+1.1cm dist:29.8cm fwd:+0.000 lat:+0.000  ← 到达 30cm±5cm
>>> MODE: STOPPED (target reached) <<<
[KEY1] → UWB_FOLLOW                              ← 按键切回 UWB
>>> MODE: UWB_FOLLOW <<<

[SW2] Exit requested                             ← SW2 强制退出
Robot stopped. (you may now re-run or enter REPL)
```

### 12.3 诊断工具

| 文件 | 用途 |
|---|---|
| `cam_raw_recv.py` | 独立运行，解析并显示 UART7 摄像头原始帧 |
| `cam_diag.py` | 独立运行，dump UART7 所有原始字节（HEX + ASCII） |
| `cam_diag_v2.py` | 第二代摄像头诊断工具 |
| `cam_demo.py` | OpenMV 端程序（在 OpenMV 上运行，非 RT1021） |
| `cam_follow.py` | ★ 视觉追踪独立运行版（级联 PID + 状态机，可脱离 main.py 单独测试） |
| `uwb_following.py` | UWB 跟随独立运行版（可脱离 main.py 单独测试） |
| `test_push.py` | 推杆控制测试 |
| `test_p_rotate.py` | 旋转控制测试 |

### 12.4 常见问题排查

| 现象 | 可能原因 | 排查方法 |
|---|---|---|
| 小车不动 | 电机驱动未初始化 | 检查 PWM 引脚连接 |
| UWB 无数据 | 基站未上电/ID 不匹配 | 运行 `uwb_following.py` 独立测试 |
| 摄像头无数据 | 波特率/接线错误 | 运行 `cam_diag.py` 查看原始字节 |
| 误切换视觉模式 | 滑动窗口阈值过低 | 调高 `WINDOW_THRESHOLD`（当前=5/8） |
| 视觉追踪抖动 | PID 增益过大或加速度限幅过松 | 降低 `PID_FWD.kp` / `PID_LAT.kp` 或减小 `ACCEL_LIMIT` |
| 视觉追踪反应慢 | 速度滤波过强 | 增大 `SPEED_FILTER_ALPHA`（当前=0.8） |
| 停车距离不准 | 摄像头标定偏差 | 检查 TARGET_DIST_CM 和 STOP_DIST_CM 设置 |
| LED 不亮 | LED 引脚冲突 | 检查 C4 是否被其他模块占用 |
| 按键无响应 | key.py 未正确初始化 | 确认主循环中调用了 `capture()` |
| 3 秒后自动复位 | 看门狗超时 | 检查主循环是否卡死，确认 `pet_watchdog()` 被调用 |

---

## 附录：文件清单

```
user/
├── main.py              # ★ 主入口，状态机核心 + 按键分发
├── uwb_tracker.py       # ★ UWB 跟随控制器 (类 UWBFollower)
├── uwb_following.py     # UWB 跟随独立运行版
├── cam_data.py          # ★ 摄像头数据接收与协议解析 (CamDataReceiver)
├── cam_follow.py        # ★ 视觉追踪独立实现 (级联 PID + 三态状态机)
├── motor.py             # 电机/编码器硬件抽象层
├── imu_motion.py        # IMU 姿态解算 + 角速度闭环
├── pid.py               # PID 控制器基类 (增量式 + 积分抗饱和)
├── ticker.py            # PIT 定时器封装
├── utils.py             # 工具函数
├── key.py               # 按键驱动 + 硬件看门狗 (PIT2 独占)
├── uart_master.py       # 蓝牙主控通信 (从车控制用)
├── uart_slave.py        # 蓝牙从控通信
├── uart_move.py         # UART 运动控制
├── cam_raw_recv.py      # 摄像头数据接收诊断
├── cam_diag.py          # UART7 原始字节诊断
├── cam_diag_v2.py       # 摄像头诊断 v2
├── cam_demo.py          # OpenMV 端目标检测程序
├── kalman_filter.py     # (未使用) 卡尔曼滤波器 (旧版视觉追踪用)
├── control.py           # (未使用) 级联 PID 控制器 (旧版视觉追踪用)
├── test/                # 测试脚本目录
│   ├── motor_test.py
│   ├── imu_test.py
│   ├── enc_calib_test.py
│   ├── motion_demo.py
│   └── ...
├── slave_main.py        # 从车主程序
├── slave_robot.py       # 从车机器人逻辑
└── slave_motor.py       # 从车电机驱动
```

---

> **文档结束** — 如需修改参数或添加功能，请参考对应模块源码注释中的调优指南。
