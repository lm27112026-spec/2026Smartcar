"""
cam_probe.py - 协议探测程序
通过观察不同条件下字节的变化来确定协议格式

【使用步骤】
1. 运行此程序，先让摄像头看到目标 → 观察基线数据
2. 把目标移远/移近 → 观察哪些字节随距离变化（= Y字段）
3. 把目标左移/右移 → 观察哪些字节随横向变化（= X字段）
4. 用不同颜色/形状的目标 → 观察哪些字节变化（= label字段）
5. 遮挡目标 → 观察哪些字节归零/变化（= status字段）

C14: 打印详细统计分析
SW2: 退出程序
"""

from machine import UART, Pin
import time, math

# ── 硬件初始化 ──────────────────────────────────────────────
btn_analyze = Pin('C14', Pin.IN, pull=Pin.PULL_UP_47K)
btn_exit    = Pin('D9',  Pin.IN, pull=Pin.PULL_UP_47K)
state_exit  = btn_exit.value()

uart = UART(7)
uart.init(baudrate=115200, bits=8, parity=None, stop=1)

FRAME_HEAD = 0xAA
FRAME_TAIL = 0xBB
FRAME_LEN = 10

# 存储最近 N 帧的数据用于统计
HISTORY_LEN = 100
history = []  # list of (timestamp_ms, [b0..b7])

print("=" * 60)
print("Protocol Probe - 协议探测")
print("=" * 60)
print()
print("操作步骤:")
print("  1) 先让摄像头看到目标，观察基线")
print("  2) 目标移远/移近 → 观察距离相关字节")
print("  3) 目标左移/右移 → 观察横向相关字节")
print("  4) 遮住目标     → 观察状态字节")
print()
print("  C14 = 打印详细分析 | SW2 = 退出")
print("=" * 60)
print()


def signed16(hi, lo):
    v = (hi << 8) | lo
    return v if v < 32768 else v - 65536


def analyze_bytes():
    """分析历史数据中每个字节的变化规律"""
    if len(history) < 10:
        print("[!] 数据不足，至少需要 10 帧")
        return

    print()
    print("=" * 60)
    print("📊 字节分析 (基于最近 {} 帧)".format(len(history)))
    print("=" * 60)

    for byte_idx in range(8):
        values = [entry[1][byte_idx] for entry in history]
        min_val = min(values)
        max_val = max(values)
        avg_val = sum(values) / len(values)
        spread = max_val - min_val

        # 判断字节特征
        if spread == 0:
            tag = "固定值"
        elif spread < 5:
            tag = "几乎不变"
        elif spread < 50:
            tag = "小范围变化"
        elif spread > 200:
            tag = "大范围变化"
        else:
            tag = "中等变化"

        print("  Byte[{}]: min={:3d} max={:3d} avg={:6.1f} spread={:3d}  ← {}".format(
            byte_idx, min_val, max_val, avg_val, spread, tag))

    # 尝试不同的协议解释
    print()
    print("=" * 60)
    print("🔍 尝试不同协议解读")
    print("=" * 60)

    # 解读方案 1: 标准 cam_demo 协议
    xs = [signed16(e[1][0], e[1][1]) / 10.0 for e in history]
    ys = [signed16(e[1][2], e[1][3]) / 10.0 for e in history]
    labels = [(e[1][4] << 8 | e[1][5]) for e in history]
    statuses = [(e[1][6] << 8 | e[1][7]) for e in history]

    print("\n方案1 [标准 AA x y label status BB]:")
    print("  X:  min={:+.1f} max={:+.1f} spread={:.1f}".format(
        min(xs), max(xs), max(xs) - min(xs)))
    print("  Y:  min={:.1f} max={:.1f} spread={:.1f}".format(
        min(ys), max(ys), max(ys) - min(ys)))
    print("  Label: min={} max={} spread={}".format(
        min(labels), max(labels), max(labels) - min(labels)))
    print("  Status: min={} max={} spread={}".format(
        min(statuses), max(statuses), max(statuses) - min(statuses)))

    # 解读方案 2: label 和 status 交换
    labels2 = [(e[1][6] << 8 | e[1][7]) for e in history]
    statuses2 = [(e[1][4] << 8 | e[1][5]) for e in history]

    print("\n方案2 [AA x y status label BB] (label/status 互换):")
    print("  Status: min={} max={} spread={}".format(
        min(statuses2), max(statuses2), max(statuses2) - min(statuses2)))
    print("  Label: min={} max={} spread={}".format(
        min(labels2), max(labels2), max(labels2) - min(labels2)))

    # 解读方案 3: byte[4] 单独是 label, byte[5] 是 confidence
    print("\n方案3 [逐字节拆解]:")
    for i in range(8):
        vals = [e[1][i] for e in history]
        print("  byte[{}]: range [{}, {}]".format(i, min(vals), max(vals)))

    print()
    print("=" * 60)
    print("判断方法:")
    print("  - 如果 X spread 很大 → byte[0:2] 确实是 X")
    print("  - 如果 Y spread 很大 → byte[2:4] 确实是 Y")
    print("  - 如果某个 byte 固定 → 可能是协议标识/版本号")
    print("  - 遮挡目标后看哪些 byte 归零 → 那就是 status")
    print("=" * 60)
    print()


# ── 主循环 ──────────────────────────────────────────────────
buf = bytearray()
frame_count = 0

print("开始采集数据... (按 C14 打印分析)")
print()

while True:
    # SW2 按下 → 退出
    if btn_exit.value() != state_exit:
        print("\n[EXIT] SW2 toggled.")
        break

    # C14 按下 → 触发分析
    if btn_analyze.value() == 0:  # 低电平有效（按下接地）
        analyze_bytes()
        time.sleep_ms(300)  # 消抖
        while btn_analyze.value() == 0:
            time.sleep_ms(10)

    # 读取串口
    if uart.any():
        chunk = uart.read()
        if chunk:
            buf.extend(chunk)

    # 解析帧
    while len(buf) >= FRAME_LEN:
        idx = buf.find(bytes([FRAME_HEAD]))
        if idx == -1:
            buf = bytearray()
            break

        if idx > 0:
            buf = buf[idx:]

        if len(buf) < FRAME_LEN:
            break

        if buf[FRAME_LEN - 1] != FRAME_TAIL:
            buf = buf[1:]
            continue

        frame = bytes(buf[:FRAME_LEN])
        buf = buf[FRAME_LEN:]

        # 记录数据
        data_bytes = [frame[i + 1] for i in range(8)]
        history.append((time.ticks_ms(), data_bytes))
        if len(history) > HISTORY_LEN:
            history.pop(0)

        frame_count += 1

        # 简洁输出
        x = signed16(data_bytes[0], data_bytes[1]) / 10.0
        y = signed16(data_bytes[2], data_bytes[3]) / 10.0

        # 每 20 帧打印一次简要信息
        if frame_count % 20 == 0:
            print("[#{:04d}] X:{:+7.1f} Y:{:6.1f} | bytes[4-7]: {:02X} {:02X} {:02X} {:02X}".format(
                frame_count, x, y, data_bytes[4], data_bytes[5], data_bytes[6], data_bytes[7]))

    time.sleep_ms(1)

print("\n" + "=" * 60)
print("Session: {} frames collected".format(frame_count))
print("=" * 60)
