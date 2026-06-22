"""
cam_raw_recv.py — 摄像头原始数据接收与显示（完整协议解析）
【协议】AA [X_H X_L] [Y_H Y_L] [LABEL_H LABEL_L] [STATUS_H STATUS_L] BB
        X = 横向偏移(cm), Y = 纵向距离(cm), LABEL = 目标ID, STATUS: 1=识别, 0=丢失
        int16 大端序，÷10 精度
【使用】直接运行此文件，SW2 拨动退出
"""

import gc, time
from machine import UART, Pin

# ── 硬件初始化 ──────────────────────────────────────────────
switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
state2  = switch2.value()

uart_cam = UART(7)
uart_cam.init(baudrate=115200, bits=8, parity=None, stop=1)

FRAME_HEAD = 0xAA
FRAME_TAIL = 0xBB
FRAME_LEN  = 10
SCALE = 10.0

frame_count = 0
error_count = 0
target_count = 0
lost_count = 0
last_print_ms = time.ticks_ms()

print("=" * 60)
print("Camera Raw Receiver — 完整协议解析")
print("AA [X] [Y] [LABEL] [STATUS] BB  (int16 大端序, ÷10)")
print("SW2 toggle = exit")
print("=" * 60)

def to_signed16(v):
    return v if v < 32768 else v - 65536

while True:
    if switch2.value() != state2:
        print("\n[EXIT] SW2 toggled.")
        break

    if uart_cam.any() < FRAME_LEN:
        time.sleep_ms(1)
        continue

    buf = uart_cam.read()
    if not buf:
        continue

    idx = buf.find(bytes([FRAME_HEAD]))
    if idx == -1:
        hex_str = ' '.join('{:02X}'.format(b) for b in buf)
        print("[RAW] {:s}".format(hex_str))
        error_count += 1
        continue

    if len(buf) < idx + FRAME_LEN:
        print("[WARN] Incomplete frame")
        error_count += 1
        continue

    if buf[idx + 9] != FRAME_TAIL:
        print("[ERR] Invalid tail: 0x{:02X}".format(buf[idx + 9]))
        error_count += 1
        continue

    # int16 大端序解析
    raw_x      = (buf[idx + 1] << 8) | buf[idx + 2]
    raw_y      = (buf[idx + 3] << 8) | buf[idx + 4]
    raw_label  = (buf[idx + 5] << 8) | buf[idx + 6]
    raw_status = (buf[idx + 7] << 8) | buf[idx + 8]

    x_cm = to_signed16(raw_x) / SCALE
    y_cm = to_signed16(raw_y) / SCALE
    label = raw_label
    status = raw_status

    frame_count += 1
    is_target = (status == 1) and not (x_cm == 0 and y_cm == 0)

    if is_target:
        target_count += 1
    else:
        lost_count += 1

    now = time.ticks_ms()
    if time.ticks_diff(now, last_print_ms) >= 200:
        state = "TGT" if is_target else "---"
        print("[#{:04d} {:s}] X:{:+7.1f}cm  Y:{:6.1f}cm  id:{:d}  status:{:d}".format(
            frame_count, state, x_cm, y_cm, label, status))
        last_print_ms = now

    if frame_count % 200 == 0:
        print("--- stats: frames={:d}  target={:d}  lost={:d}  errors={:d} ---".format(
            frame_count, target_count, lost_count, error_count))

    gc.collect()

print("\n" + "=" * 60)
print("Session summary:")
print("  Total frames  : {:d}".format(frame_count))
print("  Target frames : {:d}".format(target_count))
print("  Lost frames   : {:d}".format(lost_count))
print("  Errors        : {:d}".format(error_count))
print("=" * 60)
