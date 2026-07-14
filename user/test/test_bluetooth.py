"""
test_bluetooth.py — 主车蓝牙收发测试
发送 turn_left(1) / turn_right(0)，等待从车回复 ok。
协议、超时、重试逻辑与 main.py 完全一致。
"""

import sys, time
sys.path.insert(0, 'user')

from machine import Pin
from uart_master import MasterBT

BT_WAIT_DEADLINE_S = 10.0
SW2_PIN = 'D9'

bt = MasterBT()
sw2 = Pin(SW2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)


def check_sw2():
    return sw2.value() == 0


def send_and_wait_ok(cmd_name, send_fn):
    """发送指令并等待从车回复 ok，超时重试。与 main.py 逻辑一致。"""
    print("\n[TEST] 发送 {} ...".format(cmd_name))
    retry = 0
    while True:
        send_fn()
        retry += 1
        print("[TEST] 等待 ok (第{}次, 超时{}s)...".format(retry, BT_WAIT_DEADLINE_S))
        start = time.ticks_ms()
        while True:
            if time.ticks_diff(time.ticks_ms(), start) > BT_WAIT_DEADLINE_S * 1000:
                print("[TEST] 超时，重发...")
                break
            if check_sw2():
                print("[TEST] SW2 中断")
                return False
            resp = bt.read_response()
            if resp == "ok":
                print("[TEST] ✓ 收到 ok")
                return True
            time.sleep_ms(10)


# ══════════════════════════════════════════════════
print("=" * 40)
print("  主车蓝牙测试 — turn_right + turn_left")
print("  SW2 按下退出")
print("=" * 40)

try:
    send_and_wait_ok("turn_right (0)", bt.turn_right)
    time.sleep_ms(300)
    send_and_wait_ok("turn_left  (1)", bt.turn_left)
except Exception as e:
    print("[TEST] 异常:", e)

print("\n[TEST] 测试结束。")
