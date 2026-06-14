"""
key.py - 按键驱动模块
基于 seekfree 库 KEY_HANDLER，带消抖、长短按检测
【层】硬件抽象层
【引脚】KEY1(C8) KEY2(C9) KEY3(C14) KEY4(C15)
【功能】
  - 自动 ticker 采集（10ms 周期）
  - 短按松发检测（按下松开后触发）
  - 长按检测（500ms 生效）
  - 状态读取与清除
【使用】
  from key import key, read_keys, key_triggered, clear_key
"""

from smartcar import ticker
from seekfree import KEY_HANDLER

SCAN_PERIOD_MS = 10

key = KEY_HANDLER(SCAN_PERIOD_MS)

ticker_flag = False

def _ticker_handler(t):
    global ticker_flag
    ticker_flag = True

_pit = ticker(0)  # PIT0: 独立于 motor.py 的 enc_ticker(PIT1)
_pit.capture_list(key)
_pit.callback(_ticker_handler)
_pit.start(SCAN_PERIOD_MS)

KEY_NAMES = ["KEY1", "KEY2", "KEY3", "KEY4"]


def capture():
    if ticker_flag:
        key.capture()
        return True
    return False


def read_keys():
    return key.read()


def key_triggered(index):
    state = key.get()
    if index < 1 or index > 4:
        return False
    if state[index - 1]:
        key.clear(index)
        return True
    return False


def clear_key(index=None):
    if index is None:
        key.clear()
    else:
        key.clear(index)


def any_key():
    state = key.get()
    return any(state)


def wait_any_key(timeout_ms=0):
    import time
    start = time.ticks_ms()
    while True:
        capture()
        if any_key():
            return read_keys()
        if timeout_ms > 0 and time.ticks_ms() - start > timeout_ms:
            return None
        time.sleep_ms(1)


def stop():
    """停止按键扫描 ticker（程序退出时调用，解除对 REPL 的干扰）。"""
    _pit.stop()
