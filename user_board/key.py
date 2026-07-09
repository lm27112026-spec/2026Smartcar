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
import machine

SCAN_PERIOD_MS = 10

key = KEY_HANDLER(SCAN_PERIOD_MS)

# 按键采集由主循环直接调用 key.capture() 驱动（详见 robot.py run()），
# 不再依赖 PIT ISR，因为 PIT0 被 MicroPython 系统定时器占用，
# PIT1 被 enc_ticker 共享，均不适宜独占。
#
# KEY_HANDLER 消抖在 3-10ms 的主循环周期下稳定工作。清除 debug 打印后
# 主循环迭代足够快，不存在消抖超时问题。

# ════════════════════════════════════════════════════════
#  PIT2 — 软件看门狗（不与任何模块共用）
# ════════════════════════════════════════════════════════
# PIT0: MicroPython 系统定时器
# PIT1: motor.py enc_ticker
# PIT2: 本看门狗独占
# PIT3: imu_motion.py IMU 自动采集 ticker
#
# 连续 3 秒未喂狗 → machine.reset() 硬复位 MCU。
#
# 懒初始化：PIT2 不在 import 时启动，首次 pet_watchdog()
# 调用时才武装。避免 import/初始化阶段因耗时超 3s 导致
# 误复位死循环。主循环启动后才需要看门狗保护。
_WWDG_CNT = 0
_WWDG_THRESHOLD = 300   # 300 ticks × 10ms = 3s
_WWDG_PIT = None

def _wwdg_isr(t):
    global _WWDG_CNT
    _WWDG_CNT += 1
    if _WWDG_CNT >= _WWDG_THRESHOLD:
        machine.reset()


def pet_watchdog():
    """主循环喂狗 — 每帧调用一次。
    首次调用时启动 PIT2 软件看门狗(3s)，后续每帧重置计数器。"""
    global _WWDG_CNT, _WWDG_PIT

    if _WWDG_PIT is None:
        _WWDG_PIT = ticker(2)
        _WWDG_PIT.callback(_wwdg_isr)
        _WWDG_PIT.start(SCAN_PERIOD_MS)

    _WWDG_CNT = 0


def stop_watchdog():
    """停止看门狗 — 任务正常退出后调用，防止 REPL 中误复位。"""
    global _WWDG_PIT
    if _WWDG_PIT is not None:
        try:
            _WWDG_PIT.stop()
        except Exception:
            pass
        _WWDG_PIT = None

KEY_NAMES = ["KEY1", "KEY2", "KEY3", "KEY4"]


def capture():
    """采集按键状态。由主循环或独立脚本调用，驱动 KEY_HANDLER 消抖状态机。"""
    key.capture()
    return True


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
    """停止按键扫描（无 PIT ticker 需要停止，保留兼容 main.py 调用）。"""
    pass
