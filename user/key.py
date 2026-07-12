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
#  PIT2 — 独立硬件看门狗（不与任何模块共用）
# ════════════════════════════════════════════════════════
# 当主循环挂死时，PIT1 可能被 enc_ticker 接管，所以看门狗
# 不能依赖 PIT1。
# PIT0: MicroPython 系统定时器
# PIT1: motor.py enc_ticker
# PIT2: 本看门狗独占
# PIT3: imu_motion.py IMU 自动采集 ticker
#
# 连续 3 秒未喂狗 → machine.reset() 硬复位 MCU。
# ISR 中调用 machine.reset() 只是写硬件复位寄存器，安全无分配。
_WWDG_CNT = 0
_WWDG_THRESHOLD = 300   # 300 ticks × 10ms = 3s

def _wwdg_isr(t):
    global _WWDG_CNT
    _WWDG_CNT += 1
    if _WWDG_CNT >= _WWDG_THRESHOLD:
        machine.reset()

_WWDG_PIT = ticker(2)  # PIT2：仅供看门狗专用
_WWDG_PIT.callback(_wwdg_isr)
# _WWDG_PIT.start(SCAN_PERIOD_MS)  # 延迟启动，防止 Thonny 调试阶段误复位


def pet_watchdog():
    """主循环喂狗 — 每帧调用一次。
    若主循环（含 _dispatch_mode）卡住超过 3 秒，PIT3 ISR 触发 hard reset。"""
    global _WWDG_CNT
    _WWDG_CNT = 0


def start_watchdog():
    """显式启用独立硬件看门狗，供 main.py 在初始化完成后调用"""
    global _WWDG_CNT
    _WWDG_CNT = 0
    _WWDG_PIT.start(SCAN_PERIOD_MS)

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
    """停止按键扫描与看门狗，防止程序退出后触发复位"""
    try:
        _WWDG_PIT.stop()
        print("  [WWDG] 看门狗已安全停止")
    except Exception:
        pass
