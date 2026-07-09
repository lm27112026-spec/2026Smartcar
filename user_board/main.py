"""
test_uwb_nav.py — UWB 导航调试：发车区 → 物资区 （VOFA+ 可视化）
"""
import gc
import time
import struct
import sys
from machine import Pin
gc.collect()

from uwb_control import UWBPosition, goto_location
from IMU_hold import HeadingHold
from uwb_follow import GOTO_SUPPLIES_X, GOTO_SUPPLIES_Y, GOTO_CTRL_DT
from motor import (stop_all, omni_drive_closed_loop,
                   get_encoder_counts,
                   reset_encoder_filter, reset_wheel_pi, ENC_SCALE)
from imu import IMU
from key import pet_watchdog
gc.collect()

TARGET_X, TARGET_Y = GOTO_SUPPLIES_X, GOTO_SUPPLIES_Y
LED_PIN         = 'C4'
SW2_PIN         = 'D9'
SW2_DEBOUNCE_MS = 50
UWB_UART        = 0
UWB_BAUDRATE    = 115200
UWB_ANCHOR      = "8834"

# ═══════════════════════════════════════════════════════════════
#  VOFA+ Firewater 协议
# ═══════════════════════════════════════════════════════════════
ENABLE_VOFA       = True
VOFA_SEND_MS      = 100      # VOFA 发送间隔 (ms)
_FW_TAIL          = bytearray([0x00, 0x00, 0x80, 0x7F])  # float infinity

_last_vofa_ms = 0  # 全局节流时间戳

def send_vofa(tx, ty, cx, cy):
    """发送 4 通道 float 到 VOFA+: CH0=target_x, CH1=target_y, CH2=curr_x, CH3=curr_y"""
    data = struct.pack("<ffff", tx, ty, cx, cy)
    sys.stdout.buffer.write(data + _FW_TAIL)


_sw2_last         = 1
_sw2_changed      = False
_sw2_stable_start = 0


def _check_sw2(sw2_pin):
    global _sw2_last, _sw2_changed, _sw2_stable_start
    val = sw2_pin.value()
    if val != _sw2_last:
        _sw2_last = val
        _sw2_changed = True
        _sw2_stable_start = time.ticks_ms()
    if _sw2_changed and time.ticks_diff(time.ticks_ms(), _sw2_stable_start) >= SW2_DEBOUNCE_MS:
        _sw2_changed = False
        return True
    return False


def main():
    gc.collect()

    led = Pin(LED_PIN, Pin.OUT, value=False)
    sw2 = Pin(SW2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)

    global _sw2_last, _sw2_changed, _sw2_stable_start
    _sw2_last = sw2.value()
    _sw2_changed = False
    _sw2_stable_start = 0

    # print("  UWB nav: ({:.1f}, {:.1f})".format(TARGET_X, TARGET_Y))

    # ── IMU ──
    imu = IMU(calibrate_on_init=True, calib_samples=500, period_ms=10)
    imu.start()
    time.sleep_ms(200)
    imu.set_zero_reference()
    hold = HeadingHold(imu, target_yaw_deg=0.0)

    # ── 编码器复位 ──
    reset_encoder_filter()
    reset_wheel_pi()
    hold.reset()
    for _ in range(5):
        _ = get_encoder_counts()
        time.sleep_ms(10)

    # ── UWB ──
    uwb = UWBPosition(uart_id=UWB_UART, baudrate=UWB_BAUDRATE, target_anchor=UWB_ANCHOR)
    wait_start = time.ticks_ms()
    while uwb.get_frame_count() == 0:
        uwb.step()
        pet_watchdog()
        if _check_sw2(sw2) or time.ticks_diff(time.ticks_ms(), wait_start) > 5000:
            uwb.stop(); stop_all(); led.value(0); return
        time.sleep_ms(10)
    # print("  [UWB] ready f={}".format(uwb.get_frame_count()))

    # ── 回调 ──
    def lock_heading():  return hold.target
    def calc_wz(t):
        if t is not None and t != hold.target: hold.set_target(t)
        wz, _, _ = hold.compute(GOTO_CTRL_DT); return wz
    def get_yaw():       return imu.get_angles()[2]
    def should_abort():
        pet_watchdog(); return _check_sw2(sw2)
    def drive_fn(vx, vy, wz, dt):
        try:
            rc = get_encoder_counts()
            if rc and len(rc) >= 4:
                rs = [rc[i] / ENC_SCALE[i] / dt if ENC_SCALE[i] != 0 else 0 for i in range(4)]
                omni_drive_closed_loop(vx, vy, wz, rs, dt)
        except Exception: pass
    def stop_fn():  stop_all()
    def led_fn(on): led.value(1 if on else 0)

    # ── 编码器融合回调 ──
    def enc_fn(): return get_encoder_counts()
    def drive_with_spd(vx, vy, wz, dt, spd):
        omni_drive_closed_loop(vx, vy, wz, spd, dt)

    # ── VOFA on_progress 回调 ──
    global _last_vofa_ms
    _last_vofa_ms = time.ticks_ms()

    def on_progress(dist_cm, curr_x, curr_y):
        global _last_vofa_ms
        if not ENABLE_VOFA:
            return
        now = time.ticks_ms()
        if time.ticks_diff(now, _last_vofa_ms) >= VOFA_SEND_MS:
            _last_vofa_ms = now
            send_vofa(TARGET_X, TARGET_Y, curr_x, curr_y)

    # ── 导航 ──
    # print("  [GO]")
    try:
        arrived, reason = goto_location(
            uwb, TARGET_X, TARGET_Y,
            lock_heading, calc_wz, get_yaw,
            should_abort, drive_fn, stop_fn,
            enc_fn, ENC_SCALE, drive_with_spd,
            led_fn=led_fn, label="UWB",
            on_progress=on_progress, verbose=False
        )
        raw = uwb.get_latest_raw()
        filt = uwb.get_position()
        print("  [OK] {} raw({:.1f},{:.1f}) filt({:.1f},{:.1f}) f={}".format(
            reason, raw[0], raw[1], filt[0], filt[1], uwb.get_frame_count()))
    except KeyboardInterrupt:
        print("\n  [STOP]")
    except Exception as e:
        print("\n  [ERR] {}".format(e))
        import sys; sys.print_exception(e)
    finally:
        imu.stop()
        uwb.stop(); stop_all(); led.value(0)
        print("  [DONE] 按 SW2 或复位退出。")
        try:
            while not _check_sw2(sw2):
                pet_watchdog()
                time.sleep_ms(100)
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()

