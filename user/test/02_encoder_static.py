"""
02_encoder_static.py - Encoder noise/drift detection (stationary test)
【功能】
  机器人完全静止（电机不通电）时读取 4 个编码器 5 秒，检测非零脉冲。
  用于诊断：编码器接触抖动 / EMI 干扰 / 零漂。
【方法】
  1) 确保机器人悬空或轮子离地（电机确保不通电）。
  2) stop_all() → 等待 50ms → 归零编码器（5 次 .get()）。
  3) 以 20ms 周期循环采样 250 次（共 5s）。
  4) 记录每个非零事件，统计每个轮子的最大 |delta|。
【通过条件】
  PASS (0 events)        → 编码器干净，无漂移。
  PASS (marginal, ≤5 events, all |delta|≤1) → 可接受的接触抖动。
  FAIL (more)            → 存在 EMI / 接线 / 上拉问题。
【硬件要求】
  机器人必须 LIFTED OFF GROUND（轮子悬空或自由转动）。
  本测试不驱动任何电机 —— 纯读。
"""

import gc, time
from machine import Pin
from smartcar import ticker
from motor import (
    stop_all, get_encoder_counts,
    LED_PIN, SWITCH2_PIN,
    encoder_rf, encoder_lf, encoder_lb, encoder_rb,
)

# ---------------------------------------------------------------------------
#  Constants
# ---------------------------------------------------------------------------
TEST_DURATION_S = 5.0
SAMPLE_INTERVAL_MS = 20

# ---------------------------------------------------------------------------
#  Safety: ensure all motors are OFF
# ---------------------------------------------------------------------------
stop_all()
time.sleep_ms(50)

# ---------------------------------------------------------------------------
#  Pin / button init
# ---------------------------------------------------------------------------
led = Pin(LED_PIN, Pin.OUT, value=True)
switch2 = Pin(SWITCH2_PIN, Pin.IN, pull=Pin.PULL_UP_47K)
state2 = switch2.value()

# ---------------------------------------------------------------------------
#  Ticker – hardware pulse capture at 10 ms period
# ---------------------------------------------------------------------------
pit = ticker(1)
pit.capture_list(encoder_rf, encoder_lf, encoder_lb, encoder_rb)
pit.start(10)

# ---------------------------------------------------------------------------
#  Zero out any residual encoder counts
# ---------------------------------------------------------------------------
for _ in range(5):
    encoder_rf.get()
    encoder_lf.get()
    encoder_lb.get()
    encoder_rb.get()
    time.sleep_ms(10)

# ---------------------------------------------------------------------------
#  Header
# ---------------------------------------------------------------------------
expected_samples = int(TEST_DURATION_S * 1000 / SAMPLE_INTERVAL_MS)
print("\n" + "=" * 60)
print("  Encoder Static Drift Test ({}s, ~{} samples)".format(TEST_DURATION_S, expected_samples))
print("  Robot MUST be lifted off ground – motors stay OFF")
print("  Toggle SWITCH2 to abort at any time")
print("=" * 60)
print("")

# ---------------------------------------------------------------------------
#  Main sampling loop
# ---------------------------------------------------------------------------
wheel_labels = ["RF", "LF", "LB", "RB"]
nonzero = [0, 0, 0, 0]
max_abs = [0, 0, 0, 0]
total_nonzero = 0
total_samples = 0
aborted = False

t_start = time.ticks_ms()

for _ in range(expected_samples):
    #  Check abort button
    if switch2.value() != state2:
        aborted = True
        print("\n  [ABORT] SWITCH2 toggled – stopping.")
        break

    elapsed = time.ticks_diff(time.ticks_ms(), t_start) / 1000.0
    counts = get_encoder_counts()
    total_samples += 1

    event = False
    for i, c in enumerate(counts):
        if c != 0:
            nonzero[i] += 1
            total_nonzero += 1
            event = True
            if abs(c) > max_abs[i]:
                max_abs[i] = abs(c)

    if event:
        print("  [t={:.2f}s] DRIFT: RF={} LF={} LB={} RB={}".format(elapsed, *counts))

    time.sleep_ms(SAMPLE_INTERVAL_MS)
    gc.collect()

# ---------------------------------------------------------------------------
#  Cleanup
# ---------------------------------------------------------------------------
pit.stop()
led.off()

# ---------------------------------------------------------------------------
#  Final report
# ---------------------------------------------------------------------------
print("\n" + "=" * 60)
print("  Encoder Static Drift Test Report ({}s)".format(TEST_DURATION_S))
print("=" * 60)
print("  Total samples: {}".format(total_samples))
if aborted:
    print("  NOTE: Test was aborted by SWITCH2 – results may not reflect full duration.")
print("  Non-zero events per wheel:")
print("    RF: {} events, max |delta| = {}".format(nonzero[0], max_abs[0]))
print("    LF: {} events, max |delta| = {}".format(nonzero[1], max_abs[1]))
print("    LB: {} events, max |delta| = {}".format(nonzero[2], max_abs[2]))
print("    RB: {} events, max |delta| = {}".format(nonzero[3], max_abs[3]))
print("  Total non-zero events: {}".format(total_nonzero))
print("-" * 60)

if total_nonzero == 0:
    print("  PASS: Zero drift in 5s. Encoders are clean.")
    print("  -> Proceed to imu_static_1min test.")
elif total_nonzero <= 5 and max(max_abs) <= 1:
    print("  PASS (marginal): {}/{} samples had |delta|=1. Likely contact bounce, acceptable.".format(total_nonzero, total_samples))
    print("  -> Proceed to imu_static_1min test, but note for production use.")
else:
    print("  FAIL: {} non-zero events, max |delta|={}. Encoder noise/EMI issue.".format(total_nonzero, max(max_abs)))
    print("  -> Check: encoder wiring shielding, pull-up resistors, motor power off,")
    print("     ground loop, EMI from nearby PWM/motor drivers.")
print("=" * 60)
