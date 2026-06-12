import time
from imu_motion import imu, update_angle
import imu_motion

for _ in range(10):
    d = imu.read()
    update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    time.sleep_ms(20)

print("Turn robot 90° by hand. Ctrl+C to stop.")
while True:
    d = imu.read()
    update_angle(d[0], d[1], d[2], d[3], d[4], d[5])
    print("  yaw = {:.1f}".format(imu_motion.yaw))
    time.sleep_ms(200)