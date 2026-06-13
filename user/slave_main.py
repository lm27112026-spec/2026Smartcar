"""
slave_main.py — 从车入口
【功能】创建 SlaveRobot 实例并启动主循环
【使用】将此文件重命名为 main.py 烧录到从车上
"""
from slave_robot import SlaveRobot
import time

robot = SlaveRobot()
time.sleep_ms(50)

try:
    robot.run()
except KeyboardInterrupt:
    print("\nSlave program stopped by user.")
finally:
    from slave_motor import stop_all
    stop_all()
    print("Slave robot stopped.")
