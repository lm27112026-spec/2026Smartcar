"""
main.py — 机器人集成框架入口
【功能】最小化入口：创建 Robot 实例并启动主循环
【使用】直接运行此文件即可
"""
from robot import Robot
import time

robot = Robot()
time.sleep_ms(50)  # 等待硬件稳定

try:
    robot.run()
except KeyboardInterrupt:
    print("\nProgram stopped by user.")
finally:
    from motor import stop_all
    stop_all()
    print("Robot stopped.")
