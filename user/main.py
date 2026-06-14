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
    from motor import stop_all, pause_encoder_ticker
    stop_all()
    pause_encoder_ticker()     # 停编码器 ticker（PIT1）
    
    # 停掉所有可能干扰 REPL 的后台 ticker
    try:
        from key import stop as stop_key_ticker
        stop_key_ticker()      # 停 key.py 的按键扫描 ticker（PIT1）
    except ImportError:
        pass                   # key.py 不存在 → 走 fallback 路径
    
    from robot import stop_key_ticker as stop_fallback_key
    stop_fallback_key()        # 停 fallback 独立按键 ticker（PIT0，路径无关安全调用）
    
    time.sleep_ms(50)          # 等待硬件静默
    print("Robot stopped. (you may now re-run or enter REPL)")
