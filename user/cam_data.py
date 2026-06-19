"""
cam_data.py - 摄像头数据接收与解析模块（适配实际协议）
【协议】AA [X_H X_L] [Y_H Y_L] [FLAG] [ID] [B6] [B7] BB
        X = 横向偏移 (原始值单位: mm, ÷10后仍为mm)
        Y = 纵向距离 (原始值单位: cm, ÷10后为cm)
        FLAG: 0x02/0x03=检测到, 0x00=丢失
        ID = 目标标识 (单字节)
        B6, B7 = 附加数据 (待定)
        int16 大端序，÷10 精度
【单位说明】
        X 返回值单位是 mm，用 x_to_cm(x) 转为 cm
        Y 返回值单位是 cm，用 y_to_distance(y) 转为实际距离
【Y坐标说明】
        Y=0 对应实际距离 29cm
        Y>0 比 29cm 更近, Y<0 比 29cm 更远
【使用】
    from cam_data import CamDataReceiver, x_to_cm, y_to_distance
    recv = CamDataReceiver(uart_id=7)
    while True:
        data = recv.read()
        if data is not None:
            x_cm = x_to_cm(data['x'])      # 横向偏移 (cm)
            dist = y_to_distance(data['y']) # 实际距离 (cm)
            print(f"X:{x_cm:.1f}cm 距离:{dist:.1f}cm")
"""

from machine import UART
import time

# 协议常量
FRAME_HEAD = 0xAA
FRAME_TAIL = 0xBB
FRAME_LEN = 10
SCALE = 10.0

# Y 坐标参考点：Y=0 时对应的实际距离
Y_REF_DISTANCE = 29.0  # cm


def _to_signed16(v):
    """无符号转有符号 int16"""
    return v if v < 32768 else v - 65536


def y_to_distance(y):
    """
    返回摄像头 Y 坐标对应的实际距离
    
    参数:
        y: 摄像头返回的 Y 坐标 (cm)
    
    返回:
        float: 目标到摄像头的实际距离 (cm)
    
    说明:
        摄像头协议: Y = 纵向距离 (单位: cm)
        Y 已经是距离值，直接返回即可
    """
    return y


def x_to_cm(x):
    """
    将摄像头 X 坐标转换为 cm
    
    参数:
        x: 摄像头返回的 X 值 (单位: mm)
    
    返回:
        float: 横向偏移 (cm)
    
    说明:
        X>0 → 目标在右侧
        X<0 → 目标在左侧
    """
    return x / 10.0


class CamDataReceiver:
    """
    摄像头数据接收器
    
    用法:
        recv = CamDataReceiver(uart_id=7)
        while True:
            data = recv.read()
            if data is not None:
                if data['is_target']:
                    print(f"X:{data['x']:.1f} Y:{data['y']:.1f} flag:{data['flag']}")
    """
    
    def __init__(self, uart_id=7, baudrate=115200):
        """
        初始化接收器
        
        参数:
            uart_id: UART 编号
            baudrate: 波特率
        """
        self._uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1)
        self._buf = bytearray()
        self._frame_count = 0
        self._error_count = 0
        self._target_count = 0
        self._lost_count = 0
    
    @property
    def frame_count(self):
        """总帧数"""
        return self._frame_count
    
    @property
    def error_count(self):
        """错误帧数"""
        return self._error_count
    
    @property
    def target_count(self):
        """识别成功帧数"""
        return self._target_count
    
    @property
    def lost_count(self):
        """丢失帧数"""
        return self._lost_count
    
    def reset_stats(self):
        """重置统计计数"""
        self._frame_count = 0
        self._error_count = 0
        self._target_count = 0
        self._lost_count = 0
    
    def read(self):
        """
        非阻塞读取一帧数据
        
        返回:
            dict - 成功时返回数据字典
            None - 无数据或数据无效
        """
        # 检查是否有足够数据
        if self._uart.any() < 1:
            return None
        
        # 读取可用数据
        chunk = self._uart.read()
        if chunk:
            self._buf.extend(chunk)
        
        # 查找帧头
        while True:
            idx = self._buf.find(bytes([FRAME_HEAD]))
            if idx == -1:
                # 没有找到帧头，清空缓冲区
                self._buf = bytearray()
                return None
            
            # 检查是否有完整帧
            if len(self._buf) < idx + FRAME_LEN:
                # 数据不完整，保留从帧头开始的部分
                if idx > 0:
                    self._buf = bytearray(self._buf[idx:])
                return None
            
            # 检查帧尾
            if self._buf[idx + FRAME_LEN - 1] != FRAME_TAIL:
                # 无效帧尾，跳过这个字节继续查找
                self._buf = bytearray(self._buf[idx + 1:])
                self._error_count += 1
                continue
            
            # 提取有效帧
            frame = self._buf[idx:idx + FRAME_LEN]
            self._buf = bytearray(self._buf[idx + FRAME_LEN:])
            
            # 解析数据
            return self._parse_frame(frame)
    
    def _parse_frame(self, frame):
        """
        解析一帧数据
        
        参数:
            frame: 10 字节的帧数据
            
        返回:
            dict 或 None
        """
        self._frame_count += 1
        
        # 解析 X, Y (int16 大端序)
        raw_x = (frame[1] << 8) | frame[2]
        raw_y = (frame[3] << 8) | frame[4]
        
        x = _to_signed16(raw_x) / SCALE
        y = _to_signed16(raw_y) / SCALE
        
        # 解析附加字段
        flag = frame[5]      # 检测标志 (0x02/0x03=检测到, 0x00=丢失)
        target_id = frame[6] # 目标标识
        b6 = frame[6]        # byte 6
        b7 = frame[7]        # byte 7
        
        # 判断是否检测到目标
        # 当 X=0 且 Y=0 时，flag 也是 0，表示丢失
        is_target = (flag != 0) and not (x == 0 and y == 0)
        
        # 统计
        if is_target:
            self._target_count += 1
        else:
            self._lost_count += 1
        
        return {
            'x': x,
            'y': y,
            'flag': flag,
            'id': target_id,
            'b6': b6,
            'b7': b7,
            'is_target': is_target
        }
    
    def read_block(self, timeout_ms=100):
        """
        阻塞读取，直到收到有效数据或超时
        
        参数:
            timeout_ms: 超时时间(毫秒)
            
        返回:
            dict 或 None (超时)
        """
        deadline = time.ticks_ms() + timeout_ms
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            data = self.read()
            if data is not None:
                return data
            time.sleep_ms(1)
        return None
    
    def flush(self):
        """清空接收缓冲区"""
        while self._uart.any():
            self._uart.read()
        self._buf = bytearray()


# ── 独立运行测试 ─────────────────────────────────────────────
if __name__ == '__main__':
    from machine import Pin
    
    switch2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K)
    state2 = switch2.value()
    
    recv = CamDataReceiver(uart_id=7)
    
    print("=" * 50)
    print("CamDataReceiver Test (Adapted Protocol)")
    print("SW2 toggle = exit")
    print("=" * 50)
    
    last_print_ms = time.ticks_ms()
    
    while True:
        if switch2.value() != state2:
            print("\n[EXIT] SW2 toggled.")
            break
        
        data = recv.read()
        if data is None:
            continue
        
        now = time.ticks_ms()
        if time.ticks_diff(now, last_print_ms) >= 200:
            state = "TGT" if data['is_target'] else "---"
            print("[#{:04d} {:s}] X:{:+7.1f} Y:{:6.1f} flag:{:02X} id:{:02X}".format(
                recv.frame_count, state, data['x'], data['y'], data['flag'], data['id']))
            last_print_ms = now
        
        if recv.frame_count % 200 == 0 and recv.frame_count > 0:
            print("--- stats: frames={:d}  target={:d}  lost={:d}  errors={:d} ---".format(
                recv.frame_count, recv.target_count, recv.lost_count, recv.error_count))
        
        time.sleep_ms(1)
    
    print("\n" + "=" * 50)
    print("Session summary:")
    print("  Total frames  : {:d}".format(recv.frame_count))
    print("  Target frames : {:d}".format(recv.target_count))
    print("  Lost frames   : {:d}".format(recv.lost_count))
    print("  Errors        : {:d}".format(recv.error_count))
    print("=" * 50)
