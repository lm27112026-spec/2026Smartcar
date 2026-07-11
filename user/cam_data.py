"""
cam_data.py - 摄像头数据接收与解析模块（适配实际协议）
【协议】AA [X_H X_L] [Y_H Y_L] [LABEL_H LABEL_L] [STATUS_H STATUS_L] [LINE_FLAG_H LINE_FLAG_L] BB
        X = 横向偏移 (原始值单位: mm, ÷10后仍为mm)
        Y = 纵向距离 (原始值单位: cm, ÷10后为cm)
        LABEL = 目标标识 (int16)
        STATUS = 检测状态 (int16, 0=丢失)
        LINE_FLAG = 黄线标志 (int16)
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
FRAME_LEN = 12
SCALE = 10.0
FRAME_HEAD_BYTES = bytes([FRAME_HEAD])      # 预分配，避免每次 find() 创建临时对象
BUF_MAX = 256                                # 缓冲区上限，超出则视为噪声清空

# Y 坐标参考点：Y=0 时对应的实际距离
Y_REF_DISTANCE = 29.0  # cm


def _to_signed16(v):
    """无符号转有符号 int16"""
    return v if v < 32768 else v - 65536


def y_to_distance(y):
    """返回相机 Y 原始值 (cm)"""
    return y

def x_to_cm(x):
    """返回相机 X 原始值 (已由 _parse_frame ÷10, 单位 cm)"""
    return x


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

        # 缓冲区上限保护：超过 BUF_MAX 视为噪声，清空避免内存耗尽
        if len(self._buf) > BUF_MAX:
            self._buf = bytearray()
            return None

        # 查找帧头
        while True:
            idx = self._buf.find(FRAME_HEAD_BYTES)
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
            frame: 12 字节的帧数据
            
        返回:
            dict 或 None
        """
        self._frame_count += 1
        
        # 解析 X, Y (int16 大端序)
        raw_x = (frame[1] << 8) | frame[2]
        raw_y = (frame[3] << 8) | frame[4]
        
        x = _to_signed16(raw_x) / SCALE
        y = _to_signed16(raw_y) / SCALE
        
        # 解析 LABEL (bytes 5-6, int16 大端序)
        raw_label = (frame[5] << 8) | frame[6]
        label_val = _to_signed16(raw_label)
        
        # 解析 STATUS (bytes 7-8, int16 大端序)
        raw_status = (frame[7] << 8) | frame[8]
        status_val = _to_signed16(raw_status)
        
        # 解析 LINE_FLAG (bytes 9-10, int16 大端序)
        raw_line_flag = (frame[9] << 8) | frame[10]
        line_flag_val = _to_signed16(raw_line_flag)
        
        # 衍生字段（保持兼容）
        b6 = frame[5]        # label 高字节
        b7 = frame[6]        # label 低字节
        id = label_val       # label 有符号整数值
        flag = frame[7]      # status 高字节
        
        # 判断是否检测到目标
        # status != 0 且 X,Y 不同时为 0 才算有效目标
        is_target = (status_val != 0) and not (x == 0 and y == 0)
        
        # 统计
        if is_target:
            self._target_count += 1
        else:
            self._lost_count += 1
        
        return {
            'x': x,
            'y': y,
            'label': label_val,
            'id': id,
            'flag': flag,
            'status': status_val,
            'b6': b6,
            'b7': b7,
            'line_flag': line_flag_val,
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

    def deinit(self):
        """释放 UART 资源（调用后方可被其他模块重新打开）"""
        if self._uart is not None:
            try:
                self._uart.deinit()
            except Exception:
                pass
            self._uart = None
