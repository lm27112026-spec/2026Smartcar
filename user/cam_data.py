# cam_data.py - 摄像头数据接收与解析模块（适配实际协议）
from machine import UART
import time

FRAME_HEAD = 0xAA
FRAME_TAIL = 0xBB
FRAME_LEN = 12
SCALE = 10.0
FRAME_HEAD_BYTES = bytes([FRAME_HEAD])
BUF_MAX = 256
Y_REF_DISTANCE = 29.0

def _to_signed16(v):
    return v if v < 32768 else v - 65536

def y_to_distance(y):
    return y

def x_to_cm(x):
    return x

class CamDataReceiver:
    def __init__(self, uart_id=7, baudrate=115200):
        self._uart = UART(uart_id, baudrate=baudrate, bits=8, parity=None, stop=1)
        self._buf = bytearray()
        self._frame_count = 0
        self._error_count = 0
        self._target_count = 0
        self._lost_count = 0
        self._cached_data = {
            'x': 0.0, 'y': 0.0,
            'label': 0, 'id': 0,
            'flag': 0, 'status': 0,
            'b6': 0, 'b7': 0,
            'line_flag': 0, 'is_target': False
        }

    @property
    def frame_count(self): return self._frame_count

    @property
    def error_count(self): return self._error_count

    @property
    def target_count(self): return self._target_count

    @property
    def lost_count(self): return self._lost_count

    def reset_stats(self):
        self._frame_count = 0
        self._error_count = 0
        self._target_count = 0
        self._lost_count = 0

    def read(self):
        if self._uart.any() < 1: return None
        chunk = self._uart.read()
        if chunk: self._buf.extend(chunk)
        if len(self._buf) > BUF_MAX:
            self._buf = bytearray()
            return None
        while True:
            idx = self._buf.find(FRAME_HEAD_BYTES)
            if idx == -1:
                self._buf = bytearray()
                return None
            if idx > 0:
                self._buf[:idx] = b''
                idx = 0
            if len(self._buf) < FRAME_LEN: return None
            if self._buf[FRAME_LEN - 1] != FRAME_TAIL:
                self._buf[:1] = b''
                self._error_count += 1
                continue
            frame = self._buf[:FRAME_LEN]
            self._buf[:FRAME_LEN] = b''
            return self._parse_frame(frame)

    def _parse_frame(self, frame):
        self._frame_count += 1
        raw_x = (frame[1] << 8) | frame[2]
        raw_y = (frame[3] << 8) | frame[4]
        x = _to_signed16(raw_x) / SCALE
        y = _to_signed16(raw_y) / SCALE
        raw_label = (frame[5] << 8) | frame[6]
        label_val = _to_signed16(raw_label)
        raw_status = (frame[7] << 8) | frame[8]
        status_val = _to_signed16(raw_status)
        raw_line_flag = (frame[9] << 8) | frame[10]
        line_flag_val = _to_signed16(raw_line_flag)
        b6 = frame[5]
        b7 = frame[6]
        id = label_val
        flag = frame[7]
        is_target = (status_val != 0) and not (x == 0 and y == 0)
        if is_target: self._target_count += 1
        else: self._lost_count += 1
        self._cached_data['x'] = x
        self._cached_data['y'] = y
        self._cached_data['label'] = label_val
        self._cached_data['id'] = id
        self._cached_data['flag'] = flag
        self._cached_data['status'] = status_val
        self._cached_data['b6'] = b6
        self._cached_data['b7'] = b7
        self._cached_data['line_flag'] = line_flag_val
        self._cached_data['is_target'] = is_target
        return self._cached_data

    def read_block(self, timeout_ms=100):
        deadline = time.ticks_ms() + timeout_ms
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            data = self.read()
            if data is not None: return data
            time.sleep_ms(1)
        return None

    def flush(self):
        while self._uart.any(): self._uart.read()
        self._buf = bytearray()

    def deinit(self):
        self._uart = None
