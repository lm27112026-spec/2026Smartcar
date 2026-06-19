import sensor, image, time, math, tf, ustruct
from machine import UART

# ==========================================
# 1. 空间标定参数 (必须根据机械结构实际测量)
# ==========================================
H = 20.3# 摄像头镜头中心距离地面的垂直高度 (cm)
Y_OFFSET = 4.0            # 摄像头距离推杆中点的物理纵向距离 (cm)
PITCH_ANGLE = 15.0        # 摄像头向下的俯角 (度)
theta = math.radians(PITCH_ANGLE) # 转换为弧度

# ==========================================
# 2. 摄像头与模型初始化设置
# ==========================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # 160 x 120
# 裁切为正方形以适配模型输入，画面变为 120 x 120
sensor.set_windowing((120, 120))
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# --- 串口初始化 ---
uart = UART(7, baudrate=115200)
SCALE = 10  # 串口精度：乘以10保留1位小数

# 加载模型到帧缓冲区(加速)
face_detect = '/sd/yolo3_iou_smartcar_final_with_post_processing.tflite'
net = tf.load(face_detect, load_to_fb=True)

# ==========================================
# 3. 摄像头内参比例系数
# ==========================================
IMG_WIDTH = 120
IMG_HEIGHT = 120
CENTER_X = IMG_WIDTH / 2  # 60
CENTER_Y = IMG_HEIGHT / 2 # 60

# 每个像素代表的弧度值 (因裁剪中心，单像素视野FOV不变)
K_y = math.radians(40.0 / 120.0)  # 垂直方向单像素弧度
K_x = math.radians(54.0 / 160.0)  # 水平方向单像素弧度 (原宽160，保留原比例)

# ==========================================
# 4. 生命周期与状态机管理参数
# ==========================================
MAX_LIFE = 5               # 容错生命值：连续多少帧没看到才算丢失 (可根据帧率调大调小)
TRACK_THRESH = 30.0        # 追踪阈值：新一帧的目标距离上一帧多近，才认为是同一个物体 (像素)

target_life = 0            # 当前目标的剩余生命值
locked_target = None       # 存储当前锁定目标的字典


while(True):
    img = sensor.snapshot()

    # 获取当前帧的所有检测结果
    detections = tf.detect(net, img)

    # ---------------------------------------------------------
    # 状态 1：优先追踪锁定 (防跳动与防漏检)
    # ---------------------------------------------------------
    if locked_target is not None:
        best_match = None
        min_dist = float('inf')

        for obj in detections:
            x1, y1, x2, y2, label, scores = obj
            if scores > 0.70:
                # 解析坐标
                px_x = int((x1 - 0.1) * img.width()) + int((x2 - x1) * img.width()) / 2.0
                px_y = int(y2 * img.height())

                # 计算与上一帧锁定目标的像素欧氏距离
                dist = math.sqrt((px_x - locked_target['px_x'])**2 + (px_y - locked_target['px_y'])**2)

                # 必须满足距离最小，且在追踪阈值范围内
                if dist < min_dist and dist < TRACK_THRESH:
                    min_dist = dist
                    best_match = {
                        'draw_x': int((x1 - 0.1) * img.width()),
                        'draw_y': int(y1 * img.height()),
                        'draw_w': int((x2 - x1) * img.width()),
                        'draw_h': int((y2 - y1) * img.height()),
                        'px_x': px_x, 'px_y': px_y, 'label': label, 'scores': scores
                    }

        if best_match is not None:
            # 追踪成功：更新坐标，生命值拉满
            locked_target = best_match
            target_life = MAX_LIFE
        else:
            # 追踪失败(可能被遮挡或漏检)：扣除生命值
            target_life -= 1
            if target_life <= 0:
                # 生命值耗尽，彻底丢弃该目标，下一帧将触发全局搜索
                locked_target = None

    # ---------------------------------------------------------
    # 状态 2：全局搜索与过滤 (仅在未锁定目标时执行)
    # ---------------------------------------------------------
    if locked_target is None:
        best_new_target = None
        min_bottom_dist_sq = float('inf')

        for obj in detections:
            x1, y1, x2, y2, label, scores = obj
            if scores > 0.70:
                px_x = int((x1 - 0.1) * img.width()) + int((x2 - x1) * img.width()) / 2.0
                px_y = int(y2 * img.height())

                # 你的多目标过滤逻辑：计算到“画面正底部中心”的距离平方
                dist_sq = (px_x - CENTER_X)**2 + (IMG_HEIGHT - px_y)**2

                if dist_sq < min_bottom_dist_sq:
                    min_bottom_dist_sq = dist_sq
                    best_new_target = {
                        'draw_x': int((x1 - 0.1) * img.width()),
                        'draw_y': int(y1 * img.height()),
                        'draw_w': int((x2 - x1) * img.width()),
                        'draw_h': int((y2 - y1) * img.height()),
                        'px_x': px_x, 'px_y': px_y, 'label': label, 'scores': scores
                    }

        if best_new_target is not None:
            # 搜索到新目标：直接锁定，并赋予满格生命值
            locked_target = best_new_target
            target_life = MAX_LIFE

    # ---------------------------------------------------------
    # 状态 3：状态输出与坐标解算 (无论真检测到还是靠生命值硬撑)
    # ---------------------------------------------------------
    if locked_target is not None:
        px_x = locked_target['px_x']
        px_y = locked_target['px_y']
        draw_x = locked_target['draw_x']
        draw_y = locked_target['draw_y']

        # UI 绘制 (如果是追踪状态画绿色框，靠生命值硬撑的盲目状态可以考虑画黄色，这里统一画绿)
        if target_life == MAX_LIFE:
            box_color = (0, 255, 0) # 绿色代表实时追踪到
        else:
            box_color = (255, 255, 0) # 黄色代表当前帧漏检，靠生命值记忆维持

        img.draw_rectangle((draw_x, draw_y, locked_target['draw_w'], locked_target['draw_h']), thickness=2, color=box_color)
        img.draw_cross(int(px_x), int(px_y), color=(255, 0, 0), size=5)
        img.draw_string(draw_x, max(0, draw_y-10), f"id:{int(locked_target['label'])} HP:{target_life}", color=box_color)

        # --- 开始坐标解算 (原封不动保留你的算法) ---
        alpha = (px_y - CENTER_Y) * K_y
        gamma = theta + alpha

        if gamma > 0.01:
            # 物理坐标系推导
            Y_cam = H / math.tan(gamma)
            L = H / math.sin(gamma)
            X_cam = (px_x - CENTER_X) * K_x * L

            # 转换至推杆原点
            X_push = X_cam
            Y_push = Y_cam - Y_OFFSET

            # ==========================================
            # 串口数据打包与发送逻辑
            # 协议: 0xAA + x(2B) + y(2B) + label(2B) + status(2B) + 0xBB = 10 字节
            # 每个值 = 原始值 × SCALE，大端序 int16
            # status: 1=识别成功, 0=识别失败
            # 下位机: (high<<8)|low → /10 恢复实际值（精度0.1）
            # ==========================================
            x_int = int(X_push * SCALE)
            y_int = int(Y_push * SCALE)
            label_int = int(locked_target['label'])
            status = 1  # 识别成功标识

            payload = ustruct.pack('>hhhh', x_int, y_int, label_int, status)

            # 组合帧头 0xAA 和 帧尾 0xBB (总长度 1 + 8 + 1 = 10 字节)
            data_packet = b'\xAA' + payload + b'\xBB'
            uart.write(data_packet)

            # 屏幕打印
            coord_str = "X:{:.1f} Y:{:.1f}".format(X_push, Y_push)
            print(f"Target ID:{int(locked_target['label'])} | HP:{target_life} | {coord_str}")
        else:                       # 识别成功但超出解算范围
            uart.write(b'\xAA\x00\x00\x00\x00\x00\x00\x00\x00\xBB')

    else:                       # 丢失目标 (status=0 识别失败)
        uart.write(b'\xAA\x00\x00\x00\x00\x00\x00\x00\x00\xBB')
