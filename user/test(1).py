import sensor, image, time, math, tf, ustruct
from machine import UART

# ==========================================
# 1. 空间标定参数 (必须根据机械结构实际测量)
# ==========================================
H = 15.0
Y_OFFSET = 10.0
PITCH_ANGLE = 30.0
theta = math.radians(PITCH_ANGLE)

# ==========================================
# [新增] 5. 黄线识别参数
# ==========================================
# 需要根据实际场地的灯光使用 OpenMV IDE 的阈值编辑器微调 LAB 范围
YELLOW_THRESH = (30, 100, -20, 20, 30, 80)
LINE_BOTTOM_THRESH = 115  # 判定线：色块底部 Y 坐标大于此值(接近120)，认为正在跨越画面底部

BLIND_TOLERANCE = 3        # 遮挡容忍帧数：连续丢失几帧才确认真的越界了
MIN_LINE_WIDTH = 30        # 真实黄线最小宽度（像素）：过滤细小的黄色噪点

# 状态机变量
armed_to_trigger = False   # 越线触发武装状态
yellow_lost_count = 0      # 丢失帧计数器

# ==========================================
# 2. 摄像头与模型初始化设置
# ==========================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_windowing((120, 120))
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# --- 串口初始化 ---
uart = UART(2, baudrate=115200)
SCALE = 10

# 加载模型
face_detect = '/sd/yolo3_ciou_smartcar_final_with_post_processing.tflite'
net = tf.load(face_detect, load_to_fb=True)


# ==========================================
# 3. 摄像头内参比例系数
# ==========================================
IMG_WIDTH = 120
IMG_HEIGHT = 120
CENTER_X = IMG_WIDTH / 2
CENTER_Y = IMG_HEIGHT / 2

K_y = math.radians(40.0 / 120.0)
K_x = math.radians(54.0 / 160.0)

# ==========================================
# 4. 生命周期与状态机管理参数
# ==========================================
MAX_LIFE = 4
TRACK_THRESH = 30.0

target_life = 0
locked_target = None

# [新增] 黄线越界状态机变量
yellow_at_bottom_history = False # 记录上一帧黄线是否在底部

while(True):
    img = sensor.snapshot()

    # ---------------------------------------------------------
    # [新增] 独立状态机：黄线检测与越界触发
    # ---------------------------------------------------------
    # 寻找黄线色块 (合并连通域以应对断线情况)
    yellow_blobs = img.find_blobs([YELLOW_THRESH], roi=(0, 60, 120, 60),
                                  pixels_threshold=50, area_threshold=50, merge=True)

    current_yellow_touch_bottom = False

    if yellow_blobs:
        # 找到所有符合条件的色块中，最靠下的那一个
        bottom_blob = max(yellow_blobs, key=lambda b: b.y() + b.h())

        # 优化2：宽度校验（即使中间被遮挡断成两截，由于我们开启了 merge=True，它会尽量合体）
        # 如果断得太开没合并上，起码保证剩下的半截也有一定宽度，否则视为噪点
        if bottom_blob.w() > MIN_LINE_WIDTH:
            img.draw_rectangle(bottom_blob.rect(), color=(255, 255, 0), thickness=2)

            # 必须极其贴近画面最底部，才算真正压底，进入“武装”状态
            if bottom_blob.y() + bottom_blob.h() >= LINE_BOTTOM_THRESH:
                current_yellow_touch_bottom = True

    # 优化3：抗遮挡消抖状态机
    line_flag = 0

    if current_yellow_touch_bottom:
        # 线确确实实压在了最底边 -> 进入武装状态，并清零丢失计数器
        armed_to_trigger = True
        yellow_lost_count = 0
    elif armed_to_trigger:
        # 线不再压底（可能是真正越过了，也可能是中途被挡住了！）
        yellow_lost_count += 1

        if yellow_lost_count >= BLIND_TOLERANCE:
            # 连续好几帧都没看到压底的黄线，确认不是短暂遮挡，是真的越过了！
            line_flag = 1
            # 状态重置，等待下一根黄线
            armed_to_trigger = False
            yellow_lost_count = 0


    # ---------------------------------------------------------
    # 状态 1 & 2：目标追踪与全局搜索 (保持原逻辑)
    # ---------------------------------------------------------
    detections = tf.detect(net, img)

    if locked_target is not None:
        best_match = None
        min_dist = float('inf')

        for obj in detections:
            x1, y1, x2, y2, label, scores = obj
            if scores > 0.70:
                px_x = int(x1 * img.width()) + int((x2 - x1) * img.width()) / 2.0
                px_y = int(y2 * img.height())
                dist_sq = (px_x - CENTER_X)**2 + (IMG_HEIGHT - px_y)**2

                if dist_sq < min_bottom_dist_sq:
                    min_bottom_dist_sq = dist_sq
                    best_new_target = {
                        'draw_x': int(x1 * img.width()),
                        'draw_y': int(y1 * img.height()),
                        'draw_w': int((x2 - x1) * img.width()),
                        'draw_h': int((y2 - y1) * img.height()),
                        'px_x': px_x, 'px_y': px_y, 'label': label, 'scores': scores
                    }

        if best_match is not None:
            locked_target = best_match
            target_life = MAX_LIFE
        else:
            target_life -= 1
            if target_life <= 0:
                locked_target = None

    if locked_target is None:
        best_new_target = None
        min_bottom_dist_sq = float('inf')

        for obj in detections:
            x1, y1, x2, y2, label, scores = obj
            if scores > 0.70:
                px_x = int((x1) * img.width()) + int((x2 - x1) * img.width()) / 2.0
                px_y = int(y2 * img.height())
                dist_sq = (px_x - CENTER_X)**2 + (IMG_HEIGHT - px_y)**2

                if dist_sq < min_bottom_dist_sq:
                    min_bottom_dist_sq = dist_sq
                    best_new_target = {
                        'draw_x': int((x1) * img.width()),
                        'draw_y': int(y1 * img.height()),
                        'draw_w': int((x2 - x1) * img.width()),
                        'draw_h': int((y2 - y1) * img.height()),
                        'px_x': px_x, 'px_y': px_y, 'label': label, 'scores': scores
                    }

        if best_new_target is not None:
            locked_target = best_new_target
            target_life = MAX_LIFE

    # ---------------------------------------------------------
    # 状态 3：状态输出与坐标解算
    # ---------------------------------------------------------
    if locked_target is not None:
        px_x = locked_target['px_x']
        px_y = locked_target['px_y']
        draw_x = locked_target['draw_x']
        draw_y = locked_target['draw_y']

        if target_life == MAX_LIFE:
            box_color = (0, 255, 0)
        else:
            box_color = (255, 255, 0)

        img.draw_rectangle((draw_x, draw_y, locked_target['draw_w'], locked_target['draw_h']), thickness=2, color=box_color)
        img.draw_cross(int(px_x), int(px_y), color=(255, 0, 0), size=5)
        img.draw_string(draw_x, max(0, draw_y-10), f"id:{int(locked_target['label'])} HP:{target_life}", color=box_color)

        alpha = (px_y - CENTER_Y) * K_y
        gamma = theta + alpha

        if gamma > 0.01:
            Y_cam = H / math.tan(gamma)
            L = H / math.sin(gamma)
            X_cam = (px_x - CENTER_X) * K_x * L

            X_push = X_cam
            Y_push = Y_cam - Y_OFFSET

            # ==========================================
            # 串口数据打包 (已更新)
            # 协议: 0xAA + x(2B) + y(2B) + label(2B) + status(2B) + line_flag(2B) + 0xBB = 12 字节
            # 注意: line_flag 附加在原有数据包的最后侧
            # ==========================================
            x_int = int(X_push * SCALE)
            y_int = int(Y_push * SCALE)
            label_int = int(locked_target['label'])
            status = 1

            payload = ustruct.pack('>hhhhh', x_int, y_int, label_int, status, line_flag)
            data_packet = b'\xAA' + payload + b'\xBB'
            uart.write(data_packet)

            coord_str = "X:{:.1f} Y:{:.1f}".format(X_push, Y_push)
        else:
            # [修改点] 目标丢失但依然打包 line_flag
            payload = ustruct.pack('>hhhhh', 0, 0, 0, 0, line_flag)
            uart.write(b'\xAA' + payload + b'\xBB')

    else:
        # [修改点] 目标丢失但依然打包 line_flag
        payload = ustruct.pack('>hhhhh', 0, 0, 0, 0, line_flag)
        uart.write(b'\xAA' + payload + b'\xBB')
