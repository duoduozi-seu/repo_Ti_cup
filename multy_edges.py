from maix import image, camera, display, app, time, uart
import numpy as np 
import cv2
from dataclasses import dataclass
import math
from maix import gpio, pinmap

# 初始化串口
device = "/dev/ttyS0"

pinmap.set_pin_function("A29", "GPIOA29")
led = gpio.GPIO("GPIOA29", gpio.Mode.OUT)

led.value(1)

# 使用400×240分辨率
cam = camera.Camera(400, 240)
disp = display.Display()

# 预定义变量避免重复分配内存
kernel = np.ones((3,3), np.uint8) 
img_gray = np.zeros((240, 400), dtype=np.uint8)
img_blur = np.zeros((240, 400), dtype=np.uint8)
img_morph = np.zeros((240, 400), dtype=np.uint8)

# 轮廓处理参数
MIN_AREA = 1000  # 提高最小面积阈值以适应更高分辨率
ROI_HEIGHT = 200  # 只处理中间180行像素
ROI_Y = 20       # 从第30行开始

# 中心点对齐参数
CENTER_X = 200    # 屏幕中心X坐标 (400/2)
CENTER_Y = 118    # 屏幕中心Y坐标 (240/2)
ALIGN_THRESHOLD = 3  # 中心点对齐阈值(像素)
MAX_ITERATIONS = 150  # 最大对齐迭代次数

# 矩形实际尺寸 (单位: cm)
REAL_WIDTH = 25.4
REAL_HEIGHT = 17.4

# 电机控制参数
# 常量定义
MOTOR_ADDRESS_1 = 0x02
MOTOR_ADDRESS_2 = 0x01
COMMAND_CODE = 0xFB
DIR_CW = 0x01
DIR_CCW = 0x00
ABSOLUTE_MODE = 0x01  # 绝对位置模式
SYNC_DISABLED = 0x00

global RPM, RPM_ANGLE_LOW
RPM = 350
RPM_ANGLE_LOW = 200

# 全局变量维护当前角度
current_angle_x = 0.0  # X轴初始角度 (0-360度)
current_angle_y = 0.0  # Y轴初始角度 (-90-90度)

# 添加蓝色激光点的LAB颜色阈值
LASER_THRESHOLD_IN = (95, 100, -13, 9, -13, -7)  # L,A,B范围 - 激光点
LASER_THRESHOLD_OUT = (72, 100, -2, 64, -20, 1)

# 数据类 - 存储透视变换信息
@dataclass
class PerspectiveTransform:
    """存储透视变换信息的数据类"""
    matrix: np.ndarray  # 3x3变换矩阵
    scale_factor: float  # 像素到厘米的转换因子
    world_points: list   # 世界坐标系的四个角点 (cm)
    image_points: list   # 图像中的四个角点 (像素)

# 辅助函数 - 生成X轴命令帧（不发送）
def get_x_command_frame(x_offset):
    global current_angle_x
    ANGLE_RANGE = (-360, 360)

    # 计算角度变化
    if x_offset > 0:
        angle_delta = -0.8
    elif x_offset < 0:
        angle_delta = 0.8
    else:
        angle_delta = 0

    # 更新角度
    new_angle = current_angle_x + angle_delta
    new_angle = max(ANGLE_RANGE[0], min(ANGLE_RANGE[1], new_angle))

    # 计算脉冲数
    pulse_count = int(new_angle * 10)
    direction = DIR_CW if pulse_count >= 0 else DIR_CCW
    pulse_count = abs(pulse_count)

    # 构建命令帧
    command_frame = bytearray([
        MOTOR_ADDRESS_1,      # 地址
        COMMAND_CODE,          # 命令码
        direction,             # 方向
        (RPM >> 8) & 0xFF,    # 速度高字节
        RPM & 0xFF,           # 速度低字节
        (pulse_count >> 24) & 0xFF,  # 脉冲数字节3
        (pulse_count >> 16) & 0xFF,  # 脉冲数字节2
        (pulse_count >> 8) & 0xFF,   # 脉冲数字节1
        pulse_count & 0xFF,          # 脉冲数字节0
        ABSOLUTE_MODE,        # 绝对位置模式
        SYNC_DISABLED,        # 多机同步标志
        0x6B                  # 固定校验值
    ])

    # 更新当前角度
    current_angle_x = new_angle
    return command_frame

# 辅助函数 - 生成Y轴命令帧（不发送）
def get_y_command_frame(y_offset):
    global current_angle_y
    ANGLE_RANGE = (-90, 90)

    # 计算角度变化
    if y_offset > 0:
        angle_delta = 0.8
    elif y_offset < 0:
        angle_delta = -0.8
    else:
        angle_delta = 0

    # 更新角度
    new_angle = current_angle_y + angle_delta
    new_angle = max(ANGLE_RANGE[0], min(ANGLE_RANGE[1], new_angle))

    # 计算脉冲数
    pulse_count = int(new_angle * 10)
    direction = DIR_CW if pulse_count >= 0 else DIR_CCW
    pulse_count = abs(pulse_count)

    # 构建命令帧
    command_frame = bytearray([
        MOTOR_ADDRESS_2,      # 地址
        COMMAND_CODE,          # 命令码
        direction,             # 方向
        (RPM >> 8) & 0xFF,    # 速度高字节
        RPM & 0xFF,           # 速度低字节
        (pulse_count >> 24) & 0xFF,  # 脉冲数字节3
        (pulse_count >> 16) & 0xFF,  # 脉冲数字节2
        (pulse_count >> 8) & 0xFF,   # 脉冲数字节1
        pulse_count & 0xFF,          # 脉冲数字节0
        ABSOLUTE_MODE,        # 绝对位置模式
        SYNC_DISABLED,        # 多机同步标志
        0x6B                  # 固定校验值
    ])

    # 更新当前角度
    current_angle_y = new_angle
    return command_frame

def set_x_offset(x_offset, uart):
    set_xy_offsets(uart, x_offset, 0)

def set_x_angle(angle, uart):
    set_xy_angles(uart, angle, current_angle_y)

def set_y_offset(y_offset, uart):
    set_xy_offsets(uart, 0, y_offset)

def set_y_angle(angle, uart):
    set_xy_angles(uart, current_angle_x, angle)

def set_xy_offsets(uart, x_offset, y_offset):
    global current_angle_x, current_angle_y

    # 1. 获取X轴电机命令帧
    x_frame = get_x_command_frame(x_offset)

    # 2. 获取Y轴电机命令帧
    y_frame = get_y_command_frame(y_offset)

    # 3. 构建29字节的完整帧
    full_frame = bytearray([
        0x00, 0xAA,       # 帧头
        0x00, 0x1D,       # 长度 (29 = 0x1D)
    ])
    full_frame.extend(x_frame)  # 添加X轴命令 (12字节)
    full_frame.extend(y_frame)  # 添加Y轴命令 (12字节)
    full_frame.append(0x6B)     # 结束校验码
    full_frame.append(0x3E)

    # 4. 通过串口发送完整帧
    uart.write(bytes(full_frame))

    # 5. 打印调试信息
    # print(f"发送XY联合命令: {full_frame.hex()}")
    print(f"X轴更新到: {current_angle_x:.2f}°, Y轴更新到: {current_angle_y:.2f}°")

# 辅助函数 - 根据目标角度生成X轴命令帧
def get_x_command_frame_from_angle(x_angle):
    global current_angle_x, current_angle_y

    ANGLE_RANGE = (-360, 1080)

    # 确保角度在范围内
    x_angle = max(ANGLE_RANGE[0], min(ANGLE_RANGE[1], x_angle))

    # 计算脉冲数
    pulse_count = int(x_angle * 10)
    direction = DIR_CW if pulse_count >= 0 else DIR_CCW
    pulse_count = abs(pulse_count)

    delta = x_angle - current_angle_x
    delta = abs(delta)
    # print(delta)

    if delta >= 20:
    # 构建命令帧
        command_frame = bytearray([
            MOTOR_ADDRESS_1,      # 地址
            COMMAND_CODE,          # 命令码
            direction,             # 方向
            (RPM_ANGLE_LOW >> 8) & 0xFF,    # 速度高字节
            RPM_ANGLE_LOW & 0xFF,           # 速度低字节
            (pulse_count >> 24) & 0xFF,  # 脉冲数字节3
            (pulse_count >> 16) & 0xFF,  # 脉冲数字节2
            (pulse_count >> 8) & 0xFF,   # 脉冲数字节1
            pulse_count & 0xFF,          # 脉冲数字节0
            ABSOLUTE_MODE,        # 绝对位置模式
            SYNC_DISABLED,        # 多机同步标志
            0x6B                  # 固定校验值
        ])
        # print('低速')
    else:
        # 构建命令帧
        command_frame = bytearray([
            MOTOR_ADDRESS_1,      # 地址
            COMMAND_CODE,          # 命令码
            direction,             # 方向
            (RPM >> 8) & 0xFF,    # 速度高字节
            RPM & 0xFF,           # 速度低字节
            (pulse_count >> 24) & 0xFF,  # 脉冲数字节3
            (pulse_count >> 16) & 0xFF,  # 脉冲数字节2
            (pulse_count >> 8) & 0xFF,   # 脉冲数字节1
            pulse_count & 0xFF,          # 脉冲数字节0
            ABSOLUTE_MODE,        # 绝对位置模式
            SYNC_DISABLED,        # 多机同步标志
            0x6B                  # 固定校验值
        ])
        # print('高速')

    return command_frame

# 辅助函数 - 根据目标角度生成Y轴命令帧
def get_y_command_frame_from_angle(y_angle):
    global current_angle_x, current_angle_y
    ANGLE_RANGE = (-45, 45)

    # 确保角度在范围内
    y_angle = max(ANGLE_RANGE[0], min(ANGLE_RANGE[1], y_angle))

    # 计算脉冲数
    pulse_count = int(y_angle * 10)
    direction = DIR_CW if pulse_count >= 0 else DIR_CCW
    pulse_count = abs(pulse_count)

    delta = y_angle - current_angle_y
    delta = abs(delta)
    # print(delta)

    if delta >= 20:
    # 构建命令帧
        command_frame = bytearray([
            MOTOR_ADDRESS_2,      # 地址
            COMMAND_CODE,          # 命令码
            direction,             # 方向
            (RPM_ANGLE_LOW >> 8) & 0xFF,    # 速度高字节
            RPM_ANGLE_LOW & 0xFF,           # 速度低字节
            (pulse_count >> 24) & 0xFF,  # 脉冲数字节3
            (pulse_count >> 16) & 0xFF,  # 脉冲数字节2
            (pulse_count >> 8) & 0xFF,   # 脉冲数字节1
            pulse_count & 0xFF,          # 脉冲数字节0
            ABSOLUTE_MODE,        # 绝对位置模式
            SYNC_DISABLED,        # 多机同步标志
            0x6B                  # 固定校验值
        ])
        # print('低速')
    else:
        # 构建命令帧
        command_frame = bytearray([
            MOTOR_ADDRESS_2,      # 地址
            COMMAND_CODE,          # 命令码
            direction,             # 方向
            (RPM >> 8) & 0xFF,    # 速度高字节
            RPM & 0xFF,           # 速度低字节
            (pulse_count >> 24) & 0xFF,  # 脉冲数字节3
            (pulse_count >> 16) & 0xFF,  # 脉冲数字节2
            (pulse_count >> 8) & 0xFF,   # 脉冲数字节1
            pulse_count & 0xFF,          # 脉冲数字节0
            ABSOLUTE_MODE,        # 绝对位置模式
            SYNC_DISABLED,        # 多机同步标志
            0x6B                  # 固定校验值
        ])
        # print('高速')

    return command_frame

def set_xy_angles(uart, x_angle, y_angle):
    global current_angle_x, current_angle_y

    # 1. 获取X轴电机命令帧
    x_frame = get_x_command_frame_from_angle(x_angle)

    # 2. 获取Y轴电机命令帧
    y_frame = get_y_command_frame_from_angle(y_angle)

    # 3. 构建29字节的完整帧
    full_frame = bytearray([
        0x00, 0xAA,       # 帧头
        0x00, 0x1D,       # 长度 (29 = 0x1D)
    ])
    full_frame.extend(x_frame)  # 添加X轴命令 (12字节)
    full_frame.extend(y_frame)  # 添加Y轴命令 (12字节)
    full_frame.append(0x6B)     # 结束校验码
    full_frame.append(0x3E)

    # 4. 通过串口发送完整帧
    uart.write(bytes(full_frame))

    # 5. 更新当前角度
    current_angle_x = x_angle
    current_angle_y = y_angle

    # 6. 打印调试信息
    # print(f"发送XY联合角度命令: {full_frame.hex()}")
    print(f"X轴设置到: {current_angle_x:.2f}°, Y轴设置到: {current_angle_y:.2f}°")

def set_xy_zero(uart, x_angle, y_angle):
    global current_angle_x, current_angle_y
    current_angle_x = 20
    current_angle_y = 20

    # 1. 获取X轴电机命令帧
    x_frame = get_x_command_frame_from_angle(x_angle)

    # 2. 获取Y轴电机命令帧
    y_frame = get_y_command_frame_from_angle(y_angle)

    # 3. 构建29字节的完整帧
    full_frame = bytearray([
        0x00, 0xAA,       # 帧头
        0x00, 0x1D,       # 长度 (29 = 0x1D)
    ])
    full_frame.extend(x_frame)  # 添加X轴命令 (12字节)
    full_frame.extend(y_frame)  # 添加Y轴命令 (12字节)
    full_frame.append(0x6B)     # 结束校验码
    full_frame.append(0x3E)
    # 4. 通过串口发送完整帧
    uart.write(bytes(full_frame))

    # 5. 更新当前角度
    current_angle_x = x_angle
    current_angle_y = y_angle

    # 6. 打印调试信息
    # print(f"发送XY联合角度命令: {full_frame.hex()}")
    print(f"X轴设置到: {current_angle_x:.2f}°, Y轴设置到: {current_angle_y:.2f}°")

def sort_corners(corners):
    """
    对四边形角点进行排序（左上、右上、右下、左下）
    
    参数:
        corners: 四个角点的列表 [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
    
    返回:
        sorted_corners: 排序后的角点列表
    """
    # 将角点转换为numpy数组
    pts = np.array(corners, dtype="float32")
    
    # 计算中心点
    center = np.mean(pts, axis=0)
    
    # 按x坐标排序
    x_sorted = pts[np.argsort(pts[:, 0]), :]
    
    # 获取左右两边的点
    left_pts = x_sorted[:2, :]
    right_pts = x_sorted[2:, :]
    
    # 左上和左下（按y坐标排序）
    left_pts = left_pts[np.argsort(left_pts[:, 1]), :]
    (tl, bl) = left_pts
    
    # 右上和右下（按y坐标排序）
    right_pts = right_pts[np.argsort(right_pts[:, 1]), :]
    (tr, br) = right_pts
    
    # 返回排序后的点 (左上, 右上, 右下, 左下)
    return [tl.tolist(), tr.tolist(), br.tolist(), bl.tolist()]

def calculate_perspective_transform(corners):
    """
    根据检测到的矩形角点计算透视变换
    
    参数:
        corners: 排序后的四个角点 [[x1,y1], [x2,y2], [x3,y3], [x4,y4]]
    
    返回:
        transform: PerspectiveTransform对象
    """
    # 将角点转换为numpy数组
    image_points = np.array(corners, dtype="float32")
    
    # 计算矩形在图像中的宽度和高度
    (tl, tr, br, bl) = image_points
    
    # 自定义欧几里得距离计算函数
    def euclidean_distance(pt1, pt2):
        return np.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
    
    widthA = euclidean_distance(tl, tr)
    widthB = euclidean_distance(bl, br)
    heightA = euclidean_distance(tl, bl)
    heightB = euclidean_distance(tr, br)
    
    # 计算平均尺寸
    avg_width = (widthA + widthB) / 2.0
    avg_height = (heightA + heightB) / 2.0
    
    # 计算比例因子（像素到厘米）
    width_scale = REAL_WIDTH / avg_width
    height_scale = REAL_HEIGHT / avg_height
    scale_factor = (width_scale + height_scale) / 2.0
    
    # 定义目标矩形（世界坐标系，以矩形中心为原点）
    world_points = np.array([
        [-REAL_WIDTH/2, REAL_HEIGHT/2],   # 左上
        [REAL_WIDTH/2, REAL_HEIGHT/2],    # 右上
        [REAL_WIDTH/2, -REAL_HEIGHT/2],   # 右下
        [-REAL_WIDTH/2, -REAL_HEIGHT/2]   # 左下
    ], dtype="float32")
    
    # 计算透视变换矩阵
    matrix = cv2.getPerspectiveTransform(image_points, world_points)
    
    # 创建并返回变换对象
    return PerspectiveTransform(
        matrix=matrix,
        scale_factor=scale_factor,
        world_points=world_points.tolist(),
        image_points=image_points.tolist()
    )

def apply_perspective_transform(transform, point):
    """
    应用透视变换将图像点转换为世界坐标
    
    参数:
        transform: PerspectiveTransform对象
        point: 图像中的点 (x, y)
    
    返回:
        world_point: 世界坐标系中的点 (x, y) 单位: cm
    """
    # 转换为齐次坐标
    homogeneous_point = np.array([point[0], point[1], 1], dtype="float32")
    
    # 应用变换
    transformed = np.dot(transform.matrix, homogeneous_point)
    
    # 转换为笛卡尔坐标
    world_x = transformed[0] / transformed[2]
    world_y = transformed[1] / transformed[2]
    
    return (world_x, world_y)

def process_quadrilateral_detection(img, serial, frame_counter):
    """
    处理四边形检测的全过程，包括：
    1. 图像预处理和畸变校正
    2. ROI区域处理
    3. 四边形检测
    4. 结果绘制和串口通信
    
    参数:
        img: 原始图像 (maix.image对象)
        serial: 串口对象
        frame_counter: 当前帧计数器
        
    返回:
        result: 包含检测结果的字典，包含以下键:
            'corners': 四边形的四个角点坐标 (列表, 每个元素为[x,y])
            'center': 矩形中心点坐标 (x, y)
            'size': 矩形的宽和高 (w, h)
            'found': 是否检测到四边形 (布尔值)
    """
    # 初始化结果字典
    result = {
        'corners': [],
        'center': (0, 0),
        'size': (0, 0),
        'found': False
    }
    
    # 镜头畸变校正
    img_raw = image.image2cv(img)
    
    # 跳过部分帧以提高帧率
    if frame_counter != 0:
        # 标记ROI区域
        cv2.rectangle(img_raw, (0, ROI_Y), (400, ROI_Y+ROI_HEIGHT), (255, 0, 0), 1)
        disp.show(image.cv2image(img_raw))
        return result
    
    # 灰度转换 (只处理ROI区域)
    roi_gray = img_gray[ROI_Y:ROI_Y+ROI_HEIGHT, :]
    cv2.cvtColor(img_raw[ROI_Y:ROI_Y+ROI_HEIGHT], cv2.COLOR_BGR2GRAY, dst=roi_gray)
    
    # 快速滤波
    roi_blur = img_blur[ROI_Y:ROI_Y+ROI_HEIGHT, :]
    cv2.GaussianBlur(roi_gray, (3, 3), 0, dst=roi_blur)
    
    # 形态学处理
    roi_morph = img_morph[ROI_Y:ROI_Y+ROI_HEIGHT, :]
    cv2.morphologyEx(roi_blur, cv2.MORPH_CLOSE, kernel, dst=roi_morph)
    
    # 边缘检测 (只处理ROI区域)
    edged = cv2.Canny(roi_morph, 100, 200)
    
    # 查找轮廓
    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 查找最大矩形
    max_rect = None
    max_area = MIN_AREA
    
    for contour in contours:
        # 快速面积过滤
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue
            
        # 简化多边形近似
        epsilon = 0.03 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # 只处理四边形
        if len(approx) == 4:
            # 检查是否为凸多边形
            if not cv2.isContourConvex(approx):
                continue
                
            if area > max_area:
                max_area = area
                max_rect = approx
    
    # 如果找到矩形
    if max_rect is not None:
        # 调整坐标偏移 (ROI区域)
        offset_rect = np.array(max_rect) + [0, ROI_Y]
        
        # 直接计算边界框
        x, y, w, h = cv2.boundingRect(offset_rect)
        cx = x + w // 2
        cy = y + h // 2
        
        # 填充结果字典
        corners = [point[0].tolist() for point in offset_rect]
        # 对角点进行排序
        sorted_corners = sort_corners(corners)
        
        result['corners'] = sorted_corners
        result['center'] = (cx, cy)
        result['size'] = (w, h)
        result['found'] = True
        
        # 绘制轮廓和角点 (在全图上)
        cv2.drawContours(img_raw, [offset_rect], 0, (0, 255, 0), 2)
        
        # 绘制边界框
        # cv2.rectangle(img_raw, (x, y), (x+w, y+h), (0, 0, 255), 1)
        
        # 绘制中心点
        # cv2.circle(img_raw, (cx, cy), 3, (0, 0, 255), -1)
        
        # 在角点上绘制标记
        for i, point in enumerate(sorted_corners):
            x, y = point
            cv2.circle(img_raw, (int(x), int(y)), 4, (255, 0, 0), -1)
            cv2.putText(img_raw, f"{i}", (int(x)+5, int(y)+5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # 准备串口数据
        data = f"Corners: {result['corners']}\n"
        data += f"Center: {result['center']}\n"
        data += f"Size: {result['size']}\n\n"
        # serial.write(data.encode())
    
    # 在图像上标记ROI区域
    cv2.rectangle(img_raw, (0, ROI_Y), (400, ROI_Y+ROI_HEIGHT), (255, 0, 0), 1)
    
    # 显示处理结果
    disp.show(image.cv2image(img_raw))
    
    return result

def scan_for_rectangle(uart, cam, disp, start_angle):
    """
    控制二维云台扫描矩形
    参数:
        uart: 串口对象
        cam: 相机对象
        disp: 显示对象
    返回:
        是否找到矩形 (布尔值)
    """
    # 扫描参数
    SCAN_STEP = 30  # 每次转动角度
    FRAMES_PER_STEP = 5  # 每个位置检测帧数
    CONSECUTIVE_DETECTIONS = 2  # 连续检测到矩形的帧数
    
    # 初始化状态变量
    current_angle = 0
    detection_count = 0
    found = False
    
    print("开始矩形扫描...")
    set_x_angle(start_angle, uart)
    current_angle = start_angle

    # 开始扫描循环
    while current_angle <= 360 and not found and not app.need_exit():
        # 设置云台角度
        print(f"转动到角度: {current_angle}°")
        set_x_angle(current_angle, uart)
        
        # 等待电机移动完成
        time.sleep(0.25)
        
        # 在当前角度采集FRAMES_PER_STEP帧图像
        detection_count = 0  # 重置检测计数器
        for i in range(FRAMES_PER_STEP):
            if app.need_exit():
                return False
                
            # 获取图像
            img = cam.read()
            
            # 处理图像并检测矩形
            # 强制处理每一帧
            result = process_quadrilateral_detection(img, uart, frame_counter=0)
            
            # 更新检测计数器
            if result['found']:
                detection_count += 1
                print(f"位置 {current_angle}°: 检测到矩形 ({detection_count}/{CONSECUTIVE_DETECTIONS})")
            else:
                detection_count = 0
            
            # 检查是否连续检测到足够帧数
            if detection_count >= CONSECUTIVE_DETECTIONS:
                found = True
                print(f"在 {current_angle}° 位置成功检测到矩形!")
                break
            
            # 显示当前帧数
            print(f"位置 {current_angle}°: 处理帧 {i+1}/{FRAMES_PER_STEP}")
        
        # 移动到下一个位置
        current_angle += SCAN_STEP

        if current_angle_x >= start_angle + 70:
            current_angle = start_angle
            set_xy_angles(uart, start_angle, 0)
    
    return found

def align_center(uart, cam, disp):
    """
    识别屏幕中的矩形并控制二维云台将屏幕中心点与四边形中心点对齐（使用增量式PID控制）
    修改流程：
    1. 检测矩形角点
    2. 计算透视变换参数
    3. 将矩形的中心点（世界坐标原点）经过透视逆变换得到像素坐标作为要对准的点
    """
    aligned = False
    iterations = 0
    frame_counter = 0
    
    # 增量式PID控制参数
    Kp = 0.58   # 比例系数
    Ki = 0.005  # 积分系数
    Kd = 0.56   # 微分系数
    
    # 初始化PID状态变量
    prev_error_x = 0
    prev_error_y = 0
    prev_integral_x = 0
    prev_integral_y = 0
    
    # 角度比例系数 (像素到角度的转换因子)
    PIXEL_TO_ANGLE_X = 0.1  # 每像素对应的X轴角度变化
    PIXEL_TO_ANGLE_Y = 0.1  # 每像素对应的Y轴角度变化
    
    print("开始中心点对齐(增量式PID控制)...")
    # print("新流程：通过透视变换计算矩形中心图像坐标")
    
    # 获取初始角度
    current_x_angle, current_y_angle = current_angle_x, current_angle_y
    
    # 初始化透视变换
    perspective = None
    
    while not aligned and iterations < MAX_ITERATIONS and not app.need_exit():
        # 获取图像
        img = cam.read()
        img_raw = image.image2cv(img)
        
        # 处理图像并检测矩形
        result = process_quadrilateral_detection(img, uart, frame_counter)
        # frame_counter = (frame_counter + 1) % 2
        
        if result['found']:
            # 计算当前帧的透视变换
            perspective = calculate_perspective_transform(result['corners'])
            
            # ===== 关键修改：通过逆变换计算矩形中心的图像坐标 =====
            # 矩形中心在世界坐标系中是(0,0)
            world_center = np.array([0, 0, 1], dtype=np.float32)
            
            # 应用逆透视变换得到图像坐标
            image_center_homogeneous = np.dot(np.linalg.inv(perspective.matrix), world_center)
            target_x = image_center_homogeneous[0] / image_center_homogeneous[2]
            target_y = image_center_homogeneous[1] / image_center_homogeneous[2]
            
            # 计算与屏幕中心的偏移量
            error_x = CENTER_X - target_x
            error_y = CENTER_Y - target_y
            
            # 计算偏移距离
            distance = (error_x**2 + error_y**2)**0.5
            
            # 在图像上绘制中心点和偏移线
            cv2.circle(img_raw, (CENTER_X, CENTER_Y), 5, (0, 255, 255), -1)  # 屏幕中心(黄色)
            cv2.circle(img_raw, (int(target_x), int(target_y)), 5, (0, 255, 0), -1)  # 计算出的矩形中心(绿色)
            cv2.line(img_raw, (CENTER_X, CENTER_Y), (int(target_x), int(target_y)), (255, 0, 0), 2)
            
            # 显示偏移信息
            cv2.putText(img_raw, f"DX: {error_x:.1f} DY: {error_y:.1f}", (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(img_raw, f"Dist: {distance:.1f}", (10, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 在图像上显示世界坐标信息
            cv2.putText(img_raw, f"Target: ({target_x:.1f}, {target_y:.1f})", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 检查是否已对齐
            if distance < ALIGN_THRESHOLD:
                aligned = True
                print("中心点对齐成功!")
                break
            
            # 增量式PID控制计算
            # 比例项
            P_x = Kp * error_x
            P_y = Kp * error_y
            
            # 积分项
            I_x = Ki * (error_x + prev_error_x) * 0.5
            I_y = Ki * (error_y + prev_error_y) * 0.5
            
            # 微分项
            D_x = Kd * (error_x - prev_error_x)
            D_y = Kd * (error_y - prev_error_y)
            
            # 计算总控制输出 (角度变化量)
            delta_angle_x = P_x + I_x + D_x
            delta_angle_y = P_y + I_y + D_y
            
            # 转换为角度变化
            delta_angle_x *= PIXEL_TO_ANGLE_X
            delta_angle_y *= PIXEL_TO_ANGLE_Y
            
            # 保存当前误差用于下一次微分计算
            prev_error_x = error_x
            prev_error_y = error_y
            
            # 计算新的绝对角度
            new_x_angle = current_x_angle - delta_angle_x  # X轴反向
            new_y_angle = current_y_angle - delta_angle_y  # Y轴同向
            
            # 限制角度范围
            new_x_angle = max(-360, min(360, new_x_angle))
            new_y_angle = max(-90, min(90, new_y_angle))
            
            # 移动云台到新角度
            set_xy_angles(uart, new_x_angle, new_y_angle)
            
            # 更新当前角度
            current_x_angle, current_y_angle = new_x_angle, new_y_angle
            
        else:
            print("未检测到矩形，无法对齐")
            # 显示警告信息
            cv2.putText(img_raw, "No rectangle detected!", (50, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 重置积分项和微分项（避免累积错误）
            prev_error_x = 0
            prev_error_y = 0
        
        # 显示处理结果
        disp.show(image.cv2image(img_raw))
        iterations += 1
        print(f"对齐迭代: {iterations}/{MAX_ITERATIONS}")
    
    return aligned

def detect_laser_point(img):
    """
    检测图像中的蓝色激光点
    
    参数:
        img: 原始图像 (maix.image对象)
    
    返回:
        (x, y): 激光点中心坐标，如果未检测到返回None
    """
    # 镜头畸变校正
    img_raw = image.image2cv(img)
    
    # 转换为LAB颜色空间
    img_lab = cv2.cvtColor(img_raw, cv2.COLOR_BGR2LAB)
    
    # 分离LAB通道
    l_channel, a_channel, b_channel = cv2.split(img_lab)
    
    # 创建激光点掩膜
    mask = np.zeros((240, 400), dtype=np.uint8)
    
    # 应用两种阈值范围检测激光点
    mask_in = cv2.inRange(img_lab, 
                         (LASER_THRESHOLD_IN[0], LASER_THRESHOLD_IN[2], LASER_THRESHOLD_IN[4]),
                         (LASER_THRESHOLD_IN[1], LASER_THRESHOLD_IN[3], LASER_THRESHOLD_IN[5]))
    
    mask_out = cv2.inRange(img_lab,
                          (LASER_THRESHOLD_OUT[0], LASER_THRESHOLD_OUT[2], LASER_THRESHOLD_OUT[4]),
                          (LASER_THRESHOLD_OUT[1], LASER_THRESHOLD_OUT[3], LASER_THRESHOLD_OUT[5]))
    
    # 合并两个掩膜
    mask = cv2.bitwise_or(mask_in, mask_out)
    
    # 形态学操作增强激光点
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # 找到面积最大的轮廓
        max_contour = max(contours, key=cv2.contourArea)
        
        # 计算轮廓中心
        M = cv2.moments(max_contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
    
    return None

def align_laser_to_target(uart, cam, disp):
    """
    持续控制二维云台将激光点对准目标靶心
    
    参数:
        uart: 串口对象
        cam: 相机对象
        disp: 显示对象
    """
    # PID控制参数
    Kp = 0.05   # 比例系数
    Ki = 0.003  # 积分系数
    Kd = 0.04   # 微分系数
    
    # 角度比例系数
    PIXEL_TO_ANGLE = 0.01  # 每像素对应的角度变化
    
    # 初始化PID状态变量
    prev_error_x = 0
    prev_error_y = 0
    integral_x = 0
    integral_y = 0
    
    # 帧计数器
    frame_counter = 0
    
    # 未检测到目标的计数器
    target_missing_count = 0
    MAX_MISSING_FRAMES = 20  # 连续丢失目标的最大帧数
    
    print("开始激光点对准目标...")
    
    while not app.need_exit():
        # 获取图像
        img = cam.read()
        img_raw = image.image2cv(img)
        
        # 检测矩形目标
        result = process_quadrilateral_detection(img, uart, frame_counter)
        # frame_counter = (frame_counter + 1) % 2
        
        # 检测激光点
        laser_point = detect_laser_point(img)
        
        if result['found'] and laser_point:
            # 重置丢失计数器
            target_missing_count = 0
            
            # 获取目标中心和激光点位置
            target_center = result['center']
            laser_x, laser_y = laser_point
            
            # 计算偏移量
            error_x = target_center[0] - laser_x
            error_y = target_center[1] - laser_y
            
            # 计算偏移距离
            distance = (error_x**2 + error_y**2)**0.5
            
            # 在图像上绘制目标中心、激光点和连接线
            cv2.circle(img_raw, target_center, 5, (0, 255, 0), -1)  # 目标中心(绿色)
            cv2.circle(img_raw, laser_point, 5, (255, 0, 0), -1)    # 激光点(蓝色)
            cv2.line(img_raw, target_center, laser_point, (0, 255, 255), 2)
            
            # 显示偏移信息
            cv2.putText(img_raw, f"DX: {error_x} DY: {error_y}", (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(img_raw, f"Dist: {distance:.1f}", (10, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 检查是否已对准
            if distance < ALIGN_THRESHOLD:
                cv2.putText(img_raw, "ALIGNED!", (150, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                # PID控制计算
                # 比例项
                P_x = Kp * error_x
                P_y = Kp * error_y
                
                # 积分项
                integral_x += error_x
                integral_y += error_y
                I_x = Ki * integral_x
                I_y = Ki * integral_y
                
                # 微分项
                D_x = Kd * (error_x - prev_error_x)
                D_y = Kd * (error_y - prev_error_y)
                
                # 保存当前误差用于下一次微分计算
                prev_error_x = error_x
                prev_error_y = error_y
                
                # 计算总控制输出 (角度变化量)
                delta_angle_x = P_x + I_x + D_x
                delta_angle_y = P_y + I_y + D_y
                
                # 转换为角度变化
                delta_angle_x *= PIXEL_TO_ANGLE
                delta_angle_y *= PIXEL_TO_ANGLE
                
                # 显示PID参数
                cv2.putText(img_raw, f"PID: P:{P_x:.2f} I:{I_x:.2f} D:{D_x:.2f}", (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 获取当前云台角度
                current_x_angle, current_y_angle = current_angle_x, current_angle_y
                
                # 计算新的绝对角度
                # 注意：X轴偏移方向需要取反，因为相机视角与云台运动方向相反
                new_x_angle = current_x_angle - delta_angle_x
                new_y_angle = current_y_angle + delta_angle_y
                
                # 限制角度范围
                new_x_angle = max(0, min(360, new_x_angle))
                new_y_angle = max(-90, min(90, new_y_angle))
                
                # 移动云台到新角度
                set_xy_angles(uart, new_x_angle, new_y_angle)
                
                # 显示角度信息
                cv2.putText(img_raw, f"X: {new_x_angle:.1f}° Y: {new_y_angle:.1f}°", (10, 80), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        else:
            # 目标或激光点未检测到
            target_missing_count += 1
            cv2.putText(img_raw, "Target or laser not detected!", (80, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # 如果连续多帧未检测到目标，尝试重新扫描
            if target_missing_count > MAX_MISSING_FRAMES:
                cv2.putText(img_raw, "Re-scanning for target...", (80, 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                found = scan_for_rectangle(uart, cam, disp, 0)
                target_missing_count = 0  # 重置计数器
                if not found:
                    print("Target not found during re-scan")
                    break
        
        # 显示处理结果
        disp.show(image.cv2image(img_raw))
    
    print("激光点对准任务结束")

def get_position_from_distance(total_distance):
    """
    根据总路程计算车辆在正方形跑道上的位置
    
    参数:
    total_distance: 总路程 (单位: cm)
    
    返回:
    n: 当前圈数 (从1开始)
    x: x坐标 (单位: cm)
    y: y坐标 (单位: cm)
    """
    # 正方形边长 (单位: cm)
    side_length = 100.0
    
    # 一圈总路程 (单位: cm)
    circumference = 4 * side_length
    
    # 计算当前圈数
    n = total_distance // circumference + 1
    
    # 计算当前圈内的路程
    current_distance = total_distance % circumference
    
    # print(current_distance)

    # 确定当前所在边
    if current_distance < side_length:  # 边1: 左侧边
        x = -50
        y = 50 - current_distance
        print("边一")
    elif current_distance < 2 * side_length:  # 边2: 下侧边
        x = -50 + (current_distance - side_length)
        y = -50
        print("边二")
    elif current_distance < 3 * side_length:  # 边3: 右侧边
        x = 50
        y = -50 + (current_distance - 2 * side_length)
        print("边三")
    else:  # 边4: 上侧边
        x = 50 - (current_distance - 3 * side_length)
        y = 50
        print("边四")
    print(f"圈数：{n}, x:{x}, y:{y}")
    return int(n), round(x, 2), round(y, 2)

def calculate_angles_for_perf4(x, y, height, n, target_x=0, target_y=100, target_height=20):
    """
    计算云台的yaw和pitch角度
    
    参数:
    x, y: 车辆/云台当前位置 (单位: cm)
    height: 云台高度 (单位: cm)
    n: 当前圈数 (从1开始)
    target_x, target_y, target_height: 靶点位置 (单位: cm)
    
    返回:
    yaw: 云台相对于车头的顺时针转动角度 (0-720+度)
    pitch: 俯仰角 (度)
    """
    # 1. 计算从云台到靶点的向量
    dx = target_x - x
    dy = target_y - y
    dz = target_height - height
    
    # 2. 计算水平距离（忽略高度）
    horizontal_distance = math.sqrt(dx**2 + dy**2)
    # print(f'距离：{horizontal_distance}')

    # 3. 计算俯仰角（pitch）
    pitch = math.degrees(math.atan2(dz, horizontal_distance))
    
    # 4. 计算绝对水平角度（相对于世界坐标系）
    if 0 <= math.degrees(math.atan2(dy, dx)) <= 180:
        absolute_yaw = 90 - math.degrees(math.atan2(dy, dx))
    else:
        absolute_yaw = math.degrees(math.atan2(dy, dx)) - 270
    
    # 5. 根据车辆位置确定车头朝向
    # 跑道定义：
    # 边1: (-50,50) -> (-50,-50) - 车头向下 (y轴负方向)
    # 边2: (-50,-50) -> (50,-50) - 车头向右 (x轴正方向)
    # 边3: (50,-50) -> (50,50) - 车头向上 (y轴正方向)
    # 边4: (50,50) -> (-50,50) - 车头向左 (x轴负方向)
    
    # 确定车辆在哪条边上
    if abs(x + 50.0) < 0.001 and -50 <= y <= 50:  # 边1
        car_heading = -180  # 车头向下
    elif abs(y + 50.0) < 0.001 and -50 <= x <= 50:  # 边2
        car_heading = -270    # 车头向右
    elif abs(x - 50.0) < 0.001 and -50 <= y <= 50:  # 边3
        car_heading = -360   # 车头向上
    elif abs(y - 50.0) < 0.001 and -50 <= x <= 50:  # 边4
        car_heading = -450  # 车头向左
    else:
        # 如果不在边上，默认车头向右
        car_heading = -90
    
    # 6. 计算相对于车头朝向的偏移角
    relative_yaw = absolute_yaw - car_heading
    
    # 7. 将偏移角规范到[0, 360)范围
    relative_yaw %= 360
    if relative_yaw < 0:
        relative_yaw += 360

    # yaw = relative_yaw + 360 * (n - 1)

    # 8. 根据圈数和位置调整yaw角度
    # 在边4上时，需要加上360度乘以圈数
    if abs(y - 50) < 1 and -50 < x < 50:  # 边4
        yaw = relative_yaw + 360 * n
    else:
        yaw = relative_yaw + 360 * (n - 1)
    
    return yaw, pitch

def get_yaw_pitch_from_distance(total_distance):
    """
    根据总路程计算云台的yaw和pitch角度
    
    参数:
    total_distance: 总路程 (单位: cm)
    
    返回:
    yaw: 云台相对于车头的顺时针转动角度 (0-720+度)
    pitch: 俯仰角 (度)
    """
    # 固定参数
    height = 18  # 云台高度 (cm)
    target_x = 0   # 靶点x坐标 (cm)
    target_y = 100  # 靶点y坐标 (cm)
    target_height = 19  # 靶点高度 (cm)
    
    # 1. 根据路程计算位置和圈数
    n, x, y = get_position_from_distance(total_distance)
    
    # 2. 计算角度
    yaw, pitch = calculate_angles_for_perf4(
        x, y, height, n, 
        target_x, target_y, target_height
    )
    
    return yaw, pitch

def task0(uart, cam, disp):

    led.value(1)

    # 进行中心对位
    aligned = align_center(uart, cam, disp)
    if aligned:
        print("中心点对齐完成!")
        led.value(0)
        time.sleep(3)
        led.value(1)
    else:
        print("中心点对齐失败")

def task1(uart, cam, disp, angle_value):

    led.value(1)

    # 开始矩形扫描
    found = scan_for_rectangle(uart, cam, disp, angle_value)
    
    if found:
        print("成功找到矩形!")
        # 进行中心点对齐
        aligned = align_center(uart, cam, disp)
        if aligned:
            print("中心点对齐完成!")
            led.value(0)
            time.sleep(3)
            led.value(1)
        else:
            print("中心点对齐失败")
    else:
        print("未找到矩形，扫描完成")

def task2(uart, cam, disp):
    """
    改进版任务二：持续跟踪矩形中心
    新增功能：直角转弯角度补偿（累计60°时均匀补充至80°）
    优化：独立于串口速率的帧率控制 + 30秒时间限制
    """
    # ===== 任务时间限制 =====
    TASK_DURATION = -30000  # 任务执行时间限制（毫秒），30秒
    start_time = time.ticks_ms()  # 记录任务开始时间
    
    # ===== 角度控制PID参数 =====
    ANGLE_Kp = 0.60   # 比例系数
    ANGLE_Ki = 0.00   # 积分系数
    ANGLE_Kd = 0.48   # 微分系数
    
    # ===== 距离控制PID参数 =====
    DIST_Kp = 1.00    # 比例系数
    DIST_Ki = 0.00    # 积分系数
    DIST_Kd = 0.80    # 微分系数
    
    # 角度比例系数 (像素到角度的转换因子)
    PIXEL_TO_ANGLE_X = 0.1
    PIXEL_TO_ANGLE_Y = 0.1
    
    # 初始化PID状态变量
    prev_error_x = 0
    prev_error_y = 0
    integral_x = 0
    integral_y = 0
    
    # 帧计数器
    frame_counter = 0
    
    # 目标丢失计数器
    target_missing_count = 0
    
    # 初始化透视变换
    perspective = None
    
    # 当前角度初始值
    current_x_angle = 125.0
    current_y_angle = 0.0

    # ===== 改进的直角转弯角度补偿机制 =====
    turn_angle_accumulator = 0.0       # 累计转动角度
    remaining_compensation = 0.0       # 剩余需要补偿的角度
    compensation_active = False         # 是否正在补偿中
    TURN_ANGLE_THRESHOLD = 65.0         # 触发补偿的角度阈值
    TURN_ANGLE_COMPENSATION = 25.0     # 需要补偿的总角度
    COMPENSATION_PER_FRAME = 3.0        # 每帧补偿的角度量
    
    control_mode = 'distance'
    
    # ===== 高效串口处理变量 =====
    command_buffer = bytearray()  # 使用字节数组提高效率
    MAX_BUFFER_SIZE = 1024        # 最大缓冲区大小
    PROCESS_TIMEOUT = 10          # 命令处理超时时间(ms)
    
    # ===== 新增：角度状态跟踪 =====
    last_valid_x_angle = current_x_angle  # 上一次有效的X角度
    last_valid_y_angle = current_y_angle  # 上一次有效的Y角度
    last_angle_update_time = time.ticks_ms()  # 上次角度更新时间

    print("开始任务二：改进版持续跟踪矩形中心...")
    # print(f"任务将在 {TASK_DURATION/1000} 秒后自动结束")
    
    # 非阻塞读取设置
    uart_timeout = -1  # 设置读取超时为0，实现非阻塞读取
    
    led.value(0)

    while not app.need_exit():
        current_time = time.ticks_ms()
        
        # === 检查任务时间限制 ===
        elapsed_time = time.ticks_diff(current_time, start_time)
        if elapsed_time < TASK_DURATION:
            print(f"任务已执行 {elapsed_time/1000:.1f} 秒，达到时间限制，退出任务")
            led.value(1)
            break
        else:
            print(f"任务已执行 {elapsed_time/1000:.1f} 秒")

        # === 高效读取串口数据 ===
        data = uart.read(uart_timeout)  # 非阻塞读取
        if data:
            # 添加数据到缓冲区
            command_buffer.extend(data)
            
            # 限制缓冲区大小，防止内存溢出
            if len(command_buffer) > MAX_BUFFER_SIZE:
                # 保留最后一部分数据（可能包含未处理命令）
                keep_size = min(100, len(command_buffer))
                command_buffer = command_buffer[-keep_size:]
                print("警告：命令缓冲区溢出，已截断")
        
        # === 高效处理串口命令 ===
        start_processing_time = time.ticks_ms()
        while len(command_buffer) >= 3:  # 至少需要 <x> 形式
            # 查找命令起始位置
            start_idx = -1
            for i in range(len(command_buffer)):
                if command_buffer[i] == ord('<'):
                    start_idx = i
                    break
            
            # 未找到起始标记
            if start_idx == -1:
                # 清空缓冲区（无有效命令）
                command_buffer.clear()
                break
                
            # 查找命令结束位置
            end_idx = -1
            for i in range(start_idx + 1, len(command_buffer)):
                if command_buffer[i] == ord('>'):
                    end_idx = i
                    break
            
            # 未找到结束标记
            if end_idx == -1:
                # 保留未完成命令部分
                command_buffer = command_buffer[start_idx:]
                break
                
            # 提取命令内容
            try:
                # 跳过起始标记 '<'
                command = command_buffer[start_idx+1:end_idx].decode('utf-8')
                
                # 移除已处理部分（包括结束标记 '>'）
                del command_buffer[:end_idx+1]
                
                # 解析命令
                parts = command.split('_')
                if len(parts) >= 2:
                    # 收到角度数据，切换到角度控制模式
                    control_mode = "angle"

                    try:
                        # 获取角度增量
                        angle_increment_x = float(parts[0])
                        angle_increment_y = float(parts[1])
                        
                        # 更新累计转动角度
                        turn_angle_accumulator += abs(angle_increment_x)
                        
                        # 检查是否需要启动角度补偿
                        if not compensation_active and turn_angle_accumulator >= TURN_ANGLE_THRESHOLD:
                            # 计算需要补偿的总角度
                            remaining_compensation = TURN_ANGLE_COMPENSATION
                            compensation_active = True
                            print(f"启动角度补偿: 累计转动{round(turn_angle_accumulator, 1)}°, 需要补偿{remaining_compensation}°")
                        
                        # 正常更新角度
                        current_x_angle += angle_increment_x
                        current_y_angle += angle_increment_y
                        
                        # 更新最后有效的角度和时间
                        last_valid_x_angle = current_x_angle
                        last_valid_y_angle = current_y_angle
                        last_angle_update_time = current_time
                        
                        print(f"更新角度: X={current_x_angle}°, Y={current_y_angle}°")
                    except ValueError:
                        print(f"角度值解析失败: {parts[0]}, {parts[1]}")
                else:
                    # 收到距离数据，切换到距离控制模式
                    control_mode = "distance"
                    try:
                        # 启用预测
                        new_distance = float(command)
                        
                        # 使用真实距离值计算角度
                        yaw, pitch = get_yaw_pitch_from_distance(new_distance)
                        current_x_angle = yaw
                        current_y_angle = pitch
                        
                        # 更新最后有效的角度和时间
                        last_valid_x_angle = current_x_angle
                        last_valid_y_angle = current_y_angle
                        last_angle_update_time = current_time
                        
                        print(f'真实距离:{new_distance}')
                    except ValueError:
                        print(f"距离值解析失败: {command}")
                        
            except Exception as e:
                print(f"命令解析错误: {str(e)}")
                # 移除错误命令
                del command_buffer[:end_idx+1]
            
            # 超时保护：处理时间超过10ms则退出
            if time.ticks_diff(time.ticks_ms(), start_processing_time) > PROCESS_TIMEOUT:
                print("命令处理超时，暂停处理")
                break
        
        # === 角度补偿处理 ===
        if compensation_active:
            # 计算本帧需要补偿的角度量
            compensation_this_frame = min(remaining_compensation, COMPENSATION_PER_FRAME)
            
            # 应用补偿角度
            current_x_angle += compensation_this_frame
            remaining_compensation -= compensation_this_frame
            
            # 更新最后有效的角度
            last_valid_x_angle = current_x_angle
            last_valid_y_angle = current_y_angle
            
            print(f"角度补偿: +{compensation_this_frame:.1f}° (剩余{remaining_compensation:.1f}°)")
            
            # 检查补偿是否完成
            if remaining_compensation <= 0:
                compensation_active = False
                turn_angle_accumulator = 0  # 重置累计角度
                print("角度补偿完成!")
        
        # === 图像处理降频 ===
        # 每2帧处理一次图像，提高性能
        if frame_counter % 1 != 0:
            # 跳过图像处理，仅更新云台角度
            set_xy_angles(uart, current_x_angle, 0)
            frame_counter += 1
            continue
        
        # === 主处理逻辑 ===
        # 获取图像
        img = cam.read()
        if img is None:
            print("摄像头读取失败")
            continue
            
        img_raw = image.image2cv(img)
        
        # 处理图像并检测矩形
        result = process_quadrilateral_detection(img, uart, 0)
        
        if result['found']:
            # 重置丢失计数器
            target_missing_count = 0
            print('找到矩形')

            try:
                # 计算当前帧的透视变换
                perspective = calculate_perspective_transform(result['corners'])
                
                # 通过逆变换计算矩形中心的图像坐标
                world_center = np.array([0, 0, 1], dtype=np.float32)
                image_center_homogeneous = np.dot(np.linalg.inv(perspective.matrix), world_center)
                target_x = image_center_homogeneous[0] / image_center_homogeneous[2]
                target_y = image_center_homogeneous[1] / image_center_homogeneous[2]
                
                # 在图像上绘制计算出的矩形中心
                cv2.circle(img_raw, (int(target_x), int(target_y)), 8, (0, 0, 255), -1)
                
                # 计算与屏幕中心的偏移量
                error_x = CENTER_X - target_x
                error_y = CENTER_Y - target_y
                
                # 计算偏移距离
                distance = math.sqrt(error_x**2 + error_y**2)
                
                # 在图像上绘制中心点和偏移线
                cv2.circle(img_raw, (CENTER_X, CENTER_Y), 5, (0, 255, 255), -1)  # 屏幕中心(黄色)
                cv2.circle(img_raw, (int(target_x), int(target_y)), 3, (0, 0, 255), -1)  # 实际位置(红色)
                cv2.line(img_raw, (CENTER_X, CENTER_Y), (int(target_x), int(target_y)), (255, 0, 0), 2)
                
                # 检查是否已对齐
                if distance < ALIGN_THRESHOLD:
                    # 对齐时不显示任何文本
                    pass
                else:
                    # 根据控制模式选择PID参数
                    if control_mode == 'angle':
                        Kp, Ki, Kd = ANGLE_Kp, ANGLE_Ki, ANGLE_Kd
                    else:
                        Kp, Ki, Kd = DIST_Kp, DIST_Ki, DIST_Kd
                    
                    # 增量式PID控制计算
                    # 比例项
                    P_x = Kp * error_x
                    P_y = Kp * error_y
                    
                    # 积分项（带抗饱和）
                    integral_x += error_x
                    integral_y += error_y
                    
                    # 积分限幅
                    MAX_INTEGRAL = 1000
                    integral_x = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, integral_x))
                    integral_y = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, integral_y))
                    
                    I_x = Ki * integral_x
                    I_y = Ki * integral_y
                    
                    # 微分项（使用误差变化率）
                    D_x = Kd * (error_x - prev_error_x)
                    D_y = Kd * (error_y - prev_error_y)
                    
                    # 保存当前误差用于下一次微分计算
                    prev_error_x = error_x
                    prev_error_y = error_y
                    
                    # 转换为角度变化
                    delta_angle_x = (P_x + I_x + D_x) * PIXEL_TO_ANGLE_X
                    delta_angle_y = (P_y + I_y + D_y) * PIXEL_TO_ANGLE_Y
                    
                    # 使用串口接收的角度计算新角度
                    # 使用最后有效的角度作为基准
                    new_x_angle = last_valid_x_angle - delta_angle_x  # X轴反向
                    new_y_angle = last_valid_y_angle - delta_angle_y  # Y轴同向
                    
                    # 更新当前角度（即使没有新的串口消息）
                    current_x_angle = new_x_angle
                    current_y_angle = new_y_angle
                    
                    # 限制角度范围
                    new_x_angle = max(0, min(1080, new_x_angle))
                    new_y_angle = max(-5, min(5, new_y_angle))
                    
                    # 移动云台到新角度
                    set_xy_angles(uart, new_x_angle, 0)
                
                # 显示处理结果
                disp.show(image.cv2image(img_raw))
                
            except Exception as e:
                print(f"透视变换计算错误: {str(e)}")
                # 显示原始图像
                disp.show(image.cv2image(img_raw))
        
        else:
            # 目标未检测到
            target_missing_count += 1
            
            # 直接使用当前角度值设置云台
            set_xy_angles(uart, current_x_angle, 0)
            
            # 显示图像
            disp.show(image.cv2image(img_raw))
        
        # 更新帧计数器
        frame_counter += 1
    
    # 任务结束处理
    print(f"任务二结束，共执行 {time.ticks_diff(time.ticks_ms(), start_time)/1000:.1f} 秒")
    
    # 发送云台回到初始位置
    set_xy_angles(uart, 0.0, 0.0)
    print("云台已回到初始位置")
    
def task3(uart, cam, disp):

    led.value(1)
    """
    任务三：控制屏幕中心围绕矩形中心作半径6cm的圆周运动一周
    修改流程：
    1. 检测矩形角点
    2. 计算透视变换参数
    3. 通过逆变换计算矩形中心在图像中的位置
    4. 围绕这个点做圆周运动
    """
    # PID控制参数
    Kp = 0.45   # 比例系数
    Ki = 0.06   # 积分系数
    Kd = 0.40   # 微分系数
    
    # 角度比例系数
    PIXEL_TO_ANGLE_X = 0.1
    PIXEL_TO_ANGLE_Y = 0.1
    
    # 圆周运动参数
    RADIUS_CM = 6.0    # 半径6cm
    N_POINTS = 32      # 圆周点数量
    POINT_ANGLE_STEP = 2 * math.pi / N_POINTS
    
    # 当前圆周位置角度
    current_angle = 0.0
    
    # 目标丢失计数器
    target_missing_count = 0
    MAX_MISSING_FRAMES = 10
    
    # 帧计数器
    frame_counter = 0
    # 当前点索引
    current_point_idx = 0
    
    prev_error_x = 0
    prev_error_y = 0

    print("开始任务三：动态圆周运动跟踪(改进版)...")
    # print("新流程：通过透视变换计算矩形中心图像坐标")
    
    # 获取初始角度
    current_x_angle, current_y_angle = current_angle_x, current_angle_y
    
    # 记录起始位置，用于最后回到起点
    start_angle = current_angle
    
    # 圆周运动状态
    CIRCLE_STARTED = False
    CIRCLE_COMPLETED = False
    
    while not app.need_exit():
        # 获取图像
        img = cam.read()
        img_raw = image.image2cv(img)
        
        # 处理图像并检测矩形
        result = process_quadrilateral_detection(img, uart, frame_counter)
        # frame_counter = (frame_counter + 1) % 2

        if result['found']:
            # 重置丢失计数器
            target_missing_count = 0
            
            try:
                # 重新计算当前帧的透视变换
                perspective = calculate_perspective_transform(result['corners'])
                
                # ===== 关键修改：通过逆变换计算矩形中心在图像中的位置 =====
                # 矩形中心在世界坐标系中是(0,0)
                world_center = np.array([0, 0, 1], dtype=np.float32)
                
                # 应用逆透视变换得到图像坐标
                image_center_homogeneous = np.dot(np.linalg.inv(perspective.matrix), world_center)
                rect_center_x = image_center_homogeneous[0] / image_center_homogeneous[2]
                rect_center_y = image_center_homogeneous[1] / image_center_homogeneous[2]
                
                # 在图像上绘制矩形中心
                cv2.circle(img_raw, (int(rect_center_x), int(rect_center_y)), 8, (0, 0, 255), -1)
                
                # 计算当前圆周点在世界坐标系中的位置
                if CIRCLE_COMPLETED:
                    # 如果圆周运动已完成，目标点是起点
                    world_x = RADIUS_CM * math.cos(start_angle)
                    world_y = RADIUS_CM * math.sin(start_angle)
                else:
                    # 正常圆周运动点
                    world_x = RADIUS_CM * math.cos(current_angle)
                    world_y = RADIUS_CM * math.sin(current_angle)
                
                # 应用透视变换得到图像坐标
                homogeneous_point = np.array([world_x, world_y, 1], dtype=np.float32)
                transformed = np.dot(np.linalg.inv(perspective.matrix), homogeneous_point)
                image_x = transformed[0] / transformed[2]
                image_y = transformed[1] / transformed[2]
                
                # 如果坐标在图像范围内
                if 0 <= image_x < 400 and 0 <= image_y < 240:
                    # 计算偏移量 (屏幕中心到目标点)
                    error_x = CENTER_X - image_x
                    error_y = CENTER_Y - image_y
                    
                    # 在图像上绘制目标点和方向
                    cv2.circle(img_raw, (int(image_x), int(image_y)), 5, (0, 255, 255), -1)
                    cv2.circle(img_raw, (CENTER_X, CENTER_Y), 5, (0, 255, 0), -1)
                    cv2.line(img_raw, (CENTER_X, CENTER_Y), (int(image_x), int(image_y)), (255, 0, 0), 2)
                    
                    # 绘制圆周轨迹
                    circle_points = []
                    for i in range(0, 360, 10):
                        angle = math.radians(i)
                        wx = RADIUS_CM * math.cos(angle)
                        wy = RADIUS_CM * math.sin(angle)
                        
                        homogeneous_point = np.array([wx, wy, 1], dtype=np.float32)
                        transformed = np.dot(np.linalg.inv(perspective.matrix), homogeneous_point)
                        px = transformed[0] / transformed[2]
                        py = transformed[1] / transformed[2]
                        
                        if 0 <= px < 400 and 0 <= py < 240:
                            circle_points.append((int(px), int(py)))
                    
                    # 绘制圆周轨迹
                    if len(circle_points) > 1:
                        for i in range(len(circle_points) - 1):
                            cv2.line(img_raw, circle_points[i], circle_points[i+1], (0, 165, 255), 1)
                    
                    # 绘制矩形中心和坐标轴
                    cv2.circle(img_raw, (int(rect_center_x), int(rect_center_y)), 5, (255, 0, 255), -1)
                    
                    # 计算偏移距离
                    distance = math.sqrt(error_x**2 + error_y**2)
                    
                    # 显示偏移信息
                    if CIRCLE_COMPLETED:
                        cv2.putText(img_raw, "Returning to start...", (10, 20), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    else:
                        cv2.putText(img_raw, f"Point: {current_point_idx}/{N_POINTS}", (10, 20), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    cv2.putText(img_raw, f"DX: {int(error_x)} DY: {int(error_y)}", (10, 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(img_raw, f"Dist: {distance:.1f}", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    cv2.putText(img_raw, f"Angle: {math.degrees(current_angle):.1f}°", (10, 80), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    # 仅在偏移足够大时进行控制计算
                    if distance > ALIGN_THRESHOLD:
                        # 增量式PID控制计算
                        P_x = Kp * error_x
                        P_y = Kp * error_y
                        
                        I_x = Ki * (error_x)  # 只使用当前误差
                        I_y = Ki * (error_y)  # 只使用当前误差
                        
                        D_x = Kd * (error_x - prev_error_x)
                        D_y = Kd * (error_y - prev_error_y)
                        
                        # 计算总控制输出 (角度变化量)
                        delta_angle_x = P_x + I_x + D_x
                        delta_angle_y = P_y + I_y + D_y
                        
                        # 保存当前误差用于下一次微分计算
                        prev_error_x = error_x
                        prev_error_y = error_y
                        
                        # 显示PID参数
                        cv2.putText(img_raw, f"PID: P:{P_x:.1f} I:{I_x:.1f} D:{D_x:.1f}", (10, 100), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        
                        # 转换为角度变化
                        delta_angle_x *= PIXEL_TO_ANGLE_X
                        delta_angle_y *= PIXEL_TO_ANGLE_Y
                        
                        # 计算新的绝对角度
                        new_x_angle = current_x_angle - delta_angle_x  # X轴反向
                        new_y_angle = current_y_angle - delta_angle_y  # Y轴同向
                        
                        # 限制角度范围
                        new_x_angle = max(-360, min(360, new_x_angle))
                        new_y_angle = max(-90, min(90, new_y_angle))
                        
                        # 移动云台到新角度
                        set_xy_angles(uart, new_x_angle, new_y_angle)
                        
                        # 更新当前角度
                        current_x_angle, current_y_angle = new_x_angle, new_y_angle
                        
                        # 显示角度信息
                        cv2.putText(img_raw, f"PTZ: X:{new_x_angle:.1f}° Y:{new_y_angle:.1f}°", (10, 120), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    else:
                        # 偏移小于阈值，可以移动到下一个点
                        if not CIRCLE_COMPLETED:
                            # 圆周运动尚未完成，移动到下一个点
                            current_angle += POINT_ANGLE_STEP
                            current_point_idx += 1
                            led.value(0)

                            # 显示完成进度
                            progress = (current_point_idx / N_POINTS) * 100
                            cv2.putText(img_raw, f"Progress: {progress:.1f}%", (150, 40), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                            
                            # 检查是否完成一周
                            if current_point_idx >= N_POINTS:
                                print("圆周运动一周完成!")
                                CIRCLE_COMPLETED = True
                                # 重置起点位置
                                start_angle = current_angle
                        else:
                            # 圆周运动已完成且已回到起点
                            print("已回到起点!")
                            cv2.putText(img_raw, "CIRCLE COMPLETE!", (80, 180), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                            cv2.putText(img_raw, "Returned to start!", (90, 210), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            disp.show(image.cv2image(img_raw))
                            time.sleep(2)

                            led.value(1)
                            break
                else:
                    print("目标点超出图像范围")
                    if not CIRCLE_COMPLETED:
                        current_angle += POINT_ANGLE_STEP
                        current_point_idx += 1
            except Exception as e:
                print(f"计算错误: {str(e)}")
        else:
            # 目标未检测到
            target_missing_count += 1
            cv2.putText(img_raw, "No rectangle detected!", (50, 120), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(img_raw, f"Missing: {target_missing_count}/{MAX_MISSING_FRAMES}", (50, 150), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 如果连续多帧未检测到目标，尝试重新扫描
            if target_missing_count > MAX_MISSING_FRAMES:
                print("目标丢失，尝试重新扫描...")
                found = scan_for_rectangle(uart, cam, disp, 0)
                target_missing_count = 0
                
                if not found:
                    print("重新扫描未找到矩形")
                    break
                else:
                    # 重置圆周参数
                    current_angle = 0.0
                    current_point_idx = 0
                    CIRCLE_COMPLETED = False
                    print("重新找到矩形，重置圆周参数")
        
        # 显示处理结果
        disp.show(image.cv2image(img_raw))
        # time.sleep_ms(20)  # 适当延迟控制频率
    
    print("任务三结束")

def main():
    
    global RPM, RPM_ANGLE_LOW
    RPM = 350
    RPM_ANGLE_LOW = 200

    # 初始化串口
    serial = uart.UART(device, 115200)
    
    led.value(1)
    # 重置位置到初始状态 (0°, 0°)
    # set_xy_zero(serial, 125, 0)
    
    # 命令缓冲区
    command_buffer = ""
    
    # task0(serial, cam, disp)

    print("系统已启动，等待串口指令...")
    
    while not app.need_exit():
        # 读取串口数据
        data = serial.read()
        
        if data:
            try:
                # 将字节数据解码为字符串
                if isinstance(data, bytes):
                    command_buffer += data.decode('utf-8')
                else:
                    command_buffer += data
                
                # 检查是否有完整的命令
                while '<' in command_buffer and '>' in command_buffer:
                    start_idx = command_buffer.index('<')
                    end_idx = command_buffer.index('>', start_idx)
                    
                    if start_idx < end_idx:
                        command = command_buffer[start_idx+1:end_idx]
                        command_buffer = command_buffer[end_idx+1:]  # 移除已处理的命令
                        
                        print(f"接收到命令: <{command}>")
                        
                        # 解析命令格式：pattern_x_angle
                        if command.startswith("pattern_") and '_' in command:
                            parts = command.split('_')
                            if len(parts) >= 3:
                                pattern_num = parts[1]  # 任务编号部分
                                angle_str = parts[2]    # 角度值部分
                                
                                try:
                                    # 将角度值转换为浮点数（支持小数）
                                    angle_value = float(angle_str)
                                    print(f"解析结果 - 任务: {pattern_num}, 角度: {angle_value}°")

                                    # set_xy_zero(serial, 0, 0)

                                    # 根据命令执行相应任务
                                    if pattern_num == "2":
                                        print("执行任务2: 中心对位")
                                        # global RPM_ANGLE_LOW, RPM
                                        RPM_ANGLE_LOW = 300
                                        RPM = 500
                                        task0(serial, cam, disp)
                                        print("任务2完成")
                                        set_xy_angles(serial, 0, 0)
                                        RPM_ANGLE_LOW = 200
                                        RPM = 350

                                    elif pattern_num == "3":
                                        print("执行任务3: 扫描并对齐矩形")
                                        # 使用浮点数进行计算
                                        # global RPM_ANGLE_LOW, RPM
                                        RPM_ANGLE_LOW = 300
                                        RPM = 500
                                        target_angle = angle_value - 35
                                        set_xy_angles(serial, target_angle, 0.0)
                                        task1(serial, cam, disp, target_angle)
                                        set_xy_angles(serial, 0, 0)
                                        print("任务3完成")
                                        RPM_ANGLE_LOW = 200
                                        RPM = 350

                                    elif pattern_num == "4":
                                        print("执行任务4: 持续跟踪矩形中心")
                                        # set_xy_angles(serial, 135,0)
                                        task2(serial, cam, disp)
                                        print("任务4完成")
                                    elif pattern_num == "5":
                                        print("执行任务5: 持续跟踪矩形中心")
                                        task2(serial, cam, disp)
                                        print("任务5完成")
                                    elif pattern_num == "6":
                                        print("执行任务6: 圆周运动")
                                        task3(serial, cam, disp)
                                        print("任务6完成")
                                    else:
                                        print(f"未知任务编号: {pattern_num}")
                                except ValueError:
                                    print(f"角度值无效: {angle_str}")
                            else:
                                print(f"命令格式错误: {command}")
                        else:
                            print(f"未知命令格式: {command}")
            except Exception as e:
                print(f"串口数据处理错误: {str(e)}")
                command_buffer = ""  # 清空缓冲区

        # 短暂延时，避免过度占用CPU
        time.sleep_ms(50)

def main_test():
    global RPM, RPM_ANGLE_LOW

    # 初始化串口
    serial = uart.UART(device, 115200)
    set_xy_angles(serial, 225, 0)
    
    command_buffer = ''
    countor = 0
    led.value(0)
    
    # 新增变量用于预测
    last_distance = 0.0  # 上次接收到的距离值
    last_distance_time = time.ticks_ms()  # 上次接收距离的时间
    prediction_active = False  # 是否启用预测
    distance_velocity = 0.0  # 距离变化速度 (cm/ms)
    prediction_counter = 0  # 预测次数计数器
    
    # === PID控制参数 ===
    Kp = 0.50    # 比例系数
    Ki = 0.05    # 积分系数
    Kd = 0.35    # 微分系数
    PIXEL_TO_ANGLE = 0.1  # 像素到角度的转换因子
    
    # 初始化PID状态变量
    prev_error_x = 0
    integral_x = 0
    
    # 帧率计算变量
    frame_count = 0
    
    # 当前角度初始值
    current_x_angle = 135.0
    current_y_angle = 0.0
    
    print("开始距离数据PID控制...")
    
    while not app.need_exit():
        current_time = time.ticks_ms()
        data_received = False
        
        # === 读取串口数据 ===
        data = serial.read()
        if data:
            last_distance_time = time.ticks_ms()
            data_received = True
            try:
                if isinstance(data, bytes):
                    command_buffer += data.decode('utf-8')
                else:
                    command_buffer += data
                
                # 检查是否有完整的命令
                if '<' in command_buffer and '>' in command_buffer:
                    start_idx = command_buffer.index('<')
                    end_idx = command_buffer.index('>', start_idx)
                    
                    if start_idx < end_idx:
                        command = command_buffer[start_idx+1:end_idx]
                        command_buffer = command_buffer[end_idx+1:]
                        
                        # 解析命令格式：x_y
                        parts = command.split('_')
                        if len(parts) >= 2:
                            # 收到角度数据
                            prediction_active = False
                            try:
                                current_x_angle = float(parts[0]) + 135
                                current_y_angle = float(parts[1])
                                print(f"更新角度: X={current_x_angle}°, Y={current_y_angle}°")
                            except ValueError:
                                print(f"角度值: {parts[0]}, {parts[1]}")
                        else:
                            try:
                                # 收到距离数据
                                prediction_active = False
                                new_distance = float(command)
                                
                                # 计算距离变化速度
                                time_diff = max(1, time.ticks_diff(current_time, last_distance_time))
                                distance_velocity = (new_distance - last_distance) / time_diff
                                
                                # 更新距离值和时间戳
                                last_distance = new_distance
                                last_distance_time = current_time
                                
                                # 使用真实距离值计算角度
                                yaw, pitch = get_yaw_pitch_from_distance(new_distance)
                                current_x_angle = yaw
                                current_y_angle = pitch
                                countor += 1
                                print(f'真实距离:{new_distance}')
                                print(f'计数器:{countor}')
                            except ValueError:
                                print(f"距离接收失败")
            except Exception as e:
                print(f"串口数据处理错误: {str(e)}")
                command_buffer = ""
        
        # === 预测处理 ===
        if prediction_active and not data_received and time.ticks_diff(current_time, last_distance_time) > 50:
            # 计算预测距离
            time_diff = time.ticks_diff(current_time, last_distance_time)
            predicted_distance = last_distance + distance_velocity * time_diff
            
            # 使用预测距离计算角度
            yaw, pitch = get_yaw_pitch_from_distance(predicted_distance)
            current_x_angle = yaw
            current_y_angle = pitch
            prediction_counter += 1
            print(f'预测距离:{predicted_distance:.2f} (基于{last_distance}, 速度:{distance_velocity*1000:.2f}cm/s)')
            print(f'预测次数:{prediction_counter}')
        
        # === 图像处理和PID控制 ===
        # 获取图像
        img = cam.read()
        img_raw = image.image2cv(img)
        
        # 处理图像并检测矩形
        result = process_quadrilateral_detection(img, serial, frame_count)
        
        if result['found']:
            try:
                # 计算当前帧的透视变换
                perspective = calculate_perspective_transform(result['corners'])
                
                # 通过逆变换计算矩形中心的图像坐标
                world_center = np.array([0, 0, 1], dtype=np.float32)
                image_center_homogeneous = np.dot(np.linalg.inv(perspective.matrix), world_center)
                target_x = image_center_homogeneous[0] / image_center_homogeneous[2]
                target_y = image_center_homogeneous[1] / image_center_homogeneous[2]
                
                # 在图像上绘制计算出的矩形中心
                cv2.circle(img_raw, (int(target_x), int(target_y)), 8, (0, 0, 255), -1)
                
                # 计算与屏幕中心的偏移量
                error_x = CENTER_X - target_x
                
                # 在图像上绘制中心点和偏移线
                cv2.circle(img_raw, (CENTER_X, CENTER_Y), 5, (0, 255, 255), -1)
                cv2.circle(img_raw, (int(target_x), int(target_y)), 3, (0, 0, 255), -1)
                cv2.line(img_raw, (CENTER_X, CENTER_Y), (int(target_x), int(target_y)), (255, 0, 0), 2)
                
                # 检查是否已对齐
                if abs(error_x) < ALIGN_THRESHOLD:
                    # 已对齐，不需要调整
                    pass
                else:
                    # PID控制计算
                    # 比例项
                    P_x = Kp * error_x
                    
                    # 积分项
                    integral_x += error_x
                    
                    # 积分限幅
                    MAX_INTEGRAL = 1000
                    integral_x = max(-MAX_INTEGRAL, min(MAX_INTEGRAL, integral_x))
                    
                    I_x = Ki * integral_x
                    
                    # 微分项
                    D_x = Kd * (error_x - prev_error_x)
                    
                    # 保存当前误差用于下一次微分计算
                    prev_error_x = error_x
                    
                    # 转换为角度变化
                    delta_angle = (P_x + I_x + D_x) * PIXEL_TO_ANGLE
                    
                    # 计算新角度
                    new_x_angle = current_x_angle - delta_angle
                    
                    # 限制角度范围
                    new_x_angle = max(0, min(1080, new_x_angle))
                    
                    # 移动云台到新角度
                    set_xy_angles(serial, new_x_angle, 0)

            except Exception as e:
                print(f"处理错误: {str(e)}")
        else:
            # 未检测到矩形，直接使用当前角度
            set_xy_angles(serial, current_x_angle, 0)
        
        # 显示处理结果
        disp.show(image.cv2image(img_raw))
        
        # 短暂延时，防止过度占用CPU
        # time.sleep_ms(10)

if __name__ == "__main__":
    main()