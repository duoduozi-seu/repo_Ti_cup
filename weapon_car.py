import time, os, sys
from machine import UART
from machine import FPIOA
from media.sensor import *
from media.display import *
from media.media import *
from ybUtils.YbUart import YbUart

fpioa = FPIOA()
fpioa.set_function(11, FPIOA.UART2_TXD)
fpioa.set_function(12, FPIOA.UART2_RXD)

sensor_id = 2
sensor = None
picture_width = 640
picture_height = 480
DISPLAY_MODE = "LCD"
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
color_threshold = [(0, 67)]

global ROIS
ROIS = {
    '1':   (  0,   0, 640, 60),
    '2':   (  0,  96, 640, 60),
    '3':   (  0, 192, 640, 60),
    '4':   (  0, 288, 640, 60),
    '5':   (  0, 384, 640, 60)
}

# 区域权重 - 下方区域权重更大（更接近车辆）
REGION_WEIGHTS = {
    '1': 0.35,  # 最上方区域
    '2': 0.35,
    '3': 0.4,
    '4': 0.3,
    '5': 0.25   # 最下方区域
}

# 定义消息模式
PATTERN_1 = "<pattern_1_1>"
PATTERN_2 = "<pattern_2_1>"
PATTERN_3 = "<pattern_3_1>"
PATTERN_4 = "<pattern_4_1>"
PATTERN_5 = "<pattern_5_1>"
PATTERN_6 = "<pattern_6_1>"
END_SIGNAL = "#S!"  # 任务结束标志

def init():
    global sensor, fps, uart
    # 构造一个具有默认配置的摄像头对象
    sensor = Sensor(id=sensor_id, width=640, height=480)
    # 重置摄像头sensor
    sensor.reset()
    # 无需进行镜像和翻转
    # 设置不要水平镜像
    sensor.set_hmirror(False)
    # 设置不要垂直翻转
    sensor.set_vflip(False)
    sensor.set_framesize(width=picture_width, height=picture_height, chn=CAM_CHN_ID_0)
    # 设置通道0的输出像素格式为RGB565，要注意有些案例只支持GRAYSCALE格式
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
    Display.init(Display.ST7701, width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT, to_ide=True)

    # 初始化媒体管理器
    MediaManager.init()
    fps = time.clock()

    # 初始化串口
    UART_PORT = 3  # 使用UART3
    uart = YbUart(baudrate=115200)

def find_blobs_in_rois(img):
    # 绘制检测框边界
    for roi in ROIS.values():
        img.draw_rectangle(roi, color=(255,0,0), thickness=1)

    roi_blobs_result = {}
    for roi_id in ROIS.keys():
        roi_blobs_result[roi_id] = {
            'blob_flag': 0X00,
            'centers': [],  # 存储所有色块中心x坐标
            'blobs': [],    # 存储所有检测到的色块对象
            'largest_blob': []
        }

    for roi_id, roi in ROIS.items():
        blobs = img.find_blobs(color_threshold, roi=roi, merge=True, pixels_area=10)
        if not blobs:
            continue

        # 筛选满足条件的色块 (w和h符合要求)
        filtered_blobs = []
        for blob in blobs:
            # 筛选条件：宽20-400像素，高10-60像素
            if 800 <= blob.w() * blob.h() <= 15000:
                filtered_blobs.append(blob)

        # 如果没有满足条件的色块，标记为未检测到
        if not filtered_blobs:
            continue

        roi_blobs_result[roi_id]['blob_flag'] = 0X01

        # 按面积降序排序
        sorted_blobs = sorted(filtered_blobs, key=lambda b: b.pixels(), reverse=True)

        # 只保留最多两个最大的色块
        sorted_blobs = sorted_blobs[:2]
        roi_blobs_result[roi_id]['largest_blob'] = sorted_blobs[:1]
        # 存储色块对象
        roi_blobs_result[roi_id]['blobs'] = sorted_blobs

        for blob in sorted_blobs:
            # 绘制矩形和十字标记
            img.draw_rectangle(blob[0:4])
            img.draw_cross(blob.cx(), blob.cy())

            # 添加色块中心坐标（仅考虑有效尺寸的色块）
            if 20 <= blob.w() <= 400 and 10 <= blob.h() <= 60:
                roi_blobs_result[roi_id]['centers'].append(blob.cx())

    # 在图像上显示检测结果
    y_positions = [30, 126, 222, 318, 414]
    for i, roi_id in enumerate(ROIS.keys()):
        img.draw_string_advanced(0, y_positions[i], 16,
                                f"{roi_id}:{roi_blobs_result[roi_id]['blob_flag']}",
                                color=(255, 0, 0))

    return roi_blobs_result

# 沿右侧路线巡线函数
def detect_road_choose_right(roi_blobs_result, img):
    total_weighted_deviation = 0
    total_weight = 0
    valid_regions = 0

    # 图像中心x坐标（假设图像宽度640）
    img_center_x = 320

    for roi_id in ['1', '2', '3', '4', '5']:
        result = roi_blobs_result[roi_id]
        if not result['blob_flag'] or not result['centers']:
            continue

        # 该区域所有色块的中心x坐标列表
        centers = result['centers']

        # 选择最右侧的中心点（x坐标最大）
        if centers:
            # 按x坐标降序排序，取最大的（最右侧）
            centers_sorted = sorted(centers, reverse=True)
            center_x = centers_sorted[0]

            # 计算与图像中心的偏差
            deviation = center_x - img_center_x

            # 获取该区域的权重
            weight = REGION_WEIGHTS[roi_id]

            # 累加加权偏差
            total_weighted_deviation += deviation * weight
            total_weight += weight
            valid_regions += 1

    # 计算加权平均偏差
    if valid_regions > 0:
        weighted_error = total_weighted_deviation / total_weight
        weighted_error = weighted_error / 7.0
    else:
        weighted_error = 0  # 没有检测到任何色块

    # 1. 限幅处理：将误差限制在[-90.00, 90.00]范围内
    clamped_error = max(-90.0, min(90.0, weighted_error))

    # 2. 四舍五入到小数点后两位
    clamped_error = round(clamped_error, 2)

    # 3. 格式化误差值，确保正数有'+'号，小数点前后各两位
    # 确定符号
    sign = '+' if clamped_error >= 0 else '-'

    # 取绝对值并分解为整数和小数部分
    abs_value = abs(clamped_error)
    integer_part = int(abs_value)
    fractional_part = int(round((abs_value - integer_part) * 100))

    # 格式化整数部分和小数部分（各两位）
    integer_str = f"{integer_part:02d}"
    fractional_str = f"{fractional_part:02d}"

    # 4. 组合最终格式
    formatted_value = f"{sign}{integer_str}.{fractional_str}"

    # 构造数据包并发送
    data_str = f"<{formatted_value}>"
    uart.send(data_str.encode('utf-8'))
    print(f"Right Path: {data_str}")

    # 在屏幕上显示误差值（显示原始值）
    img.draw_string_advanced(300, 50, 24, f"Error: {clamped_error:.2f} (R)", color=(0, 255, 255))

    # 显示图像
    Display.show_image(img)

def detect_end(roi_blobs_result):
    # 只关注第四个ROI区域
    region = '4'
    result = roi_blobs_result[region]
    # print(result)
    # 如果没有检测到色块，返回False
    if not result['blob_flag']:
        return False

    # 获取该区域最大的色块（面积最大）
    # 注意：我们需要在find_blobs_in_rois中记录色块面积
    # 如果没有记录面积，可以假设最大色块是第一个
    # 这里假设result['largest_blob']存储了最大色块对象
    largest_blob_list = result.get('largest_blob', [])

    # 如果无法获取最大色块信息，返回False
    if not largest_blob_list:
        return False

    # 取列表中的第一个色块（已经是面积最大的）
    largest_blob = largest_blob_list[0]

    # 条件1: 色块面积大于阈值 (1500像素)
    AREA_THRESHOLD_1 = 6000
    AREA_THRESHOLD_2 = 12000
    if largest_blob.w() * largest_blob.h() < AREA_THRESHOLD_1:
        return False
    if largest_blob.w() * largest_blob.h() > AREA_THRESHOLD_2:
        return False
    # 条件2: 色块中心位于ROI中间区域 (水平方向)
    ROI_MIDDLE_X_MIN = 280  # 640的1/3到2/3之间
    ROI_MIDDLE_X_MAX = 360
    ROI_MIDDLE_Y_MIN = 288  # 640的1/3到2/3之间
    ROI_MIDDLE_Y_MAX = 348
    # 获取色块中心坐标
    cx = largest_blob.cx()
    if not (ROI_MIDDLE_X_MIN <= cx <= ROI_MIDDLE_X_MAX):
        return False
    cy = largest_blob.cy()
    if not (ROI_MIDDLE_Y_MIN <= cy <= ROI_MIDDLE_Y_MAX):
        return False
    # 所有条件满足，返回True
    return True

def detect_left_turn(roi_blobs_result):
    """
    检测左转路线（简化版）
    条件：
    1. ROI'1'和ROI'2'中没有色块
    2. ROI'3'或ROI'4'中：
       - 所有色块的面积之和 > 8000
       - 最大色块的width > 150
    """
    # 检查ROI'1' - 必须没有色块
    condition2 = False

    roi1_result = roi_blobs_result['1']
    if roi1_result['blob_flag'] != 0 or len(roi1_result['blobs']) != 0:
        return False

    # 收集ROI3和ROI4的所有色块
    all_blobs = []

    # 获取ROI2的色块（如果有）
    roi2_result = roi_blobs_result['2']
    if roi2_result['blob_flag'] == 1 and len(roi2_result['blobs']) > 0:
        all_blobs.extend(roi2_result['blobs'])
        condition2 = True

    # 获取ROI3的色块（如果有）
    roi3_result = roi_blobs_result['3']
    if roi3_result['blob_flag'] == 1 and len(roi3_result['blobs']) > 0:
        all_blobs.extend(roi3_result['blobs'])

    # 获取ROI4的色块（如果有）
    roi4_result = roi_blobs_result['4']
    if roi4_result['blob_flag'] == 1 and len(roi4_result['blobs']) > 0:
        all_blobs.extend(roi4_result['blobs'])

    # 如果没有检测到任何色块，直接返回False
    if len(all_blobs) == 0:
        return False

    # 计算总面积和最大宽度
    total_area = sum(blob.area() for blob in all_blobs)
    max_width = max(blob.w() for blob in all_blobs)

    # 检查条件：
    # 1. 所有色块面积之和 > 8000
    # 2. 最大色块的width > 150
    if total_area > 8000 and max_width > 150 and condition2:
        uart.write('+99.98')
        condition2 = False
        return False
    elif total_area > 8000 and max_width > 150 and not condition2:
        return True

    return False

# 主车任务一
def task_pattern_1():

    sensor.run()
    time.sleep_ms(1000)
    print("开始执行任务一")

    Turn_Flag = False

    while True:

        fps.tick()
        os.exitpoint()

        # 获取图像
        img = sensor.snapshot()

        # 在ROI区域查找色块
        roi_blobs_result = find_blobs_in_rois(img)

        # 检测转弯标志
        Turn_Flag = detect_left_turn(roi_blobs_result)
        if Turn_Flag:
            print("检测到转弯")
            # time.sleep_ms(300)
            for _ in range(5):
                uart.send("<+99.99>")
            Turn_Flag = False

        # 沿右侧道路行驶
        detect_road_choose_right(roi_blobs_result, img)

        # time.sleep_ms(10)  # 稍微延时，避免过度占用CPU

    print("任务一完成")

def task_pattern_2():
    """
    主车任务二实现
    功能：检测左转路线，当检测到左转后停止
    """

    sensor.run()
    time.sleep_ms(1000)
    print("开始执行任务二")

    left_turn_count = 0  # 左转路线计数器
    last_left_turn_time = time.ticks_ms() - 2000  # 记录上次检测时间，初始值设为2秒前

    while True:
        # 获取图像
        img = sensor.snapshot()

        # 在ROI区域查找色块
        roi_blobs_result = find_blobs_in_rois(img)

        if detect_left_turn(roi_blobs_result):
            # 发送停止信号
            # uart.send("<+99.99>")
            print("已发送停止信号: <S>")

            # 在屏幕上显示停止信息
            img.draw_string_advanced(200, 200, 48, "FINAL STOP", color=(0, 0, 255))
            Display.show_image(img)
            time.sleep(2)
            break

        # 沿右侧道路行驶
        detect_road_choose_right(roi_blobs_result, img)

        # 显示状态信息和时间差
        # status = f"Left Turns: {left_turn_count}/4 | Time diff: {time_diff}ms"
        # img.draw_string_advanced(10, 10, 24, status, color=(255, 255, 255))

        Display.show_image(img)

        # time.sleep_ms(10)  # 稍微延时，避免过度占用CPU

    print("任务二完成")

def task_pattern_3():
    """
    主车任务三实现
    功能：检测左转路线，当检测到左转后停止
    """

    sensor.run()
    time.sleep_ms(1000)
    print("开始执行任务三")

    left_turn_count = 0  # 左转路线计数器
    last_left_turn_time = time.ticks_ms() - 2000  # 记录上次检测时间，初始值设为2秒前

    while True:
        # 获取图像
        img = sensor.snapshot()

        # 在ROI区域查找色块
        roi_blobs_result = find_blobs_in_rois(img)

        if detect_left_turn(roi_blobs_result):
            # 发送停止信号
            # uart.send("<+99.99>")
            print("已发送停止信号: <S>")

            # 在屏幕上显示停止信息
            img.draw_string_advanced(200, 200, 48, "FINAL STOP", color=(0, 0, 255))
            Display.show_image(img)
            time.sleep(2)
            break

        # 沿右侧道路行驶
        detect_road_choose_right(roi_blobs_result, img)

        # 显示状态信息和时间差
        # status = f"Left Turns: {left_turn_count}/4 | Time diff: {time_diff}ms"
        # img.draw_string_advanced(10, 10, 24, status, color=(255, 255, 255))

        Display.show_image(img)

        # time.sleep_ms(10)  # 稍微延时，避免过度占用CPU

    print("任务三完成")

# 主车任务四
def task_pattern_4():

    sensor.run()
    time.sleep_ms(1000)
    print("开始执行任务四")

    end_detected = False

    while True:

        fps.tick()
        os.exitpoint()

        # 获取图像
        img = sensor.snapshot()

        # 在ROI区域查找色块
        roi_blobs_result = find_blobs_in_rois(img)

        # 检测结束标志
        End_Flag = detect_end(roi_blobs_result)
        if End_Flag:
            print("检测到终点，停止")
            time.sleep_ms(300)
            for _ in range(5):
                uart.send("<S>")
            break

        # 沿右侧道路行驶
        detect_road_choose_right(roi_blobs_result, img)

        # time.sleep_ms(10)  # 稍微延时，避免过度占用CPU

    print("任务四完成")

# 主车任务五
def task_pattern_5():

    sensor.run()
    time.sleep_ms(1000)
    print("开始执行任务五")

    end_detected = False

    while True:

        fps.tick()
        os.exitpoint()

        # 获取图像
        img = sensor.snapshot()

        # 在ROI区域查找色块
        roi_blobs_result = find_blobs_in_rois(img)

        # 检测结束标志
        End_Flag = detect_end(roi_blobs_result)
        if End_Flag:
            print("检测到终点，停止")
            time.sleep_ms(300)
            for _ in range(5):
                uart.send("<S>")
            break

        # 沿右侧道路行驶
        detect_road_choose_right(roi_blobs_result, img)

        # time.sleep_ms(10)  # 稍微延时，避免过度占用CPU

    print("任务五完成")

def main():
    init()
    # task_pattern_1()
    buffer = ""  # 串口数据缓冲区
    while True:
        # 读取串口数据
        data = uart.read()

        if data:
            try:
                # 将字节数据解码为字符串
                if isinstance(data, bytes):
                    buffer += data.decode('utf-8')
                else:
                    buffer += data
            except Exception as e:
                print("解码错误:", e)
                buffer = ""
                continue

            # 检查是否收到完整消息
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)  # 提取第一行
                line = line.strip()  # 移除首尾空白字符

                # 模式匹配与任务调度
                if line == PATTERN_1:
                    print("收到模式1指令")
                    task_pattern_1()
                elif line == PATTERN_2:
                    print("收到模式2指令")
                    task_pattern_2()
                elif line == PATTERN_3:
                    print("收到模式3指令")
                    task_pattern_3()
                elif line == PATTERN_4:
                    print("收到模式4指令")
                    task_pattern_4()
                elif line == PATTERN_5:
                    print("收到模式5指令")
                    task_pattern_5()
                elif line == PATTERN_6:
                    print("收到模式6指令")
                    task_pattern_6()
                else:
                    print("收到无效指令:", line)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("程序被用户中断")
    finally:
        # 清理资源
        if sensor:
            sensor.stop()
        Display.deinit()
        MediaManager.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
