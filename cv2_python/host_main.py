import os
import sys
import argparse
import cv2
import numpy as np
import time 
import threading
from collections import defaultdict
try:
    import serial
    SERIAL_IMPORT_ERROR = None
except Exception as exc:
    serial = None
    SERIAL_IMPORT_ERROR = exc
try:
    from pyzbar.pyzbar import decode as pyzbar_decode
    PYZBAR_IMPORT_ERROR = None
except Exception as exc:
    pyzbar_decode = None
    PYZBAR_IMPORT_ERROR = exc
# 启用优化（默认通常是True，但建议显式设置）
#cv2.setUseOptimized(True)

# 检查是否已启用优化
#print("优化状态:", cv2.useOptimized())  # 输出应为 True

# 设置线程数为 4（根据 CPU 核心数调整）
#cv2.setNumThreads(4)

# 检查当前线程数
#print("线程数:", cv2.getNumThreads())  # 输出应为 4
A = ['1','3','2','+','3','1','2']   # 用于存储二维码数据
#A=[]
C_1 = None  
C_2 = None  
C_1_loop2 = None
C_2_loop2 = None
cap=None
cap2=None

# 调试开关：默认不自动跑视觉任务，手工调试时改这里或使用命令行参数。
DEBUG_RUN_QR = False
DEBUG_RUN_RING = False
DEBUG_RUN_RING_CENTERED = False
DEBUG_RUN_MADUO = False
DEBUG_RUN_MADUO_CENTERED = False

HOST_SERIAL_PORT = os.environ.get("GCS_HOST_SERIAL", "/dev/ttyUSB0")
HOST_SERIAL_BAUD = int(os.environ.get("GCS_HOST_BAUD", "115200"))
DRY_RUN_SERIAL = os.environ.get("GCS_DRY_RUN_SERIAL", "0") == "1"
CAMERA_QR_DEVICE = os.environ.get("GCS_QR_CAMERA", "/dev/video_xia0")
CAMERA_UPPER_DEVICE = os.environ.get("GCS_UPPER_CAMERA", "/dev/video_shang0")
CAMERA_BACKEND = os.environ.get("GCS_CAMERA_BACKEND", "").lower()
roi_params_dingxin=(160,140,320,210)
roi_params = (220,120,200, 140)   # 正下方A3    c1
roi_params1 = (150,300,160,160)  # 左下角A1
roi_params2 = (330, 300, 160,160) # 右下角A2
roi_param_zhuazi=(210,300,220,160)#爪子
ser = None

def open_host_serial():
    global ser
    if ser is not None and ser.is_open:
        return ser
    if serial is None:
        raise RuntimeError(f"无法导入 pyserial，不能打开上位机通信口: {SERIAL_IMPORT_ERROR}")
    ser = serial.Serial(
        port=HOST_SERIAL_PORT,
        baudrate=HOST_SERIAL_BAUD,
        bytesize=serial.EIGHTBITS,
        stopbits=serial.STOPBITS_ONE,
        parity=serial.PARITY_NONE,
        timeout=0.1,
    )
    return ser

def close_host_serial():
    global ser
    if ser is not None and ser.is_open:
        ser.close()

def set_dry_run_serial(enabled):
    global DRY_RUN_SERIAL
    DRY_RUN_SERIAL = enabled

def parse_camera_device(device):
    if isinstance(device, str) and device.isdigit():
        return int(device)
    return device

def select_camera_backend():
    backends = {
        "any": cv2.CAP_ANY,
        "v4l2": cv2.CAP_V4L2,
        "dshow": cv2.CAP_DSHOW,
        "msmf": cv2.CAP_MSMF,
    }
    if CAMERA_BACKEND in backends:
        return backends[CAMERA_BACKEND]
    if os.name == "nt":
        return cv2.CAP_DSHOW
    return cv2.CAP_V4L2

def open_camera(device):
    return cv2.VideoCapture(parse_camera_device(device), select_camera_backend())

def ensure_upper_camera():
    global cap2
    if cap2 is None or not cap2.isOpened():
        cap2 = open_camera(CAMERA_UPPER_DEVICE)
    return cap2

def window_closed(name):
    try:
        return cv2.getWindowProperty(name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True

def apply_optional_camera_controls(cap_obj):
    controls = {
        "GCS_CAMERA_AUTOFOCUS": cv2.CAP_PROP_AUTOFOCUS,
        "GCS_CAMERA_FOCUS": cv2.CAP_PROP_FOCUS,
        "GCS_CAMERA_ZOOM": cv2.CAP_PROP_ZOOM,
    }
    for env_name, prop in controls.items():
        value = os.environ.get(env_name)
        if value is None:
            continue
        try:
            cap_obj.set(prop, float(value))
            print(f"{env_name}={cap_obj.get(prop)}")
        except ValueError:
            print(f"忽略无效摄像头参数 {env_name}={value!r}")

def adjust_camera_property(cap_obj, prop, delta, min_value=0, max_value=255):
    current = cap_obj.get(prop)
    if current < 0:
        return current
    next_value = max(min_value, min(max_value, current + delta))
    cap_obj.set(prop, next_value)
    return cap_obj.get(prop)

def build_data_packet(data):
    try:
        payload = str(data).encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError(f"下位机通信数据只支持 ASCII: {data!r}") from exc
    payload = payload[:9].ljust(9, b"\x00")
    return bytes([0xFF]) + payload + bytes([0xFE])

def send_data(data):
    data_packet = build_data_packet(data)
    if DRY_RUN_SERIAL:
        print(f"[DRY-RUN SERIAL] send_data skipped: {data_packet}")
        return
    port = open_host_serial()
    port.write(data_packet)
    print(f"发送: {data_packet}")

def receive_data():
    port = open_host_serial()
    if port.in_waiting > 0:
        return port.read(1).decode("utf-8", errors="ignore")
    return None

def restart_program():
    print("接收到重启指令，程序将在20秒后重新启动...")
    close_host_serial()
    time.sleep(20)
    python = sys.executable
    os.execl(python, python, *sys.argv)

# 获取颜色名称（共享函数）
def get_color_name(bgr_val):
    """
    根据BGR值判断颜色，仅支持红、绿、蓝三种颜色。
    """
    blue, green, red = bgr_val
    if red > blue and red > green:
        return "Red", "1"
    elif green > red and green > blue:
        return "Green", "2"
    elif blue > red and blue > green:
        return "Blue", "3"
    else:
        return "Unknown", "0"
    
def run_erweima():
    global A
    #if not ser.is_open:
        #ser.open()
    print("执行二维码检测任务...")
    
    cap0 = open_camera(CAMERA_QR_DEVICE)
    if not cap0.isOpened():
        print("无法打开摄像头")
        send_data("000000004")
        return

    # 初始化状态变量
    consistent_count = 0
    last_data = None
    last_valid_data = None

    try:
        while True:
            ret, frame = cap0.read()
            if not ret:
                print("无法接收帧，退出...")
                send_data("000000004")
                break

            # 优先使用 pyzbar；未安装 zbar 时显式改用 OpenCV 自带二维码检测器。
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            current_data = None

            if pyzbar_decode is not None:
                decoded_objects = pyzbar_decode(gray)
                # 处理所有检测到的二维码
                for obj in decoded_objects:
                    try:
                        data = obj.data.decode('utf-8').strip()
                        if len(data) == 7:  # 只处理7字符长度的数据
                            current_data = data
                            break
                    except Exception:
                        continue
            else:
                detector = cv2.QRCodeDetector()
                data, _, _ = detector.detectAndDecode(gray)
                data = data.strip() if data else ""
                if len(data) == 7:
                    current_data = data

            if current_data:
                print(f"检测到二维码数据: {current_data}")

                # 连续相同数据验证
                if current_data == last_data:
                    consistent_count += 1
                else:
                    consistent_count = 1
                    last_data = current_data

                # 连续3次相同则确认有效
                if consistent_count >= 3:
                    A = list(current_data)  # 存储到全局数组A
                    print(f"解析二维码数据到数组 A: {A}")
                    
                    # 格式化发送数据（7字符+补零）
                    formatted_data = current_data.ljust(7, '0')[:7] + '0'
                    send_data(formatted_data)
                    last_valid_data = current_data
                    break

            # 显示检测画面（原代码注释掉了，这里保持相同处理）
            # cv2.imshow('QR Code Scanner', frame)
            if cv2.waitKey(1) == ord('q'):
                print("用户手动终止识别")
                break

    finally:
        # 确保资源释放
        cap0.release()
        cv2.destroyAllWindows()
        if last_valid_data is None:
            send_data("000000004")
    
def run_wukuaiyuanxin_1():
    """第一圈物块定标（普通圆形）- 单检测区域版"""
    global cap
    if not cap.isOpened():
        print("摄像头初始化失败")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 用户配置参数
    color_thresholds = {
        'red': {'lower1': [0, 100, 100], 'upper1': [10, 255, 255],
                'lower2': [160, 100, 100], 'upper2': [180, 255, 255]},
        'green': {'lower': [35, 40, 40], 'upper': [89, 255, 255]},
        'blue': {'lower': [90, 60, 60], 'upper': [140, 255, 255]}
    }
    
    detection_area = [120, 80, 400, 217]
    circle_params = {'min_radius': 35, 'max_radius': 45, 'param1': 25, 'param2': 25}
    stability_settings = {
        'threshold': 30,
        'max_pixel_move': 10,
        'color_stable_threshold': 15,
        'color_confidence': 0.7
    }
    timeout_settings = {'timeout_ms': 100}
    kernel_size = 3
    
    # 初始化变量
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    stable_count = 0
    first_center = None
    last_detection_time = None
    color_history = []
    final_color = None
    last_radius = 0  # 添加radius变量存储
    last_color_code = None  # 添加color_code变量存储
    
    def detect_stacking_and_color(frame):
        nonlocal last_radius, last_color_code
        x, y, w, h = detection_area
        roi_img = frame[y:y+h, x:x+w]
        
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2, 2)
        
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, 1, 20,
            param1=circle_params['param1'],
            param2=circle_params['param2'],
            minRadius=circle_params['min_radius'],
            maxRadius=circle_params['max_radius']
        )
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0] + x, i[1] + y)
                radius = i[2]
                last_radius = radius  # 保存半径值
                
                mask = np.zeros_like(gray)
                cv2.circle(mask, (i[0], i[1]), radius, 255, -1)
                hsv_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
                
                # 颜色识别逻辑
                lower_red1 = np.array(color_thresholds['red']['lower1'])
                upper_red1 = np.array(color_thresholds['red']['upper1'])
                lower_red2 = np.array(color_thresholds['red']['lower2'])
                upper_red2 = np.array(color_thresholds['red']['upper2'])
                lower_green = np.array(color_thresholds['green']['lower'])
                upper_green = np.array(color_thresholds['green']['upper'])
                lower_blue = np.array(color_thresholds['blue']['lower'])
                upper_blue = np.array(color_thresholds['blue']['upper'])
                
                red_mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
                red_mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
                red_mask = cv2.bitwise_or(red_mask1, red_mask2)
                green_mask = cv2.inRange(hsv_img, lower_green, upper_green)
                blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
                
                # 形态学操作
                red_mask = cv2.erode(red_mask, kernel, iterations=1)
                red_mask = cv2.dilate(red_mask, kernel, iterations=2)
                green_mask = cv2.erode(green_mask, kernel, iterations=2)
                green_mask = cv2.dilate(green_mask, kernel, iterations=6)
                blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
                blue_mask = cv2.dilate(blue_mask, kernel, iterations=3)
                
                # 计算颜色区域面积
                red_area = cv2.countNonZero(cv2.bitwise_and(red_mask, mask))
                green_area = cv2.countNonZero(cv2.bitwise_and(green_mask, mask))
                blue_area = cv2.countNonZero(cv2.bitwise_and(blue_mask, mask))
                
                # 确定主颜色
                max_area = max(red_area, green_area, blue_area)
                if max_area == red_area:
                    color_code = "1"
                elif max_area == green_area:
                    color_code = "2"
                else:
                    color_code = "3"
                
                last_color_code = color_code  # 保存颜色代码
                formatted_data = f"{center[0]:04}{center[1]:04}{color_code}{radius:01}"
                return formatted_data, center, color_code
        return None, None, None
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头帧")
            break
        
        # 绘制检测区域
        x, y, w, h = detection_area
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        current_time = cv2.getTickCount()
        data, current_center, current_color = detect_stacking_and_color(frame)
        
        if data and current_color:
            last_detection_time = current_time
            color_history.append(current_color)
            
            if first_center is None:
                first_center = current_center
                stable_count = 1
            else:
                distance = np.sqrt((current_center[0] - first_center[0])**2 + 
                                 (current_center[1] - first_center[1])**2)
                if distance < stability_settings['max_pixel_move']:
                    stable_count += 1
                else:
                    stable_count = 1
                    first_center = current_center
                    color_history = []
            
            if len(color_history) >= stability_settings['color_stable_threshold']:
                color_counter = defaultdict(int)
                for color in color_history:
                    color_counter[color] += 1
                
                most_common_color = max(color_counter.items(), key=lambda x: x[1])[0]
                confidence = color_counter[most_common_color] / len(color_history)
                
                if confidence >= stability_settings['color_confidence']:
                    final_color = most_common_color
                    print(f"颜色识别稳定: {most_common_color} (置信度: {confidence:.2f})")
            
            if (stable_count >= stability_settings['threshold'] and 
                final_color is not None):
                # 使用保存的last_radius和last_color_code变量
                formatted_data = f"{current_center[0]:04}{current_center[1]:04}{final_color}{last_radius:01}"
                send_data(formatted_data)
                print(f"发送的物块数据: {formatted_data}")
                C_1 = final_color
                break
        else:
            if last_detection_time is not None:
                elapsed_time = (current_time - last_detection_time) / cv2.getTickFrequency() * 1000
                if elapsed_time > timeout_settings['timeout_ms']:
                    print("检测超时，跳过本次识别")
                    continue
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()
    return C_1

def run_wukuaiyuanxin_2():
    """第一圈物块定标（普通圆形）- 单检测区域版"""
    global cap
    if not cap.isOpened():
        print("摄像头初始化失败")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # 用户配置参数
    color_thresholds = {
        'red': {'lower1': [0, 100, 100], 'upper1': [10, 255, 255],
                'lower2': [160, 100, 100], 'upper2': [180, 255, 255]},
        'green': {'lower': [35, 40, 40], 'upper': [89, 255, 255]},
        'blue': {'lower': [90, 60, 60], 'upper': [140, 255, 255]}
    }
    
    detection_area = [220,80,200,200]
    circle_params = {'min_radius': 35, 'max_radius': 45, 'param1': 25, 'param2': 25}
    stability_settings = {
        'threshold': 15,
        'max_pixel_move': 10,
        'color_stable_threshold': 15,
        'color_confidence': 0.7
    }
    timeout_settings = {'timeout_ms': 1000}
    kernel_size = 3
    
    # 初始化变量
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    stable_count = 0
    first_center = None
    last_detection_time = None
    color_history = []
    final_color = None
    last_radius = 0  # 添加radius变量存储
    last_color_code = None  # 添加color_code变量存储
    
    def detect_stacking_and_color(frame):
        nonlocal last_radius, last_color_code
        x, y, w, h = detection_area
        roi_img = frame[y:y+h, x:x+w]
        
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2, 2)
        
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, 1, 20,
            param1=circle_params['param1'],
            param2=circle_params['param2'],
            minRadius=circle_params['min_radius'],
            maxRadius=circle_params['max_radius']
        )
        
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0] + x, i[1] + y)
                radius = i[2]
                last_radius = radius  # 保存半径值
                
                mask = np.zeros_like(gray)
                cv2.circle(mask, (i[0], i[1]), radius, 255, -1)
                hsv_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
                
                # 颜色识别逻辑
                lower_red1 = np.array(color_thresholds['red']['lower1'])
                upper_red1 = np.array(color_thresholds['red']['upper1'])
                lower_red2 = np.array(color_thresholds['red']['lower2'])
                upper_red2 = np.array(color_thresholds['red']['upper2'])
                lower_green = np.array(color_thresholds['green']['lower'])
                upper_green = np.array(color_thresholds['green']['upper'])
                lower_blue = np.array(color_thresholds['blue']['lower'])
                upper_blue = np.array(color_thresholds['blue']['upper'])
                
                red_mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
                red_mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
                red_mask = cv2.bitwise_or(red_mask1, red_mask2)
                green_mask = cv2.inRange(hsv_img, lower_green, upper_green)
                blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
                
                # 形态学操作
                red_mask = cv2.erode(red_mask, kernel, iterations=1)
                red_mask = cv2.dilate(red_mask, kernel, iterations=2)
                green_mask = cv2.erode(green_mask, kernel, iterations=2)
                green_mask = cv2.dilate(green_mask, kernel, iterations=6)
                blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
                blue_mask = cv2.dilate(blue_mask, kernel, iterations=3)
                
                # 计算颜色区域面积
                red_area = cv2.countNonZero(cv2.bitwise_and(red_mask, mask))
                green_area = cv2.countNonZero(cv2.bitwise_and(green_mask, mask))
                blue_area = cv2.countNonZero(cv2.bitwise_and(blue_mask, mask))
                
                # 确定主颜色
                max_area = max(red_area, green_area, blue_area)
                if max_area == red_area:
                    color_code = "1"
                elif max_area == green_area:
                    color_code = "2"
                else:
                    color_code = "3"
                
                last_color_code = color_code  # 保存颜色代码
                formatted_data = f"{center[0]:04}{center[1]:04}{color_code}{radius:01}"
                return formatted_data, center, color_code
        return None, None, None
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头帧")
            break
        
        # 绘制检测区域
        x, y, w, h = detection_area
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        current_time = cv2.getTickCount()
        data, current_center, current_color = detect_stacking_and_color(frame)
        
        if data and current_color:
            last_detection_time = current_time
            color_history.append(current_color)
            
            if first_center is None:
                first_center = current_center
                stable_count = 1
            else:
                distance = np.sqrt((current_center[0] - first_center[0])**2 + 
                                 (current_center[1] - first_center[1])**2)
                if distance < stability_settings['max_pixel_move']:
                    stable_count += 1
                else:
                    stable_count = 1
                    first_center = current_center
                    color_history = []
            
            if len(color_history) >= stability_settings['color_stable_threshold']:
                color_counter = defaultdict(int)
                for color in color_history:
                    color_counter[color] += 1
                
                most_common_color = max(color_counter.items(), key=lambda x: x[1])[0]
                confidence = color_counter[most_common_color] / len(color_history)
                
                if confidence >= stability_settings['color_confidence']:
                    final_color = most_common_color
                    print(f"颜色识别稳定: {most_common_color} (置信度: {confidence:.2f})")
            
            if (stable_count >= stability_settings['threshold'] and 
                final_color is not None):
                # 使用保存的last_radius和last_color_code变量
                formatted_data = f"{current_center[0]:04}{current_center[1]:04}{final_color}{last_radius:01}"
                #send_data(formatted_data)
                print(f"检测转过来的第二个物块数据: {formatted_data}")
                C_2 = final_color
                break
        else:
            if last_detection_time is not None:
                elapsed_time = (current_time - last_detection_time) / cv2.getTickFrequency() * 1000
                if elapsed_time > timeout_settings['timeout_ms']:
                    print("检测超时，跳过本次识别")
                    continue
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cv2.destroyAllWindows()
    return C_2


def run_wukuaiyuanxin_xuanzequyu(i, yanse):  ##第一圈物块定标（普通圆形）
    # 新增颜色阈值定义（来自ceshi022001.py），增加红色阈值并合并
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    lower_green = np.array([30, 40, 40])
    upper_green = np.array([89, 255, 255])
    lower_blue = np.array([90, 50, 50])
    upper_blue = np.array([140, 255, 255])
    
    kernel = np.ones((3, 3), np.uint8)
    
    if i == 1:
        roi_params_xuanze = (220, 120, 200, 140)   # 正下方A3
    elif i == 3:
        roi_params_xuanze = (80, 300, 200, 160)  # 左下角A1c
    elif i == 2:
        roi_params_xuanze = (320, 300, 200, 160) # 右下角A2
    elif i == 0:
        roi_params_xuanze = (210, 300, 220, 160) # 爪子识别区域
        
    def detect_stacking_and_color(frame, loop_radius=(35, 45), roi=roi_params_xuanze):
        """修改后的颜色识别逻辑"""
        roi_img = frame[roi[1]:roi[1] + roi[3], roi[0]:roi[0] + roi[2]]
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2, 2)
        circles = cv2.HoughCircles(
            gray, 
            cv2.HOUGH_GRADIENT, 
            1, 
            20, 
            param1=25, 
            param2=25, 
            minRadius=loop_radius[0], 
            maxRadius=loop_radius[1]
        )

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0] + roi[0], i[1] + roi[1])
                radius = i[2]

                # 使用HSV颜色空间进行颜色识别
                mask = np.zeros_like(gray)
                cv2.circle(mask, (i[0], i[1]), radius, 255, -1)
                hsv_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
                
                # 创建颜色掩膜，合并红色阈值
                red_mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
                red_mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
                red_mask = red_mask1 + red_mask2
                green_mask = cv2.inRange(hsv_img, lower_green, upper_green)
                blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)

                # 形态学操作
                red_mask = cv2.erode(red_mask, kernel, iterations=1)
                red_mask = cv2.dilate(red_mask, kernel, iterations=2)
                green_mask = cv2.erode(green_mask, kernel, iterations=2)
                green_mask = cv2.dilate(green_mask, kernel, iterations=6)
                blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
                blue_mask = cv2.dilate(blue_mask, kernel, iterations=3)

                # 计算颜色区域面积
                red_area = cv2.countNonZero(cv2.bitwise_and(red_mask, mask))
                green_area = cv2.countNonZero(cv2.bitwise_and(green_mask, mask))
                blue_area = cv2.countNonZero(cv2.bitwise_and(blue_mask, mask))
                
                # 确定主颜色
                max_area = max(red_area, green_area, blue_area)
                if max_area == red_area:
                    color_code = "1"
                elif max_area == green_area:
                    color_code = "2"
                else:
                    color_code = "3"

                formatted_data = f"{center[0]:04}{center[1]:04}{color_code}{radius:01}"
                return formatted_data, center
        return None, None

    # 显式初始化摄像头
    cap = open_camera(CAMERA_UPPER_DEVICE)
    if not cap.isOpened():
        print("摄像头初始化失败")
        return
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    stability_threshold = 20
    stable_count = 0
    last_center = None
    last_detection_time = None  # 记录上次成功检测的时间
    timeout_ms = 300  # 超时时间100毫秒

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头帧")
            break

        current_time = cv2.getTickCount()  # 获取当前时间戳
        data, current_center = detect_stacking_and_color(frame)

        if data and data[8] == yanse:  # 只有当颜色匹配时才处理
            last_detection_time = current_time  # 更新最后检测时间
            
            if last_center is not None:
                distance = np.sqrt((current_center[0] - last_center[0])**2 + 
                             (current_center[1] - last_center[1])**2)
                if distance < 10:
                    stable_count += 1
                else:
                    stable_count = 1  # 位置变化过大，重置计数
            else:
                stable_count = 1  # 第一次检测
            
            last_center = current_center
            
            if stable_count >= stability_threshold:
                send_data("987654321")
                print(f"判断物块颜色确实是{yanse}，稳定，发送的物块数据: 987654321")
                break
        else:
            # 检查是否超时（100ms内没有检测到）
            if last_detection_time is not None:
                elapsed_time = (current_time - last_detection_time) / cv2.getTickFrequency() * 1000  # 毫秒
                if elapsed_time > timeout_ms:
                    print("检测超时，跳过本次识别")
                    continue  # 跳过本次识别但不重置stable_count
            
            # 如果没有超时或者从未检测到过，保持stable_count不变
            # 注意：这里不重置stable_count = 0
        # 显示图像（可选）
        # cv2.imshow("物块检测", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头资源
    cap.release()
    cv2.destroyAllWindows()

def run_wukuaiyuanxin_A():
    global A1, A2, A3
    global cap
    
    # 颜色阈值定义
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    lower_green = np.array([35, 40, 40])
    upper_green = np.array([89, 255, 255])
    lower_blue = np.array([90, 60, 60])
    upper_blue = np.array([130, 255, 255])
    kernel = np.ones((3, 3), np.uint8)

    def detect_color_only(frame, roi):
        """只检测颜色，不进行圆形识别"""
        try:
            roi_img = frame[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
            hsv_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
            
            # 颜色检测
            red_mask1 = cv2.inRange(hsv_img, lower_red1, upper_red1)
            red_mask2 = cv2.inRange(hsv_img, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            green_mask = cv2.inRange(hsv_img, lower_green, upper_green)
            blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)

            # 形态学操作
            red_mask = cv2.erode(red_mask, kernel, iterations=1)
            red_mask = cv2.dilate(red_mask, kernel, iterations=2)
            green_mask = cv2.erode(green_mask, kernel, iterations=2)
            green_mask = cv2.dilate(green_mask, kernel, iterations=5)
            blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
            blue_mask = cv2.dilate(blue_mask, kernel, iterations=2)

            # 计算颜色区域面积
            red_area = cv2.countNonZero(red_mask)
            green_area = cv2.countNonZero(green_mask)
            blue_area = cv2.countNonZero(blue_mask)
            
            if red_area > green_area and red_area > blue_area:
                color_code = "1"
            elif green_area > red_area and green_area > blue_area:
                color_code = "2"
            else:
                color_code = "3"

            formatted_data = f"00000000{color_code}00"  # X和Y坐标设为0，半径设为0
            return formatted_data, (0, 0)  # 返回固定中心点
        except Exception as e:
            print(f"检测过程中发生错误: {e}")
        return None, None

    def stable_detect(roi, single_timeout_ms=200):
        """持续检测直到成功，每次检测尝试最多single_timeout_ms毫秒"""
        while True:
            start_time = cv2.getTickCount()
            detected = False
            
            # 单次检测循环
            while True:
                current_time = cv2.getTickCount()
                elapsed_time = (current_time - start_time) / cv2.getTickFrequency() * 1000
                
                # 检查是否超时
                if elapsed_time > single_timeout_ms:
                    print(f"单次检测超时 ({single_timeout_ms}ms)，重新开始检测")
                    break
                    
                ret, frame = cap.read()
                if not ret:
                    print("无法读取摄像头帧")
                    time.sleep(0.01)  # 防止CPU占用过高
                    continue
                    
                data, _ = detect_color_only(frame, roi)
                
                if data:
                    print(f"发送的物块数据: {data}")
                    print(f"本次检测时间为: {elapsed_time}ms")
                    elapsed_time=0
                    return data[8]  # 返回颜色代码
                    
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    raise KeyboardInterrupt("用户主动终止检测")
                
                time.sleep(0.01)  # 适当的延迟防止CPU占用过高

    # 主流程
    if not cap.isOpened():
        print("摄像头初始化失败")
        return None, None, None
        
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # 检测A1和A2，直到成功为止
        A1 = stable_detect(roi_params1)
        time.sleep(0.1)
        A2 = stable_detect(roi_params2)
        
        # 如果颜色相同则重新检测一次
        if A1 == A2:
            print("两个区域颜色相同，重新检测...")
            time.sleep(0.2)
            A1 = stable_detect(roi_params1)
            time.sleep(0.1)
            A2 = stable_detect(roi_params2)
        
        # 通过排除法计算A3
        all_colors = {'1', '2', '3'}
        detected_colors = {A1, A2}
        remaining_colors = all_colors - detected_colors
        A3 = remaining_colors.pop() if len(remaining_colors) == 1 else None
        
        return A1, A2, A3
        
    except KeyboardInterrupt:
        print("检测被用户中断")
        return None, None, None
    except Exception as e:
        print(f"主流程发生错误: {e}")
        return None, None, None
    finally:
        cv2.destroyAllWindows()
        
def main_logic_loop1():
    global C_1, C_2
    global INDEX
    global INDEX1
    global INDEX2
    """完整主逻辑（来自ceshi1.py改进）"""
    #A1=run_wukuaiyuanxin_A1()
    #A2=run_wukuaiyuanxin_A2()
    #A3=run_wukuaiyuanxin_A3()
    A1,A2,A3=run_wukuaiyuanxin_A()
    print(f"快速扫描结果 - A1:{A1},A2: {A2}, A3: {A3}")
    # 判断旋转方向
    is_clockwise = (A1 == C_1)
    B = [A3, A2, A1] if is_clockwise else [A3, A2, A1]
    B1 = [A2, A1, A3] if is_clockwise else [A1, A3, A2]
    B2 = [A1, A3, A2] if is_clockwise else [A2, A1, A3]
    for index,element in enumerate(B):
        if element ==A[0]:
            print(f"任务码的第一个数字在位置{index+1}")
            INDEX=index+1
    for index1,element1 in enumerate(B1):
        if element1 ==A[1]:
            print(f"任务码的第二个数字在位置{index1+1}")
            INDEX1=index1+1
    for index2,element2 in enumerate(B2):
        if element2 ==A[2]:
            print(f"任务码的第三个数字在位置{index2+1}")
            INDEX2=index2+1
    print(f"当前B数组: {B}，旋转方向:逆时针") if is_clockwise else print(f"当前B数组: {B}，旋转方向:顺时针")
    data=f"{INDEX}{INDEX1}{INDEX2}456789"
    send_data(data)
    print(f"已发送数据{INDEX}{INDEX1}{INDEX2}456789")

def LOOP1():
    global C_1,C_2
    C_2=run_wukuaiyuanxin_2()
    if C_1 != C_2:
        print("不相等，开始检测")
        main_logic_loop1()
    else:
        print("相等，开始循环")
        # 持续检测直到颜色变化
        while True:
            C_2=run_wukuaiyuanxin_2()
            if C_2 is not None and C_2 != C_1:
                print("不相等，开始检测")
                main_logic_loop1()
                break
            time.sleep(0.1)  # 降低CPU占用

def main_logic_loop2():
    global C_1_loop2, C_2_loop2
    global INDEX_loop2
    global INDEX1_loop2
    global INDEX2_loop2
    """完整主逻辑（来自ceshi1.py改进）"""
    #A1=run_wukuaiyuanxin_A1()
    #A2=run_wukuaiyuanxin_A2()
    #A3=run_wukuaiyuanxin_A3()
    A1,A2,A3=run_wukuaiyuanxin_A()
    print(f"快速扫描结果 - A1:{A1},A2: {A2}, A3: {A3}")
    # 判断旋转方向
    is_clockwise = (A1 == C_1_loop2)
    B = [A3, A2, A1] if is_clockwise else [A3, A2, A1]
    B1 = [A2, A1, A3] if is_clockwise else [A1, A3, A2]
    B2 = [A1, A3, A2] if is_clockwise else [A2, A1, A3]
    for index,element in enumerate(B):
        if element ==A[4]:
            print(f"任务码的第一个数字在位置{index+1}")
            INDEX_loop2=index+1
    for index1,element1 in enumerate(B1):
        if element1 ==A[5]:
            print(f"任务码的第二个数字在位置{index1+1}")
            INDEX1_loop2=index1+1
    for index2,element2 in enumerate(B2):
        if element2 ==A[6]:
            print(f"任务码的第三个数字在位置{index2+1}")
            INDEX2_loop2=index2+1
    print(f"当前B数组: {B}，旋转方向:逆时针") if is_clockwise else print(f"当前B数组: {B}，旋转方向:顺时针")
    data=f"{INDEX_loop2}{INDEX1_loop2}{INDEX2_loop2}456789"
    send_data(data)
    print(f"已发送数据{INDEX_loop2}{INDEX1_loop2}{INDEX2_loop2}456789")

def LOOP2():
    global C_1_loop2,C_2_loop2
    C_2_loop2=run_wukuaiyuanxin_2()
    if C_1_loop2 != C_2_loop2:
        print("不相等，开始检测")
        main_logic_loop2()
    else:
        print("相等，开始循环")
        # 持续检测直到颜色变化
        while True:
            C_2_loop2=run_wukuaiyuanxin_2()
            if C_2_loop2 is not None and C_2_loop2 != C_1_loop2:
                print("不相等，开始检测")
                main_logic_loop2()
                break
            time.sleep(0.1)  # 降低CPU占用

def detect_boundary_line():
    global cap2
    if not cap2.isOpened():
        print("无法打开摄像头")
        return
    
    # 设置摄像头分辨率
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    # 参数配置
    canny_threshold1 = 50
    canny_threshold2 = 150
    hough_threshold = 20
    min_line_length = 140
    max_line_gap = 10
    roi_top = 0.1    # ROI区域顶部边界
    roi_bottom = 0.9  # ROI区域底部边界
    roi_left = 0.1    # ROI区域左侧边界
    roi_right = 0.9   # ROI区域右侧边界
    variance_threshold = 1.0  # 方差容忍阈值(角度平方)
    max_attempts = 10  # 最大尝试次数
    
    attempt_count = 0
    while attempt_count < max_attempts:
        angle_results = []  # 存储三次有效检测的结果
        
        # 收集三次有效检测
        while len(angle_results) < 3:
            ret, frame = cap2.read()
            if not ret:
                print("无法读取帧")
                continue
                
            # 图像处理流程
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            height, width = gray.shape
            roi = gray[int(height*roi_top):int(height*roi_bottom), 
                      int(width*roi_left):int(width*roi_right)]
            blurred = cv2.GaussianBlur(roi, (5, 5), 0)
            edges = cv2.Canny(blurred, canny_threshold1, canny_threshold2)
            
            # 直线检测
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=hough_threshold,
                                  minLineLength=min_line_length, maxLineGap=max_line_gap)
            
            if lines is not None:
                angles = []
                weights = []
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    angle = np.arctan2(y2 - y1, x2 - x1)
                    angles.append(angle)
                    weights.append(np.sqrt((x2-x1)**2 + (y2-y1)**2))  # 线段长度作为权重
                
                if angles:
                    # 计算加权平均角度
                    total_weight = sum(weights)
                    avg_angle_rad = sum(a*w for a, w in zip(angles, weights)) / total_weight
                    avg_angle_deg = np.degrees(avg_angle_rad)
                    angle_results.append(avg_angle_deg)
                    print(f"第 {len(angle_results)} 次有效检测 - 角度: {avg_angle_deg:.2f}°")
            
            time.sleep(0.05)
        
        # 计算方差
        if len(angle_results) == 3:
            variance = np.var(angle_results)
            print(f"本次检测方差: {variance:.2f} (阈值: {variance_threshold})")
            
            if variance <= variance_threshold:
                # 方差在容忍范围内，计算最终角度
                final_angle = round(np.mean(angle_results), 1) - 2.0  # 减去0.7度校准值
                final_angle = max(-9.9, min(9.9, final_angle))
                
                # 准备发送数据
                sign = 1 if final_angle >= 0 else 0
                abs_angle = abs(final_angle)
                integer_part = int(abs_angle)
                decimal_part = int(round(abs_angle * 10)) % 10
                data_str = f"000000{sign}{integer_part}{decimal_part}"
                #data_str = f"000000000"
                send_data(data_str)
                print(f"角度稳定，发送数据: {data_str}，平均角度: {final_angle}°")
                return
            else:
                print("角度波动过大，重新检测...")
                attempt_count += 1
        else:
            print("未能收集到3次有效检测，重新尝试...")
            attempt_count += 1
    
    # 达到最大尝试次数仍未获得稳定结果
    send_data("000000004")  # 发送全零数据表示检测失败
    print("达到最大尝试次数仍未获得稳定角度，发送默认数据")

def run_sehuanyuanxin2():  # 初赛放物块的时候，色环第一次定标的程序（地上的圆形）
    global cap2
    # 红色双阈值范围
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    lower_green = np.array([30, 40, 40])
    upper_green = np.array([89, 255, 255])
    lower_blue = np.array([90, 50, 90])
    upper_blue = np.array([140, 255, 255])
    kernel = np.ones((3, 3), np.uint8)

    # 检测区域参数
    center_x, center_y = 320, 240
    square_side = 433
    square_side1 = 222
    square_side2 = 240
    half_side = square_side // 2
    half_side1 = square_side1 // 2
    half_side2 = square_side2 // 2

    # 初始化摄像头
    if not ensure_upper_camera().isOpened():
        print("无法读取摄像头帧")
        return
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 稳定性检测参数（调整为更宽松的条件）
    POSITION_STABLE_THRESHOLD = 10   # 位置稳定阈值(像素)
    COLOR_STABLE_THRESHOLD = 5       # 颜色稳定所需出现次数（非连续）
    COLOR_CONFIDENCE = 0.5           # 颜色置信度要求
    FRAME_WINDOW_SIZE = 15           # 检测窗口大小
    TIMEOUT_MS = 200                 # 单次检测超时(毫秒)
    TOTAL_TIMEOUT = 8.0              # 总超时时间(秒)

    # 初始化变量
    start_time = time.time()
    last_detection_time = None
    frame_count = 0
    
    # 颜色检测相关变量
    color_stats = {
        "1": {"count": 0, "positions": []},
        "2": {"count": 0, "positions": []}, 
        "3": {"count": 0, "positions": []}
    }
    confirmed_color = None
    confirmed_position = None
    
    # 位置稳定性检测
    position_history = []
    position_stable_count = 0

    while True:
        ret, frame = cap2.read()
        if not ret:
            print("无法读取摄像头帧")
            break

        current_time = cv2.getTickCount()
        frame_count += 1
        
        # 图像处理流程
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        filtered_img = cv2.GaussianBlur(gray, (3, 3), 0)
        circles = cv2.HoughCircles(filtered_img,
                                cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                param1=50, param2=30,
                                minRadius=35, maxRadius=38)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 形态学操作
        red_mask = cv2.erode(red_mask, kernel, iterations=1)
        red_mask = cv2.dilate(red_mask, kernel, iterations=2)
        green_mask = cv2.erode(green_mask, kernel, iterations=2)
        green_mask = cv2.dilate(green_mask, kernel, iterations=6)
        blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
        blue_mask = cv2.dilate(blue_mask, kernel, iterations=3)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # 检查是否在有效区域内
                if not (center_x - half_side <= x <= center_x + half_side and
                        center_y - half_side1 <= y <= center_y + half_side2):
                    continue

                x = np.clip(x, 0, frame.shape[1] - 1)
                y = np.clip(y, 0, frame.shape[0] - 1)

                # 创建圆形掩膜
                mask = np.zeros_like(gray)
                cv2.circle(mask, (x, y), int(1.4 * r), 255, -1)

                # 计算各颜色区域面积
                red_area = cv2.countNonZero(cv2.bitwise_and(red_mask, mask))
                green_area = cv2.countNonZero(cv2.bitwise_and(green_mask, mask))
                blue_area = cv2.countNonZero(cv2.bitwise_and(blue_mask, mask))

                max_area = max(red_area, green_area, blue_area)
                if max_area == 0:
                    continue
                
                # 确定当前颜色
                if max_area == red_area:
                    current_color = "1"
                elif max_area == green_area:
                    current_color = "2"
                else:
                    current_color = "3"
                
                # 更新检测时间
                last_detection_time = current_time
                
                # 记录颜色统计信息（只保留最近FRAME_WINDOW_SIZE次）
                color_stats[current_color]["count"] += 1
                color_stats[current_color]["positions"].append((x, y))
                if len(color_stats[current_color]["positions"]) > FRAME_WINDOW_SIZE:
                    color_stats[current_color]["positions"].pop(0)
                
                # 打印调试信息
                masked = cv2.bitwise_and(hsv, hsv, mask=mask)
                avg_hsv = cv2.mean(masked, mask=mask)[:3]
                print(f"帧 {frame_count}: 检测到颜色 {current_color}，坐标 ({x}, {y})，HSV: H={avg_hsv[0]:.1f}, S={avg_hsv[1]:.1f}, V={avg_hsv[2]:.1f}")
                
                # 检查是否有颜色达到稳定条件
                for color in ["1", "2", "3"]:
                    if (color_stats[color]["count"] >= COLOR_STABLE_THRESHOLD and
                        len(color_stats[color]["positions"]) >= 3 and
                        color_stats[color]["count"]/frame_count >= COLOR_CONFIDENCE):
                        
                        # 计算该颜色的平均坐标
                        avg_x = int(np.mean([p[0] for p in color_stats[color]["positions"]]))
                        avg_y = int(np.mean([p[1] for p in color_stats[color]["positions"]]))
                        
                        confirmed_color = color
                        confirmed_position = (avg_x, avg_y)
                        print(f"颜色 {color} 已达到稳定条件，平均坐标 ({avg_x}, {avg_y})")
                        break
                
                # 位置稳定性检测（基于确认颜色的坐标）
                if confirmed_color is not None:
                    current_pos = (x, y) if current_color == confirmed_color else confirmed_position
                    position_history.append(current_pos)
                    
                    # 只保留最近几次位置记录
                    if len(position_history) > 5:
                        position_history.pop(0)
                    
                    # 检查位置是否稳定
                    if len(position_history) >= 3:
                        distances = [
                            np.sqrt((position_history[i][0]-position_history[i-1][0])**2 + 
                                   (position_history[i][1]-position_history[i-1][1])**2)
                            for i in range(1, len(position_history))
                        ]
                        
                        if all(d < POSITION_STABLE_THRESHOLD for d in distances):
                            position_stable_count += 1
                        else:
                            position_stable_count = max(0, position_stable_count - 1)
                
                # 达到稳定阈值后发送数据
                if (confirmed_color is not None and 
                    position_stable_count >= 3):
                    
                    formatted_data = f"{confirmed_position[0]:04}{confirmed_position[1]:04}{confirmed_color}"
                    send_data(formatted_data)
                    print(f"最终结果: 颜色 {confirmed_color} 稳定坐标 {confirmed_position}")
                    return confirmed_color
                
                break  # 只处理第一个检测到的圆形
        else:
            # 检查是否超时
            if last_detection_time is not None:
                elapsed_time = (current_time - last_detection_time) / cv2.getTickFrequency() * 1000
                if elapsed_time > TIMEOUT_MS:
                    print(f"帧 {frame_count}: 检测超时，跳过本次识别")
                    continue
            
        # 总超时检测
        if time.time() - start_time > TOTAL_TIMEOUT:
            # 如果有候选颜色但未完全稳定，仍然尝试输出
            if confirmed_color is not None:
                avg_x = int(np.mean([p[0] for p in color_stats[confirmed_color]["positions"]]))
                avg_y = int(np.mean([p[1] for p in color_stats[confirmed_color]["positions"]]))
                formatted_data = f"{avg_x:04}{avg_y:04}{confirmed_color}"
                send_data(formatted_data)
                print(f"超时结果: 颜色 {confirmed_color} 坐标 ({avg_x}, {avg_y})")
                return confirmed_color
            else:
                # 找出出现次数最多的颜色
                best_color = max(color_stats.items(), key=lambda x: x[1]["count"])[0]
                if color_stats[best_color]["count"] > 0:
                    avg_x = int(np.mean([p[0] for p in color_stats[best_color]["positions"]]))
                    avg_y = int(np.mean([p[1] for p in color_stats[best_color]["positions"]]))
                    formatted_data = f"{avg_x:04}{avg_y:04}{best_color}"
                    send_data(formatted_data)
                    print(f"超时备用结果: 颜色 {best_color} 坐标 ({avg_x}, {avg_y})")
                    return best_color
                else:
                    send_data("000000004")
                    print("检测超时，发送默认信号")
                    return "4"
        
        time.sleep(0.02)

def run_sehuanyuanxin2_centered():  # 初赛放物块的时候，色环第二次定标的程序（地上的圆形）
    global cap2
    # 红色双阈值范围
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    lower_green = np.array([30, 40, 40])
    upper_green = np.array([89, 255, 255])
    lower_blue = np.array([90, 50, 90])
    upper_blue = np.array([140, 255, 255])
    kernel = np.ones((3, 3), np.uint8)

    center_x, center_y = 960, 540
    square_side = 500
    square_side1 = 500
    square_side2 = 500
    half_side = square_side // 2
    half_side1 = square_side1 // 2
    half_side2 = square_side2 // 2

    if not ensure_upper_camera().isOpened():
        print("无法读取摄像头帧")
        return

    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    start_time = time.time()
    last_detection_time = None  # 记录上次成功检测的时间
    timeout_ms = 300  # 超时时间300毫秒
    
    # 稳定性检测相关变量
    stable_count = 0  # 位置稳定计数
    color_stable_count = 0  # 颜色稳定计数
    last_center = None  # 上次检测到的中心点
    first_center = None  # 第一次检测到的中心点
    color_history = []  # 颜色历史记录
    final_color = None  # 最终确定的颜色
    last_radius = 0  # 最后检测到的半径
    
    # 稳定性参数配置
    STABLE_THRESHOLD = 3  # 位置稳定阈值(帧数)
    COLOR_STABLE_THRESHOLD = 3  # 颜色稳定阈值(帧数)
    COLOR_CONFIDENCE = 0.7  # 颜色置信度阈值
    MAX_PIXEL_MOVE = 10  # 最大允许移动像素(因分辨率更高)

    while True:
        ret, frame = cap2.read()
        if not ret:
            print("无法读取摄像头帧")
            break

        current_time = cv2.getTickCount()  # 获取当前时间戳
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        filtered_img = cv2.GaussianBlur(gray, (3, 3), 0)
        circles = cv2.HoughCircles(filtered_img,
                                cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                param1=50, param2=30,
                                minRadius=160, maxRadius=165)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        red_mask = cv2.erode(red_mask, kernel, iterations=1)
        red_mask = cv2.dilate(red_mask, kernel, iterations=2)
        green_mask = cv2.erode(green_mask, kernel, iterations=2)
        green_mask = cv2.dilate(green_mask, kernel, iterations=6)
        blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
        blue_mask = cv2.dilate(blue_mask, kernel, iterations=3)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                if not (center_x - half_side <= x <= center_x + half_side and
                        center_y - half_side1 <= y <= center_y + half_side2):
                    continue

                x = np.clip(x, 0, frame.shape[1] - 1)
                y = np.clip(y, 0, frame.shape[0] - 1)

                mask = np.zeros_like(gray)
                cv2.circle(mask, (x, y), int(1.4 * r), 255, -1)

                red_area = cv2.countNonZero(cv2.bitwise_and(red_mask, mask))
                green_area = cv2.countNonZero(cv2.bitwise_and(green_mask, mask))
                blue_area = cv2.countNonZero(cv2.bitwise_and(blue_mask, mask))

                max_area = max(red_area, green_area, blue_area)
                if max_area == 0:
                    print("检测到无效颜色区域（HSV全为0），跳过...")
                    continue
                
                # 更新检测时间
                last_detection_time = current_time
                current_center = (x, y)
                last_radius = r  # 记录半径
                
                # 确定当前颜色
                if max_area == red_area:
                    current_color = "1"
                    masked = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_and(red_mask, mask))
                elif max_area == green_area:
                    current_color = "2"
                    masked = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_and(green_mask, mask))
                else:
                    current_color = "3"
                    masked = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_and(blue_mask, mask))
                
                avg_hsv = cv2.mean(masked, mask=cv2.bitwise_and(red_mask if current_color == "1" else (green_mask if current_color == "2" else blue_mask), mask))[:3]
                
                if avg_hsv[0] == 0 and avg_hsv[1] == 0 and avg_hsv[2] == 0:
                    print(f"检测到无效{current_color}区域（HSV全为0），跳过...")
                    continue
                
                print(f"检测到颜色 {current_color}，HSV值: H={avg_hsv[0]:.1f}, S={avg_hsv[1]:.1f}, V={avg_hsv[2]:.1f}")
                
                # 记录颜色历史
                color_history.append(current_color)
                
                # 位置稳定性检测
                if first_center is None:
                    first_center = current_center
                    stable_count = 1
                else:
                    distance = np.sqrt((current_center[0] - first_center[0])**2 + 
                                     (current_center[1] - first_center[1])**2)
                    if distance < MAX_PIXEL_MOVE:
                        stable_count += 1
                    else:
                        stable_count = 1
                        first_center = current_center
                        color_history = []  # 位置变化大时清空颜色历史
                
                # 颜色稳定性检测
                if len(color_history) >= COLOR_STABLE_THRESHOLD:
                    # 统计颜色出现频率
                    from collections import defaultdict
                    color_counter = defaultdict(int)
                    for color in color_history:
                        color_counter[color] += 1
                    
                    # 找出出现次数最多的颜色
                    most_common_color = max(color_counter.items(), key=lambda x: x[1])[0]
                    confidence = color_counter[most_common_color] / len(color_history)
                    
                    # 如果置信度达标，确定最终颜色
                    if confidence >= COLOR_CONFIDENCE:
                        final_color = most_common_color
                        print(f"颜色识别稳定: {final_color} (置信度: {confidence:.2f})")
                
                # 达到稳定阈值后发送数据
                if (stable_count >= STABLE_THRESHOLD and 
                    final_color is not None):
                    formatted_data = f"{current_center[0]:04}{current_center[1]:04}{final_color}"
                    send_data(formatted_data)
                    print(f"稳定坐标发送: {formatted_data}")
                    return
        else:
            # 检查是否超时（200ms内没有检测到）
            if last_detection_time is not None:
                elapsed_time = (current_time - last_detection_time) / cv2.getTickFrequency() * 1000  # 毫秒
                if elapsed_time > timeout_ms:
                    print("检测超时，跳过本次识别")
                    continue  # 跳过本次识别但不重置stable_count
            
        # 总超时检测（8秒）
        if time.time() - start_time > 8.0:
            send_data("000000004")
            print("检测超时，发送默认信号")
            break
            
        time.sleep(0.05)

def run_maduoyuanxin2():  # 初赛放物块的时候，色环第一次定标的程序（地上的圆形）
    global cap2
    # 红色双阈值范围
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    lower_green = np.array([30, 40, 40])
    upper_green = np.array([89, 255, 255])
    lower_blue = np.array([90, 50, 90])
    upper_blue = np.array([140, 255, 255])
    kernel = np.ones((3, 3), np.uint8)

    # 检测区域参数
    center_x, center_y = 320, 240
    square_side = 600
    square_side1 = 200
    square_side2 = 300
    half_side = square_side // 2
    half_side1 = square_side1 // 2
    half_side2 = square_side2 // 2

    # 初始化摄像头
    if not cap2.isOpened():
        print("无法读取摄像头帧")
        return
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # 稳定性检测参数（调整为更宽松的条件）
    POSITION_STABLE_THRESHOLD = 10   # 位置稳定阈值(像素)
    COLOR_STABLE_THRESHOLD = 5       # 颜色稳定所需出现次数（非连续）
    COLOR_CONFIDENCE = 0.5           # 颜色置信度要求
    FRAME_WINDOW_SIZE = 15           # 检测窗口大小
    TIMEOUT_MS = 200                 # 单次检测超时(毫秒)
    TOTAL_TIMEOUT = 8.0              # 总超时时间(秒)

    # 初始化变量
    start_time = time.time()
    last_detection_time = None
    frame_count = 0
    
    # 颜色检测相关变量
    color_stats = {
        "1": {"count": 0, "positions": []},
        "2": {"count": 0, "positions": []}, 
        "3": {"count": 0, "positions": []}
    }
    confirmed_color = None
    confirmed_position = None
    
    # 位置稳定性检测
    position_history = []
    position_stable_count = 0

    while True:
        ret, frame = cap2.read()
        if not ret:
            print("无法读取摄像头帧")
            break

        current_time = cv2.getTickCount()
        frame_count += 1
        
        # 图像处理流程
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        filtered_img = cv2.GaussianBlur(gray, (3, 3), 0)
        circles = cv2.HoughCircles(filtered_img,
                                cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                param1=50, param2=30,
                                minRadius=20, maxRadius=27)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 形态学操作
        red_mask = cv2.erode(red_mask, kernel, iterations=1)
        red_mask = cv2.dilate(red_mask, kernel, iterations=2)
        green_mask = cv2.erode(green_mask, kernel, iterations=2)
        green_mask = cv2.dilate(green_mask, kernel, iterations=6)
        blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
        blue_mask = cv2.dilate(blue_mask, kernel, iterations=3)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # 检查是否在有效区域内
                if not (center_x - half_side <= x <= center_x + half_side and
                        center_y - half_side1 <= y <= center_y + half_side2):
                    continue

                x = np.clip(x, 0, frame.shape[1] - 1)
                y = np.clip(y, 0, frame.shape[0] - 1)

                # 创建圆形掩膜
                mask = np.zeros_like(gray)
                cv2.circle(mask, (x, y), int(1.4 * r), 255, -1)

                # 计算各颜色区域面积
                red_area = cv2.countNonZero(cv2.bitwise_and(red_mask, mask))
                green_area = cv2.countNonZero(cv2.bitwise_and(green_mask, mask))
                blue_area = cv2.countNonZero(cv2.bitwise_and(blue_mask, mask))

                max_area = max(red_area, green_area, blue_area)
                if max_area == 0:
                    continue
                
                # 确定当前颜色
                if max_area == red_area:
                    current_color = "1"
                elif max_area == green_area:
                    current_color = "2"
                else:
                    current_color = "3"
                
                # 更新检测时间
                last_detection_time = current_time
                
                # 记录颜色统计信息（只保留最近FRAME_WINDOW_SIZE次）
                color_stats[current_color]["count"] += 1
                color_stats[current_color]["positions"].append((x, y))
                if len(color_stats[current_color]["positions"]) > FRAME_WINDOW_SIZE:
                    color_stats[current_color]["positions"].pop(0)
                
                # 打印调试信息
                masked = cv2.bitwise_and(hsv, hsv, mask=mask)
                avg_hsv = cv2.mean(masked, mask=mask)[:3]
                print(f"帧 {frame_count}: 检测到颜色 {current_color}，坐标 ({x}, {y})，HSV: H={avg_hsv[0]:.1f}, S={avg_hsv[1]:.1f}, V={avg_hsv[2]:.1f}")
                
                # 检查是否有颜色达到稳定条件
                for color in ["1", "2", "3"]:
                    if (color_stats[color]["count"] >= COLOR_STABLE_THRESHOLD and
                        len(color_stats[color]["positions"]) >= 3 and
                        color_stats[color]["count"]/frame_count >= COLOR_CONFIDENCE):
                        
                        # 计算该颜色的平均坐标
                        avg_x = int(np.mean([p[0] for p in color_stats[color]["positions"]]))
                        avg_y = int(np.mean([p[1] for p in color_stats[color]["positions"]]))
                        
                        confirmed_color = color
                        confirmed_position = (avg_x, avg_y)
                        print(f"颜色 {color} 已达到稳定条件，平均坐标 ({avg_x}, {avg_y})")
                        break
                
                # 位置稳定性检测（基于确认颜色的坐标）
                if confirmed_color is not None:
                    current_pos = (x, y) if current_color == confirmed_color else confirmed_position
                    position_history.append(current_pos)
                    
                    # 只保留最近几次位置记录
                    if len(position_history) > 5:
                        position_history.pop(0)
                    
                    # 检查位置是否稳定
                    if len(position_history) >= 3:
                        distances = [
                            np.sqrt((position_history[i][0]-position_history[i-1][0])**2 + 
                                   (position_history[i][1]-position_history[i-1][1])**2)
                            for i in range(1, len(position_history))
                        ]
                        
                        if all(d < POSITION_STABLE_THRESHOLD for d in distances):
                            position_stable_count += 1
                        else:
                            position_stable_count = max(0, position_stable_count - 1)
                
                # 达到稳定阈值后发送数据
                if (confirmed_color is not None and 
                    position_stable_count >= 4):
                    
                    formatted_data = f"{confirmed_position[0]:04}{confirmed_position[1]:04}{confirmed_color}"
                    send_data(formatted_data)
                    print(f"最终结果: 颜色 {confirmed_color} 稳定坐标 {confirmed_position}")
                    return confirmed_color
                
                break  # 只处理第一个检测到的圆形
        else:
            # 检查是否超时
            if last_detection_time is not None:
                elapsed_time = (current_time - last_detection_time) / cv2.getTickFrequency() * 1000
                if elapsed_time > TIMEOUT_MS:
                    print(f"帧 {frame_count}: 检测超时，跳过本次识别")
                    continue
            
        # 总超时检测
        if time.time() - start_time > TOTAL_TIMEOUT:
            # 如果有候选颜色但未完全稳定，仍然尝试输出
            if confirmed_color is not None:
                avg_x = int(np.mean([p[0] for p in color_stats[confirmed_color]["positions"]]))
                avg_y = int(np.mean([p[1] for p in color_stats[confirmed_color]["positions"]]))
                formatted_data = f"{avg_x:04}{avg_y:04}{confirmed_color}"
                send_data(formatted_data)
                print(f"超时结果: 颜色 {confirmed_color} 坐标 ({avg_x}, {avg_y})")
                return confirmed_color
            else:
                # 找出出现次数最多的颜色
                best_color = max(color_stats.items(), key=lambda x: x[1]["count"])[0]
                if color_stats[best_color]["count"] > 0:
                    avg_x = int(np.mean([p[0] for p in color_stats[best_color]["positions"]]))
                    avg_y = int(np.mean([p[1] for p in color_stats[best_color]["positions"]]))
                    formatted_data = f"{avg_x:04}{avg_y:04}{best_color}"
                    send_data(formatted_data)
                    print(f"超时备用结果: 颜色 {best_color} 坐标 ({avg_x}, {avg_y})")
                    return best_color
                else:
                    send_data("000000004")
                    print("检测超时，发送默认信号")
                    return "4"
        
        time.sleep(0.02)

def run_maduoyuanxin2_centered():  # 初赛放物块的时候，色环第二次定标的程序（地上的圆形）
    global cap2
    # 红色双阈值范围
    lower_red1 = np.array([0, 50, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 50, 50])
    upper_red2 = np.array([180, 255, 255])
    lower_green = np.array([30, 40, 40])
    upper_green = np.array([89, 255, 255])
    lower_blue = np.array([90, 50, 90])
    upper_blue = np.array([140, 255, 255])
    kernel = np.ones((3, 3), np.uint8)

    center_x, center_y = 320, 240
    square_side = 150
    square_side1 = 150
    square_side2 = 150
    half_side = square_side // 2
    half_side1 = square_side1 // 2
    half_side2 = square_side2 // 2

    if not cap2.isOpened():
        print("无法读取摄像头帧")
        return

    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    start_time = time.time()
    last_detection_time = None  # 记录上次成功检测的时间
    timeout_ms = 200  # 超时时间300毫秒
    
    # 稳定性检测相关变量
    stable_count = 0  # 位置稳定计数
    color_stable_count = 0  # 颜色稳定计数
    last_center = None  # 上次检测到的中心点
    first_center = None  # 第一次检测到的中心点
    color_history = []  # 颜色历史记录
    final_color = None  # 最终确定的颜色
    last_radius = 0  # 最后检测到的半径
    
    # 稳定性参数配置
    STABLE_THRESHOLD = 5  # 位置稳定阈值(帧数)
    COLOR_STABLE_THRESHOLD = 5  # 颜色稳定阈值(帧数)
    COLOR_CONFIDENCE = 0.7  # 颜色置信度阈值
    MAX_PIXEL_MOVE = 10  # 最大允许移动像素(因分辨率更高)

    while True:
        ret, frame = cap2.read()
        if not ret:
            print("无法读取摄像头帧")
            break

        current_time = cv2.getTickCount()  # 获取当前时间戳
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        filtered_img = cv2.GaussianBlur(gray, (3, 3), 0)
        circles = cv2.HoughCircles(filtered_img,
                                cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                                param1=50, param2=30,
                                minRadius=20, maxRadius=27)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        red_mask = cv2.erode(red_mask, kernel, iterations=1)
        red_mask = cv2.dilate(red_mask, kernel, iterations=2)
        green_mask = cv2.erode(green_mask, kernel, iterations=2)
        green_mask = cv2.dilate(green_mask, kernel, iterations=6)
        blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
        blue_mask = cv2.dilate(blue_mask, kernel, iterations=3)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                if not (center_x - half_side <= x <= center_x + half_side and
                        center_y - half_side1 <= y <= center_y + half_side2):
                    continue

                x = np.clip(x, 0, frame.shape[1] - 1)
                y = np.clip(y, 0, frame.shape[0] - 1)

                mask = np.zeros_like(gray)
                cv2.circle(mask, (x, y), int(1.4 * r), 255, -1)

                red_area = cv2.countNonZero(cv2.bitwise_and(red_mask, mask))
                green_area = cv2.countNonZero(cv2.bitwise_and(green_mask, mask))
                blue_area = cv2.countNonZero(cv2.bitwise_and(blue_mask, mask))

                max_area = max(red_area, green_area, blue_area)
                if max_area == 0:
                    print("检测到无效颜色区域（HSV全为0），跳过...")
                    continue
                
                # 更新检测时间
                last_detection_time = current_time
                current_center = (x, y)
                last_radius = r  # 记录半径
                
                # 确定当前颜色
                if max_area == red_area:
                    current_color = "1"
                    masked = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_and(red_mask, mask))
                elif max_area == green_area:
                    current_color = "2"
                    masked = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_and(green_mask, mask))
                else:
                    current_color = "3"
                    masked = cv2.bitwise_and(hsv, hsv, mask=cv2.bitwise_and(blue_mask, mask))
                
                avg_hsv = cv2.mean(masked, mask=cv2.bitwise_and(red_mask if current_color == "1" else (green_mask if current_color == "2" else blue_mask), mask))[:3]
                
                if avg_hsv[0] == 0 and avg_hsv[1] == 0 and avg_hsv[2] == 0:
                    print(f"检测到无效{current_color}区域（HSV全为0），跳过...")
                    continue
                
                print(f"检测到颜色 {current_color}，HSV值: H={avg_hsv[0]:.1f}, S={avg_hsv[1]:.1f}, V={avg_hsv[2]:.1f}")
                
                # 记录颜色历史
                color_history.append(current_color)
                
                # 位置稳定性检测
                if first_center is None:
                    first_center = current_center
                    stable_count = 1
                else:
                    distance = np.sqrt((current_center[0] - first_center[0])**2 + 
                                     (current_center[1] - first_center[1])**2)
                    if distance < MAX_PIXEL_MOVE:
                        stable_count += 1
                    else:
                        stable_count = 1
                        first_center = current_center
                        color_history = []  # 位置变化大时清空颜色历史
                
                # 颜色稳定性检测
                if len(color_history) >= COLOR_STABLE_THRESHOLD:
                    # 统计颜色出现频率
                    from collections import defaultdict
                    color_counter = defaultdict(int)
                    for color in color_history:
                        color_counter[color] += 1
                    
                    # 找出出现次数最多的颜色
                    most_common_color = max(color_counter.items(), key=lambda x: x[1])[0]
                    confidence = color_counter[most_common_color] / len(color_history)
                    
                    # 如果置信度达标，确定最终颜色
                    if confidence >= COLOR_CONFIDENCE:
                        final_color = most_common_color
                        print(f"颜色识别稳定: {final_color} (置信度: {confidence:.2f})")
                
                # 达到稳定阈值后发送数据
                if (stable_count >= STABLE_THRESHOLD and 
                    final_color is not None):
                    formatted_data = f"{current_center[0]:04}{current_center[1]:04}{final_color}"
                    send_data(formatted_data)
                    print(f"稳定坐标发送: {formatted_data}")
                    return
        else:
            # 检查是否超时（200ms内没有检测到）
            if last_detection_time is not None:
                elapsed_time = (current_time - last_detection_time) / cv2.getTickFrequency() * 1000  # 毫秒
                if elapsed_time > timeout_ms:
                    print("检测超时，跳过本次识别")
                    continue  # 跳过本次识别但不重置stable_count
            
        # 总超时检测（8秒）
        if time.time() - start_time > 8.0:
            send_data("000000004")
            print("检测超时，发送默认信号")
            break
            
        time.sleep(0.05)

class HostSerialCommandHandler:
    def __init__(self):
        self.running = True
        self.last_data_time = time.time()

    def handle_serial_data(self, data):
        """处理下位机指令并触发对应视觉任务。"""
        global cap, cap2, C_1, C_2, C_1_loop2, C_2_loop2
        global INDEX, INDEX1, INDEX2, INDEX_loop2, INDEX1_loop2, INDEX2_loop2

        command = data.strip()
        print(f"接收到: {command}")
        self.last_data_time = time.time()
        if command == "end":
            print("任务序列完成。")
            self.running = False
            return
        if command == "r":
            restart_program()
            return

        if command == "1":
            print("检测到任务1: 执行二维码检测")
            run_erweima()
            cap = open_camera(CAMERA_UPPER_DEVICE)
        elif command == "2":
            print("检测到任务2: 执行物块检测")
            C_1 = run_wukuaiyuanxin_1()
        elif command == "a":
            print("检测到任务a: 执行第二圈物块初始检测")
            cap = open_camera(CAMERA_UPPER_DEVICE)
            C_1_loop2 = run_wukuaiyuanxin_1()
        elif command == "3":
            print("检测到任务3: 执行第一圈转盘物块检测")
            LOOP1()
            if cap and cap.isOpened():
                cap.release()
        elif command == "4":
            print("检测到任务4: 执行第二圈转盘物块检测")
            LOOP2()
            if cap and cap.isOpened():
                cap.release()
        elif command == "5":
            print("first loop 已抓完第一个物块，识别第二个物块是否稳定")
            if INDEX1 == 1:
                run_wukuaiyuanxin_xuanzequyu(INDEX1, A[1])
            else:
                run_wukuaiyuanxin_xuanzequyu(0, A[1])
        elif command == "6":
            print("first loop 已抓完第二个物块，识别第三个物块是否稳定")
            if INDEX2 == 1:
                run_wukuaiyuanxin_xuanzequyu(INDEX2, A[2])
            else:
                run_wukuaiyuanxin_xuanzequyu(0, A[2])
        elif command == "b":
            print("second loop 已抓完第一个物块，识别第二个物块是否稳定")
            if INDEX1_loop2 == 1:
                run_wukuaiyuanxin_xuanzequyu(INDEX1_loop2, A[5])
            else:
                run_wukuaiyuanxin_xuanzequyu(0, A[5])
        elif command == "c":
            print("second loop 已抓完第二个物块，识别第三个物块是否稳定")
            if INDEX2_loop2 == 1:
                run_wukuaiyuanxin_xuanzequyu(INDEX2_loop2, A[6])
            else:
                run_wukuaiyuanxin_xuanzequyu(0, A[6])
        elif command == "7":
            cap2 = open_camera(CAMERA_UPPER_DEVICE)
            print("进行色环检测的第一次定标")
            run_sehuanyuanxin2()
        elif command == "d":
            print("进行色环检测的直线纠偏")
            send_data("000000000")
        elif command == "8":
            print("进行色环检测的精细定标")
            run_sehuanyuanxin2_centered()
            if cap2 and cap2.isOpened():
                cap2.release()
        elif command == "9":
            cap2 = open_camera(CAMERA_UPPER_DEVICE)
            print("进行码垛检测的第一次定标")
            run_maduoyuanxin2()
            if cap2 and cap2.isOpened():
                cap2.release()
        elif command == "0":
            cap2 = open_camera(CAMERA_UPPER_DEVICE)
            print("进行码垛检测的精细定标")
            run_maduoyuanxin2_centered()
            if cap2 and cap2.isOpened():
                cap2.release()
        else:
            print(f"未识别指令: {command}")

    def run(self):
        port = open_host_serial()
        print("gcs-host started: host-lower serial link ready.")
        print(f"上位机通信口: {port.port} @ {HOST_SERIAL_BAUD}")
        print(f"二维码摄像头: {CAMERA_QR_DEVICE}")
        print(f"上方摄像头: {CAMERA_UPPER_DEVICE}")
        send_data("987654321")
        print("已发送启动握手 987654321")

        try:
            while self.running:
                data = receive_data()
                if data:
                    self.handle_serial_data(data)
                else:
                    time.sleep(0.01)

                if time.time() - self.last_data_time > 300:
                    print("警告: 300秒未收到任何指令，检查通信连接...")
                    self.last_data_time = time.time()
        except KeyboardInterrupt:
            print("程序被用户终止")
        finally:
            close_host_serial()
            if cap and cap.isOpened():
                cap.release()
            if cap2 and cap2.isOpened():
                cap2.release()
            cv2.destroyAllWindows()

def run_qr_preview():
    """可视化调试二维码识别，不打开通信口。"""
    cap0 = open_camera(CAMERA_QR_DEVICE)
    if not cap0.isOpened():
        raise SystemExit(f"无法打开二维码摄像头: {CAMERA_QR_DEVICE}")

    cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap0.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    detector = cv2.QRCodeDetector()
    last_data = None
    stable_count = 0
    confirmed_data = None
    confirmed_printed = False
    missed_frames = 0

    window_name = "QR Preview"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)
    startup_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(startup_frame, "OPENING CAMERA...", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imshow(window_name, startup_frame)
    cv2.waitKey(300)

    print("二维码预览已启动，按 q 或 Esc 退出。")
    print(f"二维码摄像头: {CAMERA_QR_DEVICE}")

    try:
        while True:
            if window_closed(window_name):
                break

            ok, frame = cap0.read()
            if not ok or frame is None:
                missed_frames += 1
                wait_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(wait_frame, f"WAITING CAMERA FRAME {missed_frames}", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow(window_name, wait_frame)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), 27) or window_closed(window_name):
                    break
                if missed_frames >= 100:
                    print("连续无法接收帧，退出...")
                    break
                continue
            missed_frames = 0

            data, points, _ = detector.detectAndDecode(frame)
            current_data = data.strip() if data else ""

            if points is not None:
                pts = points.astype(int).reshape(-1, 2)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

            if current_data:
                if current_data == last_data:
                    stable_count += 1
                else:
                    stable_count = 1
                    last_data = current_data

                if len(current_data) == 7 and stable_count >= 3:
                    confirmed_data = current_data
                    if not confirmed_printed:
                        print(f"二维码确认: {confirmed_data}")
                        confirmed_printed = True

            status = "WAITING"
            color = (0, 255, 255)
            if current_data:
                status = f"DETECTED {current_data} stable={stable_count}"
                color = (0, 255, 0)
            if confirmed_data:
                status = f"CONFIRMED {confirmed_data}"
                color = (0, 200, 0)

            cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27) or window_closed(window_name):
                break
    finally:
        cap0.release()
        cv2.destroyAllWindows()

RING_V3_TUNE_WINDOW = "Ring Tune V3"
RING_V3_DEFAULTS = {
    "threshold_value": 70,
    "enhance_alpha_x10": 20,
    "erode_iterations": 1,
    "dilate_kernel": 3,
    "dilate_iterations": 1,
    "gradient_kernel": 3,
    "blur1_kernel": 3,
    "blur3_kernel": 3,
    "hough_param1": 100,
    "hough_param2_x100": 75,
    "hough_min_dist": 80,
    "hough_min_radius": 25,
    "hough_max_radius": 70,
    "min_center_gap": 30,
}

def odd_kernel(value):
    value = max(1, int(value))
    return value if value % 2 == 1 else value + 1

def create_ring_v3_trackbars():
    cv2.namedWindow(RING_V3_TUNE_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(RING_V3_TUNE_WINDOW, 520, 760)
    d = RING_V3_DEFAULTS
    cv2.createTrackbar("threshold", RING_V3_TUNE_WINDOW, d["threshold_value"], 255, lambda _: None)
    cv2.createTrackbar("enhance_x10", RING_V3_TUNE_WINDOW, d["enhance_alpha_x10"], 100, lambda _: None)
    cv2.createTrackbar("erode_iter", RING_V3_TUNE_WINDOW, d["erode_iterations"], 6, lambda _: None)
    cv2.createTrackbar("dilate_kernel", RING_V3_TUNE_WINDOW, d["dilate_kernel"], 31, lambda _: None)
    cv2.createTrackbar("dilate_iter", RING_V3_TUNE_WINDOW, d["dilate_iterations"], 6, lambda _: None)
    cv2.createTrackbar("grad_kernel", RING_V3_TUNE_WINDOW, d["gradient_kernel"], 31, lambda _: None)
    cv2.createTrackbar("blur1_kernel", RING_V3_TUNE_WINDOW, d["blur1_kernel"], 31, lambda _: None)
    cv2.createTrackbar("blur3_kernel", RING_V3_TUNE_WINDOW, d["blur3_kernel"], 31, lambda _: None)
    cv2.createTrackbar("hough_p1", RING_V3_TUNE_WINDOW, d["hough_param1"], 255, lambda _: None)
    cv2.createTrackbar("hough_p2_x100", RING_V3_TUNE_WINDOW, d["hough_param2_x100"], 100, lambda _: None)
    cv2.createTrackbar("min_dist", RING_V3_TUNE_WINDOW, d["hough_min_dist"], 200, lambda _: None)
    cv2.createTrackbar("min_r", RING_V3_TUNE_WINDOW, d["hough_min_radius"], 120, lambda _: None)
    cv2.createTrackbar("max_r", RING_V3_TUNE_WINDOW, d["hough_max_radius"], 160, lambda _: None)
    cv2.createTrackbar("gap", RING_V3_TUNE_WINDOW, d["min_center_gap"], 80, lambda _: None)

def read_ring_v3_trackbars():
    min_r = max(1, cv2.getTrackbarPos("min_r", RING_V3_TUNE_WINDOW))
    max_r = max(min_r + 1, cv2.getTrackbarPos("max_r", RING_V3_TUNE_WINDOW))
    return {
        "threshold_value": cv2.getTrackbarPos("threshold", RING_V3_TUNE_WINDOW),
        "enhance_alpha": max(1, cv2.getTrackbarPos("enhance_x10", RING_V3_TUNE_WINDOW)) / 10.0,
        "erode_iterations": cv2.getTrackbarPos("erode_iter", RING_V3_TUNE_WINDOW),
        "dilate_kernel": max(1, cv2.getTrackbarPos("dilate_kernel", RING_V3_TUNE_WINDOW)),
        "dilate_iterations": cv2.getTrackbarPos("dilate_iter", RING_V3_TUNE_WINDOW),
        "gradient_kernel": max(1, cv2.getTrackbarPos("grad_kernel", RING_V3_TUNE_WINDOW)),
        "blur1_kernel": odd_kernel(cv2.getTrackbarPos("blur1_kernel", RING_V3_TUNE_WINDOW)),
        "blur3_kernel": odd_kernel(cv2.getTrackbarPos("blur3_kernel", RING_V3_TUNE_WINDOW)),
        "hough_param1": max(1, cv2.getTrackbarPos("hough_p1", RING_V3_TUNE_WINDOW)),
        "hough_param2": max(1, cv2.getTrackbarPos("hough_p2_x100", RING_V3_TUNE_WINDOW)) / 100.0,
        "hough_min_dist": max(1, cv2.getTrackbarPos("min_dist", RING_V3_TUNE_WINDOW)),
        "hough_min_radius": min_r,
        "hough_max_radius": max_r,
        "min_center_gap": max(1, cv2.getTrackbarPos("gap", RING_V3_TUNE_WINDOW)),
    }

def classify_ring_color(frame, gray, hsv, circle, red_mask, green_mask, blue_mask):
    x, y, r = circle
    mask = np.zeros_like(gray)
    cv2.circle(mask, (x, y), int(1.25 * r), 255, -1)
    cv2.circle(mask, (x, y), max(1, int(0.55 * r)), 0, -1)
    areas = {
        "R": cv2.countNonZero(cv2.bitwise_and(red_mask, mask)),
        "G": cv2.countNonZero(cv2.bitwise_and(green_mask, mask)),
        "B": cv2.countNonZero(cv2.bitwise_and(blue_mask, mask)),
    }
    color_name = max(areas.items(), key=lambda item: item[1])[0]
    if areas[color_name] <= 0:
        return "?", areas
    return color_name, areas

def draw_text_panel(image, lines, x, y, color=(0, 255, 255), font_scale=0.48):
    if not lines:
        return
    font = cv2.FONT_HERSHEY_SIMPLEX
    thickness = 1
    line_height = 20
    widths = [cv2.getTextSize(line, font, font_scale, thickness)[0][0] for line in lines]
    panel_width = min(max(widths) + 16, image.shape[1] - x - 4)
    panel_height = line_height * len(lines) + 10
    cv2.rectangle(image, (x, y), (x + panel_width, y + panel_height), (0, 0, 0), -1)
    for i, line in enumerate(lines):
        cv2.putText(image, line, (x + 8, y + 18 + i * line_height), font, font_scale, color, thickness)

def detect_ring_v3(frame, tune, roi_rect):
    frame_h, frame_w = frame.shape[:2]
    roi_left, roi_top, roi_right, roi_bottom = roi_rect
    roi_left = max(0, min(frame_w - 1, roi_left))
    roi_top = max(0, min(frame_h - 1, roi_top))
    roi_right = max(roi_left + 1, min(frame_w, roi_right))
    roi_bottom = max(roi_top + 1, min(frame_h, roi_bottom))
    roi_frame = frame[roi_top:roi_bottom, roi_left:roi_right]

    gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    work = gray
    if tune["erode_iterations"] > 0:
        work = cv2.erode(work, np.ones((3, 3), np.uint8), iterations=tune["erode_iterations"])
    if tune["dilate_iterations"] > 0:
        kernel = np.ones((tune["dilate_kernel"], tune["dilate_kernel"]), np.uint8)
        work = cv2.dilate(work, kernel, iterations=tune["dilate_iterations"])

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    work = clahe.apply(work)
    grad_kernel = np.ones((tune["gradient_kernel"], tune["gradient_kernel"]), np.uint8)
    gradient = cv2.morphologyEx(work, cv2.MORPH_GRADIENT, grad_kernel)
    blur1 = cv2.GaussianBlur(gradient, (tune["blur1_kernel"], tune["blur1_kernel"]), 0)
    enhanced = cv2.convertScaleAbs(blur1, alpha=tune["enhance_alpha"], beta=0)
    _, thresh = cv2.threshold(enhanced, tune["threshold_value"], 255, cv2.THRESH_BINARY)
    hough_src = cv2.GaussianBlur(thresh, (tune["blur3_kernel"], tune["blur3_kernel"]), 0)

    method = getattr(cv2, "HOUGH_GRADIENT_ALT", cv2.HOUGH_GRADIENT)
    param2 = tune["hough_param2"] if method == getattr(cv2, "HOUGH_GRADIENT_ALT", None) else max(1, tune["hough_param2"] * 100)
    circles = cv2.HoughCircles(hough_src,
                               method,
                               dp=1.5,
                               minDist=tune["hough_min_dist"],
                               param1=tune["hough_param1"],
                               param2=param2,
                               minRadius=tune["hough_min_radius"],
                               maxRadius=tune["hough_max_radius"])

    raw_circles = [] if circles is None else np.round(circles[0, :]).astype("int").tolist()
    selected = []
    for x, y, r in raw_circles:
        if any(np.hypot(x - sx, y - sy) < tune["min_center_gap"] for sx, sy, _ in selected):
            continue
        selected.append((x, y, r))

    hsv = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255])),
        cv2.inRange(hsv, np.array([160, 50, 50]), np.array([180, 255, 255])),
    )
    green_mask = cv2.inRange(hsv, np.array([30, 40, 40]), np.array([89, 255, 255]))
    blue_mask = cv2.inRange(hsv, np.array([90, 50, 90]), np.array([140, 255, 255]))

    results = []
    for circle in selected:
        color_name, areas = classify_ring_color(roi_frame, gray, hsv, circle, red_mask, green_mask, blue_mask)
        x, y, r = circle
        results.append({"circle": (x + roi_left, y + roi_top, r), "color": color_name, "areas": areas})

    debug = {
        "gradient": gradient,
        "threshold": thresh,
        "hough_src": hough_src,
        "raw": len(raw_circles),
        "selected": len(selected),
        "nonzero": cv2.countNonZero(thresh),
    }
    return results, debug

def detect_ring_legacy_preview(frame, roi_rect):
    roi_left, roi_top, roi_right, roi_bottom = roi_rect
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    filtered_img = cv2.GaussianBlur(gray, (3, 3), 0)
    circles = cv2.HoughCircles(filtered_img,
                               cv2.HOUGH_GRADIENT, dp=1, minDist=30,
                               param1=50, param2=30,
                               minRadius=35, maxRadius=38)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 50, 50]), np.array([10, 255, 255])),
        cv2.inRange(hsv, np.array([160, 50, 50]), np.array([180, 255, 255])),
    )
    green_mask = cv2.inRange(hsv, np.array([30, 40, 40]), np.array([89, 255, 255]))
    blue_mask = cv2.inRange(hsv, np.array([90, 50, 90]), np.array([140, 255, 255]))
    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.erode(red_mask, kernel, iterations=1)
    red_mask = cv2.dilate(red_mask, kernel, iterations=2)
    green_mask = cv2.erode(green_mask, kernel, iterations=2)
    green_mask = cv2.dilate(green_mask, kernel, iterations=6)
    blue_mask = cv2.erode(blue_mask, kernel, iterations=1)
    blue_mask = cv2.dilate(blue_mask, kernel, iterations=3)

    raw_circles = [] if circles is None else np.round(circles[0, :]).astype("int").tolist()
    results = []
    for x, y, r in raw_circles:
        if not (roi_left <= x <= roi_right and roi_top <= y <= roi_bottom):
            continue
        x = np.clip(x, 0, frame.shape[1] - 1)
        y = np.clip(y, 0, frame.shape[0] - 1)
        mask = np.zeros_like(gray)
        cv2.circle(mask, (x, y), int(1.4 * r), 255, -1)
        red_area = cv2.countNonZero(cv2.bitwise_and(red_mask, mask))
        green_area = cv2.countNonZero(cv2.bitwise_and(green_mask, mask))
        blue_area = cv2.countNonZero(cv2.bitwise_and(blue_mask, mask))
        areas = {"R": red_area, "G": green_area, "B": blue_area}
        max_area = max(red_area, green_area, blue_area)
        if max_area == 0:
            continue
        if max_area == red_area:
            color_name = "R"
        elif max_area == green_area:
            color_name = "G"
        else:
            color_name = "B"
        results.append({"circle": (x, y, r), "color": color_name, "areas": areas})

    debug = {
        "raw": len(raw_circles),
        "selected": len(results),
        "nonzero": 0,
    }
    return results, debug

def run_ring_preview():
    """可视化调试色环识别，不打开通信口。"""
    global cap2

    center_x, center_y = 320, 240
    square_side = 433
    square_side1 = 222
    square_side2 = 240
    half_side = square_side // 2
    half_side1 = square_side1 // 2
    half_side2 = square_side2 // 2
    roi_left = center_x - half_side
    roi_top = center_y - half_side1
    roi_right = center_x + half_side
    roi_bottom = center_y + half_side2

    cap0 = ensure_upper_camera()
    if not cap0.isOpened():
        raise SystemExit(f"无法打开上方摄像头: {CAMERA_UPPER_DEVICE}")

    cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap0.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    apply_optional_camera_controls(cap0)

    window_name = "Ring Preview"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)
    startup_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(startup_frame, "OPENING USB CAMERA...", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imshow(window_name, startup_frame)
    cv2.waitKey(300)
    preview_mode = os.environ.get("GCS_RING_PREVIEW_MODE", "legacy").lower()
    if preview_mode in ("v3", "hybrid"):
        create_ring_v3_trackbars()

    print("圆环预览已启动，按 q 或 Esc 退出。")
    print("调焦: a=切换自动对焦, [ / ]=手动调焦, - / ==调 zoom。")
    print(f"圆环预览算法: {preview_mode}")
    if preview_mode in ("v3", "hybrid"):
        print("V3调参窗口: threshold/enhance/hough 半径等滑条实时生效。")
    print(f"上方摄像头: {CAMERA_UPPER_DEVICE}")

    try:
        autofocus_enabled = cap0.get(cv2.CAP_PROP_AUTOFOCUS) > 0
        frame_index = 0
        process_every = max(1, int(os.environ.get("GCS_RING_PROCESS_EVERY", "1")))
        last_results = []
        last_debug = {"raw": 0, "selected": 0, "nonzero": 0}
        while True:
            if window_closed(window_name):
                break

            ok, frame = cap0.read()
            if not ok or frame is None:
                wait_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(wait_frame, "WAITING CAMERA FRAME", (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow(window_name, wait_frame)
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), 27) or window_closed(window_name):
                    break
                continue

            frame_index += 1
            tune = None
            if (frame_index - 1) % process_every == 0:
                if preview_mode == "v3":
                    tune = read_ring_v3_trackbars()
                    last_results, last_debug = detect_ring_v3(frame, tune, (roi_left, roi_top, roi_right, roi_bottom))
                elif preview_mode == "hybrid":
                    tune = read_ring_v3_trackbars()
                    legacy_results, legacy_debug = detect_ring_legacy_preview(frame, (roi_left, roi_top, roi_right, roi_bottom))
                    v3_results, v3_debug = detect_ring_v3(frame, tune, (roi_left, roi_top, roi_right, roi_bottom))
                    if legacy_results:
                        last_results = legacy_results
                        source = "legacy"
                    else:
                        last_results = v3_results
                        source = "v3-fallback"
                    last_debug = {
                        "source": source,
                        "legacy_raw": legacy_debug["raw"],
                        "legacy_selected": legacy_debug["selected"],
                        "v3_raw": v3_debug["raw"],
                        "v3_selected": v3_debug["selected"],
                        "raw": legacy_debug["raw"] + v3_debug["raw"],
                        "selected": len(last_results),
                        "nonzero": v3_debug["nonzero"],
                    }
                else:
                    last_results, last_debug = detect_ring_legacy_preview(frame, (roi_left, roi_top, roi_right, roi_bottom))
            results = last_results
            debug = last_debug
            preview = frame.copy()
            cv2.rectangle(preview, (roi_left, roi_top), (roi_right, roi_bottom), (255, 255, 0), 2)
            status_color = (0, 255, 255)
            if preview_mode in ("v3", "hybrid") and tune is None:
                tune = read_ring_v3_trackbars()
            if preview_mode == "hybrid":
                status_lines = [
                    f"HYBRID src={debug.get('source', '-')}",
                    f"L {debug.get('legacy_raw', 0)}/{debug.get('legacy_selected', 0)} | V3 {debug.get('v3_raw', 0)}/{debug.get('v3_selected', 0)}",
                ]
            else:
                status_lines = [f"{preview_mode.upper()} raw={debug['raw']} sel={debug['selected']} every={process_every}"]
            if preview_mode in ("v3", "hybrid"):
                status_lines.append(f"thr={tune['threshold_value']} p2={tune['hough_param2']:.2f} r={tune['hough_min_radius']}-{tune['hough_max_radius']}")
            else:
                status_lines.append("hough p1=50 p2=30 r=35-38")

            for result in results:
                x, y, r = result["circle"]
                color_name = result["color"]
                areas = result["areas"]
                circle_color = {"R": (0, 0, 255), "G": (0, 255, 0), "B": (255, 0, 0), "?": (0, 180, 255)}[color_name]
                label = f"{color_name} x={x} y={y} r={r}"
                cv2.circle(preview, (x, y), r, circle_color, 2)
                cv2.circle(preview, (x, y), 2, circle_color, 3)
                cv2.putText(preview, label, (max(5, x - 80), max(20, y - r - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, circle_color, 1)
                status_lines[0] = f"V3 {label}"
                status_lines[1] = f"area R{areas['R']} G{areas['G']} B{areas['B']}"
                status_color = circle_color

            focus_value = cap0.get(cv2.CAP_PROP_FOCUS)
            zoom_value = cap0.get(cv2.CAP_PROP_ZOOM)
            draw_text_panel(preview, status_lines, 10, 10, status_color, 0.45)
            draw_text_panel(preview,
                            [f"AF {int(autofocus_enabled)} | F {focus_value:.0f} | Z {zoom_value:.0f}",
                             "keys: a AF, [ ] focus, - = zoom, q exit"],
                            10, frame.shape[0] - 52, (0, 255, 255), 0.45)
            cv2.imshow(window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27) or window_closed(window_name):
                break
            if key == ord("a"):
                autofocus_enabled = not autofocus_enabled
                cap0.set(cv2.CAP_PROP_AUTOFOCUS, 1 if autofocus_enabled else 0)
                print(f"AUTOFOCUS={cap0.get(cv2.CAP_PROP_AUTOFOCUS)}")
            elif key == ord("["):
                cap0.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                autofocus_enabled = False
                print(f"FOCUS={adjust_camera_property(cap0, cv2.CAP_PROP_FOCUS, -5)}")
            elif key == ord("]"):
                cap0.set(cv2.CAP_PROP_AUTOFOCUS, 0)
                autofocus_enabled = False
                print(f"FOCUS={adjust_camera_property(cap0, cv2.CAP_PROP_FOCUS, 5)}")
            elif key == ord("-"):
                print(f"ZOOM={adjust_camera_property(cap0, cv2.CAP_PROP_ZOOM, -1, 0, 100)}")
            elif key in (ord("="), ord("+")):
                print(f"ZOOM={adjust_camera_property(cap0, cv2.CAP_PROP_ZOOM, 1, 0, 100)}")
    finally:
        cap0.release()
        cap2 = None
        cv2.destroyAllWindows()

def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="工训视觉上位机程序")
    parser.add_argument("--self-test", action="store_true", help="仅执行无硬件上位机自检，不打开通信口和摄像头")
    parser.add_argument("--camera-test", action="store_true", help="只测试配置的摄像头能否打开并读取一帧")
    parser.add_argument("--camera-test-qr", action="store_true", help="只测试二维码 USB 摄像头能否打开并读取一帧")
    parser.add_argument("--camera-test-upper", action="store_true", help="只测试上方 USB 摄像头能否打开并读取一帧")
    parser.add_argument("--qr", action="store_true", help="调试二维码识别")
    parser.add_argument("--qr-preview", action="store_true", help="打开二维码可视化预览窗口，不打开通信口")
    parser.add_argument("--ring", action="store_true", help="调试色环第一次定位")
    parser.add_argument("--ring-preview", action="store_true", help="打开色环可视化预览窗口，不打开通信口")
    parser.add_argument("--ring-centered", action="store_true", help="调试色环居中定位")
    parser.add_argument("--maduo", action="store_true", help="调试码垛第一次定位")
    parser.add_argument("--maduo-centered", action="store_true", help="调试码垛居中定位")
    parser.add_argument("--dry-run-serial", action="store_true", help="只调视觉时不打开通信口，只打印待发送数据")
    return parser.parse_args(argv)

def run_self_test():
    """只验证上位机代码与基础算法，不访问通信口或摄像头硬件。"""
    print("上位机自检开始")
    print(f"Python: {sys.version.split()[0]}")
    print(f"OpenCV: {cv2.__version__}")
    print(f"NumPy: {np.__version__}")
    if PYZBAR_IMPORT_ERROR is None:
        print("二维码解码: pyzbar")
    else:
        print(f"二维码解码: OpenCV QRCodeDetector（pyzbar 不可用: {PYZBAR_IMPORT_ERROR}）")
    print(f"上位机通信口: {HOST_SERIAL_PORT} @ {HOST_SERIAL_BAUD}（自检不打开）")
    print(f"二维码摄像头: {CAMERA_QR_DEVICE}")
    print(f"上方摄像头: {CAMERA_UPPER_DEVICE}")

    checks = [
        ("颜色判断-红", get_color_name((0, 0, 255)) == ("Red", "1")),
        ("颜色判断-绿", get_color_name((0, 255, 0)) == ("Green", "2")),
        ("颜色判断-蓝", get_color_name((255, 0, 0)) == ("Blue", "3")),
    ]
    failed = [name for name, ok in checks if not ok]
    for name, ok in checks:
        print(f"{name}: {'OK' if ok else 'FAIL'}")

    if failed:
        raise SystemExit(f"上位机自检失败: {', '.join(failed)}")
    print("上位机自检通过：未打开通信口，未访问摄像头。")

def test_camera_device(name, device):
    cap_test = open_camera(device)
    if not cap_test.isOpened():
        print(f"{name}: FAIL open {device}")
        return False

    cap_test.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_test.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ok, frame = cap_test.read()
    cap_test.release()
    if not ok or frame is None:
        print(f"{name}: FAIL read {device}")
        return False

    print(f"{name}: OK {device}, shape={frame.shape}")
    return True

def run_camera_test():
    """只验证配置的摄像头设备可打开并能读取一帧。"""
    print("摄像头自检开始")
    checks = [
        test_camera_device("二维码摄像头", CAMERA_QR_DEVICE),
        test_camera_device("上方摄像头", CAMERA_UPPER_DEVICE),
    ]
    if not all(checks):
        raise SystemExit("摄像头自检失败：请检查 GCS_QR_CAMERA / GCS_UPPER_CAMERA 指向的设备。")
    print("摄像头自检通过。")

def run_single_camera_test(name, device):
    """只验证单个 USB 摄像头，适合 Windows 本机一次只接一个摄像头调试。"""
    print("单路摄像头自检开始")
    if not test_camera_device(name, device):
        raise SystemExit(f"{name} 自检失败：请检查对应 GCS_*_CAMERA 指向的 USB 摄像头。")
    print("单路摄像头自检通过。")

def main(argv=None):
    args = parse_args(argv)
    if args.dry_run_serial:
        set_dry_run_serial(True)
    if args.self_test:
        run_self_test()
        return
    if args.camera_test:
        run_camera_test()
        return
    if args.camera_test_qr:
        run_single_camera_test("二维码摄像头", CAMERA_QR_DEVICE)
        return
    if args.camera_test_upper:
        run_single_camera_test("上方摄像头", CAMERA_UPPER_DEVICE)
        return
    if args.qr_preview:
        run_qr_preview()
        return
    if args.ring_preview:
        set_dry_run_serial(True)
        run_ring_preview()
        return
    if args.qr or DEBUG_RUN_QR:
        run_erweima()
        return
    if args.ring or DEBUG_RUN_RING:
        run_sehuanyuanxin2()
        return
    if args.ring_centered or DEBUG_RUN_RING_CENTERED:
        run_sehuanyuanxin2_centered()
        return
    if args.maduo or DEBUG_RUN_MADUO:
        run_maduoyuanxin2()
        return
    if args.maduo_centered or DEBUG_RUN_MADUO_CENTERED:
        run_maduoyuanxin2_centered()
        return

    HostSerialCommandHandler().run()

if __name__ == "__main__":
    main()
