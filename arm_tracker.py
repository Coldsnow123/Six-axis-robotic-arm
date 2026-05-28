"""
Arm Tracker — MediaPipe Hand + IK + Serial 机械臂实时跟随系统

基于 MediaPipe Hand Landmarker 检测手部 21 个关键点，
通过逆运动学 (IK) 将手腕位置映射为机械臂关节角度，
再通过串口 (UART1 有线 / UART3 蓝牙) 发送给 STM32 机械臂。


依赖: mediapipe, opencv-python, pyserial, numpy
"""

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision
import serial.tools.list_ports
import serial
import time
import math
import sys
import threading

# ============================================================
# 配置常量 (用户可修改)
# ============================================================

# 模型路径 (兼容 PyInstaller 打包和源码运行)
import sys as _sys, os as _os
_BASE_DIR = getattr(_sys, '_MEIPASS', _os.path.dirname(_os.path.abspath(__file__)))
MODEL_PATH      = _os.path.join(_BASE_DIR, "models", "hand_landmarker.task")
POSE_MODEL_PATH = _os.path.join(_BASE_DIR, "models", "pose_landmarker_full.task")
CAMERA_ID       = 0
CAMERA_W        = 1920
CAMERA_H        = 1080


# 工作空间映射 (手腕归一化坐标 → 真实空间, 单位 cm)
# x: 左右, y: 高度, z: 深度 (MediaPipe 世界坐标系)
WORKSPACE_X_MIN = -20.0
WORKSPACE_X_MAX =  20.0
WORKSPACE_Y_MIN =   5.0   # 最近 (手靠近身体 → 机械臂近端)
WORKSPACE_Y_MAX =  35.0   # 最远 (手远离身体 → 机械臂远端)
WORKSPACE_Z_MIN = -10.0   # 最低
WORKSPACE_Z_MAX =  20.0   # 最高

# 机械臂 DH 参数 (用户根据实际修改)
# 6 轴: 基座旋转 + 4 连杆平面臂 (腰/肩/肘/腕)
L1 = 6.0    # 舵机5: 腰部到肩部高度 (cm)
L2 = 11.0   # 舵机4: 上臂长 (cm)
L3 = 10.0   # 舵机3: 前臂长 (cm)
L4 = 5.0    # 舵机2: 腕部到末端 (cm), 短连杆主要控制朝向

# 关节限位 (舵机角度: 0~180°)
JOINT_LIMITS = {
    'servo6': (10, 170),    # 基座旋转
    'servo5': (60, 120),    # 腰部俯仰 (90=直立)
    'servo4': (30, 170),    # 肩关节
    'servo3': (20, 160),    # 肘关节
    'servo2': (30, 150),    # 腕关节
    'servo1': (0, 100),     # 夹爪 (0=开, 90+=合)
}

# 舵机中位 (所有回中位置)
SERVO_MID = {
    'servo6': 90,
    'servo5': 90,
    'servo4': 90,
    'servo3': 90,
    'servo2': 90,
    'servo1': 0,
}

# 滤波
FILTER_ALPHA = 0.35       # 一阶低通 (增大=响应更快)
MAX_ANGLE_DELTA = 3.0     # 每帧角度最大变化 (度)

# 串口
SERIAL_BAUDRATE = 19200
SEND_INTERVAL_MS = 33     # 30Hz (二进制包仅25字节, 19200bps下≈13ms)
SEND_TIME_MS    = 30      # 舵机运动时间 (跟随模式)
DEBUG_SERIAL    = True    # 调试: 打印 STM32 回复

# 手指开合校准默认值
GRIP_DIST_MIN = 0.02      # 捏紧时拇指-食指距离 (归一化)
GRIP_DIST_MAX = 0.25      # 张开时距离

# 手部检测
DETECT_CONFIDENCE = 0.5
TRACKING_CONFIDENCE = 0.5

# ============================================================
# 全局状态
# ============================================================

g_serial      = None        # Serial 对象
g_serial_on   = True        # 串口发送开关
g_hand_idx    = 1           # 0=右手, 1=左手 (MediaPipe 惯例)
g_grip_min    = GRIP_DIST_MIN
g_grip_max    = GRIP_DIST_MAX

# 滤波状态
g_filt_x  = 0.0
g_filt_y  = 0.0
g_filt_z  = 0.0
g_filt_grip = 0.0
g_filt_inited = False

# 上一帧关节角度 (用于限幅)
g_last_angles = {
    'servo6': SERVO_MID['servo6'],
    'servo5': SERVO_MID['servo5'],
    'servo4': SERVO_MID['servo4'],
    'servo3': SERVO_MID['servo3'],
    'servo2': SERVO_MID['servo2'],
    'servo1': SERVO_MID['servo1'],
}

# 无解时保持上一帧
g_last_valid_angles = dict(g_last_angles)
g_no_solution = False

# 无手部检测
g_hand_detected = False
g_com_status    = "调试模式"


# ============================================================
# 串口扫描与选择
# ============================================================

def scan_serial_ports():
    """扫描所有可用串口"""
    ports = list(serial.tools.list_ports.comports())
    return [(p.device, p.description) for p in ports]


def select_serial_port():
    """列出串口并让用户选择, 返回 Serial 对象或 None"""
    ports = scan_serial_ports()
    print("\n=== 可用串口 ===")
    if not ports:
        print("  未检测到串口, 将使用调试模式")
        return None

    for i, (dev, desc) in enumerate(ports):
        print(f"  [{i}] {dev} - {desc}")

    try:
        choice = input(f"选择串口 (0~{len(ports)-1}), 直接回车=调试模式: ").strip()
        if choice == "":
            print("  已选择调试模式")
            return None
        idx = int(choice)
        if 0 <= idx < len(ports):
            dev = ports[idx][0]
            ser = serial.Serial(dev, SERIAL_BAUDRATE, timeout=0.05)
            print(f"  已连接 {dev}")
            return ser
    except (ValueError, serial.SerialException) as e:
        print(f"  串口连接失败: {e}, 使用调试模式")
    return None


# ============================================================
# 逆运动学 (几何解析法)
# ============================================================

def inverse_kinematics(wx, wy, wz):
    """
    6 轴机械臂逆运动学求解 (几何解析法)

    轴映射:
        servo6 = 基座旋转 (水平面, 左右转动)
        servo5 = 腰部俯仰 (垂直面, 前俯/直立)
        servo4 = 肩关节 (垂直面, 上臂)
        servo3 = 肘关节 (垂直面, 前臂)
        servo2 = 腕关节 (垂直面, 末端朝向微调)

    求解策略:
        1. 基座旋转 → servo6 (atan2)
        2. 腰部 + 肩 + 肘 → 3 连杆平面 IK (servo5/4/3)
        3. 腕关节 → servo2 (保持末端水平, 补偿肩肘角度)
    """
    angles = {}

    # ── 1. 基座旋转 (servo6): 使臂平面指向目标 ──
    base_deg = math.degrees(math.atan2(wx, wy))
    angles['servo6'] = SERVO_MID['servo6'] - base_deg * 2.8  # 90° 中位

    # ── 2. 平面臂 IK: 在旋转后的垂直平面 (r, z) 内求解 ──
    r = math.sqrt(wx * wx + wy * wy)  # 水平距离
    z = wz

    # 2.1 腰部 (servo5): 承担部分仰角, 让 L1 朝向目标
    if r > 0.01:
        elev = math.atan2(z, r)
        target_theta5 = elev * 0.35
    else:
        target_theta5 = 0.0

    # L1 末端的 (r, z) 位置
    r1 = L1 * math.cos(target_theta5)
    z1 = L1 * math.sin(target_theta5)

    # 剩余向量: 肩部 → 目标
    dr = r - r1
    dz = z - z1
    d  = math.sqrt(dr * dr + dz * dz)

    # 2.2 肩 + 肘 2 连杆 IK (L2 + L3)
    cos_elbow = (d * d - L2 * L2 - L3 * L3) / (2.0 * L2 * L3)
    if cos_elbow < -1.0 or cos_elbow > 1.0:
        return None  # 不可达

    theta_elbow = math.acos(max(-1.0, min(1.0, cos_elbow)))  # 0~π
    theta_shoulder = math.atan2(dz, dr) - math.atan2(
        L3 * math.sin(theta_elbow), L2 + L3 * math.cos(theta_elbow)
    )

    # ── 3. 映射到舵机角度 ──
    # servo5: 腰部 (90°=直立, >90°=前俯)
    angles['servo5'] = 90.0 + math.degrees(target_theta5) * 1.2

    # servo4: 肩关节 (90°=中位)
    angles['servo4'] = 90.0 + math.degrees(theta_shoulder) * 1.8

    # servo3: 肘关节 (180°=完全伸直, 0°=完全弯曲)
    angles['servo3'] = 180.0 - math.degrees(theta_elbow) * 1.3

    # servo2: 腕关节 — 补偿肩肘, 使末端保持合理朝向
    raw_s2 = angles['servo4'] - angles['servo3'] + 90.0
    angles['servo2'] = max(30.0, min(150.0, raw_s2))

    return angles


def clamp_angle(val, limits):
    """将角度限制在关节限位内"""
    lo, hi = limits
    return max(lo, min(hi, val))


def apply_angle_limits(angles):
    """对 IK 输出施加限位 + 限幅"""
    global g_last_angles

    if angles is None:
        return None

    limited = {}
    for key in ['servo6', 'servo5', 'servo4', 'servo3', 'servo2', 'servo1']:
        raw = angles.get(key, g_last_angles[key])
        # 关节限位
        clamped = clamp_angle(raw, JOINT_LIMITS[key])
        # 每帧限幅 (防突变)
        delta = clamped - g_last_angles[key]
        if abs(delta) > MAX_ANGLE_DELTA:
            clamped = g_last_angles[key] + math.copysign(MAX_ANGLE_DELTA, delta)
        limited[key] = round(clamped, 1)
        g_last_angles[key] = limited[key]

    return limited


# ============================================================
# 低通滤波器
# ============================================================

def lowpass(new_val, prev_val):
    """一阶低通滤波"""
    return FILTER_ALPHA * new_val + (1.0 - FILTER_ALPHA) * prev_val


# ============================================================
# 手指开合度计算
# ============================================================

def calc_grip_percent(dist):
    """
    将拇指-食指距离映射为 0~100% 开合度
    0% = 捏紧, 100% = 张开
    """
    global g_grip_min, g_grip_max
    span = g_grip_max - g_grip_min
    if span < 0.001:
        return 0.0
    pct = (dist - g_grip_min) / span * 100.0
    return max(0.0, min(100.0, pct))


def grip_to_servo1(pct):
    """
    开合度 → 舵机1角度
    0% (捏紧) → 90°, 100% (张开) → 0°
    """
    return (100.0 - pct) * 0.9


# ============================================================
# 串口发送
# ============================================================

def send_angles_to_arm(angles, grip_pct):
    """发送关节角度到机械臂 (二进制高速协议, 单包6轴)"""
    global g_serial, g_serial_on

    if not g_serial or not g_serial_on:
        return

    # 舵机1 角度从开合度计算
    angles['servo1'] = grip_to_servo1(grip_pct)

    # 二进制包: 55 55 len 0x01 count tmL tmH [id angle 0]×6
    try:
        buf = bytearray()
        buf.append(0x55)       # header 1
        buf.append(0x55)       # header 2
        buf.append(22)         # len = 1+1+2+6*3
        buf.append(0x01)       # CMD_MULT_SERVO_MOVE
        buf.append(6)          # servo count
        buf.append(SEND_TIME_MS & 0xFF)      # time lo
        buf.append((SEND_TIME_MS >> 8) & 0xFF)  # time hi

        for sid in ['servo6', 'servo5', 'servo4', 'servo3', 'servo2', 'servo1']:
            buf.append(int(sid[-1]))         # servo id (1-6)
            buf.append(int(angles[sid]))     # angle (0-180)
            buf.append(0)                    # reserved

        g_serial.write(bytes(buf))
        g_serial.flush()
    except serial.SerialException as e:
        print(f"  串口发送失败: {e}")


# ============================================================
# 可视化工具
# ============================================================

def draw_pose_skeleton(img, landmarks, w, h):
    """绘制上半身骨架 (头/肩/臂/躯干), 半透明风格"""
    overlay = img.copy()

    # 关键连接: (a, b, color_bgr)
    BONE = [
        # 躯干
        (11, 12, (180, 180, 180)),  # 肩膀连线
        (11, 23, (160, 160, 160)),  # 左肩→髋
        (12, 24, (160, 160, 160)),  # 右肩→髋
        (23, 24, (140, 140, 140)),  # 髋连线
        # 左臂
        (11, 13, (200, 180, 100)),  # 左肩→肘
        (13, 15, (220, 200, 120)),  # 左肘→腕
        # 右臂
        (12, 14, (200, 180, 100)),  # 右肩→肘
        (14, 16, (220, 200, 120)),  # 右肘→腕
        # 头
        (0, 1, (100, 200, 100)),
        (1, 2, (100, 200, 100)),
        (2, 3, (100, 200, 100)),
        (3, 7, (100, 200, 100)),
        (0, 4, (100, 200, 100)),
        (4, 5, (100, 200, 100)),
        (5, 6, (100, 200, 100)),
        (6, 8, (100, 200, 100)),
        # 头→肩
        (7, 11, (120, 180, 120)),
        (8, 12, (120, 180, 120)),
        # 鼻→眼
        (0, 2, (80, 160, 80)),
        (0, 5, (80, 160, 80)),
    ]

    pts = {}
    for i, lm in enumerate(landmarks):
        x = int(lm.x * w)
        y = int(lm.y * h)
        pts[i] = (x, y)

    for a, b, color in BONE:
        if a in pts and b in pts:
            cv2.line(overlay, pts[a], pts[b], color, 3, cv2.LINE_AA)

    # 关键点
    for i, (x, y) in pts.items():
        if i in (11, 12, 13, 14, 15, 16, 0):
            cv2.circle(overlay, (x, y), 5, (255, 255, 255), -1, cv2.LINE_AA)

    # 半透明混合
    alpha = 0.55
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


def draw_hand_skeleton(img, landmarks, w, h):
    """绘制手部 21 个关键点骨架"""
    # MediaPipe Hand 关键点连接
    HAND_CONNECTIONS = [
        (0,1),(1,2),(2,3),(3,4),           # 拇指
        (0,5),(5,6),(6,7),(7,8),           # 食指
        (0,9),(9,10),(10,11),(11,12),      # 中指
        (0,13),(13,14),(14,15),(15,16),    # 无名指
        (0,17),(17,18),(18,19),(19,20),    # 小指
        (5,9),(9,13),(13,17),              # 掌骨连线
    ]

    pts = {}
    for i, lm in enumerate(landmarks):
        x = int(lm.x * w)
        y = int(lm.y * h)
        pts[i] = (x, y)

    # 连线
    for a, b in HAND_CONNECTIONS:
        if a in pts and b in pts:
            cv2.line(img, pts[a], pts[b], (0, 255, 0), 2)

    # 关键点
    for i, (x, y) in pts.items():
        r = 5 if i in (0, 4, 8) else 3
        color = (0, 0, 255) if i in (0, 4, 8) else (255, 255, 0)
        cv2.circle(img, (x, y), r, color, -1)

    return pts


def draw_overlay(img, info_lines):
    """在画面左上角绘制信息面板"""
    x0, y0 = 10, 30
    for i, text in enumerate(info_lines):
        y = y0 + i * 22
        # 背景半透明条
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
        cv2.rectangle(img, (x0 - 5, y - th - 5), (x0 + tw + 5, y + 5),
                      (0, 0, 0), -1)
        cv2.putText(img, text, (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)


# ============================================================
# 主函数
# ============================================================

def main():
    global g_serial, g_serial_on, g_hand_idx
    global g_filt_x, g_filt_y, g_filt_z, g_filt_grip, g_filt_inited
    global g_last_angles, g_last_valid_angles, g_no_solution
    global g_hand_detected, g_com_status
    global g_grip_min, g_grip_max

    print("=" * 50)
    print("  Hand IK Controller — 机械臂手部跟随系统")
    print("=" * 50)

    # ── 串口初始化 ────────────────────────────
    g_serial = select_serial_port()
    if g_serial:
        g_com_status = g_serial.port
    else:
        g_com_status = "调试模式"

    # ── MediaPipe Hand Landmarker ─────────────
    base_opts = mp_python.BaseOptions(model_asset_path=MODEL_PATH)
    opts = vision.HandLandmarkerOptions(
        base_options=base_opts,
        running_mode=vision.RunningMode.VIDEO,
        num_hands=2,
        min_hand_detection_confidence=DETECT_CONFIDENCE,
        min_tracking_confidence=TRACKING_CONFIDENCE,
    )
    detector = vision.HandLandmarker.create_from_options(opts)
    print("  Hand Landmarker 已加载")

    # ── MediaPipe Pose Landmarker ─────────────
    pose_base = mp_python.BaseOptions(model_asset_path=POSE_MODEL_PATH)
    pose_opts = vision.PoseLandmarkerOptions(
        base_options=pose_base,
        running_mode=vision.RunningMode.VIDEO,
        num_poses=1,
    )
    pose_detector = vision.PoseLandmarker.create_from_options(pose_opts)
    print("  Pose Landmarker 已加载")

    # ── 摄像头 ────────────────────────────────
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_H)
    if not cap.isOpened():
        print("  无法打开摄像头!")
        detector.close()
        pose_detector.close()
        return

    # ── 主循环 ────────────────────────────────
    fps_timer = time.time()
    fps_count = 0
    fps_value = 0
    last_send_time = 0

    print("\n  启动成功! 按 q/ESC 退出\n")

    while True:
        success, frame = cap.read()
        if not success:
            continue

        # 水平翻转 (镜像, 更自然)
        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        # BGR → RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        timestamp_ms = int(cap.get(cv2.CAP_PROP_POS_MSEC))

        # ── 检测身体姿态 (先画身体, 再叠手部) ──
        pose_result = pose_detector.detect_for_video(mp_img, timestamp_ms)
        if pose_result.pose_landmarks:
            for landmarks in pose_result.pose_landmarks:
                draw_pose_skeleton(frame, landmarks, w, h)

        # ── 检测手部 ──────────────────────────
        result = detector.detect_for_video(mp_img, timestamp_ms)

        # ── 处理检测结果 ──────────────────────
        g_hand_detected = False
        wrist_x = wrist_y = wrist_z = 0.0
        grip_dist = 0.0

        if result.hand_landmarks and len(result.hand_landmarks) > 0:
            # 选择左手或右手 (右手 Index=0 在 MediaPipe 中对应 "Right")
            target_found = False
            for idx, landmarks in enumerate(result.hand_landmarks):
                handedness = result.handedness[idx]
                # handedness[0].category_name = "Left" or "Right"
                hand_label = handedness[0].category_name
                target_label = "Right" if g_hand_idx == 0 else "Left"

                if hand_label == target_label:
                    target_found = True
                    g_hand_detected = True

                    # 绘制骨架
                    pts = draw_hand_skeleton(frame, landmarks, w, h)

                    # 手腕位置 (关键点 0)
                    lm0 = landmarks[0]
                    wrist_x = lm0.x  # 归一化 0~1
                    wrist_y = lm0.y
                    wrist_z = lm0.z  # MediaPipe 世界坐标 (近似深度)

                    # 拇指-食指距离 (关键点 4 和 8)
                    lm4  = landmarks[4]
                    lm8  = landmarks[8]
                    grip_dist = math.sqrt(
                        (lm4.x - lm8.x) ** 2 +
                        (lm4.y - lm8.y) ** 2 +
                        (lm4.z - lm8.z) ** 2
                    )

                    # 高亮手腕和指尖
                    if 0 in pts and 4 in pts and 8 in pts:
                        cv2.line(frame, pts[4], pts[8], (255, 0, 255), 2)
                    break

            if not target_found:
                g_hand_detected = False

        # ── 坐标映射 (归一化 → 真实空间 cm) ──
        if g_hand_detected:
            mx = WORKSPACE_X_MIN + wrist_x * (WORKSPACE_X_MAX - WORKSPACE_X_MIN)
            my = WORKSPACE_Y_MIN + (1.0 - wrist_y) * (WORKSPACE_Y_MAX - WORKSPACE_Y_MIN)
            mz = WORKSPACE_Z_MIN + (0.5 - wrist_z) * (WORKSPACE_Z_MAX - WORKSPACE_Z_MIN)
        else:
            mx = my = mz = 0.0

        # ── 低通滤波 ──────────────────────────
        if g_hand_detected:
            if not g_filt_inited:
                g_filt_x = mx
                g_filt_y = my
                g_filt_z = mz
                g_filt_grip = grip_dist
                g_filt_inited = True
            else:
                g_filt_x    = lowpass(mx, g_filt_x)
                g_filt_y    = lowpass(my, g_filt_y)
                g_filt_z    = lowpass(mz, g_filt_z)
                g_filt_grip = lowpass(grip_dist, g_filt_grip)

        grip_pct = calc_grip_percent(g_filt_grip)

        # ── 逆运动学 ──────────────────────────
        raw_angles = inverse_kinematics(-g_filt_x, g_filt_y, g_filt_z)  # X取反消除镜像
        g_no_solution = (raw_angles is None)

        if raw_angles is not None:
            g_last_valid_angles = raw_angles

        limited_angles = apply_angle_limits(g_last_valid_angles)
        if limited_angles is None:
            limited_angles = dict(g_last_angles)

        # ── 串口发送 (30Hz) ────────────────────
        now_ms = time.time() * 1000
        if g_serial and g_serial_on and g_hand_detected and not g_no_solution:
            if now_ms - last_send_time >= SEND_INTERVAL_MS:
                send_angles_to_arm(limited_angles, grip_pct)
                last_send_time = now_ms

        # ── 组装显示信息 ──────────────────────
        info = []
        if g_hand_detected:
            info.append(f"Wrist: ({g_filt_x:.1f}, {g_filt_y:.1f}, {g_filt_z:.1f}) cm")
        else:
            info.append("Wrist: --")
        info.append(f"Grip: {grip_pct:.0f}%  (dist={g_filt_grip:.3f})")
        info.append(f"S6={limited_angles['servo6']:.0f}  "
                     f"S5={limited_angles['servo5']:.0f}  "
                     f"S4={limited_angles['servo4']:.0f}  "
                     f"S3={limited_angles['servo3']:.0f}  "
                     f"S2={limited_angles['servo2']:.0f}")
        info.append(f"COM: {g_com_status}  "
                     f"Send: {'ON' if g_serial_on else 'OFF'}  "
                     f"Mode: {'Right' if g_hand_idx == 0 else 'Left'}  "
                     f"FPS: {fps_value}")

        if g_no_solution:
            info.append("IK: 目标不可达")
        if not g_hand_detected:
            info.append("状态: 等待手部检测...")

        draw_overlay(frame, info)

        # ── 显示 ──────────────────────────────
        cv2.imshow("Hand IK Controller", frame)

        # FPS 计算
        fps_count += 1
        if time.time() - fps_timer >= 1.0:
            fps_value = fps_count
            fps_count = 0
            fps_timer = time.time()

        # ── 键盘处理 ──────────────────────────
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):    # ESC / q → 退出
            break
        elif key == ord('s'):               # s → 切换串口发送
            g_serial_on = not g_serial_on
            print(f"  串口发送: {'ON' if g_serial_on else 'OFF'}")
        elif key == ord('c'):               # c → 校准手指
            if g_hand_detected:
                # 当前距离作为中点, ±50%
                g_grip_min = max(0.005, g_filt_grip * 0.3)
                g_grip_max = min(0.4, g_filt_grip * 2.0)
                print(f"  手指校准: min={g_grip_min:.4f}, max={g_grip_max:.4f}")
        elif key == ord('r'):               # r → 重置滤波
            g_filt_x = g_filt_y = g_filt_z = g_filt_grip = 0.0
            g_filt_inited = False
            print("  滤波器已重置")
        elif key == ord('t'):               # t → 切换左右手
            g_hand_idx ^= 1
            g_filt_inited = False
            print(f"  切换至: {'右手' if g_hand_idx == 0 else '左手'}")

    # ── 清理 ────────────────────────────────
    cap.release()
    cv2.destroyAllWindows()
    if g_serial:
        g_serial.close()
    detector.close()
    pose_detector.close()
    print("  已退出")


if __name__ == "__main__":
    main()
