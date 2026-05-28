# 六轴机械臂 — 手部视觉跟随控制系统

**自动化2402吴天玮**

基于 MediaPipe + 逆运动学 + STM32 的实时手部跟踪机械臂控制系统。摄像头检测手部姿态，通过逆运动学解算关节角度，串口发送指令驱动六轴机械臂实时跟随。

## 项目结构

```
├── HandIK_Controller.exe       # [推荐] 打包好的独立可执行程序, 双击即运行
├── arm_tracker.py              # PC 端视觉跟随程序 (Python 源码)
├── arm_tracker.spec            # PyInstaller 打包配置
├── 打包exe.bat                 # 一键重新打包 .exe
├── requirements.txt            # Python 依赖列表
├── 安装依赖.bat                # 一键安装 Python 依赖
├── models/
│   ├── hand_landmarker.task    # MediaPipe 手部 21 关键点模型
│   └── pose_landmarker_full.task  # MediaPipe 身体 33 关键点模型
├── jixiebi/                    # STM32 机械臂固件 (Keil MDK)
│   ├── jixiebi.ioc             # CubeMX 工程配置
│   ├── Core/                   # 外设初始化代码 (HAL 库)
│   ├── APP/                    # 应用层: 舵机 PWM / 串口协议 / 按键 / 调度器
│   ├── Drivers/                # STM32F1xx HAL 库 + CMSIS
│   └── MDK-ARM/                # Keil uVision 5 工程文件
└── README.md
```

## 硬件需求

| 组件 | 规格 |
|------|------|
| 主控 | STM32F103RBT6 (64KB RAM, 128KB Flash) |
| 舵机 | 6× 标准舵机 (500~2500µs, 0~180°) |
| 蓝牙 | HC-05/HC-06 模块 (UART3: PB10/PB11, 115200bps) |
| USB串口 | CH340/CP2102 (UART1: PA9/PA10, 19200bps) |
| 摄像头 | USB 摄像头, 建议 1080p |
| 电源 | 5~7.4V 舵机电源 (独立供电, 勿从 STM32 取电) |

## 引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| 舵机1 | PC10 | 夹爪 |
| 舵机2 | PC11 | 腕关节 |
| 舵机3 | PC12 | 肘关节 |
| 舵机4 | PD2 | 肩关节 |
| 舵机5 | PB5 | 腰部俯仰 |
| 舵机6 | PB8 | 基座旋转 |
| 按键 | PC0 | 模式切换 (上拉, 低有效) |
| 蓝灯 | PB9 | 模式指示 (高有效) |
| 串口1 TX | PA9 | USB 有线通信 (19200bps) |
| 串口1 RX | PA10 | |
| 串口3 TX | PB10 | 蓝牙通信 (115200bps) |
| 串口3 RX | PB11 | |

---

## 一、PC 端视觉程序 (arm_tracker.py)

### 环境要求

- Windows 10/11
- Python 3.9 ~ 3.12
- USB 摄像头

### 快速开始

**方式一 (推荐): 直接运行 exe**

```
1、双击 HandIK_Controller.exe 即可启动视觉上位机 (无需安装 Python)
2、在"dabao\jixiebi\MDK-ARM\jixiebi.uvprojx"中启动stm32下位机
3、选择好串口后即可运行
```

**方式二: Python 源码运行**

```bash
1、在cmd中：
   pip install -r requirements.txt   # 首次需安装依赖
   python arm_tracker.py
2、在"dabao\jixiebi\MDK-ARM\jixiebi.uvprojx"中启动stm32下位机
3、选择好串口后即可运行
```

程序启动后会扫描可用串口，选择机械臂对应的 COM 口 (有线连接选 19200bps 的那个)。直接回车可以进入调试模式 (不连接机械臂，仅显示)。

> **重新打包**: 若修改了py源码, 双击 `打包exe.bat` 可重新生成 `HandIK_Controller.exe`。

### 工作原理

```
摄像头 → MediaPipe 手部检测 (21关键点)
  ├─ 手腕坐标 (关键点0): x, y, z → 逆运动学 → S3~S6 关节角度
  ├─ 拇指-食指距离 (关键点4↔8) → 开合度 % → S1 夹爪角度
  └─ 身体姿态 (33关键点) → 半透明骨架叠加显示
         │
         └─ 串口 30Hz 发送二进制协议包 → STM32 → 舵机运动
```

### 逆运动学 (IK)

| 舵机 | 功能 | 计算方法 |
|------|------|---------|
| S6 | 基座旋转 | `90 - atan2(wx,wy) × 2.8` |
| S5 | 腰部俯仰 | `90 + 仰角 × 1.2`，限位 60~120° |
| S4 | 肩关节 | `90 + θ_shoulder × 1.8` |
| S3 | 肘关节 | `180 - θ_elbow × 1.3` |
| S2 | 腕关节 | `S4 - S3 + 90`，限位 30~150° |
| S1 | 夹爪 | 手指距离→0~100%→0~90° |

连杆: L1=6cm(腰→肩), L2=11cm(上臂), L3=10cm(前臂), L4=5cm(腕→末端)

### 键盘控制

| 按键 | 功能 |
|------|------|
| `q` / `ESC` | 退出 |
| `s` | 开关串口发送 |
| `c` | 校准手指开合范围 |
| `r` | 重置低通滤波器 |
| `t` | 切换左右手 (默认左手) |

### 可调参数 (arm_tracker.py 顶部)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `CAMERA_W/H` | 1920×1080 | 分辨率 |
| `WORKSPACE_*` | -20~20cm | 手部工作空间映射 |
| `FILTER_ALPHA` | 0.35 | 低通滤波 (大=快, 小=平滑) |
| `MAX_ANGLE_DELTA` | 3.0°/帧 | 角度突变限幅 |
| `SERIAL_BAUDRATE` | 19200 | 串口波特率 |
| `SEND_INTERVAL_MS` | 33 | 发送间隔 (30Hz) |
| `L1~L4` | 6,11,10,5cm | 机械臂连杆长度 |

---

## 二、STM32 固件 (jixiebi/)

### 开发环境

- **IDE**: Keil MDK-ARM (uVision 5.32+)
- **SDK**: STM32Cube FW_F1 V1.8.7
- **烧录**: ST-Link / J-Link / USB-TTL

### 编译与烧录

1. 双击 `jixiebi/MDK-ARM/jixiebi.uvprojx` 打开 Keil 工程
2. 点击 Build (F7) 编译
3. 点击 Download (F8) 烧录

### 固件架构

```
裸机超级循环 (无 RTOS)
├── 调度器 (scheduler.c): 时间片轮询
│   ├── uart_proc()   10ms  串口命令解析
│   ├── duoji_proc()  20ms  舵机 PWM 线性插值
│   └── key_proc()    10ms  按键扫描 + LED 闪烁
├── 软件 PWM (duoji_app.c): TIM3 1MHz 中断, 6路固定区域 PWM
├── 串口接收 (uart_app.c): DMA+空闲中断, 双协议支持
└── 按键逻辑 (key_app.c): 双模式 + 自动序列
```

### 串口协议

#### 文本协议 (调试用)

```
#A<id>:<angle>:<time>\n     角度控制 (0~180°)
#<id>:<pulse>:<time>\n      脉宽控制 (500~2500µs)
#ALL:<pulse>:<time>\n       全部舵机
#MID\n                      全部回中
#GRIP:OPEN/CLOSE/TOGGLE\n   夹爪控制
```

回复: `OK ...\r\n` 或 `ERR ...\r\n`

#### 二进制协议 (高速跟随, 30Hz)

```
55 55 16 01 06 <tmL> <tmH> [id angle 00]×6
│  │  │  │  │  └──────────┘ └─ 6组舵机数据 ─┘
│  │  │  │  └─ 时间(ms) uint16 LE
│  │  │  └─ 舵机数量
│  │  └─ CMD_MULT_SERVO_MOVE
│  └─ 数据长度 (22字节)
└─ 同步头
```

每包 25 字节, 19200bps 下传输仅需 ~13ms, 支持 30Hz 实时跟随。

### 按键 + LED 模式

| 操作 | 模式一 (抓取) | 模式二 (蓝牙) |
|------|-------------|-------------|
| LED | 快闪 100ms | 慢闪 500ms |
| 短按 | 自动执行 12 步抓取序列 | 无操作 |
| 长按 | 切换到模式二 | 切换到模式一 |

### 舵机角度映射

`0° → 500µs, 90° → 1500µs, 180° → 2500µs`

---

## 三、常见问题

| 问题 | 排查方向 |
|------|---------|
| 摄像头打不开 | 修改 `CAMERA_ID` (0/1/2) |
| 检测不到手 | 确保光线充足, 手在画面内 |
| 串口 PermissionError | COM 口被占用, 关闭串口助手 |
| 机械臂不动 | 检查波特率匹配 (19200), 舵机独立供电 |
| 机械臂抖动 | 增大 `FILTER_ALPHA` (更平滑) 或减小 `MAX_ANGLE_DELTA` |
| 跟随方向反了 | 修改 `inverse_kinematics` 中的伺服公式符号 |
| 舵机运动范围太大/太小 | 调整 IK 增益系数 (×1.8, ×2.8 等) |
| Keil 编译报错 | 确认安装了 STM32F1xx_DFP 包 |

---

## 四、依赖说明

### Python (pip install)

| 包 | 版本 | 用途 |
|----|------|------|
| mediapipe | >=0.10.0 | 手部+身体关键点检测 |
| opencv-python | >=4.8.0 | 摄像头采集 + 画面渲染 |
| pyserial | >=3.5 | 串口通信 |
| numpy | >=1.24.0 | 数值计算 |

### STM32 (Keil Pack)

- Keil.STM32F1xx_DFP.2.3.0+
- ARM CMSIS 4.5.0+
