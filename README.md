# Graduation-Project-V4: 基于手势识别的智能云台控制系统

English Name: Gesture-Controlled Pan-Tilt System Based on SVM Classification

## 项目简介

本项目是一个基于计算机视觉的实时手势识别与嵌入式PID控制相结合的智能云台控制系统。系统采用PC端Python进行手势识别和模糊自适应PID预补偿，STM32端执行位置式PID控制，实现手势驱动的双轴云台精确跟踪。

## 核心功能

- **实时手部追踪**：基于MediaPipe Hands SDK提取21个手部关键点
- **SVM手势分类**：支持OK、Like、Peace等多种手势识别（归一化特征）
- **自适应EMA滤波**：速度自适应的指数移动平均滤波抑制追踪抖动
- **一阶速度预测**：基于历史速度外推下一帧位置，补偿系统延迟
- **模糊自适应PID(FuzzyPID)**：PC端预处理，模糊推理+ANFIS在线学习混合架构
- **STM32位置式PID**：嵌入式闭环控制，驱动双轴舵机云台
- **自定义串口协议**：`#x$y\r\n`格式，简洁高效

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PC端 (Python)                                   │
│  MediaPipe → EMA滤波 → 速度预测 → FuzzyPID补偿 → 串口发送 #x$y\r\n         │
│       ↓                                                              ↓      │
│  SVM分类(归一化特征) ←→ 三态状态机(WAIT/CONF/TRIGGERED)                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                     │
                                     │ 串口 (115200bps)
                                     ↓
┌─────────────────────────────────────────────────────────────────────────────┐
│                           STM32端 (C)                                        │
│  协议解析 → 误差计算 → 位置PID → PWM输出 → 双轴云台                         │
└─────────────────────────────────────────────────────────────────────────────┘
```


## 文件结构

```
Graduation-Project-V4/
├── svm/                                    # PC端Python代码
│   ├── HandTrackingModule.py               # 核心模块：手部追踪+EMA+预测+FuzzyPID+状态机
│   ├── FuzzyPID.py                         # 模糊自适应PID实现（模糊推理+ANFIS）
│   ├── collect_data.py                     # 数据采集：归一化坐标存入CSV
│   ├── train_model.py                      # 模型训练：SVM分类器
│   ├── inference.py                        # 独立推理脚本
│   └── gesture_model.pkl                    # 训练好的SVM模型
│
├── control/                                # STM32端嵌入式代码
│   ├── USER/
│   │   └── main.c                          # 主循环
│   ├── HARDWARE/
│   │   ├── serial/serial.c                 # 串口协议解析
│   │   ├── PID/pid.c                       # 位置式PID实现
│   │   └── TIMER/timer.c                   # TIM3 PWM初始化(50Hz)
│   └── SYSTEM/usart/usart.c                 # USART初始化
│
├── dataset.csv                              # 手势数据集（标签+42维归一化特征）
├── 毕业设计创新点.md                        # 创新点分析文档
├── FuzzyPID与HandTrackingModule联合输出详解.md  # FuzzyPID集成说明
└── README.md
```


## 创新点

1. **自适应EMA滤波 + 速度预测控制**：速度自适应调整alpha系数，双阶离群点剔除
2. **模糊推理 + ANFIS在线学习混合架构**：专家经验+在线学习双层补偿
3. **FuzzyPID与EMA预测的级联控制**：PC端FuzzyPID预处理 + STM32位置式PID
4. **基于SVM的归一化特征手势分类**：坐标归一化到[0,1]消除位置/尺度敏感性
5. **PC-STM32自定义串口协议**：简洁高效的跨平台通信
6. **双轴云台PWM控制**：50Hz舵机标准频率，300~1200占空比范围

## 快速开始

### 环境要求

- Python 3.7+
- OpenCV
- MediaPipe
- scikit-learn
- joblib
- NumPy
- STM32F10x开发板

### PC端运行

```bash
# 1. 采集手势数据（在不同位置/距离下采集）
python svm/collect_data.py

# 2. 训练SVM模型
python svm/train_model.py

# 3. 运行主程序
python svm/HandTrackingModule.py
```

### 按键说明

| 按键 | 功能 |
|------|------|
| `s` | 退出程序 |
| `p` | 切换FuzzyPID补偿 ON/OFF（对比实验用）|

### STM32端

1. 使用Keil uVision打开`control/USER/CONTROL.uvprojx`
2. 编译并烧录到STM32F103开发板
3. 系统自动运行，接收串口数据并执行PID控制

## 技术指标

| 指标 | 数值 |
|------|------|
| 帧率 | >30 FPS |
| 端到端延迟 | <30ms |
| 跟踪误差 | <5像素 |
| 手势识别准确率 | >90% |

## 相关文档

- [毕业设计创新点.md](毕业设计创新点.md) - 六大创新点详细分析
- [FuzzyPID与HandTrackingModule联合输出详解.md](FuzzyPID与HandTrackingModule联合输出详解.md) - FuzzyPID集成说明

## 作者

- **姓名**：亓新欣 (Miaoxin Qi)
- **学校**：河南理工大学 (Henan Polytechnic University)
- **专业**：自动化

## 开源协议

MIT License