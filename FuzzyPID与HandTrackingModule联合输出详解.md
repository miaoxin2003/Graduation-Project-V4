# FuzzyPID与HandTrackingModule联合输出详解

## 一、系统整体架构

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                              PC端 (Python)                                          │
│                                                                                     │
│  ┌───────────┐    ┌────────────┐    ┌───────────┐    ┌──────────┐    ┌─────────┐ │
│  │ MediaPipe │ →  │  EMA滤波    │ →  │ 速度预测   │ →  │ FuzzyPID │ → │ 串口发送 │ │
│  │ 21关键点   │    │ 自适应α     │    │ 一阶外推   │    │ 补偿u    │    │ #x$y\r\n│ │
│  └───────────┘    └────────────┘    └───────────┘    └──────────┘    └─────────┘ │
│        ↓                                                              ↓            │
│        ↓                                                        ┌─────────┐        │
│  ┌───────────┐    ┌───────────┐                                │ STM32   │        │
│  │ SVM分类    │    │ 三态状态机  │                                │ 位置PID  │        │
│  │(归一化特征)│    │ WAIT/CONF  │                                │ PWM输出  │        │
│  └───────────┘    └───────────┘                                └─────────┘        │
└─────────────────────────────────────────────────────────────────────────────────────┘
```


---

## 二、FuzzyPID与HandTrackingModule联合输出流程

### 2.1 数据流转步骤

```
步骤1: MediaPipe提取手部关键点
        ↓
步骤2: 自适应EMA滤波（抑制高频噪声）
        ↓
步骤3: 一阶速度预测（补偿延迟）
        ↓
步骤4: 计算误差e和误差变化率de
        ↓
步骤5: FuzzyPID计算补偿控制量u
        ↓
步骤6: 叠加补偿后发送给STM32
```

### 2.2 详细流程图

```
                                    ┌─────────────────┐
                                    │  摄像头捕获图像   │
                                    └────────┬────────┘
                                             ↓
                                    ┌─────────────────┐
                                    │ MediaPipe提取    │
                                    │ 21个关键点坐标    │
                                    │ (x, y) 共42维    │
                                    └────────┬────────┘
                                             ↓
                           ┌─────────────────┴─────────────────┐
                           ↓                                       ↓
               ┌───────────────────┐                   ┌───────────────────┐
               │    SVM手势分类     │                   │   坐标提取(拇指)   │
               │  (归一化特征[0,1]) │                   │   thumb_tip_x/y   │
               └───────────────────┘                   └─────────┬─────────┘
                           ↓                                       ↓
               ┌───────────────────┐                   ┌───────────────────┐
               │   状态机逻辑       │                   │  自适应EMA滤波    │
               │ WAITING→CONFIRM→  │                   │  α动态调整+离群点 │
               │ TRIGGERED         │                   │  剔除             │
               └───────────────────┘                   └─────────┬─────────┘
                                                                  ↓
                                                      ┌───────────────────┐
                                                      │   EMA平滑值        │
                                                      │ smooth_x, smooth_y │
                                                      └─────────┬─────────┘
                                                                  ↓
                                                      ┌───────────────────┐
                                                      │   一阶速度预测     │
                                                      │ pred_x = ema_x +  │
                                                      │       velocity_x  │
                                                      └─────────┬─────────┘
                                                                  ↓
                                                      ┌───────────────────┐
                                                      │  预测位置 pred_x   │
                                                      │       pred_y      │
                                                      └─────────┬─────────┘
                                                                  ↓
                           ┌─────────────────────────────────────┴───────────────┐
                           ↓                                                               ↓
               ┌───────────────────────────────┐                           ┌───────────────────────────────┐
               │     FuzzyPID_X 计算           │                           │     FuzzyPID_Y 计算           │
               │                               │                           │                               │
               │  error_x = pred_x - 320        │                           │  error_y = pred_y - 240        │
               │  de_x = (error_x - last_err_x)/dt                           │  de_y = (error_y - last_err_y)/dt                           │
               │           ↓                   │                           │           ↓                   │
               │  u_x, kp_x, kd_x, loss_x =   │                           │  u_y, kp_y, kd_y, loss_y =   │
               │       fpid_x.update()         │                           │       fpid_y.update()         │
               └─────────────┬─────────────────┘                           └─────────────┬─────────────────┘
                             ↓                                                       ↓
               ┌───────────────────────────────┐                           ┌───────────────────────────────┐
               │  send_x = pred_x + u_x * 0.5  │                           │  send_y = pred_y + u_y * 0.5  │
               │       (补偿后坐标)            │                           │       (补偿后坐标)            │
               └─────────────┬─────────────────┘                           └─────────────┬─────────────────┘
                             └───────────────────────┬───────────────────────┘
                                                    ↓
                                    ┌───────────────────────────────┐
                                    │     串口发送 #send_x$send_y\r\n │
                                    └─────────────┬─────────────────┘
                                                  ↓
                                    ┌───────────────────────────────┐
                                    │         STM32端               │
                                    │   协议解析 → PID → PWM        │
                                    └───────────────────────────────┘
```

---

## 三、SVM特征归一化说明

### 3.1 归一化背景

**问题**：旧方案使用绝对像素坐标 `(x, y)`，导致：
- 手在图像左侧和右侧时特征值差异大
- 手靠近或远离摄像头时坐标范围变化大
- 同一手势在不同位置/距离下识别精度下降

**解决方案**：将坐标归一化到 `[0, 1]` 范围

### 3.2 归一化公式


$$x_{norm} = \frac{x}{w}, \quad y_{norm} = \frac{y}{h}$$

其中：
- `w` = 图像宽度
- `h` = 图像高度

### 3.3 归一化优势

| 版本 | 特征范围 | 位置敏感 | 尺度敏感 |
|------|----------|----------|----------|
| 旧方案 | 0~640 (x), 0~480 (y) | ✅ 高 | ✅ 高 |
| **新方案** | 0.0~1.0 (x), 0.0~1.0 (y) | ❌ 低 | ❌ 低 |

### 3.4 代码实现


**数据采集时归一化** (`collect_data.py:29-41`)：
```python
if key == ord('s'):
    if len(lmList) != 0:
        # 提取 21 个点的 x, y 坐标（归一化到[0,1]）
        h, w = img.shape[:2]
        data = [label]
        for lm in lmList:
            norm_x = lm[1] / w  # 归一化x
            norm_y = lm[2] / h  # 归一化y
            data.extend([norm_x, norm_y])
        writer.writerow(data)
        print(f"已保存: {label} (归一化坐标)")
```

**推理时归一化** (`HandTrackingModule.py:155-162`)：
```python
if len(lmList) != 0 and model:
    # 提取 21 个点的归一化坐标（与训练时一致）
    h, w = img.shape[:2]
    features = []
    for lm in lmList:
        norm_x = lm[1] / w  # 归一化x
        norm_y = lm[2] / h  # 归一化y
        features.extend([norm_x, norm_y])
    prediction = model.predict([features])
```

---

## 四、FuzzyPID内部计算流程

```
                    ┌─────────────────────────────────────────┐
                    │           FuzzyPID.update(e, de)        │
                    └────────────────────┬────────────────────┘
                                         │
                    ┌────────────────────┴────────────────────┐
                    ↓                                         ↓
        ┌─────────────────────────┐              ┌─────────────────────────┐
        │   1. 模糊查表 (教师信号)  │              │   2. ANFIS前向传播      │
        │   infer_fast(e, de)     │              │   forward(e, de)        │
        │   → dkp_fuzzy, dkd_fuzzy│              │   → dkp, dkd, w_bar     │
        └────────────┬────────────┘              └────────────┬────────────┘
                     │                                        │
                     └────────────────────┬───────────────────┘
                                          ↓
                               ┌─────────────────────────┐
                               │  3. ANFIS反向学习       │
                               │  backward(e, de,       │
                               │    dkp_fuzzy, dkd_fuzzy)│
                               │  → loss (学习误差)      │
                               └────────────┬────────────┘
                                            ↓
                               ┌─────────────────────────┐
                               │  4. 自适应参数更新       │
                               │  Kp = clip(Kp_base+dkp) │
                               │  Kd = clip(Kd_base+dkd) │
                               └────────────┬────────────┘
                                            ↓
                               ┌─────────────────────────┐
                               │  5. 控制量计算          │
                               │  u = Kp*e + Ki*∫e + Kd*de│
                               │  (积分限幅+输出平滑)     │
                               └────────────┬────────────┘
                                            ↓
                               ┌─────────────────────────┐
                               │  返回 (u, Kp, Kd, loss)  │
                               └─────────────────────────┘
```

---

## 五、默认模式（不按P键）


### 5.1 默认状态

```python
detector.use_fuzzy_compensation = True  # 初始化时默认为True
```

### 5.2 数据流

```
原始坐标 → EMA滤波 → 预测位置 → FuzzyPID计算补偿u → 叠加到预测值 → 发送
                                                    ↓
                                          send_x = pred_x + u_x * 0.5
                                          send_y = pred_y + u_y * 0.5
```

### 5.3 终端输出标记

```
[PRED][FPID] (send_x, send_y)
```

- `[PRED]` - 预测功能启用
- `[FPID]` - FuzzyPID补偿启用

### 5.4 控制效果

- **优点**：误差更小、响应更平稳
- **原因**：FuzzyPID根据误差和误差变化率动态调整补偿量，提前修正偏差

---

## 六、切换模式（按下P键）

### 6.1 切换逻辑

```python
if key == ord('p'):
    detector.use_fuzzy_compensation = not detector.use_fuzzy_compensation
    print(f"[TOGGLE] FuzzyPID: {'ON' if detector.use_fuzzy_compensation else 'OFF'}")
```


### 6.2 按下P键后

```python
detector.use_fuzzy_compensation = False  # FuzzyPID补偿关闭
```

### 6.3 数据流

```
原始坐标 → EMA滤波 → 预测位置 → 直接发送（无FuzzyPID补偿）
                                            ↓
                                  send_x = pred_x
                                  send_y = pred_y
```

### 6.4 终端输出标记

```
[PRED] (send_x, send_y)
```

- `[PRED]` - 预测功能启用
- **无 `[FPID]`** - FuzzyPID补偿已关闭

### 6.5 控制效果

- **特点**：直接发送EMA+预测结果，无额外补偿
- **适用场景**：对比实验、调试系统稳定性

---

## 七、对比总结

| 模式 | FuzzyPID状态 | 数据流 | 终端标记 | 控制特点 |
|------|--------------|--------|----------|----------|
| **默认（不按P）** | ON | pred + u×0.5 | `[PRED][FPID]` | 自适应补偿，更平稳 |
| **切换后（按P）** | OFF | pred（直接） | `[PRED]` | 预测直接输出 |

---

## 八、关键代码片段

### 8.1 特征归一化（采集数据时）

```python
# collect_data.py
h, w = img.shape[:2]
for lm in lmList:
    norm_x = lm[1] / w  # 归一化x到[0,1]
    norm_y = lm[2] / h  # 归一化y到[0,1]
    data.extend([norm_x, norm_y])
```

### 8.2 特征归一化（推理时）

```python
# HandTrackingModule.py
h, w = img.shape[:2]
features = []
for lm in lmList:
    norm_x = lm[1] / w
    norm_y = lm[2] / h
    features.extend([norm_x, norm_y])
prediction = model.predict([features])
```

### 8.3 FuzzyPID初始化

```python
# HandTrackingModule.py:52-55
self.fpid_x = FuzzyPID()  # X轴FuzzyPID控制器
self.fpid_y = FuzzyPID()  # Y轴FuzzyPID控制器
self.use_fuzzy_compensation = True  # 默认启用补偿
```

### 8.4 FuzzyPID计算调用

```python
# HandTrackingModule.py:325-334

# 计算误差（相对于图像中心320x240）
error_x = pred_x - 320
error_y = pred_y - 240

# 计算误差变化率
dt = curr_time - detector.last_time
de_x = (error_x - detector.last_error_x) / dt
de_y = (error_y - detector.last_error_y) / dt

# FuzzyPID计算
u_x, kp_x, kd_x, loss_x = detector.fpid_x.update(error_x, de_x)
u_y, kp_y, kd_y, loss_y = detector.fpid_y.update(error_y, de_y)

# 根据开关决定是否叠加补偿
if detector.use_fuzzy_compensation:
    send_x = int(pred_x + u_x * 0.5)  # 启用补偿
    send_y = int(pred_y + u_y * 0.5)
else:
    send_x = int(pred_x)  # 关闭补偿，直接使用预测值
    send_y = int(pred_y)
```

### 8.5 P键切换逻辑

```python
# HandTrackingModule.py:371-373
if key == ord('p'):
    detector.use_fuzzy_compensation = not detector.use_fuzzy_compensation
    print(f"[TOGGLE] FuzzyPID: {'ON' if detector.use_fuzzy_compensation else 'OFF'}")
```

---

## 九、运行示例

### 9.1 默认模式（FuzzyPID ON）

```
Raw: (356, 198) -> Smooth: (342, 212) -> [PRED][FPID] (344, 208) | Err:(24.0,-32.0) de:(15.2,-18.7) Kp:(0.092,0.105) u:(8.45,-12.31)
```

- 发送给STM32: `#344$208\r\n`
- FuzzyPID输出的补偿量 `u` 被叠加到预测位置

### 9.2 切换后模式（FuzzyPID OFF）

```
[TOGGLE] FuzzyPID: OFF
Raw: (356, 198) -> Smooth: (342, 212) -> [PRED] (342, 212)
```

- 发送给STM32: `#342$212\r\n`
- 直接使用EMA+预测结果，无FuzzyPID补偿

---

## 十、适用场景建议

| 场景 | 建议模式 |
|------|----------|
| 正常使用 | 默认（FuzzyPID ON） |
| 对比实验 | 按P切换对比 |
| 系统调试 | 观察terminal输出的误差/Kp/u值 |
| 性能瓶颈排查 | 切换OFF看是否FuzzyPID引入延迟 |

---

## 十一、文件对应关系

| 文件 | 作用 |
|------|------|
| `HandTrackingModule.py` | 主循环，调用FuzzyPID计算补偿；归一化特征用于SVM |
| `FuzzyPID.py` | FuzzyPID类定义，`update()`方法实现 |
| `collect_data.py` | 数据采集时归一化坐标到[0,1] |
| `train_model.py` | SVM训练（输入归一化特征） |
| `main.c` (STM32) | 接收串口数据，执行位置PID控制 |
| `pid.c` (STM32) | 位置式PID实现 |

---

## 十二、数学公式汇总

### 特征归一化
$$x_{norm} = \frac{x}{w}, \quad y_{norm} = \frac{y}{h}$$

### EMA滤波
$$EMA_k = \alpha \cdot x_k + (1-\alpha) \cdot EMA_{k-1}$$

### 速度自适应α
$$\alpha = \begin{cases} 0.15 & v > 15 \\ 0.05 & v \leq 15 \end{cases}$$

### 一阶预测
$$\hat{x}_{k+1} = EMA_k + (EMA_k - EMA_{k-1})$$

### 误差计算
$$e_x = pred\_x - 320, \quad e_y = pred\_y - 240$$

### 误差变化率
$$de_x = \frac{e_x - e_{x,last}}{\Delta t}$$

### FuzzyPID输出
$$u = K_p \cdot e + K_i \cdot \int e \cdot dt + K_d \cdot de$$

### 最终发送值
$$send\_x = pred\_x + u_x \times 0.5$$
$$send\_y = pred\_y + u_y \times 0.5$$