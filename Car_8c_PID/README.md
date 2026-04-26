# Car_8c_PID - 8通道PID智能四驱循迹小车

这是一个基于 STM32F103 系列微控制器的智能循迹小车项目。项目采用 8 路红外灰度传感器，通过 PID 控制算法和有限状态机 (FSM) 实现稳定、高速的赛道循迹，支持处理十字路口、锐角/直角弯道以及虚线/间隙等复杂工况。

## 🚀 项目特性

- **8通道高精度采样**：利用 ADC + DMA 方式并行采集 8 路传感器数据，实时感知赛道位置。
- **加权平均PID算法**：采用加权平均法计算偏差（权重范围从 -700 到 +700），配合位置式 PID 调节，转向平滑且鲁棒。当完全脱离黑线时，系统会记录上一时刻偏差以保持惯性循迹。
- **有限状态机 (FSM) 策略层**：系统不仅是简单的 PID 循迹，还通过 FSM 实现对赛道特征的语义识别：
  - **循迹态 (`LINE_FOLLOW`)**：根据 8 路传感器反馈，使用加权平均算法输出偏差，实时调整 PWM。
  - **十字路动态 (`CROSSROAD`)**：通过“特征统计(黑线点数>5)”或“时间窗过滤(左右边缘突变)”双重判定进入。进入后强制直线加速冲刺，防止 PID 在交叉路口产生摆动。
  - **原地转向态 (`LEFT/RIGHT_TURN`)**：识别到锐角或直角边缘时，系统启动原地差速转向，锁定时间内不进行 PD 运算。
  - **盲跑态 (`GAP`)**：应对虚线或赛道缺失，保持当前趋势并以固定速度推行，直至重新捕捉黑线。
  - **模式独立性**：策略层完全非阻塞实现，不干扰定时器底层逻辑。
- **模块化设计**：驱动层 (`main.c`) 与策略层 (`strategy.c`) 完全分离，易于调试和移植。

## 🛠 硬件配置

| 功能模块 | 硬件资源 | 引脚分配 | 说明 |
| :--- | :--- | :--- | :--- |
| **MCU** | STM32F103C8T6 | - | 主频 72MHz (外部晶振) |
| **电机驱动** | TIM4 PWM | PB6, PB7, PB8, PB9 | 四路 PWM，频率约为 20kHz |
| **电机方向** | GPIO | PB12-15, PA8, PA11, PB0-1 | 控制四路电机前进/后退 |
| **循迹传感器** | ADC1 + DMA | PA0-PA7 (假设) | 8 路红外灰度传感器 |
| **调试串口** | USART1 | PA9 (TX), PA10 (RX) | 用于 printf 调试输出 |

## 🔌 接线指引 (Wiring Guide)

### 1. 电机与驱动器 (Motor & Driver)
| 模块 | 信号 | STM32 引脚 | 驱动器对应端口 | 备注 |
| :--- | :--- | :--- | :--- | :--- |
| **左前轮 (M1)** | PWM | **PB6** (TIM4_CH1) | ENA / PWM | 逻辑：IN1=PB12, IN2=PB13 |
| **右前轮 (M2)** | PWM | **PB7** (TIM4_CH2) | ENB / PWM | 逻辑：IN3=PB14, IN4=PB15 |
| **左后轮 (M3)** | PWM | **PB8** (TIM4_CH3) | ENA (Driver2) | 逻辑：IN1=PB0,  IN2=PB1 |
| **右后轮 (M4)** | PWM | **PB9** (TIM4_CH4) | ENB (Driver2) | 逻辑：IN3=PA8,  IN4=PA11 |

### 2. 传感器阵列 (Sensor Array)
项目使用 ADC1 的 8 路通道进行并行采样，引脚如下：
| 传感器编号 | 引脚 | ADC 通道 | 对应数组索引 |
| :--- | :--- | :--- | :--- |
| **S0 (最左)** | **PA0** | ADC1_IN0 | `g_adc_values[0]` |
| **S1** | **PA1** | ADC1_IN1 | `g_adc_values[1]` |
| **S2** | **PA2** | ADC1_IN2 | `g_adc_values[2]` |
| **S3** | **PA3** | ADC1_IN3 | `g_adc_values[3]` |
| **S4** | **PA4** | ADC1_IN4 | `g_adc_values[4]` |
| **S5** | **PA5** | ADC1_IN5 | `g_adc_values[5]` |
| **S6** | **PA6** | ADC1_IN6 | `g_adc_values[6]` |
| **S7 (最右)** | **PA7** | ADC1_IN7 | `g_adc_values[7]` |

> [!NOTE]
> **接线逻辑说明**：电机驱动采用“PWM + 两个方向引脚”模式。`speed > 0` 时，IN1(或IN3) 置高，IN2(或IN4) 置低。若发现电机反向，请调换对应电机的 IN1/IN2 接线，或在代码中修改控制逻辑。

## 📂 目录结构

```text
Car_8c_PID/
├── Core/
│   ├── Inc/
│   │   ├── strategy.h      # 策略层参数与状态定义
│   │   └── main.h          # 硬件底层宏定义
│   └── Src/
│       ├── main.c          # 硬件初始化与底层驱动实现
│       └── strategy.c      # FSM 状态机与 PID 核心逻辑
├── MDK-ARM/                # Keil uVision 5 工程文件
└── Car_8c_PID.ioc          # STM32CubeMX 配置文件
```

### 🧠 状态机详细逻辑 (FSM Details)

| 状态 (State) | 进入触发条件 (Trigger) | 执行动作 (Action) | 退出/切换条件 (Exit) |
| :--- | :--- | :--- | :--- |
| **循迹 (LINE_FOLLOW)** | 默认状态 / 离开特殊区域 | 8路加权 PID 控制输出 | 识别到十字、弯道或丢线 |
| **十字路口 (CROSSROAD)** | 1. 黑线点数 ≥ 5 <br> 2. 左右两侧同时(时间窗)触发 | 固定 `CROSS_SPEED` 高速直行 | 离开黑色区域 或 达到 `CROSS_DELAY_MS` |
| **左/右转弯 (TURN)** | 侧向传感器(S0/S1 或 S6/S7)突变 <br> 且满足黑点数量要求 | 原地差速旋转 (`TURN_SPEED`) | 1. 中间传感器捕捉到黑线 (居中) <br> 2. 达到 `TURN_DELAY_MS` (兜底) |
| **丢线/间隙 (GAP)** | 所有传感器均未检测到黑线 | 保持 `GAP_SPEED` 直线推行 | `GAP` 超时 或 提前重新捕获黑线 |
| **尽头丁字路口 (T-JUNC)** | `GAP` 状态下突遇横向宽线 (黑点 ≥ 5) | 执行默认原地转向功能 | 达到 `TURN_DELAY_MS` 预设极角 |
| **停止 (STOP)** | 程序指令或初始化默认 | 电机停止 | 手动触发切换 |

> [!TIP]
> **时间窗算法**：十字路口的判定不仅看黑点总数，还引用了 `CROSS_DETECT_WINDOW_MS`。如果左侧边缘和右侧边缘在极短时间内先后触发，系统会判定为斜向进入十字路口，从而有效过滤误判。

### 🕒 状态切换与时间策略 (Transition Strategy)

为了保证循迹的鲁棒性，系统在状态判断中引入了“空间（点数）+ 时间（窗口/延时）”的双重校验：

#### 1. 十字路口 (Crossroad)
*   **进入逻辑**：
    *   **空间法**：单次采样中黑线点数 $≥ 5$，判定为横向宽线。
    *   **时间触发法**：当 S0/S1 和 S6/S7 在 `CROSS_DETECT_WINDOW_MS` (默认 250ms) 内先后侦测到黑线，则判定为十字。
*   **退出逻辑**：
    *   当连续黑点数回落至 5 以下，或强制直行计时超过 `CROSS_DELAY_MS` (默认 200ms) 时，切回 `LINE_FOLLOW`。

#### 2. 直角/锐角弯 (Turn)
*   **触发频率**：仅在 `LINE_FOLLOW` 状态下且黑点数 $≥ 3$ 时，根据左右边缘分布判定。
*   **智能退出策略**：为了提高转弯精度，系统不再单纯依赖延时。进入转弯态 100ms 后（避开初始信号干扰），系统会实时扫描传感器。一旦**中间传感器 (S3/S4) 捕获到黑线**，系统判定车头已对准赛道，立即切回 `LINE_FOLLOW` 模式。
*   **锁定与超时**：若在 `TURN_DELAY_MS` (默认 300ms) 内未寻找到赛道，系统会执行强制退出，作为安全保护。此设计兼顾了灵活性与安全性。

#### 3. 丢线跨越与尽头丁字路口 (Gap & T-Junction)
*   **策略描述**：当所有传感器均输出白平（点数为 0）时进入 `STATE_GAP` 盲跑模式。
*   **三重退出机制**：
    1. **遇见丁字路口**：若在盲跑期突然遭遇面积过大的宽线（黑点 ≥ 5），表明撞上了断线尽头的赛道横杆，会立即切入 `STATE_T_JUNCTION` 并根据代码中的 `T_TURN_IS_LEFT` 指定的默认方向执行 90° 旋转。
    2. **重新复位**：若捕捉到零星黑点，证明越过了虚线，立即恢复 `LINE_FOLLOW`。
    3. **保护计时**：若跑满 `GAP_DELAY_MS` (150ms) 后依然两眼一抹黑，强制跳转回循迹模式。

## ⚙️ 核心参数调试

在 `Core/Inc/strategy.h` 中可以针对不同硬件环境进行参数调整：

```c
// 传感器阈值
#define SENSOR_WHITE_BASELINE   1000  // 白纸上的 ADC 读数
#define SENSOR_BLACK_THRESHOLD  3000  // 黑线判定阈值

// PID 参数
#define PID_KP (0.3f)
#define PID_KI (0.001f)
#define PID_KD (0.2f)

// 速度与策略
#define BASE_SPEED 350       // 基础速度
#define TURN_SPEED 400       // 原地转弯速度
#define CROSS_SPEED 450      // 冲过路口速度
#define T_TURN_IS_LEFT 0     // 断线丁字路口默认方向 (1=左, 0=右)
```

### 🛠️ STM32CubeMX 配置详述 (CubeMX Configuration)

若需重新生成工程或了解硬件底层配置，请参考以下设置：

#### 1. 时钟配置 (Clock Configuration)
*   **HSE**: 选择 `Crystal/Ceramic Resonator`。
*   **PLL Source**: 选择 `HSE`。
*   **System Clock (SYSCLK)**: 设置为 `72 MHz`。

#### 2. ADC1 + DMA 设置 (传感器采集)
*   **Channels**: 勾选 `IN0` 到 `IN7` (共 8 路)。
*   **ADC Settings**:
    *   **Scan Conversion Mode**: `Enabled` (多通道扫描)。
    *   **Continuous Conversion Mode**: `Enabled` (循环转换)。
    *   **Number of Conversion**: `8`。
    *   **Rank**: 依次配置 Channel 0-7。
*   **DMA Settings**:
    *   添加 `ADC1` 请求到 `DMA1 Channel 1`。
    *   **Mode**: `Circular` (循环模式)。
    *   **Data Width**: `Word` (与代码中 `uint32_t` 缓冲区匹配)。
    *   **Memory Increment**: `Enabled`。

#### 3. TIM4 设置 (电机 PWM 驱动)
*   **Clock Source**: `Internal Clock`。
*   **Channel 1/2/3/4**: 选择 `PWM Generation CHx`。
*   **Parameter Settings**:
    *   **Prescaler (PSC)**: `71` (得到 1 MHz 计数频率)。
    *   **Counter Period (ARR)**: `999` (得到 **1 kHz** 的 PWM 频率)。
*   **引脚**: PB6, PB7, PB8, PB9。

#### 4. GPIO 输出 (电机 direction 控制)
*   配置以下引脚为 **GPIO_Output**:
    *   **左前轮 (M1)**: PB12, PB13
    *   **右前轮 (M2)**: PB14, PB15
    *   **左后轮 (M3)**: PB0, PB1
    *   **右后轮 (M4)**: PA8, PA11

#### 5. USART1 (串口调试)
*   **Mode**: `Asynchronous`。
*   **Baud Rate**: `115200` Bits/s。

## 📖 使用指南

### 1. 硬件校准 (当前 main.c 模式)
当前 `main.c` 处于**电机测试模式**：
- 烧录后，四个电机将依次转动。
- 请检查电机转向是否均为**前进**。
- 若转向不符，请在 `main.c` 的 `set_motor_speed` 函数中修改对应 GPIO 引脚逻辑。

### 2. 切换至循迹模式
校准完成后，按照 `main.c` 中的注释操作：
1. 取消 `Strategy_Init()` 的注释。
2. 将 `while(1)` 循环体内的测试代码替换为 `Strategy_Run_Main_Loop()`。

### 3. 运行与调试
- 通过串口助手查看 `printf` 输出的状态切换信息 (`FSM: 0 -> 1`)。
- 使用 Keil 的 **Live Watch** 观察 `g_adc_values` 确定传感器的黑白基准。

## 🔨 核心逻辑更新说明

当前版本对初版的代码逻辑进行了重要修复，解决了一些潜在的“人工智障”行为：

1. **PID 极性反转修复**：
   - 修复了原始代码中因加减号写反而导致的“正反馈”现象（检测到偏左时依然向右打方向，导致脱轨）。目前已修正为 `left_speed = BASE_SPEED + correction;`，确保汽车向线正确收敛。
2. **GAP（盲跑）状态提前退出**：
   - 修复了进入间隙盲跑状态（`STATE_GAP`）后必须强行等待 `GAP_DELAY_MS` 的僵硬逻辑。现在**一旦重新捕获到黑线，小车会立即退出盲跑状态**，切换回 `LINE_FOLLOW`，大幅提升了短间隙和轻微丢线时的反应速度。
3. **断头丁字路口增强截获 (`STATE_T_JUNCTION`)**：
   - 针对“线断了，且对面是一堵横线墙 (丁字路口)”的特殊地图进行了逻辑拦截。若处于断线盲跑态并迎头撞上大黑斑，不再将其简单误判为循迹后接十字路口的往前乱冲，而是触发专用判定并根据宏 `T_TURN_IS_LEFT` 完成既定转弯。
4. **解除底层电机倒转锁死 (Critical Bug Fix)**：
   - 修复了 `main.c` 底层中 `set_left_speed` 屏蔽负数（将负数全部 clamp 成 0）的系统级原生 BUG。由于该 BUG 的存在，策略层的 `set_left_speed(-TURN_SPEED)` 原地差速自旋指令其实一直是失效的（变成了单边轮子抱死）。目前已放开允许 `-999` 至 `999` 的倒转输出，FSM 终于可以正确执行原地打圈调头了！
5. **消除状态切换 PID 突变振荡 (PID Windup Fix)**：
   - 修复了从盲跑态（直角弯 / 十字 / 丢线）切回循迹态时，由于积分（I）积累的残余偏差和微分（D）因突变造成的恐怖尖峰扰动。现在状态机一旦切回 `STATE_LINE_FOLLOW`，会在瞬间自动清空上一帧历史偏差（置零 `last_error` / `integral`），保证接缝平滑无比。
6. **转弯锁定逻辑升级 (Turning Optimization)**：
   - 将原有的“固定延时盲转”升级为“**延时兜底 + 传感器寻线触发现**”模式。现在转弯状态会在捕捉到中心线条后提前退出，大幅提升了在不同角度（不仅是90度）和不同速度下的转弯成功率和流畅度。

## ⚠️ 注意事项

- **电源管理**：请确保驱动模块（如 L298N 或 TB6612）与单片机共地。
- **环境光**：红外传感器受自然光干扰较大，建议在室内环境下进行调试。
- **机械安装**：传感器支架应安装牢固且距离底面高度恒定。


