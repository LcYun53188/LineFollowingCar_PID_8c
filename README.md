# LineFollowingCar_PID_8c 🏎️

基于 STM32F103 的 8 通道高精度 PID 循迹小车项目。

## 📖 项目简介
本项目实现了一个能够处理复杂赛工况（十字路口、直角弯、虚线间隙）的智能循迹小车。核心逻辑结合了 **8 路红外灰度采集**、**加权平均 PID 算法** 以及 **有限状态机 (FSM)**。

## 🚀 核心特性
- **8通道采样**：ADC + DMA 并行采集，高实时性。
- **PID 控制**：位置式 PID 算法，配合加权平均偏差计算。
- **智能状态机**：
  - `LINE_FOLLOW`: 平滑循迹。
  - `CROSSROAD`: 十字路口策略性加速。
  - `TURN`: 原地差速旋转识别与处理。
  - `GAP`: 盲跑惯性循迹与赛道重捕。
- **模块化代码**：底层驱动与业务策略完全分离。

## 🛠️ 硬件清单
- **MCU**: STM32F103C8T6
- **传感器**: 8路红外灰度传感器 (PA0-PA7)
- **驱动**: L298N / TB6612 (TIM4 PWM 输出)
- **底盘**: 四驱/两驱智能小车底盘

## 📂 快速入门
1. **环境**: 使用 Keil uVision 5 打开 `Car_8c_PID/MDK-ARM/Car_8c_PID.uvprojx`。
2. **编译**: 确认硬件接线后，编译并烧录至 STM32。
3. **调试**: 默认处于电机测试模式，根据 `main.c` 注释切换至 `Strategy_Run_Main_Loop()` 即可开始循迹。

## 📁 目录说明
- `Car_8c_PID/Core/Src/strategy.c`: **核心算法逻辑**（状态机与 PID）。
- `Car_8c_PID/Core/Src/main.c`: 硬件驱动与初始化。
- `Car_8c_PID/Car_8c_PID.ioc`: CubeMX 工程文件。

---
*更多详细的技术实现细节、接线图及参数调试指南，请参阅 [Car_8c_PID/README.md](Car_8c_PID/README.md)。*
