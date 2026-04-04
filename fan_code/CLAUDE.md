# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

基于 STC8H1K08 系列 MCU（8051 架构，35MHz）的 2S 无刷直流风扇控制器。对开源参考代码 `Violent-fan-v2.0`（单文件）进行模块化重写。目标器件：STC8H1K08，IRAM 0xFF，XRAM 0x03FF，IROM 0x1FF8。

## 构建系统

项目并行支持两套构建环境：

- **Keil uVision 5**：`fan_fork.uvproj`，目标 `fan_fork`，输出到 `Objects/` 和 `Listings/`
- **EIDE（VSCode 插件）**：`.eide/eide.yml`，工具链 Keil_C51，输出到 `build/fan_fork/`

在 VSCode 中通过 EIDE 侧边栏或 `.vscode/tasks.json` 中定义的任务构建：
- **编译**：`Ctrl+Shift+P` → `Tasks: Run Build Task`（label: `build`）
- **重新编译**：label `rebuild`
- **烧录**：label `flash`（烧录命令需在 `.eide/eide.yml` 的 `uploadConfigMap` 中手动配置）

本项目为嵌入式固件，无宿主机测试套件。

## 代码架构

### 中断驱动时序模型

所有时间敏感逻辑均在 ISR 中执行；主循环轮询 Timer2 设置的 `tick_10ms` 标志：

- **Timer2 ISR**（`interrupt 12`，10ms 周期）：置 `tick_10ms`，调用 `key_scan()`，喂狗
- **Timer0 ISR**（`interrupt 1`）：两阶段换相延时——第一次触发执行下一步换相并启动消磁定时（`phase_time/4`）；第二次触发清除消磁门控
- **Timer1 ISR**（`interrupt 3`）：置溢出标志，用于换相间隔测量
- **比较器 ISR**（`interrupt 21`）：反电动势过零检测，三层滤波（时间窗口、连续采样验证、IIR）；触发 Timer0 换相延时

### 系统状态机（`main.c`）

`status` 字节使用位标志：`STA_SLEEP | STA_WAKING | STA_RUNNING | STA_BATLOW | STA_CHARGING`。状态转移：
- 上电 → `STA_SLEEP`（IDL 低功耗，Timer2 保持运行）
- 长按 → `STA_WAKING` → 检测 `VBUS_SENSE`（P3.0，低有效）→ 进入 `STA_RUNNING` 或 `STA_CHARGING|STA_SLEEP`
- 运行中插入充电线 → 立即 `motor_stop()` → `STA_CHARGING|STA_SLEEP`
- 运行中低压 → 蓝灯闪烁 × 4 → `STA_SLEEP`
- 闲置超时（油门=0 时 30s）或运行超时（5min）→ `STA_SLEEP`

### 电机换相（`motor.c`）

开环强制启动（`motor_start`）后切入闭环 BEMF（`motor_enter_run`）。**关键约束**：`B_start` 必须在 `motor_start()` 执行期间保持为 1，开环结束后才可清零——`motor_step()` 末尾检测 `B_start==1` 时会将 `CMPCR1` 重置为 0x8C（关闭边沿中断），防止 BEMF ISR 干扰开环强制换相阶段。

闭环控制流程：
1. 比较器 ISR 检测过零，IIR 更新 `phase_time`（`0.75*旧值 + 0.25*新值`）
2. 置 `demag_step=1`，Timer0 装入换相提前延时（`wait = phase_time/2 - (phase_time/8)*ADVANCE_LEVEL`，最小 10）
3. Timer0 触发：`step++`，调用 `motor_step()`（通过 `PWMA_ENO` + 低侧 GPIO 实现六步换相），装入消磁定时（`phase_time/4`）
4. Timer0 再次触发：`demag_step=0`，重新允许过零检测

`motor_step()` 每步同时切换 ADC 通道（ch11/12/13 轮换测量悬空相）和比较器边沿方向（偶数步下降沿、奇数步上升沿，由 `step & 0x01` 决定）。

PWM 为 PWMA 3 路互补输出，34kHz（PSCR=3，ARR=255），死区时间 24T。

### 主循环 PWM 斜坡

每个 10ms tick 依次执行：
1. 一阶低通滤波：`pwm = (pwm*9 + 目标值) / 10`
2. 变化率钳位：最大增量 = `PWM_RAMP_SLOW=3`（低转速）或 `PWM_RAMP_FAST=10`（高转速）
3. 失同步恢复：油门减半，重置 `zero_crosses`

油门档位表：`THROTTLE[4] = {0, 50, 80, 150}`（占空比，0–255 范围）。

### ADC / 电池检测（`adc_bat.c`）

- 仅在电机停止时（`!B_run && !B_start`）采样 ADC 通道 10，避免干扰反电动势比较器
- 16 点环形缓冲，去极值均值，与 `BAT_ADC_LOW=770`（~6.4V，分压比 470/(330+470)）比较
- 样本数 &lt; 4 时不输出判定结果（`bat_count < 4` 保护，避免上电瞬间误报低压）

### 按键模块（`key.c`）

6 状态消抖状态机，由 Timer2 ISR 每 10ms 调用一次。产生 `KEY_EVT_SHORT`、`KEY_EVT_LONG`（1s）、`KEY_EVT_DOUBLE`（250ms 窗口）事件。事件由主循环中的 `key_get_event()` 读取，在 `key_handle()` 中消费。

## 硬件引脚对照

| 信号 | 引脚 | 说明 |
|------|------|------|
| PWM AH/AL/BH/BL/CH/CL | P1.0–P1.5 | 推挽，PWMA 互补输出 |
| LED R/G/B | P1.7/P5.4/P1.6 | 共阳，低电平有效 |
| 按键 | P3.7 | 内部上拉使能 |
| VBUS_SENSE | P3.0 | 高阻输入，0=充电线接入 |
| POWER_SWITCH | P3.1 | MOS 栅极，1=开启电源 |
| BEMF（比较器负输入）| P3.6 | 高阻 |
| ADC 电池 | P3.2（通道10）| 高阻 |

## 编译器与内存注意事项

- Keil C51，SMALL RAM 模式，LARGE ROM 模式；优化等级 8（速度优先）
- 时间敏感变量显式声明为 `data` 段：`pwm_val`、`step`、`phase_time` 等
- 访问 PWMA 及扩展 SFR 寄存器前必须先置 `P_SW2 |= 0x80`；`init()` 末尾已保持其使能状态。`clk_init()` 中的 `CLKSEL`/`CLKDIV` 也是 xdata SFR（0xfe00/0xfe01），同样需要先开启 P_SW2
- 时钟校准常量通过 `T35M_ROMADDR`、`VRT35M_ROMADDR` 从固定 ROM 地址读取

## 待开发

- 蜂鸣器模块（`bat_low_beebee()` 在 `main.c` 中为桩函数）
