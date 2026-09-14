# motor_control

面向一体化关节/伺服执行器的开源电机控制项目。项目以 HC32F4 系列 MCU 为控制核心，当前固件实现三相无刷电机 FOC、MA732 磁编码器采样、MIT 风格的位置/速度/前馈力矩控制、CAN 与串口命令，以及减速器输出端的多圈位置记忆。

> [!WARNING]
> 电机驱动涉及高速旋转、大电流和功率器件。首次运行前请断开负载、使用限流电源并准备急停。默认参数仅对应作者当前样机，不可直接视为其他电机或功率板的安全参数。

## 项目状态

本仓库正在整理为包含电控、机械和调试资料的完整开源项目。

| 模块 | 当前状态 | 位置 |
| --- | --- | --- |
| 电机控制固件 | 已公开，持续整理 | [`source/`](source/) |
| Python CAN 测试工具 | 已公开 | [`can_trans.py`](can_trans.py) |
| 控制原理与参数说明 | 已公开 | [`docs/control/`](docs/control/) |
| CAN/串口协议 | 已整理 | [`docs/protocol/`](docs/protocol/) |
| 电路原理图与 PCB | 待公开 | [`hardware/electronics/`](hardware/electronics/) |
| 第二编码器设计 | 待公开 | [`hardware/electronics/encoder-2/`](hardware/electronics/encoder-2/) |
| 机械结构与工程图 | 待公开 | [`hardware/mechanical/`](hardware/mechanical/) |
| BOM | 待公开 | [`hardware/bom/`](hardware/bom/) |
| 装配与调试 | 框架已建立，内容待补充 | [`docs/assembly/`](docs/assembly/)、[`docs/tuning/`](docs/tuning/) |

当前版本是源码快照，尚未包含完整 IDE 工程、芯片启动文件、链接脚本和 HC32 DDL。补齐这些依赖前，仓库不能作为独立工程直接编译。

## 功能概览

- 三相电流采样、Clarke/Park 变换、dq 电流 PI 和 SVPWM
- 20 kHz 中心对齐 PWM；电流环按 4 分频运行
- MA732 14 位磁编码器，SPI3 + DMA 读取
- MIT 风格控制：位置误差、速度误差和前馈力矩共同生成目标转矩
- 57:7 减速比下的输出轴角度换算与掉电多圈位置记忆
- 经典 CAN 2.0 命令、ACK/心跳与总线诊断
- UART 命令和运行状态输出
- ADC 零点校准、相电流/三相和校验、编码器超时等保护

## 控制结构

```text
目标位置/速度/前馈力矩
          │
          ▼
τout = Kp·位置误差 + Kd·速度误差 + τff
          │  限幅、静摩擦补偿
          ▼
τmotor = τout / (减速比 × 传动效率)
          │
          ▼
Iq_ref = τmotor / Kt，Id_ref = 0
          │
          ▼
dq 电流 PI → 反 Park → SVPWM → 三相逆变器 → 电机
          ▲
          └──── 三相电流 + MA732 转子角度
```

主要参数集中在 [`source/motor/config.h`](source/motor/config.h)。控制原理和已有测试流程见 [`docs/control/mit-control.md`](docs/control/mit-control.md)。

## 仓库结构

```text
motor_control/
├─ source/                     # MCU 固件源码
│  ├─ inc/                    # 外设接口头文件
│  ├─ src/                    # ADC/CAN/DMA/GPIO/SPI/TIM/UART 等
│  └─ motor/                  # FOC、控制器、编码器、位置记忆
├─ tools/                      # 上位机工具说明及后续工具
├─ docs/                       # 控制、协议、装配、调参文档
├─ hardware/                   # 电路、第二编码器、机械与 BOM
├─ can_trans.py               # 达妙 USB-CAN 适配器命令行工具
├─ can_code_control_test.py   # 角度序列测试
└─ vla_can_control_example.py # VLA 输出接入示例
```

详细导航见 [`docs/README.md`](docs/README.md)。

## 快速开始

### 1. 准备固件工程

当前仓库只包含应用源码。请先在 HC32F4 工程中加入 `source/inc/`、`source/src/*.c`、`source/motor/*.c`，以及对应芯片的启动文件、链接脚本、CMSIS、HC32 DDL 和数学库。当前配置中的 BSP 宏为 `BSP_EV_HC32F448_LQFP80`。正式接线表、工程文件与下载步骤将在硬件资料发布后补充。

### 2. 核对参数

通电前至少确认 [`source/motor/config.h`](source/motor/config.h) 中的电机极对数、编码器方向/零点、减速比、传动效率、`Kt`、电流采样比例/方向、过流阈值、PWM/死区、CAN 配置及 Flash 第 31 扇区占用情况。

默认 `APP_ALLOW_OPEN_LOOP_RUN = 1`，ADC 零点校准完成后会自动进入电机启动流程。初次移植建议先改为 `0` 并完成静态检查。

### 3. Python CAN 工具

```bash
python -m pip install -r requirements.txt
python can_trans.py --port COM5 --serial-baud 921600 --pos 10 --vel 0 --tau 0
```

交互或持续发送：

```bash
python can_trans.py --port COM5 --interactive
python can_trans.py --port COM5 --pos 10 --repeat --hz 20
```

默认示例面向达妙 USB-CAN 串口协议，不是通用 `python-can` 驱动。参数单位和帧格式见 [`docs/protocol/can.md`](docs/protocol/can.md)。

### 4. 小角度空载验证

完成接线、限流和方向检查后，等待状态进入 `st=2`，再通过 UART 依次发送：

```text
m0
m10
m30
m0
```

若出现冲击、反向、啸叫、持续大电流或位置跳变，应立即断电并按 [`docs/tuning/README.md`](docs/tuning/README.md) 排查。

## 默认关键参数

以下值来自当前代码，只用于帮助阅读；实际使用以 `config.h` 为准。

| 参数 | 默认值 |
| --- | --- |
| PWM / 快速控制频率 | 20 kHz |
| dq 电流环频率 | 5 kHz |
| MIT 外环频率 | 1 kHz |
| 电机极对数 | 14 |
| 减速比（电机:输出） | 57:7 |
| 编码器 | MA732，14 bit，SPI mode 3 |
| 输出力矩 / `Iq` 限幅 | 0.020 N·m / 0.120 A |
| CAN 命令/应答 ID | `0x201` / `0x202` |
| CAN 标称速率 | 代码注释对应 1 Mbit/s 配置 |

## 参与贡献

欢迎提交 Issue 和 Pull Request。请先阅读 [`CONTRIBUTING.md`](CONTRIBUTING.md)，并说明所用硬件版本、电机/编码器、供电与可复现步骤。涉及功率级或控制参数的修改，请同时说明安全边界和实测条件。

## 开源许可

本项目的开源许可证尚未确定。在正式加入 `LICENSE` 前，仓库内容默认受版权保护；“公开可见”不等于已经授予复制、修改或再分发许可。电路、机械和第三方库可能需要分别标注许可。

## 联系与引用

- 项目主页：<https://github.com/ZJYSII/motor_control>
- 问题与建议：请使用 GitHub Issues

如果本项目对你的研究或开发有帮助，欢迎 Star；引用格式将在首个稳定版本发布时补充。
