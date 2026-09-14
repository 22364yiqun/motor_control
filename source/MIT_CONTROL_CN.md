# MIT 控制修改说明

> 本文记录早期 MIT 控制改造过程。当前默认参数已经改为 57:7 减速结构，实际数值请始终以 `source/motor/config.h` 为准；项目文档入口见仓库根目录 `README.md`。

## 当前控制结构

现在工程已经去掉标准三环外环，不再保留：

```text
位置环 -> 速度环 -> 电流环
```

当前只保留 MIT 控制：

```text
MIT PD 力矩控制 -> 力矩转电流 -> 电流环 -> SVPWM
```

也就是说，外环只做一件事：根据位置误差、速度误差和前馈力矩生成目标力矩，再换算成 `Iq_ref`，直接进入原来的电流环。

## MIT 控制公式

输出端力矩：

```text
tau_out = Kp * pos_err + Kd * vel_err + tau_ff
```

力矩转换层：

```text
tau_motor = tau_out / (N * eta)
Iq_ref = tau_motor / Kt
Id_ref = 0
```

参数含义：

```text
N      = 减速比，直驱时为 1.0
eta    = 传动效率，直驱时为 1.0
Kt     = 电机力矩常数，单位 N*m/A
tau_ff = 输出端前馈力矩，单位 N*m
```

早期版本默认直驱：

```c
#define APP_MIT_GEAR_RATIO              (1.0f)
#define APP_MIT_TRANSMISSION_EFF        (1.0f)
```

当前版本已接入减速比配置，并且命令力矩表示输出轴力矩；更换传动结构时需同步修改减速比和效率参数。

## 串口命令

现在只接受 MIT 命令。

推荐格式：

```text
m位置deg[,目标速度deg/s[,前馈力矩mN*m]]
```

例子：

```text
m0
m10
m30
m90,0,0
m90,0,5
```

含义：

```text
m10       目标位置 10 deg，目标速度 0，前馈力矩 0
m90,0,0   目标位置 90 deg，目标速度 0，前馈力矩 0
m90,0,5   目标位置 90 deg，目标速度 0，输出端前馈力矩 5 mN*m
```

为了方便测试，裸数字也按 MIT 目标位置处理：

```text
10
30
0
```

等价于：

```text
m10
m30
m0
```

旧命令 `p90`、`s20`、速度模式、标准三环位置模式已经不再使用。

## 关键保护

MIT 模式保留以下限幅：

```c
APP_MIT_POS_ERR_CLAMP_DEG
APP_MIT_VEL_ERR_CLAMP_DEG_S
APP_MIT_TAU_OUT_LIMIT_NM
APP_MIT_IQ_LIMIT_A
```

当前默认：

```c
#define APP_MIT_TAU_OUT_LIMIT_NM        (0.020f)
#define APP_MIT_IQ_LIMIT_A              (0.120f)
```

`Kt` 不准确时，不要把 `Kp/Kd` 调太大。

## 测试步骤

上电后等待 `st=2`，然后小角度测试：

```text
m0
m10
m30
m0
```

观察串口打印：

```text
mode=0
posErrX100
mitVelX10
mitTauOutmNm
mitTauMotormNm
refIqMA
iqMA
```

调参建议：

```text
响应太软：小幅增加 APP_MIT_KP_NM_PER_RAD
到位抖动/啸叫：小幅增加 APP_MIT_KD_NM_PER_RAD_S，或降低 APP_MIT_KP_NM_PER_RAD
一给命令就冲太猛：降低 APP_MIT_TAU_OUT_LIMIT_NM 和 APP_MIT_IQ_LIMIT_A
```

## 修改位置

主要文件：

```text
source/motor/config.h    MIT 参数和唯一模式定义
source/motor/control.c   MIT 控制、力矩转电流、电流环入口
source/motor/control.h   调试字段和 MIT 接口声明
source/src/usart.c       只解析 MIT 命令
source/src/main.c        启动提示只显示 MIT
source/motor/encoder.c   状态打印 MIT 字段
```
