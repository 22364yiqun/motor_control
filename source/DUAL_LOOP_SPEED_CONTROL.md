# 速度-电流双环控制修改说明

## 控制结构

当前工程已经从“串口直接给 Iq 电流”改成了“速度外环 + 电流内环”的串级控制：

```text
UART 输入目标速度 deg/s
    -> 速度目标斜坡
    -> 速度 PI，输出 Iq 参考电流
    -> 原来的 Id/Iq 电流 PI
    -> Vd/Vq
    -> SVPWM
```

这里速度环没有直接输出 `Vq`。速度环输出的是 `Iq_ref`，所以之前已经调通的电流环仍然作为内环使用。

## 修改文件

### `source/motor/config.h`

新增了速度环参数：

- `APP_SPEED_CMD_ABS_MAX_DEG_S`：串口速度命令最大值，单位 `deg/s`。
- `APP_SPEED_RAMP_DEG_S2`：目标速度斜坡，避免速度目标突变。
- `APP_SPEED_LOOP_DIV`：速度环分频。当前值为 `20`，即 `20kHz / 20 = 1kHz`。
- `APP_SPEED_KP_IQ`、`APP_SPEED_KI_IQ`：速度 PI 参数，输出单位是 A。
- `APP_SPEED_IQ_LIMIT_A`：速度环输出的 Iq 限幅。
- `APP_STARTUP_MIN_IQ_A`：低速未起动时的 Iq 保底值，用来克服静摩擦和齿槽力。
- `APP_STARTUP_EXIT_SPEED_DEG_S`：超过该速度后退出起动保底，让速度 PI 自己调节。

当前初始参数：

```c
#define APP_SPEED_KP_IQ                 (0.0015f)
#define APP_SPEED_KI_IQ                 (0.006f)
#define APP_SPEED_IQ_LIMIT_A            (0.080f)
#define APP_STARTUP_MIN_IQ_A            (0.055f)
#define APP_STARTUP_EXIT_SPEED_DEG_S    (20.0f)
```

这些值比较保守，适合先验证双环能不能稳定工作。

### `source/motor/control.c`

新增了速度环状态量：

- 串口最终目标速度，单位 `deg/s`
- 斜坡后的目标速度，单位 `rad/s`
- 速度误差，单位 `rad/s`
- 速度 PI 积分项
- 速度环分频计数器

新增函数：

- `Motor_ControlUpdateSpeedRamp()`：目标速度斜坡。
- `Motor_ControlResetSpeedLoop()`：复位速度环积分和误差。
- `Motor_ControlSpeedLoopStep()`：速度 PI，一次计算输出 `Iq_ref`。
- `Motor_ControlApplyStartupMinIq()`：低速起动 Iq 保底。
- `Motor_ControlSetSpeedTargetDegS()`：串口速度命令入口。

`Motor_ControlFastLoop()` 的主要逻辑改为：

1. 对目标速度做斜坡；
2. 每 `APP_SPEED_LOOP_DIV` 个 fast loop 跑一次速度 PI；
3. 速度 PI 输出 `Iq_ref`；
4. 低速未起动时使用 Iq 保底；
5. 对最终 `Iq_ref` 做限幅和速度保护；
6. 调用 `Motor_FOC_SetCurrentRef(0, Iq_ref)`；
7. 继续使用原来的 `Motor_FOC_CurrentLoop()` 做电流闭环。

原来的 `Motor_ControlSetIqTargetMA()` 仍然保留，方便以后回退到扭矩电流模式调试。

### `source/motor/control.h`

新增速度命令接口：

```c
void Motor_ControlSetSpeedTargetDegS(int32_t speed_target_deg_s);
```

新增调试字段：

- `speed_final_deg_s`
- `speed_target_rad_s`
- `speed_error_rad_s`

### `source/src/usart.c`

串口输入含义已经改成目标机械速度，单位 `deg/s`。

示例：

```text
20
0
-20
50
```

串口回显从：

```text
CMD Iq=...
```

改成：

```text
CMD speed=... deg/s
```

### `source/motor/encoder.c`

状态打印新增速度环字段：

- `cmdSpdX10`：最终速度命令，单位 `deg/s * 10`
- `refSpdX10`：斜坡后的目标速度，单位 `deg/s * 10`
- `spdErrX10`：速度误差，单位 `deg/s * 10`

原来的电流环字段仍然保留：

- `refIqMA`
- `idMA`
- `iqMA`
- `vdX10000`
- `vqX10000`

调速度环时重点看：

```text
velX10, cmdSpdX10, refSpdX10, spdErrX10, refIqMA, iqMA, vqX10000
```

### `source/src/main.c`

启动提示改为：

```text
UART-IRQ speed-current dual-loop FOC start
```

## 首次测试步骤

建议先从小速度开始，不要一上来给大速度：

```text
20
0
-20
0
50
0
-50
0
100
0
```

观察字段：

```text
velX10
cmdSpdX10
refSpdX10
spdErrX10
refIqMA
iqMA
vqX10000
```

期望现象：

- `refSpdX10` 应该平滑接近 `cmdSpdX10`，不是瞬间跳变。
- 起动瞬间 `refIqMA` 可能会到 `55mA` 左右，这是起动保底电流在工作。
- 电机转起来后，`refIqMA` 应该逐渐下降。
- 在 `vqX10000` 没有接近饱和时，`iqMA` 应该能跟住 `refIqMA`。
- 低速测试时，`vqX10000` 不应该长期贴近 `±500`。

## 调参方向

如果电机起不来：

- 先适当增大 `APP_STARTUP_MIN_IQ_A`，例如从 `0.055f` 改到 `0.060f`；
- 或者略微增大 `APP_SPEED_IQ_LIMIT_A`，但初期建议不要超过 `0.100f`。

如果速度来回震荡：

- 先减小 `APP_SPEED_KP_IQ`；
- 再减小 `APP_SPEED_KI_IQ`；
- 如果震荡主要发生在刚起动时，减小 `APP_STARTUP_MIN_IQ_A` 或减小保底退出速度 `APP_STARTUP_EXIT_SPEED_DEG_S`。

如果响应太慢但很稳定：

- 先小幅增大 `APP_SPEED_KP_IQ`；
- 比例响应稳定后，再小幅增大 `APP_SPEED_KI_IQ`。

## 注意事项

这次修改实现的是速度-电流双环，还不是 MIT 模式。

后续 MIT 模式可以在当前电流环基础上继续增加一个外环：

```text
Kp * 位置误差 + Kd * 速度误差 + 力矩前馈
    -> Iq_ref
    -> 电流环
```

也就是说，当前双环是后续实现 MIT 模式的基础。
