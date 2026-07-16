#ifndef __MOTOR_CONFIG_H__
#define __MOTOR_CONFIG_H__

#include "hc32_ll.h"
#include <stdint.h>

/* PWM和Timer4参数 */
#define APP_TMR4_SRC_CLK_HZ             (200000000UL)  /* TMR4时钟源频率。 */
#define APP_PWM_FREQ_HZ                 (20000UL)      /* 中心对齐PWM载波频率。 */
#define APP_PWM_PERIOD_VALUE            ((uint16_t)((APP_TMR4_SRC_CLK_HZ / (2UL * 2UL * APP_PWM_FREQ_HZ)) - 1UL))
#define MOTOR_PWM_COMPARE_VALUE         ((uint16_t)((APP_PWM_PERIOD_VALUE + 1U) / 2U)) /* 50%占空比。 */
#define MOTOR_PWM_DEAD_TIME_COUNT       (55U)

#define APP_DUTY_MIN                    (0.02f)        /* 占空比下限，避免贴近0%。 */
#define APP_DUTY_MAX                    (0.98f)        /* 占空比上限，避免贴近100%。 */

/* 控制周期 */
#define APP_CONTROL_FREQ_HZ             (20000.0f)
#define APP_CONTROL_TS                  (1.0f / APP_CONTROL_FREQ_HZ)
#define APP_CURRENT_LOOP_DIV            (4U)           /* 每N次ADC中断运行一次电流PI。 */
#define APP_CURRENT_LOOP_TS             (APP_CONTROL_TS * (float)APP_CURRENT_LOOP_DIV)

/* 数学常量 */
#define APP_TWO_PI                      (6.28318530718f)
#define APP_SQRT3_BY_2                  (0.86602540378f)

/* 启动流程 / 状态 */
#define APP_ALLOW_OPEN_LOOP_RUN         (1U)           /* 1：ADC零点校准完成后自动使能电机。 */
#define APP_ALIGN_TICKS                 (20000UL)      /* 转子对齐持续时间，单位为控制周期。 */
#define APP_ALIGN_VD                    (0.015f)       /* 对齐阶段固定d轴电压指令。 */

/* FOC对应状态 */
#define APP_FOC_STATE_ADC_OFFSET        (0U)
#define APP_FOC_STATE_ALIGN             (1U)
#define APP_FOC_STATE_MIT               (2U)
#define APP_FOC_STATE_FAULT             (3U)

/* 编码器 */
#define APP_POLE_PAIRS                  (14.0f)        /* 电机极对数，机械一圈对应的电角度周期数。 */
#define APP_ENCODER_DIR                 (1.0f)         /* 编码器方向反了就改成-1.0f。 */
#define APP_ENCODER_ZERO_COMP_RAD       (0.0f)         /* 对齐后的电角度零点手动补偿。 */
#define APP_ENCODER_PREDICT_ENABLE      (1U)           /* 两次MA732读取之间是否做角度预测。 */
#define APP_ENCODER_PREDICT_MAX_TICK    (400UL)        /* 角度预测最大时间钳位。 */
#define APP_ENCODER_STALE_FAULT_TICK    (4000UL)       /* 编码器长时间未更新则进入故障。 */
#define APP_SPEED_LPF_ALPHA             (0.015f)       /* 机械速度低通滤波系数。 */
#define APP_SPEED_RAW_CLAMP_DEG_S       (600.0f)       /* 单次速度估算限幅，抑制编码器偶发跳变。 */
#define APP_SPEED_RAW_CLAMP_RAD_S       (APP_SPEED_RAW_CLAMP_DEG_S * APP_TWO_PI / 360.0f)
#define APP_SPEED_ACCEL_CLAMP_DEG_S2    (4000.0f)      /* 速度估算最大加速度，抑制速度打印和速度环毛刺。 */
#define APP_SPEED_ACCEL_CLAMP_RAD_S2    (APP_SPEED_ACCEL_CLAMP_DEG_S2 * APP_TWO_PI / 360.0f)

/* Multi-turn position memory.
 * The motor-side single-turn encoder is accumulated into a multi-turn motor
 * angle, then converted to output-side angle by OUTPUT = MOTOR * 7 / 57.
 * Sector 31 is the last 8KB sector of the 256KB internal flash.
 */
#define APP_POSMEM_ENABLE               (DDL_ON)
#define APP_POSMEM_FLASH_SECTOR         (31U)
#define APP_POSMEM_SAVE_INTERVAL_TICKS  (40000UL)
#define APP_POSMEM_SAVE_DELTA_X100      (100)
#define APP_GEAR_RATIO_NUM              (57)
#define APP_GEAR_RATIO_DEN              (7)

/* MIT torque mode.
 * tau_out = Kp * pos_err + Kd * vel_err + tau_ff
 * tau_motor = tau_out / (gear_ratio * efficiency)
 * iq_ref = tau_motor / Kt
 */
#define APP_CONTROL_MODE_MIT            (0U)
#define APP_MIT_LOOP_DIV                (20U) // MIT环运行周期数，20次控制周期运行一次MIT环
#define APP_MIT_KP_NM_PER_RAD           (0.006f)  // MIT环位置环比例系数
#define APP_MIT_KD_NM_PER_RAD_S         (0.00020f) // MIT环速度环比例系数
#define APP_MIT_GEAR_RATIO              ((float)APP_GEAR_RATIO_NUM / (float)APP_GEAR_RATIO_DEN) // 减速比，电机转速/负载转速
#define APP_MIT_TRANSMISSION_EFF        (1.0f) // 传动效率，0~1
#define APP_MIT_MOTOR_KT_NM_PER_A       (0.080f) // 电机力矩常数，单位Nm/A
#define APP_MIT_VEL_TARGET_LIMIT_DEG_S  (300) // MIT环目标速度限幅，单位deg/s
#define APP_MIT_POS_ERR_CLAMP_DEG       (90.0f) // MIT环位置误差限幅，单位deg
#define APP_MIT_POS_ERR_CLAMP_RAD       (APP_MIT_POS_ERR_CLAMP_DEG * APP_TWO_PI / 360.0f) // MIT环位置误差限幅，单位rad
#define APP_MIT_VEL_ERR_CLAMP_DEG_S     (300.0f) // MIT环速度误差限幅，单位deg/s
#define APP_MIT_VEL_ERR_CLAMP_RAD_S     (APP_MIT_VEL_ERR_CLAMP_DEG_S * APP_TWO_PI / 360.0f)
#define APP_MIT_TAU_OUT_LIMIT_NM        (0.020f) // MIT环输出力矩限幅，单位Nm
#define APP_MIT_IQ_LIMIT_A              (0.120f) // MIT环电流指令限幅，单位A
#define APP_MIT_STATIC_FRICTION_NM      (0.003f)
#define APP_MIT_STATIC_FRICTION_BAND_DEG (0.5f)
#define APP_MIT_STATIC_FRICTION_BAND_RAD (APP_MIT_STATIC_FRICTION_BAND_DEG * APP_TWO_PI / 360.0f)
#define APP_MIT_DONE_BAND_DEG           (0.3f) // MIT环完成位置误差带宽，单位deg
#define APP_MIT_DONE_BAND_RAD           (APP_MIT_DONE_BAND_DEG * APP_TWO_PI / 360.0f) // MIT环完成位置误差带宽，单位rad
#define APP_MIT_RESTART_BAND_DEG        (0.8f) // MIT环重新启动位置误差带宽，单位deg
#define APP_MIT_RESTART_BAND_RAD        (APP_MIT_RESTART_BAND_DEG * APP_TWO_PI / 360.0f) // MIT环重新启动位置误差带宽，单位rad
#define APP_MIT_DONE_SPEED_BAND_DEG_S   (3.0f) // MIT环完成速度误差带宽，单位deg/s
#define APP_MIT_DONE_SPEED_BAND_RAD_S   (APP_MIT_DONE_SPEED_BAND_DEG_S * APP_TWO_PI / 360.0f)

/* MA732的SPI-DMA参数 */
#define MA732_CS_PORT                   (GPIO_PORT_B)
#define MA732_CS_PIN                    (GPIO_PIN_05)
#define MA732_SPI_TIMEOUT_LOOP          (200000UL)
#ifndef MA732_SPI_DR_ADDR
#define MA732_SPI_DR_ADDR               ((uint32_t)(&CM_SPI3->DR))
#endif
#define MA732_SPI_TX_DMA_CH             DMA_CH1
#define MA732_SPI_RX_DMA_CH             DMA_CH0
#define MA732_SPI_TX_DMA_EVENT          EVT_SRC_SPI3_SPTI
#define MA732_SPI_RX_DMA_EVENT          EVT_SRC_SPI3_SPRI
#define MA732_CMD_READ_ANGLE            (0x0000U)
#define MA732_RAW14_FROM_FRAME16(x)     ((uint16_t)(((x) >> 2U) & 0x3FFFU))

/* ADC / 电流换算 */
#define APP_ADC_OFFSET_CAL_SAMPLES      (60000U)       /* ADC零点平均采样次数。 */
#define APP_ADC_OVERCURRENT_DELTA       (80)           /* 快速过流保护的原始ADC计数阈值。 */
#define APP_CURRENT_A_PER_COUNT         (0.005f)       /* 每个ADC计数对应的相电流，单位A。 */
#define APP_CURRENT_A_SIGN              (-1.0f)        /* A相电流方向修正。 */
#define APP_CURRENT_B_SIGN              (-1.0f)        /* B相电流方向修正。 */
#define APP_CURRENT_C_SIGN              (-1.0f)        /* C相电流方向修正。 */
#define ADC_VREF                        (3.3f)
#define ADC_ACCURACY                    (1UL << 12U)
#define ADC_CURRENT_DIV                 (0.004f)

/* 电流指令和保护 */
#define APP_CURRENT_ABS_FAULT_A         (0.900f)       /* 相电流过流阈值。 */
#define APP_CURRENT_CNT_FAULT           (180)          /* 去零点后的ADC计数故障阈值。 */
#define APP_CURRENT_SUM_FAULT_A         (0.350f)       /* ia+ib+ic一致性检查阈值。 */
#define APP_ZERO_IQ_OFF_BAND_A          (0.001f)       /* Iq小于该值时认为指令已经回到零。 */

/* 电流环PI */
#define APP_ID_KP                       (1.0f)       /* d轴电流PI比例系数。0.035 */
#define APP_ID_KI                       (0.0f)         /* d轴电流PI积分系数。2.0 */
#define APP_IQ_KP                       (1.0f)       /* q轴电流PI比例系数。 */
#define APP_IQ_KI                       (0.0f)         /* q轴电流PI积分系数。 */
#define APP_CURRENT_PI_I_LIMIT          (0.050f)       /* 电流PI积分限幅。 */
#define APP_CURRENT_V_LIMIT             (0.160f)       /* 单轴PI输出电压限幅。 */
#define APP_CURRENT_V_VECTOR_LIMIT      (0.160f)       /* SVPWM前的dq电压矢量限幅。 */

/* 串口参数 */
#define APP_USART_CMD_BUF_LEN           (48U)
#define APP_USART_RX_RING_SIZE          (128U)
#define APP_USART_TX_RING_SIZE          (1024U)
#define APP_USART_TX_PUMP_BYTES         (24U)

/* CAN MIT command input.
 * ID 0x201, classic CAN, 8 bytes, little-endian:
 * byte0-1: int16 pos_deg_x100
 * byte2-3: int16 vel_deg_s_x10
 * byte4-5: int16 tau_ff_mNm
 * byte6:   command, 0 = MIT target
 * byte7:   sequence/reserved
 */
#define APP_CAN_MIT_CMD_STD_ID          (0x201UL)
#define APP_CAN_MIT_ACK_STD_ID          (0x202UL)
#define APP_CAN_MIT_CMD_DLC             (MCAN_DLC8)
#define APP_CAN_MIT_CMD_CODE            (0U)
#define APP_CAN_RX_FIFO0_NUM            (4U)
#define APP_CAN_TX_FIFO_NUM             (3U)
#define APP_CAN_HEARTBEAT_ENABLE        (DDL_ON)
#define APP_CAN_HEARTBEAT_TICKS         (10000UL)
#define APP_CAN_NOMINAL_PRESCALER       (1U)
/* XTAL=8MHz, HC32 DDL MCAN timing uses TimeSeg1 + TimeSeg2.
 * 500k: prescaler=1, seg1=13, seg2=3, sjw=2
 * 1M:   prescaler=1, seg1=6,  seg2=2, sjw=2
 */
#define APP_CAN_NOMINAL_TIME_SEG1       (6U)
#define APP_CAN_NOMINAL_TIME_SEG2       (2U)
#define APP_CAN_NOMINAL_SJW             (2U)

/* CAN transceiver standby/enable control.
 * Many CAN PHYs use STB high = standby, STB low = normal.
 * Set APP_CAN_PHY_STB_CONTROL to DDL_ON after confirming the board pin.
 */
#define APP_CAN_PHY_STB_CONTROL         (DDL_OFF)
#define APP_CAN_PHY_STB_PORT            (GPIO_PORT_B)
#define APP_CAN_PHY_STB_PIN             (GPIO_PIN_03)
#define APP_CAN_PHY_STB_ENABLE_LEVEL    (0U)
#define APP_CAN_INTERNAL_LOOPBACK       (DDL_OFF)

/* 状态打印 */
#define APP_STATUS_PRINT_INTERVAL_TICKS (10000UL)

#endif /* __MOTOR_CONFIG_H__ */
