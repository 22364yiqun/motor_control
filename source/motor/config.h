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
#define APP_FOC_STATE_TORQUE            (2U)
#define APP_FOC_STATE_FAULT             (3U)

/* 编码器 */
#define APP_POLE_PAIRS                  (14.0f)        /* 电机极对数，机械一圈对应的电角度周期数。 */
#define APP_ENCODER_DIR                 (1.0f)         /* 编码器方向反了就改成-1.0f。 */
#define APP_ENCODER_ZERO_COMP_RAD       (0.0f)         /* 对齐后的电角度零点手动补偿。 */
#define APP_ENCODER_PREDICT_ENABLE      (1U)           /* 两次MA732读取之间是否做角度预测。 */
#define APP_ENCODER_PREDICT_MAX_TICK    (400UL)        /* 角度预测最大时间钳位。 */
#define APP_ENCODER_STALE_FAULT_TICK    (4000UL)       /* 编码器长时间未更新则进入故障。 */
#define APP_SPEED_LPF_ALPHA             (0.02f)        /* 机械速度低通滤波系数。 */

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
#define APP_TORQUE_CMD_ABS_MAX_MA       (480)          /* 串口Iq指令最大值，单位mA。 */
#define APP_CURRENT_REF_ABS_MAX_A       (0.480f)       /* Id/Iq参考电流最大值。 */
#define APP_CURRENT_ABS_FAULT_A         (0.900f)       /* 相电流过流阈值。 */
#define APP_CURRENT_CNT_FAULT           (180)          /* 去零点后的ADC计数故障阈值。 */
#define APP_CURRENT_SUM_FAULT_A         (0.350f)       /* ia+ib+ic一致性检查阈值。 */
#define APP_ZERO_IQ_OFF_BAND_A          (0.001f)       /* Iq小于该值时认为指令已经回到零。 */
#define APP_IQ_RAMP_A_PER_S             (0.240f)       /* Iq指令斜坡变化率。 */
#define APP_IQ_RAMP_A_PER_TICK          (APP_IQ_RAMP_A_PER_S * APP_CONTROL_TS)

/* 转矩模式速度限制 */
#define APP_TORQUE_SPEED_LIMIT_DEG_S       (500.0f) // 
#define APP_TORQUE_SPEED_LIMIT_RAD_S       (APP_TORQUE_SPEED_LIMIT_DEG_S * APP_TWO_PI / 360.0f)
#define APP_TORQUE_SPEED_LIMIT_BAND_DEG_S  (200.0f) //
#define APP_TORQUE_SPEED_LIMIT_BAND_RAD_S  (APP_TORQUE_SPEED_LIMIT_BAND_DEG_S * APP_TWO_PI / 360.0f)

/* 电流环PI */
#define APP_ID_KP                       (0.035f)       /* d轴电流PI比例系数。 */
#define APP_ID_KI                       (2.0f)         /* d轴电流PI积分系数。 */
#define APP_IQ_KP                       (0.035f)       /* q轴电流PI比例系数。 */
#define APP_IQ_KI                       (2.0f)         /* q轴电流PI积分系数。 */
#define APP_CURRENT_PI_I_LIMIT          (0.050f)       /* 电流PI积分限幅。 */
#define APP_CURRENT_V_LIMIT             (0.160f)       /* 单轴PI输出电压限幅。 */
#define APP_CURRENT_V_VECTOR_LIMIT      (0.160f)       /* SVPWM前的dq电压矢量限幅。 */

/* 串口参数 */
#define APP_USART_CMD_BUF_LEN           (16U)
#define APP_USART_RX_RING_SIZE          (128U)
#define APP_USART_TX_RING_SIZE          (1024U)
#define APP_USART_TX_PUMP_BYTES         (24U)

/* 状态打印 */
#define APP_STATUS_PRINT_INTERVAL_TICKS (10000UL)

#endif /* __MOTOR_CONFIG_H__ */
