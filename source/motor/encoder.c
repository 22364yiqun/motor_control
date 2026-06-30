#include "encoder.h"

#include "config.h"
#include "control.h"
#include "foc.h"
#include "usart.h"

#include "hc32_ll.h"

#include <stddef.h>

static volatile uint8_t m_u8RxDmaDone = 0U;       /* DMA接收完成中断置位。 */
static volatile uint8_t m_u8DmaError = 0U;        /* DMA错误中断置位。 */
static volatile uint16_t m_u16TxDmaBuf = 0U;      /* MA732单帧发送命令/空数据。 */
static volatile uint16_t m_u16RxDmaBuf = 0U;      /* MA732单帧接收数据。 */
static volatile int32_t m_i32DmaStatus = 0;       /* 0=正常，-1=超时，-2=DMA错误。 */

static uint16_t m_u16RawFrame16 = 0U;             /* MA732原始16位数据帧。 */
static uint16_t m_u16RawAngle14 = 0U;             /* 从原始帧提取出的高14位角度。 */
static volatile float m_f32MechAngleRad = 0.0f;   /* 电机轴机械角度，单位rad。 */
static volatile float m_f32MechSpeedRadS = 0.0f;  /* 电机轴机械角速度，单位rad/s。 */
static volatile float m_f32AngleDegrees = 0.0f;   /* 机械角度，单位度，仅用于调试。 */
static volatile float m_f32ElecZeroOffset = 0.0f; /* 对齐后记录的电角度零点偏移。 */
static volatile uint32_t m_u32UpdateTick = 0UL;   /* 最近一次编码器更新对应的控制tick。 */

static float Encoder_WrapTwoPi(float angle)
{
    while (angle >= APP_TWO_PI) {
        angle -= APP_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += APP_TWO_PI;
    }
    return angle;
}

static float Encoder_WrapPi(float angle)
{
    while (angle >= (APP_TWO_PI * 0.5f)) {
        angle -= APP_TWO_PI;
    }
    while (angle < -(APP_TWO_PI * 0.5f)) {
        angle += APP_TWO_PI;
    }
    return angle;
}

/* 读取一帧MA732数据：CS拉低，开启RX DMA，写SPI数据产生SCK。 */
static int32_t MA732_DmaTransReceive16(uint16_t tx_data, uint16_t *rx_data)
{
    stc_dma_init_t dma_init;
    uint32_t timeout = MA732_SPI_TIMEOUT_LOOP;
    uint32_t post_delay;

    if (rx_data == NULL) {
        m_i32DmaStatus = -2;
        return LL_ERR;
    }

    m_u16TxDmaBuf = tx_data;
    m_u16RxDmaBuf = 0U;
    m_u8RxDmaDone = 0U;
    m_u8DmaError = 0U;
    m_i32DmaStatus = 0;

    (void)DMA_ChCmd(CM_DMA1, MA732_SPI_RX_DMA_CH, DISABLE);
    (void)DMA_ChCmd(CM_DMA1, MA732_SPI_TX_DMA_CH, DISABLE);

    DMA_ClearTransCompleteStatus(CM_DMA1, DMA_INT_BTC_CH0);
    DMA_ClearTransCompleteStatus(CM_DMA1, DMA_INT_TRANS_ERR_CH0);
    DMA_ClearTransCompleteStatus(CM_DMA1, DMA_INT_TRANS_ERR_CH1);

    (void)DMA_StructInit(&dma_init);
    dma_init.u32IntEn = DMA_INT_ENABLE;
    dma_init.u32BlockSize = 1UL;
    dma_init.u32TransCount = 1UL;
    dma_init.u32DataWidth = DMA_DATAWIDTH_16BIT;
    dma_init.u32SrcAddr = MA732_SPI_DR_ADDR;
    dma_init.u32DestAddr = (uint32_t)(&m_u16RxDmaBuf);
    dma_init.u32SrcAddrInc = DMA_SRC_ADDR_FIX;
    dma_init.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    (void)DMA_Init(CM_DMA1, MA732_SPI_RX_DMA_CH, &dma_init);

    GPIO_ResetPins(MA732_CS_PORT, MA732_CS_PIN);

    (void)DMA_ChCmd(CM_DMA1, MA732_SPI_RX_DMA_CH, ENABLE);
    CM_SPI3->DR = (uint32_t)m_u16TxDmaBuf;

    while ((m_u8RxDmaDone == 0U) && (m_u8DmaError == 0U) && (timeout > 0UL)) {
        timeout--;
    }

    for (post_delay = 0UL; post_delay < 64UL; post_delay++) {
        __NOP();
    }

    GPIO_SetPins(MA732_CS_PORT, MA732_CS_PIN);
    (void)DMA_ChCmd(CM_DMA1, MA732_SPI_RX_DMA_CH, DISABLE);

    if (m_u8DmaError != 0U) {
        m_i32DmaStatus = -2;
        return LL_ERR;
    }

    if (timeout == 0UL) {
        m_i32DmaStatus = -1;
        return LL_ERR;
    }

    *rx_data = (uint16_t)m_u16RxDmaBuf;
    m_i32DmaStatus = 0;
    return LL_OK;
}

/* 返回最新有效帧；如果SPI-DMA失败，则保留上一次有效值。 */
static uint16_t MA732_ReadRawFrame16(void)
{
    uint16_t rx_frame = 0U;

    if (MA732_DmaTransReceive16(MA732_CMD_READ_ANGLE, &rx_frame) != LL_OK) {
        return m_u16RawFrame16;
    }

    return rx_frame;
}

/* 将MA732原始帧解码成机械角度。 */
static void MA732_UpdateAngle(void)
{
    m_u16RawFrame16 = MA732_ReadRawFrame16();
    m_u16RawAngle14 = MA732_RAW14_FROM_FRAME16(m_u16RawFrame16);
    m_f32MechAngleRad = (float)m_u16RawAngle14 * APP_TWO_PI / 16384.0f;
    m_f32AngleDegrees = (float)m_u16RawAngle14 * 360.0f / 16384.0f;
}

/* 主循环中的编码器任务：读角度、算速度、更新电角度。 */
void Encoder_Task(void)
{
    static uint8_t speed_init = 0U;
    static float last_mech_angle_rad = 0.0f;
    static uint32_t last_control_tick = 0UL;

    const uint32_t now_tick = Motor_ControlGetTick();
    float now_mech_angle_rad;
    uint32_t dt_tick;

    MA732_UpdateAngle();

    if (m_i32DmaStatus != 0) {
        if (Motor_ControlIsEncoderZeroDone() != 0U) {
            Motor_ControlEnterFault();
        }
        return;
    }

    now_mech_angle_rad = m_f32MechAngleRad;

    if (speed_init == 0U) {
        last_mech_angle_rad = now_mech_angle_rad;
        last_control_tick = now_tick;
        m_f32MechSpeedRadS = 0.0f;
        speed_init = 1U;
    } else {
        dt_tick = now_tick - last_control_tick;
        if (dt_tick > 0UL) {
            const float delta_angle = Encoder_WrapPi(now_mech_angle_rad - last_mech_angle_rad);
            const float dt = (float)dt_tick * APP_CONTROL_TS;
            const float max_speed_step = APP_SPEED_ACCEL_CLAMP_RAD_S2 * dt;
            float speed_raw = delta_angle / dt;
            float speed_err;

            if (speed_raw > APP_SPEED_RAW_CLAMP_RAD_S) {
                speed_raw = APP_SPEED_RAW_CLAMP_RAD_S;
            } else if (speed_raw < -APP_SPEED_RAW_CLAMP_RAD_S) {
                speed_raw = -APP_SPEED_RAW_CLAMP_RAD_S;
            }

            speed_err = speed_raw - m_f32MechSpeedRadS;
            if (speed_err > max_speed_step) {
                speed_raw = m_f32MechSpeedRadS + max_speed_step;
            } else if (speed_err < -max_speed_step) {
                speed_raw = m_f32MechSpeedRadS - max_speed_step;
            }

            m_f32MechSpeedRadS = (APP_SPEED_LPF_ALPHA * speed_raw) +
                                 ((1.0f - APP_SPEED_LPF_ALPHA) * m_f32MechSpeedRadS);
            last_mech_angle_rad = now_mech_angle_rad;
            last_control_tick = now_tick;
        }
    }

    if (Motor_ControlIsEncoderZeroDone() != 0U) {
        Motor_ControlSetElectricalAngle(Encoder_GetElectricalAngle());
    }
    m_u32UpdateTick = now_tick;
}

/* 低频调试打印任务，不要在ADC/DMA/USART中断里调用。 */
void Encoder_StatusPrintTask(void)
{
    static uint32_t last_print_tick = 0UL;
    const uint32_t now_tick = Motor_ControlGetTick();
    motor_control_debug_t control_debug;
    motor_foc_debug_t foc_debug;

    char line[448];
    char raw14[8];
    char pos_x100[12];
    char vel_x10[12];
    char state[4];
    char cmd_spd_x10[12];
    char ref_spd_x10[12];
    char spd_err_x10[12];
    char cmd_iq_ma[12];
    char ref_iq_ma[12];
    char id_ma[12];
    char iq_ma[12];
    char vd_x10000[12];
    char vq_x10000[12];
    char ia_raw[8];
    char ib_raw[8];
    char ic_raw[8];
    char ia_cnt[12];
    char ib_cnt[12];
    char ic_cnt[12];
    char ia_ma[12];
    char ib_ma[12];
    char ic_ma[12];
    char eia_ma[12];
    char eib_ma[12];
    char eic_ma[12];
    uint32_t idx = 0UL;

    if ((now_tick - last_print_tick) < APP_STATUS_PRINT_INTERVAL_TICKS) {
        return;
    }
    last_print_tick = now_tick;

    Motor_ControlGetDebug(&control_debug);
    Motor_FOC_GetDebug(&foc_debug);

    (void)App_U32ToDecStr(raw14, (uint32_t)m_u16RawAngle14);
    (void)App_U32ToDecStr(pos_x100, ((uint32_t)m_u16RawAngle14 * 36000UL) / 16384UL);
    (void)App_I32ToDecStr(vel_x10, (int32_t)(m_f32MechSpeedRadS * 572.957795f));
    (void)App_U32ToDecStr(state, (uint32_t)control_debug.foc_state);
    (void)App_I32ToDecStr(cmd_spd_x10, control_debug.speed_final_deg_s * 10);
    (void)App_I32ToDecStr(ref_spd_x10, (int32_t)(control_debug.speed_target_rad_s * 572.957795f));
    (void)App_I32ToDecStr(spd_err_x10, (int32_t)(control_debug.speed_error_rad_s * 572.957795f));
    (void)App_I32ToDecStr(cmd_iq_ma, control_debug.iq_final_ma);
    (void)App_I32ToDecStr(ref_iq_ma, (int32_t)(foc_debug.iq_ref_a * 1000.0f));
    (void)App_I32ToDecStr(id_ma, (int32_t)(foc_debug.id_a * 1000.0f));
    (void)App_I32ToDecStr(iq_ma, (int32_t)(foc_debug.iq_a * 1000.0f));
    (void)App_I32ToDecStr(vd_x10000, (int32_t)(foc_debug.vd_cmd * 10000.0f));
    (void)App_I32ToDecStr(vq_x10000, (int32_t)(foc_debug.vq_cmd * 10000.0f));
    (void)App_U32ToDecStr(ia_raw, (uint32_t)control_debug.adc_raw_u);
    (void)App_U32ToDecStr(ib_raw, (uint32_t)control_debug.adc_raw_v);
    (void)App_U32ToDecStr(ic_raw, (uint32_t)control_debug.adc_raw_w);
    (void)App_I32ToDecStr(ia_cnt, control_debug.ia_cnt);
    (void)App_I32ToDecStr(ib_cnt, control_debug.ib_cnt);
    (void)App_I32ToDecStr(ic_cnt, control_debug.ic_cnt);
    (void)App_I32ToDecStr(ia_ma, (int32_t)(control_debug.ia_a * 1000.0f));
    (void)App_I32ToDecStr(ib_ma, (int32_t)(control_debug.ib_a * 1000.0f));
    (void)App_I32ToDecStr(ic_ma, (int32_t)(control_debug.ic_a * 1000.0f));
    (void)App_I32ToDecStr(eia_ma, (int32_t)(foc_debug.ia_ref_a * 1000.0f));
    (void)App_I32ToDecStr(eib_ma, (int32_t)(foc_debug.ib_ref_a * 1000.0f));
    (void)App_I32ToDecStr(eic_ma, (int32_t)(foc_debug.ic_ref_a * 1000.0f));

#define APPEND_STR(s) do { const char *p = (s); while (*p != '\0') { line[idx++] = *p++; } } while (0)
    APPEND_STR("st=");
    APPEND_STR(state);
    APPEND_STR(", spi=");
    if (m_i32DmaStatus == 0) {
        APPEND_STR("OK");
    } else if (m_i32DmaStatus == -1) {
        APPEND_STR("TIMEOUT");
    } else {
        APPEND_STR("DMA_ERR");
    }
    APPEND_STR(", raw=");
    APPEND_STR(raw14);
    APPEND_STR(", posX100=");
    APPEND_STR(pos_x100);
    APPEND_STR(", velX10=");
    APPEND_STR(vel_x10);
    APPEND_STR(", cmdSpdX10=");
    APPEND_STR(cmd_spd_x10);
    APPEND_STR(", refSpdX10=");
    APPEND_STR(ref_spd_x10);
    APPEND_STR(", spdErrX10=");
    APPEND_STR(spd_err_x10);
    APPEND_STR(", cmdIqMA=");
    APPEND_STR(cmd_iq_ma);
    APPEND_STR(", refIqMA=");
    APPEND_STR(ref_iq_ma);
    APPEND_STR(", idMA=");
    APPEND_STR(id_ma);
    APPEND_STR(", iqMA=");
    APPEND_STR(iq_ma);
    APPEND_STR(", vdX10000=");
    APPEND_STR(vd_x10000);
    APPEND_STR(", vqX10000=");
    APPEND_STR(vq_x10000);
    APPEND_STR(", iaRaw=");
    APPEND_STR(ia_raw);
    APPEND_STR(", ibRaw=");
    APPEND_STR(ib_raw);
    APPEND_STR(", icRaw=");
    APPEND_STR(ic_raw);
    APPEND_STR(", iaCnt=");
    APPEND_STR(ia_cnt);
    APPEND_STR(", ibCnt=");
    APPEND_STR(ib_cnt);
    APPEND_STR(", icCnt=");
    APPEND_STR(ic_cnt);
    APPEND_STR(", iaMA=");
    APPEND_STR(ia_ma);
    APPEND_STR(", ibMA=");
    APPEND_STR(ib_ma);
    APPEND_STR(", icMA=");
    APPEND_STR(ic_ma);
    APPEND_STR(", EiaMA=");
    APPEND_STR(eia_ma);
    APPEND_STR(", EibMA=");
    APPEND_STR(eib_ma);
    APPEND_STR(", EicMA=");
    APPEND_STR(eic_ma);
    APPEND_STR("\r\n");
    line[idx] = '\0';
#undef APPEND_STR

    App_USART1SendString(line);
}

/* 保存转子对齐阶段测得的电角度零点。 */
void Encoder_SetElectricalZeroOffset(float offset)
{
    m_f32ElecZeroOffset = Encoder_WrapTwoPi(offset);
}

/* 将编码器机械角度转换为FOC使用的电角度。 */
float Encoder_GetElectricalAngle(void)
{
    const float theta_e = (APP_ENCODER_DIR * APP_POLE_PAIRS * m_f32MechAngleRad) -
                          m_f32ElecZeroOffset;

    return Encoder_WrapTwoPi(theta_e);
}

float Encoder_GetMechAngleRad(void)
{
    return m_f32MechAngleRad;
}

float Encoder_GetMechSpeedRadS(void)
{
    return m_f32MechSpeedRadS;
}

uint32_t Encoder_GetUpdateTick(void)
{
    return m_u32UpdateTick;
}

int32_t Encoder_GetDmaStatus(void)
{
    return m_i32DmaStatus;
}

void Encoder_GetDebug(encoder_debug_t *debug)
{
    if (debug == NULL) {
        return;
    }

    debug->raw_frame16 = m_u16RawFrame16;
    debug->raw_angle14 = m_u16RawAngle14;
    debug->mech_angle_rad = m_f32MechAngleRad;
    debug->mech_speed_rad_s = m_f32MechSpeedRadS;
    debug->angle_degrees = m_f32AngleDegrees;
    debug->dma_status = m_i32DmaStatus;
    debug->update_tick = m_u32UpdateTick;
}

void MA732_DMA_RxDoneCallback(void)
{
    m_u8RxDmaDone = 1U;
}

void MA732_DMA_ErrorCallback(void)
{
    m_u8DmaError = 1U;
}
