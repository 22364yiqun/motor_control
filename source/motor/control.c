#include "control.h"

#include "config.h"
#include "encoder.h"
#include "foc.h"

#include <stddef.h>

static volatile uint8_t m_u8OffsetDone = 0U;        /* ADC零点校准完成标志。 */
static volatile uint8_t m_u8MotorEnable = 0U;       /* 电机运行使能，允许进入对齐/转矩阶段。 */
static volatile uint8_t m_u8FaultOverCurrent = 0U;  /* 软件过流故障锁存标志。 */
static volatile uint8_t m_u8FocState = APP_FOC_STATE_ADC_OFFSET;
static volatile uint8_t m_u8EncoderZeroDone = 0U;   /* 对齐后已记录电角度零点。 */
static volatile uint32_t m_u32ControlTick = 0UL;    /* ADC快速环每进一次加1。 */

static uint16_t m_u16AdcOffsetU = 2048U;            /* U采样通道ADC零点。 */
static uint16_t m_u16AdcOffsetV = 2048U;            /* V采样通道ADC零点。 */
static uint16_t m_u16AdcOffsetW = 2048U;            /* W采样通道ADC零点。 */

static volatile uint16_t m_u16AdcRawU = 0U;
static volatile uint16_t m_u16AdcRawV = 0U;
static volatile uint16_t m_u16AdcRawW = 0U;

static volatile int32_t m_i32IaCnt = 0;             /* A相电流反馈，单位ADC计数。 */
static volatile int32_t m_i32IbCnt = 0;             /* B相电流反馈，单位ADC计数。 */
static volatile int32_t m_i32IcCnt = 0;             /* C相电流反馈，单位ADC计数。 */
static volatile float m_f32IaA = 0.0f;              /* A相电流反馈，单位A。 */
static volatile float m_f32IbA = 0.0f;              /* B相电流反馈，单位A。 */
static volatile float m_f32IcA = 0.0f;              /* C相电流反馈，单位A。 */
static volatile float m_f32ThetaE = 0.0f;           /* 当前控制使用的电角度，单位rad。 */

static uint32_t m_u32OffsetSumU = 0UL;
static uint32_t m_u32OffsetSumV = 0UL;
static uint32_t m_u32OffsetSumW = 0UL;
static uint32_t m_u32OffsetCnt = 0UL;

static volatile int32_t m_i32IqRefFinalMA = 0;      /* 串口输入的Iq目标值，单位mA。 */
static volatile float m_f32IqRefFinalA = 0.0f;      /* 串口Iq目标值换算成A。 */
static volatile float m_f32IqRefA = 0.0f;           /* 经过斜坡后的Iq参考值。 */
static volatile int32_t m_i32SpeedTargetFinalDegS = 0;
static volatile float m_f32SpeedTargetFinalRadS = 0.0f;
static volatile float m_f32SpeedTargetRadS = 0.0f;
static volatile float m_f32SpeedErrorRadS = 0.0f;
static float m_f32SpeedIntegralIqA = 0.0f;
static uint16_t m_u16SpeedLoopDiv = 0U;
static uint32_t m_u32StartupHoldUntilTick = 0UL;

static float Motor_ControlWrapTwoPi(float angle)
{
    while (angle >= APP_TWO_PI) {
        angle -= APP_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += APP_TWO_PI;
    }
    return angle;
}

static float Motor_ControlClamp(float val, float min, float max)
{
    if (val < min) {
        return min;
    }
    if (val > max) {
        return max;
    }
    return val;
}

static void Motor_ControlUpdatePhaseCurrentFeedback(uint16_t raw_u, uint16_t raw_v, uint16_t raw_w)
{
    const int32_t cnt_u = (int32_t)raw_u - (int32_t)m_u16AdcOffsetU;
    const int32_t cnt_v = (int32_t)raw_v - (int32_t)m_u16AdcOffsetV;
    const int32_t cnt_w = (int32_t)raw_w - (int32_t)m_u16AdcOffsetW;

    /* 相序映射：Timer U=物理C相，V=物理B相，W=物理A相。 */
    m_i32IaCnt = cnt_u;
    m_i32IbCnt = cnt_v;
    m_i32IcCnt = cnt_w;

    m_f32IaA = APP_CURRENT_A_SIGN * (float)m_i32IaCnt * APP_CURRENT_A_PER_COUNT;
    m_f32IbA = APP_CURRENT_B_SIGN * (float)m_i32IbCnt * APP_CURRENT_A_PER_COUNT;
    m_f32IcA = APP_CURRENT_C_SIGN * (float)m_i32IcCnt * APP_CURRENT_A_PER_COUNT;
}

/* 快速ADC原始计数保护，优先于电流换算和控制输出。 */
static uint8_t Motor_ControlIsAdcOverCurrentNow(uint16_t raw_u, uint16_t raw_v, uint16_t raw_w)
{
    const int32_t cnt_u = (int32_t)raw_u - (int32_t)m_u16AdcOffsetU;
    const int32_t cnt_v = (int32_t)raw_v - (int32_t)m_u16AdcOffsetV;
    const int32_t cnt_w = (int32_t)raw_w - (int32_t)m_u16AdcOffsetW;

    if (m_u8OffsetDone == 0U) {
        return 0U;
    }

    if ((cnt_u > APP_ADC_OVERCURRENT_DELTA) || (cnt_u < -APP_ADC_OVERCURRENT_DELTA) ||
        (cnt_v > APP_ADC_OVERCURRENT_DELTA) || (cnt_v < -APP_ADC_OVERCURRENT_DELTA) ||
        (cnt_w > APP_ADC_OVERCURRENT_DELTA) || (cnt_w < -APP_ADC_OVERCURRENT_DELTA)) {
        return 1U;
    }

    return 0U;
}

/* 换算后的相电流保护，同时检查ia+ib+ic是否接近0。 */
static uint8_t Motor_ControlIsCurrentTooHigh(void)
{
    const float sum = m_f32IaA + m_f32IbA + m_f32IcA;

    if (m_u8OffsetDone == 0U) {
        return 0U;
    }

    if ((m_i32IaCnt > APP_CURRENT_CNT_FAULT) || (m_i32IaCnt < -APP_CURRENT_CNT_FAULT) ||
        (m_i32IbCnt > APP_CURRENT_CNT_FAULT) || (m_i32IbCnt < -APP_CURRENT_CNT_FAULT) ||
        (m_i32IcCnt > APP_CURRENT_CNT_FAULT) || (m_i32IcCnt < -APP_CURRENT_CNT_FAULT)) {
        return 1U;
    }

    if ((m_f32IaA > APP_CURRENT_ABS_FAULT_A) || (m_f32IaA < -APP_CURRENT_ABS_FAULT_A) ||
        (m_f32IbA > APP_CURRENT_ABS_FAULT_A) || (m_f32IbA < -APP_CURRENT_ABS_FAULT_A) ||
        (m_f32IcA > APP_CURRENT_ABS_FAULT_A) || (m_f32IcA < -APP_CURRENT_ABS_FAULT_A)) {
        return 1U;
    }

    if ((sum > APP_CURRENT_SUM_FAULT_A) || (sum < -APP_CURRENT_SUM_FAULT_A)) {
        return 1U;
    }

    return 0U;
}

/* 对Iq指令做斜坡限制，避免转矩突变。 */
static float Motor_ControlUpdateIqRamp(float current_iq_a, float final_iq_a)
{
    const float max_step = APP_IQ_RAMP_A_PER_TICK;
    const float err = final_iq_a - current_iq_a;

    if (err > max_step) {
        current_iq_a += max_step;
    } else if (err < -max_step) {
        current_iq_a -= max_step;
    } else {
        current_iq_a = final_iq_a;
    }

    return Motor_ControlClamp(current_iq_a, -APP_CURRENT_REF_ABS_MAX_A, APP_CURRENT_REF_ABS_MAX_A);
}

static float Motor_ControlUpdateSpeedRamp(float current_speed_rad_s, float final_speed_rad_s)
{
    const float max_step = APP_SPEED_RAMP_RAD_S2 * APP_CONTROL_TS;
    const float err = final_speed_rad_s - current_speed_rad_s;

    if (err > max_step) {
        current_speed_rad_s += max_step;
    } else if (err < -max_step) {
        current_speed_rad_s -= max_step;
    } else {
        current_speed_rad_s = final_speed_rad_s;
    }

    return current_speed_rad_s;
}

static void Motor_ControlResetSpeedLoop(void)
{
    m_f32SpeedIntegralIqA = 0.0f;
    m_f32SpeedErrorRadS = 0.0f;
    m_u16SpeedLoopDiv = 0U;
    m_u32StartupHoldUntilTick = 0UL;
}

static float Motor_ControlSpeedLoopStep(float target_speed_rad_s, float measured_speed_rad_s)
{
    float err = target_speed_rad_s - measured_speed_rad_s;
    float err_limited;
    float iq_target_a;

    m_f32SpeedErrorRadS = err;
    err_limited = Motor_ControlClamp(err,
                                     -APP_SPEED_ERR_CLAMP_RAD_S,
                                      APP_SPEED_ERR_CLAMP_RAD_S);

    if ((target_speed_rad_s > APP_SPEED_ZERO_BAND_RAD_S) ||
        (target_speed_rad_s < -APP_SPEED_ZERO_BAND_RAD_S)) {
        m_f32SpeedIntegralIqA += APP_SPEED_KI_IQ * err_limited * APP_SPEED_LOOP_TS;
        if (target_speed_rad_s > APP_SPEED_ZERO_BAND_RAD_S) {
            m_f32SpeedIntegralIqA = Motor_ControlClamp(m_f32SpeedIntegralIqA,
                                                       0.0f,
                                                       APP_SPEED_IQ_LIMIT_A);
        } else {
            m_f32SpeedIntegralIqA = Motor_ControlClamp(m_f32SpeedIntegralIqA,
                                                       -APP_SPEED_IQ_LIMIT_A,
                                                       0.0f);
        }
    } else {
        m_f32SpeedIntegralIqA = 0.0f;
    }

    iq_target_a = (APP_SPEED_KP_IQ * err_limited) + m_f32SpeedIntegralIqA;
    if (target_speed_rad_s > APP_SPEED_ZERO_BAND_RAD_S) {
        iq_target_a += APP_RUN_FF_IQ_A;
        if (iq_target_a < 0.0f) {
            iq_target_a = 0.0f;
        }
    } else if (target_speed_rad_s < -APP_SPEED_ZERO_BAND_RAD_S) {
        iq_target_a -= APP_RUN_FF_IQ_A;
        if (iq_target_a > 0.0f) {
            iq_target_a = 0.0f;
        }
    }

    return Motor_ControlClamp(iq_target_a, -APP_SPEED_IQ_LIMIT_A, APP_SPEED_IQ_LIMIT_A);
}

static float Motor_ControlApplyStartupMinIq(float iq_target_a,
                                            float target_speed_rad_s,
                                            float measured_speed_rad_s)
{
    (void)measured_speed_rad_s;

    if (m_u32ControlTick < m_u32StartupHoldUntilTick) {
        if (target_speed_rad_s > APP_SPEED_ZERO_BAND_RAD_S) {
            if (iq_target_a < APP_STARTUP_MIN_IQ_A) {
                iq_target_a = APP_STARTUP_MIN_IQ_A;
            }
        } else if (target_speed_rad_s < -APP_SPEED_ZERO_BAND_RAD_S) {
            if (iq_target_a > -APP_STARTUP_MIN_IQ_A) {
                iq_target_a = -APP_STARTUP_MIN_IQ_A;
            }
        }
    }

    return Motor_ControlClamp(iq_target_a, -APP_SPEED_IQ_LIMIT_A, APP_SPEED_IQ_LIMIT_A);
}

/* 接近速度限制时削减加速方向转矩，制动方向转矩仍然允许。 */
static float Motor_ControlApplyTorqueSpeedLimit(float iq_target_a)
{
    const float speed = Encoder_GetMechSpeedRadS();
    float abs_speed = speed;

    if (abs_speed < 0.0f) {
        abs_speed = -abs_speed;
    }

    if ((speed * iq_target_a) > 0.0f) {
        if (abs_speed >= APP_TORQUE_SPEED_LIMIT_RAD_S) {
            iq_target_a = 0.0f;
        } else if (abs_speed > (APP_TORQUE_SPEED_LIMIT_RAD_S - APP_TORQUE_SPEED_LIMIT_BAND_RAD_S)) {
            const float scale = Motor_ControlClamp((APP_TORQUE_SPEED_LIMIT_RAD_S - abs_speed) /
                                                   APP_TORQUE_SPEED_LIMIT_BAND_RAD_S,
                                                   0.0f,
                                                   1.0f);
            iq_target_a *= scale;
        }
    }

    return iq_target_a;
}

/* 判断Iq指令是否已经足够接近0，可关闭PWM输出。 */
static uint8_t Motor_ControlIsTorqueZeroCommand(void)
{
    if ((m_i32SpeedTargetFinalDegS == 0) &&
        (m_f32SpeedTargetRadS < APP_SPEED_ZERO_BAND_RAD_S) &&
        (m_f32SpeedTargetRadS > -APP_SPEED_ZERO_BAND_RAD_S) &&
        (m_f32IqRefA < APP_ZERO_IQ_OFF_BAND_A) &&
        (m_f32IqRefA > -APP_ZERO_IQ_OFF_BAND_A)) {
        return 1U;
    }

    return 0U;
}

/* 快速环使用的电角度：最新编码器角度 + 可选速度预测。 */
static float Motor_ControlGetElectricalAngle(void)
{
    float theta = m_f32ThetaE;

#if (APP_ENCODER_PREDICT_ENABLE != 0U)
    if (m_u8EncoderZeroDone != 0U) {
        uint32_t dt_tick = m_u32ControlTick - Encoder_GetUpdateTick();
        if (dt_tick > APP_ENCODER_PREDICT_MAX_TICK) {
            dt_tick = APP_ENCODER_PREDICT_MAX_TICK;
        }
        theta += APP_ENCODER_DIR * APP_POLE_PAIRS * Encoder_GetMechSpeedRadS() *
                 ((float)dt_tick * APP_CONTROL_TS);
    }
#endif

    return Motor_ControlWrapTwoPi(theta);
}

void Motor_ControlInit(void)
{
    m_u8OffsetDone = 0U;
    m_u8MotorEnable = 0U;
    m_u8FaultOverCurrent = 0U;
    m_u8FocState = APP_FOC_STATE_ADC_OFFSET;
    m_u8EncoderZeroDone = 0U;
    m_u32ControlTick = 0UL;
    m_u16AdcOffsetU = 2048U;
    m_u16AdcOffsetV = 2048U;
    m_u16AdcOffsetW = 2048U;
    m_u32OffsetSumU = 0UL;
    m_u32OffsetSumV = 0UL;
    m_u32OffsetSumW = 0UL;
    m_u32OffsetCnt = 0UL;
    m_i32IqRefFinalMA = 0;
    m_f32IqRefFinalA = 0.0f;
    m_f32IqRefA = 0.0f;
    m_i32SpeedTargetFinalDegS = 0;
    m_f32SpeedTargetFinalRadS = 0.0f;
    m_f32SpeedTargetRadS = 0.0f;
    Motor_ControlResetSpeedLoop();

    Motor_FOC_Init();
    Motor_FOC_SafeOff();
}

/* ADC触发的快速路径：零点校准 -> 保护 -> 对齐 -> 闭环电流控制。 */
void Motor_ControlFastLoop(uint16_t raw_u, uint16_t raw_v, uint16_t raw_w)
{
    static uint32_t run_tick = 0UL;
    static uint8_t current_loop_div_cnt = 0U;
    float theta;

    m_u32ControlTick++;
    m_u16AdcRawU = raw_u;
    m_u16AdcRawV = raw_v;
    m_u16AdcRawW = raw_w;

    if (m_u8OffsetDone == 0U) {
        Motor_FOC_SafeOff();
        m_u32OffsetSumU += raw_u;
        m_u32OffsetSumV += raw_v;
        m_u32OffsetSumW += raw_w;
        m_u32OffsetCnt++;

        if (m_u32OffsetCnt >= APP_ADC_OFFSET_CAL_SAMPLES) {
            m_u16AdcOffsetU = (uint16_t)(m_u32OffsetSumU / APP_ADC_OFFSET_CAL_SAMPLES);
            m_u16AdcOffsetV = (uint16_t)(m_u32OffsetSumV / APP_ADC_OFFSET_CAL_SAMPLES);
            m_u16AdcOffsetW = (uint16_t)(m_u32OffsetSumW / APP_ADC_OFFSET_CAL_SAMPLES);
            m_u8OffsetDone = 1U;
            m_u8FaultOverCurrent = 0U;
            m_u8FocState = APP_FOC_STATE_ALIGN;
            run_tick = 0UL;
#if (APP_ALLOW_OPEN_LOOP_RUN != 0U)
            m_u8MotorEnable = 1U;
#else
            m_u8MotorEnable = 0U;
#endif
        }
        return;
    }

    Motor_ControlUpdatePhaseCurrentFeedback(raw_u, raw_v, raw_w);

    if ((Motor_ControlIsAdcOverCurrentNow(raw_u, raw_v, raw_w) != 0U) ||
        (Motor_ControlIsCurrentTooHigh() != 0U)) {
        Motor_ControlEnterFault();
        return;
    }

    if ((m_u8MotorEnable == 0U) || (m_u8FaultOverCurrent != 0U)) {
        run_tick = 0UL;
        current_loop_div_cnt = 0U;
        m_f32SpeedTargetRadS = 0.0f;
        Motor_ControlResetSpeedLoop();
        Motor_FOC_Reset();
        Motor_FOC_SafeOff();
        return;
    }

    run_tick++;
    if (run_tick < APP_ALIGN_TICKS) {
        m_u8FocState = APP_FOC_STATE_ALIGN;
        m_u8EncoderZeroDone = 0U;
        Motor_FOC_Reset();
        Motor_FOC_OutputDQ(0.0f, APP_ALIGN_VD, 0.0f);
        return;
    }

    if (m_u8EncoderZeroDone == 0U) {
        const float offset = (APP_ENCODER_DIR * APP_POLE_PAIRS * Encoder_GetMechAngleRad()) +
                             APP_ENCODER_ZERO_COMP_RAD;
        Encoder_SetElectricalZeroOffset(offset);
        m_u8EncoderZeroDone = 1U;
        m_f32ThetaE = 0.0f;
        m_f32IqRefA = 0.0f;
        m_f32SpeedTargetRadS = 0.0f;
        Motor_ControlResetSpeedLoop();
        Motor_FOC_Reset();
    }

    m_u8FocState = APP_FOC_STATE_SPEED;
    theta = Motor_ControlGetElectricalAngle();
    m_f32ThetaE = theta;

    if ((m_u32ControlTick - Encoder_GetUpdateTick()) > APP_ENCODER_STALE_FAULT_TICK) {
        Motor_ControlEnterFault();
        return;
    }

    m_f32SpeedTargetRadS = Motor_ControlUpdateSpeedRamp(m_f32SpeedTargetRadS,
                                                        m_f32SpeedTargetFinalRadS);

    m_u16SpeedLoopDiv++;
    if (m_u16SpeedLoopDiv >= APP_SPEED_LOOP_DIV) {
        const float measured_speed = Encoder_GetMechSpeedRadS();
        float iq_target_a;

        m_u16SpeedLoopDiv = 0U;
        iq_target_a = Motor_ControlSpeedLoopStep(m_f32SpeedTargetRadS, measured_speed);
        iq_target_a = Motor_ControlApplyStartupMinIq(iq_target_a,
                                                     m_f32SpeedTargetRadS,
                                                     measured_speed);
        m_f32IqRefA = Motor_ControlClamp(iq_target_a,
                                         -APP_SPEED_IQ_LIMIT_A,
                                          APP_SPEED_IQ_LIMIT_A);
    }

    current_loop_div_cnt++;
    if (current_loop_div_cnt >= APP_CURRENT_LOOP_DIV) {
        current_loop_div_cnt = 0U;
        Motor_FOC_SetCurrentRef(0.0f, m_f32IqRefA);
    }

    if (Motor_ControlIsTorqueZeroCommand() != 0U) {
        Motor_FOC_Reset();
        Motor_FOC_SafeOff();
    } else {
        Motor_FOC_CurrentLoop(m_f32IaA, m_f32IbA, m_f32IcA, theta);
    }
}

/* 串口指令入口，数值表示产生转矩的q轴电流，单位mA。 */
void Motor_ControlSetIqTargetMA(int32_t iq_target_ma)
{
    const int32_t old_cmd = m_i32IqRefFinalMA;

    if (iq_target_ma > APP_TORQUE_CMD_ABS_MAX_MA) {
        iq_target_ma = APP_TORQUE_CMD_ABS_MAX_MA;
    } else if (iq_target_ma < -APP_TORQUE_CMD_ABS_MAX_MA) {
        iq_target_ma = -APP_TORQUE_CMD_ABS_MAX_MA;
    }

    m_i32IqRefFinalMA = iq_target_ma;
    m_f32IqRefFinalA = Motor_ControlClamp(((float)iq_target_ma) * 0.001f,
                                          -APP_CURRENT_REF_ABS_MAX_A,
                                           APP_CURRENT_REF_ABS_MAX_A);

    if (((old_cmd > 0) && (iq_target_ma <= 0)) ||
        ((old_cmd < 0) && (iq_target_ma >= 0)) ||
        (iq_target_ma == 0)) {
        m_f32IqRefA = 0.0f;
        Motor_FOC_Reset();
    }
}

void Motor_ControlSetSpeedTargetDegS(int32_t speed_target_deg_s)
{
    const int32_t old_speed_target = m_i32SpeedTargetFinalDegS;

    if (speed_target_deg_s > APP_SPEED_CMD_ABS_MAX_DEG_S) {
        speed_target_deg_s = APP_SPEED_CMD_ABS_MAX_DEG_S;
    } else if (speed_target_deg_s < -APP_SPEED_CMD_ABS_MAX_DEG_S) {
        speed_target_deg_s = -APP_SPEED_CMD_ABS_MAX_DEG_S;
    }

    m_i32SpeedTargetFinalDegS = speed_target_deg_s;
    m_f32SpeedTargetFinalRadS = ((float)speed_target_deg_s) * APP_TWO_PI / 360.0f;

    if (speed_target_deg_s == 0) {
        m_f32SpeedTargetRadS = 0.0f;
        m_f32IqRefA = 0.0f;
        Motor_ControlResetSpeedLoop();
        Motor_FOC_Reset();
    } else if (((old_speed_target == 0) ||
                ((old_speed_target > 0) && (speed_target_deg_s < 0)) ||
                ((old_speed_target < 0) && (speed_target_deg_s > 0)))) {
        m_u32StartupHoldUntilTick = m_u32ControlTick + APP_STARTUP_HOLD_TICKS;
    }
}

/* 锁存故障并强制PWM进入安全共模输出。 */
void Motor_ControlEnterFault(void)
{
    m_u8FaultOverCurrent = 1U;
    m_u8MotorEnable = 0U;
    m_u8FocState = APP_FOC_STATE_FAULT;
    Motor_FOC_Reset();
    Motor_FOC_SafeOff();
}

void Motor_ControlSetElectricalAngle(float theta_e)
{
    m_f32ThetaE = Motor_ControlWrapTwoPi(theta_e);
}

uint8_t Motor_ControlIsEncoderZeroDone(void)
{
    return m_u8EncoderZeroDone;
}

uint32_t Motor_ControlGetTick(void)
{
    return m_u32ControlTick;
}

uint8_t Motor_ControlIsOffsetDone(void)
{
    return m_u8OffsetDone;
}

uint8_t Motor_ControlIsFault(void)
{
    return m_u8FaultOverCurrent;
}

uint8_t Motor_ControlIsEnabled(void)
{
    return m_u8MotorEnable;
}

void Motor_ControlGetDebug(motor_control_debug_t *debug)
{
    if (debug == NULL) {
        return;
    }

    debug->foc_state = m_u8FocState;
    debug->motor_enable = m_u8MotorEnable;
    debug->fault = m_u8FaultOverCurrent;
    debug->offset_done = m_u8OffsetDone;
    debug->iq_final_ma = m_i32IqRefFinalMA;
    debug->iq_ref_a = m_f32IqRefA;
    debug->speed_final_deg_s = m_i32SpeedTargetFinalDegS;
    debug->speed_target_rad_s = m_f32SpeedTargetRadS;
    debug->speed_error_rad_s = m_f32SpeedErrorRadS;
    debug->adc_raw_u = m_u16AdcRawU;
    debug->adc_raw_v = m_u16AdcRawV;
    debug->adc_raw_w = m_u16AdcRawW;
    debug->ia_cnt = m_i32IaCnt;
    debug->ib_cnt = m_i32IbCnt;
    debug->ic_cnt = m_i32IcCnt;
    debug->ia_a = m_f32IaA;
    debug->ib_a = m_f32IbA;
    debug->ic_a = m_f32IcA;
}
