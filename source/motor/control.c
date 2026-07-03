#include "control.h"

#include "config.h"
#include "encoder.h"
#include "foc.h"

#include <stddef.h>

static volatile uint8_t m_u8OffsetDone = 0U;
static volatile uint8_t m_u8MotorEnable = 0U;
static volatile uint8_t m_u8FaultOverCurrent = 0U;
static volatile uint8_t m_u8FocState = APP_FOC_STATE_ADC_OFFSET;
static volatile uint8_t m_u8EncoderZeroDone = 0U;
static volatile uint32_t m_u32ControlTick = 0UL;

static uint16_t m_u16AdcOffsetU = 2048U;
static uint16_t m_u16AdcOffsetV = 2048U;
static uint16_t m_u16AdcOffsetW = 2048U;

static volatile uint16_t m_u16AdcRawU = 0U;
static volatile uint16_t m_u16AdcRawV = 0U;
static volatile uint16_t m_u16AdcRawW = 0U;

static volatile int32_t m_i32IaCnt = 0;
static volatile int32_t m_i32IbCnt = 0;
static volatile int32_t m_i32IcCnt = 0;
static volatile float m_f32IaA = 0.0f;
static volatile float m_f32IbA = 0.0f;
static volatile float m_f32IcA = 0.0f;
static volatile float m_f32ThetaE = 0.0f;

static uint32_t m_u32OffsetSumU = 0UL;
static uint32_t m_u32OffsetSumV = 0UL;
static uint32_t m_u32OffsetSumW = 0UL;
static uint32_t m_u32OffsetCnt = 0UL;

static volatile float m_f32IqRefA = 0.0f;
static volatile uint8_t m_u8ControlMode = APP_CONTROL_MODE_MIT;
static volatile int32_t m_i32PositionTargetDeg = 0;
static volatile float m_f32PositionTargetRad = 0.0f;
static volatile float m_f32PositionErrorRad = 0.0f;
static volatile int32_t m_i32MitVelocityTargetDegS = 0;
static volatile float m_f32MitVelocityTargetRadS = 0.0f;
static volatile float m_f32MitVelocityErrorRadS = 0.0f;
static volatile float m_f32MitTauFfNm = 0.0f;
static volatile float m_f32MitTauStaticNm = 0.0f;
static volatile float m_f32MitTauOutRefNm = 0.0f;
static volatile float m_f32MitTauMotorRefNm = 0.0f;
static volatile uint8_t m_u8MitDone = 0U;
static uint16_t m_u16MitLoopDiv = 0U;

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

static float Motor_ControlWrapPi(float angle)
{
    while (angle >= (APP_TWO_PI * 0.5f)) {
        angle -= APP_TWO_PI;
    }
    while (angle < -(APP_TWO_PI * 0.5f)) {
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

    m_i32IaCnt = cnt_u;
    m_i32IbCnt = cnt_v;
    m_i32IcCnt = cnt_w;

    m_f32IaA = APP_CURRENT_A_SIGN * (float)m_i32IaCnt * APP_CURRENT_A_PER_COUNT;
    m_f32IbA = APP_CURRENT_B_SIGN * (float)m_i32IbCnt * APP_CURRENT_A_PER_COUNT;
    m_f32IcA = APP_CURRENT_C_SIGN * (float)m_i32IcCnt * APP_CURRENT_A_PER_COUNT;
}

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

/* MIT外环：位置/速度PD生成输出端力矩，再通过减速比、效率和Kt换算成Iq。 */
static float Motor_ControlMitLoopStep(void)
{
    const float measured_pos_rad = Encoder_GetMechAngleRad();
    const float measured_vel_rad_s = Encoder_GetMechSpeedRadS();
    const float torque_den = APP_MIT_GEAR_RATIO * APP_MIT_TRANSMISSION_EFF;
    float pos_err_rad = Motor_ControlWrapPi(m_f32PositionTargetRad - measured_pos_rad);
    float vel_err_rad_s = m_f32MitVelocityTargetRadS - measured_vel_rad_s;
    float abs_pos_err_rad = pos_err_rad;
    float tau_static_nm = 0.0f;
    float tau_out_nm;
    float tau_motor_nm;
    float iq_ref_a;

    if (abs_pos_err_rad < 0.0f) {
        abs_pos_err_rad = -abs_pos_err_rad;
    }

    m_f32PositionErrorRad = pos_err_rad;
    m_f32MitVelocityErrorRadS = vel_err_rad_s;

    if (m_u8MitDone != 0U) {
        if (abs_pos_err_rad > APP_MIT_RESTART_BAND_RAD) {
            m_u8MitDone = 0U;
        }
    } else if ((abs_pos_err_rad < APP_MIT_DONE_BAND_RAD) &&
               (measured_vel_rad_s < APP_MIT_DONE_SPEED_BAND_RAD_S) &&
               (measured_vel_rad_s > -APP_MIT_DONE_SPEED_BAND_RAD_S)) {
        m_u8MitDone = 1U;
    }

    if (m_u8MitDone != 0U) {
        tau_out_nm = 0.0f;
    } else {
        pos_err_rad = Motor_ControlClamp(pos_err_rad,
                                         -APP_MIT_POS_ERR_CLAMP_RAD,
                                          APP_MIT_POS_ERR_CLAMP_RAD);
        vel_err_rad_s = Motor_ControlClamp(vel_err_rad_s,
                                           -APP_MIT_VEL_ERR_CLAMP_RAD_S,
                                            APP_MIT_VEL_ERR_CLAMP_RAD_S);
        if (pos_err_rad > APP_MIT_STATIC_FRICTION_BAND_RAD) {
            tau_static_nm = APP_MIT_STATIC_FRICTION_NM;
        } else if (pos_err_rad < -APP_MIT_STATIC_FRICTION_BAND_RAD) {
            tau_static_nm = -APP_MIT_STATIC_FRICTION_NM;
        }

        tau_out_nm = (APP_MIT_KP_NM_PER_RAD * pos_err_rad) +
                     (APP_MIT_KD_NM_PER_RAD_S * vel_err_rad_s) +
                     tau_static_nm +
                     m_f32MitTauFfNm;
        tau_out_nm = Motor_ControlClamp(tau_out_nm,
                                        -APP_MIT_TAU_OUT_LIMIT_NM,
                                         APP_MIT_TAU_OUT_LIMIT_NM);
    }

    m_f32MitTauStaticNm = tau_static_nm;
    m_f32MitTauOutRefNm = tau_out_nm;

    if ((torque_den <= 0.0f) || (APP_MIT_MOTOR_KT_NM_PER_A <= 0.0f)) {
        m_f32MitTauMotorRefNm = 0.0f;
        return 0.0f;
    }

    tau_motor_nm = tau_out_nm / torque_den;
    iq_ref_a = tau_motor_nm / APP_MIT_MOTOR_KT_NM_PER_A;

    m_f32MitTauMotorRefNm = tau_motor_nm;
    return Motor_ControlClamp(iq_ref_a, -APP_MIT_IQ_LIMIT_A, APP_MIT_IQ_LIMIT_A);
}

static uint8_t Motor_ControlIsMitZeroCommand(void)
{
    if ((m_u8MitDone != 0U) &&
        (m_f32IqRefA < APP_ZERO_IQ_OFF_BAND_A) &&
        (m_f32IqRefA > -APP_ZERO_IQ_OFF_BAND_A)) {
        return 1U;
    }

    return 0U;
}

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
    m_f32IqRefA = 0.0f;
    m_u8ControlMode = APP_CONTROL_MODE_MIT;
    m_i32PositionTargetDeg = 0;
    m_f32PositionTargetRad = 0.0f;
    m_f32PositionErrorRad = 0.0f;
    m_i32MitVelocityTargetDegS = 0;
    m_f32MitVelocityTargetRadS = 0.0f;
    m_f32MitVelocityErrorRadS = 0.0f;
    m_f32MitTauFfNm = 0.0f;
    m_f32MitTauOutRefNm = 0.0f;
    m_f32MitTauMotorRefNm = 0.0f;
    m_u8MitDone = 0U;
    m_u16MitLoopDiv = 0U;

    Motor_FOC_Init();
    Motor_FOC_SafeOff();
}

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
        m_u16MitLoopDiv = 0U;
        m_f32IqRefA = 0.0f;
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
        m_u16MitLoopDiv = 0U;
        Motor_FOC_Reset();
    }

    m_u8FocState = APP_FOC_STATE_MIT;
    theta = Motor_ControlGetElectricalAngle();
    m_f32ThetaE = theta;

    if ((m_u32ControlTick - Encoder_GetUpdateTick()) > APP_ENCODER_STALE_FAULT_TICK) {
        Motor_ControlEnterFault();
        return;
    }

    m_u16MitLoopDiv++;
    if (m_u16MitLoopDiv >= APP_MIT_LOOP_DIV) {
        m_u16MitLoopDiv = 0U;
        m_f32IqRefA = Motor_ControlMitLoopStep();
    }

    current_loop_div_cnt++;
    if (current_loop_div_cnt >= APP_CURRENT_LOOP_DIV) {
        current_loop_div_cnt = 0U;
        Motor_FOC_SetCurrentRef(0.0f, m_f32IqRefA);
    }

    if (Motor_ControlIsMitZeroCommand() != 0U) {
        Motor_FOC_Reset();
        Motor_FOC_SafeOff();
    } else {
        Motor_FOC_CurrentLoop(m_f32IaA, m_f32IbA, m_f32IcA, theta);
    }
}

/* 串口MIT入口：设置单圈绝对位置、目标速度和输出端前馈力矩。 */
void Motor_ControlSetMitTarget(int32_t position_target_deg,
                               int32_t velocity_target_deg_s,
                               int32_t tau_ff_mnm)
{
    int32_t pos_deg = position_target_deg % 360;
    float tau_ff_nm = ((float)tau_ff_mnm) * 0.001f;

    if (pos_deg < 0) {
        pos_deg += 360;
    }

    if (velocity_target_deg_s > APP_MIT_VEL_TARGET_LIMIT_DEG_S) {
        velocity_target_deg_s = APP_MIT_VEL_TARGET_LIMIT_DEG_S;
    } else if (velocity_target_deg_s < -APP_MIT_VEL_TARGET_LIMIT_DEG_S) {
        velocity_target_deg_s = -APP_MIT_VEL_TARGET_LIMIT_DEG_S;
    }

    tau_ff_nm = Motor_ControlClamp(tau_ff_nm,
                                   -APP_MIT_TAU_OUT_LIMIT_NM,
                                    APP_MIT_TAU_OUT_LIMIT_NM);

    m_u8ControlMode = APP_CONTROL_MODE_MIT;
    m_i32PositionTargetDeg = pos_deg;
    m_f32PositionTargetRad = ((float)pos_deg) * APP_TWO_PI / 360.0f;
    m_i32MitVelocityTargetDegS = velocity_target_deg_s;
    m_f32MitVelocityTargetRadS = ((float)velocity_target_deg_s) * APP_TWO_PI / 360.0f;
    m_f32MitTauFfNm = tau_ff_nm;
    m_f32IqRefA = 0.0f;
    m_f32MitTauOutRefNm = 0.0f;
    m_f32MitTauMotorRefNm = 0.0f;
    m_u8MitDone = 0U;
    m_u16MitLoopDiv = 0U;
    Motor_FOC_Reset();
}

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
    debug->iq_ref_a = m_f32IqRefA;
    debug->control_mode = m_u8ControlMode;
    debug->position_final_deg = m_i32PositionTargetDeg;
    debug->position_error_rad = m_f32PositionErrorRad;
    debug->mit_velocity_final_deg_s = m_i32MitVelocityTargetDegS;
    debug->mit_velocity_target_rad_s = m_f32MitVelocityTargetRadS;
    debug->mit_tau_ff_nm = m_f32MitTauFfNm;
    debug->mit_tau_static_nm = m_f32MitTauStaticNm;
    debug->mit_tau_out_ref_nm = m_f32MitTauOutRefNm;
    debug->mit_tau_motor_ref_nm = m_f32MitTauMotorRefNm;
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
