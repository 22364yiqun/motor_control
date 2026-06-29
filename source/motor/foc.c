#include "foc.h"

#include "config.h"
#include "hc32_ll.h"

#include <math.h>
#include <stddef.h>

static volatile float m_f32IdRefA = 0.0f;     /* d轴电流指令，单位A。 */
static volatile float m_f32IqRefA = 0.0f;     /* q轴转矩电流指令，单位A。 */

static volatile float m_f32IdA = 0.0f;        /* 实际d轴电流，单位A。 */
static volatile float m_f32IqA = 0.0f;        /* 实际q轴电流，单位A。 */

static volatile float m_f32VdCmd = 0.0f;      /* SVPWM使用的d轴电压指令。 */
static volatile float m_f32VqCmd = 0.0f;      /* SVPWM使用的q轴电压指令。 */

static volatile float m_f32IaRefA = 0.0f;     /* 仅用于打印显示的A相期望电流。 */
static volatile float m_f32IbRefA = 0.0f;     /* 仅用于打印显示的B相期望电流。 */
static volatile float m_f32IcRefA = 0.0f;     /* 仅用于打印显示的C相期望电流。 */

static float m_f32IdIntegralV = 0.0f;         /* d轴PI积分状态。 */
static float m_f32IqIntegralV = 0.0f;         /* q轴PI积分状态。 */

static float Motor_FOC_Clamp(float val, float min, float max)
{
    if (val < min) {
        return min;
    }

    if (val > max) {
        return max;
    }

    return val;
}

static float Motor_FOC_SqrtApprox(float val)
{
    float x;
    uint8_t i;

    if (val <= 0.0f) {
        return 0.0f;
    }

    x = val;
    if (x < 1.0f) {
        x = 1.0f;
    }

    for (i = 0U; i < 6U; i++) {
        x = 0.5f * (x + (val / x));
    }

    return x;
}

/* 三相电流 -> 静止坐标系alpha/beta电流。 */
static void Motor_FOC_Clarke(float ia, float ib, float ic, float *i_alpha, float *i_beta)
{
    (void)ic;

    *i_alpha = ia;
    *i_beta = (ia + (2.0f * ib)) * 0.57735026919f;
}

/* 静止坐标系alpha/beta电流 -> 转子坐标系d/q电流。 */
static void Motor_FOC_Park(float i_alpha, float i_beta, float theta_e, float *id, float *iq)
{
    const float sin_t = sinf(theta_e);
    const float cos_t = cosf(theta_e);

    *id = (i_alpha * cos_t) + (i_beta * sin_t);
    *iq = (-i_alpha * sin_t) + (i_beta * cos_t);
}

/* d/q电流指令 -> 期望三相电流，仅用于调试打印。 */
static void Motor_FOC_InvParkClarke(float id, float iq, float theta_e,
                                    float *ia, float *ib, float *ic)
{
    const float sin_t = sinf(theta_e);
    const float cos_t = cosf(theta_e);
    const float i_alpha = (id * cos_t) - (iq * sin_t);
    const float i_beta = (id * sin_t) + (iq * cos_t);

    *ia = i_alpha;
    *ib = (-0.5f * i_alpha) + (APP_SQRT3_BY_2 * i_beta);
    *ic = (-0.5f * i_alpha) - (APP_SQRT3_BY_2 * i_beta);
}

/* Id或Iq电流环的一次PI计算，包含积分限幅和输出限幅。 */
static float Motor_FOC_PiStep(float err, float kp, float ki, float *integral)
{
    float out;

    *integral += ki * err * APP_CURRENT_LOOP_TS;
    *integral = Motor_FOC_Clamp(*integral, -APP_CURRENT_PI_I_LIMIT, APP_CURRENT_PI_I_LIMIT);

    out = (kp * err) + (*integral);
    return Motor_FOC_Clamp(out, -APP_CURRENT_V_LIMIT, APP_CURRENT_V_LIMIT);
}

/* 归一化占空比 -> Timer4比较值。 */
static uint16_t Motor_FOC_DutyToCompare(float duty)
{
    duty = Motor_FOC_Clamp(duty, APP_DUTY_MIN, APP_DUTY_MAX);
    return (uint16_t)(duty * (float)APP_PWM_PERIOD_VALUE);
}

/* 把三相占空比写入PWM比较寄存器。 */
static void Motor_FOC_UpdateCompare(float duty_u, float duty_v, float duty_w)
{
    const uint16_t cmp_u = Motor_FOC_DutyToCompare(duty_u);
    const uint16_t cmp_v = Motor_FOC_DutyToCompare(duty_v);
    const uint16_t cmp_w = Motor_FOC_DutyToCompare(duty_w);

    TMR4_OC_SetCompareValue(CM_TMR4_1, TMR4_OC_CH_UL, cmp_u);
    TMR4_OC_SetCompareValue(CM_TMR4_1, TMR4_OC_CH_VL, cmp_v);
    TMR4_OC_SetCompareValue(CM_TMR4_1, TMR4_OC_CH_WL, cmp_w);
}

/* d/q电压指令 -> 三相SVPWM占空比。 */
static void Motor_FOC_SVPWMGenerateDQ(float theta_e, float vd, float vq)
{
    const float sin_t = sinf(theta_e);
    const float cos_t = cosf(theta_e);
    const float u_alpha = (vd * cos_t) - (vq * sin_t);
    const float u_beta = (vd * sin_t) + (vq * cos_t);

    float ua = u_alpha;
    float ub = (-0.5f * u_alpha) + (APP_SQRT3_BY_2 * u_beta);
    float uc = (-0.5f * u_alpha) - (APP_SQRT3_BY_2 * u_beta);
    float u_max = ua;
    float u_min = ua;
    float offset;

    if (ub > u_max) {
        u_max = ub;
    }
    if (uc > u_max) {
        u_max = uc;
    }

    if (ub < u_min) {
        u_min = ub;
    }
    if (uc < u_min) {
        u_min = uc;
    }

    offset = -0.5f * (u_max + u_min);

    /*
     * 已验证的相序映射：
     * Timer U = 物理C相，Timer V = 物理B相，Timer W = 物理A相。
     */
    Motor_FOC_UpdateCompare(0.5f + uc + offset,
                            0.5f + ub + offset,
                            0.5f + ua + offset);
}

void Motor_FOC_Init(void)
{
    Motor_FOC_Reset();
}

void Motor_FOC_Reset(void)
{
    m_f32IdIntegralV = 0.0f;
    m_f32IqIntegralV = 0.0f;

    m_f32IdRefA = 0.0f;
    m_f32IqRefA = 0.0f;
    m_f32IdA = 0.0f;
    m_f32IqA = 0.0f;
    m_f32VdCmd = 0.0f;
    m_f32VqCmd = 0.0f;
    m_f32IaRefA = 0.0f;
    m_f32IbRefA = 0.0f;
    m_f32IcRefA = 0.0f;
}

/* 设置电流参考值；斜坡和安全判断由control.c负责。 */
void Motor_FOC_SetCurrentRef(float id_ref_a, float iq_ref_a)
{
    m_f32IdRefA = id_ref_a;
    m_f32IqRefA = iq_ref_a;
}

/* 闭环电流控制：相电流反馈 + 电角度 -> 更新PWM。 */
void Motor_FOC_CurrentLoop(float ia_a, float ib_a, float ic_a, float theta_e)
{
    float i_alpha;
    float i_beta;
    float id;
    float iq;
    float vd;
    float vq;
    float mag2;
    float ia_ref;
    float ib_ref;
    float ic_ref;

    Motor_FOC_Clarke(ia_a, ib_a, ic_a, &i_alpha, &i_beta);
    Motor_FOC_Park(i_alpha, i_beta, theta_e, &id, &iq);

    m_f32IdA = id;
    m_f32IqA = iq;

    vd = Motor_FOC_PiStep(m_f32IdRefA - id, APP_ID_KP, APP_ID_KI, &m_f32IdIntegralV);
    vq = Motor_FOC_PiStep(m_f32IqRefA - iq, APP_IQ_KP, APP_IQ_KI, &m_f32IqIntegralV);

    mag2 = (vd * vd) + (vq * vq);
    if (mag2 > (APP_CURRENT_V_VECTOR_LIMIT * APP_CURRENT_V_VECTOR_LIMIT)) {
        const float scale = APP_CURRENT_V_VECTOR_LIMIT / Motor_FOC_SqrtApprox(mag2);
        vd *= scale;
        vq *= scale;
    }

    m_f32VdCmd = vd;
    m_f32VqCmd = vq;

    Motor_FOC_InvParkClarke(m_f32IdRefA, m_f32IqRefA, theta_e,
                            &ia_ref,
                            &ib_ref,
                            &ic_ref);
    m_f32IaRefA = ia_ref;
    m_f32IbRefA = ib_ref;
    m_f32IcRefA = ic_ref;

    Motor_FOC_SVPWMGenerateDQ(theta_e, vd, vq);
}

/* 直接输出d/q电压，用于转子对齐阶段。 */
void Motor_FOC_OutputDQ(float theta_e, float vd, float vq)
{
    m_f32VdCmd = vd;
    m_f32VqCmd = vq;
    Motor_FOC_SVPWMGenerateDQ(theta_e, vd, vq);
}

/* PWM输出50%共模占空比，不产生有效相电压。 */
void Motor_FOC_SafeOff(void)
{
    Motor_FOC_UpdateCompare(0.5f, 0.5f, 0.5f);
}

void Motor_FOC_GetDebug(motor_foc_debug_t *debug)
{
    if (debug == NULL) {
        return;
    }

    debug->id_ref_a = m_f32IdRefA;
    debug->iq_ref_a = m_f32IqRefA;
    debug->id_a = m_f32IdA;
    debug->iq_a = m_f32IqA;
    debug->vd_cmd = m_f32VdCmd;
    debug->vq_cmd = m_f32VqCmd;
    debug->ia_ref_a = m_f32IaRefA;
    debug->ib_ref_a = m_f32IbRefA;
    debug->ic_ref_a = m_f32IcRefA;
}
