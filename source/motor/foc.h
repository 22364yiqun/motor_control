#ifndef __FOC_H__
#define __FOC_H__

#include <stdint.h>

typedef struct {
    float id_ref_a;
    float iq_ref_a;
    float id_a;
    float iq_a;
    float vd_cmd;
    float vq_cmd;
    float ia_ref_a;
    float ib_ref_a;
    float ic_ref_a;
} motor_foc_debug_t;

void Motor_FOC_Init(void);
void Motor_FOC_Reset(void);
void Motor_FOC_SetCurrentRef(float id_ref_a, float iq_ref_a);
void Motor_FOC_CurrentLoop(float ia_a, float ib_a, float ic_a, float theta_e);
void Motor_FOC_OutputDQ(float theta_e, float vd, float vq);
void Motor_FOC_SafeOff(void);
void Motor_FOC_GetDebug(motor_foc_debug_t *debug);

#endif /* __FOC_H__ */
