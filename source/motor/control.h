#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>

typedef struct {
    uint8_t foc_state;
    uint8_t motor_enable;
    uint8_t fault;
    uint8_t offset_done;
    int32_t iq_final_ma;
    float iq_ref_a;
    uint16_t adc_raw_u;
    uint16_t adc_raw_v;
    uint16_t adc_raw_w;
    int32_t ia_cnt;
    int32_t ib_cnt;
    int32_t ic_cnt;
    float ia_a;
    float ib_a;
    float ic_a;
} motor_control_debug_t;

void Motor_ControlInit(void);
void Motor_ControlFastLoop(uint16_t raw_u, uint16_t raw_v, uint16_t raw_w);
void Motor_ControlSetIqTargetMA(int32_t iq_target_ma);
void Motor_ControlEnterFault(void);
void Motor_ControlSetElectricalAngle(float theta_e);
uint8_t Motor_ControlIsEncoderZeroDone(void);
uint32_t Motor_ControlGetTick(void);
uint8_t Motor_ControlIsOffsetDone(void);
uint8_t Motor_ControlIsFault(void);
uint8_t Motor_ControlIsEnabled(void);
void Motor_ControlGetDebug(motor_control_debug_t *debug);

#endif /* __CONTROL_H__ */
