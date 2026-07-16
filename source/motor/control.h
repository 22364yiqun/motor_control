#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <stdint.h>

typedef struct {
    uint8_t foc_state;
    uint8_t motor_enable;
    uint8_t fault;
    uint8_t offset_done;
    float iq_ref_a;
    uint8_t control_mode;
    int32_t position_final_deg;
    float position_error_rad;
    int32_t mit_velocity_final_deg_s;
    float mit_velocity_target_rad_s;
    float mit_tau_ff_nm;
    float mit_tau_static_nm;
    float mit_tau_out_ref_nm;
    float mit_tau_motor_ref_nm;
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
void Motor_ControlSetOutputTargetDeg(int32_t output_position_deg,
                                     int32_t output_velocity_deg_s,
                                     int32_t tau_ff_mnm);
void Motor_ControlSetOutputTargetX100(int32_t output_position_x100,
                                      int32_t output_velocity_x10,
                                      int32_t tau_ff_mnm);
void Motor_ControlSetMitTarget(int32_t position_target_deg,
                               int32_t velocity_target_deg_s,
                               int32_t tau_ff_mnm);
void Motor_ControlSetMitTargetX100(int32_t position_target_x100,
                                   int32_t velocity_target_x10,
                                   int32_t tau_ff_mnm);
void Motor_ControlEnterFault(void);
void Motor_ControlSetElectricalAngle(float theta_e);
uint8_t Motor_ControlIsEncoderZeroDone(void);
uint32_t Motor_ControlGetTick(void);
uint8_t Motor_ControlIsOffsetDone(void);
uint8_t Motor_ControlIsFault(void);
uint8_t Motor_ControlIsEnabled(void);
void Motor_ControlGetDebug(motor_control_debug_t *debug);

#endif /* __CONTROL_H__ */
