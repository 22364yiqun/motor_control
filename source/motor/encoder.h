#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdint.h>

typedef struct {
    uint16_t raw_frame16;
    uint16_t raw_angle14;
    float mech_angle_rad;
    float mech_speed_rad_s;
    float angle_degrees;
    int32_t dma_status;
    uint32_t update_tick;
} encoder_debug_t;

void Encoder_Task(void);
void Encoder_StatusPrintTask(void);
void Encoder_SetElectricalZeroOffset(float offset);
float Encoder_GetElectricalAngle(void);
float Encoder_GetMechAngleRad(void);
float Encoder_GetMechSpeedRadS(void);
uint32_t Encoder_GetUpdateTick(void);
int32_t Encoder_GetDmaStatus(void);
void Encoder_GetDebug(encoder_debug_t *debug);

void MA732_DMA_RxDoneCallback(void);
void MA732_DMA_ErrorCallback(void);

#endif /* __ENCODER_H__ */
