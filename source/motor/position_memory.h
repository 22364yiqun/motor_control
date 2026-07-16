#ifndef __POSITION_MEMORY_H__
#define __POSITION_MEMORY_H__

#include <stdint.h>

void PositionMemory_Init(void);
void PositionMemory_UpdateMotorSingleTurn(int32_t motor_single_deg_x100);
void PositionMemory_Task(void);
int32_t PositionMemory_GetMotorTotalDegX100(void);
int32_t PositionMemory_GetOutputTotalDegX100(void);
uint8_t PositionMemory_HasValidFlashRecord(void);
int32_t PositionMemory_GetLastFlashStatus(void);

#endif /* __POSITION_MEMORY_H__ */
