/*lic*/
#ifndef __ADC_H__
#define __ADC_H__

#include "hc32_ll.h"
#include <stdint.h>

typedef struct
{
    uint16_t raw_u;
    uint16_t raw_v;
    uint16_t raw_w;
    uint8_t ready;
} adc_sample_t;

void AdcCfg(void);
void ADC1_Handler(void);
void ADC_GetSample(adc_sample_t *pstSample);
uint8_t ADC_IsReady(void);
void ADC_ClearReady(void);

#endif /* __ADC_H__ */

/*eof*/
