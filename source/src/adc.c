#include "adc.h"
#include "control.h"
#include "config.h"

static volatile uint16_t m_u16AdcCh0 = 0U;
static volatile uint16_t m_u16AdcCh4 = 0U;
static volatile uint16_t m_u16AdcCh6 = 0U;

static volatile uint8_t  m_u8AdcReady = 0U;

static volatile uint8_t m_u8OffsetDone = 0U; // ADC 零点校准是否完成


void AdcCfg(void)
{
    //independent mode
    //ADC1 config 
    stc_adc_init_t stcAdcInit;

    /* 1. Enable ADC1 peripheral clock. */
    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_ADC1, ENABLE);

    /* 2. Modify the default value depends on the application. */
    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode = ADC_MD_SEQA_SINGLESHOT;
    stcAdcInit.u16Resolution = ADC_RESOLUTION_12BIT;
    stcAdcInit.u16DataAlign = ADC_DATAALIGN_RIGHT;

    /* 3. Initializes ADC. */
    (void)ADC_Init(CM_ADC1, &stcAdcInit);

    /* 4. ADC sequence configuration. */
    /* Config trigger event source if needed*/
    FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_AOS, ENABLE);
    AOS_SetTriggerEventSrc(AOS_ADC1_0, EVT_SRC_TMR4_1_OVF);// 谷值触发，可能后面还需要修改
    /* ADC sequence A configuration. */
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH0, ENABLE);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH4, ENABLE);
    ADC_ChCmd(CM_ADC1, ADC_SEQ_A, ADC_CH6, ENABLE);

    ADC_TriggerConfig(CM_ADC1, ADC_SEQ_A, ADC_HARDTRIG_EVT0);
    ADC_TriggerCmd(CM_ADC1, ADC_SEQ_A, ENABLE);
    /* ADC Int configuration */
    ADC_IntCmd(CM_ADC1, ADC_INT_EOCA, ENABLE);
}

void ADC_GetSample(adc_sample_t *pstSample)
{
    if (pstSample != NULL) {
        pstSample->raw_u = m_u16AdcCh0;
        pstSample->raw_v = m_u16AdcCh4;
        pstSample->raw_w = m_u16AdcCh6;
        pstSample->ready = m_u8AdcReady;
    }
}

uint8_t ADC_IsReady(void)
{
    return m_u8AdcReady;
}

void ADC_ClearReady(void)
{
    m_u8AdcReady = 0U;
}

void ADC1_Handler(void)
{
    if (SET == ADC_GetStatus(CM_ADC1, ADC_FLAG_EOCA)) {

        ADC_ClearStatus(CM_ADC1, ADC_FLAG_EOCA);

        m_u16AdcCh0 = ADC_GetValue(CM_ADC1, ADC_CH0);
        m_u16AdcCh4 = ADC_GetValue(CM_ADC1, ADC_CH4);
        m_u16AdcCh6 = ADC_GetValue(CM_ADC1, ADC_CH6);
        m_u8AdcReady = 1U;

        Motor_ControlFastLoop(m_u16AdcCh6, m_u16AdcCh4, m_u16AdcCh0);
    }
}
