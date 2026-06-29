#include "int.h"

#include "adc.h"
#include "dma.h"
#include "usart.h"

void IntCfg(void)
{
    stc_irq_signin_config_t irq;

    irq.enIntSrc = INT_SRC_USART1_RI;
    irq.enIRQn = INT000_IRQn;
    irq.pfnCallback = &USART1_RxIrqCallback;
    (void)INTC_IrqSignIn(&irq);
    NVIC_ClearPendingIRQ(INT000_IRQn);
    NVIC_SetPriority(INT000_IRQn, DDL_IRQ_PRIO_13);
    NVIC_EnableIRQ(INT000_IRQn);

    irq.enIntSrc = INT_SRC_USART1_EI;
    irq.enIRQn = INT001_IRQn;
    irq.pfnCallback = &USART1_ErrorIrqCallback;
    (void)INTC_IrqSignIn(&irq);
    NVIC_ClearPendingIRQ(INT001_IRQn);
    NVIC_SetPriority(INT001_IRQn, DDL_IRQ_PRIO_13);
    NVIC_EnableIRQ(INT001_IRQn);

    NVIC_ClearPendingIRQ(ADC1_IRQn);
    NVIC_SetPriority(ADC1_IRQn, DDL_IRQ_PRIO_15);
    NVIC_EnableIRQ(ADC1_IRQn);

    NVIC_ClearPendingIRQ(DMA1_TC0_BTC0_IRQn);
    NVIC_SetPriority(DMA1_TC0_BTC0_IRQn, DDL_IRQ_PRIO_14);
    NVIC_EnableIRQ(DMA1_TC0_BTC0_IRQn);

    NVIC_ClearPendingIRQ(DMA1_ERR_IRQn);
    NVIC_SetPriority(DMA1_ERR_IRQn, DDL_IRQ_PRIO_14);
    NVIC_EnableIRQ(DMA1_ERR_IRQn);
}
