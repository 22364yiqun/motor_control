/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"
#include "clock.h"
#include "gpio.h"
#include "config.h"
#include "usart.h"
#include "can.h"
#include "adc.h"
#include "control.h"
#include "tim.h"
#include "spi.h"
#include "int.h"
#include "dma.h"
#include "encoder.h"
#include "position_memory.h"
#include <math.h>

/**
 * @brief  Main function of the project
 * @param  None
 * @retval int32_t return value, if needed
 */
int main(void)
{
    /* Register write unprotected for some required peripherals. */
    LL_PERIPH_WE(LL_PERIPH_ALL);
    //Clock Config
    ClockCfg();
    //Port Config
    PortCfg();
    //Int Config
    IntCfg();
    //ADC Config
    AdcCfg();
    //DMA Config
    DmaCfg();
    //Timer4 Config
    Timer4Cfg();
    //Motor control Config
    Motor_ControlInit();
    //USARTx Config
    UsartCfg();
    //CAN Config
    CanCfg();
    //SPI Config
    SpiCfg();
    //Multi-turn position memory
    PositionMemory_Init();
    /* Register write protected for some required peripherals. */
    LL_PERIPH_WP(LL_PERIPH_ALL);

    App_USART1SendString("CAN+UART MIT FOC start\r\n");
    App_USART1SendString("UART: m90 or m90,0,0 = output angle. CAN ID=0x201 DLC=8.\r\n");

    for (;;) {
        App_USART1TxPumpInMainLoop();
        App_USART1RxCommandTask();
        App_CANTask();
        Encoder_Task();
        PositionMemory_Task();
        Encoder_StatusPrintTask();
        App_USART1TxPumpInMainLoop();
    }
}

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
