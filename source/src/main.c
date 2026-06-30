/**
 *******************************************************************************
 * @file  main.c
 * @brief Main program.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2026-05-31       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"
#include "clock.h"
#include "gpio.h"
#include "config.h"
#include "usart.h"
#include "adc.h"
#include "control.h"
#include "tim.h"
#include "spi.h"
#include "int.h"
#include "dma.h"
#include "encoder.h"
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
    //SPI Config
    SpiCfg();
    /* Register write protected for some required peripherals. */
    LL_PERIPH_WP(LL_PERIPH_ALL);

    App_USART1SendString("UART-IRQ speed-current dual-loop FOC start\r\n");
    App_USART1SendString("Input integer deg/s, e.g. 20, -20, 0, then Enter.\r\n");

    for (;;) {
        App_USART1TxPumpInMainLoop();
        App_USART1RxCommandTask();
        Encoder_Task();
        App_USART1RxCommandTask();
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
