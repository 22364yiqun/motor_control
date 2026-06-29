/*lic*/
#include "clock.h"

void ClockCfg(void)
{
    stc_clock_xtal_init_t stcXtalInit;
    stc_clock_pll_init_t stcPLLHInit;

    /* Set bus clock div. */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 |
                                      CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));

    /* Flash read wait cycle setting. */
    EFM_SetWaitCycle(EFM_WAIT_CYCLE3);

    /* XTAL config. */
    (void)CLK_XtalStructInit(&stcXtalInit);
    stcXtalInit.u8State = CLK_XTAL_ON;
    stcXtalInit.u8Drv = CLK_XTAL_DRV_HIGH;
    stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stcXtalInit);

    /* PLLH config. */
    (void)CLK_PLLStructInit(&stcPLLHInit);
    stcPLLHInit.PLLCFGR = 0UL;
    stcPLLHInit.PLLCFGR_f.PLLM = (1UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLN = (100UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLP = (4UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLQ = (4UL - 1UL);
    stcPLLHInit.PLLCFGR_f.PLLR = (4UL - 1UL);
    stcPLLHInit.u8PLLState = CLK_PLL_ON;
    stcPLLHInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
    (void)CLK_PLLInit(&stcPLLHInit);

    /* 3 cycles for 150MHz ~ 200MHz. */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);

    /* Switch driver ability. */
    PWC_LowSpeedToHighSpeed();

    /* Set the system clock source. */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
}

/*eof*/
