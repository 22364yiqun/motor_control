/*lic*/
#include "tim.h"
#include "config.h"

void Timer4Cfg(void)
{
    stc_tmr4_init_t stcTmr4Init;
    stc_tmr4_oc_init_t stcTmr4OcInit;
    un_tmr4_oc_ocmrh_t unTmr4OcOcmrh;
    un_tmr4_oc_ocmrl_t unTmr4OcOcmrl;
    stc_tmr4_pwm_init_t stcTmr4PwmInit;

    /* Enable TMR4_1 peripheral clock */
    FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR4_1, ENABLE);

    /************************* Configure TMR4_1 counter *************************/
    stcTmr4Init.u16ClockSrc = TMR4_CLK_SRC_INTERNCLK;
    stcTmr4Init.u16ClockDiv = TMR4_CLK_DIV2;
    stcTmr4Init.u16CountMode = TMR4_MD_TRIANGLE;
    stcTmr4Init.u16PeriodValue = (uint16_t)(APP_PWM_PERIOD_VALUE);
    // 设置成62.5us一个周期，即16kHz的PWM频率，配合死区时间55个计数（约3.44us）
    (void)TMR4_Init(CM_TMR4_1, &stcTmr4Init);

    /* Enable TMR4 period buffer function. */
    TMR4_PeriodBufCmd(CM_TMR4_1, ENABLE);

    /************************* Configure TMR4_1_UH output-compare ******************/
    /* Initialize TMR4_1_UH OC structure */
    (void)TMR4_OC_StructInit(&stcTmr4OcInit);

    /* TMR4_1_UH OC channel initialize */
    stcTmr4OcInit.u16CompareValue = MOTOR_PWM_COMPARE_VALUE;
    stcTmr4OcInit.u16OcInvalidPolarity = TMR4_OC_INVD_LOW;
    stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_VALLEY;
    stcTmr4OcInit.u16CompareModeBufCond = TMR4_OC_BUF_COND_IMMED;
    stcTmr4OcInit.u16BufLinkTransObject = TMR4_OC_BUF_CMP_VALUE ;
    (void)TMR4_OC_Init(CM_TMR4_1, TMR4_OC_CH_UH, &stcTmr4OcInit);

    /* TMR4_1_UH OC channel: compare mode OCMR */
    unTmr4OcOcmrh.OCMRx = (uint16_t)0x0 ; 

    TMR4_OC_SetHighChCompareMode(CM_TMR4_1, TMR4_OC_CH_UH, unTmr4OcOcmrh);

    TMR4_PWM_SetPortOutputMode(CM_TMR4_1, TMR4_PWM_PIN_OUH, TMR4_PWM_PIN_OUTPUT_NORMAL);
    TMR4_PWM_SetAbnormalPinStatus(CM_TMR4_1, TMR4_PWM_PIN_OUH, TMR4_PWM_ABNORMAL_PIN_LOW);

    /* TMR4_1_UH OC channel: enable output-compare */
    TMR4_OC_Cmd(CM_TMR4_1, TMR4_OC_CH_UH, ENABLE);

    /************************* Configure TMR4_1_UL output-compare ******************/
    /* Initialize TMR4_1_UL OC structure */
    (void)TMR4_OC_StructInit(&stcTmr4OcInit);

    /* TMR4_1_UL OC channel initialize */
    stcTmr4OcInit.u16CompareValue = MOTOR_PWM_COMPARE_VALUE;
    stcTmr4OcInit.u16OcInvalidPolarity = TMR4_OC_INVD_LOW;
    stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_VALLEY;
    stcTmr4OcInit.u16CompareModeBufCond = TMR4_OC_BUF_COND_IMMED;
    stcTmr4OcInit.u16BufLinkTransObject = TMR4_OC_BUF_CMP_VALUE ;
    (void)TMR4_OC_Init(CM_TMR4_1, TMR4_OC_CH_UL, &stcTmr4OcInit);

    /* TMR4_1_UL OC channel: compare mode OCMR */
    unTmr4OcOcmrl.OCMRx = (uint32_t)0x46104615;
    // unTmr4OcOcmrl.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET; /* bit[0]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET  ; /* bit[1]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET; /* bit[2]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET  ; /* bit[3]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OPDCL   = TMR4_OC_LOW;    /* bit[5:4]   01 */
    // unTmr4OcOcmrl.OCMRx_f.OPPKL   = TMR4_OC_HOLD;    /* Bit[7:6]   01 */
    // unTmr4OcOcmrl.OCMRx_f.OPUCL   = TMR4_OC_HIGH;     /* bit[9:8]   10 */
    // unTmr4OcOcmrl.OCMRx_f.OPZRL   = TMR4_OC_LOW;    /* Bit[11:10] 00 */
    // unTmr4OcOcmrl.OCMRx_f.OPNPKL  = TMR4_OC_HOLD;     /* Bit[13:12] 10 */ 
    // unTmr4OcOcmrl.OCMRx_f.OPNZRL  = TMR4_OC_LOW;    /* bit[15:14] 00 */

    // unTmr4OcOcmrl.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;    /* bit[17:16] 00 */
    // unTmr4OcOcmrl.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;    /* bit[19:18] 00 */
    // unTmr4OcOcmrl.OCMRx_f.EOPDCL  = TMR4_OC_LOW;    /* bit[21:20] 01 */
    // unTmr4OcOcmrl.OCMRx_f.EOPPKL  = TMR4_OC_HOLD;    /* Bit[7:6]   01 */
    // unTmr4OcOcmrl.OCMRx_f.EOPUCL  = TMR4_OC_HIGH;     /* bit[25:24] 10 */
    // unTmr4OcOcmrl.OCMRx_f.EOPZRL  = TMR4_OC_LOW;    /* Bit[11:10] 00 */
    // unTmr4OcOcmrl.OCMRx_f.EOPNPKL = TMR4_OC_HOLD;     /* Bit[13:12] 10 */
    // unTmr4OcOcmrl.OCMRx_f.EOPNZRL = TMR4_OC_LOW;    /* bit[31:30] 00 */

    TMR4_OC_SetLowChCompareMode(CM_TMR4_1, TMR4_OC_CH_UL, unTmr4OcOcmrl);

    /* Set TMR4_1_UL port output mode */
    TMR4_PWM_SetPortOutputMode(CM_TMR4_1, TMR4_PWM_PIN_OUL, TMR4_PWM_PIN_OUTPUT_NORMAL);

    /* Set TMR4_1_UL pin status when below conditions occur:1.EMB 2.MOE=0 3.MOE=1&OExy=0 */
    TMR4_PWM_SetAbnormalPinStatus(CM_TMR4_1, TMR4_PWM_PIN_OUL, TMR4_PWM_ABNORMAL_PIN_LOW);


    /* TMR4_1_UL OC channel: enable output-compare */
    TMR4_OC_Cmd(CM_TMR4_1, TMR4_OC_CH_UL, ENABLE);

    /************************* Configure TMR4_1_VH output-compare ******************/
    /* Initialize TMR4_1_VH OC structure */
    (void)TMR4_OC_StructInit(&stcTmr4OcInit);

    /* TMR4_1_VH OC channel initialize */
    stcTmr4OcInit.u16CompareValue = MOTOR_PWM_COMPARE_VALUE;
    stcTmr4OcInit.u16OcInvalidPolarity = TMR4_OC_INVD_LOW;
    stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_VALLEY;
    stcTmr4OcInit.u16CompareModeBufCond = TMR4_OC_BUF_COND_IMMED;
    stcTmr4OcInit.u16BufLinkTransObject = TMR4_OC_BUF_CMP_VALUE ;
    (void)TMR4_OC_Init(CM_TMR4_1, TMR4_OC_CH_VH, &stcTmr4OcInit);

    /* TMR4_1_VH OC channel: compare mode OCMR */
    unTmr4OcOcmrh.OCMRx = (uint16_t)0x0;

    TMR4_OC_SetHighChCompareMode(CM_TMR4_1, TMR4_OC_CH_VH, unTmr4OcOcmrh);

    /* Set TMR4_1_VH port output mode */
    TMR4_PWM_SetPortOutputMode(CM_TMR4_1, TMR4_PWM_PIN_OVH, TMR4_PWM_PIN_OUTPUT_NORMAL);

    /* Set TMR4_1_VH pin status when below conditions occur:1.EMB 2.MOE=0 3.MOE=1&OExy=0 */
    TMR4_PWM_SetAbnormalPinStatus(CM_TMR4_1, TMR4_PWM_PIN_OVH, TMR4_PWM_ABNORMAL_PIN_LOW);

    /* TMR4_1_VH OC channel: enable output-compare */
    TMR4_OC_Cmd(CM_TMR4_1, TMR4_OC_CH_VH, ENABLE);

    /************************* Configure TMR4_1_VL output-compare ******************/
    /* Initialize TMR4_1_VL OC structure */
    (void)TMR4_OC_StructInit(&stcTmr4OcInit);

    /* TMR4_1_VL OC channel initialize */
    stcTmr4OcInit.u16CompareValue = MOTOR_PWM_COMPARE_VALUE;
    stcTmr4OcInit.u16OcInvalidPolarity = TMR4_OC_INVD_LOW;
    stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_VALLEY;
    stcTmr4OcInit.u16CompareModeBufCond = TMR4_OC_BUF_COND_IMMED;
    stcTmr4OcInit.u16BufLinkTransObject = TMR4_OC_BUF_CMP_VALUE ;
    (void)TMR4_OC_Init(CM_TMR4_1, TMR4_OC_CH_VL, &stcTmr4OcInit);

    /* TMR4_1_VL OC channel: compare mode OCMR */
    unTmr4OcOcmrl.OCMRx = (uint32_t)0x46104615;;
    // unTmr4OcOcmrl.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET; /* bit[0]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET  ; /* bit[1]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET; /* bit[2]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET  ; /* bit[3]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OPDCL   = TMR4_OC_LOW;    /* bit[5:4]   01 */
    // unTmr4OcOcmrl.OCMRx_f.OPPKL   = TMR4_OC_HOLD;    /* Bit[7:6]   01 */
    // unTmr4OcOcmrl.OCMRx_f.OPUCL   = TMR4_OC_HIGH;     /* bit[9:8]   10 */
    // unTmr4OcOcmrl.OCMRx_f.OPZRL   = TMR4_OC_LOW;    /* Bit[11:10] 00 */
    // unTmr4OcOcmrl.OCMRx_f.OPNPKL  = TMR4_OC_HOLD;     /* Bit[13:12] 10 */ 
    // unTmr4OcOcmrl.OCMRx_f.OPNZRL  = TMR4_OC_LOW;    /* bit[15:14] 00 */

    // unTmr4OcOcmrl.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;    /* bit[17:16] 00 */
    // unTmr4OcOcmrl.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;    /* bit[19:18] 00 */
    // unTmr4OcOcmrl.OCMRx_f.EOPDCL  = TMR4_OC_LOW;    /* bit[21:20] 01 */
    // unTmr4OcOcmrl.OCMRx_f.EOPPKL  = TMR4_OC_HOLD;    /* Bit[7:6]   01 */
    // unTmr4OcOcmrl.OCMRx_f.EOPUCL  = TMR4_OC_HIGH;     /* bit[25:24] 10 */
    // unTmr4OcOcmrl.OCMRx_f.EOPZRL  = TMR4_OC_LOW;    /* Bit[11:10] 00 */
    // unTmr4OcOcmrl.OCMRx_f.EOPNPKL = TMR4_OC_HOLD;     /* Bit[13:12] 10 */
    // unTmr4OcOcmrl.OCMRx_f.EOPNZRL = TMR4_OC_LOW;    /* bit[31:30] 00 */
    TMR4_OC_SetLowChCompareMode(CM_TMR4_1, TMR4_OC_CH_VL, unTmr4OcOcmrl);

    /* Set TMR4_1_VL port output mode */
    TMR4_PWM_SetPortOutputMode(CM_TMR4_1, TMR4_PWM_PIN_OVL, TMR4_PWM_PIN_OUTPUT_NORMAL);

    /* Set TMR4_1_VL pin status when below conditions occur:1.EMB 2.MOE=0 3.MOE=1&OExy=0 */
    TMR4_PWM_SetAbnormalPinStatus(CM_TMR4_1, TMR4_PWM_PIN_OVL, TMR4_PWM_ABNORMAL_PIN_LOW);


    /* TMR4_1_VL OC channel: enable output-compare */
    TMR4_OC_Cmd(CM_TMR4_1, TMR4_OC_CH_VL, ENABLE);

    /************************* Configure TMR4_1_WH output-compare ******************/
    /* Initialize TMR4_1_WH OC structure */
    (void)TMR4_OC_StructInit(&stcTmr4OcInit);

    /* TMR4_1_WH OC channel initialize */
    stcTmr4OcInit.u16CompareValue = MOTOR_PWM_COMPARE_VALUE;
    stcTmr4OcInit.u16OcInvalidPolarity = TMR4_OC_INVD_LOW;
    stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_VALLEY;
    stcTmr4OcInit.u16CompareModeBufCond = TMR4_OC_BUF_COND_IMMED;
    stcTmr4OcInit.u16BufLinkTransObject = TMR4_OC_BUF_CMP_VALUE ;
    (void)TMR4_OC_Init(CM_TMR4_1, TMR4_OC_CH_WH, &stcTmr4OcInit);

    /* TMR4_1_WH OC channel: compare mode OCMR */
    unTmr4OcOcmrh.OCMRx = (uint32_t)0x0;

    TMR4_OC_SetHighChCompareMode(CM_TMR4_1, TMR4_OC_CH_WH, unTmr4OcOcmrh);

    /* Set TMR4_1_WH port output mode */
    TMR4_PWM_SetPortOutputMode(CM_TMR4_1, TMR4_PWM_PIN_OWH, TMR4_PWM_PIN_OUTPUT_NORMAL);

    /* Set TMR4_1_WH pin status when below conditions occur:1.EMB 2.MOE=0 3.MOE=1&OExy=0 */
    TMR4_PWM_SetAbnormalPinStatus(CM_TMR4_1, TMR4_PWM_PIN_OWH, TMR4_PWM_ABNORMAL_PIN_LOW);


    /* TMR4_1_WH OC channel: enable output-compare */
    TMR4_OC_Cmd(CM_TMR4_1, TMR4_OC_CH_WH, ENABLE);

    /************************* Configure TMR4_1_WL output-compare ******************/
    /* Initialize TMR4_1_WL OC structure */
    (void)TMR4_OC_StructInit(&stcTmr4OcInit);

    /* TMR4_1_WL OC channel initialize */
    stcTmr4OcInit.u16CompareValue = MOTOR_PWM_COMPARE_VALUE;
    stcTmr4OcInit.u16OcInvalidPolarity = TMR4_OC_INVD_LOW;
    stcTmr4OcInit.u16CompareValueBufCond = TMR4_OC_BUF_COND_VALLEY;
    stcTmr4OcInit.u16CompareModeBufCond = TMR4_OC_BUF_COND_IMMED;
    stcTmr4OcInit.u16BufLinkTransObject = TMR4_OC_BUF_CMP_VALUE ;
    (void)TMR4_OC_Init(CM_TMR4_1, TMR4_OC_CH_WL, &stcTmr4OcInit);

    /* TMR4_1_UL OC channel: compare mode OCMR */
    unTmr4OcOcmrl.OCMRx = (uint32_t)0x46104615;
    // unTmr4OcOcmrl.OCMRx_f.OCFDCL  = TMR4_OC_OCF_SET; /* bit[0]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OCFPKL  = TMR4_OC_OCF_SET  ; /* bit[1]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OCFUCL  = TMR4_OC_OCF_SET; /* bit[2]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OCFZRL  = TMR4_OC_OCF_SET  ; /* bit[3]     1  */
    // unTmr4OcOcmrl.OCMRx_f.OPDCL   = TMR4_OC_LOW;    /* bit[5:4]   01 */
    // unTmr4OcOcmrl.OCMRx_f.OPPKL   = TMR4_OC_HOLD;    /* Bit[7:6]   01 */
    // unTmr4OcOcmrl.OCMRx_f.OPUCL   = TMR4_OC_HIGH;     /* bit[9:8]   10 */
    // unTmr4OcOcmrl.OCMRx_f.OPZRL   = TMR4_OC_LOW;    /* Bit[11:10] 00 */
    // unTmr4OcOcmrl.OCMRx_f.OPNPKL  = TMR4_OC_HOLD;     /* Bit[13:12] 10 */ 
    // unTmr4OcOcmrl.OCMRx_f.OPNZRL  = TMR4_OC_LOW;    /* bit[15:14] 00 */

    // unTmr4OcOcmrl.OCMRx_f.EOPNDCL = TMR4_OC_HOLD;    /* bit[17:16] 00 */
    // unTmr4OcOcmrl.OCMRx_f.EOPNUCL = TMR4_OC_HOLD;    /* bit[19:18] 00 */
    // unTmr4OcOcmrl.OCMRx_f.EOPDCL  = TMR4_OC_LOW;    /* bit[21:20] 01 */
    // unTmr4OcOcmrl.OCMRx_f.EOPPKL  = TMR4_OC_HOLD;    /* Bit[7:6]   01 */
    // unTmr4OcOcmrl.OCMRx_f.EOPUCL  = TMR4_OC_HIGH;     /* bit[25:24] 10 */
    // unTmr4OcOcmrl.OCMRx_f.EOPZRL  = TMR4_OC_LOW;    /* Bit[11:10] 00 */
    // unTmr4OcOcmrl.OCMRx_f.EOPNPKL = TMR4_OC_HOLD;     /* Bit[13:12] 10 */
    // unTmr4OcOcmrl.OCMRx_f.EOPNZRL = TMR4_OC_LOW;    /* bit[31:30] 00 */
    TMR4_OC_SetLowChCompareMode(CM_TMR4_1, TMR4_OC_CH_WL, unTmr4OcOcmrl);

    /* Set TMR4_1_WL port output mode */
    TMR4_PWM_SetPortOutputMode(CM_TMR4_1, TMR4_PWM_PIN_OWL, TMR4_PWM_PIN_OUTPUT_NORMAL);

    /* Set TMR4_1_WL pin status when below conditions occur:1.EMB 2.MOE=0 3.MOE=1&OExy=0 */
    TMR4_PWM_SetAbnormalPinStatus(CM_TMR4_1, TMR4_PWM_PIN_OWL, TMR4_PWM_ABNORMAL_PIN_LOW);

    /* TMR4_1_WL OC channel: enable output-compare */
    TMR4_OC_Cmd(CM_TMR4_1, TMR4_OC_CH_WL, ENABLE);

    /************************* Configure TMR4_1_U PWM *****************************/
    /* TMR4_1_U PWM: initialize */
    (void)TMR4_PWM_StructInit(&stcTmr4PwmInit);
    stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_DEAD_TMR;
    stcTmr4PwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
    stcTmr4PwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
    (void)TMR4_PWM_Init(CM_TMR4_1, TMR4_PWM_CH_U, &stcTmr4PwmInit);
    TMR4_PWM_SetDeadTimeValue(CM_TMR4_1, TMR4_PWM_CH_U, TMR4_PWM_PDAR_IDX, MOTOR_PWM_DEAD_TIME_COUNT);
    TMR4_PWM_SetDeadTimeValue(CM_TMR4_1, TMR4_PWM_CH_U, TMR4_PWM_PDBR_IDX, MOTOR_PWM_DEAD_TIME_COUNT);

    /************************* Configure TMR4_1_V PWM *****************************/
    /* TMR4_1_V PWM: initialize */
    (void)TMR4_PWM_StructInit(&stcTmr4PwmInit);
    stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_DEAD_TMR;
    stcTmr4PwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
    stcTmr4PwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
    (void)TMR4_PWM_Init(CM_TMR4_1, TMR4_PWM_CH_V, &stcTmr4PwmInit);
    TMR4_PWM_SetDeadTimeValue(CM_TMR4_1, TMR4_PWM_CH_V, TMR4_PWM_PDAR_IDX, MOTOR_PWM_DEAD_TIME_COUNT);
    TMR4_PWM_SetDeadTimeValue(CM_TMR4_1, TMR4_PWM_CH_V, TMR4_PWM_PDBR_IDX,MOTOR_PWM_DEAD_TIME_COUNT);

    /************************* Configure TMR4_1_W PWM *****************************/
    /* TMR4_1_W PWM: initialize */
    (void)TMR4_PWM_StructInit(&stcTmr4PwmInit);
    stcTmr4PwmInit.u16Mode = TMR4_PWM_MD_DEAD_TMR;
    stcTmr4PwmInit.u16ClockDiv = TMR4_PWM_CLK_DIV1;
    stcTmr4PwmInit.u16Polarity = TMR4_PWM_OXH_HOLD_OXL_HOLD;
    (void)TMR4_PWM_Init(CM_TMR4_1, TMR4_PWM_CH_W, &stcTmr4PwmInit);
    TMR4_PWM_SetDeadTimeValue(CM_TMR4_1, TMR4_PWM_CH_W, TMR4_PWM_PDAR_IDX, MOTOR_PWM_DEAD_TIME_COUNT);
    TMR4_PWM_SetDeadTimeValue(CM_TMR4_1, TMR4_PWM_CH_W, TMR4_PWM_PDBR_IDX, MOTOR_PWM_DEAD_TIME_COUNT);

    TMR4_PWM_MainOutputCmd(CM_TMR4_1, ENABLE);

    TMR4_Start(CM_TMR4_1);


}

/*eof*/
