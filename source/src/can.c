#include "can.h"

//CANx Config
static void App_CANxCfg(void)
{
    stc_mcan_init_t stcMcanInit;

    /************************* MCAN1***************************/

    /* Enable MCAN1 clock */
    CLK_SetCANClockSrc(CLK_MCAN1, CLK_MCANCLK_XTAL);
    /************************* Configure MCAN1***************************/
    (void)MCAN_StructInit(&stcMcanInit);
    stcMcanInit.u32Mode              = MCAN_MD_NORMAL;
    stcMcanInit.u32FrameFormat       = MCAN_FRAME_CLASSIC;
    stcMcanInit.u32AutoRetx          = MCAN_AUTO_RETX_ENABLE;
    stcMcanInit.u32TxPause           = MCAN_TX_PAUSE_DISABLE;
    stcMcanInit.u32ProtocolException = MCAN_PROTOCOL_EXP_ENABLE;
    /* Classic CAN */
    stcMcanInit.stcBitTime.u32NominalPrescaler     = 1U;
    stcMcanInit.stcBitTime.u32NominalTimeSeg1      = 12U;
    stcMcanInit.stcBitTime.u32NominalTimeSeg2      = 4U;
    stcMcanInit.stcBitTime.u32NominalSyncJumpWidth = 4U;
    /* Message RAM */
    stcMcanInit.stcMsgRam.u32AddrOffset        = 0x0UL;
    stcMcanInit.stcMsgRam.u32StdFilterNum      = 0U;
    stcMcanInit.stcMsgRam.u32ExtFilterNum      = 0U;
    stcMcanInit.stcMsgRam.u32RxFifo0Num        = 0U;
    stcMcanInit.stcMsgRam.u32RxFifo0DataSize   = MCAN_DATA_SIZE_8BYTE;
    stcMcanInit.stcMsgRam.u32RxFifo1Num        = 0U;
    stcMcanInit.stcMsgRam.u32RxFifo1DataSize   = MCAN_DATA_SIZE_8BYTE;
    stcMcanInit.stcMsgRam.u32RxBufferNum       = 0U;
    stcMcanInit.stcMsgRam.u32RxBufferDataSize  = MCAN_DATA_SIZE_8BYTE;
    stcMcanInit.stcMsgRam.u32TxEventNum        = 0U;
    stcMcanInit.stcMsgRam.u32TxBufferNum       = 0U;
    stcMcanInit.stcMsgRam.u32TxFifoQueueNum    = 0U;
    stcMcanInit.stcMsgRam.u32TxFifoQueueMode   = MCAN_TX_FIFO_MD;
    stcMcanInit.stcMsgRam.u32TxDataSize        = MCAN_DATA_SIZE_8BYTE;
    /* Acceptance filter */
    stcMcanInit.stcFilter.u32StdFilterConfigNum = stcMcanInit.stcMsgRam.u32StdFilterNum;
    stcMcanInit.stcFilter.u32ExtFilterConfigNum = stcMcanInit.stcMsgRam.u32ExtFilterNum;

    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_MCAN1, ENABLE);
    (void)MCAN_Init(CM_MCAN1, &stcMcanInit);
}
