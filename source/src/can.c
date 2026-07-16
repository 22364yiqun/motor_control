#include "can.h"

#include "config.h"
#include "control.h"
#include "usart.h"

#include <stddef.h>

// 打印参数
static int32_t g_i32LastCanPrintPosDeg = 0;
static int32_t g_i32LastCanPrintVelDegS = 0;
static int32_t g_i32LastCanPrintTauMNm = 0;
static uint8_t g_u8CanPrintValid = 0U;

static uint8_t g_u8CanAckSeq = 0U;
static uint8_t g_u8CanHeartbeatSeq = 0U;
static uint32_t g_u32LastHeartbeatTick = 0UL;
static uint32_t g_u32LastCanDiagTick = 0UL;
static uint8_t g_u8LastCanDiagValid = 0U;
static uint8_t g_u8LastCanDiagTxErr = 0U;
static uint8_t g_u8LastCanDiagRxErr = 0U;
static uint8_t g_u8LastCanDiagBusOff = 0U;
static uint8_t g_u8LastCanDiagPassive = 0U;
static uint8_t g_u8LastCanDiagLec = 0U;
static uint32_t g_u32LastCanDiagFree = 0UL;
static uint32_t g_u32LastCanDiagOccurred = 0UL;
static uint32_t g_u32LastCanDiagAborted = 0UL;
static uint32_t g_u32LastCanDiagPending = 0UL;
static uint8_t g_u8LastCanDiagComState = 0U;
static uint8_t g_u8LastCanDiagBusLevel = 0U;
static uint8_t g_u8LastCanDiagInit = 0U;
static uint8_t g_u8LastCanDiagAsm = 0U;
static uint8_t g_u8LastCanDiagMon = 0U;
static uint32_t g_u32LastCanDiagTxbar = 0UL;
static int32_t g_i32LastCanDiagTxRet = 0;
static int32_t g_i32LastCanTxRet = 0;

static int16_t App_CANReadI16Le(const uint8_t *data)
{
    // 读取小端16位整数，将两个字节合并为一个16位整数
    uint16_t value;

    value = (uint16_t)data[0];
    value |= ((uint16_t)data[1] << 8U);

    return (int16_t)value;
}

static int32_t App_CANRoundDivS32(int32_t value, int32_t div)
{
    // 对32位整数进行四舍五入除法，返回结果为整数
    if (value >= 0) {
        return (value + (div / 2)) / div;
    }

    return (value - (div / 2)) / div;
}

static void App_CANWriteI16Le(uint8_t *data, int32_t value)
{
    // 将一个32位整数写入小端16位整数的字节数组中，只保留低16位
    const int16_t svalue = (int16_t)value;
    const uint16_t uvalue = (uint16_t)svalue;

    data[0] = (uint8_t)(uvalue & 0xFFU);
    data[1] = (uint8_t)((uvalue >> 8U) & 0xFFU);
}

static char *App_CANAppendStr(char *dst, const char *src)
{
    // 将源字符串追加到目标字符串的末尾，返回新的目标字符串指针
    while (*src != '\0') {
        *dst++ = *src++;
    }

    return dst;
}

static char *App_CANAppendI32(char *dst, int32_t value)
{
    // 将32位整数转换为字符串并追加到目标字符串的末尾，返回新的目标字符串指针
    char tmp[12];
    uint32_t idx = 0U;
    uint32_t out_idx = 0U;
    uint32_t uvalue;

    if (value < 0) {
        *dst++ = '-';
        uvalue = (uint32_t)(-value);
    } else {
        uvalue = (uint32_t)value;
    }

    do {
        tmp[idx++] = (char)('0' + (uvalue % 10U));
        uvalue /= 10U;
    } while (uvalue != 0U);

    while (out_idx < idx) {
        *dst++ = tmp[idx - 1U - out_idx];
        out_idx++;
    }

    return dst;
}

static void App_CANPrintMitCommand(int32_t pos_deg, int32_t vel_deg_s, int32_t tau_ff_mnm)
{
    // 打印MIT命令的参数，如果参数没有变化则不打印
    char line[96];
    char *p;

    if ((g_u8CanPrintValid != 0U) &&
        (g_i32LastCanPrintPosDeg == pos_deg) &&
        (g_i32LastCanPrintVelDegS == vel_deg_s) &&
        (g_i32LastCanPrintTauMNm == tau_ff_mnm)) {
        return;
    }

    g_u8CanPrintValid = 1U;
    g_i32LastCanPrintPosDeg = pos_deg;
    g_i32LastCanPrintVelDegS = vel_deg_s;
    g_i32LastCanPrintTauMNm = tau_ff_mnm;

    p = line;
    p = App_CANAppendStr(p, "CAN CMD outPos=");
    p = App_CANAppendI32(p, pos_deg);
    p = App_CANAppendStr(p, " deg vel=");
    p = App_CANAppendI32(p, vel_deg_s);
    p = App_CANAppendStr(p, " deg/s tauFF=");
    p = App_CANAppendI32(p, tau_ff_mnm);
    p = App_CANAppendStr(p, " mNm\r\n");
    *p = '\0';

    App_USART1SendString(line);
}

static void App_CANPrintDiag(void)
{
    stc_mcan_protocol_status_t protocol_status;
    stc_mcan_error_counter_t error_counter;
    const uint32_t now_tick = Motor_ControlGetTick();
    const uint32_t free_level = MCAN_GetTxFifoFreeLevel(CM_MCAN1);
    const uint32_t occurred = MCAN_GetTxOccurredList(CM_MCAN1);
    const uint32_t aborted = MCAN_GetTxAbortedList(CM_MCAN1);
    const uint32_t pending = CM_MCAN1->TXBRP;
    const uint8_t bus_level = MCAN_GetBusLogicalState(CM_MCAN1);
    const uint8_t init = ((CM_MCAN1->CCCR & MCAN_CCCR_INIT) != 0UL) ? 1U : 0U;
    const uint8_t asm_mode = ((CM_MCAN1->CCCR & MCAN_CCCR_ASM) != 0UL) ? 1U : 0U;
    const uint8_t mon_mode = ((CM_MCAN1->CCCR & MCAN_CCCR_MON) != 0UL) ? 1U : 0U;
    const uint32_t txbar = CM_MCAN1->TXBAR;
    char line[160];
    char *p;
    uint8_t changed;

    if ((now_tick - g_u32LastCanDiagTick) < 20000UL) {
        return;
    }
    g_u32LastCanDiagTick = now_tick;

    if ((MCAN_GetProtocolStatus(CM_MCAN1, &protocol_status) != LL_OK) ||
        (MCAN_GetErrorCounter(CM_MCAN1, &error_counter) != LL_OK)) {
        return;
    }

    changed = (g_u8LastCanDiagValid == 0U) ||
              (g_u8LastCanDiagTxErr != error_counter.u8TxErrorCount) ||
              (g_u8LastCanDiagRxErr != error_counter.u8RxErrorCount) ||
              (g_u8LastCanDiagBusOff != protocol_status.u8BusOffFlag) ||
              (g_u8LastCanDiagPassive != protocol_status.u8ErrorPassiveFlag) ||
              (g_u8LastCanDiagLec != protocol_status.u8LastErrorCode) ||
              (g_u32LastCanDiagFree != free_level) ||
              (g_u32LastCanDiagOccurred != occurred) ||
              (g_u32LastCanDiagAborted != aborted) ||
              (g_u32LastCanDiagPending != pending) ||
              (g_u8LastCanDiagComState != protocol_status.u8ComState) ||
              (g_u8LastCanDiagBusLevel != bus_level) ||
              (g_u8LastCanDiagInit != init) ||
              (g_u8LastCanDiagAsm != asm_mode) ||
              (g_u8LastCanDiagMon != mon_mode) ||
              (g_u32LastCanDiagTxbar != txbar) ||
              (g_i32LastCanDiagTxRet != g_i32LastCanTxRet);

    if (changed == 0U) {
        return;
    }

    g_u8LastCanDiagValid = 1U;
    g_u8LastCanDiagTxErr = error_counter.u8TxErrorCount;
    g_u8LastCanDiagRxErr = error_counter.u8RxErrorCount;
    g_u8LastCanDiagBusOff = protocol_status.u8BusOffFlag;
    g_u8LastCanDiagPassive = protocol_status.u8ErrorPassiveFlag;
    g_u8LastCanDiagLec = protocol_status.u8LastErrorCode;
    g_u32LastCanDiagFree = free_level;
    g_u32LastCanDiagOccurred = occurred;
    g_u32LastCanDiagAborted = aborted;
    g_u32LastCanDiagPending = pending;
    g_u8LastCanDiagComState = protocol_status.u8ComState;
    g_u8LastCanDiagBusLevel = bus_level;
    g_u8LastCanDiagInit = init;
    g_u8LastCanDiagAsm = asm_mode;
    g_u8LastCanDiagMon = mon_mode;
    g_u32LastCanDiagTxbar = txbar;
    g_i32LastCanDiagTxRet = g_i32LastCanTxRet;

    p = line;
    p = App_CANAppendStr(p, "CAN diag free=");
    p = App_CANAppendI32(p, (int32_t)free_level);
    p = App_CANAppendStr(p, " pend=");
    p = App_CANAppendI32(p, (int32_t)pending);
    p = App_CANAppendStr(p, " txok=");
    p = App_CANAppendI32(p, (int32_t)occurred);
    p = App_CANAppendStr(p, " abort=");
    p = App_CANAppendI32(p, (int32_t)aborted);
    p = App_CANAppendStr(p, " txerr=");
    p = App_CANAppendI32(p, (int32_t)error_counter.u8TxErrorCount);
    p = App_CANAppendStr(p, " rxerr=");
    p = App_CANAppendI32(p, (int32_t)error_counter.u8RxErrorCount);
    p = App_CANAppendStr(p, " bo=");
    p = App_CANAppendI32(p, (int32_t)protocol_status.u8BusOffFlag);
    p = App_CANAppendStr(p, " ep=");
    p = App_CANAppendI32(p, (int32_t)protocol_status.u8ErrorPassiveFlag);
    p = App_CANAppendStr(p, " lec=");
    p = App_CANAppendI32(p, (int32_t)protocol_status.u8LastErrorCode);
    p = App_CANAppendStr(p, " act=");
    p = App_CANAppendI32(p, (int32_t)protocol_status.u8ComState);
    p = App_CANAppendStr(p, " bus=");
    p = App_CANAppendI32(p, (int32_t)bus_level);
    p = App_CANAppendStr(p, " init=");
    p = App_CANAppendI32(p, (int32_t)init);
    p = App_CANAppendStr(p, " asm=");
    p = App_CANAppendI32(p, (int32_t)asm_mode);
    p = App_CANAppendStr(p, " mon=");
    p = App_CANAppendI32(p, (int32_t)mon_mode);
    p = App_CANAppendStr(p, " txbar=");
    p = App_CANAppendI32(p, (int32_t)txbar);
    p = App_CANAppendStr(p, " txret=");
    p = App_CANAppendI32(p, g_i32LastCanTxRet);
    p = App_CANAppendStr(p, "\r\n");
    *p = '\0';

    App_USART1SendString(line);
}

static void App_CANPhyEnable(void)
{
#if (APP_CAN_PHY_STB_CONTROL != DDL_OFF)
    stc_gpio_init_t gpio_init;

    (void)GPIO_StructInit(&gpio_init);
    gpio_init.u16PinDir = PIN_DIR_OUT;
    gpio_init.u16PinAttr = PIN_ATTR_DIGITAL;
    (void)GPIO_Init(APP_CAN_PHY_STB_PORT, APP_CAN_PHY_STB_PIN, &gpio_init);

#if (APP_CAN_PHY_STB_ENABLE_LEVEL != 0U)
    GPIO_SetPins(APP_CAN_PHY_STB_PORT, APP_CAN_PHY_STB_PIN);
#else
    GPIO_ResetPins(APP_CAN_PHY_STB_PORT, APP_CAN_PHY_STB_PIN);
#endif
#endif
}

static void App_CANSendMitAck(int32_t pos_x100, int32_t vel_x10, int32_t tau_ff_mnm)
{
    stc_mcan_tx_msg_t tx_msg = {0};

    if (MCAN_GetTxFifoFreeLevel(CM_MCAN1) == 0UL) {
        return;
    }

    tx_msg.ID = APP_CAN_MIT_ACK_STD_ID;
    tx_msg.IDE = MCAN_STD_ID;
    tx_msg.RTR = 0U;
    tx_msg.DLC = MCAN_DLC8;
    tx_msg.ESI = 0U;
    tx_msg.BRS = 0U;
    tx_msg.FDF = 0U;
    tx_msg.u32StoreTxEvent = 0U;
    tx_msg.u32MsgMarker = 0U;
    tx_msg.u32TxBuffer = 0U;
    tx_msg.u32LastTxFifoQueueRequest = 0U;

    App_CANWriteI16Le(&tx_msg.au8Data[0], pos_x100);
    App_CANWriteI16Le(&tx_msg.au8Data[2], vel_x10);
    App_CANWriteI16Le(&tx_msg.au8Data[4], tau_ff_mnm);
    tx_msg.au8Data[6] = APP_CAN_MIT_CMD_CODE;
    tx_msg.au8Data[7] = g_u8CanAckSeq++;

    g_i32LastCanTxRet = MCAN_AddMsgToTxFifoQueue(CM_MCAN1, &tx_msg);
}

static void App_CANSendHeartbeat(void)
{
    stc_mcan_tx_msg_t tx_msg = {0};
    const uint32_t now_tick = Motor_ControlGetTick();

    if ((now_tick - g_u32LastHeartbeatTick) < APP_CAN_HEARTBEAT_TICKS) {
        return;
    }
    g_u32LastHeartbeatTick = now_tick;

    if (MCAN_GetTxFifoFreeLevel(CM_MCAN1) == 0UL) {
        return;
    }

    tx_msg.ID = APP_CAN_MIT_ACK_STD_ID;
    tx_msg.IDE = MCAN_STD_ID;
    tx_msg.RTR = 0U;
    tx_msg.DLC = MCAN_DLC8;
    tx_msg.ESI = 0U;
    tx_msg.BRS = 0U;
    tx_msg.FDF = 0U;
    tx_msg.u32StoreTxEvent = 0U;
    tx_msg.u32MsgMarker = 0U;
    tx_msg.u32TxBuffer = 0U;
    tx_msg.u32LastTxFifoQueueRequest = 0U;
    tx_msg.au8Data[0] = 'H';
    tx_msg.au8Data[1] = 'B';
    tx_msg.au8Data[2] = g_u8CanHeartbeatSeq++;
    tx_msg.au8Data[3] = Motor_ControlIsOffsetDone();
    tx_msg.au8Data[4] = Motor_ControlIsEnabled();
    tx_msg.au8Data[5] = Motor_ControlIsFault();
    tx_msg.au8Data[6] = 0U;
    tx_msg.au8Data[7] = 0U;

    g_i32LastCanTxRet = MCAN_AddMsgToTxFifoQueue(CM_MCAN1, &tx_msg);
}

void CanCfg(void)
{
    stc_mcan_init_t mcan_init;
    stc_mcan_filter_t std_filter;

    App_CANPhyEnable();
    CLK_SetCANClockSrc(CLK_MCAN1, CLK_MCANCLK_XTAL);
    FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_MCAN1, ENABLE);

    (void)MCAN_StructInit(&mcan_init);
#if (APP_CAN_INTERNAL_LOOPBACK != DDL_OFF)
    mcan_init.u32Mode              = MCAN_MD_INTERN_LOOPBACK;
#else
    mcan_init.u32Mode              = MCAN_MD_NORMAL;
#endif
    mcan_init.u32FrameFormat       = MCAN_FRAME_CLASSIC;
    mcan_init.u32AutoRetx          = MCAN_AUTO_RETX_ENABLE;
    mcan_init.u32TxPause           = MCAN_TX_PAUSE_DISABLE;
    mcan_init.u32ProtocolException = MCAN_PROTOCOL_EXP_ENABLE;

    mcan_init.stcBitTime.u32NominalPrescaler     = APP_CAN_NOMINAL_PRESCALER;
    mcan_init.stcBitTime.u32NominalTimeSeg1      = APP_CAN_NOMINAL_TIME_SEG1;
    mcan_init.stcBitTime.u32NominalTimeSeg2      = APP_CAN_NOMINAL_TIME_SEG2;
    mcan_init.stcBitTime.u32NominalSyncJumpWidth = APP_CAN_NOMINAL_SJW;

    mcan_init.stcMsgRam.u32AddrOffset       = 0x0UL;
    mcan_init.stcMsgRam.u32StdFilterNum     = 1U;
    mcan_init.stcMsgRam.u32ExtFilterNum     = 0U;
    mcan_init.stcMsgRam.u32RxFifo0Num       = APP_CAN_RX_FIFO0_NUM;
    mcan_init.stcMsgRam.u32RxFifo0DataSize  = MCAN_DATA_SIZE_8BYTE;
    mcan_init.stcMsgRam.u32RxFifo1Num       = 0U;
    mcan_init.stcMsgRam.u32RxFifo1DataSize  = MCAN_DATA_SIZE_8BYTE;
    mcan_init.stcMsgRam.u32RxBufferNum      = 0U;
    mcan_init.stcMsgRam.u32RxBufferDataSize = MCAN_DATA_SIZE_8BYTE;
    mcan_init.stcMsgRam.u32TxEventNum       = 0U;
    mcan_init.stcMsgRam.u32TxBufferNum      = 0U;
    mcan_init.stcMsgRam.u32TxFifoQueueNum   = APP_CAN_TX_FIFO_NUM;
    mcan_init.stcMsgRam.u32TxFifoQueueMode  = MCAN_TX_FIFO_MD;
    mcan_init.stcMsgRam.u32TxDataSize       = MCAN_DATA_SIZE_8BYTE;

    std_filter.u32IdType        = MCAN_STD_ID;
    std_filter.u32FilterIndex   = 0U;
    std_filter.u32FilterType    = MCAN_FILTER_MASK;
    std_filter.u32FilterConfig  = MCAN_FILTER_TO_RX_FIFO0;
    std_filter.u32FilterId1     = APP_CAN_MIT_CMD_STD_ID;
    std_filter.u32FilterId2     = MCAN_STD_ID_MASK;
    std_filter.u32RxBufferIndex = 0U;

    mcan_init.stcFilter.pstcStdFilterList    = &std_filter;
    mcan_init.stcFilter.pstcExtFilterList    = NULL;
    mcan_init.stcFilter.u32StdFilterConfigNum = 1U;
    mcan_init.stcFilter.u32ExtFilterConfigNum = 0U;

    (void)MCAN_Init(CM_MCAN1, &mcan_init);
    MCAN_GlobalFilterConfig(CM_MCAN1,
                            MCAN_NMF_REJECT,
                            MCAN_NMF_REJECT,
                            MCAN_REMOTE_FRAME_REJECT,
                            MCAN_REMOTE_FRAME_REJECT);
    MCAN_RxFifoOperationModeConfig(CM_MCAN1, MCAN_RX_FIFO0, MCAN_RX_FIFO_OVERWRITE);
    MCAN_Start(CM_MCAN1);
}

void App_CANTask(void)
{
    stc_mcan_rx_msg_t rx_msg;

#if (APP_CAN_HEARTBEAT_ENABLE != DDL_OFF)
    App_CANSendHeartbeat();
#endif
    App_CANPrintDiag();

    while (MCAN_GetRxFifoFillLevel(CM_MCAN1, MCAN_RX_FIFO0) > 0UL) {
        if (MCAN_GetRxMsg(CM_MCAN1, MCAN_RX_FIFO0, &rx_msg) != LL_OK) {
            break;
        }

        if ((rx_msg.IDE == MCAN_STD_ID) &&
            (rx_msg.RTR == 0U) &&
            (rx_msg.ID == APP_CAN_MIT_CMD_STD_ID) &&
            (rx_msg.DLC == APP_CAN_MIT_CMD_DLC) &&
            (rx_msg.au8Data[6] == APP_CAN_MIT_CMD_CODE)) {
            const int32_t pos_x100 = (int32_t)App_CANReadI16Le(&rx_msg.au8Data[0]);
            const int32_t vel_x10 = (int32_t)App_CANReadI16Le(&rx_msg.au8Data[2]);
            const int32_t tau_ff_mnm = (int32_t)App_CANReadI16Le(&rx_msg.au8Data[4]);
            const int32_t pos_deg = App_CANRoundDivS32(pos_x100, 100);
            const int32_t vel_deg_s = App_CANRoundDivS32(vel_x10, 10);

            Motor_ControlSetOutputTargetX100(pos_x100, vel_x10, tau_ff_mnm);
            App_CANSendMitAck(pos_x100, vel_x10, tau_ff_mnm);
            App_CANPrintMitCommand(pos_deg, vel_deg_s, tau_ff_mnm);
        }
    }
}
