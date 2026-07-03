#include "usart.h"

#include "config.h"
#include "control.h"

#include <stddef.h>

static volatile char g_usart1RxRing[APP_USART_RX_RING_SIZE];
static volatile uint16_t g_u16Usart1RxHead = 0U;
static volatile uint16_t g_u16Usart1RxTail = 0U;
static volatile uint8_t g_u8Usart1RxOverflow = 0U;

static volatile char g_usart1TxRing[APP_USART_TX_RING_SIZE];
static volatile uint16_t g_u16Usart1TxHead = 0U;
static volatile uint16_t g_u16Usart1TxTail = 0U;
static volatile uint8_t g_u8Usart1TxOverflow = 0U;

void UsartCfg(void)
{
    stc_usart_uart_init_t uart_init;

    FCG_Fcg3PeriphClockCmd(FCG3_PERIPH_USART1, ENABLE);
    (void)USART_DeInit(CM_USART1);
    (void)USART_UART_StructInit(&uart_init);
    uart_init.u32ClockSrc = USART_CLK_SRC_INTERNCLK;
    uart_init.u32ClockDiv = USART_CLK_DIV1;
    uart_init.u32CKOutput = USART_CK_OUTPUT_DISABLE;
    uart_init.u32Baudrate = 115200UL;
    uart_init.u32DataWidth = USART_DATA_WIDTH_8BIT;
    uart_init.u32StopBit = USART_STOPBIT_1BIT;
    uart_init.u32Parity = USART_PARITY_NONE;
    uart_init.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    uart_init.u32FirstBit = USART_FIRST_BIT_LSB;
    uart_init.u32StartBitPolarity = USART_START_BIT_FALLING;
    uart_init.u32HWFlowControl = USART_HW_FLOWCTRL_NONE;
    (void)USART_UART_Init(CM_USART1, &uart_init, NULL);
    USART_FuncCmd(CM_USART1, (USART_TX | USART_RX | USART_INT_RX), ENABLE);
}

void USART1_ErrorIrqCallback(void)
{
    (void)USART_ReadData(CM_USART1);
    USART_ClearStatus(CM_USART1, (USART_FLAG_PARITY_ERR | USART_FLAG_FRAME_ERR | USART_FLAG_OVERRUN));
}

static void App_USART1RxRingPush(char ch)
{
    const uint16_t next = (uint16_t)((g_u16Usart1RxHead + 1U) % APP_USART_RX_RING_SIZE);

    if (next == g_u16Usart1RxTail) {
        g_u8Usart1RxOverflow = 1U;
    } else {
        g_usart1RxRing[g_u16Usart1RxHead] = ch;
        g_u16Usart1RxHead = next;
    }
}

static int32_t App_USART1RxRingPop(char *ch)
{
    if (g_u16Usart1RxTail == g_u16Usart1RxHead) {
        return LL_ERR;
    }

    *ch = g_usart1RxRing[g_u16Usart1RxTail];
    g_u16Usart1RxTail = (uint16_t)((g_u16Usart1RxTail + 1U) % APP_USART_RX_RING_SIZE);
    return LL_OK;
}

void USART1_RxIrqCallback(void)
{
    while (USART_GetStatus(CM_USART1, USART_FLAG_RX_FULL) == SET) {
        const uint16_t rx_data = USART_ReadData(CM_USART1);
        App_USART1RxRingPush((char)(rx_data & 0xFFU));
    }
}

static uint8_t App_ParseI32Field(const char **str, int32_t *value)
{
    uint8_t negative = 0U;
    uint8_t has_digit = 0U;
    int32_t out = 0;
    const char *p;

    if ((str == NULL) || (*str == NULL) || (value == NULL)) {
        return 0U;
    }

    p = *str;
    if (*p == '-') {
        negative = 1U;
        p++;
    } else if (*p == '+') {
        p++;
    }

    while ((*p >= '0') && (*p <= '9')) {
        has_digit = 1U;
        out = (out * 10) + (int32_t)(*p - '0');
        if (out > 100000) {
            return 0U;
        }
        p++;
    }

    if (has_digit == 0U) {
        return 0U;
    }

    *value = (negative != 0U) ? -out : out;
    *str = p;
    return 1U;
}

/* MIT命令解析：m位置deg[,目标速度deg/s[,前馈力矩mN*m]]，例如 m90 或 m90,0,0。 */
static uint8_t App_ParseMitLine(const char *str,
                                int32_t *position_deg,
                                int32_t *velocity_deg_s,
                                int32_t *tau_ff_mnm)
{
    const char *p = str;

    if ((position_deg == NULL) || (velocity_deg_s == NULL) || (tau_ff_mnm == NULL)) {
        return 0U;
    }

    *velocity_deg_s = 0;
    *tau_ff_mnm = 0;

    if (App_ParseI32Field(&p, position_deg) == 0U) {
        return 0U;
    }

    if (*p == '\0') {
        return 1U;
    }
    if (*p != ',') {
        return 0U;
    }
    p++;

    if (App_ParseI32Field(&p, velocity_deg_s) == 0U) {
        return 0U;
    }

    if (*p == '\0') {
        return 1U;
    }
    if (*p != ',') {
        return 0U;
    }
    p++;

    if (App_ParseI32Field(&p, tau_ff_mnm) == 0U) {
        return 0U;
    }

    return (*p == '\0') ? 1U : 0U;
}

void App_USART1RxCommandTask(void)
{
    static char cmd_buf[APP_USART_CMD_BUF_LEN];
    static uint32_t cmd_len = 0UL;
    char ch;

    if (g_u8Usart1RxOverflow != 0U) {
        g_u8Usart1RxOverflow = 0U;
        cmd_len = 0UL;
        App_USART1SendString("ERR: rx overflow\r\n");
    }

    while (App_USART1RxRingPop(&ch) == LL_OK) {
        if ((ch == '\r') || (ch == '\n')) {
            if (cmd_len > 0UL) {
                char str_cmd[16];
                char ack[96];
                uint32_t idx = 0UL;
                const char *parse_str = cmd_buf;

                cmd_buf[cmd_len] = '\0';
                if ((cmd_buf[0] == 'm') || (cmd_buf[0] == 'M')) {
                    parse_str = &cmd_buf[1];
                } else {
                    parse_str = cmd_buf;
                }

                {
                    int32_t pos_deg;
                    int32_t vel_deg_s;
                    int32_t tau_ff_mnm;

                    if (App_ParseMitLine(parse_str, &pos_deg, &vel_deg_s, &tau_ff_mnm) != 0U) {
#define APPEND_STR(s) do { const char *p = (s); while (*p != '\0') { ack[idx++] = *p++; } } while (0)
                        int32_t pos_print = pos_deg % 360;
                        if (pos_print < 0) {
                            pos_print += 360;
                        }
                        Motor_ControlSetMitTarget(pos_deg, vel_deg_s, tau_ff_mnm);
                        (void)App_I32ToDecStr(str_cmd, pos_print);
                        APPEND_STR("CMD mit pos=");
                        APPEND_STR(str_cmd);
                        (void)App_I32ToDecStr(str_cmd, vel_deg_s);
                        APPEND_STR(" deg vel=");
                        APPEND_STR(str_cmd);
                        (void)App_I32ToDecStr(str_cmd, tau_ff_mnm);
                        APPEND_STR(" deg/s tauFF=");
                        APPEND_STR(str_cmd);
                        APPEND_STR(" mNm\r\n");
                        ack[idx] = '\0';
                        App_USART1SendString(ack);
#undef APPEND_STR
                    } else {
                        App_USART1SendString("ERR: MIT m90 or m90,0,0\r\n");
                    }
                }
            }
            cmd_len = 0UL;
        } else if ((ch == '\b') || (ch == 0x7F)) {
            if (cmd_len > 0UL) {
                cmd_len--;
            }
        } else if ((ch == ' ') || (ch == '\t')) {
            /* Ignore whitespace. */
        } else if (((ch >= '0') && (ch <= '9')) || (ch == '-') || (ch == '+') ||
                   (ch == ',') || (ch == 'm') || (ch == 'M')) {
            if (cmd_len < (APP_USART_CMD_BUF_LEN - 1UL)) {
                cmd_buf[cmd_len++] = ch;
            } else {
                cmd_len = 0UL;
                App_USART1SendString("ERR: command too long\r\n");
            }
        }
    }
}

static void App_USART1TxRingPush(char ch)
{
    const uint16_t next = (uint16_t)((g_u16Usart1TxHead + 1U) % APP_USART_TX_RING_SIZE);

    if (next == g_u16Usart1TxTail) {
        g_u8Usart1TxOverflow = 1U;
        return;
    }

    g_usart1TxRing[g_u16Usart1TxHead] = ch;
    g_u16Usart1TxHead = next;
}

static int32_t App_USART1TxRingPop(char *ch)
{
    if (g_u16Usart1TxTail == g_u16Usart1TxHead) {
        return LL_ERR;
    }

    *ch = g_usart1TxRing[g_u16Usart1TxTail];
    g_u16Usart1TxTail = (uint16_t)((g_u16Usart1TxTail + 1U) % APP_USART_TX_RING_SIZE);
    return LL_OK;
}

void App_USART1TxPumpInMainLoop(void)
{
    uint32_t sent = 0UL;
    char ch;

    while ((sent < APP_USART_TX_PUMP_BYTES) &&
           (USART_GetStatus(CM_USART1, USART_FLAG_TX_EMPTY) == SET) &&
           (App_USART1TxRingPop(&ch) == LL_OK)) {
        USART_WriteData(CM_USART1, (uint16_t)ch);
        sent++;
    }
}

void App_USART1SendString(const char *str)
{
    const char *p = str;

    if (str == NULL) {
        return;
    }

    while (*p != '\0') {
        App_USART1TxRingPush(*p);
        p++;
    }
}

char *App_U32ToDecStr(char *str, uint32_t value)
{
    char tmp[10];
    uint32_t in_idx = 0UL;
    uint32_t out_idx = 0UL;

    if (value == 0UL) {
        str[0] = '0';
        str[1] = '\0';
        return str;
    }

    while (value > 0UL) {
        tmp[in_idx++] = (char)('0' + (value % 10UL));
        value /= 10UL;
    }

    while (in_idx > 0UL) {
        str[out_idx++] = tmp[--in_idx];
    }
    str[out_idx] = '\0';
    return str;
}

char *App_I32ToDecStr(char *str, int32_t value)
{
    uint32_t abs_value;

    if (value < 0) {
        str[0] = '-';
        abs_value = (uint32_t)(-(value + 1)) + 1UL;
        (void)App_U32ToDecStr(&str[1], abs_value);
    } else {
        (void)App_U32ToDecStr(str, (uint32_t)value);
    }

    return str;
}
