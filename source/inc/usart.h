#ifndef __USART_H__
#define __USART_H__

#include "hc32_ll.h"
#include <stdint.h>

void UsartCfg(void);
void USART1_RxIrqCallback(void);
void USART1_ErrorIrqCallback(void);
void App_USART1SendString(const char *str);
void App_USART1TxPumpInMainLoop(void);
void App_USART1RxCommandTask(void);
char *App_U32ToDecStr(char *str, uint32_t value);
char *App_I32ToDecStr(char *str, int32_t value);

#endif /* __USART_H__ */
