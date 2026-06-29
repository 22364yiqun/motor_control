/*lic*/
#ifndef __DMA_H__
#define __DMA_H__

#include "hc32_ll.h"

void DmaCfg(void);
void DMA1_Error_Handler(void);
void DMA1_TC0_BTC0_Handler(void);

#endif /* __DMA_H__ */

/*eof*/
