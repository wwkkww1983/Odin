#ifndef __DRIVER_VISION_H
#define __DRIVER_VISION_H

#include "bsp.h"

void Driver_Vision_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority);

#endif

