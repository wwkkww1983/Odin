#ifndef __DRIVER_MULTI_MASTER_H
#define __DRIVER_MULTI_MASTER_H

#include "bsp.h"
#include "BSP_GPIO.h"


/**********************************多主控初始化串口****************************************/
#define MULTI_MASTER_USARTX										UART6			  //多主控串口号
#define MULTI_MASTER_USARTX_RX_PIN						BSP_GPIOC7	//多主控接收引脚
#define MULTI_MASTER_USARTX_TX_PIN						BSP_GPIOC6	//多主控发送引脚
#define MULTI_MASTER_USART_PreemptionPriority 2						//MULTI_MASTER_USART中断抢占优先级
#define MULTI_MASTER_USART_SubPriority 				0						//MULTI_MASTER_USART中断响应优先级

void Driver_Multi_Master_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority);

#endif






