#ifndef __DRIVER_DTU_H
#define __DRIVER_DTU_H

#include "bsp.h"
#include "BSP_GPIO.h"


/**********************************DTU初始化串口****************************************/
#define WIRELESS_USARTX										UART7			  //DTU串口号
#define WIRELESS_USARTX_RX_PIN						BSP_GPIOE8	//DTU接收引脚
#define WIRELESS_USARTX_TX_PIN						BSP_GPIOE7	//DTU发送引脚
#define WIRELESS_USART_PreemptionPriority 2						//WIRELESS_USART中断抢占优先级
#define WIRELESS_USART_SubPriority 				0						//WIRELESS_USART中断响应优先级

void Driver_DTU_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority);
void Driver_Upper_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority);

#endif






