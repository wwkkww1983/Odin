#ifndef __DRIVER_DTU_H
#define __DRIVER_DTU_H

#include "bsp.h"
#include "BSP_GPIO.h"


/**********************************DTU��ʼ������****************************************/
#define WIRELESS_USARTX										UART7			  //DTU���ں�
#define WIRELESS_USARTX_RX_PIN						BSP_GPIOE8	//DTU��������
#define WIRELESS_USARTX_TX_PIN						BSP_GPIOE7	//DTU��������
#define WIRELESS_USART_PreemptionPriority 2						//WIRELESS_USART�ж���ռ���ȼ�
#define WIRELESS_USART_SubPriority 				0						//WIRELESS_USART�ж���Ӧ���ȼ�

void Driver_DTU_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority);
void Driver_Upper_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority);

#endif






