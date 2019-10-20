#ifndef __DRIVER_MULTI_MASTER_H
#define __DRIVER_MULTI_MASTER_H

#include "bsp.h"
#include "BSP_GPIO.h"


/**********************************�����س�ʼ������****************************************/
#define MULTI_MASTER_USARTX										UART6			  //�����ش��ں�
#define MULTI_MASTER_USARTX_RX_PIN						BSP_GPIOC7	//�����ؽ�������
#define MULTI_MASTER_USARTX_TX_PIN						BSP_GPIOC6	//�����ط�������
#define MULTI_MASTER_USART_PreemptionPriority 2						//MULTI_MASTER_USART�ж���ռ���ȼ�
#define MULTI_MASTER_USART_SubPriority 				0						//MULTI_MASTER_USART�ж���Ӧ���ȼ�

void Driver_Multi_Master_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority);

#endif






