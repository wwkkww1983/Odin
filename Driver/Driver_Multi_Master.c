#include "Driver_Multi_Master.h"

/*
***************************************************
��������Driver_Multi_Master_Init
���ܣ�����������ʼ��
��ڲ�����	I can understand it at one glance.
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void Driver_Multi_Master_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef MULTI_MASTER_USART;
	MULTI_MASTER_USART.USARTx = USARTx;
	MULTI_MASTER_USART.USART_RX = USART_RX;
	MULTI_MASTER_USART.USART_TX = USART_TX;
	MULTI_MASTER_USART.USART_InitStructure.USART_BaudRate = 115200;									/*������Ϊ115200*/
	MULTI_MASTER_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*�ֳ�Ϊ8λ���ݸ�ʽ*/
	MULTI_MASTER_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*һ��ֹͣλ*/
	MULTI_MASTER_USART.USART_InitStructure.USART_Parity = USART_Parity_No;				/*��У��λ*/
	MULTI_MASTER_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*����/����ģʽ*/	
	MULTI_MASTER_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
	
	BSP_USART_Init(&MULTI_MASTER_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&MULTI_MASTER_USART);																	
}

