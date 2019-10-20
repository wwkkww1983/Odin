#include "Driver_DTU.h"

/*
***************************************************
��������Driver_DTU_Init
���ܣ�����������ʼ��
��ڲ�����	I can understand it at one glance.
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void Driver_DTU_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef WIRELESS_USART;
	WIRELESS_USART.USARTx = USARTx;
	WIRELESS_USART.USART_RX = USART_RX;
	WIRELESS_USART.USART_TX = USART_TX;
	WIRELESS_USART.USART_InitStructure.USART_BaudRate = 500000;									/*������Ϊ500000*/
	WIRELESS_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*�ֳ�Ϊ8λ���ݸ�ʽ*/
	WIRELESS_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*һ��ֹͣλ*/
	WIRELESS_USART.USART_InitStructure.USART_Parity = USART_Parity_No;				/*��У��λ*/
	WIRELESS_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*����/����ģʽ*/	
	WIRELESS_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
	
	BSP_USART_Init(&WIRELESS_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&WIRELESS_USART);																	
}

/*
***************************************************
��������Driver_Upper_Init
���ܣ���λ����ʼ��
��ڲ�����	I can understand it at one glance.
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void Driver_Upper_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef WIRELESS_USART;
	WIRELESS_USART.USARTx = USARTx;
	WIRELESS_USART.USART_RX = USART_RX;
	WIRELESS_USART.USART_TX = USART_TX;
	WIRELESS_USART.USART_InitStructure.USART_BaudRate = 115200;									/*������Ϊ500000*/
	WIRELESS_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*�ֳ�Ϊ8λ���ݸ�ʽ*/
	WIRELESS_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*һ��ֹͣλ*/
	WIRELESS_USART.USART_InitStructure.USART_Parity = USART_Parity_No;				/*��У��λ*/
	WIRELESS_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*����/����ģʽ*/	
	WIRELESS_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
	
	BSP_USART_Init(&WIRELESS_USART,PreemptionPriority,SubPriority);
	USART_ITConfig(USARTx, USART_IT_IDLE, DISABLE);//�رմ��ڿ����ж�
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); //���������ж�
	BSP_USART_RX_DMA_Init(&WIRELESS_USART);																	
}


