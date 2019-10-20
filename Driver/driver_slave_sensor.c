#include "Driver_Slave_Sensor.h"

void driver_slaveSensorInit(USART_TypeDef* USARTx, BSP_GPIOSource_TypeDef *USART_RX, BSP_GPIOSource_TypeDef *USART_TX, \
														uint32_t baudRate, uint8_t PreemptionPriority, uint8_t SubPriority){
	BSP_USART_TypeDef slaveSensor_USART;
	slaveSensor_USART.USARTx = USARTx;
	slaveSensor_USART.USART_RX = USART_RX;
	slaveSensor_USART.USART_TX = USART_TX;
	slaveSensor_USART.USART_InitStructure.USART_BaudRate = baudRate;							
	slaveSensor_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*�ֳ�Ϊ8λ���ݸ�ʽ*/
	slaveSensor_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;			/*һ��ֹͣλ*/
	slaveSensor_USART.USART_InitStructure.USART_Parity = USART_Parity_No;					/*��У��λ*/
	slaveSensor_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*����/����ģʽ*/	
	slaveSensor_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
	
	BSP_USART_Init(&slaveSensor_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&slaveSensor_USART);	
	BSP_USART_TX_DMA_Init(&slaveSensor_USART);	
}
