#include "Driver_powerContol.h"
#include "auto_infantry.h"

void Driver_Power_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef POWER_USART;
	POWER_USART.USARTx = USARTx;
	POWER_USART.USART_RX = USART_RX;
	POWER_USART.USART_TX = USART_TX;
	POWER_USART.USART_InitStructure.USART_BaudRate = 115200;									/*������Ϊ115200*/
	POWER_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*�ֳ�Ϊ8λ���ݸ�ʽ*/
	POWER_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*һ��ֹͣλ*/
	POWER_USART.USART_InitStructure.USART_Parity = USART_Parity_No;				/*��У��λ*/
	POWER_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*����/����ģʽ*/	
	POWER_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
	
	BSP_USART_Init(&POWER_USART,PreemptionPriority,SubPriority);
	USART_ITConfig(UART7, USART_IT_IDLE, DISABLE); 	//�رմ��ڿ����ж�
	USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);		//����������ͨ�����ж�
//	BSP_USART_RX_DMA_Init(&POWER_USART);
//	BSP_USART_TX_DMA_Init(&POWER_USART);	
}
