#incldue "Driver_Vison.h"

void Driver_Vision_Init(USART_TypeDef* USARTx,
											BSP_GPIOSource_TypeDef *USART_RX,
											BSP_GPIOSource_TypeDef *USART_TX,
											u8 PreemptionPriority,u8 SubPriority)
{
	BSP_USART_TypeDef VISION_USART;
	VISION_USART.USARTx = USARTx;
	VISION_USART.USART_TX = USART_TX;
	VISION_USART.USART_RX = USART_RX;
	VISION_USART.USART_InitStructure.USART_BaudRate = 230400;									/*������Ϊ230400*/
	VISION_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*�ֳ�Ϊ8λ���ݸ�ʽ*/
	VISION_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*һ��ֹͣλ*/
	VISION_USART.USART_InitStructure.USART_Parity = USART_Parity_Even;				/*żУ��λ*/
	VISION_USART.USART_InitStructure.USART_Mode = USART_Mode_Rx;							/*����ģʽ*/	
	VISION_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
	
	BSP_USART_Init(&VISION_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&VISION_USART);
}
