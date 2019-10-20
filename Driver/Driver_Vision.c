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
	VISION_USART.USART_InitStructure.USART_BaudRate = 230400;									/*波特率为230400*/
	VISION_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*字长为8位数据格式*/
	VISION_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*一个停止位*/
	VISION_USART.USART_InitStructure.USART_Parity = USART_Parity_Even;				/*偶校验位*/
	VISION_USART.USART_InitStructure.USART_Mode = USART_Mode_Rx;							/*接收模式*/	
	VISION_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*无硬件数据流控制*/	
	
	BSP_USART_Init(&VISION_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&VISION_USART);
}
