#include "Driver_powerContol.h"
#include "auto_infantry.h"

void Driver_Power_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef POWER_USART;
	POWER_USART.USARTx = USARTx;
	POWER_USART.USART_RX = USART_RX;
	POWER_USART.USART_TX = USART_TX;
	POWER_USART.USART_InitStructure.USART_BaudRate = 115200;									/*波特率为115200*/
	POWER_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*字长为8位数据格式*/
	POWER_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*一个停止位*/
	POWER_USART.USART_InitStructure.USART_Parity = USART_Parity_No;				/*无校验位*/
	POWER_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*接收/发送模式*/	
	POWER_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*无硬件数据流控制*/	
	
	BSP_USART_Init(&POWER_USART,PreemptionPriority,SubPriority);
	USART_ITConfig(UART7, USART_IT_IDLE, DISABLE); 	//关闭串口空闲中断
	USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);		//开启串口普通接受中断
//	BSP_USART_RX_DMA_Init(&POWER_USART);
//	BSP_USART_TX_DMA_Init(&POWER_USART);	
}
