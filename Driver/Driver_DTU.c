#include "Driver_DTU.h"

/*
***************************************************
函数名：Driver_DTU_Init
功能：无线数传初始化
入口参数：	I can understand it at one glance.
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void Driver_DTU_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef WIRELESS_USART;
	WIRELESS_USART.USARTx = USARTx;
	WIRELESS_USART.USART_RX = USART_RX;
	WIRELESS_USART.USART_TX = USART_TX;
	WIRELESS_USART.USART_InitStructure.USART_BaudRate = 500000;									/*波特率为500000*/
	WIRELESS_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*字长为8位数据格式*/
	WIRELESS_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*一个停止位*/
	WIRELESS_USART.USART_InitStructure.USART_Parity = USART_Parity_No;				/*无校验位*/
	WIRELESS_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*接收/发送模式*/	
	WIRELESS_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*无硬件数据流控制*/	
	
	BSP_USART_Init(&WIRELESS_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&WIRELESS_USART);																	
}

/*
***************************************************
函数名：Driver_Upper_Init
功能：上位机初始化
入口参数：	I can understand it at one glance.
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void Driver_Upper_Init(USART_TypeDef *USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef WIRELESS_USART;
	WIRELESS_USART.USARTx = USARTx;
	WIRELESS_USART.USART_RX = USART_RX;
	WIRELESS_USART.USART_TX = USART_TX;
	WIRELESS_USART.USART_InitStructure.USART_BaudRate = 115200;									/*波特率为500000*/
	WIRELESS_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*字长为8位数据格式*/
	WIRELESS_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*一个停止位*/
	WIRELESS_USART.USART_InitStructure.USART_Parity = USART_Parity_No;				/*无校验位*/
	WIRELESS_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*接收/发送模式*/	
	WIRELESS_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*无硬件数据流控制*/	
	
	BSP_USART_Init(&WIRELESS_USART,PreemptionPriority,SubPriority);
	USART_ITConfig(USARTx, USART_IT_IDLE, DISABLE);//关闭串口空闲中断
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); //开启接收中断
	BSP_USART_RX_DMA_Init(&WIRELESS_USART);																	
}


