#include "Driver_RMDT7.h"
#include "rc.h"
#include "Util.h"
/******************************外部调用函数************************************/
void Driver_RMDT7_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,u8 PreemptionPriority,u8 SubPriority);
void Driver_RMDT7_Decode_RemoteData(dt7RcSturct_t *RC_Ctrl,u8 *Array_Dbus);
/*****************************************************************************/


//BSP_USART_TypeDef DBUS_USART = DBUS_USART_DEFAULT;
/*
***************************************************
函数名：Driver_RMDT7_Init
功能：RM遥控器接收机初始化
入口参数：	无
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void Driver_RMDT7_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef DBUS_USART;
	DBUS_USART.USARTx = USARTx;
	DBUS_USART.USART_TX = NULL;
	DBUS_USART.USART_RX = USART_RX;
	DBUS_USART.USART_InitStructure.USART_BaudRate = 100000;									/*波特率为100000*/
	DBUS_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*字长为8位数据格式*/
	DBUS_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*一个停止位*/
	DBUS_USART.USART_InitStructure.USART_Parity = USART_Parity_Even;				/*偶校验位*/
	DBUS_USART.USART_InitStructure.USART_Mode = USART_Mode_Rx;							/*接收模式*/	
	DBUS_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*无硬件数据流控制*/	
	
	BSP_USART_Init(&DBUS_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&DBUS_USART);
}

/*
***************************************************
函 数 名：	Driver_RMDT7_Decode_RemoteData
功		能：	DR16/DT7遥控器协议解码
入口参数：	RC_Ctrl：遥控数据结构体指针
						Array_Dbus：Dbus数据接收数组指针
返 回 值：	无
应用范围：	外部调用
备		注：
***************************************************
*/
void Driver_RMDT7_Decode_RemoteData(dt7RcSturct_t *RC_Ctrl,u8 *Array_Dbus){	
	/******************* 遥控器摇杆数据 *******************/
	RC_Ctrl->rcRawData.CH0 = ((Array_Dbus[0] | Array_Dbus[1]<<8) & 0x07FF) - REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->rcRawData.CH1 = ((Array_Dbus[1]>>3 | Array_Dbus[2]<<5 ) & 0x07FF) - REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->rcRawData.CH2 = (((Array_Dbus[2]>>6 | Array_Dbus[3]<<2 | Array_Dbus[4]<<10) & 0x07FF) & 0x07FF) - REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->rcRawData.CH3 = ((Array_Dbus[4]>>1 | Array_Dbus[5]<<7) & 0x07FF) - REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->rcRawData.CH4 = ((Array_Dbus[16] | Array_Dbus[17]<<8) & 0x07FF) - REMOTE_CONTROLLER_STICK_OFFSET;
	if(RC_Ctrl->rcRawData.CH0<10&&RC_Ctrl->rcRawData.CH0>-10)RC_Ctrl->rcRawData.CH0 = 0;
	if(RC_Ctrl->rcRawData.CH1<10&&RC_Ctrl->rcRawData.CH1>-10)RC_Ctrl->rcRawData.CH1 = 0;
	if(RC_Ctrl->rcRawData.CH2<10&&RC_Ctrl->rcRawData.CH2>-10)RC_Ctrl->rcRawData.CH2 = 0;
	if(RC_Ctrl->rcRawData.CH3<10&&RC_Ctrl->rcRawData.CH3>-10)RC_Ctrl->rcRawData.CH3 = 0;
	if(RC_Ctrl->rcRawData.CH4<10&&RC_Ctrl->rcRawData.CH4>-10)RC_Ctrl->rcRawData.CH4 = 0;
	RC_Ctrl->rcRawData.S1 = ((Array_Dbus[5] >> 4) & 0x000C) >> 2;
	RC_Ctrl->rcRawData.S2 = ((Array_Dbus[5] >> 4) & 0x0003);
	
	/******************* 鼠标数据 *******************/
	RC_Ctrl->mouse.X = (Array_Dbus[6] | (Array_Dbus[7] << 8));
	RC_Ctrl->mouse.Y = (Array_Dbus[8] | (Array_Dbus[9] << 8));
	RC_Ctrl->mouse.Z = Array_Dbus[10]| (Array_Dbus[11] << 8);
	
	RC_Ctrl->mouse.X_Last = RC_Ctrl->mouse.X;
	RC_Ctrl->mouse.Press_L = Array_Dbus[12];
	RC_Ctrl->mouse.Press_R = Array_Dbus[13];
	
	/******************* 键盘数据 *******************/
	RC_Ctrl->keyBoard.key_code = Array_Dbus[14] | Array_Dbus[15] << 8;
}

/*
***************************************************
函 数 名：	Driver_SBUS_Decode_RemoteData
功		能：	普通SBUS协议解码
入口参数：	RC_Ctrl：遥控数据结构体指针
						Array_Dbus：Sbus数据接收数组指针
返 回 值：	无
应用范围：	外部调用
备		注：
***************************************************
*/
void Driver_SBUS_Decode_RemoteData(sbusStruct_t *RC_Ctrl,u8 *Array_Dbus){
	/******************* 遥控器摇杆数据 *******************/
	RC_Ctrl->CH0 = ((Array_Dbus[1] | Array_Dbus[2]<<8) & 0x07FF) - REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH1 = ((Array_Dbus[2]>>3 | Array_Dbus[3]<<5 ) & 0x07FF) - REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH2 = ((Array_Dbus[3]>>6 | Array_Dbus[4]<<2 | Array_Dbus[5]<<10) & 0x07FF) - REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH3 = ((Array_Dbus[5]>>1 | Array_Dbus[6]<<7) & 0x07FF) - REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH4 = ((Array_Dbus[6]>>4 | Array_Dbus[7]<<4) & 0x07ff) - REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH5 = ((Array_Dbus[7]>>7 | Array_Dbus[8]<<1 | Array_Dbus[9]<<9) & 0x07ff) -REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH6 = ((Array_Dbus[9]>>2 | Array_Dbus[10]<<6) & 0x07ff) -REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH7 = ((Array_Dbus[10]>>5| Array_Dbus[11]<<3) & 0x07ff) -REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH8 = ((Array_Dbus[12] | Array_Dbus[13]<<8) & 0x07ff) -REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH9 = ((Array_Dbus[14] | Array_Dbus[15]<<8) & 0x07ff) -REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH10 = ((Array_Dbus[15]>>3 | Array_Dbus[16]<<5) & 0x07ff) -REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH11 = ((Array_Dbus[16]>>6 | Array_Dbus[17]<<2 | Array_Dbus[18]<<10) & 0x07ff) -REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH12 = ((Array_Dbus[19]>>1 | Array_Dbus[20]<<7) & 0x07ff) -REMOTE_CONTROLLER_STICK_OFFSET;
	RC_Ctrl->CH13 = ((Array_Dbus[20]>>4 | Array_Dbus[21]<<4) & 0x07ff) -REMOTE_CONTROLLER_STICK_OFFSET;
}

