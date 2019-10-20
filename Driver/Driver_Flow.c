#include "Driver_Flow.h"
#include "Util.h"

ano_tc_data ano_tc_of;

/////////////////////////////////////////////////////////////////////////////////////
// 	匿名光流数据解算
void ANO_DT_Data_Receive_OF(u8 *data_buf,u8 num)
{
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAA))		return;		//判断帧头
	
	ano_tc_of.function = data_buf[2];
	ano_tc_of.length = data_buf[3];	
	ano_tc_of.mode = data_buf[4];		
	
	if(ano_tc_of.function == 0x51) 			 // 光流信息帧
	{
		if(ano_tc_of.mode == 0)
		{
			ano_tc_of.quality = data_buf[5];
			ano_tc_of.dx = data_buf[6];
			ano_tc_of.dy = data_buf[7];
			ano_tc_of.light = data_buf[8];

			ano_tc_of.dx = (ano_tc_of.dx>127 ? ano_tc_of.dx-255:ano_tc_of.dx);  	//这里单位人为切换到mm			
			ano_tc_of.dy = (ano_tc_of.dy>127 ? ano_tc_of.dy-255:ano_tc_of.dy);
		}			
		if(ano_tc_of.mode == 1)
		{
			ano_tc_of.quality = data_buf[5];
			ano_tc_of.dx2 = data_buf[6]<<8|data_buf[7];
			ano_tc_of.dy2 = data_buf[8]<<8|data_buf[9];
			ano_tc_of.dx_fix = data_buf[10]<<8|data_buf[11];
			ano_tc_of.dy_fix = data_buf[12]<<8|data_buf[13];			

			ano_tc_of.dx2 = (ano_tc_of.dx2>32767 ? ano_tc_of.dx2-65535 : ano_tc_of.dx2);    //这里单位人为切换到mm		
			ano_tc_of.dy2 = (ano_tc_of.dy2>32767 ? ano_tc_of.dy2-65535 : ano_tc_of.dy2);
			ano_tc_of.dx_fix = (ano_tc_of.dx_fix>32767 ? ano_tc_of.dx_fix-65535 : ano_tc_of.dx_fix);
			ano_tc_of.dy_fix = (ano_tc_of.dy_fix>32767 ? ano_tc_of.dy_fix-65535 : ano_tc_of.dy_fix);
			
			ano_tc_of.light = data_buf[14];			
     }
	}
	else if(ano_tc_of.function == 0x52)	 // 高度信息帧
	{
			ano_tc_of.alt = (data_buf[5]<<8|data_buf[6]);		//这里单位人为切换到mm
			ano_tc_of.realAlt = (float) ano_tc_of.alt/100; 
			digitalHi(&ano_tc_of.work);
	}
	else if(ano_tc_of.function == 0x53)  // 惯性数据帧
	{
			ano_tc_of.gyr_x = data_buf[5]<<8|data_buf[6];
			ano_tc_of.gyr_y = data_buf[7]<<8|data_buf[8];
			ano_tc_of.gyr_z = data_buf[9]<<8|data_buf[10];		
		
			ano_tc_of.acc_x = data_buf[11]<<8|data_buf[12];
			ano_tc_of.acc_y = data_buf[13]<<8|data_buf[14];
			ano_tc_of.acc_z = data_buf[15]<<8|data_buf[16];			

			ano_tc_of.gyr_x = ano_tc_of.gyr_x>32767 ? ano_tc_of.gyr_x-65535 : ano_tc_of.gyr_x;
			ano_tc_of.gyr_y = ano_tc_of.gyr_y>32767 ? ano_tc_of.gyr_y-65535 : ano_tc_of.gyr_y;
			ano_tc_of.gyr_z = ano_tc_of.gyr_z>32767 ? ano_tc_of.gyr_z-65535 : ano_tc_of.gyr_z;
		
			ano_tc_of.acc_x = ano_tc_of.acc_x>32767 ? ano_tc_of.acc_x-65535 : ano_tc_of.acc_x;
			ano_tc_of.acc_y = ano_tc_of.acc_y>32767 ? ano_tc_of.acc_y-65535 : ano_tc_of.acc_y;
			ano_tc_of.acc_z = ano_tc_of.acc_z>32767 ? ano_tc_of.acc_z-65535 : ano_tc_of.acc_z;
	}
	else if(ano_tc_of.function == 0x54)  // 姿态数据帧
	{
		if(ano_tc_of.mode == 0)				// 输出欧拉角
		{
			ano_tc_of.rol = data_buf[5]<<8|data_buf[6];
			ano_tc_of.pit = data_buf[7]<<8|data_buf[8];
			ano_tc_of.yaw = data_buf[9]<<8|data_buf[10];
			
			ano_tc_of.rol = ano_tc_of.rol>32767 ? ano_tc_of.rol-65535 : ano_tc_of.rol;
			ano_tc_of.pit = ano_tc_of.pit>32767 ? ano_tc_of.pit-65535 : ano_tc_of.pit;
			ano_tc_of.yaw = ano_tc_of.yaw>32767 ? ano_tc_of.yaw-65535 : ano_tc_of.yaw;
		}
		else if(ano_tc_of.mode == 1)	//输出四元数
		{
			ano_tc_of.s1 = data_buf[5]<<8|data_buf[6];
			ano_tc_of.s2 = data_buf[7]<<8|data_buf[8];
			ano_tc_of.s3 = data_buf[9]<<8|data_buf[10];	
			ano_tc_of.s4 = data_buf[11]<<8|data_buf[12];		

			ano_tc_of.s1 = ano_tc_of.s1>32767 ? ano_tc_of.s1-65535 : ano_tc_of.s1;
			ano_tc_of.s2 = ano_tc_of.s2>32767 ? ano_tc_of.s2-65535 : ano_tc_of.s2;
			ano_tc_of.s3 = ano_tc_of.s3>32767 ? ano_tc_of.s3-65535 : ano_tc_of.s3;
			ano_tc_of.s4 = ano_tc_of.s4>32767 ? ano_tc_of.s4-65535 : ano_tc_of.s4;

		}
	}	
}

void Driver_Flow_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority)
{
	BSP_USART_TypeDef Flow_USART;
	Flow_USART.USARTx = USARTx;
	Flow_USART.USART_RX = USART_RX;
	Flow_USART.USART_TX = USART_TX;
	Flow_USART.USART_InitStructure.USART_BaudRate = 500000;									/*波特率为500000*/
	Flow_USART.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*字长为8位数据格式*/
	Flow_USART.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*一个停止位*/
	Flow_USART.USART_InitStructure.USART_Parity = USART_Parity_No;				/*无校验位*/
	Flow_USART.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*接收/发送模式*/	
	Flow_USART.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*无硬件数据流控制*/	
	
	BSP_USART_Init(&Flow_USART,PreemptionPriority,SubPriority);
	BSP_USART_RX_DMA_Init(&Flow_USART);																		
}

void flowInit(void)
{
	Driver_Flow_Init(FLOW_USARTX,FLOW_USARTX_RX_PIN,FLOW_USARTX_TX_PIN, \
						FLOW_USART_PreemptionPriority,FLOW_USART_SubPriority);
	digitalLo(&ano_tc_of.work);
}

