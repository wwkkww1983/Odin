#include "application.h"
#include "bsp.h"
#include "driver.h"
#include "power.h"

/*
***************************************************
函数名：USART1_IRQHandler
功能：串口1中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/

void USART1_IRQHandler(void){																	//DMA2-2
	u16 USART1_len;	//接收数据的长度
	//检测是否是空闲中断
	if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET){
		USART1_len = USART1->SR;
		USART1_len = USART1->DR; //清USART_IT_IDLE标志
		
		DMA_Cmd(DMA2_Stream2,DISABLE);    //关闭DMA
		USART1_len = Length_USART1_RX_Buff - DMA2_Stream2->NDTR;	//读取接收数据长度
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	//清空标志位
		while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
		DMA2_Stream2->NDTR = Length_USART1_RX_Buff;
		DMA_Cmd(DMA2_Stream2, ENABLE);
		/*********以下是自定义部分**********/
#if UAV_SBUS		
		if(USART1_len==25){
			digitalHi(&remoteControlData.rcIspReady);
			digitalIncreasing(&remoteControlData.rcError.errorCount);
		}
#else
		if(USART1_len==18){
			digitalHi(&remoteControlData.rcIspReady);
			digitalHi(&controlTransData.otherRcReadly);
			digitalIncreasing(&remoteControlData.rcError.errorCount);
		}
#endif
	}
}


/*
***************************************************
函数名：USART2_IRQHandler
功能：串口2中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/	
void USART2_IRQHandler(void){																		//DMA1-5
	u16 USART2_len;	//接收数据的长度pdFAILpdFALSE
	if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET){
		USART2_len = USART2->SR;
		USART2_len = USART2->DR; //清USART_IT_IDLE标志
		
		DMA_Cmd(DMA1_Stream5,DISABLE);    //关闭DMA
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);	//清空标志位
		while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
		USART2_len = Length_USART2_RX_Buff - DMA1_Stream5->NDTR;	//读取接收数据长度
		DMA1_Stream5->NDTR = Length_USART2_RX_Buff;
		DMA_Cmd(DMA1_Stream5, ENABLE);
		
		/*********以下是自定义部分**********/	
		//裁判系统上电时会发送两次超过512个字节的数据
		if(USART2_len<256){
			u8 i;
			//把DMA数据存到环形buffer
			for(i=0;i<USART2_len;i++){
				bufferLoop.buffer[(u8)(bufferLoop.tail+i)] = Array_USART2_RX[i];
			}
			bufferLoop.tail += USART2_len;
			digitalIncreasing(&judgeData.judgeErrorCount);
		}
	}
}

/*
***************************************************
函数名：USART3_IRQHandler
功能：串口3中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/
void USART3_IRQHandler(void){																		//DMA1-1
	u16 USART3_len;	//接收数据的长度
	if(USART_GetITStatus(USART3,USART_IT_IDLE) == SET){
		USART3_len = USART3->SR;
		USART3_len = USART3->DR; //清USART_IT_IDLE标志
		
		DMA_Cmd(DMA1_Stream1,DISABLE);    //关闭DMA
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);	//清空标志位
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
		USART3_len = Length_USART3_RX_Buff - DMA1_Stream1->NDTR;		//读取接收数据长度
		DMA1_Stream1->NDTR = Length_USART3_RX_Buff;
		DMA_Cmd(DMA1_Stream1, ENABLE);
		
		/*********以下是自定义部分**********/	
		if(USART3_len){
			slaveSensorRead(Array_USART3_RX);
			digitalIncreasing(&slaveSensorData.slaveErrorCount);
		}
	}
}

/*
***************************************************
函数名：UART4_IRQHandler
功能：串口4中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/
void UART4_IRQHandler(void){																			//DMA1-2
	u16 UART4_len;	//接收数据的长度
	if(USART_GetITStatus(UART4,USART_IT_IDLE) == SET){
		UART4_len = UART4->SR;
		UART4_len = UART4->DR; //清USART_IT_IDLE标志
		
		DMA_Cmd(DMA1_Stream2,DISABLE);    //关闭DMA
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	//清空标志位
		while(DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);
		UART4_len = Length_UART4_RX_Buff - DMA1_Stream2->NDTR;				//读取接收数据长度
		DMA1_Stream2->NDTR = Length_UART4_RX_Buff;
		DMA_Cmd(DMA1_Stream2, ENABLE);
		
		/*********以下是自定义部分**********/	
		if(UART4_len){
		};	//这里只是为了减少warning提示，使用时可以删掉
	}
}

/*
***************************************************
函数名：UART5_IRQHandler
功能：串口5中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/
void UART5_IRQHandler(void){																			//DMA1-0
	u16 UART5_len;	//接收数据的长度
	if(USART_GetITStatus(UART5,USART_IT_IDLE) == SET){
		UART5_len = UART5->SR;
		UART5_len = UART5->DR; //清USART_IT_IDLE标志
		
		DMA_Cmd(DMA1_Stream0,DISABLE);    //关闭DMA
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);	//清空标志位
		while(DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);
		UART5_len = Length_UART5_RX_Buff - DMA1_Stream0->NDTR;	//读取接收数据长度
		DMA1_Stream0->NDTR = Length_UART5_RX_Buff;
		DMA_Cmd(DMA1_Stream0, ENABLE);
		
		/*********以下是自定义部分**********/	
		if(UART5_len){
		}
	}
}

/*
***************************************************
函数名：USART6_IRQHandler
功能：串口6中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/
void USART6_IRQHandler(void){																			//DMA2-6
	u16 USART6_len;	//接收数据的长度
	if(USART_GetITStatus(USART6,USART_IT_IDLE) == SET){
		USART6_len = USART6->SR;
		USART6_len = USART6->DR; //清USART_IT_IDLE标志
		
		DMA_Cmd(DMA2_Stream1,DISABLE);    //关闭DMA
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);	//清空标志位
		while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
		USART6_len = Length_USART6_RX_Buff - DMA2_Stream1->NDTR;			//读取接收数据长度
		DMA2_Stream1->NDTR = Length_USART6_RX_Buff;
		DMA_Cmd(DMA2_Stream1, ENABLE);
		
		/*********以下是自定义部分**********/	
		if(USART6_len){
			otherControlRead(Array_USART6_RX);
		}	//这里只是为了减少warning提示，使用时可以删掉
	}
}


/*
***************************************************
函数名：UART7_IRQHandler
功能：串口7中断服务函数
备注：本串口中断为空闲中断+DMA中断
***************************************************
*/
//*
//***************************************************
//函数名：UART7_IRQHandler
//功能：串口7中断服务函数
//备注：本串口中断为空闲中断+DMA中断
//***************************************************

#ifdef USE_WIRELESS
	void UART7_IRQHandler(void){																			//DMA1-3
		u16 UART7_len;	//接收数据的长度
		if(USART_GetITStatus(UART7,USART_IT_IDLE) == SET){
			UART7_len = UART7->SR;
			UART7_len = UART7->DR; //清USART_IT_IDLE标志
			
			DMA_Cmd(DMA1_Stream3,DISABLE);    //关闭DMA
			DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);	//清空标志位
			while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
			UART7_len = Length_UART7_RX_Buff - DMA1_Stream3->NDTR;				//读取接收数据长度
			DMA1_Stream3->NDTR = Length_UART7_RX_Buff;
			DMA_Cmd(DMA1_Stream3, ENABLE);
			
			/*********以下是自定义部分**********/	
			if(UART7_len){
				for(uint16_t i = 0;i < UART7_len;i++)
					ANO_DT_Data_Receive_Prepare(Array_UART7_RX[i]);
			}
		}
	}
#endif

/*
***************************************************
函数名：UART7_IRQHandler
功能：串口7普通中断服务函数
备注：本串口中断为普通中断
***************************************************
*/
#ifdef USE_UPPER
	void UART7_IRQHandler(void)                	
	{
		static u16 RxCount = 0;
		u8 res;
		if(USART_GetITStatus(UART7,USART_IT_RXNE) != RESET)  
		{	
			res = UART7->DR;        /*USART_ReceiveData(UART7); */ //读取接收到的数据
			Array_UART7_RX[RxCount++] = res; 
			if(RxCount==5)
			{
			 RxCount=0;
			 GetupperMonitorReceiveData(Array_UART7_RX);
			}	
		} 
	}
#endif

#ifdef USE_POWER_LIMIT
void UART7_IRQHandler(void)                	
{
	static u16 RxCount = 0;
	u8 res;
	if(USART_GetITStatus(UART7,USART_IT_RXNE) != RESET)  
	{	
		res = UART7->DR;        /*USART_ReceiveData(UART7); */ //读取接收到的数据
		Array_UART7_RX[RxCount++] = res; 
		if(RxCount==5)
		{
		 RxCount=0;
//		 powerDataReceive(Array_UART7_RX);
		}	
	} 
}
#endif


/*
***************************************************
函数名：UART8_IRQHandler
功能：串口8中断服务函数
备注：本串口中断为空闲中断+DMA中断					DMA1-6
***************************************************
*/
/********串口8DMA1数据流与裁判系统串口2DMA数据流冲突*************/

//void UART8_IRQHandler(void){	
//	u16 UART8_len;	//接收数据的长度
//		if(USART_GetITStatus(UART8,USART_IT_IDLE) == SET){
//			UART8_len = UART8->SR;
//			UART8_len = UART8->DR; //清USART_IT_IDLE标志
//			
//			DMA_Cmd(DMA1_Stream6,DISABLE);    //关闭DMA
//			DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6);	//清空标志位
//			while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);
//			UART8_len = Length_UART8_RX_Buff - DMA1_Stream6->NDTR;				//读取接收数据长度
//			DMA1_Stream6->NDTR = Length_UART8_RX_Buff;
//			DMA_Cmd(DMA1_Stream6, ENABLE);
//			
//			/*********以下是自定义部分**********/	
//			if(UART8_len){
//			  rs485ReceiveData(Array_UART8_RX,YAW);
//			}
//		}
//}

/*串口8中断服务函数*/
void UART8_IRQHandler(void){	
static u8 RxCount = 0;	
	if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET){
		Array_UART8_RX[RxCount++] = UART8->DR; 
		if(RxCount == 16){																	//用缓存数组接收
			RxCount = 0;
			rs485ReceiveData(Array_UART8_RX,YAW);
		}
		digitalIncreasing(&gimbalData.gimbalError[0].errorCount);
	}	
}
