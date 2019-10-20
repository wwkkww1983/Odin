#include "application.h"
#include "bsp.h"
#include "driver.h"
#include "power.h"

/*
***************************************************
��������USART1_IRQHandler
���ܣ�����1�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/

void USART1_IRQHandler(void){																	//DMA2-2
	u16 USART1_len;	//�������ݵĳ���
	//����Ƿ��ǿ����ж�
	if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET){
		USART1_len = USART1->SR;
		USART1_len = USART1->DR; //��USART_IT_IDLE��־
		
		DMA_Cmd(DMA2_Stream2,DISABLE);    //�ر�DMA
		USART1_len = Length_USART1_RX_Buff - DMA2_Stream2->NDTR;	//��ȡ�������ݳ���
		DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	//��ձ�־λ
		while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
		DMA2_Stream2->NDTR = Length_USART1_RX_Buff;
		DMA_Cmd(DMA2_Stream2, ENABLE);
		/*********�������Զ��岿��**********/
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
��������USART2_IRQHandler
���ܣ�����2�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/	
void USART2_IRQHandler(void){																		//DMA1-5
	u16 USART2_len;	//�������ݵĳ���pdFAILpdFALSE
	if(USART_GetITStatus(USART2,USART_IT_IDLE) == SET){
		USART2_len = USART2->SR;
		USART2_len = USART2->DR; //��USART_IT_IDLE��־
		
		DMA_Cmd(DMA1_Stream5,DISABLE);    //�ر�DMA
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);	//��ձ�־λ
		while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
		USART2_len = Length_USART2_RX_Buff - DMA1_Stream5->NDTR;	//��ȡ�������ݳ���
		DMA1_Stream5->NDTR = Length_USART2_RX_Buff;
		DMA_Cmd(DMA1_Stream5, ENABLE);
		
		/*********�������Զ��岿��**********/	
		//����ϵͳ�ϵ�ʱ�ᷢ�����γ���512���ֽڵ�����
		if(USART2_len<256){
			u8 i;
			//��DMA���ݴ浽����buffer
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
��������USART3_IRQHandler
���ܣ�����3�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/
void USART3_IRQHandler(void){																		//DMA1-1
	u16 USART3_len;	//�������ݵĳ���
	if(USART_GetITStatus(USART3,USART_IT_IDLE) == SET){
		USART3_len = USART3->SR;
		USART3_len = USART3->DR; //��USART_IT_IDLE��־
		
		DMA_Cmd(DMA1_Stream1,DISABLE);    //�ر�DMA
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);	//��ձ�־λ
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
		USART3_len = Length_USART3_RX_Buff - DMA1_Stream1->NDTR;		//��ȡ�������ݳ���
		DMA1_Stream1->NDTR = Length_USART3_RX_Buff;
		DMA_Cmd(DMA1_Stream1, ENABLE);
		
		/*********�������Զ��岿��**********/	
		if(USART3_len){
			slaveSensorRead(Array_USART3_RX);
			digitalIncreasing(&slaveSensorData.slaveErrorCount);
		}
	}
}

/*
***************************************************
��������UART4_IRQHandler
���ܣ�����4�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/
void UART4_IRQHandler(void){																			//DMA1-2
	u16 UART4_len;	//�������ݵĳ���
	if(USART_GetITStatus(UART4,USART_IT_IDLE) == SET){
		UART4_len = UART4->SR;
		UART4_len = UART4->DR; //��USART_IT_IDLE��־
		
		DMA_Cmd(DMA1_Stream2,DISABLE);    //�ر�DMA
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	//��ձ�־λ
		while(DMA_GetCmdStatus(DMA1_Stream2) != DISABLE);
		UART4_len = Length_UART4_RX_Buff - DMA1_Stream2->NDTR;				//��ȡ�������ݳ���
		DMA1_Stream2->NDTR = Length_UART4_RX_Buff;
		DMA_Cmd(DMA1_Stream2, ENABLE);
		
		/*********�������Զ��岿��**********/	
		if(UART4_len){
		};	//����ֻ��Ϊ�˼���warning��ʾ��ʹ��ʱ����ɾ��
	}
}

/*
***************************************************
��������UART5_IRQHandler
���ܣ�����5�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/
void UART5_IRQHandler(void){																			//DMA1-0
	u16 UART5_len;	//�������ݵĳ���
	if(USART_GetITStatus(UART5,USART_IT_IDLE) == SET){
		UART5_len = UART5->SR;
		UART5_len = UART5->DR; //��USART_IT_IDLE��־
		
		DMA_Cmd(DMA1_Stream0,DISABLE);    //�ر�DMA
		DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);	//��ձ�־λ
		while(DMA_GetCmdStatus(DMA1_Stream0) != DISABLE);
		UART5_len = Length_UART5_RX_Buff - DMA1_Stream0->NDTR;	//��ȡ�������ݳ���
		DMA1_Stream0->NDTR = Length_UART5_RX_Buff;
		DMA_Cmd(DMA1_Stream0, ENABLE);
		
		/*********�������Զ��岿��**********/	
		if(UART5_len){
		}
	}
}

/*
***************************************************
��������USART6_IRQHandler
���ܣ�����6�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/
void USART6_IRQHandler(void){																			//DMA2-6
	u16 USART6_len;	//�������ݵĳ���
	if(USART_GetITStatus(USART6,USART_IT_IDLE) == SET){
		USART6_len = USART6->SR;
		USART6_len = USART6->DR; //��USART_IT_IDLE��־
		
		DMA_Cmd(DMA2_Stream1,DISABLE);    //�ر�DMA
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);	//��ձ�־λ
		while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
		USART6_len = Length_USART6_RX_Buff - DMA2_Stream1->NDTR;			//��ȡ�������ݳ���
		DMA2_Stream1->NDTR = Length_USART6_RX_Buff;
		DMA_Cmd(DMA2_Stream1, ENABLE);
		
		/*********�������Զ��岿��**********/	
		if(USART6_len){
			otherControlRead(Array_USART6_RX);
		}	//����ֻ��Ϊ�˼���warning��ʾ��ʹ��ʱ����ɾ��
	}
}


/*
***************************************************
��������UART7_IRQHandler
���ܣ�����7�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�
***************************************************
*/
//*
//***************************************************
//��������UART7_IRQHandler
//���ܣ�����7�жϷ�����
//��ע���������ж�Ϊ�����ж�+DMA�ж�
//***************************************************

#ifdef USE_WIRELESS
	void UART7_IRQHandler(void){																			//DMA1-3
		u16 UART7_len;	//�������ݵĳ���
		if(USART_GetITStatus(UART7,USART_IT_IDLE) == SET){
			UART7_len = UART7->SR;
			UART7_len = UART7->DR; //��USART_IT_IDLE��־
			
			DMA_Cmd(DMA1_Stream3,DISABLE);    //�ر�DMA
			DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);	//��ձ�־λ
			while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
			UART7_len = Length_UART7_RX_Buff - DMA1_Stream3->NDTR;				//��ȡ�������ݳ���
			DMA1_Stream3->NDTR = Length_UART7_RX_Buff;
			DMA_Cmd(DMA1_Stream3, ENABLE);
			
			/*********�������Զ��岿��**********/	
			if(UART7_len){
				for(uint16_t i = 0;i < UART7_len;i++)
					ANO_DT_Data_Receive_Prepare(Array_UART7_RX[i]);
			}
		}
	}
#endif

/*
***************************************************
��������UART7_IRQHandler
���ܣ�����7��ͨ�жϷ�����
��ע���������ж�Ϊ��ͨ�ж�
***************************************************
*/
#ifdef USE_UPPER
	void UART7_IRQHandler(void)                	
	{
		static u16 RxCount = 0;
		u8 res;
		if(USART_GetITStatus(UART7,USART_IT_RXNE) != RESET)  
		{	
			res = UART7->DR;        /*USART_ReceiveData(UART7); */ //��ȡ���յ�������
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
		res = UART7->DR;        /*USART_ReceiveData(UART7); */ //��ȡ���յ�������
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
��������UART8_IRQHandler
���ܣ�����8�жϷ�����
��ע���������ж�Ϊ�����ж�+DMA�ж�					DMA1-6
***************************************************
*/
/********����8DMA1�����������ϵͳ����2DMA��������ͻ*************/

//void UART8_IRQHandler(void){	
//	u16 UART8_len;	//�������ݵĳ���
//		if(USART_GetITStatus(UART8,USART_IT_IDLE) == SET){
//			UART8_len = UART8->SR;
//			UART8_len = UART8->DR; //��USART_IT_IDLE��־
//			
//			DMA_Cmd(DMA1_Stream6,DISABLE);    //�ر�DMA
//			DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6);	//��ձ�־λ
//			while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);
//			UART8_len = Length_UART8_RX_Buff - DMA1_Stream6->NDTR;				//��ȡ�������ݳ���
//			DMA1_Stream6->NDTR = Length_UART8_RX_Buff;
//			DMA_Cmd(DMA1_Stream6, ENABLE);
//			
//			/*********�������Զ��岿��**********/	
//			if(UART8_len){
//			  rs485ReceiveData(Array_UART8_RX,YAW);
//			}
//		}
//}

/*����8�жϷ�����*/
void UART8_IRQHandler(void){	
static u8 RxCount = 0;	
	if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET){
		Array_UART8_RX[RxCount++] = UART8->DR; 
		if(RxCount == 16){																	//�û����������
			RxCount = 0;
			rs485ReceiveData(Array_UART8_RX,YAW);
		}
		digitalIncreasing(&gimbalData.gimbalError[0].errorCount);
	}	
}
