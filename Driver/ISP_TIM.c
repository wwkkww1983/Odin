#include "board.h"
#include "imu.h"

/*
***************************************************
��������TIM2_IRQHandler
���ܣ���ʱ��2�ж�
��ע��
***************************************************
*/
void TIM2_IRQHandler(void)
{
	/*************��ʱ�ж�****************/
	if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		 
		/*********�������Զ��岿��**********/
		
	}
}

/*
***************************************************
��������TIM3_IRQHandler
���ܣ���ʱ��3�ж�
��ע��
***************************************************
*/
void TIM3_IRQHandler(void)
{
	/*************��ʱ�ж�****************/
	if (TIM_GetITStatus(TIM3,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
		 
		/*********�������Զ��岿��**********/
		
	}
}

/*
***************************************************
��������TIM4_IRQHandler
���ܣ���ʱ��4�ж�
��ע��
***************************************************
*/
void TIM4_IRQHandler(void)
{
	/*************��ʱ�ж�****************/
	if (TIM_GetITStatus(TIM4,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		 
		/*********�������Զ��岿��**********/
		
	}
}

/*
***************************************************
��������TIM5_IRQHandler
���ܣ���ʱ��5�ж�
��ע��
***************************************************
*/
void TIM5_IRQHandler(void)
{
	/*************��ʱ�ж�****************/
	if (TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		/*********�������Զ��岿��**********/	
		clockCountData.saveTimer ++;
	}
}

/*
***************************************************
��������TIM6_DAC_IRQHandler
���ܣ���ʱ��6�ж�
��ע��
***************************************************
*/
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		TIM_ClearFlag(TIM6, TIM_FLAG_Update);
		
		/*********�������Զ��岿��**********/
		ulHighFrequencyTimerTicks++;
	}
}

/*
***************************************************
��������TIM7_IRQHandler
���ܣ���ʱ��7�ж�
��ע��
***************************************************
*/
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
		TIM_ClearFlag(TIM7, TIM_FLAG_Update);
		/*********�������Զ��岿��**********/
	}
}

/*
***************************************************
��������TIM11_IRQHandler
���ܣ���ʱ��11�ж�
��ע��
***************************************************
*/
void TIM11_IRQHandler(void)
{
	/*************��ʱ�ж�****************/
	if (TIM_GetITStatus(TIM11,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM11,TIM_IT_Update);
		TIM_ClearFlag(TIM11, TIM_FLAG_Update);
		/*********�������Զ��岿��**********/
		
	}         
}

void TIM1_CC_IRQHandler(void)
{
	
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);	

	}
}

void TIM8_BRK_TIM12_IRQHandler(void)
{

}

void TIM8_UP_TIM13_IRQHandler(void)
{
	
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	
}
