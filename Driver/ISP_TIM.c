#include "board.h"
#include "imu.h"

/*
***************************************************
函数名：TIM2_IRQHandler
功能：定时器2中断
备注：
***************************************************
*/
void TIM2_IRQHandler(void)
{
	/*************定时中断****************/
	if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		 
		/*********以下是自定义部分**********/
		
	}
}

/*
***************************************************
函数名：TIM3_IRQHandler
功能：定时器3中断
备注：
***************************************************
*/
void TIM3_IRQHandler(void)
{
	/*************定时中断****************/
	if (TIM_GetITStatus(TIM3,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
		 
		/*********以下是自定义部分**********/
		
	}
}

/*
***************************************************
函数名：TIM4_IRQHandler
功能：定时器4中断
备注：
***************************************************
*/
void TIM4_IRQHandler(void)
{
	/*************定时中断****************/
	if (TIM_GetITStatus(TIM4,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);
		 
		/*********以下是自定义部分**********/
		
	}
}

/*
***************************************************
函数名：TIM5_IRQHandler
功能：定时器5中断
备注：
***************************************************
*/
void TIM5_IRQHandler(void)
{
	/*************定时中断****************/
	if (TIM_GetITStatus(TIM5,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);
		/*********以下是自定义部分**********/	
		clockCountData.saveTimer ++;
	}
}

/*
***************************************************
函数名：TIM6_DAC_IRQHandler
功能：定时器6中断
备注：
***************************************************
*/
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
		TIM_ClearFlag(TIM6, TIM_FLAG_Update);
		
		/*********以下是自定义部分**********/
		ulHighFrequencyTimerTicks++;
	}
}

/*
***************************************************
函数名：TIM7_IRQHandler
功能：定时器7中断
备注：
***************************************************
*/
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
		TIM_ClearFlag(TIM7, TIM_FLAG_Update);
		/*********以下是自定义部分**********/
	}
}

/*
***************************************************
函数名：TIM11_IRQHandler
功能：定时器11中断
备注：
***************************************************
*/
void TIM11_IRQHandler(void)
{
	/*************定时中断****************/
	if (TIM_GetITStatus(TIM11,TIM_IT_Update)!= RESET) 
	{
		TIM_ClearITPendingBit(TIM11,TIM_IT_Update);
		TIM_ClearFlag(TIM11, TIM_FLAG_Update);
		/*********以下是自定义部分**********/
		
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
