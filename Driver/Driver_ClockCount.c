#include "Driver_ClockCount.h"

void clockCountInit(void){	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	BSP_TIM_RCC_Init(TIM5);		                                            //使能定时器TIMx时钟
	
	TIM_TimeBaseInitStructure.TIM_Period 				= CLOCKCOUNT_PERIOD; 			//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler 		= CLOCKCOUNT_PRESCALER;  	//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode 	= TIM_CounterMode_Up;     //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);                               //允许定时器TIMx更新中断
	TIM_Cmd(TIM5,ENABLE); 		                                             //使能定时器TIMx
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;                        //定时器TIMx中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CLOCKCOUNT_PRE; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = CLOCKCOUNT_SUB;        //子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		
}
