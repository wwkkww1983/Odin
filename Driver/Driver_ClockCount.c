#include "Driver_ClockCount.h"

void clockCountInit(void){	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	BSP_TIM_RCC_Init(TIM5);		                                            //ʹ�ܶ�ʱ��TIMxʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period 				= CLOCKCOUNT_PERIOD; 			//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler 		= CLOCKCOUNT_PRESCALER;  	//��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode 	= TIM_CounterMode_Up;     //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);                               //����ʱ��TIMx�����ж�
	TIM_Cmd(TIM5,ENABLE); 		                                             //ʹ�ܶ�ʱ��TIMx
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;                        //��ʱ��TIMx�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = CLOCKCOUNT_PRE; //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = CLOCKCOUNT_SUB;        //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		
}
