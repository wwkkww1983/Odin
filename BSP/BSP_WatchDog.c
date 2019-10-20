#include "BSP_WatchDog.h"

u8 WWDG_CNT=0X7F;

/*
***************************************************
��������BSP_IWDG_Init
���ܣ��������Ź���ʼ��
��ڲ�����	prer:��Ƶ��:0~7(ֻ�е�3λ��Ч!)
								��Ƶ����=4*2^prer.�����ֵֻ����256!
	        rlr:�Զ���װ��ֵ,0~0xFFF.
								��װ�ؼĴ���ֵ:��11λ��Ч.
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��ʱ�����(���):Tout=((4*2^prer)*rlr)/32 (ms).
***************************************************
*/
void BSP_IWDG_Init(u8 prer,u16 rlr){
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //ʹ�ܶ�IWDG->PR IWDG->RLR��д
	IWDG_SetPrescaler(prer);	//����IWDG��Ƶϵ��
	IWDG_SetReload(rlr);   		//����IWDGװ��ֵ
	IWDG_ReloadCounter(); 		//reload
	IWDG_Enable();       			//ʹ�ܿ��Ź�
}

/*
***************************************************
��������BSP_IWDG_Feed
���ܣ�ι�������Ź�
��ڲ�������
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void BSP_IWDG_Feed(void){
	IWDG_ReloadCounter();//reload
}

/*
***************************************************
��������BSP_WWDG_Init
���ܣ���ʼ�����ڿ��Ź�
��ڲ�����	tr   :T[6:0],������ֵ 
					wr   :W[6:0],����ֵ 
					fprer:��Ƶϵ����WDGTB��,�����2λ��Ч
					PreemptionPriority����ռ���ȼ�
					SubPriority�������ȼ�
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��Fwwdg=PCLK1/(4096*2^fprer). һ��PCLK1=42Mhz
***************************************************
*/
void BSP_WWDG_Init(u8 tr,u8 wr,u32 fprer,u8 PreemptionPriority,u8 SubPriority){
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*************��ʼ�����ڿ��Ź�ʱ��***************/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,ENABLE);
	
	/*************���ô��ڿ��Ź�***************/
	WWDG_CNT = tr&WWDG_CNT;   //��ʼ��WWDG_CNT. 
	WWDG_SetPrescaler(fprer); //���÷�Ƶֵ
	WWDG_SetWindowValue(wr); 	//���ô���ֵ
//	WWDG_SetCounter(WWDG_CNT);//���ü���ֵ
	WWDG_Enable(WWDG_CNT);  	//�������Ź�
	
	/*************���ô��ڿ��Ź��ж�***************/
	NVIC_InitStructure.NVIC_IRQChannel=WWDG_IRQn;  //���ڿ��Ź��ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;  //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;					//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  //ʹ�ܴ��ڿ��Ź�
	NVIC_Init(&NVIC_InitStructure);
	
	WWDG_ClearFlag();	//�����ǰ�����жϱ�־λ
  WWDG_EnableIT();	//������ǰ�����ж�
}

/*
***************************************************
��������BSP_WWDG_IRQHandler
���ܣ����ڿ��Ź��жϷ������ 
��ע��
***************************************************
*/
void WWDG_IRQHandler(void){
//	WWDG_SetCounter(WWDG_CNT); 	//���贰�ڿ��Ź�ֵ
//	WWDG_ClearFlag();						//�����ǰ�����жϱ�־λ
//	supervisorStateSwitch(STATE_SENSOR_ERROR,ENABLE);
}
