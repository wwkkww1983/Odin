#include "BSP_WatchDog.h"

u8 WWDG_CNT=0X7F;

/*
***************************************************
函数名：BSP_IWDG_Init
功能：独立看门狗初始化
入口参数：	prer:分频数:0~7(只有低3位有效!)
								分频因子=4*2^prer.但最大值只能是256!
	        rlr:自动重装载值,0~0xFFF.
								重装载寄存器值:低11位有效.
返回值：无
应用范围：外部调用
备注：时间计算(大概):Tout=((4*2^prer)*rlr)/32 (ms).
***************************************************
*/
void BSP_IWDG_Init(u8 prer,u16 rlr){
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //使能对IWDG->PR IWDG->RLR的写
	IWDG_SetPrescaler(prer);	//设置IWDG分频系数
	IWDG_SetReload(rlr);   		//设置IWDG装载值
	IWDG_ReloadCounter(); 		//reload
	IWDG_Enable();       			//使能看门狗
}

/*
***************************************************
函数名：BSP_IWDG_Feed
功能：喂独立看门狗
入口参数：无
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void BSP_IWDG_Feed(void){
	IWDG_ReloadCounter();//reload
}

/*
***************************************************
函数名：BSP_WWDG_Init
功能：初始化窗口看门狗
入口参数：	tr   :T[6:0],计数器值 
					wr   :W[6:0],窗口值 
					fprer:分频系数（WDGTB）,仅最低2位有效
					PreemptionPriority：抢占优先级
					SubPriority：子优先级
返回值：无
应用范围：外部调用
备注：Fwwdg=PCLK1/(4096*2^fprer). 一般PCLK1=42Mhz
***************************************************
*/
void BSP_WWDG_Init(u8 tr,u8 wr,u32 fprer,u8 PreemptionPriority,u8 SubPriority){
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*************初始化窗口看门狗时钟***************/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,ENABLE);
	
	/*************配置窗口看门狗***************/
	WWDG_CNT = tr&WWDG_CNT;   //初始化WWDG_CNT. 
	WWDG_SetPrescaler(fprer); //设置分频值
	WWDG_SetWindowValue(wr); 	//设置窗口值
//	WWDG_SetCounter(WWDG_CNT);//设置计数值
	WWDG_Enable(WWDG_CNT);  	//开启看门狗
	
	/*************配置窗口看门狗中断***************/
	NVIC_InitStructure.NVIC_IRQChannel=WWDG_IRQn;  //窗口看门狗中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;  //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;					//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  //使能窗口看门狗
	NVIC_Init(&NVIC_InitStructure);
	
	WWDG_ClearFlag();	//清除提前唤醒中断标志位
  WWDG_EnableIT();	//开启提前唤醒中断
}

/*
***************************************************
函数名：BSP_WWDG_IRQHandler
功能：窗口看门狗中断服务程序 
备注：
***************************************************
*/
void WWDG_IRQHandler(void){
//	WWDG_SetCounter(WWDG_CNT); 	//重设窗口看门狗值
//	WWDG_ClearFlag();						//清除提前唤醒中断标志位
//	supervisorStateSwitch(STATE_SENSOR_ERROR,ENABLE);
}
