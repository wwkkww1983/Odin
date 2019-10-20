
#include "Driver_Motor_Dshot.h"
#include "math.h"
#include "Driver_Motor.h"
#include "BSP.h"
#include "parameter.h"

motorDma_t motorDmaBuffer[MOTORS_NUM];

uint32_t getDshotHz(motorPwmProtocolTypes_e pwmProtocolType){									//得到当前Dshot的频率
	switch (pwmProtocolType) {
	case(PWM_TYPE_PROSHOT1000):
			return MOTOR_PROSHOT1000_HZ;
	case(PWM_TYPE_DSHOT1200):
			return MOTOR_DSHOT1200_HZ;
	case(PWM_TYPE_DSHOT600):
			return MOTOR_DSHOT600_HZ;
	case(PWM_TYPE_DSHOT300):
			return MOTOR_DSHOT300_HZ;
	default:
	case(PWM_TYPE_DSHOT150):
			return MOTOR_DSHOT150_HZ;
	}
}

uint16_t prepareDshotPacket(motorDma_t *const motor, const uint16_t value){		 //准备Dshot的信息包，输入：电机序号，电机转速
	uint16_t packet = (value << 1) | (motor->requestTelemetry ? 1 : 0);				   //电机转速前移左移一位，再填充一位电调状态信息,这一位用于区分是否处于命令模式
	motor->requestTelemetry = false;    																				 //重置遥测请求以确保它只在一行中触发一次														
	int csum = 0;                                                          		   //计算校验和
	int csum_data = packet;
	for (int i = 0; i < 3; i++) {
		csum ^=  csum_data;   	//通过半字节异或数据
		csum_data >>= 4;
	}
	csum &= 0xf;								//与下一步操作一起完成把校验位加入后四位的操作
															//附加校验
	packet = (packet << 4) | csum;
	return packet;							//返回16位的校验位数据包
}

void pwmCompleteDshotMotorUpdate(void){
	TIM_SetCounter(TIM3, 0);					//设置TIMx中CNT寄存器的值
	TIM_DMACmd(TIM3,TIM_DMA_CC1, ENABLE);
	TIM_DMACmd(TIM3,TIM_DMA_CC2, ENABLE);
	TIM_DMACmd(TIM3,TIM_DMA_CC3, ENABLE);
	TIM_DMACmd(TIM3,TIM_DMA_CC4, ENABLE);		//四个定时器通道的DMA传输全部激活
}

static uint8_t loadDmaBufferDshot(motorDma_t *const motor, uint16_t packet){
	for (int i = 0; i < 16; i++) {											//用16次循环，把最高位依次转换为PWM意义上的1和0发送出去
		motor->dmaBuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
		packet <<= 1;
	}
	return DSHOT_DMA_BUFFER_SIZE;
}

static void pwmWriteDshotInt(uint8_t index, uint16_t value){	//PWM写入Dshot，用于写入命令
	motorDma_t *const motor = &motorDmaBuffer[index];
	BSP_DMA_TypeDef * dmaRef;
	motor->value = value;
	uint16_t packet = prepareDshotPacket(motor, value);					//根据电机转速数据得出数据包
	uint8_t bufferSize = loadDmaBufferDshot(motor, packet);			//loadDmaBuffer()等同于loadDmaBufferDshot()，得到PWM意义上的1和0
	switch(index) { 
		case RIGHT_FRONT_TOP:{
			dmaRef = &BSP_DMA_TIM3_CH1; 				
			break;
		}
		case LEFT_FRONT_TOP:{
			dmaRef = &BSP_DMA_TIM3_CH2; 				
			break;
		}
		case LEFT_REAR_TOP:{
			dmaRef = &BSP_DMA_TIM3_CH3; 				
			break;
		}
		case RIGHT_REAR_TOP:{
			dmaRef = &BSP_DMA_TIM3_CH4; 				
			break;
		}
		default : return;
	}
	DMA_SetCurrDataCounter(dmaRef->DMA_Streamx, bufferSize);	//放入DMA缓存区
	DMA_Cmd(dmaRef->DMA_Streamx, ENABLE);  
}

void pwmWriteDshot(uint8_t index, float value){							//PWM写入Dshot
	pwmWriteDshotInt(index, lrintf(value));										//第二项输入的是value的四舍五入项
}

void dshotWriteMotors(float *motor){
	for (int i = 0; i < DSHOT_MOTOR_NUM; i++) {								//在DSHOT通信下，暂时只有4个电机的控制
		pwmWriteDshot(i, motor[i]);					//第一个输入的参数是电机编号，第二个输入的参数是电机转速
  }
	pwmCompleteDshotMotorUpdate();
}

void pwmDshotDmaIrqnConfig(uint8_t irqN,uint8_t PreemptionPriority,uint8_t SubPriority){
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = irqN;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void dshotMotorConfig(motorPwmProtocolTypes_e typeMode){
	uint16_t const typePrescaler = (MHZ_TO_HZ(84) / getDshotHz(typeMode)) - 1;
	BSP_TIM_PWM_Init(TIM3,MOTOR_BITLENGTH,typePrescaler,BSP_GPIOB4,BSP_GPIOB5,BSP_GPIOB0,BSP_GPIOB1);		
	pwmDshotDmaIrqnConfig(DMA1_Stream4_IRQn,1,0);	
	BSP_DMA_DSHOT_Init(&BSP_DMA_TIM3_CH1,(uint32_t)&TIM3->CCR1,(uint32_t)motorDmaBuffer[RIGHT_FRONT_TOP].dmaBuffer,DSHOT_DMA_BUFFER_SIZE);
	pwmDshotDmaIrqnConfig(DMA1_Stream5_IRQn,1,0);	
	BSP_DMA_DSHOT_Init(&BSP_DMA_TIM3_CH2,(uint32_t)&TIM3->CCR2,(uint32_t)motorDmaBuffer[LEFT_FRONT_TOP].dmaBuffer,DSHOT_DMA_BUFFER_SIZE);
	pwmDshotDmaIrqnConfig(DMA1_Stream7_IRQn,1,0);
	BSP_DMA_DSHOT_Init(&BSP_DMA_TIM3_CH3,(uint32_t)&TIM3->CCR3,(uint32_t)motorDmaBuffer[LEFT_REAR_TOP].dmaBuffer,DSHOT_DMA_BUFFER_SIZE);
	pwmDshotDmaIrqnConfig(DMA1_Stream2_IRQn,1,0);
	BSP_DMA_DSHOT_Init(&BSP_DMA_TIM3_CH4,(uint32_t)&TIM3->CCR4,(uint32_t)motorDmaBuffer[RIGHT_REAR_TOP].dmaBuffer,DSHOT_DMA_BUFFER_SIZE);
}

//需要时解除注释
//void DMA1_Stream4_IRQHandler(void)                                  //DMA1-4
//{
//	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4))
//	{
//		DMA_Cmd(DMA1_Stream4, DISABLE);						//失能
//		TIM_DMACmd(TIM3,TIM_DMA_CC1, DISABLE);		//取消dma更新tim比较
//		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);							//清除IT_FLAG
//	}
//}

//void DMA1_Stream5_IRQHandler(void)                                  //DMA1-5
//{
//	if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5))
//	{
//		DMA_Cmd(DMA1_Stream5, DISABLE);						//失能
//		TIM_DMACmd(TIM3,TIM_DMA_CC2, DISABLE);		//取消dma更新tim比较
//		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);							//清除IT_FLAG
//	}
//}

//void DMA1_Stream7_IRQHandler(void)                                  //DMA1-7
//{
//	if(DMA_GetITStatus(DMA1_Stream7,DMA_IT_TCIF7))
//	{
//		DMA_Cmd(DMA1_Stream7, DISABLE);						//失能
//		TIM_DMACmd(TIM3,TIM_DMA_CC3, DISABLE);		//取消dma更新tim比较
//		DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);							//清除IT_FLAG
//	}
//}

//void DMA1_Stream2_IRQHandler(void)                                  //DMA1-2
//{                                                                   
//	if(DMA_GetITStatus(DMA1_Stream2,DMA_IT_TCIF2))
//	{
//		DMA_Cmd(DMA1_Stream2, DISABLE);						//失能
//		TIM_DMACmd(TIM3,TIM_DMA_CC4, DISABLE);		//取消dma更新tim比较
//		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);							//清除IT_FLAG
//	}
//}


