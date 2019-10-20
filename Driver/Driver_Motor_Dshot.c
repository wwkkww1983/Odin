
#include "Driver_Motor_Dshot.h"
#include "math.h"
#include "Driver_Motor.h"
#include "BSP.h"
#include "parameter.h"

motorDma_t motorDmaBuffer[MOTORS_NUM];

uint32_t getDshotHz(motorPwmProtocolTypes_e pwmProtocolType){									//�õ���ǰDshot��Ƶ��
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

uint16_t prepareDshotPacket(motorDma_t *const motor, const uint16_t value){		 //׼��Dshot����Ϣ�������룺�����ţ����ת��
	uint16_t packet = (value << 1) | (motor->requestTelemetry ? 1 : 0);				   //���ת��ǰ������һλ�������һλ���״̬��Ϣ,��һλ���������Ƿ�������ģʽ
	motor->requestTelemetry = false;    																				 //����ң��������ȷ����ֻ��һ���д���һ��														
	int csum = 0;                                                          		   //����У���
	int csum_data = packet;
	for (int i = 0; i < 3; i++) {
		csum ^=  csum_data;   	//ͨ�����ֽ��������
		csum_data >>= 4;
	}
	csum &= 0xf;								//����һ������һ����ɰ�У��λ�������λ�Ĳ���
															//����У��
	packet = (packet << 4) | csum;
	return packet;							//����16λ��У��λ���ݰ�
}

void pwmCompleteDshotMotorUpdate(void){
	TIM_SetCounter(TIM3, 0);					//����TIMx��CNT�Ĵ�����ֵ
	TIM_DMACmd(TIM3,TIM_DMA_CC1, ENABLE);
	TIM_DMACmd(TIM3,TIM_DMA_CC2, ENABLE);
	TIM_DMACmd(TIM3,TIM_DMA_CC3, ENABLE);
	TIM_DMACmd(TIM3,TIM_DMA_CC4, ENABLE);		//�ĸ���ʱ��ͨ����DMA����ȫ������
}

static uint8_t loadDmaBufferDshot(motorDma_t *const motor, uint16_t packet){
	for (int i = 0; i < 16; i++) {											//��16��ѭ���������λ����ת��ΪPWM�����ϵ�1��0���ͳ�ȥ
		motor->dmaBuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;  // MSB first
		packet <<= 1;
	}
	return DSHOT_DMA_BUFFER_SIZE;
}

static void pwmWriteDshotInt(uint8_t index, uint16_t value){	//PWMд��Dshot������д������
	motorDma_t *const motor = &motorDmaBuffer[index];
	BSP_DMA_TypeDef * dmaRef;
	motor->value = value;
	uint16_t packet = prepareDshotPacket(motor, value);					//���ݵ��ת�����ݵó����ݰ�
	uint8_t bufferSize = loadDmaBufferDshot(motor, packet);			//loadDmaBuffer()��ͬ��loadDmaBufferDshot()���õ�PWM�����ϵ�1��0
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
	DMA_SetCurrDataCounter(dmaRef->DMA_Streamx, bufferSize);	//����DMA������
	DMA_Cmd(dmaRef->DMA_Streamx, ENABLE);  
}

void pwmWriteDshot(uint8_t index, float value){							//PWMд��Dshot
	pwmWriteDshotInt(index, lrintf(value));										//�ڶ����������value������������
}

void dshotWriteMotors(float *motor){
	for (int i = 0; i < DSHOT_MOTOR_NUM; i++) {								//��DSHOTͨ���£���ʱֻ��4������Ŀ���
		pwmWriteDshot(i, motor[i]);					//��һ������Ĳ����ǵ����ţ��ڶ�������Ĳ����ǵ��ת��
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

//��Ҫʱ���ע��
//void DMA1_Stream4_IRQHandler(void)                                  //DMA1-4
//{
//	if(DMA_GetITStatus(DMA1_Stream4,DMA_IT_TCIF4))
//	{
//		DMA_Cmd(DMA1_Stream4, DISABLE);						//ʧ��
//		TIM_DMACmd(TIM3,TIM_DMA_CC1, DISABLE);		//ȡ��dma����tim�Ƚ�
//		DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);							//���IT_FLAG
//	}
//}

//void DMA1_Stream5_IRQHandler(void)                                  //DMA1-5
//{
//	if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5))
//	{
//		DMA_Cmd(DMA1_Stream5, DISABLE);						//ʧ��
//		TIM_DMACmd(TIM3,TIM_DMA_CC2, DISABLE);		//ȡ��dma����tim�Ƚ�
//		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);							//���IT_FLAG
//	}
//}

//void DMA1_Stream7_IRQHandler(void)                                  //DMA1-7
//{
//	if(DMA_GetITStatus(DMA1_Stream7,DMA_IT_TCIF7))
//	{
//		DMA_Cmd(DMA1_Stream7, DISABLE);						//ʧ��
//		TIM_DMACmd(TIM3,TIM_DMA_CC3, DISABLE);		//ȡ��dma����tim�Ƚ�
//		DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);							//���IT_FLAG
//	}
//}

//void DMA1_Stream2_IRQHandler(void)                                  //DMA1-2
//{                                                                   
//	if(DMA_GetITStatus(DMA1_Stream2,DMA_IT_TCIF2))
//	{
//		DMA_Cmd(DMA1_Stream2, DISABLE);						//ʧ��
//		TIM_DMACmd(TIM3,TIM_DMA_CC4, DISABLE);		//ȡ��dma����tim�Ƚ�
//		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);							//���IT_FLAG
//	}
//}


