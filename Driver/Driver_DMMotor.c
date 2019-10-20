#include "Driver_DMMotor.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )

rs485Struct_t rs485Data;
BSP_USART_TypeDef MAX3485;

void rs485DataUpdata(uint8_t typeOfMotor){
	short sendData;
	if(typeOfMotor == YAW){
		sendData = (short)gimbalData.yawSpeedOut;
		rs485Data.rs485Id = RS485ID_1;
	}
	else if(typeOfMotor == PITCH){
		sendData = (short)gimbalData.pitchSpeedOut;
		rs485Data.rs485Id = RS485ID_2;
	}
	rs485Data.rs485PowerLow =  BYTE0(sendData);
	rs485Data.rs485PowerHigh = BYTE1(sendData);
}

void rs485ReceiveData(u8 *arrayYawReceive,u8 typeOfMotor){
	static u8 count = 0;
	u8  sum = 0;
	for(count =0; count <16; count ++)
		if(arrayYawReceive[count] != 0x3E){
		}
		else{
			rs485Data.yawReceiveData[0] = arrayYawReceive[count];
			rs485Data.yawReceiveData[1] = arrayYawReceive[count+1];
			rs485Data.yawReceiveData[2] = arrayYawReceive[count+2];
			rs485Data.yawReceiveData[3] = arrayYawReceive[count+3];
			rs485Data.yawReceiveData[4] = arrayYawReceive[count+4];
			rs485Data.yawReceiveData[5] = arrayYawReceive[count+5];
			rs485Data.yawReceiveData[6] = arrayYawReceive[count+6];
			rs485Data.yawReceiveData[7] = arrayYawReceive[count+7];
			count = 0;
			break;
		}
	if(count >= 16){
	}
	else{
		for(u8 cnt=0;cnt<4;cnt++)
			sum += rs485Data.yawReceiveData[cnt];
		if(sum!=rs485Data.yawReceiveData[4]){					//У��ʧ�� ����������
		}
		else{
			sum = 0;
			for(u8 cnt=5;cnt<7;cnt++)
				sum += rs485Data.yawReceiveData[cnt];
			if(sum!=rs485Data.yawReceiveData[7]){
				//У��ʧ�� ����������
			}
			else{
				if((rs485Data.yawReceiveData[5] | (rs485Data.yawReceiveData[6]<<8)) > 4096){
					//���ݳ��� ����������		
				}
				else{
					switch(rs485Data.yawReceiveData[2]){
						case YAW :
							gimbal_readSlaveData(CODEBOARD_VALUE,(4096-((rs485Data.yawReceiveData[6]<<8) | rs485Data.yawReceiveData[5]))*2,&yawMotorData); 
							digitalIncreasing(&gimbalData.gimbalError[0].errorCount);
						break;
						case PITCH :
							gimbal_readSlaveData(CODEBOARD_VALUE,(4096-((rs485Data.yawReceiveData[6]<<8) | rs485Data.yawReceiveData[5]))*2,&pitchMotorData);
							digitalIncreasing(&gimbalData.gimbalError[1].errorCount);
						break;
					}
				}
		  }
	  }
  }
}

void rs485SendGimbalUpdata(uint8_t typeOfMotor){
	s8 sum = 0;
	rs485DataUpdata(typeOfMotor);
	RS485_EN = SEND_ENABLE;
	Array_UART8_TX[0] = 0x3E;
	Array_UART8_TX[1] = 0xA0;
	Array_UART8_TX[2] = rs485Data.rs485Id;
	Array_UART8_TX[3] = 0x02;
	Array_UART8_TX[4] = 0x00;
	Array_UART8_TX[5] = rs485Data.rs485PowerLow;
	Array_UART8_TX[6] = rs485Data.rs485PowerHigh;			
	Array_UART8_TX[7] = 0X00;
	
	for(u8 cnt=0;cnt<4;cnt++)
		sum += Array_UART8_TX[cnt];
	Array_UART8_TX[4]=sum;
	sum = 0;
	for(u8 cnt=5;cnt<7;cnt++)
		sum += Array_UART8_TX[cnt];
	Array_UART8_TX[7]=sum;
	MAX3485.USARTx = UART8;
	BSP_USART_SendData(&MAX3485,Array_UART8_TX,8);				
//	BSP_UART8_DMA_SendData(Array_UART8_TX,8);														//DMA������
//	while((UART8->SR & USART_FLAG_TC)==0);															//�ȴ����ͽ���
	USART_ClearFlag(UART8,USART_FLAG_TC);																//���������ɱ�־
	RS485_EN = RECEIVE_ENABLE;																					//�л�Ϊ����ģʽ
}

void rs485DeviceInit(void){
	Driver_RS485_Init(RS485_USARTX,RS485_USARTX_RX_PIN,RS485_USARTX_TX_PIN,RS485_USART_PRE,RS485_USART_SUB);
}

/*
***************************************************
��������Driver_RS485_Init
���ܣ���ع����DM���RS485ͨ�ų�ʼ��
��ڲ�����	I can understand it at one glance.
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void Driver_RS485_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef RS485;
	RS485.USARTx = USARTx;
	RS485.USART_RX = USART_RX;
	RS485.USART_TX = USART_TX;
	RS485.USART_InitStructure.USART_BaudRate = 115200;									/*������Ϊ1M*/
	RS485.USART_InitStructure.USART_WordLength = USART_WordLength_8b;	/*�ֳ�Ϊ8λ���ݸ�ʽ*/
	RS485.USART_InitStructure.USART_StopBits = USART_StopBits_1;				/*һ��ֹͣλ*/
	RS485.USART_InitStructure.USART_Parity = USART_Parity_No;				/*��У��λ*/
	RS485.USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;							/*����/����ģʽ*/	
	RS485.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*��Ӳ������������*/	
	
	BSP_USART_Init(&RS485,PreemptionPriority,SubPriority);
	USART_ITConfig(UART8, USART_IT_IDLE, DISABLE); 	//�رմ��ڿ����ж�
	USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);		//����������ͨ�����ж�
//	BSP_USART_RX_DMA_Init(&RS485);						 		//���ô���DMA�����жϽ��� �����ϵͳDMAͨ����ͻ
//	BSP_USART_TX_DMA_Init(&RS485);	
	BSP_GPIO_Init(BSP_GPIOD3,GPIO_Mode_Out_PP);
	RS485_EN = SEND_ENABLE;						//Ĭ����Ϊ����ģʽ
}
