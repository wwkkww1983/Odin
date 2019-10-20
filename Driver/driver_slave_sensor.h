#ifndef __DRIVER_SLAVE_SENSOR_H
#define __DRIVER_SLAVE_SENSOR_H

#include "bsp.h"
#include "imu.h"

/**********************************SLAVE_SENSORʼ������****************************************/
#define SLAVE_SENSOR_USARTX							USART3			//SLAVE_SENSOR���ں�
#define SLAVE_SENSOR_USARTX_RX_PIN			BSP_GPIOD9		//SLAVE_SENSOR��������
#define SLAVE_SENSOR_USARTX_TX_PIN			BSP_GPIOD8		//SLAVE_SENSOR��������
#define SLAVE_SENSOR_USART_PRE				3				//SLAVE_SENSOR_USART�ж���ռ���ȼ�
#define SLAVE_SENSOR_USART_SUB				0				//SLAVE_SENSOR_USART�ж���Ӧ���ȼ�
#define SLAVE_SENSOR_USART_BOUND			961200

#define MAIN_OR_SLAVE_CONTROL_USARTX			USART6		//�������ؼ��ͨ�Ŵ��ں�
#define MAIN_OR_SLAVE_CONTROL_USARTX_RX_PIN		BSP_GPIOC7	//�������ؼ��ͨ�Ŵ��ڽ�������
#define MAIN_OR_SLAVE_CONTROL_USARTX_TX_PIN		BSP_GPIOC6	//�������ؼ��ͨ�Ŵ��ڷ�������
#define MAIN_OR_SLAVE_CONTROL_USARTX_PRE		3			//�������ؼ��ͨ�Ŵ����ж���ռ���ȼ�
#define MAIN_OR_SLAVE_CONTROL_USARTX_SUB		0			//�������ؼ��ͨ�Ŵ����ж���Ӧ���ȼ�
#define MIAN_OR_SLAVE_CONTROL_USARTS_BOUND		230400

void driver_slaveSensorInit(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,\
														BSP_GPIOSource_TypeDef *USART_TX,u32 baudRate,\
														u8 PreemptionPriority,u8 SubPriority);

#endif
