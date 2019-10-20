#ifndef __DRIVE_DMMOTOR_H
#define __DRIVE_DMMOTOR_H


#include "bsp.h"
#include "BSP_GPIO.h"
#include "Driver_RMMotor.h"
#include "Driver_DMMotor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal.h"


/**********************************RS485初始化串口****************************************/
#define RS485_USARTX									UART8			  //RS485串口号
#define RS485_USARTX_RX_PIN						BSP_GPIOE0	//RS485接收引脚
#define RS485_USARTX_TX_PIN						BSP_GPIOE1	//RS485发送引脚
#define RS485_USART_PRE 1						//RS485_USART中断抢占优先级
#define RS485_USART_SUB 0						//RS485_USART中断响应优先级
#define RS485_EN												PDout(3)		// 0：接收    1：发送
#define RS485_LEN  											64 
#define RECEIVE_ENABLE                  0
#define SEND_ENABLE 										1

#define  YAW                            0X01
#define  PITCH       										0X02

typedef  enum{
	RS485ID_1 = 0x01,
  RS485ID_2,
  RS485ID_3,
  RS485ID_4,	
}	rs485Id_e;

typedef struct{
	rs485Id_e	 rs485Id;
	u8   				 rs485PowerLow;
	u8					 rs485PowerHigh;		
	vs16				 yawReceiveData[RS485_LEN];
}	rs485Struct_t;


extern rs485Struct_t rs485Data;

void rs485DataUpdata(uint8_t typeOfMotor);
void rs485ReceiveData(u8 *arrayYawReceive,u8 typeOfMotor);
void rs485SendGimbalUpdata(uint8_t typeOfMotor);
void rs485DeviceInit(void);
void Driver_RS485_Init(USART_TypeDef* USARTx,BSP_GPIOSource_TypeDef *USART_RX,BSP_GPIOSource_TypeDef *USART_TX,u8 PreemptionPriority,u8 SubPriority);

#endif







