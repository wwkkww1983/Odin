#ifndef __DRIVER_FLOW_H
#define __DRIVER_FLOW_H

#include "bsp.h"

//匿名光流数据包
typedef __packed struct
{
	signed char function; 
	unsigned char length;
	unsigned char mode;   // 是否滤波 0为原始数据  1为滤波后的数据
	
	unsigned char quality;// 光流数据可信度
	int dx;
	int dy;
	int dx2;
	int dy2;
	int dx_fix;
	int dy_fix;	
	unsigned char light;  // 光纤强度
	
	unsigned int alt;
	float realAlt;
	
	int gyr_x;
	int gyr_y;
	int gyr_z;
	
	int acc_x;
	int acc_y;
	int acc_z;	
	
	int rol;
	int pit;
	int yaw;	
	
	int s1;
	int s2;
	int s3;		
	int s4;			
	
	signed char sum;	 //和校验
	 
	uint8_t work;
}ano_tc_data;

#define FLOW_USARTX						USART3
#define FLOW_USARTX_RX_PIN		BSP_GPIOD9
#define FLOW_USARTX_TX_PIN 		BSP_GPIOD8
#define FLOW_USART_PreemptionPriority 1
#define FLOW_USART_SubPriority 0

extern ano_tc_data ano_tc_of;

void ANO_DT_Data_Receive_OF(u8 *data_buf,u8 num);
void flowInit(void);

#endif

