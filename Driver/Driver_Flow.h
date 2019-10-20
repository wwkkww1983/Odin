#ifndef __DRIVER_FLOW_H
#define __DRIVER_FLOW_H

#include "bsp.h"

//�����������ݰ�
typedef __packed struct
{
	signed char function; 
	unsigned char length;
	unsigned char mode;   // �Ƿ��˲� 0Ϊԭʼ����  1Ϊ�˲��������
	
	unsigned char quality;// �������ݿ��Ŷ�
	int dx;
	int dy;
	int dx2;
	int dy2;
	int dx_fix;
	int dy_fix;	
	unsigned char light;  // ����ǿ��
	
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
	
	signed char sum;	 //��У��
	 
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

