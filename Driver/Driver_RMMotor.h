#ifndef __DRIVER_RMMOTOR_H
#define __DRIVER_RMMOTOR_H

#include "bsp.h"
#include "BSP_GPIO.h"
#define ENCODER_ANGLE_RATIO    (360.0f/8192.0f)
#define FILTER_BUF 3
/************************* ID���� *********************/
#define ID_MASTER_BOARD						0x101
#define ID_SLAVE_BOARD						0x102

#define ID_RMMOTOR_RIGHT_FRONT 		0x201
#define ID_RMMOTOR_LEFT_FRONT 		0x202
#define ID_RMMOTOR_LEFT_BACK 			0x203
#define ID_RMMOTOR_RIGHT_BACK 		0x204
#define ID_RM6623_YAW		          0x205
#define ID_RM6623_PITCH	          0x206
#define ID_POKEMOTOR		          0x207
#define ID_PAW_L									0x207
#define ID_PAW_R									0x208
#define ID_SUPPLY				          0x208
#define ID_FIRE_TNF_R_OR_YAW      0x205
#define ID_FIRE_TNF_L_OR_PITCH    0x206
#define ID_YAW_INF					 			0x209
#define ID_PITCH_INF         			0x20A
#define ID_TURNTABLE              0x203     //ɸ�����
#define ID_BIGPOKE                0x204     //��������������
#define ID_CURRENT                0x301

enum{
	VERSION_RM2016 = 0,
	VERSION_RM2017
};

enum{                         //����б�
	MOTOR_NO = 0,								//û��ID���Ṥ��  
	MOTOR_6623,		   						//6623���
	MOTOR_6020,									//GM6020���
	MOTOR_3510,								  //GM3510���
	MOTOR_9015,                 //DM9015���
};

enum{                         //���ŷ�����Ϣ�б� 
	CODEBOARD_VALUE = 0,				//����ֵ  
	REALTORQUE_CURRENT,		   		//ʵ��ת�ص���
	TORQUE_CURRENT,							//����ת�ص���
	TURN_SPEED,								  //ת��
	TEMPER,                     //�¶�
};


typedef struct{
	vs16 currunt1;		//��ǰ��
	vs16 currunt2;   	//��ǰ��
	vs16 currunt3;    //���
	vs16 currunt4;   	//�Һ�
}motorSerialNumber_t;

typedef struct{
	vs16 	encoderAngle;	//����Ƕȱ�����ֵ  
	vs16	realcurrent;	//ʵ��ת�ص�������ֵ				
}dm9015DataRecv_t;

typedef struct{
	vs16 	rawangle;	//����Ƕȱ�����ֵ
	vs16 	speed;		//���ת��
	vs16  currunt;  //ת�ص���
	uint8_t  temperature;	//����¶�
}motorCanDataRecv_t;

typedef struct{
	vs16 fdbPosition;        //����ı���������ֵ
	vs16 last_fdbPosition;   //����ϴεı���������ֵ
	vs16 bias_position;      //�����˳�ʼ״̬���λ�û��趨ֵ
	vs16 fdbSpeed;           //���������ת��/rpm
	vs16 round;              //���ת����Ȧ��
	vs32 real_position;      //���㴦���ĵ��ת��λ��
}rmmotorTarData_t;

typedef struct{
	vs16 	encoderAngle;	//����Ƕȱ�����ֵ  
	vs16	realcurrent;	//ʵ��ת�ص�������ֵ
	vs16	current;			//ת�ص�������ֵ
	float realAngle;		//ת���ɽǶȵ�λ						
}gimbalCanDataRecv_t;

typedef struct{                         //ͨ�õ���б�              
	gimbalCanDataRecv_t 	M6623Data;	    //6623���������Ϣ          
	gimbalCanDataRecv_t 	GM3510Data;    //GM3510�������                      
	motorCanDataRecv_t    GM6020Data;    //GM6020�������           
  dm9015DataRecv_t      DM9015Data;	   //DM9015�������
	uint8_t  motorID;                     //�������ID               
}rmmotorCanDataRecv_t;

typedef struct{
  uint16_t ecd;
  float  rotationalSpeed;
  int16_t  current;
  int16_t  temp;
	
	uint16_t lastEcd;
  int32_t  roundCnt;
  int32_t  totalEcd;
  int32_t  totalAngle;
  
  uint16_t offsetEcd;
  uint32_t msgCnt;
  
  int32_t  ecdRawRate;
  int32_t  rateBuf[FILTER_BUF];
  uint8_t  bufCut;
  int32_t  filterRate;
} motorMeasureData_t;

typedef struct{
	vs16 currentAll;		//idΪ�ܵ���ĵ���ֵ	
	vs16 current201;		//idΪ0x201����ĵ���ֵ				
	vs16 current202;		//idΪ0x202����ĵ���ֵ				
	vs16 current203;		//idΪ0x203����ĵ���ֵ				
	vs16 current204;		//idΪ0x204����ĵ���ֵ
	vs16 powerRank ;    //�����ʵȼ�
}currentCanDataRecv_t;

/*******************************************************************/
void driver_can1_init(CAN_TypeDef* rm_canx,BSP_GPIOSource_TypeDef *rm_canx_rx,BSP_GPIOSource_TypeDef *rm_canx_tx,u8 Preemption,u8 Sub);
void driver_can2_init(CAN_TypeDef* rm_canx,BSP_GPIOSource_TypeDef *rm_canx_rx,BSP_GPIOSource_TypeDef *rm_canx_tx,u8 Preemption,u8 Sub);
void gimbal_readdata(CanRxMsg *can_rx_msg,gimbalCanDataRecv_t *gimbal_data);
void rmmotor_readdata(CanRxMsg *can_rx_msg,motorCanDataRecv_t *chassisData);
void rmmotor_senddata(CAN_TypeDef *CANx, u32 ID_CAN, motorSerialNumber_t *rmmotor_cansend);
int16_t getRelativePos(int16_t rawEcd, int16_t centerOffset);
void driver_rm6623_calibration(CAN_TypeDef *canx, unsigned char version);
void mecanumCalculate(float V_X,float V_Y,float V_Rotate,float MaxWheelSpeed,float* Wheel_Speed);
void gimbal_motor6020_readdata(CanRxMsg *can_rx_msg,motorCanDataRecv_t *motor6020Data);

void gimbal_readData(CanRxMsg *can_rx_msg,rmmotorCanDataRecv_t *gimbal_data); 
vs16 gimbal_chooseData(uint8_t dataNumber,rmmotorCanDataRecv_t *gimbal_data);
void gimbal_readSlaveData(uint8_t dataNumber,vs16 writeData,rmmotorCanDataRecv_t *gimbal_data);

extern motorCanDataRecv_t	 		fricWheelData[2];
extern motorCanDataRecv_t	 		pokeData;
extern motorCanDataRecv_t	 		lidData;
extern motorCanDataRecv_t  		turntableData;
extern motorCanDataRecv_t	 		bigPokeData;      
extern motorCanDataRecv_t	 		wheelData[4];
extern rmmotorCanDataRecv_t 	pitchMotorData,yawMotorData; 
extern motorMeasureData_t   	motorViewPitData,motorViewYawData;
extern currentCanDataRecv_t 	currentDate;
extern motorMeasureData_t   	motorGM3510PitData;
extern motorCanDataRecv_t	 		pawMotorData[2];												//����צ��
extern motorCanDataRecv_t	 		deformMotorData[2];										//���̺�����λ���
extern motorCanDataRecv_t	 		liftMotorData[2];											//����̧������
extern rmmotorTarData_t 	 		pawMotorTarData[2];		
#endif


