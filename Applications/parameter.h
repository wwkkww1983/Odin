#ifndef __PARAMETER_H
#define __PARAMETER_H

#include "bsp.h"

enum parameterslist{
	CONFIG_VERSION = 0,
	ROBOT_TYPE,
	DEAD_BAND,
	SHOOT_LOW_PWM,
	SHOOT_HIGH_PWM,
	MOUSE_FEEL,
	GIMBAL_CTR_SCALE,
	GIMBAL_KB_SCALE,
	CHASSIS_KB_SPEED,
	CHASSIS_RC_SPEED,
	CHASSIS_KB_ACC,
	
	TILT_ANG_P,											//��̨pitch��roll�ǶȻ�����
	TILT_ANG_I,
	TILT_ANG_D,
	TILT_ANG_F,
	TILT_ANG_PM,
	TILT_ANG_IM,
	TILT_ANG_DM,
	TILT_ANG_OM,
	
	TILT_RATE_P,   									//��̨pitch��roll���ٶȻ�����
	TILT_RATE_I,
	TILT_RATE_D,
	TILT_RATE_F,
	TILT_RATE_PM,
	TILT_RATE_IM,
	TILT_RATE_DM,
	TILT_RATE_OM,
	
	YAW_ANG_P,											//��̨yaw�ǶȻ�����
	YAW_ANG_I,
	YAW_ANG_D,
	YAW_ANG_F,
	YAW_ANG_PM,
	YAW_ANG_IM,
	YAW_ANG_DM,
	YAW_ANG_OM,
	
	YAW_RATE_P,    									//��̨yaw���ٶȻ�����
	YAW_RATE_I,
	YAW_RATE_D,
	YAW_RATE_F,
	YAW_RATE_PM,
	YAW_RATE_IM,
	YAW_RATE_DM,
	YAW_RATE_OM,
	
	CHASSIS_SPEED_P,								//�����ٶȻ�����
	CHASSIS_SPEED_I,
	CHASSIS_SPEED_D,
	CHASSIS_SPEED_F,
	CHASSIS_SPEED_PM,
	CHASSIS_SPEED_IM,
	CHASSIS_SPEED_DM,
	CHASSIS_SPEED_OM,
	
	CHASSIS_POS_P,									//����λ�û�����
	CHASSIS_POS_I,
	CHASSIS_POS_D,
	CHASSIS_POS_F,
	CHASSIS_POS_PM,
	CHASSIS_POS_IM,
	CHASSIS_POS_DM,
	CHASSIS_POS_OM,
	
	CHASSIS_CHASE_P,								//���̸�����ٶȻ�����
	CHASSIS_CHASE_I,
	CHASSIS_CHASE_D,
	CHASSIS_CHASE_F,
	CHASSIS_CHASE_PM,
	CHASSIS_CHASE_IM,
	CHASSIS_CHASE_DM,
	CHASSIS_CHASE_OM,
	
	CHASSIS_RATE_P,									//���̸�����ٶȻ�����
	CHASSIS_RATE_I,
	CHASSIS_RATE_D,
	CHASSIS_RATE_F,
	CHASSIS_RATE_PM,
	CHASSIS_RATE_IM,
	CHASSIS_RATE_DM,
	CHASSIS_RATE_OM,
	
	POWER_LIMIT_P,									//���̹��ʻ�����
	POWER_LIMIT_I,
	POWER_LIMIT_D,
	POWER_LIMIT_F,
	POWER_LIMIT_PM,
	POWER_LIMIT_IM,
	POWER_LIMIT_DM,
	POWER_LIMIT_OM,
	
	PROPORTIONAL_SPEED_P,						//���������ٶȻ�
	PROPORTIONAL_SPEED_I,
	PROPORTIONAL_SPEED_D,
	PROPORTIONAL_SPEED_F,
	PROPORTIONAL_SPEED_PM,
	PROPORTIONAL_SPEED_IM,
	PROPORTIONAL_SPEED_DM,
	PROPORTIONAL_SPEED_OM,
	
	PROPORTIONAL_POS_P,							//��������λ�û�
	PROPORTIONAL_POS_I,
	PROPORTIONAL_POS_D,
	PROPORTIONAL_POS_F,
	PROPORTIONAL_POS_PM,
	PROPORTIONAL_POS_IM,
	PROPORTIONAL_POS_DM,
	PROPORTIONAL_POS_OM,
	
	HOLD_PILLAR_SPEED_P,						//ץ�������ٶȻ�
	HOLD_PILLAR_SPEED_I,
	HOLD_PILLAR_SPEED_D,
	HOLD_PILLAR_SPEED_F,
	HOLD_PILLAR_SPEED_PM,
	HOLD_PILLAR_SPEED_IM,
	HOLD_PILLAR_SPEED_DM,
	HOLD_PILLAR_SPEED_OM,
	
	DEFORM1_SPEED_P,								//���λ���1�ٶȻ�
	DEFORM1_SPEED_I,
	DEFORM1_SPEED_D,
	DEFORM1_SPEED_F,
	DEFORM1_SPEED_PM,
	DEFORM1_SPEED_IM,
	DEFORM1_SPEED_DM,
	DEFORM1_SPEED_OM,
	
	DEFORM2_SPEED_P,								//���λ���2�ٶȻ�
	DEFORM2_SPEED_I,
	DEFORM2_SPEED_D,
	DEFORM2_SPEED_F,
	DEFORM2_SPEED_PM,
	DEFORM2_SPEED_IM,
	DEFORM2_SPEED_DM,
	DEFORM2_SPEED_OM,

	SHOOT_SPEED_P,									//��������ٶȻ�
	SHOOT_SPEED_I,
	SHOOT_SPEED_D,
	SHOOT_SPEED_F,
	SHOOT_SPEED_PM,
	SHOOT_SPEED_IM,
	SHOOT_SPEED_DM,
	SHOOT_SPEED_OM,
	
	LOADED_SPEED_P,									//���������ٶȻ�
	LOADED_SPEED_I,
	LOADED_SPEED_D,
	LOADED_SPEED_F,
	LOADED_SPEED_PM,
	LOADED_SPEED_IM,
	LOADED_SPEED_DM,
	LOADED_SPEED_OM,
	
	ADRC_R,													//�����ٶ�ADRC
	ADRC_H,
	ADRC_N0,
	ADRC_BETA01,
	ADRC_BETA02,
	ADRC_BETA03,
	ADRC_B0,
	ADRC_BETA0,
	ADRC_BETA1,
	ADRC_BETA2,
	ADRC_N1,
	ADRC_C,
	ADRC_ALPHA1,
	ADRC_ALPHA2,
	ADRC_ZETA,
	ADRC_B,
	ADRC_OMAX,
/*----------------	���²����洢��TF����һ���ļ�	��motor�ļ���----------------*/
	LOCAL_ID,                           //�öβ������µ�������ļ��б������ͬ��ֻ�����ֲ�ͬ
	WEAPON_TYPE,
	PITCH_INSTALL,
	YAW_INSTALL,
	BACK_CENTER_TIME,
	CHASSIS_CURRENT,
	RC_RESOLUTION,
	YAW_CENTER,	
	PITCH_CENTER,
	PITCH_MIN_RANGE,
	PITCH_MAX_RANGE,
	YAW_TYPE,       
	PITCH_TYPE,
	YAW_FIX,
	YAW_TURN,
	PITCH_FIX,
	PITCH_TURN,    
	IMU_ACC_BIAS_X,
	IMU_ACC_BIAS_Y,
	IMU_ACC_BIAS_Z,
	IMU_MAG_BIAS_X,
	IMU_MAG_BIAS_Y,
	IMU_MAG_BIAS_Z,
	IMU_GYO_BIAS_X,
	IMU_GYO_BIAS_Y,
	IMU_GYO_BIAS_Z,
	NUM_OF_LIST
};

enum motormessagelist{                                //��������ļ� 
  ROBOT_ID,                //����������               
	NATIVE_ID,              //������ID  ͬLOCAL_ID ��ͬ
	ARMS_TYPE,              //��������
	PITCH_ORIENT,           //pitch���ܷ���
	YAW_ORIENT,             //yaw���ܷ���
	RETURN_TIME,            //����ʱ��
	CHASSIS_POWER,          //���������
	RC_RATIO,               //ң�������ֵ
	YAW_MIDDLE,             //yaw���м�λ�� 
	PITCH_MIDDLE,           //pitch���м�λ��
	PITCH_MIN_SEAT,         //piych�����Ƕ� 
	PITCH_MAX_SEAT,         //pitch����С�Ƕ�
	YAW_ID,                 //yaw��������
  PITCH_ID,               //pitch��������          
  YAW_FASTEN,             //yaw������װ����        
	YAW_SPIN,               //yaw����ת����
	PITCH_FASTEN,            //pitch������װ����      
	PITCH_SPIN,              //pitch����ת����  
  IMU_ACC_INIT_X,
	IMU_ACC_INIT_Y,
	IMU_ACC_INIT_Z,
	IMU_MAG_INIT_X,
	IMU_MAG_INIT_Y,
	IMU_MAG_INIT_Z,
	IMU_GYO_INIT_X,
	IMU_GYO_INIT_Y,
	IMU_GYO_INIT_Z,
	ABOVE_TOTAL              //���ϲ���������  
};

typedef struct{
	uint8_t TFInsertState;						//��һ��TF�����״̬
	uint8_t TFInsertLastState;				//��һ��TF�������״̬
	uint8_t TFInsertFlag;							//TF��������
	uint8_t TFError;									//TF�����־
}parameterStruct_t;

#define TFCARD_NUM_LIST NUM_OF_LIST-26
#define TFCARD_INSERT 0
#define TFCARD_OUT 1
#define TFCARD_INSERT_IO	PDin(10)

#define USE_DIGITAL_IMU
#define RC_SBUS
#define DSHOT_USE
#define MAG9250_ENABLE

#define DIMU_FLAG 	(1 << 0)
#define IMU_FLAG 		(1 << 1)
#define SENSOR_FLAG (1 << 2)
#define DBUS_FLAG 	(1 << 3)
#define SUPERVISOR_FLAG (1 << 4)

#define waitForFlag(xFLAG)	xTaskNotifyWait(0x00000000,0xFFFFFFFF,&xFLAG,portMAX_DELAY);					//��õȴ�500ms

extern parameterStruct_t parameterRunData;

void tFCardUpdate(void);
uint8_t parameterWriteDataFormFlash(uint8_t robotId);
uint8_t parameterReadDataFromTFCard(uint8_t robotId);
uint8_t motorMessageReadDataFromTFCard(uint8_t robotId);
uint8_t motorMessageWriteDataFormFlash(uint8_t robotId);
void writeMotormessage(void);
void readMotormessage(void);

#endif




