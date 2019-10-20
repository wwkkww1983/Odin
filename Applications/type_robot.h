#ifndef __TYPE_ROBOT_H
#define __TYPE_ROBOT_H

#include "NavAndData.h"
#include "bsp.h"

#define TYPE_ROBOT_PRIORITY	12
#define TYPE_ROBOT_STACK_SIZE 512
#define TYPE_ROBOT_PERIOD	10

#define UAV_SBUS DISABLLE											//ʹ��Ϊ����SBUS,ʧ��ΪRM��

#define INFANTRY_DEVICE_LIST	0x07EF						
#define TANK_DEVICE_LIST	0x00FF
#define AUXILIARY_DEVICE_LIST	0x00D7
#define AUXILIARY_SLAVE_DEVICE_LIST 0x0011
#define SENTRY_DEVICE_LIST	0x00EF
#define UAV_DEVICE_LIST	0x004B
#define SMALLGIMBAL_LIST 0x001B

#define CHASSIS_NO_LIMIT	0.0f

#define INFANTR_CHASSIS_CURRENT								64000.0f
#define INFANTR_POWER_LIMIT                   80.0f
#define INFANTR_WARNING_POWER                 40.0f
#define INFANTR_WARNING_POWER_BUFF            50.0f
#define INFANTR_NO_JUDGE_TOTAL_CURRENT_LIMIT  64000.0f
#define INFANTR_JUDGE_TOTAL_CURRENT_LIMIT     32000.0f
#define INFANTR_ADD_POWER_CURRENT  						18000.0f 

#define TANK_CHASSIS_CURRENT							 64000.0f
#define TANK_POWER_LIMIT                   80.0f
#define TANK_WARNING_POWER                 40.0f
#define TANK_WARNING_POWER_BUFF            50.0f
#define TANK_NO_JUDGE_TOTAL_CURRENT_LIMIT  64000.0f
#define TANK_JUDGE_TOTAL_CURRENT_LIMIT     32000.0f
#define TANK_ADD_POWER_CURRENT  					 18000.0f 

#define SENTRY_CHASSIS_CURRENT							 16000.0f
#define SENTRY_POWER_LIMIT                   20.0f
#define SENTRY_WARNING_POWER                 10.0f
#define SENTRY_WARNING_POWER_BUFF            190.0f
#define SENTRY_NO_JUDGE_TOTAL_CURRENT_LIMIT  16000.0f
#define SENTRY_JUDGE_TOTAL_CURRENT_LIMIT     16000.0f
#define SENTRY_ADD_POWER_CURRENT  					 2000.0f 

#define INFANTRY_PITCH_MIN_RANGE -35.0f				//��������С������
#define INFANTRY_PITCH_MAX_RANGE 40.0f				//�������������
#define TANK_PITCH_MIN_RANGE	-10.0f					//Ӣ�۳�����С������
#define TANK_PITCH_MAX_RANGE	55.0f						//Ӣ�۳����������
#define AUXILIARY_PITCH_MIN_RANGE	-30.0f			//���̳�����С������
#define AUXILIARY_PITCH_MAX_RANGE 30.0f				//���̳����������
#define SENTRY_PITCH_MIN_RANGE	-45.0f				//�ڱ�����С������
#define SENTRY_PITCH_MAX_RANGE	0.0f					//�ڱ����������
#define UAV_PITCH_MIN_RANGE	-45.0f						//���˻�����С������
#define UAV_PITCH_MAX_RANGE	10.0f							//���˻����������	
#define SMALLGIMBAL_PITCH_MIN_RANGE -30.0f    //С��̨����С������
#define SMALLGIMBAL_PITCH_MAX_RANGE 20.0f     //С��̨����С������

#define CW_INSTALL 0x02     //���� 
#define CCW_INSTALL 0X00    //����
#define CW_TURN	0x01        //˳ʱ��
#define CCW_TURN 0x00       //��ʱ��         //CW˳ʱ��   CCW��ʱ��

#define INFANTRY_PITCH_INSTALL		CW_INSTALL + CW_TURN
#define INFANTRY_YAW_INSTALL			CW_INSTALL + CW_TURN
#define TANK_PITCH_INSTALL				CCW_INSTALL + CCW_TURN
#define TANK_YAW_INSTALL					CW_INSTALL + CW_TURN
#define TANK_SLAVE_PITCH_INSTALL  CCW_INSTALL + CW_TURN
#define TANK_SLAVE_YAW_INSTALL		CCW_INSTALL + CW_TURN
#define AUXILIARY_PITCH_INSTALL		CW_INSTALL + CW_TURN
#define AUXILIARY_YAW_INSTALL			CW_INSTALL + CW_TURN
#define SENTRY_PITCH_INSTALL			CCW_INSTALL + CW_TURN
#define SENTRY_YAW_INSTALL				CCW_INSTALL + CW_TURN
#define UAV_PITCH_INSTALL					CCW_INSTALL + CW_TURN
#define UAV_YAW_INSTALL						CCW_INSTALL + CW_TURN
#define SMALLGIMBAL_PITCH_INSTALL CCW_INSTALL + CW_TURN
#define SMALLGIMBAL_YAW_INSTALL   CCW_INSTALL + CW_TURN

enum{
	CAMP_BLUE = 0,
	CAMP_RED,
};

enum{
	NO_ID = 0,									//û��ID���Ṥ��
	INFANTRY_ID,								//����
	TANK_ID,										//Ӣ�۳�
	AUXILIARY_ID,								//���̳�
	SENTRY_ID,									//�ڱ�
	UAV_ID,											//���˻�
	SMALLGIMBAL_ID              //С��̨
};

enum{
	NO_WEAPONS = 0,
	SMALL_LAUNCHER,
	BIG_LAUNCHER,
	DOUBLE_LAUNCHER
};

enum robotDeviceList{
	DEVICE_CANSEND = 0x0001,			    //CANͨ��
	DEVICE_GIMBAL = 0x0002,				    //�ø����������̨
	DEVICE_CHASSIS = 0x0004,			    //�ø����������̶���
	DEVICE_SHOOT = 0x0008,				    //�ø������������
	DEVICE_DEFORMING = 0x0010,        //�ø������������ϵͳ
	DEVICE_CURRENT = 0x0020,			    //�ø��������ʰ�
	DEVICE_VISION = 0x0040,				    //�ø������������
	DEVICE_WIFI = 0x0080,					    //�ø��������WIFI��������
	DEVICE_VIEW = 0x0100,						  //�ø���������ӽ��л�����
	DEVICE_SUPPLY = 0x0200,						//�ø���������װ��
	DEVICE_BuiltInPokeMotor = 0x0400  //�ø���������������
};

enum robotDistinguishSate{
	ROBOT_NO_NEED_TO_IDENTIFY = 0,
	ROBOT_BEING_IDENTIFIED,
	ROBOT_COMPLETE_IDENTIFIED
};

typedef struct{
	TaskHandle_t xHandleTask;
	uint8_t typeOfRobot;
	uint8_t camp;
	uint8_t distinguishState;
	uint8_t distinguishLever;
	uint8_t distinguishLastLever;
	float picthMinConstrain;
	float picthMaxConstrain;
	uint16_t robotDeviceList;
	uint32_t loops;
}robotConfigStruct_t;

void currentRobotParameterConfig(void);
void robotDistinguish(void);

extern robotConfigStruct_t robotConfigData; 

#endif
