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
	
	YAW_ANG_P,											//云台yaw角度环参数
	YAW_ANG_I,
	YAW_ANG_D,
	YAW_ANG_F,
	YAW_ANG_PM,
	YAW_ANG_IM,
	YAW_ANG_DM,
	YAW_ANG_OM,
	
	YAW_RATE_P,    									//云台yaw角速度环参数
	YAW_RATE_I,
	YAW_RATE_D,
	YAW_RATE_F,
	YAW_RATE_PM,
	YAW_RATE_IM,
	YAW_RATE_DM,
	YAW_RATE_OM,
	
	PITCH_ANG_P,											//云台pitch、roll角度环参数
	PITCH_ANG_I,
	PITCH_ANG_D,
	PITCH_ANG_F,
	PITCH_ANG_PM,
	PITCH_ANG_IM,
	PITCH_ANG_DM,
	PITCH_ANG_OM,
	
	PITCH_RATE_P,   									//云台pitch、roll角速度环参数
	PITCH_RATE_I,
	PITCH_RATE_D,
	PITCH_RATE_F,
	PITCH_RATE_PM,
	PITCH_RATE_IM,
	PITCH_RATE_DM,
	PITCH_RATE_OM,
	
	ROLL_ANG_P,											//云台yaw角度环参数
	ROLL_ANG_I,
	ROLL_ANG_D,
	ROLL_ANG_F,
	ROLL_ANG_PM,
	ROLL_ANG_IM,
	ROLL_ANG_DM,
	ROLL_ANG_OM,
	
	ROLL_RATE_P,    									//云台ROLL角速度环参数
	ROLL_RATE_I,
	ROLL_RATE_D,
	ROLL_RATE_F,
	ROLL_RATE_PM,
	ROLL_RATE_IM,
	ROLL_RATE_DM,
	ROLL_RATE_OM,
	
	CHASSIS_SPEED_P,								//底盘速度环参数
	CHASSIS_SPEED_I,
	CHASSIS_SPEED_D,
	CHASSIS_SPEED_F,
	CHASSIS_SPEED_PM,
	CHASSIS_SPEED_IM,
	CHASSIS_SPEED_DM,
	CHASSIS_SPEED_OM,
	
	CHASSIS_POS_P,									//底盘位置环参数
	CHASSIS_POS_I,
	CHASSIS_POS_D,
	CHASSIS_POS_F,
	CHASSIS_POS_PM,
	CHASSIS_POS_IM,
	CHASSIS_POS_DM,
	CHASSIS_POS_OM,
	
	CHASSIS_CHASE_P,								//底盘跟随角速度环参数
	CHASSIS_CHASE_I,
	CHASSIS_CHASE_D,
	CHASSIS_CHASE_F,
	CHASSIS_CHASE_PM,
	CHASSIS_CHASE_IM,
	CHASSIS_CHASE_DM,
	CHASSIS_CHASE_OM,
	
	CHASSIS_RATE_P,									//底盘跟随角速度环参数
	CHASSIS_RATE_I,
	CHASSIS_RATE_D,
	CHASSIS_RATE_F,
	CHASSIS_RATE_PM,
	CHASSIS_RATE_IM,
	CHASSIS_RATE_DM,
	CHASSIS_RATE_OM,
	
	POWER_LIMIT_P,									//底盘功率环参数
	POWER_LIMIT_I,
	POWER_LIMIT_D,
	POWER_LIMIT_F,
	POWER_LIMIT_PM,
	POWER_LIMIT_IM,
	POWER_LIMIT_DM,
	POWER_LIMIT_OM,
	
	SHOOT_SPEED_P,									//发射机构速度环
	SHOOT_SPEED_I,
	SHOOT_SPEED_D,
	SHOOT_SPEED_F,
	SHOOT_SPEED_PM,
	SHOOT_SPEED_IM,
	SHOOT_SPEED_DM,
	SHOOT_SPEED_OM,
	
	ROLLBULL_SPEED_P,								//大弹丸拨弹机构
	ROLLBULL_SPEED_I,
	ROLLBULL_SPEED_D,
	ROLLBULL_SPEED_F,
	ROLLBULL_SPEED_PM,
	ROLLBULL_SPEED_IM,
	ROLLBULL_SPEED_DM,
	ROLLBULL_SPEED_OM,
	
	LOADED_SPEED_P,									//拨弹机构速度环
	LOADED_SPEED_I,
	LOADED_SPEED_D,
	LOADED_SPEED_F,
	LOADED_SPEED_PM,
	LOADED_SPEED_IM,
	LOADED_SPEED_DM,
	LOADED_SPEED_OM,


/*----------------	以下参数存储到TF卡另一个文件	（motor文件）----------------*/
	LOCAL_ID,                           //该段参数与下电机参数文件列表参数相同，只是名字不同
	WEAPON_TYPE,
	YAW_INSTALL,
	PITCH_INSTALL,
	ROLL_INSTALL,
	BACK_CENTER_TIME,
	CHASSIS_CURRENT,
	RC_RESOLUTION,
	YAW_CENTER,	
	PITCH_CENTER,
	ROLL_CENTER,
	PITCH_MIN_RANGE,
	PITCH_MAX_RANGE,
	YAW_TYPE,       
	PITCH_TYPE,
	ROLL_TYPE,
	YAW_FIX,
	YAW_TURN,
	PITCH_FIX,
	PITCH_TURN,   
	ROLL_FIX,
	ROLL_TURN,  	
	NUM_OF_LIST,
};


typedef struct{
	uint8_t TFInsertState;						//这一刻TF插入的状态
	uint8_t TFInsertLastState;				//上一刻TF卡插入的状态
	uint8_t TFInsertFlag;							//TF卡插入检测
	uint8_t TFError;									//TF错误标志
}parameterStruct_t;

#define TFCARD_NUM_LIST NUM_OF_LIST
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

#define waitForFlag(xFLAG)	xTaskNotifyWait(0x00000000,0xFFFFFFFF,&xFLAG,portMAX_DELAY);					//最久等待500ms

extern parameterStruct_t parameterRunData;

void tFCardUpdate(void);
uint8_t parameterWriteDataFormFlash(uint8_t robotId);
uint8_t parameterReadDataFromTFCard(uint8_t robotId);
void writeMotormessage(void);
void readMotormessage(void);

#endif




