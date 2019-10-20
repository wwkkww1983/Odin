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
	
	TILT_ANG_P,											//云台pitch、roll角度环参数
	TILT_ANG_I,
	TILT_ANG_D,
	TILT_ANG_F,
	TILT_ANG_PM,
	TILT_ANG_IM,
	TILT_ANG_DM,
	TILT_ANG_OM,
	
	TILT_RATE_P,   									//云台pitch、roll角速度环参数
	TILT_RATE_I,
	TILT_RATE_D,
	TILT_RATE_F,
	TILT_RATE_PM,
	TILT_RATE_IM,
	TILT_RATE_DM,
	TILT_RATE_OM,
	
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
	
	PROPORTIONAL_SPEED_P,						//升降机构速度环
	PROPORTIONAL_SPEED_I,
	PROPORTIONAL_SPEED_D,
	PROPORTIONAL_SPEED_F,
	PROPORTIONAL_SPEED_PM,
	PROPORTIONAL_SPEED_IM,
	PROPORTIONAL_SPEED_DM,
	PROPORTIONAL_SPEED_OM,
	
	PROPORTIONAL_POS_P,							//升降机构位置环
	PROPORTIONAL_POS_I,
	PROPORTIONAL_POS_D,
	PROPORTIONAL_POS_F,
	PROPORTIONAL_POS_PM,
	PROPORTIONAL_POS_IM,
	PROPORTIONAL_POS_DM,
	PROPORTIONAL_POS_OM,
	
	HOLD_PILLAR_SPEED_P,						//抓弹机构速度环
	HOLD_PILLAR_SPEED_I,
	HOLD_PILLAR_SPEED_D,
	HOLD_PILLAR_SPEED_F,
	HOLD_PILLAR_SPEED_PM,
	HOLD_PILLAR_SPEED_IM,
	HOLD_PILLAR_SPEED_DM,
	HOLD_PILLAR_SPEED_OM,
	
	DEFORM1_SPEED_P,								//变形机构1速度环
	DEFORM1_SPEED_I,
	DEFORM1_SPEED_D,
	DEFORM1_SPEED_F,
	DEFORM1_SPEED_PM,
	DEFORM1_SPEED_IM,
	DEFORM1_SPEED_DM,
	DEFORM1_SPEED_OM,
	
	DEFORM2_SPEED_P,								//变形机构2速度环
	DEFORM2_SPEED_I,
	DEFORM2_SPEED_D,
	DEFORM2_SPEED_F,
	DEFORM2_SPEED_PM,
	DEFORM2_SPEED_IM,
	DEFORM2_SPEED_DM,
	DEFORM2_SPEED_OM,

	SHOOT_SPEED_P,									//发射机构速度环
	SHOOT_SPEED_I,
	SHOOT_SPEED_D,
	SHOOT_SPEED_F,
	SHOOT_SPEED_PM,
	SHOOT_SPEED_IM,
	SHOOT_SPEED_DM,
	SHOOT_SPEED_OM,
	
	LOADED_SPEED_P,									//拨弹机构速度环
	LOADED_SPEED_I,
	LOADED_SPEED_D,
	LOADED_SPEED_F,
	LOADED_SPEED_PM,
	LOADED_SPEED_IM,
	LOADED_SPEED_DM,
	LOADED_SPEED_OM,
	
	ADRC_R,													//底盘速度ADRC
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
/*----------------	以下参数存储到TF卡另一个文件	（motor文件）----------------*/
	LOCAL_ID,                           //该段参数与下电机参数文件列表参数相同，只是名字不同
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

enum motormessagelist{                                //电机参数文件 
  ROBOT_ID,                //机器人种类               
	NATIVE_ID,              //主副控ID  同LOCAL_ID 下同
	ARMS_TYPE,              //武器类型
	PITCH_ORIENT,           //pitch轴总方向
	YAW_ORIENT,             //yaw轴总方向
	RETURN_TIME,            //回中时间
	CHASSIS_POWER,          //底盘最大功率
	RC_RATIO,               //遥控器最大值
	YAW_MIDDLE,             //yaw轴中间位置 
	PITCH_MIDDLE,           //pitch轴中间位置
	PITCH_MIN_SEAT,         //piych轴最大角度 
	PITCH_MAX_SEAT,         //pitch轴最小角度
	YAW_ID,                 //yaw轴电机种类
  PITCH_ID,               //pitch轴电机种类          
  YAW_FASTEN,             //yaw轴电机安装方向        
	YAW_SPIN,               //yaw轴旋转方向
	PITCH_FASTEN,            //pitch轴电机安装方向      
	PITCH_SPIN,              //pitch轴旋转方向  
  IMU_ACC_INIT_X,
	IMU_ACC_INIT_Y,
	IMU_ACC_INIT_Z,
	IMU_MAG_INIT_X,
	IMU_MAG_INIT_Y,
	IMU_MAG_INIT_Z,
	IMU_GYO_INIT_X,
	IMU_GYO_INIT_Y,
	IMU_GYO_INIT_Z,
	ABOVE_TOTAL              //以上参数总数量  
};

typedef struct{
	uint8_t TFInsertState;						//这一刻TF插入的状态
	uint8_t TFInsertLastState;				//上一刻TF卡插入的状态
	uint8_t TFInsertFlag;							//TF卡插入检测
	uint8_t TFError;									//TF错误标志
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

#define waitForFlag(xFLAG)	xTaskNotifyWait(0x00000000,0xFFFFFFFF,&xFLAG,portMAX_DELAY);					//最久等待500ms

extern parameterStruct_t parameterRunData;

void tFCardUpdate(void);
uint8_t parameterWriteDataFormFlash(uint8_t robotId);
uint8_t parameterReadDataFromTFCard(uint8_t robotId);
uint8_t motorMessageReadDataFromTFCard(uint8_t robotId);
uint8_t motorMessageWriteDataFormFlash(uint8_t robotId);
void writeMotormessage(void);
void readMotormessage(void);

#endif




