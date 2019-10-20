#include "Driver_Flash.h"
#include "config.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "Util.h"
#include "tf_card_parameter.h"
#include "type_robot.h"
#include "supervisor.h"

float parameter[NUM_OF_LIST] __attribute__((section(".ccm")));
float motormessage[ABOVE_TOTAL] __attribute__((section(".ccm")));
/*----------------------------
== 此处的命名不可超过16个字符 ==
----------------------------*/
const char *configParameterStrings[] = {
	"CONFIG_VERSION",									//版本号
	"ROBOT_TYPE",
	"DEAD_BAND",
	"SHOOT_LOW_PWM",
	"SHOOT_HIGH_PWM",
	"MOUSE_FEEL",
	"GIMBAL_CTR_SCALE",
	"GIMBAL_KB_SCALE",
	"CHASSIS_KB_SPEED",
	"CHASSIS_RC_SPEED",
	"CHASSIS_KB_ACC",
	
	"TILT_ANGLE_P",
	"TILT_ANGLE_I",
	"TILT_ANGLE_D",
	"TILT_ANGLE_F",
	"TILT_ANGLE_PM",
	"TILT_ANGLE_IM",
	"TILT_ANGLE_DM",
	"TILT_ANGLE_OM",
	
	"TILT_RATE_P",
	"TILT_RATE_I",
	"TILT_RATE_D",
	"TILT_RATE_F",
	"TILT_RATE_PM",
	"TILT_RATE_IM",
	"TILT_RATE_DM",
	"TILT_RATE_OM",
	
	"YAW_ANGLE_P",
	"YAW_ANGLE_I",
	"YAW_ANGLE_D",
	"YAW_ANGLE_F",
	"YAW_ANGLE_PM",
	"YAW_ANGLE_IM",
	"YAW_ANGLE_DM",
	"YAW_ANGLE_OM",
	
	"YAW_RATE_P",
	"YAW_RATE_I",
	"YAW_RATE_D",
	"YAW_RATE_F",
	"YAW_RATE_PM",
	"YAW_RATE_IM",
	"YAW_RATE_DM",
	"YAW_RATE_OM",
	
	"CHASSIS_SPEED_P",
	"CHASSIS_SPEED_I",
	"CHASSIS_SPEED_D",
	"CHASSIS_SPEED_F",
	"CHASSIS_SPEED_PM",
	"CHASSIS_SPEED_IM",
	"CHASSIS_SPEED_DM",
	"CHASSIS_SPEED_OM",
	
	"CHASSIS_POS_P",
	"CHASSIS_POS_I",
	"CHASSIS_POS_D",
	"CHASSIS_POS_F",
	"CHASSIS_POS_PM",
	"CHASSIS_POS_IM",
	"CHASSIS_POS_DM",
	"CHASSIS_POS_OM",
	
	"CHASSIS_CHASE_P",
	"CHASSIS_CHASE_I",
	"CHASSIS_CHASE_D",
	"CHASSIS_CHASE_F",
	"CHASSIS_CHASE_PM",
	"CHASSIS_CHASE_IM",
	"CHASSIS_CHASE_DM",
	"CHASSIS_CHASE_OM",
	
	"CHASSIS_RATE_P",								//底盘跟随角速度环参数  IMU反馈角速度闭环
	"CHASSIS_RATE_I",
	"CHASSIS_RATE_D",
	"CHASSIS_RATE_F",
	"CHASSIS_RATE_PM",
	"CHASSIS_RATE_IM",
	"CHASSIS_RATE_DM",
	"CHASSIS_RATE_OM",
	
	"POWER_LIMIT_P",
	"POWER_LIMIT_I",
	"POWER_LIMIT_D",
	"POWER_LIMIT_F",
	"POWER_LIMIT_PM",
	"POWER_LIMIT_IM",
	"POWER_LIMIT_DM",
	"POWER_LIMIT_OM",
	
	"PRO_SPEED_P",
	"PRO_SPEED_I",
	"PRO_SPEED_D",
	"PRO_SPEED_F",
	"PRO_SPEED_PM",
	"PRO_SPEED_IM",
	"PRO_SPEED_DM",
	"PRO_SPEED_OM",
	
	"PRO_POS_P",
	"PRO_POS_I",
	"PRO_POS_D",
	"PRO_POS_F",
	"PRO_POS_PM",
	"PRO_POS_IM",
	"PRO_POS_DM",
	"PRO_POS_OM",
	
	"HOLD_SPEED_P",									
	"HOLD_SPEED_I",
	"HOLD_SPEED_D",
	"HOLD_SPEED_F",
	"HOLD_SPEED_FM",
	"HOLD_SPEED_IM",
	"HOLD_SPEED_DM",
	"HOLD_SPEED_OM",
	
	"DEFORM1_SPEED_P",				
	"DEFORM1_SPEED_I",
	"DEFORM1_SPEED_D",
	"DEFORM1_SPEED_F",
	"DEFORM1_SPEED_PM",
	"DEFORM1_SPEED_IM",
	"DEFORM1_SPEED_DM",
	"DEFORM1_SPEED_OM",
	
	"DEFORM2_SPEED_P",				
	"DEFORM2_SPEED_I",
	"DEFORM2_SPEED_D",
	"DEFORM2_SPEED_F",
	"DEFORM2_SPEED_PM",
	"DEFORM2_SPEED_IM",
	"DEFORM2_SPEED_DM",
	"DEFORM2_SPEED_OM",
	
	"SHOOT_SPEED_P",
	"SHOOT_SPEED_I",
	"SHOOT_SPEED_D",
	"SHOOT_SPEED_F",
	"SHOOT_SPEED_PM",
	"SHOOT_SPEED_IM",
	"SHOOT_SPEED_DM",
	"SHOOT_SPEED_OM",
	
	"LOADED_SPEED_P",
	"LOADED_SPEED_I",
	"LOADED_SPEED_D",
	"LOADED_SPEED_F",
	"LOADED_SPEED_PM",
	"LOADED_SPEED_IM",
	"LOADED_SPEED_DM",
	"LOADED_SPEED_OM",
	
	"ADRC_R",
	"ADRC_H",
	"ADRC_N0",
	"ADRC_BETA01",
	"ADRC_BETA02",
	"ADRC_BETA03",
	"ADRC_B0",
	"ADRC_BETA0",
	"ADRC_BETA1",
	"ADRC_BETA2",
	"ADRC_N1",
	"ADRC_C",
	"ADRC_ALPHA1",
	"ADRC_ALPHA2",
	"ADRC_ZETA",
	"ADRC_B",
	"ADRC_OMAX",
/*----------------		以下参数存储到TF卡另一个文件	（motor文件）		----------------*/
	"LOCAL_ID",
	"WEAPON_TYPE",
	"PITCH_INSTALL",
	"YAW_INSTALL",
	"BACK_CENTER_TIME",
	"CHASSIS_CURRENT",
	"RC_RESOLUTION",
	"YAW_CENTER",	
	"PITCH_CENTER",
	"PITCH_MIN_RANGE",
	"PITCH_MAX_RANGE",
	"YAW_TYPE",       
	"PITCH_TYPE",
	"YAW_FIX",
	"YAW_TURN",
	"PITCH_FIX",
	"PITCH_TURN",
	"IMU_ACC_BIAS_X",
	"IMU_ACC_BIAS_Y",
	"IMU_ACC_BIAS_Z",
	"IMU_MAG_BIAS_X",
	"IMU_MAG_BIAS_Y",
	"IMU_MAG_BIAS_Z",
	"IMU_GYO_BIAS_X",
	"IMU_GYO_BIAS_Y",
	"IMU_GYO_BIAS_Z",
};

const char *configMotorStrings[] = {   //电机配置文件         /*************测试*************/
  "ROBOT_ID",                //机器人种类               
	"NATIVE_ID",              //主副控ID  同LOCAL_ID 下同
	"ARMS_TYPE",              //武器类型
	"PITCH_ORIENT",           //pitch轴总方向
	"YAW_ORIENT",             //yaw轴总方向
	"RETURN_TIME",            //回中时间
	"CHASSIS_POWER",          //底盘最大功率
	"RC_RATIO",               //遥控器最大值
	"YAW_MIDDLE",             //yaw轴中间位置 
	"PITCH_MIDDLE",           //pitch轴中间位置
	"PITCH_MIN_SEAT",         //piych轴最大角度 
	"PITCH_MAX_SEAT",         //pitch轴最小角度
	"YAW_ID",                 //yaw轴电机种类
  "PITCH_ID",               //pitch轴电机种类          
  "YAW_FASTEN",             //yaw轴电机安装方向        
	"YAW_SPIN",               //yaw轴旋转方向
	"PITCH_FASTEN",            //pitch轴电机安装方向      
	"PITCH_SPIN",              //pitch轴旋转方向  
  "IMU_ACC_INIT_X",
	"IMU_ACC_INIT_Y",
	"IMU_ACC_INIT_Z",
	"IMU_MAG_INIT_X",
	"IMU_MAG_INIT_Y",
	"IMU_MAG_INIT_Z",
	"IMU_GYO_INIT_X",
	"IMU_GYO_INIT_Y",
	"IMU_GYO_INIT_Z",
};

void configLoadDefault(void){	
	parameter[CONFIG_VERSION] = DEFAULT_CONFIG_VERSION;
	parameter[ROBOT_TYPE] = DEFAULT_ROBOT_TYPE;
	parameter[DEAD_BAND] = DEFAULT_DEAD_BAND;
	parameter[SHOOT_LOW_PWM] = DEFAULT_SHOOT_LOW_PWM;
	parameter[SHOOT_HIGH_PWM] = DEFAULT_SHOOT_HIGH_PWM;
	parameter[MOUSE_FEEL] = DEFAULT_MOUSE_FEEL;
	parameter[GIMBAL_CTR_SCALE] = DEFAULT_GIMBAL_CTR_SCALE;
	parameter[GIMBAL_KB_SCALE] = DEFAULT_GIMBAL_KB_SCALE;
	parameter[CHASSIS_KB_SPEED] = DEFAULT_CHASSIS_KB_SPEED;
	parameter[CHASSIS_RC_SPEED] = DEFAULT_CHASSIS_RC_SPEED;
	parameter[CHASSIS_KB_ACC] = DEFAULT_CHASSIS_KB_ACC;
	
	parameter[TILT_ANG_P] = DEFAULT_TILT_ANGLE_P;
	parameter[TILT_ANG_I] = DEFAULT_TILT_ANGLE_I;
	parameter[TILT_ANG_D] = DEFAULT_TILT_ANGLE_D;
	parameter[TILT_ANG_F] = DEFAULT_TILT_ANGLE_F;
	parameter[TILT_ANG_PM] = DEFAULT_TILT_ANGLE_PM;
	parameter[TILT_ANG_IM] = DEFAULT_TILT_ANGLE_IM;
	parameter[TILT_ANG_DM] = DEFAULT_TILT_ANGLE_DM;
	parameter[TILT_ANG_OM] = DEFAULT_TILT_ANGLE_OM;
	
	parameter[TILT_RATE_P] = DEFAULT_TILT_RATE_P;
	parameter[TILT_RATE_I] = DEFAULT_TILT_RATE_I;
	parameter[TILT_RATE_D] = DEFAULT_TILT_RATE_D;
	parameter[TILT_RATE_F] = DEFAULT_TILT_RATE_F;
	parameter[TILT_RATE_PM] = DEFAULT_TILT_RATE_PM;
	parameter[TILT_RATE_IM] = DEFAULT_TILT_RATE_IM;
	parameter[TILT_RATE_DM] = DEFAULT_TILT_RATE_DM;
	parameter[TILT_RATE_OM] = DEFAULT_TILT_RATE_OM;
	
	parameter[YAW_ANG_P] = DEFAULT_YAW_ANGLE_P;
	parameter[YAW_ANG_I] = DEFAULT_YAW_ANGLE_I;
	parameter[YAW_ANG_D] = DEFAULT_YAW_ANGLE_D;
	parameter[YAW_ANG_F] = DEFAULT_YAW_ANGLE_F;
	parameter[YAW_ANG_PM] = DEFAULT_YAW_ANGLE_PM;
	parameter[YAW_ANG_IM] = DEFAULT_YAW_ANGLE_IM;
	parameter[YAW_ANG_DM] = DEFAULT_YAW_ANGLE_DM;
	parameter[YAW_ANG_OM] = DEFAULT_YAW_ANGLE_OM;
	
	parameter[YAW_RATE_P] = DEFAULT_YAW_RATE_P;
	parameter[YAW_RATE_I] = DEFAULT_YAW_RATE_I;
	parameter[YAW_RATE_D] = DEFAULT_YAW_RATE_D;
	parameter[YAW_RATE_F] = DEFAULT_YAW_RATE_F;
	parameter[YAW_RATE_PM] = DEFAULT_YAW_RATE_PM;
	parameter[YAW_RATE_IM] = DEFAULT_YAW_RATE_IM;
	parameter[YAW_RATE_DM] = DEFAULT_YAW_RATE_DM;
	parameter[YAW_RATE_OM] = DEFAULT_YAW_RATE_OM;
	
	parameter[CHASSIS_SPEED_P] = DEFAULT_CHASSIS_SPEED_P;
	parameter[CHASSIS_SPEED_I] = DEFAULT_CHASSIS_SPEED_I;
	parameter[CHASSIS_SPEED_D] = DEFAULT_CHASSIS_SPEED_D;
	parameter[CHASSIS_SPEED_F] = DEFAULT_CHASSIS_SPEED_F;
	parameter[CHASSIS_SPEED_PM] = DEFAULT_CHASSIS_SPEED_PM;
	parameter[CHASSIS_SPEED_IM] = DEFAULT_CHASSIS_SPEED_IM;
	parameter[CHASSIS_SPEED_DM] = DEFAULT_CHASSIS_SPEED_DM;
	parameter[CHASSIS_SPEED_OM] = DEFAULT_CHASSIS_SPEED_OM;
	
	parameter[CHASSIS_POS_P] = DEFAULT_CHASSIS_POS_P;
	parameter[CHASSIS_POS_I] = DEFAULT_CHASSIS_POS_I;
	parameter[CHASSIS_POS_D] = DEFAULT_CHASSIS_POS_D;
	parameter[CHASSIS_POS_F] = DEFAULT_CHASSIS_POS_F;
	parameter[CHASSIS_POS_PM] = DEFAULT_CHASSIS_POS_PM;
	parameter[CHASSIS_POS_IM] = DEFAULT_CHASSIS_POS_IM;
	parameter[CHASSIS_POS_DM] = DEFAULT_CHASSIS_POS_DM;
	parameter[CHASSIS_POS_OM] = DEFAULT_CHASSIS_POS_OM;
	
	parameter[CHASSIS_CHASE_P] = DEFAULT_CHASSIS_CHASE_P;
	parameter[CHASSIS_CHASE_I] = DEFAULT_CHASSIS_CHASE_I;
	parameter[CHASSIS_CHASE_D] = DEFAULT_CHASSIS_CHASE_D;
	parameter[CHASSIS_CHASE_F] = DEFAULT_CHASSIS_CHASE_F;
	parameter[CHASSIS_CHASE_PM] = DEFAULT_CHASSIS_CHASE_PM;
	parameter[CHASSIS_CHASE_IM] = DEFAULT_CHASSIS_CHASE_IM;
	parameter[CHASSIS_CHASE_DM] = DEFAULT_CHASSIS_CHASE_DM;
	parameter[CHASSIS_CHASE_OM] = DEFAULT_CHASSIS_CHASE_OM;	
	
	parameter[CHASSIS_RATE_P] = DEFAULT_CHASSIS_RATE_P;								//底盘跟随角速度环参数
	parameter[CHASSIS_RATE_I] = DEFAULT_CHASSIS_RATE_I;
	parameter[CHASSIS_RATE_D] = DEFAULT_CHASSIS_RATE_D;
	parameter[CHASSIS_RATE_F] = DEFAULT_CHASSIS_RATE_F;
	parameter[CHASSIS_RATE_PM] = DEFAULT_CHASSIS_RATE_PM;
	parameter[CHASSIS_RATE_IM] = DEFAULT_CHASSIS_RATE_IM;
	parameter[CHASSIS_RATE_DM] = DEFAULT_CHASSIS_RATE_DM;
	parameter[CHASSIS_RATE_OM] = DEFAULT_CHASSIS_RATE_OM;
	
	parameter[POWER_LIMIT_P] = DEFAULT_POWER_LIMIT_P;
	parameter[POWER_LIMIT_I] = DEFAULT_POWER_LIMIT_I;
	parameter[POWER_LIMIT_D] = DEFAULT_POWER_LIMIT_D;
	parameter[POWER_LIMIT_F] = DEFAULT_POWER_LIMIT_F;
	parameter[POWER_LIMIT_PM] = DEFAULT_POWER_LIMIT_PM;
	parameter[POWER_LIMIT_IM] = DEFAULT_POWER_LIMIT_IM;
	parameter[POWER_LIMIT_DM] = DEFAULT_POWER_LIMIT_DM;
	parameter[POWER_LIMIT_OM] = DEFAULT_POWER_LIMIT_OM;	
	
	parameter[PROPORTIONAL_SPEED_P] = DEFAULT_PROPORTIONAL_SPEED_P;
	parameter[PROPORTIONAL_SPEED_I] = DEFAULT_PROPORTIONAL_SPEED_I;
	parameter[PROPORTIONAL_SPEED_D] = DEFAULT_PROPORTIONAL_SPEED_D;
	parameter[PROPORTIONAL_SPEED_F] = DEFAULT_PROPORTIONAL_SPEED_F;
	parameter[PROPORTIONAL_SPEED_PM] = DEFAULT_PROPORTIONAL_SPEED_PM;
	parameter[PROPORTIONAL_SPEED_IM] = DEFAULT_PROPORTIONAL_SPEED_IM;
	parameter[PROPORTIONAL_SPEED_DM] = DEFAULT_PROPORTIONAL_SPEED_DM;
	parameter[PROPORTIONAL_SPEED_OM] = DEFAULT_PROPORTIONAL_SPEED_OM;	
	
	parameter[PROPORTIONAL_POS_P] = DEFAULT_PROPORTIONAL_POS_P;
	parameter[PROPORTIONAL_POS_I] = DEFAULT_PROPORTIONAL_POS_I;
	parameter[PROPORTIONAL_POS_D] = DEFAULT_PROPORTIONAL_POS_D;
	parameter[PROPORTIONAL_POS_F] = DEFAULT_PROPORTIONAL_POS_F;
	parameter[PROPORTIONAL_POS_PM] = DEFAULT_PROPORTIONAL_POS_PM;
	parameter[PROPORTIONAL_POS_IM] = DEFAULT_PROPORTIONAL_POS_IM;
	parameter[PROPORTIONAL_POS_DM] = DEFAULT_PROPORTIONAL_POS_DM;
	parameter[PROPORTIONAL_POS_OM] = DEFAULT_PROPORTIONAL_POS_OM;	
	
	parameter[HOLD_PILLAR_SPEED_P] = DEFAULT_HOLD_PILLAR_SPEED_P;
	parameter[HOLD_PILLAR_SPEED_I] = DEFAULT_HOLD_PILLAR_SPEED_I;
	parameter[HOLD_PILLAR_SPEED_D] = DEFAULT_HOLD_PILLAR_SPEED_D;
	parameter[HOLD_PILLAR_SPEED_F] = DEFAULT_HOLD_PILLAR_SPEED_F;
	parameter[HOLD_PILLAR_SPEED_PM] = DEFAULT_HOLD_PILLAR_SPEED_PM;
	parameter[HOLD_PILLAR_SPEED_IM] = DEFAULT_HOLD_PILLAR_SPEED_IM;
	parameter[HOLD_PILLAR_SPEED_DM] = DEFAULT_HOLD_PILLAR_SPEED_DM;
	parameter[HOLD_PILLAR_SPEED_OM] = DEFAULT_HOLD_PILLAR_SPEED_OM;	
	
	parameter[DEFORM1_SPEED_P] = DEFAULT_DEFORM1_SPEED_P;
	parameter[DEFORM1_SPEED_I] = DEFAULT_DEFORM1_SPEED_I;
	parameter[DEFORM1_SPEED_D] = DEFAULT_DEFORM1_SPEED_D;
	parameter[DEFORM1_SPEED_F] = DEFAULT_DEFORM1_SPEED_F;
	parameter[DEFORM1_SPEED_PM] = DEFAULT_DEFORM1_SPEED_PM;
	parameter[DEFORM1_SPEED_IM] = DEFAULT_DEFORM1_SPEED_IM;
	parameter[DEFORM1_SPEED_DM] = DEFAULT_DEFORM1_SPEED_DM;
	parameter[DEFORM1_SPEED_OM] = DEFAULT_DEFORM1_SPEED_OM;
	
	parameter[DEFORM2_SPEED_P] = DEFAULT_DEFORM2_SPEED_P;
	parameter[DEFORM2_SPEED_I] = DEFAULT_DEFORM2_SPEED_I;
	parameter[DEFORM2_SPEED_D] = DEFAULT_DEFORM2_SPEED_D;
	parameter[DEFORM2_SPEED_F] = DEFAULT_DEFORM2_SPEED_F;
	parameter[DEFORM2_SPEED_PM] = DEFAULT_DEFORM2_SPEED_PM;
	parameter[DEFORM2_SPEED_IM] = DEFAULT_DEFORM2_SPEED_IM;
	parameter[DEFORM2_SPEED_DM] = DEFAULT_DEFORM2_SPEED_DM;
	parameter[DEFORM2_SPEED_OM] = DEFAULT_DEFORM2_SPEED_OM;
	
	parameter[SHOOT_SPEED_P] = DEFAULT_SHOOT_SPEED_P;
	parameter[SHOOT_SPEED_I] = DEFAULT_SHOOT_SPEED_I;
	parameter[SHOOT_SPEED_D] = DEFAULT_SHOOT_SPEED_D;
	parameter[SHOOT_SPEED_F] = DEFAULT_SHOOT_SPEED_F;
	parameter[SHOOT_SPEED_PM] = DEFAULT_SHOOT_SPEED_PM;
	parameter[SHOOT_SPEED_IM] = DEFAULT_SHOOT_SPEED_IM;
	parameter[SHOOT_SPEED_DM] = DEFAULT_SHOOT_SPEED_DM;
	parameter[SHOOT_SPEED_OM] = DEFAULT_SHOOT_SPEED_OM;	
	
	parameter[LOADED_SPEED_P] = DEFAULT_LOADED_SPEED_P;
	parameter[LOADED_SPEED_I] = DEFAULT_LOADED_SPEED_I;
	parameter[LOADED_SPEED_D] = DEFAULT_LOADED_SPEED_D;
	parameter[LOADED_SPEED_F] = DEFAULT_LOADED_SPEED_F;
	parameter[LOADED_SPEED_PM] = DEFAULT_LOADED_SPEED_PM;
	parameter[LOADED_SPEED_IM] = DEFAULT_LOADED_SPEED_IM;
	parameter[LOADED_SPEED_DM] = DEFAULT_LOADED_SPEED_DM;
	parameter[LOADED_SPEED_OM] = DEFAULT_LOADED_SPEED_OM;			
	
	parameter[ADRC_R] = DEFAULT_ADRC_R;						
	parameter[ADRC_H] = DEFAULT_ADRC_H;						
	parameter[ADRC_N0] = DEFAULT_ADRC_N0;					
	parameter[ADRC_BETA01] = DEFAULT_ADRC_BETA01;	
	parameter[ADRC_BETA02] = DEFAULT_ADRC_BETA02;	
	parameter[ADRC_BETA03] = DEFAULT_ADRC_BETA03;	
	parameter[ADRC_B0] = DEFAULT_ADRC_B0;					
	parameter[ADRC_BETA0] = DEFAULT_ADRC_BETA0;		
	parameter[ADRC_BETA1] = DEFAULT_ADRC_BETA1;		
	parameter[ADRC_BETA02] = DEFAULT_ADRC_BETA2;	
	parameter[ADRC_N1] = DEFAULT_ADRC_N1;					
	parameter[ADRC_C] = DEFAULT_ADRC_C;						
	parameter[ADRC_ALPHA1] = DEFAULT_ADRC_ALPHA1;	
	parameter[ADRC_ALPHA2] = DEFAULT_ADRC_ALPHA2;	
	parameter[ADRC_ZETA] = DEFAULT_ADRC_ZETA;			
	parameter[ADRC_B] = DEFAULT_ADRC_B;						
	parameter[ADRC_OMAX] = DEFAULT_ADRC_OMAX;			
	
/*----------------	以下参数存储到TF卡另一个文件	（motor文件）	----------------*/
	parameter[LOCAL_ID] = DEFAULT_LOCAL_ID;											
	parameter[WEAPON_TYPE] = DEFAULT_WEAPON_TYPE;								
	parameter[PITCH_INSTALL] = DEFAULT_PITCH_INSTALL;				
	parameter[YAW_INSTALL] = DEFAULT_YAW_INSTALL;			
	parameter[BACK_CENTER_TIME] = DEFAULT_BACK_CENTER_TIME;			
	parameter[CHASSIS_CURRENT] = DEFAULT_CHASSIS_CURRENT;				
	parameter[RC_RESOLUTION] = DEFAULT_RC_RESOLUTION;						
	parameter[YAW_CENTER] = DEFAULT_YAW_CENTER;								
	parameter[PITCH_CENTER] = DEFAULT_PITCH_CENTER;														
	parameter[PITCH_MIN_RANGE] = DEFAULT_PITCH_MIN_RANGE;				
	parameter[PITCH_MAX_RANGE] = DEFAULT_PITCH_MAX_RANGE;
  parameter[YAW_TYPE] =	DEFAULT_YAW_TYPE;
	parameter[PITCH_TYPE] =	DEFAULT_PITCH_TYPE; 
	parameter[YAW_FIX] =	DEFAULT_YAW_FIX;
	parameter[YAW_TURN] =	DEFAULT_YAW_TURN;
	parameter[PITCH_FIX] =	DEFAULT_PITCH_FIX; 
	parameter[PITCH_TURN] =	DEFAULT_PITCH_TURN;
	parameter[IMU_ACC_BIAS_X] = DEFAULT_IMU_ACC_BIAS_X;					
	parameter[IMU_ACC_BIAS_Y] = DEFAULT_IMU_ACC_BIAS_Y;					
	parameter[IMU_ACC_BIAS_Z] = DEFAULT_IMU_ACC_BIAS_Z;					
	parameter[IMU_MAG_BIAS_X] = DEFAULT_IMU_MAG_BIAS_X;					
	parameter[IMU_MAG_BIAS_Y] = DEFAULT_IMU_MAG_BIAS_Y;					
	parameter[IMU_MAG_BIAS_Z] = DEFAULT_IMU_MAG_BIAS_Z;					
	parameter[IMU_GYO_BIAS_X] = DEFAULT_IMU_GYO_BIAS_X;					
	parameter[IMU_GYO_BIAS_Y] = DEFAULT_IMU_GYO_BIAS_Y;					
	parameter[IMU_GYO_BIAS_Z] = DEFAULT_IMU_GYO_BIAS_Z;				
}


configToken_t *configTokenFindEmpty(void) {
	configToken_t *p = (configToken_t *)(FLASH_END_ADDR + 1);
	do {
		p--;
	} while (p->key != 0xffffffff);
	return p;
}

void configTokenStore(configToken_t *token) {
  flashAddress((uint32_t)configTokenFindEmpty(), (uint32_t *)token, sizeof(configToken_t)/sizeof(uint32_t));
}

configToken_t *configTokenGet(uint32_t key) {
	configToken_t *p, *t;
	p = (configToken_t *)(FLASH_END_ADDR + 1);
	t = 0;
	do {
		p--;
		if (p->key == key)
			t = p;
	} while (p->key != 0xffffffff);
	return t;
}
/*------------------- 配置FLASH读取 ---------------------*/
void configFlashRead(void) {
	configRec_t *recs;
	int i, j;
	recs = (void *)flashStartAddr();
	for (i = 0; i < NUM_OF_LIST; i++) {
		for (j = 0; j < NUM_OF_LIST; j++)																							//目的是对两个变量的名称进行对比，名称一致则读取flash所带的值
			if (!strncasecmp(recs[i].name, configParameterStrings[j], 16))					
				parameter[j] = recs[i].val;															
	}
}

configToken_t *configTokenIterate(configToken_t *t) {
	if (t == 0)
		t = (configToken_t *)(FLASH_END_ADDR + 1);
	t--;
	if (t->key != 0xffffffff)
		return t;
	else
		return 0;
}
/*------------------- 配置FLASH写入 ---------------------*/
uint8_t configFlashWrite(void) {
	configRec_t *recs;
	uint8_t ret = 0;
	int i;
	recs = (void *)aqCalloc(NUM_OF_LIST, sizeof(configRec_t));
	if (recs) {
		configToken_t *tr = (configToken_t *)recs;
		configToken_t *tf = 0;																			
		do {																									//读取所有令牌
			tf = configTokenIterate(tf);												//复制到RAM							
			if (tf) {																														
				do {																							//每个令牌只有一个参数
					if (tr->key == 0 || tr->key == tf->key) {
						memcpy(tr, tf, sizeof(configToken_t));
						break;
					}
					tr++;
				} while (1);
			}
		} while (tf);
		ret = flashErase(flashStartAddr(), NUM_OF_LIST*sizeof(configRec_t)/sizeof(uint32_t));							
		FLASH_DataCacheCmd(DISABLE);													//使闪存数据缓存无效			
		FLASH_DataCacheReset();
		FLASH_DataCacheCmd(ENABLE);
		if (ret) {
			tr = (configToken_t *)recs;												
			while (tr->key)																			//将令牌复制回闪存
				configTokenStore(tr++);																		
			for (i = 0; i < NUM_OF_LIST; i++) {									//在内存中创建参数列表
				memcpy(recs[i].name, configParameterStrings[i], 16);				//复制令牌
				recs[i].val = parameter[i];												//参数传参
			}
			ret = flashAddress(flashStartAddr(), (uint32_t *)recs, NUM_OF_LIST*sizeof(configRec_t)/sizeof(uint32_t));
		}
		aqFree(recs, NUM_OF_LIST, sizeof(configRec_t));
	}
	else {
	}
	return ret;
}

void configInit(void) {
	float ver;	
	uint8_t tfRec;	
	configFlashRead();																															//从Flash中开始
  writeMotormessage();                                                            //将从flash里面读出来数据写到TF卡的数组里面
	
	tfRec = tFCardConfig();																													//加载tf卡的配置
	if(!tfRec && parameter[ROBOT_TYPE] > NO_ID){
		parameterReadDataFromTFCard(parameter[ROBOT_TYPE]);                           //从TF卡里读PID参数
    motorMessageReadDataFromTFCard(parameter[ROBOT_TYPE]);                        //从TF卡里面读电机配置参数
	  readMotormessage();
	}
	ver = *(float *)(flashStartAddr()+16);																					//读取当前flash版本
	if (isnan(ver))
		ver = 0.0f;
																																									//如果编译的默认值大于flash版本和加载版本
	if (DEFAULT_CONFIG_VERSION > ver && DEFAULT_CONFIG_VERSION > parameter[CONFIG_VERSION]){
		configLoadDefault();																													//加载默认值
		digitalHi(&supervisorData.flashSave);	
	}
	else if (ver >= parameter[CONFIG_VERSION]){  //如果flash版本大于当前或等于当前版本
	  configFlashRead();																														//读取flash		
    writeMotormessage();	
	}																			
	else if (parameter[CONFIG_VERSION] > ver){  //如果加载的版本大于flash版本
		readMotormessage();                                                          
		configFlashWrite();	
	}																																								//写入flash,这个情况只存在于有SD卡时，且SD卡中的版本高于flash中的版本才会发生
}

unsigned int configParameterRead(void *data) {
  paramStruct_t *par = (paramStruct_t *)data;

  if (par->paramId + par->num > NUM_OF_LIST)
		par->num = NUM_OF_LIST - par->paramId;

	memcpy((char *)par->values, (char *)&parameter[par->paramId], par->num * sizeof(float));

	return par->num * sizeof(float);
}

unsigned int configParameterWrite(void *data) {
	paramStruct_t *par = (paramStruct_t *)data;

	memcpy((char *)&parameter[par->paramId], (char *)par->values, par->num * sizeof(float));

	return configParameterRead(data);
}

int configParseParams(char *fileBuf, int size, int p1) {
	static char lineBuf[CONFIG_LINE_BUF_SIZE];
	char *param;
	float value;
	char c;
	int p2;
	int n;
	int i, j;
	
	param = (char *)aqCalloc(17, sizeof(char));
	if (param==NULL)
		return -1;

	p2 = 0;
	for (i = 0; i < size; i++) {
		c = fileBuf[p2++];
		if (c == '\n' || p1 == (CONFIG_LINE_BUF_SIZE-1)) {
			lineBuf[p1] = 0;

			n = sscanf(lineBuf, "#define DEFAULT_%17s %f", param, &value);
			if (n != 2) {
				n = sscanf(lineBuf, "%17s %f", param, &value);
				if (n != 2) {
					n = sscanf(lineBuf, "#define %17s %f", param, &value);
				}
			}
			if (n == 2) {
				for (j = 0; j < NUM_OF_LIST; j++) {
					if (!strncasecmp(param, configParameterStrings[j], sizeof(char)*17))
					parameter[j] = value;
				}
			}
			p1 = 0;
		}
		else {
			lineBuf[p1++] = c;
		}
	}
	if (param)
		aqFree(param, 17, sizeof(char));
	return p1;
}

int8_t configFormatParam(char *buf, int n){
	char *str;
	int8_t ret = 0;
	str = (char *)aqCalloc(16, sizeof(char));
  if (str == NULL)
		return ret;
	ftoa(str, parameter[n], 10);
	ret = sprintf(buf, "%-17s\t\t%s\n", configParameterStrings[n], str);
  if (str)
		aqFree(str, 16, sizeof(char));
  return ret;
}


void configSetParamByID(int id, float value) {
  parameter[id] = value;
}

int configGetParamIdByName(char *name) {
	int i = -1;
	for (i = 0; i < NUM_OF_LIST; i++)
		if (!strncmp(name, configParameterStrings[i], 16))
			break;
	return i;
}

void configGetParamFromID(int id,float *param,uint16_t loops){
	for(uint16_t i = 0;i < loops;i++)
		param[i] = parameter[id + i];	 
}

