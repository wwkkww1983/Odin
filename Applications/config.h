#ifndef __CONFIG_H
#define __CONFIG_H

#include "stm32f4xx.h"
#include "parameter.h"

#define CONFIG_FILE_BUF_SIZE	  512
#define CONFIG_LINE_BUF_SIZE	  128

#define DEFAULT_CONFIG_VERSION	   129
#define DEFAULT_ROBOT_TYPE			   1
#define DEFAULT_DEAD_BAND     	   30.0f
#define DEFAULT_SHOOT_LOW_PWM		   1120
#define DEFAULT_SHOOT_HIGH_PWM	   1180
#define DEFAULT_MOUSE_FEEL			   0.0025f
#define DEFAULT_GIMBAL_CTR_SCALE   0.0012f
#define DEFAULT_GIMBAL_KB_SCALE		 0.013f
#define DEFAULT_CHASSIS_KB_SPEED   8000.0f
#define DEFAULT_CHASSIS_RC_SPEED   8000.0f
#define DEFAULT_CHASSIS_KB_ACC		 20.0f

#define DEFAULT_TILT_ANGLE_P		0.45f
#define DEFAULT_TILT_ANGLE_I		0.07f
#define DEFAULT_TILT_ANGLE_D		0.30f
#define DEFAULT_TILT_ANGLE_F  	0.25f
#define DEFAULT_TILT_ANGLE_PM		8.00f	
#define DEFAULT_TILT_ANGLE_IM		0.30f
#define DEFAULT_TILT_ANGLE_DM		8.00f
#define DEFAULT_TILT_ANGLE_OM		8.30f

#define DEFAULT_TILT_RATE_P			1900.00f
#define DEFAULT_TILT_RATE_I			1600.00f
#define DEFAULT_TILT_RATE_D			135.00f
#define DEFAULT_TILT_RATE_F			0.25f
#define DEFAULT_TILT_RATE_PM		5000.00f
#define DEFAULT_TILT_RATE_IM		2000.00f
#define DEFAULT_TILT_RATE_DM		3000.00f
#define DEFAULT_TILT_RATE_OM		5000.00f

#define DEFAULT_YAW_ANGLE_P			0.22f
#define DEFAULT_YAW_ANGLE_I			0.01f
#define DEFAULT_YAW_ANGLE_D			0.30f
#define DEFAULT_YAW_ANGLE_F			0.25f
#define DEFAULT_YAW_ANGLE_PM		8.00f
#define DEFAULT_YAW_ANGLE_IM		0.30f
#define DEFAULT_YAW_ANGLE_DM		8.00f
#define DEFAULT_YAW_ANGLE_OM		8.30f    

#define DEFAULT_YAW_RATE_P			610.00f
#define DEFAULT_YAW_RATE_I			500.00f
#define DEFAULT_YAW_RATE_D			125.00f
#define DEFAULT_YAW_RATE_F			0.25f
#define DEFAULT_YAW_RATE_PM			850.0f
#define DEFAULT_YAW_RATE_IM			500.0f
#define DEFAULT_YAW_RATE_DM			500.0f
#define DEFAULT_YAW_RATE_OM			850.0f

#define DEFAULT_CHASSIS_SPEED_P			32000.0f
#define DEFAULT_CHASSIS_SPEED_I			0.0f
#define DEFAULT_CHASSIS_SPEED_D			0.0f
#define DEFAULT_CHASSIS_SPEED_F			0.2f
#define DEFAULT_CHASSIS_SPEED_PM		16000.0f
#define DEFAULT_CHASSIS_SPEED_IM		2000.0f
#define DEFAULT_CHASSIS_SPEED_DM		8000.0f
#define DEFAULT_CHASSIS_SPEED_OM		16000.0f

#define DEFAULT_CHASSIS_POS_P  0.0f
#define DEFAULT_CHASSIS_POS_I  0.0f
#define DEFAULT_CHASSIS_POS_D  0.0f
#define DEFAULT_CHASSIS_POS_F  1.00f
#define DEFAULT_CHASSIS_POS_PM 0.0f
#define DEFAULT_CHASSIS_POS_IM 0.0f
#define DEFAULT_CHASSIS_POS_DM 0.0f
#define DEFAULT_CHASSIS_POS_OM 0.0f

#define DEFAULT_CHASSIS_CHASE_P     0.345f
#define DEFAULT_CHASSIS_CHASE_I     0.0f
#define	DEFAULT_CHASSIS_CHASE_D     0.0f
#define	DEFAULT_CHASSIS_CHASE_F     0.2f
#define	DEFAULT_CHASSIS_CHASE_PM    12.0f
#define	DEFAULT_CHASSIS_CHASE_IM    0.3f
#define	DEFAULT_CHASSIS_CHASE_DM    12.0f
#define	DEFAULT_CHASSIS_CHASE_OM    12.3f

#define DEFAULT_CHASSIS_RATE_P     600.0f
#define DEFAULT_CHASSIS_RATE_I     0.0f
#define	DEFAULT_CHASSIS_RATE_D     0.0f
#define	DEFAULT_CHASSIS_RATE_F     0.2f
#define	DEFAULT_CHASSIS_RATE_PM    8000.0f
#define	DEFAULT_CHASSIS_RATE_IM    2000.0f
#define	DEFAULT_CHASSIS_RATE_DM    5000.0f
#define	DEFAULT_CHASSIS_RATE_OM    8000.0f

#define DEFAULT_POWER_LIMIT_P		1.2f	
#define DEFAULT_POWER_LIMIT_I		1.0f
#define DEFAULT_POWER_LIMIT_D		0.0f
#define DEFAULT_POWER_LIMIT_F		0.2f
#define DEFAULT_POWER_LIMIT_PM	16000.0f
#define DEFAULT_POWER_LIMIT_IM	2000.0f
#define DEFAULT_POWER_LIMIT_DM	8000.0f
#define DEFAULT_POWER_LIMIT_OM	16000.0f

#define DEFAULT_PROPORTIONAL_SPEED_P		3.70f
#define DEFAULT_PROPORTIONAL_SPEED_I		0.0f
#define DEFAULT_PROPORTIONAL_SPEED_D		0.50f
#define DEFAULT_PROPORTIONAL_SPEED_F		1.00f
#define DEFAULT_PROPORTIONAL_SPEED_PM		1000.0f
#define DEFAULT_PROPORTIONAL_SPEED_IM		500.0f
#define DEFAULT_PROPORTIONAL_SPEED_DM		1000.0f
#define DEFAULT_PROPORTIONAL_SPEED_OM		1000.0f

#define DEFAULT_PROPORTIONAL_POS_P			3.75f
#define DEFAULT_PROPORTIONAL_POS_I			0.50f
#define DEFAULT_PROPORTIONAL_POS_D			0.0f
#define DEFAULT_PROPORTIONAL_POS_F			1.00f
#define DEFAULT_PROPORTIONAL_POS_PM			300.0f
#define DEFAULT_PROPORTIONAL_POS_IM			200.0f
#define DEFAULT_PROPORTIONAL_POS_DM			300.0f
#define DEFAULT_PROPORTIONAL_POS_OM			300.0f

#define	DEFAULT_HOLD_PILLAR_SPEED_P  0.0f
#define DEFAULT_HOLD_PILLAR_SPEED_I  0.0f
#define DEFAULT_HOLD_PILLAR_SPEED_D  0.0f
#define DEFAULT_HOLD_PILLAR_SPEED_F  1.00f
#define DEFAULT_HOLD_PILLAR_SPEED_PM 0.0f
#define DEFAULT_HOLD_PILLAR_SPEED_IM 0.0f
#define DEFAULT_HOLD_PILLAR_SPEED_DM 0.0f
#define DEFAULT_HOLD_PILLAR_SPEED_OM 0.0f
	
#define DEFAULT_DEFORM1_SPEED_P	 0.0f								
#define DEFAULT_DEFORM1_SPEED_I	 0.0f
#define DEFAULT_DEFORM1_SPEED_D	 0.0f
#define DEFAULT_DEFORM1_SPEED_F	 1.00f
#define DEFAULT_DEFORM1_SPEED_PM 0.0f
#define DEFAULT_DEFORM1_SPEED_IM 0.0f
#define DEFAULT_DEFORM1_SPEED_DM 0.0f
#define DEFAULT_DEFORM1_SPEED_OM 0.0f
	
#define DEFAULT_DEFORM2_SPEED_P  0.0f
#define DEFAULT_DEFORM2_SPEED_I  0.0f
#define DEFAULT_DEFORM2_SPEED_D  0.0f
#define DEFAULT_DEFORM2_SPEED_F  1.00f
#define DEFAULT_DEFORM2_SPEED_PM 0.0f
#define DEFAULT_DEFORM2_SPEED_IM 0.0f
#define DEFAULT_DEFORM2_SPEED_DM 0.0f
#define DEFAULT_DEFORM2_SPEED_OM 0.0f

#define DEFAULT_SHOOT_SPEED_P	 10.0f
#define DEFAULT_SHOOT_SPEED_I	 0.0f
#define DEFAULT_SHOOT_SPEED_D	 0.0f
#define DEFAULT_SHOOT_SPEED_F	 0.2f
#define DEFAULT_SHOOT_SPEED_PM 16000.0f
#define DEFAULT_SHOOT_SPEED_IM 8000.0f
#define DEFAULT_SHOOT_SPEED_DM 8000.0f
#define DEFAULT_SHOOT_SPEED_OM 16000.0f

#define DEFAULT_LOADED_SPEED_P  8.0f	
#define DEFAULT_LOADED_SPEED_I  0.0f
#define DEFAULT_LOADED_SPEED_D  0.0f
#define DEFAULT_LOADED_SPEED_F  0.2f
#define DEFAULT_LOADED_SPEED_PM 10000.0f
#define DEFAULT_LOADED_SPEED_IM 2000.0f
#define DEFAULT_LOADED_SPEED_DM 10000.0f
#define DEFAULT_LOADED_SPEED_OM 10000.0f

// 底盘速度ADRC参数
#define	DEFAULT_ADRC_R					10000000.0f
#define	DEFAULT_ADRC_H					0.002f
#define	DEFAULT_ADRC_N0					5.0f
#define	DEFAULT_ADRC_BETA01			500.0f
#define	DEFAULT_ADRC_BETA02			80000.0f
#define	DEFAULT_ADRC_BETA03			2000.0f
#define	DEFAULT_ADRC_B0					20.0f
#define	DEFAULT_ADRC_BETA0			0.8f
#define	DEFAULT_ADRC_BETA1			1.5f
#define	DEFAULT_ADRC_BETA2			0.0003f
#define	DEFAULT_ADRC_N1					200.0f
#define	DEFAULT_ADRC_C					5.0f
#define	DEFAULT_ADRC_ALPHA1			1.3f
#define	DEFAULT_ADRC_ALPHA2			1.2f
#define	DEFAULT_ADRC_ZETA				0.03f
#define	DEFAULT_ADRC_B					10.0f
#define	DEFAULT_ADRC_OMAX				10000.0f
/*----------------		以下参数存储到TF卡另一个文件	（motor文件）		----------------*/
#define DEFAULT_LOCAL_ID						0x0101
#define DEFAULT_WEAPON_TYPE					0
#define DEFAULT_PITCH_INSTALL				0
#define DEFAULT_YAW_INSTALL					0
#define DEFAULT_BACK_CENTER_TIME    1000.0f
#define DEFAULT_CHASSIS_CURRENT			3000.0f
#define DEFAULT_RC_RESOLUTION				500.0f
#define DEFAULT_YAW_CENTER 					0.0f
#define	DEFAULT_PITCH_CENTER				0.0f
#define DEFAULT_PITCH_MIN_RANGE			-30.f
#define DEFAULT_PITCH_MAX_RANGE			30.0f
#define	DEFAULT_YAW_TYPE            4
#define	DEFAULT_PITCH_TYPE          1  
#define	DEFAULT_YAW_FIX             2
#define	DEFAULT_YAW_TURN            1
#define	DEFAULT_PITCH_FIX           2 
#define	DEFAULT_PITCH_TURN          1
#define DEFAULT_IMU_ACC_BIAS_X      0.0f
#define DEFAULT_IMU_ACC_BIAS_Y      0.0f
#define DEFAULT_IMU_ACC_BIAS_Z      0.0f
#define DEFAULT_IMU_MAG_BIAS_X      0.0f
#define DEFAULT_IMU_MAG_BIAS_Y      0.0f
#define DEFAULT_IMU_MAG_BIAS_Z      0.0f
#define DEFAULT_IMU_GYO_BIAS_X      0.0f
#define DEFAULT_IMU_GYO_BIAS_Y      0.0f
#define DEFAULT_IMU_GYO_BIAS_Z      0.0f

typedef struct {
    uint32_t key;
    char data[24];
} configToken_t;

typedef struct {
    char name[16];
    float val;
} configRec_t;

typedef struct {
    unsigned int paramId;
    unsigned int num;
    float values[NUM_OF_LIST];
} __attribute__((packed)) paramStruct_t;

extern float parameter[NUM_OF_LIST];
extern const char *configParameterStrings[];
extern float motormessage[ABOVE_TOTAL];
extern const char *configMotorStrings[];

void configFlashRead(void);
uint8_t configFlashWrite(void);
void configLoadDefault(void);
unsigned int configParameterRead(void *data);
unsigned int configParameterWrite(void *data);
int8_t configReadFile(char *fname);
int8_t configWriteFile(char *fname);
void configSetParamByID(int id, float value);
int configGetParamIdByName(char *name);
int8_t configFormatParam(char *buf, int n);
void configGetParamFromID(int id,float *param,uint16_t loops);
int configParseParams(char *fileBuf, int size, int p1);
configToken_t *configTokenGet(uint32_t key);
void configTokenStore(configToken_t *token);
void configLoadDefault(void);
void configInit(void);

#endif





