#ifndef __GIMBAL_H
#define __GIMBAL_H
#include "BSP.h"
#include "Util.h"
#include "pid.h"
#include "clockcount.h"
#include "stdbool.h"

#define GIMBAL_TRACK 1
#define GIMBAL_POS 0

#define INSTALL_ENCODER	1
#define INSTALL_TURN	0

#define YAW_SPEED_SINGLE 0

typedef enum
{
  GIMBAL_RELAX = 0,
  GIMBAL_STOP,
  GIMBAL_INIT,
  GIMBAL_NORMAL,//F
  GIMBAL_TRACK_ARMOR,//B
  GIMBAL_SUPPLY_MODE,//R
  GIMBAL_SHOOT_BUFF,//V

} gimbalMode_e;

typedef struct
{
	gimbalMode_e ctrlMode;
	gimbalMode_e lastCtrlMode;
	
	float yawMotorAngle;//¬Î≈ÃΩ«∂»
	float pitchMotorAngle;

	float yawGyroAngle;//Õ”¬›“«Ω«∂»
	float pitchGyroAngle;

  float yawAngleStop;//¬Î≈ÃÕ£÷πΩ«∂»
	float pitchAngleStop;	
	
	float yawAngleStopSet;//¬Î≈ÃÕ£÷πΩ«∂»…Ë÷√
	float pitchAngleStopSet;	

	float yawAngleSave;//”√”⁄ƒ£ Ω«–ªª±£¥ÊΩ«∂»
	float pitchAngleSave;
	
	float yawSpeedRef;
	float pitchSpeedRef;
	float yawAngleRef;
	float pitchAngleRef;

	float yawSpeedFbd;
	float pitchSpeedFbd;
	float yawAngleFbd;
	float pitchAngleFbd;

	float yawSpeedOut;
	float pitchSpeedOut;
	float yawAngleOut;
	float pitchAngleOut;	

	float slaveGimbalPitchRef;
	uint8_t autoMode;
	uint8_t angleCycleStep;
	errorScanStruct_t gimbalError[2];
  pidStruct_t *yawSpeedPID;
	pidStruct_t *yawAnglePID;

	pidStruct_t *pitchSpeedPID;
	pidStruct_t *pitchAnglePID;

	volatile uint8_t initFinishFlag;
	uint8_t motorFlag;
	bool followLock;
	float intervalTime;
	double time[2];
}gimbalStruct_t;

extern gimbalStruct_t gimbalData;
extern float shiftAngle;
int8_t getInstallDirect(uint8_t installPara,bool type);
void gimbalUpdate(void);
void gimbalInit(void);
void gimbalRampInit(void);
static void gimbalInitHandle(void);
static void gimbalFollowHandle(void);
static void gimbalStopHandle(void);
static void gimbalRelaxHandle(void);
void gimbalStopSwitch(uint8_t active); 

#endif
