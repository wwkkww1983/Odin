#ifndef __RC_H
#define __RC_H
#include "Driver_RMMotor.h"
#include "Driver_RMDT7.h"
#include "FreeRTOS_board.h"
#include "type_robot.h"
#include "imu.h"
#define MODE_RC_CTRL (1<<0)
#define MODE_KM_CTRL (1<<1)

//以下宏定义为SBUS协议用
#if UAV_SBUS 
#define RC_ROLL						remoteControlData.sbusValue.rcRealData.CH0
#define RC_PITCH					remoteControlData.sbusValue.rcRealData.CH1	
#define RC_THROT 					remoteControlData.sbusValue.rcRealData.CH2
#define RC_RUDD	 					remoteControlData.sbusValue.rcRealData.CH3
#define RC_GEAR						remoteControlData.sbusValue.rcRealData.CH4
#define RC_MODE 					remoteControlData.sbusValue.rcRealData.CH5
#define RC_AUX1						remoteControlData.sbusValue.rcRealData.CH6
#define RC_AUX2						remoteControlData.sbusValue.rcRealData.CH7
#define RC_HOME						remoteControlData.sbusValue.rcRealData.CH8
#else
#define RC_TRANSVERSE			remoteControlData.dt7Value.rcRealData.CH0					//横向运动
#define RC_LONGITUDINAL		remoteControlData.dt7Value.rcRealData.CH1					//纵向运动
#define RC_RUDD 					remoteControlData.dt7Value.rcRealData.CH2					//YAW轴运动
#define RC_PITCH 					remoteControlData.dt7Value.rcRealData.CH3					//PITCH轴运动
#define RC_ROTATE					remoteControlData.dt7Value.rcRealData.CH4         //小陀螺运动
#define RC_GEAR						remoteControlData.dt7Value.rcRealData.S1
#define RC_MODE 					remoteControlData.dt7Value.rcRealData.S2
#endif

#if UAV_SBUS 
#define RC_RELAX  ((RC_ROLL >-10 &&  RC_ROLL < 10)&&(RC_PITCH >-10 &&  RC_PITCH < 10)&&(RC_THROT >-10 &&  RC_THROT < 10) &&(RC_RUDD >-10 &&  RC_RUDD < 10) \
                  &&(RC_GEAR >-10 &&  RC_GEAR < 10)&&(RC_MODE >-10 &&  RC_MODE < 10)&&(RC_AUX1 >-10 &&  RC_AUX1 < 10) &&(RC_AUX2 >-10 &&  RC_AUX2 < 10) \
									&&(RC_HOME >-10 &&  RC_HOME < 10))
#else
#define RC_RELAX	((RC_TRANSVERSE >-10 &&  RC_TRANSVERSE < 10)&&(RC_LONGITUDINAL >-10 &&  RC_LONGITUDINAL < 10)&&(RC_RUDD >-10 &&  RC_RUDD < 10) \
                  &&(RC_PITCH >-10 &&  RC_PITCH < 10)&&(RC_ROTATE >-10 && RC_ROTATE < 10)&&(RC_GEAR == 2)&&(RC_MODE == 2))
                    
#endif

typedef struct {
	TaskHandle_t xHandleTask;
	dt7RcSturct_t dt7Value;	
	sbusRcStruct_t sbusValue;
	coordinateFloat_t chassisSpeedTarget; 
  float pitchGyroTarget;
  float yawGyroTarget;
	bool initFlag;
	uint32_t loops;
	uint8_t rcIspReady;
	errorScanStruct_t rcError;
} remoteControlStruct_t;

void rcUpdateTask(void);
void rcInit(void);
void getGimbalCtrlDate(void);

extern remoteControlStruct_t remoteControlData;
extern BaseType_t RCEvent;
extern TaskHandle_t xHandleTaskRc;
#endif
