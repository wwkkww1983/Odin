#ifndef __CONTROL_H
#define __CONTROL_H

#include "Util.h"
#include "pid.h"
#include "chassis.h"
#include "deforming.h"
#include "gimbal.h"
#include "shoot.h"
#include "supply.h"
#include "cansend.h" 

#define SHIELD_JUDGE

#define CONTROL_PRIORITY 7
#define CONTROL_PERIOD   2
#define CONTROL_STACK_SIZE 512
#define CONTROL_MIN_YAW_OVERRIDE	1000       		//在舵向摇杆摇动后保持最后期望1s

typedef void DeviceActivation_t(void);

typedef enum {
	MODE_INIT  = 0,		 //初始化模式
	MODE_KM,    			 //键鼠控制
	MODE_RELAX,	  		 //解除控制权
	MODE_RC,	  			 //摇杆控制
	MODE_STOP,    		 //丢控停止模式
}robotModeStruct_t;

typedef struct {
	TaskHandle_t xHandleTask;
	uint8_t dataInitFlag;
	uint32_t loops;
} controlStruct_t;
extern controlStruct_t controlData;
extern robotModeStruct_t robotMode;
extern robotModeStruct_t lastRobotMode;
extern BaseType_t ControlEvent;
extern TaskHandle_t xHandleTaskControl;
extern float controlGYRO[3];
extern float controlRollTargetRateTest,controlPitchTargetRateTest,controlYawTargetRateTest;
void controlDeviceConfirm(uint16_t deviceFlag,DeviceActivation_t *deviceFunction);
void controlUpdateTask(void *Parameters);
void controlInit(void);

#endif


