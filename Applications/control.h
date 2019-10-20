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
#define CONTROL_MIN_YAW_OVERRIDE	1000       		//�ڶ���ҡ��ҡ���󱣳��������1s

typedef void DeviceActivation_t(void);

typedef enum {
	MODE_INIT  = 0,		 //��ʼ��ģʽ
	MODE_KM,    			 //�������
	MODE_RELAX,	  		 //�������Ȩ
	MODE_RC,	  			 //ҡ�˿���
	MODE_STOP,    		 //����ֹͣģʽ
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


