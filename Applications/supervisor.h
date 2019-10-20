#ifndef __SUPERVISOR_H
#define __SUPERVISOR_H

#include "stm32f4xx.h"
#include "NavAndData.h"
#include "stdbool.h"	

#define SPUER_PRIORITY   10
#define SPUER_STACK_SIZE 512
#define SUPER_STACK_PERIOD 100

#define SPUER_CHECK_FREQ	10
#define WAITLOOPS 20

#define CALI_RC_VALUE	490

enum {
	CALI_ONLY_GYO = 0x01,
	CALI_GYO_ACC = 0x03
};

enum SupervisorConfig{
	IMU = 0,
	MAG,
	FLASH_OPERATION,
	YAW_CALI,
	DEFINE_ROBOT,
	DEFINE_ID,
	GIMBAL_CALI,
	CONFIG_LIST
};

enum {
	TASK_REGULAR = 1,
	TASK_FAULT,
	TASK_SUSPEND
};

enum {
	VISION_TASK = 0,
	CONTROL_TASK,
	WIRELESS_TASK,
	SUPERVISOR_TASK,
	TYPE_ROBOT_TASK,
	TYPE_LOCAL_ID_TASK,
	PARAMETER_TASK,
	LIST_OF_TASK
};

typedef struct {
	TaskHandle_t xHandleTask;
	float soc;
	float flightTime;		   			 		// seconds
	float flightSecondsAvg;	    		// avg flight time seconds for every percentage of SOC
	float flightTimeRemaining;	    // seconds
	uint32_t armTime;
	uint32_t lastGoodRadioMicros;
	float vInLPF;
	bool busyState;
	uint16_t state;
	uint8_t diskWait;
	uint8_t configRead;
	uint8_t beepState;
	uint8_t ledState;
	uint8_t ArmSwitch;
	uint8_t flashSave;
	uint8_t tfState;
	float tempStd;
	uint8_t imuCali;
	uint8_t gimbalCaliReset;
	BaseType_t taskEvent[LIST_OF_TASK];
	uint8_t taskState[LIST_OF_TASK];
	uint32_t loops;
} supervisorStruct_t;

enum supervisorStates {
	STATE_INITIALIZING	= 0x0000,														//初始化状态
	STATE_MAGCALI	= 0x0001,																	//磁力计校准状态
	STATE_DISARMED	= 0x0002,																//上锁
	STATE_ARMED		= 0x0004,																	//解锁
	STATE_IMUCALI	= 0x0008,																	//IMU校准
	STATE_RADIO_LOSS	= 0x0010,															//丢失
	STATE_SENSOR_ERROR	= 0x0020,														//传感器错误
	STATE_LOW_BATTERY	= 0x0040,															//电池
	STATE_JUDGE_ERROR	= 0x0080,															//裁判系统错误
	STATE_VISION_ERROR = 0x0100,														//视觉端错误
	STATE_CURRENT_ERROR = 0x0200,														//功率板错误
	STATE_MOTOR_ERROR	= 0x0400,															//电机错误
};

void supervisorTaskCheck(void);
void supervisorStateSwitch(uint16_t state,uint8_t valve); 
void supervisorImuCali(uint8_t accTare);
void supervisorGimbalCali(void);
void supervisorInit(void);
extern supervisorStruct_t supervisorData __attribute__((section(".ccm")));
	
#endif


