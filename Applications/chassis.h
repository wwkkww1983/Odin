#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "Util.h"
#include "pid.h"
#include "adrc.h"
#include "imu.h"
#include "clockcount.h"
#include "Driver_Power.h"
#include "power.h"

#define DIAMETER_OF_MOTOR  0.152f	//麦轮直径
#define GEAR_RATIO	13.67f		//电机减速比，如果换减速箱请更改此数，优化后可能需要通过上位机来确定此值
#define ELETRO_MECHANICAL_EFFICIENCY 0.98f	//机电转化效率，一般情况下电机不可能达到最大速率，这个值需要负载后跑动测得，其决定电机满速时底盘的速度，需要更具减速箱来更改
#define REAL_MOTOR_SPEED_SCALE (PI * DIAMETER_OF_MOTOR / (60.0f * GEAR_RATIO)) * ELETRO_MECHANICAL_EFFICIENCY
#define MIN_CHASSIS_SPEED  0.2f	//键盘控制的最小速度m/s

#define POWER_LIMIT   								 powerData.powerLimit              		//80.0f
#define WARNING_POWER 								 powerData.warningPower 					 		//40.0f
#define WARNING_POWER_BUFF 						 powerData.WarningPowerBuff 			 		//50.0f
#define NO_JUDGE_TOTAL_CURRENT_LIMIT   powerData.noJudgeTotalCurrentLimit 	//64000.0f
#define JUDGE_TOTAL_CURRENT_LIMIT      powerData.judgeTotalCurrentLimit 		//38000.0f 更改这个可以改变启动功率
#define ADD_POWER_CURRENT              powerData.addPowerCurrent            //18000.0f

enum {
	R_F_WHEEL = 0,
	L_F_WHEEL,
	L_B_WHEEL,
	R_B_WHEEL,
	NUMBER_OF_WHEEL
};

typedef enum {
	CHASSIS_RELAX = 0,
	CHASSIS_INIT,
	CHASSIS_STOP,
	CHASSIS_SEPARATE_GIMBAL,
	CHASSIS_FOLLOW_GIMBAL,
	CHASSIS_AVOID_MODE,
} chassisMode_e;

enum floorType{
	LEVEL_LAND = 0,
	SLOPE_LAND
};

typedef struct {	
	coordinateFloat_t manualSpeedTarget; 
	coordinateFloat_t	autoSpeedTarget;
	uint8_t autoMode;
	uint8_t changeHeadOrder;
	uint8_t changeHeadSchedule;
	uint8_t changeChassisSchedule;
	chassisMode_e ctrlMode;   
	chassisMode_e lastCtrlMode;
	int8_t  direction;
	float landingSpeedx;
	float landingSpeedy;
	float landingSpeedz;
	float speedLimit;
	float powerCurrent[4];
	float current[4];	
	float chaseRef;
	float chaseFbd;
	float chaseAngleOut;
	float chaseSpeedRef;
	float chaseSpeedFbd;
	float autoSpeedMote;
	float speedFbdMax;
	float speedLimitFloor;
	float DecelerateRatio;
	uint8_t floorType;
	float yawCenterSave;
	coordinateFloat_t posRef;
	coordinateFloat_t posFbd;
	float speedFbd[4];
	float speedRef[4];
	float drivenRef[2];
	float drivenFbd[2];
	float drivenOut[2];
	float scale[4];
	float averageScale;
	errorScanStruct_t wheelError[4];
	errorScanStruct_t currentError;
	pidStruct_t *chasePID;
	pidStruct_t *chaseSpeedPID;
	pidStruct_t *posPID;
	pidStruct_t *speedPID[4];
	pidStruct_t *currentPID[4];
	pidStruct_t *drivenWheelPID[2]; 
	adrcStruct_t *speedADRC[4];
	adrcStruct_t *drivenWheelADRC[2];
	
	float intervalTime;
	double time[2];
} chassisStruct_t;

extern chassisStruct_t chassisData;

void chassisUpdate(void);
static void chassisStopHandle(void);
static void chassisRelaxHandle(void);
static void followGimbalHandle(void);
static void separateGimbalHandle(void);
void avoidHandle(void);
static void powerLimitHandle(void);
void chassisInit(void);

#endif
