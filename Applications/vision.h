#ifndef __VISION_H
#define __VISION_H

#include "bsp.h"
#include "Driver_Slave_Sensor.h"
#include "shoot.h"
#include "util.h"
#include "stdbool.h"
#include "auto_infantry.h"
#include "auto_tank.h"

#define VISION_PRIORITY	  	6
#define VISION_STACK_SIZE	  1024
#define VISION_PERIOD 4

#define VISION_SENSOR_HIST	5

#define ENABLE_AUTOMATIC_AIM_17MM  (((robotConfigData.typeOfRobot == INFANTRY_ID) || (robotConfigData.typeOfRobot == UAV_ID))  && (autoTaskData->currentTask == INFANTRY_AUTOMATIC_AIM))					
#define DISENABLE_AUTOMATIC_AIM_17MM  (((robotConfigData.typeOfRobot == INFANTRY_ID) || (robotConfigData.typeOfRobot == UAV_ID)) &&!(autoTaskData->currentTask == INFANTRY_AUTOMATIC_AIM))

#define ENABLE_AUTOMATIC_AIM_42MM  ((robotConfigData.typeOfRobot == TANK_ID)&&(autoTaskData->currentTask == TANK_AUTOMATIC_AIM))					
#define DISENABLE_AUTOMATIC_AIM_42MM  ((robotConfigData.typeOfRobot == TANK_ID)&&!(autoTaskData->currentTask == TANK_AUTOMATIC_AIM))

#define ROBOT_ID  judgeData.extGameRobotState.robot_id

#define MM_TO_M_CONVERSION(p) (float)(*p) / 1000.0f
#define M_TO_MM_CONVERSION(p)	(float)(*p) * 1000.0f

#define ANGLE_TO_RADIAN(p) *p * PI / 180.0f
#define RADIAN_TO_ANGLE(p) *p * 180.0f / PI

#define SMALL_MANUAL_SINGLE		26.0f
#define SMALL_MANUAL_CONTINUOUS 20.0f
#define SMALL_AUTO_CONTINUOUS 	16.0f
#define BIG_SHOOT				15.0f

#define SMALL_PZT_TO_BARREL_BIAS_X 0
#define SMALL_PZT_TO_BARREL_BIAS_Y 0
#define SMALL_PZT_TO_BARREL_BIAS_Z -139.48

#define BIG_PZT_TO_BARREL_BIAS_X 0
#define BIG_PZT_TO_BARREL_BIAS_Y -30.28
#define BIG_PZT_TO_BARREL_BIAS_Z -161.84

#define MANUAL_PREJUDG_SCALE 0.1f

#define VISION_STORED_LENGTH 50	

typedef enum{
	TX2_STOP = 0,							//停止工作		
	TX2_DISTINGUISH_ARMOR ,		//识别装甲板
	TX2_DISTINGUISH_BUFF 			//识别大小符	
} visionWorkMode_e;

typedef enum{
	SMALL_BULLET = 0,
	BIG_BULLET
} bulletType_e;

typedef enum{
	ENEMY_RED = 0,
	ENEMY_BLUE
} enemyType_e;

typedef enum{
	STOP = 0,
	CLOCKWISE,
	ANTICLOCKWISE,
	UNKNOW
} buffOfDirRota_e;
typedef struct{
	TaskHandle_t xHandleTask;	
	formatTrans16Struct_t coordinateBase[3];				//装甲板坐标信息，从tx2接收，发送到主控
	formatTrans16Struct_t coordinate[3];						//装甲板实际坐标信息，从tx2接收，发送到主控
	uint8_t distingushState;
	bool captureFlag;																//捕获指令，从tx2接收，发送到主控
	bool sameTargetFlag;														//同一目标指令，从tx2接收，发送到主控
	buffOfDirRota_e buffOfDirRota;
	formatTrans16Struct_t CNTR; 										//TX2回传的包序号，从tx2接收，发送到主控
	uint16_t lastCNTR;
	
	uint16_t CNTR_DataSet[VISION_STORED_LENGTH];		//历史储存值
	float pitchDataSet[VISION_STORED_LENGTH];
	float yawDataSet[VISION_STORED_LENGTH];
	uint8_t storedIndex;
	float pitchReal;
	float yawReal;
	
	shootMode_e fireMode;								//开火模式，从主控接收，发送到tx2
	visionWorkMode_e workMode;					//视觉工作模式，从主控接收，发送到tx2
	bulletType_e bullet;								//子弹类型，从主控接收，发送到tx2
	uint8_t enemyType;									//敌方颜色， 从主控接收，发送到tx2
	
	float shootSpeed;
	float shootDelayTime;
	float realDelayTime;
	float offsetGravity;
	float rateRamp[3];
	
	uint32_t sensorHistIndex;
	float sumPos[3];
	float posHist[3][VISION_SENSOR_HIST];
	float averPos[3];
	float targeRate[3];
	
	float lastCoordinate[3];
	float coordinatePredict[3];							//装甲板预测坐标信息
	float angleBias[2];									//与目标角度的差
	float pitchCmd;										//最终输出的目标角度
	float yawCmd;
	bool fireFbd;										//开火指令
	
	float manualPitchBias;
	float manualYawBias;
	bool miniPTZEnableAim;
	uint32_t visionErrorCount;							// 误判或丢失目标 次数
	double time[2];
	float intervalTime;
	uint32_t visionLastErrorCount;
	uint32_t intervalNum;
	uint16_t counter;
	bool initFlag;
	bool prejudgFlag;
	bool cailSuccess;
	
	float pitchCmdBuff;
	float yawCmdBuff;
	float lastYawCmdBuff;
	float lastYawCmd;
	float baseBiasX;
	float baseBiasBuffX;
	float baseBiasBuffY ;
	float predictBias;
	float lastPredictBias;
	float lastPitchCmd ;
	float predictShootLimit;
	
	float mortarPitCmd;
	float mortarYawCmd;
	float mortarPitCmdBuff;
	float mortarYawCmdBuff;
	
	uint8_t rBiasMode;
	float buffBias;
	
	uint32_t loops;
} visionStruct_t;

extern visionStruct_t visionData;

void visionStoreHistoricalData(float pitch, float yaw, uint16_t CNTR);
void visionReadDataUpdate(u8 *arraySlaveVision);
void visionSendDataUpdate(uint8_t workMode,uint8_t bullet);
void visionSendDataInit(void);
void identifyCamp(void);
void shootVisionUpdate(keyBoardState_e realKeyValue,visionWorkMode_e mode,uint8_t * shootFlag);
void visionInit(void);
void shootVisionInit(void);
void miniPTZAutomaticAimUpdate(uint8_t * schedule);
void visionFireFbdUpdata(uint8_t * shootFlag);
void visionMortar(void);//（迫击炮）

#endif


