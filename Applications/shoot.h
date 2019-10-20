#ifndef __SHOOT_H
#define __SHOOT_H
#include "BSP.h"
#include "Util.h"
#include "pid.h"
#include "Driver_RMMotor.h"
#include "control.h"

/* 发射机构选择，同时只能定义一个 */
#define USE_PNEUMATIC								//使用气动
//#define USE_FRICWHEEL								//使用摩擦轮

#define FricMotor_L TIM9->CCR1
#define FricMotor_R TIM9->CCR2

#define BULLET_MONITOR_FLAG					shootData.bulletMonitorFlag

#define LASER_ON										PBout(5)=1//暂无
#define LASER_OFF										PBout(5)=0//暂无

#define SHOOT_HEAT(shootSpeed)			(float)shootSpeed
#define FIRE_FLAG 			         		shootData.fireFlag_17mm	//开火信号 
#define AIRDROP_FLAG								shootData.airdropFlag		//收到补给信号			请注意给这两个赋值.
	
#define SHOOT_TIMES_CONTINUOUS			 3											//连续发射情况下发射子弹数目
#define SMALL_BULLET_HARM						 10											//小子弹伤害
#define BIG_BULLET_HARM							 100										//大子弹伤害
#define BULLET_SPEED_DEFAULT    		 26.0f			    				//小子弹默认射速

#define MIN_SPEED                    0
#define MAX_SPEED                    5000
#define SNAIL_MOTOR                  DISABLE

#define protectHeartTank             100										//英雄保护热量

typedef enum{
	MANUAL_SINGLE = 0,
	MANUAL_CONTINUOUS,
	AUTO_CONTINUOUS,
}shootMode_e;      						//射击模式

typedef enum{
	SAFE = 0,
//	WARNING,
	DANGEROUS,
}shootStatus_e;								//射击状态

typedef enum{
	MOTOR_REVERSAL = -1,
	MOTOR_STOP = 0,
	MOTOR_FOWARD = 1,
}motorStatus_e;								//电机状态

typedef struct{
	TickType_t xLastWakeTime;	  //时间监测
	shootMode_e shootMode;		  //射击模式
	shootStatus_e shootStatus;  //射击状态
	shootStatus_e shootStatusMode;//射击安全模式
  TaskHandle_t xHandleTaskshoot;	
  pidStruct_t *speedPID;
	pidStruct_t *testSpeedPID;
	pidStruct_t *fricWheelSpeedPID[2];
	pidStruct_t *turntablePID;
	pidStruct_t *loadedSpeedPID;
	errorScanStruct_t fricWheelError[2];
	errorScanStruct_t lidError;
	errorScanStruct_t pokeError;
	bool  clearBulletFlag;      //清弹标志位
	float smallPokeSpeedRef;  	//拨弹盘速度期望值
  float smallPokeSpeedOut; 	  //拨弹盘PID输出值
	float bigPokeSpeedRef;  		//拨弹盘速度期望值
  float bigPokeSpeedOut; 	  	//拨弹盘PID输出值
	uint8_t autoMode;
	uint8_t airdropFlag;			  //接收到子弹标志
	float shootSpeedLast;				//上次子弹射速
	uint16_t fricSpeedSet_42mm; //42mm摩擦轮转速设定
	uint16_t fricSpeedSet_17mm;	//17mm摩擦轮转速设定
	float fricWheelSpeedRef[2];	//英雄车子弹射速期望值
	float	fricWheelSpeedOut[2];	//英雄车子弹射速PID输出值
	uint8_t bulletMonitor[3]; 	//前3次光电传感器状态
	float OutputGain;						//根据大小符是否激活来判断输出增益
	uint16_t bulletRemain;		  //剩余子弹数
	uint16_t shootManualNumber; //剩余手动开火次数
	uint8_t fireDataInit;				//射击命令flag
	uint8_t shootTrigger; 			//射击开启指令
	float shootHPRef;   		  	//射击预计伤害
	float shooterJudgeLastHeat_17mm;
	float shooterJudgeHeat_17mm;
	float shooterHeat;			  	//枪口热量
	float shooterHeatCoolingPs;	//枪口热量每0.1秒冷却值
	float shooterHeatMax;	  		//枪口热量最大值
	float shooterHeatRemain;  	//枪口热量可用值
	TickType_t shootWaitTime;
	uint8_t monitorFlag;
	uint8_t fricMotorFlag;		  //摩擦轮开启标志
	uint8_t fireFlag_42mm;		  //42mm子弹开火标志
	uint8_t fireFlag_17mm;		  //17mm子弹开火标志
	uint8_t leval;						  //机器人等级
	
	shootMode_e shootModeTank;		  //射击模式
	shootStatus_e shootStatusTank;  //射击状态
	shootStatus_e shootStatusModeTank;//射击安全模式
	
	uint8_t loadStep;									//装填步骤
	bool ballisticClean;							//弹道清理标志
	bool ballisticFill;								//弹道填充标志
	bool bulletRelax;									//装填标志
	bool bulletReady;									//上膛标志
	bool bulletExist;									//上膛标志
  int8_t turntableStall;						//滚筒堵转标志
	uint8_t suicideFireFlag;					//自杀开火标志
	
  int8_t pokeStall;                //拨弹堵转位
	uint8_t pokeFlag;             //拨叉标志
	uint8_t shootTriggerTank;   
  uint8_t shootOutNumber;       //已发射子弹数
  uint8_t shootFillNumber;      //填充子弹数
	uint16_t shootTimesTank;      //预打子弹数
	uint16_t shooterTimesOutTank; //已发射子弹数
	uint16_t shooterTimesLimitTank; //可发射子弹数
	float shooterHeatCoolingPsTank;
	float turntableOut;
	float turntableRef;
	float turnSpeed;                //滚筒转速
	float lastShooterHeat1;  
	float shootSpeedLastTank;				//上次子弹射速	
	float shooterHeatTank;			  	//枪口热量
	float shooterHeatMaxTank;	  		//枪口热量最大值
	float shooterHeatRemainTank;  	//枪口热量可用值
	uint8_t bulletMonitorFlag;
	float intervalTime;
	double time[2];
	uint32_t loops;
}shootStruct_t;

void shootDataReset(void);
void shooterHeatIncreasing(void);
void shooterHeatCooling(void);
void shooterHeatAbjust(void);
void pokeMoterReadData( CanRxMsg *can_rx_msg, motorCanDataRecv_t *pokeMoterData );
void shootUpdate(void);
void shootInit(void);
void shooterHeatIncreasingTank(void);
void TankShootHeatAbjust(void);	
void smallPokeSpeedRefChange(void);
void shootProtectTank(void);

extern shootStruct_t shootData;

#endif
