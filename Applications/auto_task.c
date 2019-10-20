#include "type_robot.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "deforming.h"
#include "auto_task.h"
#include "auto_infantry.h"
#include "auto_tank.h"
#include "auto_auxiliary.h"
#include "auto_sentry.h"
#include "auto_uav.h"
#include "rc.h"
#include "judge.h"
#include "vision.h"
#include "config.h"

AutoTaskStruct_t *autoTaskData;

void gimbalSwitch(uint8_t active){																				//云台激活函数
	if(active){
		gimbalData.autoMode = ENABLE;
	}
	else{
		gimbalData.autoMode = DISABLE;
	}	
}

void chassisSwitch(uint8_t active){																				//底盘激活函数
	if(active){
		chassisData.autoMode = ENABLE;
	}
	else{
		chassisData.autoMode = DISABLE;
		chassisData.autoSpeedTarget.x = 0.0f;
		chassisData.autoSpeedTarget.y = 0.0f;
		chassisData.autoSpeedTarget.z = 0.0f;
	}	
}

void shootSwitch(uint8_t active){																					//发射机构激活函数
	if(active){
		shootData.autoMode = ENABLE;
	}
	else{
		shootData.autoMode = DISABLE;
	}	
}

void deformingSwitch(uint8_t active){																			//变形激活参数
	if(active){
		
	}
	else{
		
		if(robotConfigData.typeOfRobot == AUXILIARY_ID){
			resSetPress();

		}
		
		
		chassisData.speedLimit = 1.0f;
	}
}

bool keyboradScanForTask(void){ 
static u8 lastKeyState_CTRL = 0;
static u8 lastKeyState_G = 0;	
	if(robotConfigData.typeOfRobot != SENTRY_ID){
		if(RC_MODE != RCSW_TOP){																							//必须在键鼠模式或哨兵下才能启动状态机扫描
			return false;																												//返回假
		}
	}
	/* ---- 当前存在任务的情况下 ---- */
	if(autoTaskData->currentTask != 0 || autoTaskData->avoidTask){																																				
		if(TASK_PRESS_F){																											//如果摁下F键，则终止当前任务，并对当前任务内容进行清零
			digitalHi(&autoTaskData->breakSign);
			autoTaskData->avoidTask = DISABLE;
			autoTaskData->aviodFlag = false;
			digitalLo(&infantryAutoData.avoidSchedule);
		}
//		if(autoTaskData->currentTask == CTRL_TASK){															
//			if(!TASK_PRESS_CTRL)																							//如果CTRL没有持续摁下,则终止当前任务，并对当前任务内容进行清零
//				autoDataInit(autoTaskData);
//		}
		if(autoTaskData->currentTask == G_TASK){
			if(!TASK_PRESS_G){																									//如果G没有持续摁下,则终止当前任务，并对当前任务内容进行清零
				static uint16_t time;
				time ++ ;//开始计时
				if(time > 25){//50ms松开消抖
					time = 0;//下一次松开计时
					autoDataInit(autoTaskData);
				}
			}
		}
		if(autoTaskData->currentTask == Z_TASK){
			if(!TASK_PRESS_Z){																									//如果Z没有持续摁下,则终止当前任务，并对当前任务内容进行清零
				static uint16_t time;
				time ++ ;//开始计时
				if(time > 25){//50ms松开消抖
					time = 0;//下一次松开计时
					autoDataInit(autoTaskData);
				}
			}
		}
		if(autoTaskData->currentTask == V_TASK){
			if(!TASK_PRESS_V){																									//如果V没有持续摁下,则终止当前任务，并对当前任务内容进行清零
				static uint16_t time;
				time ++ ;//开始计时
				if(time > 25){//50ms松开消抖
					time = 0;//下一次松开计时
					autoDataInit(autoTaskData);
				}
			}
		}
	}
	if(autoTaskData->currentTask != CTRL_TASK){
		/* --- 当前非CTRL任务下 --- */
		if(TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){							  //只摁下X的情况下
			autoTaskData->currentTask = X_TASK;
		}
		else if(!TASK_PRESS_X && TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){						//只摁下R的情况下
			autoTaskData->currentTask = R_TASK;
		}	
		else if(!TASK_PRESS_X && !TASK_PRESS_R && TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){						//只摁下CTRL的情况下
			autoTaskData->currentTask = CTRL_TASK;
		}
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && TASK_PRESS_Z){						//只摁下Z的情况下
			autoTaskData->currentTask = Z_TASK;
		}
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){						//只摁下V的情况下
			autoTaskData->currentTask = V_TASK;
		}
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && TASK_PRESS_Q && !TASK_PRESS_Z){						//只摁下Q的情况下
			autoTaskData->currentTask = Q_TASK;
		}
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){						//只摁下G的情况下
			autoTaskData->currentTask = G_TASK;
		}
	}
	if(TASK_PRESS_E){																																												//按下E扭腰
		autoTaskData->avoidTask = ENABLE;
		autoTaskData->aviodFlag = true;
		autoTaskData->aviodDuration = 0.0f;
	}
	//按下直接满速
	if(TASK_PRESS_CTRL && TASK_PRESS_G){    
		if(!lastKeyState_CTRL || !lastKeyState_G){
			if(!autoTaskData->fastSeed)
				autoTaskData->fastSeed = 1;
			else
				autoTaskData->fastSeed = 0;
		}
	}
	lastKeyState_CTRL = TASK_PRESS_CTRL;	
	lastKeyState_G    = TASK_PRESS_G;
	
	return true;
}

void autoTaskUpdate(void){																								//自动任务更新
	if(keyboradScanForTask()){																							//如果不再哨兵模式下，SW2又不在键鼠档位，则无法执行任何自动任务
		switch(robotConfigData.typeOfRobot){
			case INFANTRY_ID:	
				infantryAutoTaskUpdate(); 
				break;																	//步兵任务
			case TANK_ID : 
					tankAutoTaskUpdate(); 
				break;																	//英雄任务
			case AUXILIARY_ID: 
				auxiliaryAutoTaskUpdate(); 
				break;																	//工程车任务
			case SENTRY_ID: 
				sentryAutoTaskUpdate(); 
				break;																	//哨兵任务
			case UAV_ID: 
				uavAutoTaskUpdate(); 
				break;																	//无人机任务
		}
	}
	else{
		autoDataInit(autoTaskData);																						//终止当前任务并对当前任务内容进行清零
	}
}

void autoDataInit(AutoTaskStruct_t *autoContrlData){											//对自动任务的结构体内容进行清零
	digitalClan(&autoContrlData->currentTask);
	digitalClan(&autoContrlData->breakSign);
	digitalClan(&autoContrlData->currentTask);
	digitalClan(&autoContrlData->schedule);
	digitalClan(&autoContrlData->taskDuration);
	digitalClan(&autoContrlData->taskState);
	digitalClan(&autoContrlData->autoDeviceData.gimbalAutoSate);
	digitalClan(&autoContrlData->autoDeviceData.chassisAutoState);
	digitalClan(&autoContrlData->autoDeviceData.shootAutoState);
	digitalClan(&autoContrlData->autoDeviceData.deformingAutoState);
	gimbalSwitch(DISABLE);
	chassisSwitch(DISABLE);
	shootSwitch(DISABLE);
	deformingSwitch(DISABLE);
	visionSendDataUpdate(TX2_STOP,visionData.bullet);
}

void autoTaskInit(void){
	switch(robotConfigData.typeOfRobot){																		//通过tf卡写机器人种类
		case INFANTRY_ID:	
			autoTaskData = &infantryAutoData;
			break;																															//步兵任务
		case TANK_ID : 
			autoTaskData = &tankAutoData;
			break;																															//英雄任务
		case AUXILIARY_ID: 
			autoTaskData = &auxiliaryAutoData;
			break;																															//工程车任务
		case SENTRY_ID: 
			autoTaskData = &sentryAutoData;
			break;																															//哨兵任务
		case UAV_ID: 
			autoTaskData = &uavAutoData; 
			break;																															//无人机任务
	}
	autoDataInit(autoTaskData);																							//初始化所有任务数据
}
