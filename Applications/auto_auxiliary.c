#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "vision.h"
#include "auto_auxiliary.h"
#include "DRIVER_VL53L0X.h"
#include "rc.h"
#include "keyboard.h"
#include "auto_tank.h"
#include "deforming.h"
#include "config.h"

AutoTaskStruct_t auxiliaryAutoData;
void auxiliaryTaskBegin(void){
	auxiliaryAutoData.taskState = EXECUTIONG;								//标记为进行中
	digitalIncreasing(&auxiliaryAutoData.schedule);					//给执行序列加一
}

/****************************************************/
/********************救援任务************************/
/****************************************************/
void auxiliaryRescueUpdate(void){													//救援任务更新
	static u8 lastMouseL;
	if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;											//如果有打断则结束任务
	}
	
	switch(auxiliaryAutoData.schedule){
		case 1:{
			chassisData.speedLimit = 0.25f;											//限速，更好对位
			shiftAngle = 25.0f;
			RESCUE_LOOSEN;
			if(remoteControlData.dt7Value.mouse.Press_L)				//只需点一次鼠标左键，就可以放下扣子救援
				digitalIncreasing(&auxiliaryAutoData.schedule);	
		}break;
		case 2:{
			chassisData.speedLimit = 1.0f;											//钩住后取消限速
			RESCUE_TIGHT;
			if(!lastMouseL && remoteControlData.dt7Value.mouse.Press_L)
				shiftAngle = 0.0f;
			if(remoteControlData.dt7Value.mouse.Press_R)				//按下鼠标右键，返回第一步骤，关闭气缸
				auxiliaryAutoData.schedule = 1;
		}break;
		
		case 99:{
		  auxiliaryAutoData.taskState = END_OF_EXECUTION;
			grabZeroState();
			shiftAngle = 0.0f;
			chassisData.speedLimit = 1.0f;
			break;
		}
		default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
	}
	lastMouseL = remoteControlData.dt7Value.mouse.Press_L;
	auxiliaryAutoData.taskDuration += AUTO_TASK_DURATION;
}

/****************************************************/
/********************登岛任务************************/
/****************************************************/
void auxiliaryLandingUpdate(void){													//登岛任务更新
	
}

/****************************************************/
/********************下岛任务************************/
/****************************************************/
void  auxiliaryUnderIslandUpdate(void){

}
/****************************************************/
/******************子弹交接任务***********************/
/***************************************************/
void auxiliaryBulletTransferUpdate(void){									//子弹交接任务更新
	if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;											//如果有打断则结束任务
	}
	shiftAngle = 150.0f;																		//便于察看对位
	UP_OPEN;																								//车身抬起
	switch (auxiliaryAutoData.schedule){
		case 1:{
			chassisData.speedLimit = 0.4f;											//限速，更好对位
			SMALLMAGAZINE_CLOSE;																//关小弹舱
			MAGAZINE_CLOSE;																			//关大弹舱
			if(remoteControlData.dt7Value.mouse.Press_L)				//只需点一次鼠标左键，就可以开弹仓
				digitalIncreasing(&auxiliaryAutoData.schedule);
		}break;
		case 2:{
			SMALLMAGAZINE_OPEN;																	//开小弹舱
			MAGAZINE_OPEN;																			//开大弹舱
			if(remoteControlData.dt7Value.mouse.Press_R)				//按下鼠标右键，返回第一步骤，关闭弹仓
				auxiliaryAutoData.schedule = 1;
		}break;
		case 99:{
		  auxiliaryAutoData.taskState = END_OF_EXECUTION;
			shiftAngle = 0.0f;
			grabZeroState();
			chassisData.speedLimit = 1.0f;
			break;
		}
	default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
	}
	auxiliaryAutoData.taskDuration += AUTO_TASK_DURATION;
}

/****************************************************/
/****************摇摆躲避子弹任务*********************/
/***************************************************/
void auxiliaryAviodUpdate(void){														//摇摆躲避子弹任务更新
	static uint8_t avoidTurn = 0; 
	if(auxiliaryAutoData.avoidTask){
		if(avoidTurn){
			if(chassisData.chaseRef < AVOID_RANGE)
				chassisData.chaseRef += AVOID_RATE;
			else if(chassisData.chaseRef > AVOID_RANGE){
				chassisData.chaseRef -= AVOID_RATE;
				avoidTurn = 0;
			}
		}
		else{
			if(chassisData.chaseRef > -AVOID_RANGE)
				chassisData.chaseRef -= AVOID_RATE;
			else if(chassisData.chaseRef < -AVOID_RANGE){
				chassisData.chaseRef += AVOID_RATE;
				avoidTurn = 1;
			}
		}
		if(auxiliaryAutoData.aviodDuration > R_TIME){					//如果计时大于6.0s的时间，则将当前任务结束
			chassisData.chaseRef = 0.0f;
			auxiliaryAutoData.avoidTask = DISABLE;
		}
		auxiliaryAutoData.aviodDuration += AUTO_TASK_DURATION;	
	}
	else{
		chassisData.chaseRef = 0.0f;
	}
}

/****************************************************/
/********************抓取任务*************************/
/***************************************************/
void auxiliaryGrabUpdate(void){															//抓取任务更新
}

/****************************************************/
/********************自动任务*************************/
/***************************************************/
void auxiliaryAutoTaskUpdate(void){
	auxiliaryAviodUpdate();
	if(auxiliaryAutoData.currentTask != AUXILIARY_MANUAL){				//必须有可执行任务
		if(auxiliaryAutoData.taskState == UNEXECUTED){							//如果任务刚刚开始执行
			auxiliaryTaskBegin();																			//执行开始序列
		}
		else if(auxiliaryAutoData.taskState == EXECUTIONG){					//如果在执行中
			switch(auxiliaryAutoData.currentTask){
				case AUXILIARY_RESCUE: {
					auxiliaryRescueUpdate();															//Z 救援任务
					break;
				}
				case AUXILIARY_BULLET_TRANSFER: {
					auxiliaryBulletTransferUpdate();											//V 子弹交接任务
					break;
				}
				case AUXILIARY_GRAB: {
					auxiliaryGrabUpdate();																//X 抓取任务
					break;
				}
				default: {																							//如果是其他命令，则直接重新初始化结构体
					autoDataInit(&auxiliaryAutoData); 
					break;
				}
			}
		}
		else{
			autoDataInit(&auxiliaryAutoData);														//如果执行完毕或执行错误，则重新
			
		}
	}
}
