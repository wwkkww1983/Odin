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
	if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;											//如果有打断则结束任务
	}
	
	switch (auxiliaryAutoData.schedule){
		case 1:{

		}break;
		case 99: {
			auxiliaryAutoData.taskState = END_OF_EXECUTION;	             //只有调到99时才能退出
			break;						
		}
		default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
	}
	auxiliaryAutoData.taskDuration += AUTO_TASK_DURATION;
}

/****************************************************/
/********************下岛任务************************/
/****************************************************/
void  auxiliaryUnderIslandUpdate(void){
	if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;											//如果有打断则结束任务
	}
	switch (auxiliaryAutoData.schedule){
		case 1:{

		}break;
		case 99: {
			auxiliaryAutoData.taskState = END_OF_EXECUTION;
		}
		default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
	}
	auxiliaryAutoData.taskDuration += AUTO_TASK_DURATION;
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
	static float speed = 500 * REAL_MOTOR_SPEED_SCALE;
	static u8 executeNum = 0,lastPressX = 0,deathMode = 0;
	if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;												//如果有打断则结束任务
	}		
	
	/* 按移动键 */
	if(PRESS_A)																	
		chassisData.autoSpeedTarget.x = -2000 * REAL_MOTOR_SPEED_SCALE;
	else if(PRESS_D)
		chassisData.autoSpeedTarget.x = 2000 * REAL_MOTOR_SPEED_SCALE;
	else
		chassisData.autoSpeedTarget.x = 0;
	
	if(PRESS_W)
		chassisData.autoSpeedTarget.y = -600 * REAL_MOTOR_SPEED_SCALE;
	else if(PRESS_S)
		chassisData.autoSpeedTarget.y = 600 * REAL_MOTOR_SPEED_SCALE;
	else
		chassisData.autoSpeedTarget.y = 0;
	
	/* 任务进程 */
	switch (auxiliaryAutoData.schedule){
		case 1:{
			chassisSwitch(ENABLE);
			shiftAngle = 45.0f;
			UP_OPEN;
			digitalIncreasing(&auxiliaryAutoData.schedule);			//车身抬高、接近前面跳下一步	
		}break;
		
    case 2:{																							//选择执行模式
			if(remoteControlData.dt7Value.mouse.Press_L){
				if(!PRESS_C)
					auxiliaryAutoData.schedule = 3;									//自动岛下前排
				else
					auxiliaryAutoData.schedule = 4;									//强制岛下前排
			}
			else if(remoteControlData.dt7Value.mouse.Press_R)
				if(!PRESS_C)
					auxiliaryAutoData.schedule = 12;								//自动岛下后排
				else
					auxiliaryAutoData.schedule = 13;								//强制岛下后排
			else if(!lastPressX && TASK_PRESS_X)
				auxiliaryAutoData.schedule = 14;									//岛上
			else if(PRESS_E){
				auxiliaryAutoData.schedule = 3;										//死亡模式
				deathMode = 1;
			}
		}break;
		
		/* 岛下前排抓弹 */	
		case 3:{
			chassisData.autoSpeedTarget.x = speed;							//横移对位
			if(/*(ONE_LEFT == 1) ||*/ (ONE_RIGHT == 1)){
				digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
				if(auxiliaryAutoData.countTimes > 5){							//持续检测到10ms
					auxiliaryAutoData.countTimes = 0;								//时间清零
					digitalIncreasing(&auxiliaryAutoData.schedule);	//跳下一步	
				}					
			}
			else
				digitalClan(&auxiliaryAutoData.countTimes);
		}break;	
		
		case 4:{
			PAWGO_BACK;																					//爪子后拉抓第一排
			digitalIncreasing(&auxiliaryAutoData.schedule);			//跳下一步
		}break;
		
		/* 第一次抓弹抓中间 */
		case 5:{
			chassisData.autoSpeedTarget.x = 0;								//车身停止
			PAW_OPEN;	     																		//爪子打开
      PAWGORIGHT_CLOSE;  																//爪子右移关	
	    PAWGOLEFT_CLOSE;    		                          //爪子左移关
			SMALLMAGAZINE_CLOSE;															//关闭弹舱
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
			if(auxiliaryAutoData.countTimes > 1){						
				auxiliaryAutoData.countTimes = 0;								//时间清零
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		/* 抓弹过程 */
		case 6:{
			PAWTURN_OPEN;			                              	//翻转下去	
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间			
			if(auxiliaryAutoData.countTimes > 500){
				auxiliaryAutoData.countTimes = 0;								//时间清零
				digitalIncreasing(&auxiliaryAutoData.schedule);
				executeNum++;																			//计数执行次数
			}
		}break;
		case 7:{
			PAW_CLOSE;																				//爪子夹紧
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
			if(auxiliaryAutoData.countTimes > 80){
				auxiliaryAutoData.countTimes = 0;								//时间清零
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 8:{
			PAWTURN_CLOSE;																		//爪子收回
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
//			if( (auxiliaryAutoData.countTimes > 200) && deathMode && (executeNum == 3) )
//				chassisData.autoSpeedTarget.y = speed;
			if(!PAWTURN_SENSOR || auxiliaryAutoData.countTimes > 700){
				auxiliaryAutoData.countTimes = 0;								//时间清零
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 9:{
			PAW_OPEN;																		      //爪子打开	
			EJECT_OPEN;																				//弹射打开
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
			if(auxiliaryAutoData.countTimes > 50){						
				auxiliaryAutoData.countTimes = 0;								//时间清零
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 10:{
			EJECT_CLOSE;																			//弹射关闭
			if(executeNum == 1){
				PAWGOLEFT_OPEN;																	//到最左
				auxiliaryAutoData.schedule = 6;
			}
			else if(executeNum == 2){
				PAWGORIGHT_OPEN;																//到最右
				PAWGOLEFT_CLOSE;
				auxiliaryAutoData.schedule = 6;
			}
			else if(executeNum == 3){
				if(deathMode){
					PAWGO_FORWARD;																//爪子往前
					TOP_RIGHT;																		//顶层右移
					auxiliaryAutoData.schedule = 6;								//继续抓后排最右侧
				}
				else{
					PAWGORIGHT_CLOSE;																//回中
					TOP_LEFT;																				//移半个弹药箱关，兼容抓第二排
					PAWGO_BACK;																			//爪子收回，兼容抓第二排
				}
			}
			else if(executeNum == 4){
				auxiliaryAutoData.schedule = 5;										//抓中间的
			}
			else if(executeNum == 5){
				PAWGOLEFT_OPEN;
				auxiliaryAutoData.schedule = 6;
			}
			else if(executeNum == 6){
				deathMode = 0;
				PAWGOLEFT_CLOSE;																	//回中
				SMALLMAGAZINE_OPEN;																//打开弹舱
				executeNum = 0;																		//清除抓弹计数
				auxiliaryAutoData.schedule = 2;										//调回模式选择处
			}
			digitalIncreasing(&auxiliaryAutoData.countTimes);		//开始累计时间
			if(auxiliaryAutoData.countTimes > 100){
				auxiliaryAutoData.countTimes = 0;									//时间清零
				executeNum = 0;																		//抓弹药箱数清0
				auxiliaryAutoData.schedule = 2;
				SMALLMAGAZINE_OPEN;
			}
		}break;
		case 11:{
			PAWGO_FORWARD;
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
			if(auxiliaryAutoData.countTimes > 300){						
				auxiliaryAutoData.countTimes = 0;								//时间清零
				SMALLMAGAZINE_OPEN;
				EJECT_CLOSE;																		//弹射关闭
				auxiliaryAutoData.schedule = 2;
			}
		}break;
		
		/* 岛下后排抓弹 */
		case 12:{
			chassisData.autoSpeedTarget.x = speed;
			if( TWO_LEFT == 1 ){  
				digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
				if(auxiliaryAutoData.countTimes > 5){							//持续检测到10ms
					digitalClan(&auxiliaryAutoData.countTimes);			//时间清零
					digitalIncreasing(&auxiliaryAutoData.schedule);	//跳下一步	
				}					
			}
			else
				digitalClan(&auxiliaryAutoData.countTimes);
		}break;
		case 13:{
			PAWGO_FORWARD;
			TOP_RIGHT;
			auxiliaryAutoData.schedule = 5;
		}break;
		
		/* 岛上抓弹 */
		case 14:{
			chassisData.autoSpeedTarget.x = speed;
			if((ONE_LEFT == 1) || (ONE_RIGHT == 1)){
				digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
				if(auxiliaryAutoData.countTimes > 5){							//小500ms时
					auxiliaryAutoData.countTimes = 0;								//时间清零
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
		}break;
		case 15:{
			chassisData.autoSpeedTarget.x = 0;
			SMALLMAGAZINE_CLOSE;															//关闭弹舱
			PAW_OPEN;																					//爪子打开
			PAWGO_FORWARD;																		//爪子前进
			PAWTURN_OPEN;			                              	//翻转下去	
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间	
			if(auxiliaryAutoData.countTimes > 600){						
				auxiliaryAutoData.countTimes = 0;								//时间清零
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 16:{
			PAW_CLOSE;																				//爪子夹紧
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
			if(auxiliaryAutoData.countTimes > 100){						
				auxiliaryAutoData.countTimes = 0;								//时间清零
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 17:{
			PAWTURN_CLOSE;																			//爪子收回
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
			if(!PAWTURN_SENSOR || auxiliaryAutoData.countTimes > 1000){						
				auxiliaryAutoData.countTimes = 0;								//时间清零
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 18:{
			PAW_OPEN;																		      //爪子打开
			EJECT_OPEN;                                       //弹射打开
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//开始累计时间
			if(auxiliaryAutoData.countTimes > 300){						
				auxiliaryAutoData.countTimes = 0;								//时间清零
				SMALLMAGAZINE_OPEN;
				EJECT_CLOSE;                                    //弹射关闭
				auxiliaryAutoData.schedule = 2;
			}
		}break;
		
		
		case 99: {//只有调到99时才能退出
			auxiliaryAutoData.taskState = END_OF_EXECUTION;
			grabZeroState();
			chassisSwitch(DISABLE);
			deathMode = 0;
			executeNum = 0;
			auxiliaryAutoData.countTimes = 0;
			shiftAngle = 0.0f;
			chassisData.autoSpeedTarget.x = 0;
		}break;	
		default : auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
  }
	lastPressX = TASK_PRESS_X;
	auxiliaryAutoData.taskDuration += AUTO_TASK_DURATION;
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
