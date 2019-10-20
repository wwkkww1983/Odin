#include "config.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "deforming.h"
#include "vision.h"
#include "auto_tank.h"
#include "rc.h"
#include "DRIVER_VL53L0X.h"
#include "keyboard.h"
AutoTaskStruct_t tankAutoData;

void tankTaskBegin(void){
	tankAutoData.taskState = EXECUTIONG;								//标记为进行中
	digitalIncreasing(&tankAutoData.schedule);					//给执行序列加一
}

/* 键鼠模式遥控器更新 */
void tankRCUpdate(void){
	if(RC_ROTATE > 400 || RC_ROTATE < -400){
		if(PRESS_Q || RC_ROTATE > 400){
			digitalHi(&P_HERO_42_LID);							//开启大弹仓
			digitalHi(&P_HERO_17_LID);							//开启小弹仓
		}
		else if(PRESS_E || RC_ROTATE < -400){
			digitalLo(&P_HERO_42_LID);							//关闭大弹仓
			digitalLo(&P_HERO_17_LID);							//关闭小弹仓
			digitalHi(&shootData.ballisticFill);							//开始弹道填充
		}
	}
}

/* 辅助打击&自瞄任务更新 */
void tankAutomaticAimUpdate(void){										
	switch(tankAutoData.schedule){											//用schedule来增加、减少和判断任务进度
		case 1: gimbalSwitch(DISABLE);												
						chassisSwitch(DISABLE);
						shootVisionInit();
						//配置TX2数据
						visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,BIG_BULLET);							
						digitalIncreasing(&(tankAutoData.schedule));	
						break;
		case 2: gimbalSwitch(ENABLE);	
						//自瞄模式决策
						if(KB_PJEJUDGMENT){												//右键带预判打击
							visionData.prejudgFlag = true;						
							visionData.manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE * 0.5f;
							visionData.manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
						}
						else{																			//左键 不带预判打击
							visionData.prejudgFlag = false;											
							digitalClan(&visionData.manualPitchBias);
							digitalClan(&visionData.manualYawBias);
						}
						//自瞄射击决策
						if(!visionData.prejudgFlag){
						shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_42mm);
						}
						else{
							shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_42mm);
						}		
						if(tankAutoData.breakSign){
							tankAutoData.schedule = 99;							//如果有打断则结束任务
							visionData.prejudgFlag = false;	
							visionData.fireFbd = false;
							shootDataReset();	
						}			
						break;
		case 99: tankAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						  //只有调到99时才能退出
		default: tankAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							//如果到达列表中没有进度，则任务出错
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* 42mm弹药交接任务更新 */
void tankBulletTransferUpdate(void){
	if(tankAutoData.breakSign){													//如果有打断（中断）
		tankAutoData.schedule = 3;												//直接跳到最后一步
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_X && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 1.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 1;
	}
	switch(tankAutoData.schedule){											//用schedule来增加、减少和判断任务进度
		case 1: 
					 if(tankAutoData.taskDuration < 1.0f){
						 digitalHi(&P_HERO_42_LID);      //开启大弹仓
						 digitalHi(&P_HERO_TV);   		   //升起小屏幕
						 gimbalData.yawAngleStopSet = -80.0f;
						 gimbalData.pitchAngleStopSet = -30.0f;
						 gimbalStopSwitch(ENABLE);
					   chassisData.speedLimit = 0.5f;								//加速过程和最大速度都调整到原来的50%
					 }
					 else{
						 digitalIncreasing(&tankAutoData.schedule);
					 }
					 break;
		case 2: 
					 if(tankAutoData.taskDuration > 30.0f){			//30秒内完成补弹
						 digitalIncreasing(&tankAutoData.schedule);
					 }
					 break;
		case 3:
					 digitalLo(&P_HERO_42_LID);      //关闭大弹仓
					 digitalLo(&P_HERO_17_LID);      //关闭小弹仓
					 digitalLo(&P_HERO_TV);   		   //降下小屏幕
					 chassisData.speedLimit = 1.0f;								  //调整回原来的值
					 digitalHi(&shootData.ballisticFill);							//开始弹道填充
					 gimbalStopSwitch(DISABLE);
					 tankAutoData.schedule = 99;
					 break;
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						//只有调到99时才能退出
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							//如果到达列表中没有进度，则任务出错
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* 自动掉头42mm弹药交接任务更新 */
void tankBulletAroundUpdate(void){
	if(tankAutoData.breakSign){													//如果有打断（中断）
		tankAutoData.schedule = 5;												//直接跳到最后一步
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_X && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 3.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 3;
	}
	switch(tankAutoData.schedule){											//用schedule来增加、减少和判断任务进度
		case 1:{
				autoTaskData->fastSeed = 1;									//快速起步
				gimbalData.yawAngleRef -= 180.0f;						//车体旋转180°
				digitalIncreasing(&tankAutoData.schedule);
		} break;
		case 2:{
			if(tankAutoData.taskDuration > 0.5f){
				if(tankAutoData.taskDuration < 3.0f){			//3秒内完成掉头
					if((gimbalData.yawAngleFbd > gimbalData.yawAngleRef - 5.0f && gimbalData.yawAngleFbd < gimbalData.yawAngleRef + 5.0f)\
						&& (chassisData.chaseFbd > -5.0f && chassisData.chaseFbd < 5.0f)){    //掉头完成
						autoTaskData->fastSeed = 0;
						digitalIncreasing(&tankAutoData.schedule);
					}
				}
				else
					digitalIncreasing(&tankAutoData.schedule);
			}
		}		break;
		case 3:{
						 digitalHi(&P_HERO_42_LID);      //开启大弹仓
						 digitalHi(&P_HERO_TV);   		   //升起小屏幕
						 gimbalData.yawAngleStopSet = -80.0f;
						 gimbalData.pitchAngleStopSet = -30.0f;
						 gimbalStopSwitch(ENABLE);
					   chassisData.speedLimit = 0.5f;								//加速过程和最大速度都调整到原来的50%
						 digitalIncreasing(&tankAutoData.schedule);
					 } break;
		case 4: 
					 if(tankAutoData.taskDuration > 30.0f){			//20秒内完成补弹
						 digitalIncreasing(&tankAutoData.schedule);
					 }
					 break;
		case 5:{
						 digitalLo(&P_HERO_42_LID);      //关闭大弹仓
						 digitalLo(&P_HERO_17_LID);      //关闭小弹仓
						 digitalLo(&P_HERO_TV);   		   //降下小屏幕
						 chassisData.speedLimit = 1.0f;								  //调整回原来的值
						 digitalHi(&shootData.ballisticFill);							//开始弹道填充
						 gimbalData.yawAngleRef = gimbalData.yawAngleFbd;					//立即停止旋转
						 autoTaskData->fastSeed = 0;
						 gimbalStopSwitch(DISABLE);
						 tankAutoData.schedule = 99;
					 } break;
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						//只有调到99时才能退出
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							//如果到达列表中没有进度，则任务出错
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}


/* 17mm弹药补给站交互任务更新 */
void tankBulletSupplyUpdate(void){
	if(tankAutoData.breakSign){													//如果有打断（中断）
		tankAutoData.schedule = 3;												//直接跳到最后一步
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_R && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 1.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 1;
	}
	switch(tankAutoData.schedule){											//用schedule来增加、减少和判断任务进度
		case 1: 
					 if(tankAutoData.taskDuration < 1.0f){
						 digitalHi(&P_HERO_17_LID);      //开启小弹仓
					   chassisData.speedLimit = 0.5f;								//加速过程和最大速度都调整到原来的50%
					 }
					 else{
						 digitalIncreasing(&tankAutoData.schedule);
					 }
					 break;
		case 2: 
					 if(tankAutoData.taskDuration > 30.0f){			//30秒内完成补弹
						 digitalIncreasing(&tankAutoData.schedule);
					 }
					 break;
		case 3:
					 digitalLo(&P_HERO_42_LID);      //关闭大弹仓
					 digitalLo(&P_HERO_17_LID);      //关闭小弹仓
					 chassisData.speedLimit = 1.0f;								  //调整回原来的值
					 tankAutoData.schedule = 99;
					 break;
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						//只有调到99时才能退出
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							//如果到达列表中没有进度，则任务出错
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* 摇摆躲避子弹任务更新 */
void tankAviodUpdate(void){
	static uint8_t avoidTurn = 0; 
	if(tankAutoData.avoidTask){
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
		if(tankAutoData.aviodDuration > R_TIME){					//如果计时大于6.0s的时间，则将当前任务结束
			chassisData.chaseRef = 0.0f;
			tankAutoData.avoidTask = DISABLE;
		}	
		tankAutoData.aviodDuration += AUTO_TASK_DURATION;	
	}
	else{
		chassisData.chaseRef = 0.0f;
	}
}

/* 快速掉头任务更新 */
void tankTurnAroundUpdate(void){
	if(tankAutoData.breakSign){													//如果有打断（中断）
		tankAutoData.schedule = 3;												//直接跳到最后一步
		digitalLo(&tankAutoData.breakSign);
	}
	switch(tankAutoData.schedule){											//用schedule来增加、减少和判断任务进度
		case 1:{
				autoTaskData->fastSeed = 1;									//快速起步
				gimbalData.yawAngleRef -= 180.0f;						//车体旋转180°
				digitalIncreasing(&tankAutoData.schedule);
		} break;
		case 2:{
			if(tankAutoData.taskDuration > 0.5f){
				if(tankAutoData.taskDuration < 3.0f){			//3秒内完成掉头
					if(gimbalData.yawAngleFbd > gimbalData.yawAngleRef - 5.0f || gimbalData.yawAngleFbd < gimbalData.yawAngleRef + 5.0f){    //掉头完成
						autoTaskData->fastSeed = 0;
						tankAutoData.schedule = 99;
					}
				}
				else
					digitalIncreasing(&tankAutoData.schedule);
			}
		}		break;
		case 3:
				gimbalData.yawAngleRef = gimbalData.yawAngleFbd;					//立即停止旋转
				autoTaskData->fastSeed = 0;
				tankAutoData.schedule = 99;
				break;
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						//只有调到99时才能退出
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							//如果到达列表中没有进度，则任务出错
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* 迫击炮更新 */
void tankMortarUpdate(void){
	if(tankAutoData.breakSign){													//如果有打断（中断）
		tankAutoData.schedule = 4;												//直接跳到最后一步
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_CTRL && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 1.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 3;
	}
	switch(tankAutoData.schedule){											//用schedule来增加、减少和判断任务进度
		case 1: {
					 gimbalStopSwitch(DISABLE);
					 gimbalSwitch(DISABLE);
					 
					 digitalIncreasing(&tankAutoData.schedule);
				 } break;
		case 2:{ 		 
					 if(gimbalData.initFinishFlag){
						 digitalHi(&gimbalData.followLock);
						 gimbalSwitch(ENABLE);
						 chassisData.ctrlMode = CHASSIS_STOP;
						 digitalIncreasing(&tankAutoData.schedule);
					 }
				 } break;
		case 3:{ 
					 if(tankAutoData.taskDuration < 30.0f){			//迫击炮模式维持20s
						 visionMortar();
						 if(KB_PJEJUDGMENT){												//右键带预判打击					
						 	 visionData.manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
							 visionData.manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE;
						 }
					 }
					 else{
						 digitalIncreasing(&tankAutoData.schedule);
					 }
				 } break;
		case 4: {
					 gimbalStopSwitch(DISABLE);
					 gimbalSwitch(DISABLE);
					 visionData.prejudgFlag = false;	
					 visionData.fireFbd = false;
					 shootDataReset();	
					 digitalLo(&gimbalData.followLock);
					 digitalClan(&visionData.manualPitchBias);
					 digitalClan(&visionData.manualYawBias);
					 tankAutoData.schedule = 99;
				}  break;
					 
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						//只有调到99时才能退出
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							//如果到达列表中没有进度，则任务出错
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* 自杀开火模式更新 */
void tankSuicideFireUpdate(void){
	if(PRESS_C){
		digitalHi(&shootData.suicideFireFlag);
	}
	else{
		digitalLo(&shootData.suicideFireFlag);
	}
}

void tankAutoTaskUpdate(void){
	tankAviodUpdate();																	//躲避任务（和步兵躲避任务相同）
	tankSuicideFireUpdate();														//自杀开火模式更新
	tankRCUpdate();                                     //遥控器更新
	if(tankAutoData.currentTask != TANK_MANUAL){										//必须有可执行任务  TANK_MANUAL（手动)
		if(tankAutoData.taskState == UNEXECUTED){										//如果任务刚刚开始执行
			tankTaskBegin();																//执行开始序列
		}
		else if(tankAutoData.taskState == EXECUTIONG){		//如果在执行中
			switch(tankAutoData.currentTask){
				case TANK_AUTOMATIC_AIM: {										//辅助瞄准
					tankAutomaticAimUpdate();
					break;
				}
				case TANK_TURN_AROUND: {											//快速掉头
					tankTurnAroundUpdate();
					break;
				}
				case TANK_MORTAR: {														//迫击炮模式
					tankMortarUpdate();
					break;
				}
				case TANK_BULLET_TRANSFER: {									//42mm弹药交接
//					tankBulletTransferUpdate();
					tankBulletAroundUpdate();
					break;
				}
				case TANK_BULLET_SUPPLY: {									   //17mm弹药补给站交互
					tankBulletSupplyUpdate();
					break;
				}
				default: {																				//如果是其他命令，则直接重新初始化结构体
					autoDataInit(&tankAutoData); 										//将数据清零
					break;
				}
			}
		}
		else{																									//如果执行完毕或执行错误，则重新
			autoDataInit(&tankAutoData);
		}
	}
}
