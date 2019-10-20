#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "vision.h"
#include "supply.h"
#include "rc.h"
#include "auto_infantry.h"
#include "config.h"
#include "imu.h"

AutoTaskStruct_t infantryAutoData;

void infantryTaskBegin(void){
	infantryAutoData.taskState = EXECUTIONG;										//标记为进行中
	digitalIncreasing(&infantryAutoData.schedule);							//给执行序列加一
}

void infantryAutomaticAimUpdate(void){												//辅助打击/自瞄任务更新
	switch(infantryAutoData.schedule){													//用schedule来增加、减少和判断任务进度
		case 1: gimbalSwitch(DISABLE);												
						chassisSwitch(DISABLE);
						//配置TX2数据
						visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);							
						digitalIncreasing(&(infantryAutoData.schedule));	
						break;
		case 2: gimbalSwitch(ENABLE);	
						shootVisionInit();
						//自瞄模式决策
						if(KB_PJEJUDGMENT){																//右键 带预判打击
							visionData.prejudgFlag = true;	
							visionData.manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE * 0.5f;
							visionData.manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
						}
						else{																							//左键 不带预判打击
							visionData.prejudgFlag = false;											
							digitalClan(&visionData.manualPitchBias);
							digitalClan(&visionData.manualYawBias);
						}
						//自瞄射击决策
						if(!visionData.prejudgFlag){
							shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_17mm);
						}
						else{
							shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_17mm);	
						}										
						if(infantryAutoData.breakSign){
							infantryAutoData.schedule = 99;									//如果有打断则结束任务
							visionData.prejudgFlag = false;
							visionData.fireFbd = false;
							shootDataReset();	
						}			
						break;
		case 99: infantryAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						  //只有调到99时才能退出
		default: infantryAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							//如果到达列表中没有进度，则任务出错
	}
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;
}

void infantryAutomaticBuffUpdate(void){										//辅助打击/自瞄任务更新
	static shootMode_e originalShootMode = MANUAL_SINGLE;
	static bool wKey = 0;
	static bool sKey = 0;
	switch(infantryAutoData.schedule){													//用schedule来增加、减少和判断任务进度
		case 1: gimbalSwitch(DISABLE);
						chassisSwitch(DISABLE);
						originalShootMode = shootData.shootMode;
						//配置TX2数据
						visionSendDataUpdate(TX2_DISTINGUISH_BUFF,SMALL_BULLET);							
						digitalIncreasing(&(infantryAutoData.schedule));	
						break;
		case 2: gimbalSwitch(ENABLE);	
						//if(visionData.captureFlag&&visionData.cailSuccess){
							chassisSwitch(ENABLE);
							chassisData.posRef.x = 0.0f;
							chassisData.posRef.y = 0.0f;
							chassisData.posRef.z = 0.0f;
							chassisData.posFbd.x = 0.0f;
							chassisData.posFbd.y = 0.0f;
							chassisData.posFbd.z = 0.0f;
						//}						
						shootVisionInit();
						shootData.shootMode = MANUAL_SINGLE;
						//自瞄模式决策
						if(KB_NO_PJEJUDGMENT){														//左键
							visionData.prejudgFlag = false;														
						}
						
						if(remoteControlData.dt7Value.keyBoard.bit.E){
							visionData.rBiasMode = 1;
						}
						else if(remoteControlData.dt7Value.keyBoard.bit.D){
							visionData.rBiasMode = 2;
						}
						else{
							visionData.rBiasMode = 0;
						}
						
						if(remoteControlData.dt7Value.keyBoard.bit.W){
							wKey = 1;
						}
						else if(remoteControlData.dt7Value.keyBoard.bit.S){
							sKey = 1;
						}
						
						if(wKey){
							if(!remoteControlData.dt7Value.keyBoard.bit.W){
								wKey = 0;
								visionData.buffBias += 0.1f;
							}
						}
						if(sKey){
							if(!remoteControlData.dt7Value.keyBoard.bit.S){
								sKey = 0;
								visionData.buffBias -= 0.1f;
							}
						}
						
						shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_BUFF,&shootData.fireFlag_17mm);						
						if(infantryAutoData.breakSign){
							infantryAutoData.schedule = 99;									//如果有打断则结束任务
							visionData.prejudgFlag = false;
							visionData.fireFbd = false;
							shootData.shootMode = originalShootMode;
							shootDataReset();	
						}			
						break;
		case 99: infantryAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						  //只有调到99时才能退出
		default: infantryAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							//如果到达列表中没有进度，则任务出错
	}
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;
}

void infantryBulletTransferUpdate(void){									//子弹交接任务更新（拨弹）
	if(infantryAutoData.breakSign){													//如果有打断（中断）
		supplyData.supplySpeedRef = 2000;										//翻转机构速度期望值
		infantryAutoData.taskDuration = 62.1f;
		infantryAutoData.schedule = 3;												//直接跳到最后一步
		digitalLo(&infantryAutoData.breakSign) ;
	}
	if(TASK_PRESS_R && (infantryAutoData.taskDuration < 60.0f && infantryAutoData.taskDuration > 2.1f)){
		infantryAutoData.taskDuration = 0.0f;
		infantryAutoData.schedule = 1;
	}
	switch(infantryAutoData.schedule){											//用schedule来增加、减少和判断任务进度
		case 1: 
					 if(infantryAutoData.taskDuration < 1.0f){
						 supplyData.supplySpeedRef = -2000;
					   chassisData.speedLimit = 0.4f;								//加速过程和最大速度都调整到原来的40%
					 }
					 else{
						 supplyData.supplySpeedRef = -80;
						 digitalIncreasing(&infantryAutoData.schedule);
					 }
					 break;
		case 2: 
					 if(infantryAutoData.taskDuration > 62.0f){			//这里改成了30秒
						 supplyData.supplySpeedRef = 2000;
						 digitalIncreasing(&infantryAutoData.schedule);
					 }
					 break;
		case 3:
					 chassisData.speedLimit = 1.0f;								  //调整回原来的值
					 if(infantryAutoData.taskDuration > 63.0f){			//2秒钟时间关闭
						 supplyData.supplySpeedRef = 80;
						 infantryAutoData.schedule = 99;
					 }
					 break;
		case 99: infantryAutoData.taskState = END_OF_EXECUTION; break;						//只有调到99时才能退出
		default: infantryAutoData.taskState = EXECUTION_ERROR; break;							//如果到达列表中没有进度，则任务出错
	}
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;	
}

void infantryAviodUpdate(void){													 
	static uint8_t avoidTurn = 0;

#if	USE_CHANGE_HEAD 
	//扭腰，小陀螺加调头加调底盘
	static uint8_t releaseFlag = 0;
	static uint8_t pressInitFlag = 0;
	static uint8_t pressExchangeFlag = 0;
	if(infantryAutoData.avoidTask && !chassisData.changeChassisSchedule && !chassisData.changeHeadSchedule){ 			//摇摆躲避子弹任务更新		以45度角面对敌人		
		 switch(infantryAutoData.avoidSchedule){
			case 0:
				if(chassisData.chaseRef < AVIOD_INITIAL_ANGEL)				// 车体以45度角迎敌
					chassisData.chaseRef += AVOID_RATE;
				else
					digitalIncreasing(&infantryAutoData.avoidSchedule);
				break;
			case 1:
				if(avoidTurn){
					if(chassisData.chaseRef < (45.0f+AVOID_RANGE))			//120
						chassisData.chaseRef += AVOID_RATE;
					else if(chassisData.chaseRef > (45.0f+AVOID_RANGE)){
						chassisData.chaseRef -= AVOID_RATE;
						digitalLo(&avoidTurn);
					}
				}
				else{
					if(chassisData.chaseRef > (45.0f-AVOID_RANGE))			//-30
						chassisData.chaseRef -= AVOID_RATE;
					else if(chassisData.chaseRef < (45.0f-AVOID_RANGE)){
						chassisData.chaseRef += AVOID_RATE;
						digitalHi(&avoidTurn);
					}
				}
				break;
		}
		if(infantryAutoData.aviodDuration > R_TIME){							 //如果计时大于6.0s的时间，则将当前任务结束
			chassisData.chaseRef = 0.0f;
			digitalClan(&infantryAutoData.avoidSchedule);							 //在扭腰结束后能保证下次扭腰先45度角迎敌
			infantryAutoData.avoidTask = DISABLE;		
		}	
		infantryAutoData.aviodDuration += AUTO_TASK_DURATION;	
	}
	else{																										     //小陀螺旋转躲避
		if(PRESS_Q || PRESS_E || RC_ROTATE > 100 || RC_ROTATE < -100 ){
			if(PRESS_Q || RC_ROTATE > 100){
				chassisData.chaseRef += (AVOID_RATE * 4);
			}
			if(PRESS_E || RC_ROTATE < -100){
				chassisData.chaseRef -= (AVOID_RATE * 4);
			}
		}
		else{																																				//可以删除 规则规定不能以灯条方向作为前进方向
			if(!chassisData.changeChassisSchedule){																		//底盘没有进行调头
				switch(chassisData.changeHeadSchedule){																			
					case 0:
						if(RC_TRANSVERSE > 490 && RC_LONGITUDINAL > 490){                   //给掉头指令
							digitalIncreasing(&chassisData.changeHeadSchedule);
						}	
					break;				
					case 1:
						chassisData.changeHeadOrder = 1.0f;
						gimbalData.yawAngleRef +=180;
						digitalIncreasing(&chassisData.changeHeadSchedule);
						break;
					case 2:
						if(ABS(RC_TRANSVERSE) < 10 && ABS(RC_LONGITUDINAL) < 10){   							 //松手检测
							releaseFlag = 1;
						}
						if(( ABS(IMU_RATEZ) < 0.008f ) && ( ABS(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData)-parameter[YAW_CENTER]) > 2000 )){
							parameter[YAW_CENTER] =  (parameter[YAW_CENTER] + 4096 ) > 8192?  (parameter[YAW_CENTER] - 4096) : (parameter[YAW_CENTER] + 4096);
							gimbalData.yawMotorAngle = ENCODER_ANGLE_RATIO * getRelativePos(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData),parameter[YAW_CENTER]);
							chassisData.changeHeadOrder = 0.0f;           //开始跟随
							chassisData.direction = -chassisData.direction; 
							digitalIncreasing(&chassisData.changeHeadSchedule);
						}
						break;
					case 3: 
						if(releaseFlag){																													  //调头完成之前松手检测
							releaseFlag = 0;
							digitalClan(&chassisData.changeHeadSchedule);						
						}
						if(ABS(RC_TRANSVERSE) < 10 && ABS(RC_LONGITUDINAL) < 10){   							 //调头完成之后松手检测
							digitalClan(&chassisData.changeHeadSchedule);														 //可以进行下一次掉头
						}
						break;
					default:
						chassisData.chaseRef = 0.0f;
						break;
				}
			}
			if(!chassisData.changeHeadSchedule){																			 			 	//云台没有进行调头
				switch(chassisData.changeChassisSchedule){
					case 0:
						if(RC_TRANSVERSE < -490 && RC_LONGITUDINAL < -490){													//遥控器给调头指令给调头指令
							pressExchangeFlag = 2;
							digitalIncreasing(&chassisData.changeChassisSchedule);
						}
						else
							if(TASK_PRESS_CTRL && (TASK_PRESS_X ^ TASK_PRESS_Z)){											//键盘给调头指令
								if(!pressInitFlag){
									pressExchangeFlag = 0;
									digitalHi(&pressInitFlag);
								}
									digitalIncreasing(&chassisData.changeChassisSchedule);
							}	
							else
								chassisData.chaseRef = 0.0f;
						break;
					case 1:	
						switch(pressExchangeFlag){																									
							case 0:
								if(TASK_PRESS_X){
									pressExchangeFlag = 1;
									chassisData.chaseRef +=180;
									digitalIncreasing(&chassisData.changeChassisSchedule);
								}
								break;
							case 1:
								if(TASK_PRESS_Z){
									pressExchangeFlag = 0;
									chassisData.chaseRef +=180;
									digitalIncreasing(&chassisData.changeChassisSchedule);
								}
								break;
							default:
								if(RC_TRANSVERSE < -490 && RC_LONGITUDINAL < -490)					
									chassisData.chaseRef +=180;
									digitalIncreasing(&chassisData.changeChassisSchedule);
								break;
						}
						break;
					case 2:
						switch(pressExchangeFlag){																			//完成前松手检测
							case 0:
								if(!TASK_PRESS_Z)
									releaseFlag = 1;
								break;
							case 1:
								if(!TASK_PRESS_X)
									releaseFlag = 1;
								break;
							default:
								if(ABS(RC_TRANSVERSE) < 10 && ABS(RC_LONGITUDINAL) < 10){
									digitalClan(&pressInitFlag);
									releaseFlag = 1;	
								}
								break;
						}
//						if(( ABS(IMU_CHASE_RATEZ) < 0.005f ) && ( ABS(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData)-parameter[YAW_CENTER]) > 4000)){							
					  if(ABS(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData)-parameter[YAW_CENTER]) > 4000){					//机器人自我判断调头完成
							parameter[YAW_CENTER] =  (parameter[YAW_CENTER] + 4096 ) > 8192?  (parameter[YAW_CENTER] - 4096) : (parameter[YAW_CENTER] + 4096);
							gimbalData.yawMotorAngle = ENCODER_ANGLE_RATIO * getRelativePos(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData),parameter[YAW_CENTER]);
							chassisData.chaseRef = 0.0f;
							chassisData.direction = -chassisData.direction;
							digitalIncreasing(&chassisData.changeChassisSchedule);
						}
						break;
					case 3:
						if(releaseFlag){																													 //调头完成之前松手检测
							releaseFlag = 0;
							digitalClan(&chassisData.changeChassisSchedule);						
						}
						switch(pressExchangeFlag){                                                 //完成后松手检测
							case 0:
								if(!TASK_PRESS_Z){
									digitalClan(&chassisData.changeChassisSchedule);	
								}
								break;
							case 1:
								if(!TASK_PRESS_X){
									digitalClan(&chassisData.changeChassisSchedule);
								}									
								break;
							default:
								if(ABS(RC_TRANSVERSE) < 10 && ABS(RC_LONGITUDINAL) < 10){									 //调头完成之后松手检测
									digitalClan(&pressInitFlag);											
									digitalClan(&chassisData.changeChassisSchedule);												 //可以进行下次底盘掉头
								}
								break;
						}
						break;
					default:
						chassisData.chaseRef = 0.0f;
						break;		
			  }
		  } 
	  }
  }
#else
	//扭腰加小陀螺
	if(infantryAutoData.avoidTask){
		switch(infantryAutoData.avoidSchedule){
			case 0:
				if(chassisData.chaseRef < AVIOD_INITIAL_ANGEL)				// 车体以45度角迎敌
					chassisData.chaseRef += AVOID_RATE;
				else
					digitalIncreasing(&infantryAutoData.avoidSchedule);
				break;
			case 1:
				if(avoidTurn){
					if(chassisData.chaseRef < (AVIOD_INITIAL_ANGEL+AVOID_RANGE))			//120
						chassisData.chaseRef += AVOID_RATE;
					else if(chassisData.chaseRef >= (AVIOD_INITIAL_ANGEL+AVOID_RANGE)){
						chassisData.chaseRef -= AVOID_RATE;
						digitalLo(&avoidTurn);
					}
				}
				else{
					if(chassisData.chaseRef > (AVIOD_INITIAL_ANGEL-AVOID_RANGE))			//-30
						chassisData.chaseRef -= AVOID_RATE;
					else if(chassisData.chaseRef <= (AVIOD_INITIAL_ANGEL-AVOID_RANGE)){
						chassisData.chaseRef += AVOID_RATE;
						digitalHi(&avoidTurn);
					}
				}
			 break;
		}
		if(infantryAutoData.aviodDuration > R_TIME){							 //如果计时大于6.0s的时间，则将当前任务结束
			chassisData.chaseRef = 0.0f;
			infantryAutoData.aviodFlag = false;
			digitalClan(&infantryAutoData.avoidSchedule);							 //在扭腰结束后能保证下次扭腰先45度角迎敌
			infantryAutoData.avoidTask = DISABLE;		
		}	
		infantryAutoData.aviodDuration += AUTO_TASK_DURATION;		
	}
	else{	//小陀螺旋转躲避
	static uint8_t lastKeySate,kmRotateFlag,rcRotateFlag,rotateDirection;		
		if(RC_ROTATE > 100)
				rcRotateFlag = 1;
			else
				rcRotateFlag = 0;
			//一键按下旋转
			if(!lastKeySate && !kmRotateFlag){																	
				if(PRESS_Q){
					kmRotateFlag = 1;
					//下一次自转改变转向
					rotateDirection = !rotateDirection;
				}
			}
			else{
				//再次按下解除旋转
				if(!lastKeySate && kmRotateFlag){
					if(PRESS_Q || TASK_PRESS_F)
						kmRotateFlag = 0;
				}
				//死亡自动解除小陀螺
				if(infantryAutoData.closeRotate){
					kmRotateFlag = 0;
					infantryAutoData.closeRotate = false;
				}
			}
			lastKeySate = PRESS_Q;
			
			if(kmRotateFlag || rcRotateFlag){
				if(rotateDirection)
				 chassisData.chaseRef -= (AVOID_RATE * 4);
				else
					chassisData.chaseRef += (AVOID_RATE * 4);
				infantryAutoData.rotateFlag = true;
			}
			else{
				infantryAutoData.rotateFlag = false;
				chassisData.chaseRef = 0.0f;
			}
	}
#endif
}

void infantryAutoTaskUpdate(void){
	if(infantryAutoData.currentTask != INFANTRY_ACTIVE_BUFF){
		infantryAviodUpdate(); //躲避任务  （INFANTRY_MANUAL）手动模式
	}
	else{
		//打符时结束躲避
		chassisData.chaseRef = 0.0f;
		infantryAutoData.aviodFlag = false;
		digitalClan(&infantryAutoData.avoidSchedule);				
		infantryAutoData.avoidTask = DISABLE;		
	}
																		
	if(infantryAutoData.currentTask != INFANTRY_MANUAL){		//必须有可执行任务
		if(infantryAutoData.currentTask != INFANTRY_BULLET_TRANSFER \
			&& infantryAutoData.lastTask == INFANTRY_BULLET_TRANSFER){
			supplyData.supplySpeedRef = 80;
		}
		if(infantryAutoData.taskState == UNEXECUTED){					//如果任务刚刚开始执行   未执行（UNEXECUTED）
			infantryTaskBegin();																//执行开始序列
		}
		else if(infantryAutoData.taskState == EXECUTIONG){		//如果在执行中
			switch(infantryAutoData.currentTask){								//当前任务
				case INFANTRY_AUTOMATIC_AIM:  {										//辅助瞄准
					infantryAutomaticAimUpdate();
					break;
				}
				case INFANTRY_ACTIVE_BUFF: {											//神符打击
					infantryAutomaticBuffUpdate();
					break;
				}
				case INFANTRY_BULLET_TRANSFER: {									//子弹交接
					infantryBulletTransferUpdate();
					break;
				}
				default: {																				//如果是其他命令，则直接重新初始化结构体
					autoDataInit(&infantryAutoData); 
					break;
				}
			}
		}
		else{																									//如果执行完毕或执行错误，则重新初始化结构体
			autoDataInit(&infantryAutoData);
		}
	}
	//更新上一刻的任务
	infantryAutoData.lastTask = infantryAutoData.currentTask;
}

