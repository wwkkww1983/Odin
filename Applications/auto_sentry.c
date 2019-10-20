#include "auto_sentry.h"
#include "vision.h"
#include "gimbal.h"
#include "chassis.h"
#include "shoot.h"
#include "config.h"
#include "rc.h"
#include "Util.h"
#include "supervisor.h"
#include "judge.h"

#define STAY_TIME 250

AutoTaskStruct_t sentryAutoData;
u8 sentryLeft,sentryRight;
uint8_t attackMode = 0;
uint32_t lostTargetCNT = 0;
uint8_t visionSchedule = 1;
uint32_t baseSpeed = SPEEDHIGH;
float yawAdd = 0.05f;
float pitchAdd = 0.10f;
bool upFlag = 0;
float pitchPatrolMax = -10.0f;
float pitchPatrolMin = -45.0f;
float turnRate = 8.0f;
uint32_t stopTime = 0;
bool direction = 0;
float AngleSave = 0;
int8_t turnFlag = 0;     //转弯标志位
int8_t lastTurnFlag = 0; //记录上一次转弯标志位

void sentryTaskBegin(void){
	sentryAutoData.taskState = EXECUTIONG;									//标记为进行中
	digitalIncreasing(&sentryAutoData.schedule);						//给执行序列加一
}

uint16_t n = 0;
void sentryAngleUpdate(void){
		if(!upFlag){
			switch(stopTime/STAY_TIME){
				case 1 :gimbalData.yawAngleRef = AngleSave-60.0f;
								gimbalData.pitchAngleRef = pitchPatrolMin;break;
				case 2 :gimbalData.yawAngleRef =  AngleSave;
								gimbalData.pitchAngleRef = pitchPatrolMax;break;
				case 3 :gimbalData.yawAngleRef =  AngleSave;
								gimbalData.pitchAngleRef = pitchPatrolMin;break;
				case 4 :gimbalData.yawAngleRef =   AngleSave + 60.0f;
								gimbalData.pitchAngleRef = pitchPatrolMax; 
								upFlag = true;
								digitalClan(&stopTime);
								break;
			}
		}
		else{
			switch(stopTime/STAY_TIME){
				case 4 :gimbalData.yawAngleRef =  AngleSave-60.0f;
								gimbalData.pitchAngleRef = pitchPatrolMax;
								upFlag = false;
								digitalClan(&stopTime);
								break;
				case 3 :gimbalData.yawAngleRef =  AngleSave;
								gimbalData.pitchAngleRef = pitchPatrolMin;break;
				case 2 :gimbalData.yawAngleRef =  AngleSave;
								gimbalData.pitchAngleRef = pitchPatrolMax;break;
				case 1 :gimbalData.yawAngleRef =   AngleSave+60.0f;
								gimbalData.pitchAngleRef = pitchPatrolMin;break;
			}
		}	
		stopTime++;
		n = stopTime/STAY_TIME;
}

void sentryPatrolTaskUpdate(void){
	static int8_t coefficient;
	static u8 flagAddEncoderAngle = 0;
	static u8 flagReduceEncoderAngle = 0;
	static float frontCenterAngle = 0;
	static float backCenterAngle = 0;
	switch(sentryAutoData.schedule){
		case 1:{
			chassisSwitch(DISABLE);
			gimbalSwitch(DISABLE);
			srand(controlData.loops);
			if(parameter[YAW_CENTER] > 4096){												//判断中心位置
				frontCenterAngle = parameter[YAW_CENTER];							//得出前后装甲板中心的码盘值
				backCenterAngle = parameter[YAW_CENTER] - 4096;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > backCenterAngle && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < frontCenterAngle){
					digitalHi(&flagAddEncoderAngle);
					digitalLo(&flagReduceEncoderAngle);
				}
				else{
					digitalLo(&flagAddEncoderAngle);
					digitalHi(&flagReduceEncoderAngle);
				}
			}
			else{
				frontCenterAngle = parameter[YAW_CENTER];
				backCenterAngle = parameter[YAW_CENTER] + 4096;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < backCenterAngle && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > frontCenterAngle){
					digitalLo(&flagAddEncoderAngle);
					digitalHi(&flagReduceEncoderAngle);
				}
				else{
					digitalHi(&flagAddEncoderAngle);
					digitalLo(&flagReduceEncoderAngle);
				}
			}
			digitalIncreasing(&sentryAutoData.schedule);
		}break;
		case 2:{																										//转动到1号装甲板的中心
			if(flagAddEncoderAngle){
				coefficient = 2;
				direction = 1;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > frontCenterAngle - 100 && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < frontCenterAngle + 100){
					digitalLo(&flagAddEncoderAngle);
					coefficient = 1;
					AngleSave = gimbalData.yawAngleRef;
					sentryAutoData.schedule = 3;
				}
			}
			else if(flagReduceEncoderAngle){
				coefficient = -2;
				direction = 0;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > frontCenterAngle - 100 && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < frontCenterAngle + 100){
					digitalLo(&flagReduceEncoderAngle);
					coefficient = -1;
					AngleSave = gimbalData.yawAngleRef;
					sentryAutoData.schedule = 3;
				}
			}
			/* 云台执行转动 */
			gimbalData.yawAngleRef += turnRate * yawAdd * coefficient;
		}break;
		case 3:{
			if(judgeData.extGameState.game_progress == 4)
				sentryAngleUpdate();
		}break;
		case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		//在哨兵中，只有摇杆拨到切出控制权位置时才能跳出任务
		default: sentryAutoData.taskState = EXECUTION_ERROR; break;			//如果到达列表中没有进度，则任务出错
	}
}

//void sentryPitchUpdate(void){
//	gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;					//设置pitch轴反馈			
//	if( (gimbalData.pitchAngleRef > parameter[PITCH_MIN_RANGE]) && !upFlag )
//		gimbalData.pitchAngleRef -=  pitchAdd;
//	else
//		upFlag = 1;
//	if( (gimbalData.pitchAngleRef < pitchPatrolMax) && upFlag )
//		gimbalData.pitchAngleRef +=  pitchAdd;
//	else
//		upFlag = 0;
//}

//void sentryPatrolTaskUpdate(void){
//	static int8_t coefficient;
//	static u8 flagAddEncoderAngle = 0;
//	static u8 flagReduceEncoderAngle = 0;
//	static float AngleSave = 0;
//	static float frontCenterAngle = 0;
//	static float backCenterAngle = 0;
//	switch(sentryAutoData.schedule){
//		case 1:{
//			chassisSwitch(DISABLE);
//			gimbalSwitch(DISABLE);
//			srand(controlData.loops);
//			if(parameter[YAW_CENTER] > 4096){												//判断中心位置
//				frontCenterAngle = parameter[YAW_CENTER];							//得出前后装甲板中心的码盘值
//				backCenterAngle = parameter[YAW_CENTER] - 4096;
//				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > backCenterAngle && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < frontCenterAngle){
//					digitalHi(&flagAddEncoderAngle);
//					digitalLo(&flagReduceEncoderAngle);
//				}
//				else{
//					digitalLo(&flagAddEncoderAngle);
//					digitalHi(&flagReduceEncoderAngle);
//				}
//			}
//			else{
//				frontCenterAngle = parameter[YAW_CENTER];
//				backCenterAngle = parameter[YAW_CENTER] + 4096;
//				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < backCenterAngle && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > frontCenterAngle){
//					digitalLo(&flagAddEncoderAngle);
//					digitalHi(&flagReduceEncoderAngle);
//				}
//				else{
//					digitalHi(&flagAddEncoderAngle);
//					digitalLo(&flagReduceEncoderAngle);
//				}
//			}
//			digitalIncreasing(&sentryAutoData.schedule);
//		}break;
//		case 2:{																										//转动到1号装甲板的中心
//			if(flagAddEncoderAngle){
//				coefficient = 2;
//				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > frontCenterAngle - 100 && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < frontCenterAngle + 100){
//					digitalLo(&flagAddEncoderAngle);
//					coefficient = 1;
//					AngleSave = gimbalData.yawAngleRef;
//					sentryAutoData.schedule = 3;
//				}
//			}
//			else if(flagReduceEncoderAngle){
//				coefficient = -2;
//				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > frontCenterAngle - 100 && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < frontCenterAngle + 100){
//					digitalLo(&flagReduceEncoderAngle);
//					coefficient = -1;
//					AngleSave = gimbalData.yawAngleRef;
//					sentryAutoData.schedule = 3;
//				}
//			}
//			/* 云台执行转动 */
//			gimbalData.yawAngleRef += turnRate * yawAdd * coefficient;
//		}break;
//		case 3:{
//			sentryPitchUpdate();
//			/* 120°扇形扫描 */
//			if( gimbalData.yawAngleRef > AngleSave + 60 )
//					coefficient = -coefficient;
//			else
//				if( gimbalData.yawAngleRef < AngleSave - 60 )
//					coefficient = -coefficient;
//			/* 云台执行转动 */
//			gimbalData.yawAngleRef += 2 * yawAdd * coefficient;
//		}break;
//		case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		//在哨兵中，只有摇杆拨到切出控制权位置时才能跳出任务
//		default: sentryAutoData.taskState = EXECUTION_ERROR; break;			//如果到达列表中没有进度，则任务出错
//	}
//}

//void sentryPatrolTaskUpdate(void){
//	static u8 upFlag = 0;
//	switch(sentryAutoData.schedule){
//		case 1:{
//			chassisSwitch(ENABLE);
//			gimbalSwitch(DISABLE);
//			digitalIncreasing(&sentryAutoData.schedule);
//		}break;
//		case 2:{
//			gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;					//设置pitch轴反馈
//			if( (gimbalData.pitchAngleRef > 0.0f) && !upFlag )
//				gimbalData.pitchAngleRef -=  0.06f;
//			else
//				upFlag = 1;
//			if( (gimbalData.pitchAngleRef < 20.0f) && upFlag )
//				gimbalData.pitchAngleRef +=  0.06f;
//			else
//				upFlag = 0;
//			
//			gimbalData.yawAngleRef += 0.1f;
//			
//		}break;
//		case 3: break;
//		case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		//在哨兵中，只有摇杆拨到切出控制权位置时才能跳出任务
//		default: sentryAutoData.taskState = EXECUTION_ERROR; break;			//如果到达列表中没有进度，则任务出错
//	}
//}

/************************************
反馈的深度信息为装甲板大小信息
armorTypeFbd			<-->	depthFbd
armorTypeLastFbd	<-->	depthLastFbd
************************************/
void sentryAttackTaskUpdate(void){
	static TickType_t sentryFireLastWakeTime = 0;	  //时间监测
	switch(attackMode){
		//未捕获目标巡逻模式
		case 0:{
			sentryAutoData.currentTask = SENTRY_PATROL;
			break;
		}
		case 1:{
			if(lostTargetCNT > 300){	
				attackMode = 0;
			}		
			else{
				attackMode = 1;
			}
//			gimbalData.pitchAngleRef = visionData.pitchCmd;				 //自动模式下同步当前值到角度期望
//			gimbalData.yawAngleRef = visionData.yawCmd;	
//			//如果是自瞄状态云台角度反馈取陀螺仪角度（不加偏置）
//			gimbalData.pitchAngleFbd =  gimbalData.pitchGyroAngle;			
			switch(visionSchedule){
				case 1: gimbalSwitch(DISABLE);												
								chassisSwitch(DISABLE);
								//配置TX2数据
								visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);							
								digitalIncreasing(&visionSchedule);	
								sentryFireLastWakeTime = xTaskGetTickCount();
								break;
				case 2: gimbalSwitch(ENABLE);	
								shootVisionInit();
								//自瞄模式决策
								visionData.prejudgFlag = false;											
								//自瞄射击决策
								visionFireFbdUpdata(&shootData.fireFlag_17mm);	
								if(visionData.fireFbd){
									if(shootData.fricMotorFlag&&(judgeData.extGameState.game_progress == 4)){
//									if(shootData.fricMotorFlag){
										sentryFireLastWakeTime = xTaskGetTickCount();
										digitalHi(&shootData.fireFlag_17mm);   //射击使能
									}
								}	
								if(xTaskGetTickCount() - sentryFireLastWakeTime > 200){
										digitalLo(&shootData.fireFlag_17mm);		//射击失能
										digitalLo(&shootData.shootTrigger);	//拨盘失能
								};	//拨盘失能
								break;
//				case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		//在哨兵中，只有摇杆拨到切出控制权位置时才能跳出任务
//				default: sentryAutoData.taskState = EXECUTION_ERROR; break;			//如果到达列表中没有进度，则任务出错
			}
			break;
		}
		
	}	
}


void sentryUnderAttackTaskUpdate(void){
	static int8_t coefficient;
	static u8 flagAddEncoderAngle = 0;
	static u8 flagReduceEncoderAngle = 0;
	static float frontCenterAngle = 0;
	static float backCenterAngle = 0;
	switch(sentryAutoData.schedule){
		case 1:{
			stopTime = 2*STAY_TIME;
			gimbalSwitch(DISABLE);
			chassisSwitch(DISABLE);
			if(parameter[YAW_CENTER] > 4096){												//判断中心位置
				frontCenterAngle = parameter[YAW_CENTER];							//得出前后装甲板中心的码盘值
				backCenterAngle = parameter[YAW_CENTER] - 4096;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > backCenterAngle && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < frontCenterAngle){	//计算往哪边走最快到达中心
					digitalHi(&flagAddEncoderAngle);
					digitalLo(&flagReduceEncoderAngle);
				}
				else{
					digitalLo(&flagAddEncoderAngle);
					digitalHi(&flagReduceEncoderAngle);
				}
			}
			else{
				frontCenterAngle = parameter[YAW_CENTER];							//得出前后装甲板中心的码盘值
				backCenterAngle = parameter[YAW_CENTER] + 4096;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < backCenterAngle && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > frontCenterAngle){	//计算往哪边走最快到达中心
					digitalLo(&flagAddEncoderAngle);
					digitalHi(&flagReduceEncoderAngle);
				}
				else{
					digitalHi(&flagAddEncoderAngle);
					digitalLo(&flagReduceEncoderAngle);
				}
			}
			/* 靠近血条的装甲板（0号）被击打 */
			if( judgeData.extRobotHurt.armor_id == 0x00 &&  judgeData.extRobotHurt.hurt_type == 0x00)											
					sentryAutoData.schedule = 3;
			/* 1号装甲板被击打 */
			if( judgeData.extRobotHurt.armor_id == 0x01 &&  judgeData.extRobotHurt.hurt_type == 0x00 )									
				sentryAutoData.schedule = 2;
		}break;
		case 2:{																									//转动到被击打装甲板的中心
			if(flagAddEncoderAngle){
				coefficient = -2;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > backCenterAngle - 100 && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < backCenterAngle + 100){
					digitalLo(&flagAddEncoderAngle);
					coefficient = 1;
					AngleSave = gimbalData.yawAngleRef;
					sentryAutoData.schedule = 4;
				}
			}
			else if(flagReduceEncoderAngle){
				coefficient = 2;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > backCenterAngle - 100 && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < backCenterAngle + 100){
					digitalLo(&flagReduceEncoderAngle);
					coefficient = -1;
					AngleSave = gimbalData.yawAngleRef;
					sentryAutoData.schedule = 4;
				}
			}
			/* 云台执行转动 */
			gimbalData.yawAngleRef += turnRate * yawAdd * coefficient;
		}break;
		case 3:{																									//转动到被击打装甲板的中心
			if(flagAddEncoderAngle){
				coefficient = 2;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > frontCenterAngle - 100 && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < frontCenterAngle + 100){
					digitalLo(&flagAddEncoderAngle);
					coefficient = 1;
					AngleSave = gimbalData.yawAngleRef;
					sentryAutoData.schedule = 4;
				}
			}
			else if(flagReduceEncoderAngle){
				coefficient = -2;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > frontCenterAngle - 100 && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < frontCenterAngle + 100){
					digitalLo(&flagReduceEncoderAngle);
					coefficient = -1;
					AngleSave = gimbalData.yawAngleRef;
					sentryAutoData.schedule = 4;
				}
			}
			/* 云台执行转动 */
			gimbalData.yawAngleRef +=  turnRate * yawAdd * coefficient;
		}break;
		case 4:{																									//120度扇形扫描
			sentryAngleUpdate();
		}break;
		case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		//在哨兵中，只有摇杆拨到切出控制权位置时才能跳出任务
		default: sentryAutoData.taskState = EXECUTION_ERROR; break;			//如果到达列表中没有进度，则任务出错
	}
}

void sentryModeUpdate(){
	static u8 lastMode;
	static u16 lastHP;
	static bool captureFlag = 0,lastCaptureFlag = 0,modeInit = false;
	
	if(!modeInit){
		lastHP = judgeData.extGameRobotState.remain_HP;
		modeInit = true;
	}
	
	captureFlag = visionData.captureFlag;
	if(!captureFlag){	//没有识别到目标则重置自瞄状态
		lastCaptureFlag = captureFlag;
		lostTargetCNT ++;
	}
	else{
		lostTargetCNT = 0;
		attackMode = 1;
		if((!lastCaptureFlag) && (sentryAutoData.currentTask == SENTRY_ATTACK)){
			sentryAutoData.schedule = 1;	
			attackMode = 0;	
			visionSchedule = 1;
			visionData.prejudgFlag = false;
			visionData.fireFbd = false;
			shootDataReset();
			visionData.miniPTZEnableAim = false;
			visionData.workMode = TX2_STOP;
		}
		lastCaptureFlag = captureFlag;
	}
	
	if(attackMode){	
		sentryAutoData.taskDuration = 0;
		sentryAutoData.currentTask = SENTRY_ATTACK;		//有视觉反馈进入攻击模式
	}
	else if((judgeData.extGameRobotState.remain_HP != lastHP)&&( judgeData.extRobotHurt.hurt_type == 0x00)){
		sentryAutoData.taskDuration = 0;
		sentryAutoData.currentTask = SENTRY_UNTER_ATTACK;	
	}
	else if( (sentryAutoData.taskDuration > 0.8f && sentryAutoData.currentTask == SENTRY_ATTACK) \
				|| (sentryAutoData.taskDuration > 6.0f && sentryAutoData.currentTask == SENTRY_UNTER_ATTACK)
				|| (sentryAutoData.currentTask == SENTRY_MANUAL) )	
		sentryAutoData.currentTask = SENTRY_PATROL;		//以上条件都不满足就是巡逻模式
	else
		sentryAutoData.taskDuration += AUTO_TASK_DURATION;
	
	if( lastMode != sentryAutoData.currentTask )
		sentryAutoData.schedule = 1;
	
	lastHP = judgeData.extGameRobotState.remain_HP;
	
	lastMode = sentryAutoData.currentTask;
}
bool speedFloag = 0;
static void sentryChassisControl(void) 
{
	
	chassisData.speedLimit = 1.0f;
	if(!speedFloag){
		chassisData.landingSpeedy =  baseSpeed /*+ (rand() % 666)*/;
//		if(sentryAutoData.currentTask == SENTRY_ATTACK){
//			chassisData.landingSpeedy = chassisData.landingSpeedy / 3.0f;
//		}
//		if((rand() % 32767) % 2)
//			chassisData.landingSpeedy = -chassisData.landingSpeedy;
		chassisData.landingSpeedy = (float)chassisData.landingSpeedy * REAL_MOTOR_SPEED_SCALE / 1.0f;
		speedFloag = 1;
//		if(sentryAutoData.currentTask == SENTRY_ATTACK){
//			if(chassisData.landingSpeedy > 0)
//				chassisData.landingSpeedy -= 3000.0f;
//			else
//				chassisData.landingSpeedy += 3000.0f;
//		}
	}
}

static void escapeControl(void){
static float escapeTime = 0;
//static u16 initHP;
//static bool initFloag = false;	
//	if(!initFloag){
//		initHP = judgeData.extGameRobotState.remain_HP;
//		initFloag = true;
//	}
	if(escapeTime > 0.3f){
//	  if(judgeData.extGameRobotState.remain_HP != initHP){
//			chassisData.landingSpeedy = -chassisData.landingSpeedy;
//			baseSpeed = SPEEDHIGH;		
//			escapeTime = 0;
//			initHP = 	judgeData.extGameRobotState.remain_HP;
//		  if(turnFlag == 0){      //自动规避
//			  if(turnFlag == 3){
//				  lastTurnFlag = 1;
//				}
//				else
//			    turnFlag = 1;
//			}
//			else if(turnFlag == 1){
//			  if(turnFlag == 3){
//				  lastTurnFlag = 2;
//				}
//			else
//					turnFlag = 2;
//			}
//			else if(turnFlag == 2){
//			  if(turnFlag == 3){
//				  lastTurnFlag = 1;
//				}
//			else			
//			  turnFlag = 1;
//			}
//    }	
//			else{
//		  baseSpeed = SPEEDLOW;
//		}
			if((coderData.xLocation > 4500)||(coderData.xLocation < -3600)){
			  baseSpeed = SPEEDLOW;
			}
			else{
				baseSpeed = SPEEDHIGH;
			}
    if(chassisData.landingSpeedy > 0){
			chassisData.landingSpeedy = baseSpeed + (rand() % 2000);         //谢一源的代码 6299 baseSpeed
		  chassisData.landingSpeedy = (float)chassisData.landingSpeedy * REAL_MOTOR_SPEED_SCALE / 1.0f;
		}
		else if(chassisData.landingSpeedy <= 0){
			chassisData.landingSpeedy = baseSpeed + (rand() % 2000);         //
		  chassisData.landingSpeedy = -(float)chassisData.landingSpeedy * REAL_MOTOR_SPEED_SCALE / 1.0f;
//			chassisData.landingSpeedy = -chassisData.landingSpeedy;			
		}

	}
	if( (!sentryLeft && ( chassisData.landingSpeedy < 0)) \
	||(!sentryRight && ( chassisData.landingSpeedy > 0)) ){
		chassisData.landingSpeedy = -chassisData.landingSpeedy;			//不能撞柱子
	}
escapeTime += AUTO_TASK_DURATION;

}

//static void flagCheck(void){   //车间通信检测
//static uint8_t lastData = 0x01;

//if(judgeData.extReceiveData.data[0] != lastData){
//	if(judgeData.extReceiveData.data[0] == 0x02){   //车间通信
//	if(turnFlag == 3){
//	   lastTurnFlag = 1;
//	}
//	else
//	   turnFlag = 1;
//		 coderData.coderAhead = 0;
//		 coderData.coderBack = 0;
//	}
//	else if(judgeData.extReceiveData.data[0] == 0x03){
//	if(turnFlag == 3){
//	   lastTurnFlag = 2;
//	}
//	else	
//	   turnFlag = 2;
//		 coderData.coderAhead = 0;
//		 coderData.coderBack = 0;      
//	}
//	else{
//	   turnFlag = 0;
//		 coderData.coderAhead = 0;
//		 coderData.coderBack = 0;  
//	}
//}	
//lastData = judgeData.extReceiveData.data[0];
//}

static void curveCheck(void){                          
static int32_t lastHead,lastBack = 0;
static int32_t dHead,dBack = 0;
static float checkTime = 0;
	
		if(turnFlag == 0){
			if( (coderData.coderAhead > (coderData.coderBack + 360)) && ( chassisData.landingSpeedy < 0) ){   //中间轨道
//			 chassisData.landingSpeedy = -chassisData.landingSpeedy;
				baseSpeed = SPEEDLOW;
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			 lastTurnFlag = 0;
			 turnFlag = 3;
			}
			else if( (coderData.coderBack > (coderData.coderAhead + 360)) && ( chassisData.landingSpeedy > 0) ){
//			 chassisData.landingSpeedy = -chassisData.landingSpeedy;
			 baseSpeed = SPEEDLOW;
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			 lastTurnFlag = 0;
			 turnFlag = 3;
			}
			else if( (coderData.coderBack > (coderData.coderAhead + 100)) && ( chassisData.landingSpeedy < 0) ){
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			}
			else if( (coderData.coderAhead > (coderData.coderBack + 100)) && ( chassisData.landingSpeedy > 0) ){
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			}
		}
		
		else if(turnFlag == 1){
			if( (coderData.coderBack > (coderData.coderAhead + 360)) && ( chassisData.landingSpeedy < 0) ){  //左边轨道
			 chassisData.landingSpeedy = -chassisData.landingSpeedy;
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			 lastTurnFlag = 1;
			 turnFlag = 3;
			}
			else if( (coderData.coderBack > (coderData.coderAhead + 100)) && ( chassisData.landingSpeedy > 0) ){
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
							}
			else if( (coderData.coderAhead > (coderData.coderBack + 100)) && ( chassisData.landingSpeedy < 0) ){
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			}
			else if( (coderData.coderAhead > (coderData.coderBack + 100)) && ( chassisData.landingSpeedy > 0) ){
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			}
	}
		
		else if(turnFlag == 2){
			if( (coderData.coderAhead > (coderData.coderBack + 150)) && ( chassisData.landingSpeedy > 0) ){  //右边轨道
			 chassisData.landingSpeedy = -chassisData.landingSpeedy;
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			 lastTurnFlag = 2;
			 turnFlag = 3;
			}
			else if( (coderData.coderBack > (coderData.coderAhead + 100)) && ( chassisData.landingSpeedy < 0) ){
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			}
			else if( (coderData.coderBack > (coderData.coderAhead + 100)) && ( chassisData.landingSpeedy > 0) ){
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			}
			else if( (coderData.coderAhead > (coderData.coderBack + 100)) && ( chassisData.landingSpeedy < 0) ){
			 coderData.coderAhead = 0;
			 coderData.coderBack = 0;
			}
		}
		
		else if(turnFlag == 3){            //检测到弯道后的缓冲
			checkTime += AUTO_TASK_DURATION;
			if((lastTurnFlag == 1)||(lastTurnFlag == 2)){
				 if(checkTime >= 1.0f){
				 dHead = coderData.coderAhead - lastHead;
				 dBack = coderData.coderBack - lastBack;
				 lastHead = coderData.coderAhead;
				 lastBack = coderData.coderBack;
				 checkTime = 0;
				 if((dHead <= (dBack + 50)) && (dHead >= (dBack - 50))){
//				 if((dBack <= (dHead + 50)) && (dBack >= (dHead - 50))){ 
						coderData.coderAhead = 0;
						coderData.coderBack = 0;
						turnFlag = lastTurnFlag;
					}
				 checkTime = 0;
				}
			}
			else{
				 if(checkTime >= 1.0f){
				 dHead = coderData.coderAhead - lastHead;
				 dBack = coderData.coderBack - lastBack;
				 lastHead = coderData.coderAhead;
				 lastBack = coderData.coderBack;
				 checkTime = 0;
				 if((dHead <= (dBack + 50)) && (dHead >= (dBack - 50))){
						coderData.coderAhead = 0;
						coderData.coderBack = 0;
						turnFlag = lastTurnFlag;
					}
				 checkTime = 0;
				}
			}
		}
	if( (!sentryLeft && ( chassisData.landingSpeedy < 0)) \
		||(!sentryRight && ( chassisData.landingSpeedy > 0)) ){       //黄管检测
			chassisData.landingSpeedy = -chassisData.landingSpeedy;			//不能撞柱子
	  }
	
}

static void communicationWithManifold(void){
	if( !(controlData.loops % 1000) ){														//1000ms就给妙算发一次数据 防止与妙算失去联系.
		visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);
	}
}
uint8_t sentryInit = 0;
bool init = false;
void sentryAutoTaskUpdate(void){
	if(((robotMode == MODE_RC)||(robotMode == MODE_KM))&&(!init)){
		AngleSave = gimbalData.yawAngleSave;
		init = true;
	}
	else if(!((robotMode == MODE_RC)||(robotMode == MODE_KM))){
		init = false;
	}
	if(RC_MODE == RCSW_TOP &&( judgeData.extGameRobotState.remain_HP != 0 )&&init){				//键鼠模式为全自动
		sentryModeUpdate();
		sentryChassisControl();					//检测到柱子时方向运动
		sentryRight = PEin(2);					//获取左光电传感器数据
		sentryLeft  = PEin(4);					//获取右光电传感器数据
              
		switch(sentryAutoData.currentTask){
			case SENTRY_PATROL :				sentryPatrolTaskUpdate();				break;
			case SENTRY_ATTACK :				sentryAttackTaskUpdate();				break;
			case SENTRY_UNTER_ATTACK  :	sentryUnderAttackTaskUpdate();	break;
		}
		communicationWithManifold();		//发送识别信息给妙算
	  escapeControl();                //被打后加速逃跑检测
	  curveCheck(); 			            //弯道检测 		
	}
	else{
		gimbalSwitch(ENABLE);
		chassisSwitch(DISABLE);
		chassisData.landingSpeedy = 0;
		autoDataInit(&sentryAutoData);
		sentryAutoData.schedule = 1;
		sentryAutoData.currentTask = SENTRY_PATROL;
		gimbalData.yawSpeedOut = 0;
		gimbalData.yawAngleOut = 0;
		coderData.coderAhead = 0;
	  coderData.coderBack = 0;
		coderData.xLocation = 0;
		speedFloag = 0;
	}
	if(!sentryInit){//判断是否初始化
			BSP_GPIO_Init(BSP_GPIOE2,GPIO_Mode_IPU);//黄管初始化
			BSP_GPIO_Init(BSP_GPIOE4,GPIO_Mode_IPU);
		  CoderInit();    //编码器初始化
			digitalHi(&sentryInit);
	}
            
}
