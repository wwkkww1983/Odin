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
int8_t turnFlag = 0;     //ת���־λ
int8_t lastTurnFlag = 0; //��¼��һ��ת���־λ

void sentryTaskBegin(void){
	sentryAutoData.taskState = EXECUTIONG;									//���Ϊ������
	digitalIncreasing(&sentryAutoData.schedule);						//��ִ�����м�һ
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
			if(parameter[YAW_CENTER] > 4096){												//�ж�����λ��
				frontCenterAngle = parameter[YAW_CENTER];							//�ó�ǰ��װ�װ����ĵ�����ֵ
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
		case 2:{																										//ת����1��װ�װ������
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
			/* ��ִ̨��ת�� */
			gimbalData.yawAngleRef += turnRate * yawAdd * coefficient;
		}break;
		case 3:{
			if(judgeData.extGameState.game_progress == 4)
				sentryAngleUpdate();
		}break;
		case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		//���ڱ��У�ֻ��ҡ�˲����г�����Ȩλ��ʱ������������
		default: sentryAutoData.taskState = EXECUTION_ERROR; break;			//��������б���û�н��ȣ����������
	}
}

//void sentryPitchUpdate(void){
//	gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;					//����pitch�ᷴ��			
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
//			if(parameter[YAW_CENTER] > 4096){												//�ж�����λ��
//				frontCenterAngle = parameter[YAW_CENTER];							//�ó�ǰ��װ�װ����ĵ�����ֵ
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
//		case 2:{																										//ת����1��װ�װ������
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
//			/* ��ִ̨��ת�� */
//			gimbalData.yawAngleRef += turnRate * yawAdd * coefficient;
//		}break;
//		case 3:{
//			sentryPitchUpdate();
//			/* 120������ɨ�� */
//			if( gimbalData.yawAngleRef > AngleSave + 60 )
//					coefficient = -coefficient;
//			else
//				if( gimbalData.yawAngleRef < AngleSave - 60 )
//					coefficient = -coefficient;
//			/* ��ִ̨��ת�� */
//			gimbalData.yawAngleRef += 2 * yawAdd * coefficient;
//		}break;
//		case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		//���ڱ��У�ֻ��ҡ�˲����г�����Ȩλ��ʱ������������
//		default: sentryAutoData.taskState = EXECUTION_ERROR; break;			//��������б���û�н��ȣ����������
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
//			gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;					//����pitch�ᷴ��
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
//		case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		//���ڱ��У�ֻ��ҡ�˲����г�����Ȩλ��ʱ������������
//		default: sentryAutoData.taskState = EXECUTION_ERROR; break;			//��������б���û�н��ȣ����������
//	}
//}

/************************************
�����������ϢΪװ�װ��С��Ϣ
armorTypeFbd			<-->	depthFbd
armorTypeLastFbd	<-->	depthLastFbd
************************************/
void sentryAttackTaskUpdate(void){
	static TickType_t sentryFireLastWakeTime = 0;	  //ʱ����
	switch(attackMode){
		//δ����Ŀ��Ѳ��ģʽ
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
//			gimbalData.pitchAngleRef = visionData.pitchCmd;				 //�Զ�ģʽ��ͬ����ǰֵ���Ƕ�����
//			gimbalData.yawAngleRef = visionData.yawCmd;	
//			//���������״̬��̨�Ƕȷ���ȡ�����ǽǶȣ�����ƫ�ã�
//			gimbalData.pitchAngleFbd =  gimbalData.pitchGyroAngle;			
			switch(visionSchedule){
				case 1: gimbalSwitch(DISABLE);												
								chassisSwitch(DISABLE);
								//����TX2����
								visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);							
								digitalIncreasing(&visionSchedule);	
								sentryFireLastWakeTime = xTaskGetTickCount();
								break;
				case 2: gimbalSwitch(ENABLE);	
								shootVisionInit();
								//����ģʽ����
								visionData.prejudgFlag = false;											
								//�����������
								visionFireFbdUpdata(&shootData.fireFlag_17mm);	
								if(visionData.fireFbd){
									if(shootData.fricMotorFlag&&(judgeData.extGameState.game_progress == 4)){
//									if(shootData.fricMotorFlag){
										sentryFireLastWakeTime = xTaskGetTickCount();
										digitalHi(&shootData.fireFlag_17mm);   //���ʹ��
									}
								}	
								if(xTaskGetTickCount() - sentryFireLastWakeTime > 200){
										digitalLo(&shootData.fireFlag_17mm);		//���ʧ��
										digitalLo(&shootData.shootTrigger);	//����ʧ��
								};	//����ʧ��
								break;
//				case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		//���ڱ��У�ֻ��ҡ�˲����г�����Ȩλ��ʱ������������
//				default: sentryAutoData.taskState = EXECUTION_ERROR; break;			//��������б���û�н��ȣ����������
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
			if(parameter[YAW_CENTER] > 4096){												//�ж�����λ��
				frontCenterAngle = parameter[YAW_CENTER];							//�ó�ǰ��װ�װ����ĵ�����ֵ
				backCenterAngle = parameter[YAW_CENTER] - 4096;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > backCenterAngle && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < frontCenterAngle){	//�������ı�����쵽������
					digitalHi(&flagAddEncoderAngle);
					digitalLo(&flagReduceEncoderAngle);
				}
				else{
					digitalLo(&flagAddEncoderAngle);
					digitalHi(&flagReduceEncoderAngle);
				}
			}
			else{
				frontCenterAngle = parameter[YAW_CENTER];							//�ó�ǰ��װ�װ����ĵ�����ֵ
				backCenterAngle = parameter[YAW_CENTER] + 4096;
				if(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) < backCenterAngle && gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData) > frontCenterAngle){	//�������ı�����쵽������
					digitalLo(&flagAddEncoderAngle);
					digitalHi(&flagReduceEncoderAngle);
				}
				else{
					digitalHi(&flagAddEncoderAngle);
					digitalLo(&flagReduceEncoderAngle);
				}
			}
			/* ����Ѫ����װ�װ壨0�ţ������� */
			if( judgeData.extRobotHurt.armor_id == 0x00 &&  judgeData.extRobotHurt.hurt_type == 0x00)											
					sentryAutoData.schedule = 3;
			/* 1��װ�װ屻���� */
			if( judgeData.extRobotHurt.armor_id == 0x01 &&  judgeData.extRobotHurt.hurt_type == 0x00 )									
				sentryAutoData.schedule = 2;
		}break;
		case 2:{																									//ת����������װ�װ������
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
			/* ��ִ̨��ת�� */
			gimbalData.yawAngleRef += turnRate * yawAdd * coefficient;
		}break;
		case 3:{																									//ת����������װ�װ������
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
			/* ��ִ̨��ת�� */
			gimbalData.yawAngleRef +=  turnRate * yawAdd * coefficient;
		}break;
		case 4:{																									//120������ɨ��
			sentryAngleUpdate();
		}break;
		case 99: sentryAutoData.taskState = END_OF_EXECUTION; break;		//���ڱ��У�ֻ��ҡ�˲����г�����Ȩλ��ʱ������������
		default: sentryAutoData.taskState = EXECUTION_ERROR; break;			//��������б���û�н��ȣ����������
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
	if(!captureFlag){	//û��ʶ��Ŀ������������״̬
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
		sentryAutoData.currentTask = SENTRY_ATTACK;		//���Ӿ��������빥��ģʽ
	}
	else if((judgeData.extGameRobotState.remain_HP != lastHP)&&( judgeData.extRobotHurt.hurt_type == 0x00)){
		sentryAutoData.taskDuration = 0;
		sentryAutoData.currentTask = SENTRY_UNTER_ATTACK;	
	}
	else if( (sentryAutoData.taskDuration > 0.8f && sentryAutoData.currentTask == SENTRY_ATTACK) \
				|| (sentryAutoData.taskDuration > 6.0f && sentryAutoData.currentTask == SENTRY_UNTER_ATTACK)
				|| (sentryAutoData.currentTask == SENTRY_MANUAL) )	
		sentryAutoData.currentTask = SENTRY_PATROL;		//�������������������Ѳ��ģʽ
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
//		  if(turnFlag == 0){      //�Զ����
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
			chassisData.landingSpeedy = baseSpeed + (rand() % 2000);         //лһԴ�Ĵ��� 6299 baseSpeed
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
		chassisData.landingSpeedy = -chassisData.landingSpeedy;			//����ײ����
	}
escapeTime += AUTO_TASK_DURATION;

}

//static void flagCheck(void){   //����ͨ�ż��
//static uint8_t lastData = 0x01;

//if(judgeData.extReceiveData.data[0] != lastData){
//	if(judgeData.extReceiveData.data[0] == 0x02){   //����ͨ��
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
			if( (coderData.coderAhead > (coderData.coderBack + 360)) && ( chassisData.landingSpeedy < 0) ){   //�м���
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
			if( (coderData.coderBack > (coderData.coderAhead + 360)) && ( chassisData.landingSpeedy < 0) ){  //��߹��
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
			if( (coderData.coderAhead > (coderData.coderBack + 150)) && ( chassisData.landingSpeedy > 0) ){  //�ұ߹��
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
		
		else if(turnFlag == 3){            //��⵽�����Ļ���
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
		||(!sentryRight && ( chassisData.landingSpeedy > 0)) ){       //�ƹܼ��
			chassisData.landingSpeedy = -chassisData.landingSpeedy;			//����ײ����
	  }
	
}

static void communicationWithManifold(void){
	if( !(controlData.loops % 1000) ){														//1000ms�͸����㷢һ������ ��ֹ������ʧȥ��ϵ.
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
	if(RC_MODE == RCSW_TOP &&( judgeData.extGameRobotState.remain_HP != 0 )&&init){				//����ģʽΪȫ�Զ�
		sentryModeUpdate();
		sentryChassisControl();					//��⵽����ʱ�����˶�
		sentryRight = PEin(2);					//��ȡ���紫��������
		sentryLeft  = PEin(4);					//��ȡ�ҹ�紫��������
              
		switch(sentryAutoData.currentTask){
			case SENTRY_PATROL :				sentryPatrolTaskUpdate();				break;
			case SENTRY_ATTACK :				sentryAttackTaskUpdate();				break;
			case SENTRY_UNTER_ATTACK  :	sentryUnderAttackTaskUpdate();	break;
		}
		communicationWithManifold();		//����ʶ����Ϣ������
	  escapeControl();                //�����������ܼ��
	  curveCheck(); 			            //������ 		
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
	if(!sentryInit){//�ж��Ƿ��ʼ��
			BSP_GPIO_Init(BSP_GPIOE2,GPIO_Mode_IPU);//�ƹܳ�ʼ��
			BSP_GPIO_Init(BSP_GPIOE4,GPIO_Mode_IPU);
		  CoderInit();    //��������ʼ��
			digitalHi(&sentryInit);
	}
            
}
