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
	tankAutoData.taskState = EXECUTIONG;								//���Ϊ������
	digitalIncreasing(&tankAutoData.schedule);					//��ִ�����м�һ
}

/* ����ģʽң�������� */
void tankRCUpdate(void){
	if(RC_ROTATE > 400 || RC_ROTATE < -400){
		if(PRESS_Q || RC_ROTATE > 400){
			digitalHi(&P_HERO_42_LID);							//�����󵯲�
			digitalHi(&P_HERO_17_LID);							//����С����
		}
		else if(PRESS_E || RC_ROTATE < -400){
			digitalLo(&P_HERO_42_LID);							//�رմ󵯲�
			digitalLo(&P_HERO_17_LID);							//�ر�С����
			digitalHi(&shootData.ballisticFill);							//��ʼ�������
		}
	}
}

/* �������&����������� */
void tankAutomaticAimUpdate(void){										
	switch(tankAutoData.schedule){											//��schedule�����ӡ����ٺ��ж��������
		case 1: gimbalSwitch(DISABLE);												
						chassisSwitch(DISABLE);
						shootVisionInit();
						//����TX2����
						visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,BIG_BULLET);							
						digitalIncreasing(&(tankAutoData.schedule));	
						break;
		case 2: gimbalSwitch(ENABLE);	
						//����ģʽ����
						if(KB_PJEJUDGMENT){												//�Ҽ���Ԥ�д��
							visionData.prejudgFlag = true;						
							visionData.manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE * 0.5f;
							visionData.manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
						}
						else{																			//��� ����Ԥ�д��
							visionData.prejudgFlag = false;											
							digitalClan(&visionData.manualPitchBias);
							digitalClan(&visionData.manualYawBias);
						}
						//�����������
						if(!visionData.prejudgFlag){
						shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_42mm);
						}
						else{
							shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_42mm);
						}		
						if(tankAutoData.breakSign){
							tankAutoData.schedule = 99;							//����д�����������
							visionData.prejudgFlag = false;	
							visionData.fireFbd = false;
							shootDataReset();	
						}			
						break;
		case 99: tankAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						  //ֻ�е���99ʱ�����˳�
		default: tankAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							//��������б���û�н��ȣ����������
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* 42mm��ҩ����������� */
void tankBulletTransferUpdate(void){
	if(tankAutoData.breakSign){													//����д�ϣ��жϣ�
		tankAutoData.schedule = 3;												//ֱ���������һ��
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_X && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 1.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 1;
	}
	switch(tankAutoData.schedule){											//��schedule�����ӡ����ٺ��ж��������
		case 1: 
					 if(tankAutoData.taskDuration < 1.0f){
						 digitalHi(&P_HERO_42_LID);      //�����󵯲�
						 digitalHi(&P_HERO_TV);   		   //����С��Ļ
						 gimbalData.yawAngleStopSet = -80.0f;
						 gimbalData.pitchAngleStopSet = -30.0f;
						 gimbalStopSwitch(ENABLE);
					   chassisData.speedLimit = 0.5f;								//���ٹ��̺�����ٶȶ�������ԭ����50%
					 }
					 else{
						 digitalIncreasing(&tankAutoData.schedule);
					 }
					 break;
		case 2: 
					 if(tankAutoData.taskDuration > 30.0f){			//30������ɲ���
						 digitalIncreasing(&tankAutoData.schedule);
					 }
					 break;
		case 3:
					 digitalLo(&P_HERO_42_LID);      //�رմ󵯲�
					 digitalLo(&P_HERO_17_LID);      //�ر�С����
					 digitalLo(&P_HERO_TV);   		   //����С��Ļ
					 chassisData.speedLimit = 1.0f;								  //������ԭ����ֵ
					 digitalHi(&shootData.ballisticFill);							//��ʼ�������
					 gimbalStopSwitch(DISABLE);
					 tankAutoData.schedule = 99;
					 break;
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						//ֻ�е���99ʱ�����˳�
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							//��������б���û�н��ȣ����������
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* �Զ���ͷ42mm��ҩ����������� */
void tankBulletAroundUpdate(void){
	if(tankAutoData.breakSign){													//����д�ϣ��жϣ�
		tankAutoData.schedule = 5;												//ֱ���������һ��
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_X && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 3.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 3;
	}
	switch(tankAutoData.schedule){											//��schedule�����ӡ����ٺ��ж��������
		case 1:{
				autoTaskData->fastSeed = 1;									//������
				gimbalData.yawAngleRef -= 180.0f;						//������ת180��
				digitalIncreasing(&tankAutoData.schedule);
		} break;
		case 2:{
			if(tankAutoData.taskDuration > 0.5f){
				if(tankAutoData.taskDuration < 3.0f){			//3������ɵ�ͷ
					if((gimbalData.yawAngleFbd > gimbalData.yawAngleRef - 5.0f && gimbalData.yawAngleFbd < gimbalData.yawAngleRef + 5.0f)\
						&& (chassisData.chaseFbd > -5.0f && chassisData.chaseFbd < 5.0f)){    //��ͷ���
						autoTaskData->fastSeed = 0;
						digitalIncreasing(&tankAutoData.schedule);
					}
				}
				else
					digitalIncreasing(&tankAutoData.schedule);
			}
		}		break;
		case 3:{
						 digitalHi(&P_HERO_42_LID);      //�����󵯲�
						 digitalHi(&P_HERO_TV);   		   //����С��Ļ
						 gimbalData.yawAngleStopSet = -80.0f;
						 gimbalData.pitchAngleStopSet = -30.0f;
						 gimbalStopSwitch(ENABLE);
					   chassisData.speedLimit = 0.5f;								//���ٹ��̺�����ٶȶ�������ԭ����50%
						 digitalIncreasing(&tankAutoData.schedule);
					 } break;
		case 4: 
					 if(tankAutoData.taskDuration > 30.0f){			//20������ɲ���
						 digitalIncreasing(&tankAutoData.schedule);
					 }
					 break;
		case 5:{
						 digitalLo(&P_HERO_42_LID);      //�رմ󵯲�
						 digitalLo(&P_HERO_17_LID);      //�ر�С����
						 digitalLo(&P_HERO_TV);   		   //����С��Ļ
						 chassisData.speedLimit = 1.0f;								  //������ԭ����ֵ
						 digitalHi(&shootData.ballisticFill);							//��ʼ�������
						 gimbalData.yawAngleRef = gimbalData.yawAngleFbd;					//����ֹͣ��ת
						 autoTaskData->fastSeed = 0;
						 gimbalStopSwitch(DISABLE);
						 tankAutoData.schedule = 99;
					 } break;
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						//ֻ�е���99ʱ�����˳�
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							//��������б���û�н��ȣ����������
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}


/* 17mm��ҩ����վ����������� */
void tankBulletSupplyUpdate(void){
	if(tankAutoData.breakSign){													//����д�ϣ��жϣ�
		tankAutoData.schedule = 3;												//ֱ���������һ��
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_R && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 1.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 1;
	}
	switch(tankAutoData.schedule){											//��schedule�����ӡ����ٺ��ж��������
		case 1: 
					 if(tankAutoData.taskDuration < 1.0f){
						 digitalHi(&P_HERO_17_LID);      //����С����
					   chassisData.speedLimit = 0.5f;								//���ٹ��̺�����ٶȶ�������ԭ����50%
					 }
					 else{
						 digitalIncreasing(&tankAutoData.schedule);
					 }
					 break;
		case 2: 
					 if(tankAutoData.taskDuration > 30.0f){			//30������ɲ���
						 digitalIncreasing(&tankAutoData.schedule);
					 }
					 break;
		case 3:
					 digitalLo(&P_HERO_42_LID);      //�رմ󵯲�
					 digitalLo(&P_HERO_17_LID);      //�ر�С����
					 chassisData.speedLimit = 1.0f;								  //������ԭ����ֵ
					 tankAutoData.schedule = 99;
					 break;
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						//ֻ�е���99ʱ�����˳�
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							//��������б���û�н��ȣ����������
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* ҡ�ڶ���ӵ�������� */
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
		if(tankAutoData.aviodDuration > R_TIME){					//�����ʱ����6.0s��ʱ�䣬�򽫵�ǰ�������
			chassisData.chaseRef = 0.0f;
			tankAutoData.avoidTask = DISABLE;
		}	
		tankAutoData.aviodDuration += AUTO_TASK_DURATION;	
	}
	else{
		chassisData.chaseRef = 0.0f;
	}
}

/* ���ٵ�ͷ������� */
void tankTurnAroundUpdate(void){
	if(tankAutoData.breakSign){													//����д�ϣ��жϣ�
		tankAutoData.schedule = 3;												//ֱ���������һ��
		digitalLo(&tankAutoData.breakSign);
	}
	switch(tankAutoData.schedule){											//��schedule�����ӡ����ٺ��ж��������
		case 1:{
				autoTaskData->fastSeed = 1;									//������
				gimbalData.yawAngleRef -= 180.0f;						//������ת180��
				digitalIncreasing(&tankAutoData.schedule);
		} break;
		case 2:{
			if(tankAutoData.taskDuration > 0.5f){
				if(tankAutoData.taskDuration < 3.0f){			//3������ɵ�ͷ
					if(gimbalData.yawAngleFbd > gimbalData.yawAngleRef - 5.0f || gimbalData.yawAngleFbd < gimbalData.yawAngleRef + 5.0f){    //��ͷ���
						autoTaskData->fastSeed = 0;
						tankAutoData.schedule = 99;
					}
				}
				else
					digitalIncreasing(&tankAutoData.schedule);
			}
		}		break;
		case 3:
				gimbalData.yawAngleRef = gimbalData.yawAngleFbd;					//����ֹͣ��ת
				autoTaskData->fastSeed = 0;
				tankAutoData.schedule = 99;
				break;
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						//ֻ�е���99ʱ�����˳�
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							//��������б���û�н��ȣ����������
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* �Ȼ��ڸ��� */
void tankMortarUpdate(void){
	if(tankAutoData.breakSign){													//����д�ϣ��жϣ�
		tankAutoData.schedule = 4;												//ֱ���������һ��
		digitalLo(&tankAutoData.breakSign);
	}
	if(TASK_PRESS_CTRL && (tankAutoData.taskDuration < 30.0f && tankAutoData.taskDuration > 1.1f)){
		tankAutoData.taskDuration = 0.0f;
		tankAutoData.schedule = 3;
	}
	switch(tankAutoData.schedule){											//��schedule�����ӡ����ٺ��ж��������
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
					 if(tankAutoData.taskDuration < 30.0f){			//�Ȼ���ģʽά��20s
						 visionMortar();
						 if(KB_PJEJUDGMENT){												//�Ҽ���Ԥ�д��					
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
					 
		case 99: tankAutoData.taskState = END_OF_EXECUTION; break;						//ֻ�е���99ʱ�����˳�
		default: tankAutoData.taskState = EXECUTION_ERROR; break;							//��������б���û�н��ȣ����������
	}
	tankAutoData.taskDuration += AUTO_TASK_DURATION;
}

/* ��ɱ����ģʽ���� */
void tankSuicideFireUpdate(void){
	if(PRESS_C){
		digitalHi(&shootData.suicideFireFlag);
	}
	else{
		digitalLo(&shootData.suicideFireFlag);
	}
}

void tankAutoTaskUpdate(void){
	tankAviodUpdate();																	//������񣨺Ͳ������������ͬ��
	tankSuicideFireUpdate();														//��ɱ����ģʽ����
	tankRCUpdate();                                     //ң��������
	if(tankAutoData.currentTask != TANK_MANUAL){										//�����п�ִ������  TANK_MANUAL���ֶ�)
		if(tankAutoData.taskState == UNEXECUTED){										//�������ոտ�ʼִ��
			tankTaskBegin();																//ִ�п�ʼ����
		}
		else if(tankAutoData.taskState == EXECUTIONG){		//�����ִ����
			switch(tankAutoData.currentTask){
				case TANK_AUTOMATIC_AIM: {										//������׼
					tankAutomaticAimUpdate();
					break;
				}
				case TANK_TURN_AROUND: {											//���ٵ�ͷ
					tankTurnAroundUpdate();
					break;
				}
				case TANK_MORTAR: {														//�Ȼ���ģʽ
					tankMortarUpdate();
					break;
				}
				case TANK_BULLET_TRANSFER: {									//42mm��ҩ����
//					tankBulletTransferUpdate();
					tankBulletAroundUpdate();
					break;
				}
				case TANK_BULLET_SUPPLY: {									   //17mm��ҩ����վ����
					tankBulletSupplyUpdate();
					break;
				}
				default: {																				//��������������ֱ�����³�ʼ���ṹ��
					autoDataInit(&tankAutoData); 										//����������
					break;
				}
			}
		}
		else{																									//���ִ����ϻ�ִ�д���������
			autoDataInit(&tankAutoData);
		}
	}
}
