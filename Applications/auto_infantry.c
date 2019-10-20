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
	infantryAutoData.taskState = EXECUTIONG;										//���Ϊ������
	digitalIncreasing(&infantryAutoData.schedule);							//��ִ�����м�һ
}

void infantryAutomaticAimUpdate(void){												//�������/�����������
	switch(infantryAutoData.schedule){													//��schedule�����ӡ����ٺ��ж��������
		case 1: gimbalSwitch(DISABLE);												
						chassisSwitch(DISABLE);
						//����TX2����
						visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);							
						digitalIncreasing(&(infantryAutoData.schedule));	
						break;
		case 2: gimbalSwitch(ENABLE);	
						shootVisionInit();
						//����ģʽ����
						if(KB_PJEJUDGMENT){																//�Ҽ� ��Ԥ�д��
							visionData.prejudgFlag = true;	
							visionData.manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE * 0.5f;
							visionData.manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
						}
						else{																							//��� ����Ԥ�д��
							visionData.prejudgFlag = false;											
							digitalClan(&visionData.manualPitchBias);
							digitalClan(&visionData.manualYawBias);
						}
						//�����������
						if(!visionData.prejudgFlag){
							shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_17mm);
						}
						else{
							shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_17mm);	
						}										
						if(infantryAutoData.breakSign){
							infantryAutoData.schedule = 99;									//����д�����������
							visionData.prejudgFlag = false;
							visionData.fireFbd = false;
							shootDataReset();	
						}			
						break;
		case 99: infantryAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						  //ֻ�е���99ʱ�����˳�
		default: infantryAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							//��������б���û�н��ȣ����������
	}
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;
}

void infantryAutomaticBuffUpdate(void){										//�������/�����������
	static shootMode_e originalShootMode = MANUAL_SINGLE;
	static bool wKey = 0;
	static bool sKey = 0;
	switch(infantryAutoData.schedule){													//��schedule�����ӡ����ٺ��ж��������
		case 1: gimbalSwitch(DISABLE);
						chassisSwitch(DISABLE);
						originalShootMode = shootData.shootMode;
						//����TX2����
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
						//����ģʽ����
						if(KB_NO_PJEJUDGMENT){														//���
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
							infantryAutoData.schedule = 99;									//����д�����������
							visionData.prejudgFlag = false;
							visionData.fireFbd = false;
							shootData.shootMode = originalShootMode;
							shootDataReset();	
						}			
						break;
		case 99: infantryAutoData.taskState = END_OF_EXECUTION;gimbalSwitch(DISABLE);break;						  //ֻ�е���99ʱ�����˳�
		default: infantryAutoData.taskState = EXECUTION_ERROR;gimbalSwitch(DISABLE);break;							//��������б���û�н��ȣ����������
	}
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;
}

void infantryBulletTransferUpdate(void){									//�ӵ�����������£�������
	if(infantryAutoData.breakSign){													//����д�ϣ��жϣ�
		supplyData.supplySpeedRef = 2000;										//��ת�����ٶ�����ֵ
		infantryAutoData.taskDuration = 62.1f;
		infantryAutoData.schedule = 3;												//ֱ���������һ��
		digitalLo(&infantryAutoData.breakSign) ;
	}
	if(TASK_PRESS_R && (infantryAutoData.taskDuration < 60.0f && infantryAutoData.taskDuration > 2.1f)){
		infantryAutoData.taskDuration = 0.0f;
		infantryAutoData.schedule = 1;
	}
	switch(infantryAutoData.schedule){											//��schedule�����ӡ����ٺ��ж��������
		case 1: 
					 if(infantryAutoData.taskDuration < 1.0f){
						 supplyData.supplySpeedRef = -2000;
					   chassisData.speedLimit = 0.4f;								//���ٹ��̺�����ٶȶ�������ԭ����40%
					 }
					 else{
						 supplyData.supplySpeedRef = -80;
						 digitalIncreasing(&infantryAutoData.schedule);
					 }
					 break;
		case 2: 
					 if(infantryAutoData.taskDuration > 62.0f){			//����ĳ���30��
						 supplyData.supplySpeedRef = 2000;
						 digitalIncreasing(&infantryAutoData.schedule);
					 }
					 break;
		case 3:
					 chassisData.speedLimit = 1.0f;								  //������ԭ����ֵ
					 if(infantryAutoData.taskDuration > 63.0f){			//2����ʱ��ر�
						 supplyData.supplySpeedRef = 80;
						 infantryAutoData.schedule = 99;
					 }
					 break;
		case 99: infantryAutoData.taskState = END_OF_EXECUTION; break;						//ֻ�е���99ʱ�����˳�
		default: infantryAutoData.taskState = EXECUTION_ERROR; break;							//��������б���û�н��ȣ����������
	}
	infantryAutoData.taskDuration += AUTO_TASK_DURATION;	
}

void infantryAviodUpdate(void){													 
	static uint8_t avoidTurn = 0;

#if	USE_CHANGE_HEAD 
	//Ť����С���ݼӵ�ͷ�ӵ�����
	static uint8_t releaseFlag = 0;
	static uint8_t pressInitFlag = 0;
	static uint8_t pressExchangeFlag = 0;
	if(infantryAutoData.avoidTask && !chassisData.changeChassisSchedule && !chassisData.changeHeadSchedule){ 			//ҡ�ڶ���ӵ��������		��45�Ƚ���Ե���		
		 switch(infantryAutoData.avoidSchedule){
			case 0:
				if(chassisData.chaseRef < AVIOD_INITIAL_ANGEL)				// ������45�Ƚ�ӭ��
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
		if(infantryAutoData.aviodDuration > R_TIME){							 //�����ʱ����6.0s��ʱ�䣬�򽫵�ǰ�������
			chassisData.chaseRef = 0.0f;
			digitalClan(&infantryAutoData.avoidSchedule);							 //��Ť���������ܱ�֤�´�Ť����45�Ƚ�ӭ��
			infantryAutoData.avoidTask = DISABLE;		
		}	
		infantryAutoData.aviodDuration += AUTO_TASK_DURATION;	
	}
	else{																										     //С������ת���
		if(PRESS_Q || PRESS_E || RC_ROTATE > 100 || RC_ROTATE < -100 ){
			if(PRESS_Q || RC_ROTATE > 100){
				chassisData.chaseRef += (AVOID_RATE * 4);
			}
			if(PRESS_E || RC_ROTATE < -100){
				chassisData.chaseRef -= (AVOID_RATE * 4);
			}
		}
		else{																																				//����ɾ�� ����涨�����Ե���������Ϊǰ������
			if(!chassisData.changeChassisSchedule){																		//����û�н��е�ͷ
				switch(chassisData.changeHeadSchedule){																			
					case 0:
						if(RC_TRANSVERSE > 490 && RC_LONGITUDINAL > 490){                   //����ͷָ��
							digitalIncreasing(&chassisData.changeHeadSchedule);
						}	
					break;				
					case 1:
						chassisData.changeHeadOrder = 1.0f;
						gimbalData.yawAngleRef +=180;
						digitalIncreasing(&chassisData.changeHeadSchedule);
						break;
					case 2:
						if(ABS(RC_TRANSVERSE) < 10 && ABS(RC_LONGITUDINAL) < 10){   							 //���ּ��
							releaseFlag = 1;
						}
						if(( ABS(IMU_RATEZ) < 0.008f ) && ( ABS(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData)-parameter[YAW_CENTER]) > 2000 )){
							parameter[YAW_CENTER] =  (parameter[YAW_CENTER] + 4096 ) > 8192?  (parameter[YAW_CENTER] - 4096) : (parameter[YAW_CENTER] + 4096);
							gimbalData.yawMotorAngle = ENCODER_ANGLE_RATIO * getRelativePos(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData),parameter[YAW_CENTER]);
							chassisData.changeHeadOrder = 0.0f;           //��ʼ����
							chassisData.direction = -chassisData.direction; 
							digitalIncreasing(&chassisData.changeHeadSchedule);
						}
						break;
					case 3: 
						if(releaseFlag){																													  //��ͷ���֮ǰ���ּ��
							releaseFlag = 0;
							digitalClan(&chassisData.changeHeadSchedule);						
						}
						if(ABS(RC_TRANSVERSE) < 10 && ABS(RC_LONGITUDINAL) < 10){   							 //��ͷ���֮�����ּ��
							digitalClan(&chassisData.changeHeadSchedule);														 //���Խ�����һ�ε�ͷ
						}
						break;
					default:
						chassisData.chaseRef = 0.0f;
						break;
				}
			}
			if(!chassisData.changeHeadSchedule){																			 			 	//��̨û�н��е�ͷ
				switch(chassisData.changeChassisSchedule){
					case 0:
						if(RC_TRANSVERSE < -490 && RC_LONGITUDINAL < -490){													//ң��������ͷָ�����ͷָ��
							pressExchangeFlag = 2;
							digitalIncreasing(&chassisData.changeChassisSchedule);
						}
						else
							if(TASK_PRESS_CTRL && (TASK_PRESS_X ^ TASK_PRESS_Z)){											//���̸���ͷָ��
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
						switch(pressExchangeFlag){																			//���ǰ���ּ��
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
					  if(ABS(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData)-parameter[YAW_CENTER]) > 4000){					//�����������жϵ�ͷ���
							parameter[YAW_CENTER] =  (parameter[YAW_CENTER] + 4096 ) > 8192?  (parameter[YAW_CENTER] - 4096) : (parameter[YAW_CENTER] + 4096);
							gimbalData.yawMotorAngle = ENCODER_ANGLE_RATIO * getRelativePos(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData),parameter[YAW_CENTER]);
							chassisData.chaseRef = 0.0f;
							chassisData.direction = -chassisData.direction;
							digitalIncreasing(&chassisData.changeChassisSchedule);
						}
						break;
					case 3:
						if(releaseFlag){																													 //��ͷ���֮ǰ���ּ��
							releaseFlag = 0;
							digitalClan(&chassisData.changeChassisSchedule);						
						}
						switch(pressExchangeFlag){                                                 //��ɺ����ּ��
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
								if(ABS(RC_TRANSVERSE) < 10 && ABS(RC_LONGITUDINAL) < 10){									 //��ͷ���֮�����ּ��
									digitalClan(&pressInitFlag);											
									digitalClan(&chassisData.changeChassisSchedule);												 //���Խ����´ε��̵�ͷ
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
	//Ť����С����
	if(infantryAutoData.avoidTask){
		switch(infantryAutoData.avoidSchedule){
			case 0:
				if(chassisData.chaseRef < AVIOD_INITIAL_ANGEL)				// ������45�Ƚ�ӭ��
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
		if(infantryAutoData.aviodDuration > R_TIME){							 //�����ʱ����6.0s��ʱ�䣬�򽫵�ǰ�������
			chassisData.chaseRef = 0.0f;
			infantryAutoData.aviodFlag = false;
			digitalClan(&infantryAutoData.avoidSchedule);							 //��Ť���������ܱ�֤�´�Ť����45�Ƚ�ӭ��
			infantryAutoData.avoidTask = DISABLE;		
		}	
		infantryAutoData.aviodDuration += AUTO_TASK_DURATION;		
	}
	else{	//С������ת���
	static uint8_t lastKeySate,kmRotateFlag,rcRotateFlag,rotateDirection;		
		if(RC_ROTATE > 100)
				rcRotateFlag = 1;
			else
				rcRotateFlag = 0;
			//һ��������ת
			if(!lastKeySate && !kmRotateFlag){																	
				if(PRESS_Q){
					kmRotateFlag = 1;
					//��һ����ת�ı�ת��
					rotateDirection = !rotateDirection;
				}
			}
			else{
				//�ٴΰ��½����ת
				if(!lastKeySate && kmRotateFlag){
					if(PRESS_Q || TASK_PRESS_F)
						kmRotateFlag = 0;
				}
				//�����Զ����С����
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
		infantryAviodUpdate(); //�������  ��INFANTRY_MANUAL���ֶ�ģʽ
	}
	else{
		//���ʱ�������
		chassisData.chaseRef = 0.0f;
		infantryAutoData.aviodFlag = false;
		digitalClan(&infantryAutoData.avoidSchedule);				
		infantryAutoData.avoidTask = DISABLE;		
	}
																		
	if(infantryAutoData.currentTask != INFANTRY_MANUAL){		//�����п�ִ������
		if(infantryAutoData.currentTask != INFANTRY_BULLET_TRANSFER \
			&& infantryAutoData.lastTask == INFANTRY_BULLET_TRANSFER){
			supplyData.supplySpeedRef = 80;
		}
		if(infantryAutoData.taskState == UNEXECUTED){					//�������ոտ�ʼִ��   δִ�У�UNEXECUTED��
			infantryTaskBegin();																//ִ�п�ʼ����
		}
		else if(infantryAutoData.taskState == EXECUTIONG){		//�����ִ����
			switch(infantryAutoData.currentTask){								//��ǰ����
				case INFANTRY_AUTOMATIC_AIM:  {										//������׼
					infantryAutomaticAimUpdate();
					break;
				}
				case INFANTRY_ACTIVE_BUFF: {											//������
					infantryAutomaticBuffUpdate();
					break;
				}
				case INFANTRY_BULLET_TRANSFER: {									//�ӵ�����
					infantryBulletTransferUpdate();
					break;
				}
				default: {																				//��������������ֱ�����³�ʼ���ṹ��
					autoDataInit(&infantryAutoData); 
					break;
				}
			}
		}
		else{																									//���ִ����ϻ�ִ�д��������³�ʼ���ṹ��
			autoDataInit(&infantryAutoData);
		}
	}
	//������һ�̵�����
	infantryAutoData.lastTask = infantryAutoData.currentTask;
}

