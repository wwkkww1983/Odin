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

void gimbalSwitch(uint8_t active){																				//��̨�����
	if(active){
		gimbalData.autoMode = ENABLE;
	}
	else{
		gimbalData.autoMode = DISABLE;
	}	
}

void chassisSwitch(uint8_t active){																				//���̼����
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

void shootSwitch(uint8_t active){																					//������������
	if(active){
		shootData.autoMode = ENABLE;
	}
	else{
		shootData.autoMode = DISABLE;
	}	
}

void deformingSwitch(uint8_t active){																			//���μ������
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
		if(RC_MODE != RCSW_TOP){																							//�����ڼ���ģʽ���ڱ��²�������״̬��ɨ��
			return false;																												//���ؼ�
		}
	}
	/* ---- ��ǰ�������������� ---- */
	if(autoTaskData->currentTask != 0 || autoTaskData->avoidTask){																																				
		if(TASK_PRESS_F){																											//�������F��������ֹ��ǰ���񣬲��Ե�ǰ�������ݽ�������
			digitalHi(&autoTaskData->breakSign);
			autoTaskData->avoidTask = DISABLE;
			autoTaskData->aviodFlag = false;
			digitalLo(&infantryAutoData.avoidSchedule);
		}
//		if(autoTaskData->currentTask == CTRL_TASK){															
//			if(!TASK_PRESS_CTRL)																							//���CTRLû�г�������,����ֹ��ǰ���񣬲��Ե�ǰ�������ݽ�������
//				autoDataInit(autoTaskData);
//		}
		if(autoTaskData->currentTask == G_TASK){
			if(!TASK_PRESS_G){																									//���Gû�г�������,����ֹ��ǰ���񣬲��Ե�ǰ�������ݽ�������
				static uint16_t time;
				time ++ ;//��ʼ��ʱ
				if(time > 25){//50ms�ɿ�����
					time = 0;//��һ���ɿ���ʱ
					autoDataInit(autoTaskData);
				}
			}
		}
		if(autoTaskData->currentTask == Z_TASK){
			if(!TASK_PRESS_Z){																									//���Zû�г�������,����ֹ��ǰ���񣬲��Ե�ǰ�������ݽ�������
				static uint16_t time;
				time ++ ;//��ʼ��ʱ
				if(time > 25){//50ms�ɿ�����
					time = 0;//��һ���ɿ���ʱ
					autoDataInit(autoTaskData);
				}
			}
		}
		if(autoTaskData->currentTask == V_TASK){
			if(!TASK_PRESS_V){																									//���Vû�г�������,����ֹ��ǰ���񣬲��Ե�ǰ�������ݽ�������
				static uint16_t time;
				time ++ ;//��ʼ��ʱ
				if(time > 25){//50ms�ɿ�����
					time = 0;//��һ���ɿ���ʱ
					autoDataInit(autoTaskData);
				}
			}
		}
	}
	if(autoTaskData->currentTask != CTRL_TASK){
		/* --- ��ǰ��CTRL������ --- */
		if(TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){							  //ֻ����X�������
			autoTaskData->currentTask = X_TASK;
		}
		else if(!TASK_PRESS_X && TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){						//ֻ����R�������
			autoTaskData->currentTask = R_TASK;
		}	
		else if(!TASK_PRESS_X && !TASK_PRESS_R && TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){						//ֻ����CTRL�������
			autoTaskData->currentTask = CTRL_TASK;
		}
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && TASK_PRESS_Z){						//ֻ����Z�������
			autoTaskData->currentTask = Z_TASK;
		}
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){						//ֻ����V�������
			autoTaskData->currentTask = V_TASK;
		}
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && !TASK_PRESS_G && !TASK_PRESS_V && TASK_PRESS_Q && !TASK_PRESS_Z){						//ֻ����Q�������
			autoTaskData->currentTask = Q_TASK;
		}
		else if(!TASK_PRESS_X && !TASK_PRESS_R && !TASK_PRESS_CTRL && TASK_PRESS_G && !TASK_PRESS_V && !TASK_PRESS_Q && !TASK_PRESS_Z){						//ֻ����G�������
			autoTaskData->currentTask = G_TASK;
		}
	}
	if(TASK_PRESS_E){																																												//����EŤ��
		autoTaskData->avoidTask = ENABLE;
		autoTaskData->aviodFlag = true;
		autoTaskData->aviodDuration = 0.0f;
	}
	//����ֱ������
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

void autoTaskUpdate(void){																								//�Զ��������
	if(keyboradScanForTask()){																							//��������ڱ�ģʽ�£�SW2�ֲ��ڼ���λ�����޷�ִ���κ��Զ�����
		switch(robotConfigData.typeOfRobot){
			case INFANTRY_ID:	
				infantryAutoTaskUpdate(); 
				break;																	//��������
			case TANK_ID : 
					tankAutoTaskUpdate(); 
				break;																	//Ӣ������
			case AUXILIARY_ID: 
				auxiliaryAutoTaskUpdate(); 
				break;																	//���̳�����
			case SENTRY_ID: 
				sentryAutoTaskUpdate(); 
				break;																	//�ڱ�����
			case UAV_ID: 
				uavAutoTaskUpdate(); 
				break;																	//���˻�����
		}
	}
	else{
		autoDataInit(autoTaskData);																						//��ֹ��ǰ���񲢶Ե�ǰ�������ݽ�������
	}
}

void autoDataInit(AutoTaskStruct_t *autoContrlData){											//���Զ�����Ľṹ�����ݽ�������
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
	switch(robotConfigData.typeOfRobot){																		//ͨ��tf��д����������
		case INFANTRY_ID:	
			autoTaskData = &infantryAutoData;
			break;																															//��������
		case TANK_ID : 
			autoTaskData = &tankAutoData;
			break;																															//Ӣ������
		case AUXILIARY_ID: 
			autoTaskData = &auxiliaryAutoData;
			break;																															//���̳�����
		case SENTRY_ID: 
			autoTaskData = &sentryAutoData;
			break;																															//�ڱ�����
		case UAV_ID: 
			autoTaskData = &uavAutoData; 
			break;																															//���˻�����
	}
	autoDataInit(autoTaskData);																							//��ʼ��������������
}
