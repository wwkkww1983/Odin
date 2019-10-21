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
	auxiliaryAutoData.taskState = EXECUTIONG;								//���Ϊ������
	digitalIncreasing(&auxiliaryAutoData.schedule);					//��ִ�����м�һ
}

/****************************************************/
/********************��Ԯ����************************/
/****************************************************/
void auxiliaryRescueUpdate(void){													//��Ԯ�������
	static u8 lastMouseL;
	if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;											//����д�����������
	}
	
	switch(auxiliaryAutoData.schedule){
		case 1:{
			chassisData.speedLimit = 0.25f;											//���٣����ö�λ
			shiftAngle = 25.0f;
			RESCUE_LOOSEN;
			if(remoteControlData.dt7Value.mouse.Press_L)				//ֻ���һ�����������Ϳ��Է��¿��Ӿ�Ԯ
				digitalIncreasing(&auxiliaryAutoData.schedule);	
		}break;
		case 2:{
			chassisData.speedLimit = 1.0f;											//��ס��ȡ������
			RESCUE_TIGHT;
			if(!lastMouseL && remoteControlData.dt7Value.mouse.Press_L)
				shiftAngle = 0.0f;
			if(remoteControlData.dt7Value.mouse.Press_R)				//��������Ҽ������ص�һ���裬�ر�����
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
/********************�ǵ�����************************/
/****************************************************/
void auxiliaryLandingUpdate(void){													//�ǵ��������
	
}

/****************************************************/
/********************�µ�����************************/
/****************************************************/
void  auxiliaryUnderIslandUpdate(void){

}
/****************************************************/
/******************�ӵ���������***********************/
/***************************************************/
void auxiliaryBulletTransferUpdate(void){									//�ӵ������������
	if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;											//����д�����������
	}
	shiftAngle = 150.0f;																		//���ڲ쿴��λ
	UP_OPEN;																								//����̧��
	switch (auxiliaryAutoData.schedule){
		case 1:{
			chassisData.speedLimit = 0.4f;											//���٣����ö�λ
			SMALLMAGAZINE_CLOSE;																//��С����
			MAGAZINE_CLOSE;																			//�ش󵯲�
			if(remoteControlData.dt7Value.mouse.Press_L)				//ֻ���һ�����������Ϳ��Կ�����
				digitalIncreasing(&auxiliaryAutoData.schedule);
		}break;
		case 2:{
			SMALLMAGAZINE_OPEN;																	//��С����
			MAGAZINE_OPEN;																			//���󵯲�
			if(remoteControlData.dt7Value.mouse.Press_R)				//��������Ҽ������ص�һ���裬�رյ���
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
/****************ҡ�ڶ���ӵ�����*********************/
/***************************************************/
void auxiliaryAviodUpdate(void){														//ҡ�ڶ���ӵ��������
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
		if(auxiliaryAutoData.aviodDuration > R_TIME){					//�����ʱ����6.0s��ʱ�䣬�򽫵�ǰ�������
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
/********************ץȡ����*************************/
/***************************************************/
void auxiliaryGrabUpdate(void){															//ץȡ�������
}

/****************************************************/
/********************�Զ�����*************************/
/***************************************************/
void auxiliaryAutoTaskUpdate(void){
	auxiliaryAviodUpdate();
	if(auxiliaryAutoData.currentTask != AUXILIARY_MANUAL){				//�����п�ִ������
		if(auxiliaryAutoData.taskState == UNEXECUTED){							//�������ոտ�ʼִ��
			auxiliaryTaskBegin();																			//ִ�п�ʼ����
		}
		else if(auxiliaryAutoData.taskState == EXECUTIONG){					//�����ִ����
			switch(auxiliaryAutoData.currentTask){
				case AUXILIARY_RESCUE: {
					auxiliaryRescueUpdate();															//Z ��Ԯ����
					break;
				}
				case AUXILIARY_BULLET_TRANSFER: {
					auxiliaryBulletTransferUpdate();											//V �ӵ���������
					break;
				}
				case AUXILIARY_GRAB: {
					auxiliaryGrabUpdate();																//X ץȡ����
					break;
				}
				default: {																							//��������������ֱ�����³�ʼ���ṹ��
					autoDataInit(&auxiliaryAutoData); 
					break;
				}
			}
		}
		else{
			autoDataInit(&auxiliaryAutoData);														//���ִ����ϻ�ִ�д���������
			
		}
	}
}
