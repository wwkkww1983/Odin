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
	if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;											//����д�����������
	}
	
	switch (auxiliaryAutoData.schedule){
		case 1:{

		}break;
		case 99: {
			auxiliaryAutoData.taskState = END_OF_EXECUTION;	             //ֻ�е���99ʱ�����˳�
			break;						
		}
		default: auxiliaryAutoData.taskState = EXECUTION_ERROR;break;
	}
	auxiliaryAutoData.taskDuration += AUTO_TASK_DURATION;
}

/****************************************************/
/********************�µ�����************************/
/****************************************************/
void  auxiliaryUnderIslandUpdate(void){
	if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;											//����д�����������
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
	static float speed = 500 * REAL_MOTOR_SPEED_SCALE;
	static u8 executeNum = 0,lastPressX = 0,deathMode = 0;
	if(auxiliaryAutoData.breakSign){
		auxiliaryAutoData.schedule = 99;												//����д�����������
	}		
	
	/* ���ƶ��� */
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
	
	/* ������� */
	switch (auxiliaryAutoData.schedule){
		case 1:{
			chassisSwitch(ENABLE);
			shiftAngle = 45.0f;
			UP_OPEN;
			digitalIncreasing(&auxiliaryAutoData.schedule);			//����̧�ߡ��ӽ�ǰ������һ��	
		}break;
		
    case 2:{																							//ѡ��ִ��ģʽ
			if(remoteControlData.dt7Value.mouse.Press_L){
				if(!PRESS_C)
					auxiliaryAutoData.schedule = 3;									//�Զ�����ǰ��
				else
					auxiliaryAutoData.schedule = 4;									//ǿ�Ƶ���ǰ��
			}
			else if(remoteControlData.dt7Value.mouse.Press_R)
				if(!PRESS_C)
					auxiliaryAutoData.schedule = 12;								//�Զ����º���
				else
					auxiliaryAutoData.schedule = 13;								//ǿ�Ƶ��º���
			else if(!lastPressX && TASK_PRESS_X)
				auxiliaryAutoData.schedule = 14;									//����
			else if(PRESS_E){
				auxiliaryAutoData.schedule = 3;										//����ģʽ
				deathMode = 1;
			}
		}break;
		
		/* ����ǰ��ץ�� */	
		case 3:{
			chassisData.autoSpeedTarget.x = speed;							//���ƶ�λ
			if(/*(ONE_LEFT == 1) ||*/ (ONE_RIGHT == 1)){
				digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
				if(auxiliaryAutoData.countTimes > 5){							//������⵽10ms
					auxiliaryAutoData.countTimes = 0;								//ʱ������
					digitalIncreasing(&auxiliaryAutoData.schedule);	//����һ��	
				}					
			}
			else
				digitalClan(&auxiliaryAutoData.countTimes);
		}break;	
		
		case 4:{
			PAWGO_BACK;																					//צ�Ӻ���ץ��һ��
			digitalIncreasing(&auxiliaryAutoData.schedule);			//����һ��
		}break;
		
		/* ��һ��ץ��ץ�м� */
		case 5:{
			chassisData.autoSpeedTarget.x = 0;								//����ֹͣ
			PAW_OPEN;	     																		//צ�Ӵ�
      PAWGORIGHT_CLOSE;  																//צ�����ƹ�	
	    PAWGOLEFT_CLOSE;    		                          //צ�����ƹ�
			SMALLMAGAZINE_CLOSE;															//�رյ���
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
			if(auxiliaryAutoData.countTimes > 1){						
				auxiliaryAutoData.countTimes = 0;								//ʱ������
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		/* ץ������ */
		case 6:{
			PAWTURN_OPEN;			                              	//��ת��ȥ	
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��			
			if(auxiliaryAutoData.countTimes > 500){
				auxiliaryAutoData.countTimes = 0;								//ʱ������
				digitalIncreasing(&auxiliaryAutoData.schedule);
				executeNum++;																			//����ִ�д���
			}
		}break;
		case 7:{
			PAW_CLOSE;																				//צ�Ӽн�
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
			if(auxiliaryAutoData.countTimes > 80){
				auxiliaryAutoData.countTimes = 0;								//ʱ������
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 8:{
			PAWTURN_CLOSE;																		//צ���ջ�
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
//			if( (auxiliaryAutoData.countTimes > 200) && deathMode && (executeNum == 3) )
//				chassisData.autoSpeedTarget.y = speed;
			if(!PAWTURN_SENSOR || auxiliaryAutoData.countTimes > 700){
				auxiliaryAutoData.countTimes = 0;								//ʱ������
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 9:{
			PAW_OPEN;																		      //צ�Ӵ�	
			EJECT_OPEN;																				//�����
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
			if(auxiliaryAutoData.countTimes > 50){						
				auxiliaryAutoData.countTimes = 0;								//ʱ������
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 10:{
			EJECT_CLOSE;																			//����ر�
			if(executeNum == 1){
				PAWGOLEFT_OPEN;																	//������
				auxiliaryAutoData.schedule = 6;
			}
			else if(executeNum == 2){
				PAWGORIGHT_OPEN;																//������
				PAWGOLEFT_CLOSE;
				auxiliaryAutoData.schedule = 6;
			}
			else if(executeNum == 3){
				if(deathMode){
					PAWGO_FORWARD;																//צ����ǰ
					TOP_RIGHT;																		//��������
					auxiliaryAutoData.schedule = 6;								//����ץ�������Ҳ�
				}
				else{
					PAWGORIGHT_CLOSE;																//����
					TOP_LEFT;																				//�ư����ҩ��أ�����ץ�ڶ���
					PAWGO_BACK;																			//צ���ջأ�����ץ�ڶ���
				}
			}
			else if(executeNum == 4){
				auxiliaryAutoData.schedule = 5;										//ץ�м��
			}
			else if(executeNum == 5){
				PAWGOLEFT_OPEN;
				auxiliaryAutoData.schedule = 6;
			}
			else if(executeNum == 6){
				deathMode = 0;
				PAWGOLEFT_CLOSE;																	//����
				SMALLMAGAZINE_OPEN;																//�򿪵���
				executeNum = 0;																		//���ץ������
				auxiliaryAutoData.schedule = 2;										//����ģʽѡ��
			}
			digitalIncreasing(&auxiliaryAutoData.countTimes);		//��ʼ�ۼ�ʱ��
			if(auxiliaryAutoData.countTimes > 100){
				auxiliaryAutoData.countTimes = 0;									//ʱ������
				executeNum = 0;																		//ץ��ҩ������0
				auxiliaryAutoData.schedule = 2;
				SMALLMAGAZINE_OPEN;
			}
		}break;
		case 11:{
			PAWGO_FORWARD;
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
			if(auxiliaryAutoData.countTimes > 300){						
				auxiliaryAutoData.countTimes = 0;								//ʱ������
				SMALLMAGAZINE_OPEN;
				EJECT_CLOSE;																		//����ر�
				auxiliaryAutoData.schedule = 2;
			}
		}break;
		
		/* ���º���ץ�� */
		case 12:{
			chassisData.autoSpeedTarget.x = speed;
			if( TWO_LEFT == 1 ){  
				digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
				if(auxiliaryAutoData.countTimes > 5){							//������⵽10ms
					digitalClan(&auxiliaryAutoData.countTimes);			//ʱ������
					digitalIncreasing(&auxiliaryAutoData.schedule);	//����һ��	
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
		
		/* ����ץ�� */
		case 14:{
			chassisData.autoSpeedTarget.x = speed;
			if((ONE_LEFT == 1) || (ONE_RIGHT == 1)){
				digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
				if(auxiliaryAutoData.countTimes > 5){							//С500msʱ
					auxiliaryAutoData.countTimes = 0;								//ʱ������
					digitalIncreasing(&auxiliaryAutoData.schedule);
				}
			}
		}break;
		case 15:{
			chassisData.autoSpeedTarget.x = 0;
			SMALLMAGAZINE_CLOSE;															//�رյ���
			PAW_OPEN;																					//צ�Ӵ�
			PAWGO_FORWARD;																		//צ��ǰ��
			PAWTURN_OPEN;			                              	//��ת��ȥ	
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��	
			if(auxiliaryAutoData.countTimes > 600){						
				auxiliaryAutoData.countTimes = 0;								//ʱ������
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 16:{
			PAW_CLOSE;																				//צ�Ӽн�
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
			if(auxiliaryAutoData.countTimes > 100){						
				auxiliaryAutoData.countTimes = 0;								//ʱ������
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 17:{
			PAWTURN_CLOSE;																			//צ���ջ�
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
			if(!PAWTURN_SENSOR || auxiliaryAutoData.countTimes > 1000){						
				auxiliaryAutoData.countTimes = 0;								//ʱ������
				digitalIncreasing(&auxiliaryAutoData.schedule);
			}
		}break;
		case 18:{
			PAW_OPEN;																		      //צ�Ӵ�
			EJECT_OPEN;                                       //�����
			digitalIncreasing(&auxiliaryAutoData.countTimes);	//��ʼ�ۼ�ʱ��
			if(auxiliaryAutoData.countTimes > 300){						
				auxiliaryAutoData.countTimes = 0;								//ʱ������
				SMALLMAGAZINE_OPEN;
				EJECT_CLOSE;                                    //����ر�
				auxiliaryAutoData.schedule = 2;
			}
		}break;
		
		
		case 99: {//ֻ�е���99ʱ�����˳�
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
