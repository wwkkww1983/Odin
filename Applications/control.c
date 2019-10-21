#include "application.h"
#include "driver.h"
#include "vision.h"

controlStruct_t controlData;
robotModeStruct_t robotMode = MODE_RELAX;			//�������
robotModeStruct_t lastRobotMode = MODE_RELAX;
uint8_t ControlMode(void){
	return RC_MODE;
}

void getGlobalMode(){													//�˺������ڻ�ȡȫ�ֿ���ģʽ��Ϣ
#ifndef SHIELD_JUDGE
//	if((supervisorData.state & STATE_MOTOR_ERROR)&& (supervisorData.state & STATE_CURRENT_ERROR) 
//		&& parameter[LOCAL_ID] == 0x101 && supervisorData.state & STATE_ARMED){				
//		robotMode = MODE_RELAX;										//����ģʽ��������Ѫ���������
//	}
	if(judgeData.extGameRobotState.remain_HP == 0){
		robotMode = MODE_RELAX;
		//������ʱ����С����
		if(infantryAutoData.rotateFlag){
			//�ر�С����
			infantryAutoData.closeRotate = true;
		}
	}
#endif
	if(supervisorData.state & STATE_RADIO_LOSS){
		robotMode = MODE_STOP;               		  //�ϵ���ºú󶪿ؾ�STOPģʽ
	}
	else{
		switch(robotMode){
			case MODE_INIT :{ 										  //�����ǰΪMODE_INIT
				if(RC_MODE == RCSW_BOTTOM){					  //���SW2����͵�ʱ
					robotMode = MODE_RELAX;             //����MODE_RELAX
				}
				else if(gimbalData.initFinishFlag){	  //�����̨������� ����ģʽ�л�
					digitalLo(&gimbalData.initFinishFlag);
					robotMode = (robotModeStruct_t)ControlMode(); 	//��ȡң����sw2��״̬		
				}
			}break;
			
			case MODE_KM :        								  //�����ǰΪ����ģʽ 																						
			case MODE_RC :{ 											  //�����ǰΪҡ��ģʽ			
				if(RC_MODE == RCSW_BOTTOM){					  //���SW2����͵�ʱ
					robotMode = MODE_RELAX;             //����ģʽΪRELAXģʽ	
				}
				else {
					robotMode = (robotModeStruct_t)ControlMode();//������Խ���ģʽ�л�	��ȡSW2��״̬
				}
			}break;
			
			case MODE_STOP :{ 										  //�����ǰ��⵽ģʽΪ����ģʽ��˵���ּ�⵽ң��������Ȼ�������в������û�ж���
				robotMode = MODE_INIT;        			  //��⵽ң�����л�����ʼ��ģʽ
			}break;
			
			case MODE_RELAX:{
				if(RC_MODE != RCSW_BOTTOM){
					robotMode = MODE_INIT;										
				}
			}break;
			default : break;
		}
	}
}

static void fricWheelSwitch(robotModeStruct_t robotmode, uint8_t switch_now){
	static uint8_t lastmode = 0;
	static uint8_t lastswitch = 0;
  if(lastmode != robotmode){                        		//�����������ģʽ�л�
		LASER_OFF;
	  digitalLo(&shootData.fricMotorFlag);								//Ħ���ֹرգ���������׼���ر�
		digitalLo(&P_HERO_42_LID);									//�رմ󵯲�
		digitalLo(&P_HERO_17_LID);									//�ر�С����
	}
	else{                             
	  if(robotmode == MODE_RC||robotmode == MODE_KM){			//��ҡ�˻����ģʽ��
			if(switch_now == RCSW_BOTTOM){               			//SW1����͵�ʱĦ���ֹر�
				LASER_OFF;																			//�����
				digitalLo(&shootData.fricMotorFlag);						//Ħ���ֹر�
			}
			else if(lastswitch == RCSW_BOTTOM && switch_now == RCSW_MID){
				LASER_ON;																				//���⿪
				digitalHi(&shootData.fricMotorFlag);           	//ֻ��SW1����͵����м䵵ʱĦ���ֿ���
			}
		}
		else{
			LASER_OFF;
		  digitalLo(&shootData.fricMotorFlag);              //��ҡ�˻��߼���ģʽ��Ħ���ֶ��ر�
		}
	}
	lastmode = robotmode;																	//��ִ�еĻ�����ģʽ�����ϴε�ģʽ
	lastswitch = switch_now;															//��ִ�е�SW1�����ϴε�SW1		RC_GEAR	
}

void getShootMode(void){
	static uint8_t lastswitch = 0;
	static uint8_t KB_TYPY_SHOOT_LAST = 0;
//	static uint8_t KB_TYPY_HEAT_PROTECT_LAST = 0;
	
	fricWheelSwitch(robotMode,RC_GEAR);
	if(robotMode == MODE_RELAX){
		shootData.shootMode = MANUAL_SINGLE;			//�ͷſ���Ȩ��ص�����ģʽ
		digitalHi(&shootData.ballisticFill);			//��ʼ�������
		digitalClan(&shootData.loadStep);					//����������������
	}	
	if(shootData.fricMotorFlag){									//Ħ���ֿ���״̬�ſ��Է����ӵ�
		if(robotMode == MODE_RC){										//ң��������ģʽ
			//RCģʽ�Ҳ��ֲ������������嵯ģʽ
			if(RC_ROTATE < -100 && ROBOT == INFANTRY_ID){	//������������嵯
				shootData.shootMode = AUTO_CONTINUOUS;
				shootData.clearBulletFlag = true;
			}
			//RCģʽ��ֻ�е���
			else{
				shootData.shootMode = MANUAL_SINGLE;
				shootData.clearBulletFlag = false;
			}
			if(lastswitch == RCSW_MID && RC_GEAR == RCSW_TOP){
				digitalHi(&shootData.fireFlag_17mm);
				digitalHi(&shootData.fireFlag_42mm);
			}
			else if(RC_GEAR != RCSW_TOP){							//��MID��BOTTOM
				shootDataReset();												//�����������
			}
		}	
		else if(robotMode == MODE_KM){							//���̿���ģʽ
			//����û���嵯����
			shootData.clearBulletFlag = false;
			if(KB_TYPY_SHOOT && !KB_TYPY_SHOOT_LAST){
					digitalIncreasing(&shootData.shootMode);
				if(shootData.shootMode > AUTO_CONTINUOUS)
					shootData.shootMode = MANUAL_SINGLE;
			}
			if(ROBOT == UAV_ID){	
				shootData.shootMode = AUTO_CONTINUOUS;
			}	
			
//			else if(KB_TYPY_HEAT_PROTECT){
//				if(!KB_TYPY_HEAT_PROTECT_LAST)
//					digitalIncreasing(&shootData.shootStatusMode);
//				if(shootData.shootStatusMode > DANGEROUS )
//					shootData.shootStatusMode = SAFE;
//			}	
			
			//��ȫģʽ��Σ��ģʽ֮���л�
			else if(KB_TYPY_HEAT_PROTECT)						 //�޾���ģʽ
				shootData.shootStatusMode = DANGEROUS;
			else 
				shootData.shootStatusMode = SAFE;
			 //�ѹرղ�������
			if(DISENABLE_AUTOMATIC_AIM_17MM){					
				if(shootData.shootMode == MANUAL_SINGLE || shootData.shootMode == MANUAL_CONTINUOUS){
					if(KB_17MM_SHOOT){
						digitalHi(&shootData.fireFlag_17mm);
					}
				}
				else if(shootData.shootMode == AUTO_CONTINUOUS){
					if(KB_17MM_SHOOT_CONTINUOUS){
						digitalHi(&shootData.fireFlag_17mm);
					}
				}
			}
			//�ѹر�Ӣ������
			else if(DISENABLE_AUTOMATIC_AIM_42MM){			
				if(shootData.shootMode == MANUAL_SINGLE){
					if(KB_42MM_SHOOT){
//						digitalHi(&shootData.fireFlag_17mm);
						digitalHi(&shootData.fireFlag_42mm);
					}
				}
				else if(shootData.shootMode == AUTO_CONTINUOUS){
					if(KB_42MM_SHOOT_CONTINUOUS){
//						digitalHi(&shootData.fireFlag_17mm);
						digitalHi(&shootData.fireFlag_42mm);
					}
					else{
						digitalLo(&shootData.fireFlag_17mm);
						digitalLo(&shootData.fireFlag_42mm);
					}
				}
			}
			
			KB_TYPY_SHOOT_LAST = KB_TYPY_SHOOT;
//			KB_TYPY_HEAT_PROTECT_LAST = KB_TYPY_HEAT_PROTECT;
		}			
	}
	else{
		digitalLo(&shootData.fireFlag_17mm);
		digitalLo(&shootData.fireFlag_42mm);
	}	
	//RC_GEAR=switch_now
	lastswitch = RC_GEAR;																									
}

//��ʼ������º���
void controlDeviceConfirm(uint16_t deviceFlag,DeviceActivation_t *deviceFunction){
	if(robotConfigData.robotDeviceList & deviceFlag){
		deviceFunction();
	}
}

void congtrolGlobalInit(void){
	controlDeviceConfirm(DEVICE_GIMBAL,gimbalInit);													//��̨��ʼ��
	controlDeviceConfirm(DEVICE_CHASSIS,chassisInit);												//���̳�ʼ��
	controlDeviceConfirm(DEVICE_CANSEND,canSendInit);												//CAN���ͳ�ʼ��										
	controlDeviceConfirm(DEVICE_DEFORMING,mechaDeformingInit);							//���ױ��γ�ʼ�� 
	controlDeviceConfirm(DEVICE_SHOOT,shootInit);														//���������ʼ��
	controlDeviceConfirm(DEVICE_SUPPLY,supplyInit);													//����������ʼ��
	adcInit();                                                              //ADC�ɼ���ʼ��
	visionSendDataInit();                           												//�Ӿ�������Ϣ��ʼ��
	pneumaticInit();                                                        //������ʼ��
}

void controlUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(true){	
		vTaskDelayUntil(&xLastWakeTime,CONTROL_PERIOD);
		
		if(remoteControlData.initFlag)
			rcUpdateTask();																											//ң�������ݸ���
		if(!(controlData.loops % 2)){	
			if(judgeData.initFlag)
				judgeTask();																											//����ϵͳ���ݸ���
		}
		if(supervisorData.state & STATE_SENSOR_ERROR)													//imu�����������򲻼���ִ��
			continue;
		if(!supervisorData.taskEvent[SUPERVISOR_TASK])
			continue;		
		//�����ڳɹ���ȡ����������Ϣ�����ִ�г�ʼ��
		if(robotConfigData.distinguishState == ROBOT_COMPLETE_IDENTIFIED \
			|| localIdData.distinguishState == ID_COMPLETE_IDENTIFIED){					
			congtrolGlobalInit();																								//���п���ȫ����ʼ�����������еĽṹ���г�ʼ��
				
			robotConfigData.distinguishState = ROBOT_NO_NEED_TO_IDENTIFY;				//���ͷ�ֹ�ظ���ʼ������ֹ�ظ���ID
			localIdData.distinguishState = ID_NO_NEED_TO_IDENTIFY;
			autoTaskInit();																											//�Զ������ʼ��
			digitalHi(&controlData.dataInitFlag);
		}
		//������л�����ID���ܿ��ƻ����ˣ������Զ������ʼ�����
		if(robotConfigData.typeOfRobot != NO_ID && controlData.dataInitFlag){	
			getGlobalMode();                    																//��ȡ������ģʽ
			getShootMode();											 																//��ȡĦ����ģʽ
			if(slaveSensorData.initFlag){
				moduleCommandUpload(SLAVE_SENSOR_USARTX);													//���Ϳ���ָ���̨��
				if(robotConfigData.typeOfRobot == TANK_ID)
					controlDataUpload();
			}
			controlDeviceConfirm(DEVICE_DEFORMING,mechaDeformingUpdate);				//���ױ��θ���		
			controlDeviceConfirm(DEVICE_GIMBAL,gimbalUpdate);										//��̨����
			controlDeviceConfirm(DEVICE_CHASSIS,chassisUpdate);									//���̸���,���̿���Ƶ�ʱ������̨��ͬ
			controlDeviceConfirm(DEVICE_SHOOT,shootUpdate);											//�����������
			if(!(controlData.loops % 2)){																				//4msһ�εĿ���
				controlDeviceConfirm(DEVICE_SUPPLY,supplyUpdate);									//������������
        lastRobotMode = robotMode;	
        gimbalData.lastCtrlMode =	gimbalData.ctrlMode	;					
			}
			controlDeviceConfirm(DEVICE_CANSEND,canSendUpdate);								  //CAN���͸���	
			autoTaskUpdate();										 																//�Զ�����ִ��
		}
		digitalIncreasing(&controlData.loops);
	}
}

void controlInit(void){
	supervisorData.taskEvent[CONTROL_TASK] = xTaskCreate(controlUpdateTask,"CONTROL",CONTROL_STACK_SIZE,NULL,CONTROL_PRIORITY,&controlData.xHandleTask);
}



