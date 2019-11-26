#include "application.h"

supervisorStruct_t supervisorData;

void supervisorTaskCheck(void){																				//��������Ƿ����� ÿ��������loop+1
	static uint32_t lastTaskLoops[LIST_OF_TASK];
	if((wirelessData.loops - lastTaskLoops[WIRELESS_TASK])>0)
		supervisorData.taskState[WIRELESS_TASK] = TASK_REGULAR;
	else
		supervisorData.taskState[WIRELESS_TASK] = TASK_FAULT;
	if((controlData.loops - lastTaskLoops[CONTROL_TASK])>0)
		supervisorData.taskState[CONTROL_TASK] = TASK_REGULAR;
	else
		supervisorData.taskState[CONTROL_TASK] = TASK_FAULT;
	lastTaskLoops[WIRELESS_TASK] = wirelessData.loops;				
	lastTaskLoops[CONTROL_TASK] = controlData.loops;
}

void supervisorArmedSwitch(uint8_t valve){														//���ڿ����л�����״̬��
	if(valve){
		supervisorData.state &= ~STATE_DISARMED;													//����  ����0
		supervisorData.state |= STATE_ARMED;															//����
	}
	else{
		supervisorData.state &= ~STATE_ARMED;															//����		����0
		supervisorData.state |= STATE_DISARMED;														//�л�������״̬
	}
}

void supervisorStateSwitch(uint16_t state,uint8_t valve){							//�����л����״̬��״̬��																												
	if (valve)
		supervisorData.state |= state;
	else
		supervisorData.state &= ~state;
}

void supervisorMagCali(void){																					//MAGУ׼ָ��
	supervisorStateSwitch(STATE_MAGCALI,ENABLE);
	calibInit();
}

void supervisorImuCali(uint8_t accTare){															//IMUУ׼ָ��
	if(accTare)
		supervisorData.imuCali = CALI_GYO_ACC;
	else
		supervisorData.imuCali = CALI_ONLY_GYO;
}

void supervisorLedSwitch(void){																				//����RGB��˸״̬
	if(supervisorData.state & STATE_SENSOR_ERROR){
		supervisorData.ledState = LED_HARDWORK_FAIL;											//��������������
	}
	else if(supervisorData.state & STATE_MAGCALI){
		supervisorData.ledState = LED_MAG_CALI;                        	 	//������У׼������
	}
	else if(supervisorData.state & STATE_IMUCALI){
		supervisorData.ledState = LED_IMU_CALI;                         	//IMUУ׼������
	}
	else if(robotConfigData.distinguishState || localIdData.distinguishState){													
		supervisorData.ledState = LED_DEFINE_ID;  												//IDʶ��
	}
	else if(supervisorData.gimbalCaliReset){														
		supervisorData.ledState = LED_GIMBAL_CALI;												//��̨У׼
	}
	else if(supervisorData.state & STATE_RADIO_LOSS){
		supervisorData.ledState = LED_RADIO_LOSS;                       	//ң������ʧ������
	}
	else{
		supervisorData.ledState = LED_WORKINGORDER;                     	//����������
	}
}

void supervisorRadioLoss(void){																				//��ⶪ��
	remoteControlData.rcError.intervalNum = remoteControlData.rcError.errorCount - remoteControlData.rcError.lastErrorCount;
	remoteControlData.rcError.lastErrorCount = remoteControlData.rcError.errorCount;
	if(!remoteControlData.rcError.intervalNum){
		supervisorStateSwitch(STATE_RADIO_LOSS,ENABLE);                  	//��⵽����
		supervisorData.beepState = MUSIC_RADIO_LOSS;
	}
	else{
		supervisorStateSwitch(STATE_RADIO_LOSS,DISABLE);                  //����޶���
		digitalLo(&controlTransData.otherRcReadly);
	}
}

void supervisorJudgeError(void){																			//������ϵͳ
	judgeData.intervalNum = judgeData.judgeErrorCount - judgeData.judgeLastErrorCount;
	judgeData.judgeLastErrorCount = judgeData.judgeErrorCount;
	if(!judgeData.intervalNum && robotConfigData.typeOfRobot != NO_ID){
		supervisorStateSwitch(STATE_JUDGE_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_JUDGE_ERROR,DISABLE); 
	}
}

void supervisorVisionError(void){																			//����Ӿ���
	visionData.visionErrorCount = visionData.CNTR.u16_temp;
	visionData.intervalNum = visionData.visionErrorCount - visionData.visionLastErrorCount;
	visionData.visionLastErrorCount = visionData.visionErrorCount;
	if(!visionData.intervalNum && robotConfigData.typeOfRobot != NO_ID){
		supervisorStateSwitch(STATE_VISION_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_VISION_ERROR,DISABLE); 
	}	
}

void supervisorSlaveError(void){																			//�����̨���Ƿ������ݷ���
	slaveSensorData.intervalNum = slaveSensorData.slaveErrorCount - slaveSensorData.slaveLastErrorCount;
	slaveSensorData.slaveLastErrorCount =slaveSensorData.slaveErrorCount;
	if(!slaveSensorData.intervalNum && robotConfigData.typeOfRobot != NO_ID){
		supervisorStateSwitch(STATE_SENSOR_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_SENSOR_ERROR,DISABLE); 
	}
}

void supervisorCurrentError(void){																		//��⹦�ʰ�
	chassisData.currentError.intervalNum = chassisData.currentError.errorCount - chassisData.currentError.lastErrorCount;
	chassisData.currentError.lastErrorCount = chassisData.currentError.errorCount;
	if(!chassisData.currentError.intervalNum && robotConfigData.typeOfRobot != NO_ID){
		supervisorStateSwitch(STATE_CURRENT_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_CURRENT_ERROR,DISABLE); 
	}	
}

void supervisorMotorError(void){																			//�����������Ƿ�����
	uint8_t wheelNum;
	uint8_t scanErrorNum = 0;
	switch(robotConfigData.typeOfRobot){
		case INFANTRY_ID: 
		case TANK_ID: 
		case AUXILIARY_ID: 
			wheelNum = 4;
			break;
		case SENTRY_ID:
			wheelNum = 2;
			break;
		case UAV_ID:
			wheelNum = 0;
			break;
		default: 
			wheelNum = 0;                                                     //û��IDʱ������Ϊ0
			break;
	}
	//���̵�����
	for(uint8_t index = 0;index < wheelNum;index++){
		chassisData.wheelError[index].intervalNum = chassisData.wheelError[index].errorCount - chassisData.wheelError[index].lastErrorCount;
		chassisData.wheelError[index].lastErrorCount = chassisData.wheelError[index].errorCount;
		if(!chassisData.wheelError[index].intervalNum && robotConfigData.typeOfRobot != NO_ID)
			scanErrorNum++;
	}
	//��̨������
	for(uint8_t index = 0;index < 2;index++){
		gimbalData.gimbalError[index].intervalNum = gimbalData.gimbalError[index].errorCount - gimbalData.gimbalError[index].lastErrorCount;
		gimbalData.gimbalError[index].lastErrorCount = gimbalData.gimbalError[index].errorCount;
		if(!gimbalData.gimbalError[index].intervalNum && robotConfigData.typeOfRobot != NO_ID)
			scanErrorNum++;
	}
	//����������
	shootData.pokeError.intervalNum = shootData.pokeError.errorCount - shootData.pokeError.lastErrorCount;
	shootData.pokeError.lastErrorCount = shootData.pokeError.errorCount;
	if(!shootData.pokeError.intervalNum && robotConfigData.typeOfRobot != NO_ID)
		scanErrorNum++;
	//���ָǵ�����
	shootData.lidError.intervalNum = shootData.lidError.errorCount - shootData.lidError.lastErrorCount;
	shootData.lidError.lastErrorCount = shootData.lidError.errorCount;
	if(!shootData.lidError.intervalNum && robotConfigData.typeOfRobot != NO_ID)
		scanErrorNum++;
	//Ħ���ֵ�����
	for(uint8_t index = 0;index < 2;index++){
		shootData.fricWheelError[index].intervalNum = shootData.fricWheelError[index].errorCount - shootData.fricWheelError[index].lastErrorCount;
		shootData.fricWheelError[index].lastErrorCount = shootData.fricWheelError[index].errorCount;
		if(!shootData.fricWheelError[index].intervalNum && robotConfigData.typeOfRobot != NO_ID)
			scanErrorNum++;
	}
	if(scanErrorNum){
		supervisorStateSwitch(STATE_MOTOR_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_MOTOR_ERROR,DISABLE); 
	}
}

void supervisorFlash(void){	   																				//Flash����
	if(supervisorData.flashSave){                                       //�����Ҫ��flash���в�����ֱ�ӽ�supervisorData.flashSave����һ�μ���
		supervisorData.beepState = MUSIC_PARAMCALI;												//��⵽��Ҫ����Flash		����������ʾ��
		configFlashWrite();                                             	//д��flash
		parameterWriteDataFormFlash(robotConfigData.typeOfRobot);					//д��TF��PID����
		digitalLo(&supervisorData.flashSave);                             	
	} 	
}

void supervisorGimbalCali(void){																			//��̨����ֵУ׼
	parameter[PITCH_CENTER] = gimbal_chooseData(CODEBOARD_VALUE,&pitchMotorData);
	parameter[YAW_CENTER] = gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData);
	chassisData.yawCenterSave = parameter[YAW_CENTER];
	digitalHi(&supervisorData.flashSave);
}

void supervisorDefineRbot(void){																			//����������У׼
	parameter[ROBOT_TYPE] = NO_ID;
	robotDistinguish();
}

void supervisorDefineID(void){
	localIdDistinguish();
}

void supervisorUpdateTask(void *Parameters){
	uint16_t Loops[CONFIG_LIST] = {0,0,0,0,0,0};
	uint8_t Digital[CONFIG_LIST] = {0,0,0,0,0,0};
	TickType_t xLastWakeTime = xTaskGetTickCount();											//��ȡ����ϵͳ���е�ʱ�ӽ�����
	while(true){
		vTaskDelayUntil(&xLastWakeTime,SUPER_STACK_PERIOD);								//������ʱ����������ʱ������
		supervisorTaskCheck();																						//��������Ƿ�����
		supervisorFlash();                                                //Flashд����
		tFCardUpdate();																										//tf�����	
		warningUpdate();																									//������RGB�����������
			
    /*-- �ڽ���״̬�µ�״̬����� --*/
		if(supervisorData.state & STATE_ARMED){														//�ڽ���״̬��
			if(RC_MODE == RCSW_BOTTOM){		  																//����Ȩ���		����ڵײ�������
				supervisorArmedSwitch(DISABLE);																//������״̬�л�������״̬
				supervisorData.beepState = MUSIC_DISARMED;										//��������ʾ��
			}
		}
		/*-- ������״̬�µ�״̬����� --*/
		else if(supervisorData.state & STATE_DISARMED){										//supervisorData.state��STATE_DISARMED��ͬ
			if(!(supervisorData.state & STATE_MAGCALI) \
				&& !(supervisorData.state & STATE_IMUCALI) \
				&& (robotConfigData.distinguishState != ROBOT_BEING_IDENTIFIED) \
				&& (supervisorData.gimbalCaliReset != ENABLE)
				&& (robotConfigData.typeOfRobot != NO_ID)\
				&& !localIdData.distinguishState){
				supervisorData.busyState = false;															//ִ�и���У׼���봦�ڲ�æµ����ID��״̬
			}
			else{
				supervisorData.busyState = true;	
			}
			
#if UAV_SBUS			
			if(((RC_ROLL > CALI_RC_VALUE) && (RC_PITCH > CALI_RC_VALUE)) \
				|| (wirelessData.imuCalibrate)){
#else 
			if((((RC_TRANSVERSE > CALI_RC_VALUE) && (RC_LONGITUDINAL > CALI_RC_VALUE)) \
				|| (wirelessData.imuCalibrate))){
#endif			
				if(!supervisorData.busyState){
					//IMUУ׼
					if(Loops[IMU] >= WAITLOOPS && Digital[IMU]!= High){
						supervisorImuCali(DISABLE);																				//ң���������е�У׼���ǲ������ٶȼ�У׼��		
						supervisorData.beepState = MUSIC_IMUCALI;
						digitalHi(&Digital[IMU]);
						digitalClan(&Loops[IMU]);
					}
					else if(wirelessData.imuCalibrate){
						supervisorImuCali(ENABLE);                                        //IMUУ׼
						supervisorData.beepState = MUSIC_IMUCALI;
						digitalLo(&wirelessData.imuCalibrate);
					}
					else
						digitalIncreasing(&Loops[IMU]);
				}
			}
			else{
				digitalClan(&Loops[IMU]);
			}
			
			
#if UAV_SBUS																														//RM��
			if(((RC_ROLL < -CALI_RC_VALUE) && (RC_PITCH > CALI_RC_VALUE)) \
				|| (wirelessData.magCalibrate)){
#else
			if((((RC_TRANSVERSE < -CALI_RC_VALUE) && (RC_LONGITUDINAL > CALI_RC_VALUE)) \
				|| (wirelessData.magCalibrate))){
#endif
				if(!supervisorData.busyState){	
					//MAGУ׼
					if(Loops[MAG] >= WAITLOOPS && Digital[MAG]!= High){
						supervisorMagCali();																				//MAGУ׼ָ��
						supervisorData.beepState = MUSIC_MAGCALI;
						digitalHi(&Digital[MAG]);
						digitalClan(&Loops[MAG]);
					}
					else if(wirelessData.magCalibrate){
						supervisorMagCali();                                        //������У׼
						supervisorData.beepState = MUSIC_MAGCALI;
						digitalLo(&wirelessData.magCalibrate);
					}
					else
						digitalIncreasing(&Loops[MAG]);
				}
			}
			else{
				digitalClan(&Loops[MAG]);
			}
			
			
#if UAV_SBUS
			if((RC_ROLL < -CALI_RC_VALUE) && (RC_PITCH < -CALI_RC_VALUE)){
#else
			if((RC_TRANSVERSE < -CALI_RC_VALUE) && (RC_LONGITUDINAL < -CALI_RC_VALUE)){
#endif
				if(!supervisorData.busyState){	
					digitalIncreasing(&Loops[FLASH_OPERATION]);
					if(Loops[FLASH_OPERATION] >= WAITLOOPS && Digital[FLASH_OPERATION]!= High){
						digitalHi(&supervisorData.flashSave);												//����flash
						digitalHi(&Digital[FLASH_OPERATION]);
						digitalClan(&Loops[FLASH_OPERATION]);
					}
				}
			}
			else{																														
				digitalClan(&Loops[FLASH_OPERATION]);
			}
			
			
#if UAV_SBUS	
			if((RC_THROT > CALI_RC_VALUE) && (RC_RUDD > CALI_RC_VALUE)){
#else	
			if((RC_PITCH > CALI_RC_VALUE) && (RC_RUDD > CALI_RC_VALUE)){
#endif	
				if(!supervisorData.busyState){
					digitalIncreasing(&Loops[DEFINE_ROBOT]);
					if(Loops[DEFINE_ROBOT] >= WAITLOOPS && Digital[DEFINE_ROBOT]!= High){
						supervisorDefineRbot();																			//��������ʶ��
						digitalHi(&Digital[DEFINE_ROBOT]);
						digitalClan(&Loops[DEFINE_ROBOT]);
					}
				}
			}
			else{
				digitalClan(&Loops[DEFINE_ROBOT]);
			}
			

#if UAV_SBUS	
			if((RC_THROT > CALI_RC_VALUE) && (RC_RUDD < -CALI_RC_VALUE)){
#else	
			if((RC_PITCH > CALI_RC_VALUE) && (RC_RUDD < -CALI_RC_VALUE)){
#endif			
				if(!supervisorData.busyState){
					digitalIncreasing(&Loops[DEFINE_ID]);
					if(Loops[DEFINE_ID] >= WAITLOOPS && Digital[DEFINE_ID]!= High){
						supervisorDefineID();																				//���ذ�ID����
						digitalHi(&Digital[DEFINE_ID]);
						digitalClan(&Loops[DEFINE_ID]);						
					}
				}
			}
			else{
				digitalClan(&Loops[DEFINE_ID]);
			}
			
#if UAV_SBUS
			if((RC_RUDD < -CALI_RC_VALUE) && (RC_PITCH < -CALI_RC_VALUE)){
#else
			if((RC_TRANSVERSE > CALI_RC_VALUE) && (RC_LONGITUDINAL < -CALI_RC_VALUE)){
#endif	
				if(!supervisorData.busyState){
					digitalIncreasing(&Loops[GIMBAL_CALI]);
					if(Loops[GIMBAL_CALI] >= WAITLOOPS && Digital[GIMBAL_CALI]!= High){
						digitalHi(&supervisorData.gimbalCaliReset);									//��̨����ֵУ׼
						digitalHi(&Digital[GIMBAL_CALI]);
						digitalClan(&Loops[GIMBAL_CALI]);
					}
				}
			}
			else{
				if(supervisorData.gimbalCaliReset){
					supervisorGimbalCali();																				//ֻ�����ɿ�ҡ�˺�Ż�ִ��У׼��̨����
					digitalLo(&supervisorData.gimbalCaliReset);
				}
				digitalClan(&Loops[GIMBAL_CALI]);
			}
		
#if UAV_SBUS			
			if((Digital[IMU] != Low \
				|| Digital[MAG] != Low \
				|| Digital[FLASH_OPERATION] != Low \
				|| Digital[DEFINE_ROBOT] != Low \
				|| Digital[YAW_CALI] != Low \
				|| Digital[GIMBAL_CALI] !=Low\
				|| Digital[DEFINE_ID] !=Low)
				&&  (abs(RC_ROLL) < 10 && abs(RC_PITCH) < 10 \
				&& abs(RC_RUDD) < 10 && abs(RC_THROT) < 10))){								//У׼������ҡ����ȫ�ɿ��ſ�ʼ
#else
			if((Digital[IMU] != Low \
				|| Digital[MAG] != Low \
				|| Digital[FLASH_OPERATION] != Low \
				|| Digital[DEFINE_ROBOT] != Low \
				|| Digital[YAW_CALI] != Low \
				|| Digital[GIMBAL_CALI] !=Low\
				|| Digital[DEFINE_ID] !=Low)
				&& (abs(RC_LONGITUDINAL) < 10 && abs(RC_TRANSVERSE) < 10 \
				&& abs(RC_PITCH) < 10 && abs(RC_RUDD) < 10)){										//У׼������ҡ����ȫ�ɿ��ſ�ʼ
#endif
				digitalLo(&Digital[IMU]);
				digitalLo(&Digital[MAG]);
				digitalLo(&Digital[FLASH_OPERATION]);
				digitalLo(&Digital[DEFINE_ROBOT]);
				digitalLo(&Digital[YAW_CALI]);
				digitalLo(&Digital[GIMBAL_CALI]);
			  digitalLo(&Digital[DEFINE_ID]);
			}

			if(RC_MODE == RCSW_MID || RC_MODE== RCSW_TOP){									//��������ɨ��
				supervisorArmedSwitch(ENABLE);
				supervisorData.beepState=MUSIC_ARMED;												  //������ʾ��
				supervisorData.ArmSwitch=ENABLE;
				digitalClan(&supervisorData.armTime);
			}
		}
		
		if(robotConfigData.typeOfRobot == NO_ID){
			supervisorData.beepState = MUSIC_NO_ID;													//û��ID��һֱ��
		}
		
		supervisorLedSwitch();																						//������ɫLED״̬�л�	���õƵļ��ָʾ
		if(!(supervisorData.loops % 10)){
			supervisorRadioLoss();																					//����״̬��⣬1Hz
			supervisorMotorError();
			supervisorJudgeError();																					//����ϵͳ���
			if(robotConfigData.typeOfRobot == INFANTRY_ID || robotConfigData.typeOfRobot == TANK_ID){
				supervisorSlaveError();
			}
			if(robotConfigData.robotDeviceList & DEVICE_VISION)
				supervisorVisionError();																			//�Ӿ��˼��
			if(robotConfigData.robotDeviceList & DEVICE_CURRENT)
				supervisorCurrentError();																			//���ʰ���
		}
		digitalIncreasing(&supervisorData.loops);
	}
}

void supervisorInit(void){
	supervisorArmedSwitch(DISABLE);
	beepConfig();																												//��������ʼ������LEDһ�����ڼ��״̬��
	supervisorData.taskEvent[SUPERVISOR_TASK] = xTaskCreate(supervisorUpdateTask,"SUPE",SPUER_STACK_SIZE,NULL,SPUER_PRIORITY,&supervisorData.xHandleTask);
}
