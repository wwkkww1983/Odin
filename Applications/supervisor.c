#include "application.h"

supervisorStruct_t supervisorData;

void supervisorTaskCheck(void){																				//监控任务是否正常 每个任务都有loop+1
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

void supervisorArmedSwitch(uint8_t valve){														//用于快速切换解锁状态的
	if(valve){
		supervisorData.state &= ~STATE_DISARMED;													//上锁  先清0
		supervisorData.state |= STATE_ARMED;															//解锁
	}
	else{
		supervisorData.state &= ~STATE_ARMED;															//解锁		先清0
		supervisorData.state |= STATE_DISARMED;														//切换成上锁状态
	}
}

void supervisorStateSwitch(uint16_t state,uint8_t valve){							//用于切换监控状态机状态的																												
	if (valve)
		supervisorData.state |= state;
	else
		supervisorData.state &= ~state;
}

void supervisorMagCali(void){																					//MAG校准指令
	supervisorStateSwitch(STATE_MAGCALI,ENABLE);
	calibInit();
}

void supervisorImuCali(uint8_t accTare){															//IMU校准指令
	if(accTare)
		supervisorData.imuCali = CALI_GYO_ACC;
	else
		supervisorData.imuCali = CALI_ONLY_GYO;
}

void supervisorLedSwitch(void){																				//设置RGB闪烁状态
	if(supervisorData.state & STATE_SENSOR_ERROR){
		supervisorData.ledState = LED_HARDWORK_FAIL;											//传感器错误工作灯
	}
	else if(supervisorData.state & STATE_MAGCALI){
		supervisorData.ledState = LED_MAG_CALI;                        	 	//磁力计校准工作灯
	}
	else if(supervisorData.state & STATE_IMUCALI){
		supervisorData.ledState = LED_IMU_CALI;                         	//IMU校准工作灯
	}
	else if(robotConfigData.distinguishState || localIdData.distinguishState){													
		supervisorData.ledState = LED_DEFINE_ID;  												//ID识别
	}
	else if(supervisorData.gimbalCaliReset){														
		supervisorData.ledState = LED_GIMBAL_CALI;												//云台校准
	}
	else if(supervisorData.state & STATE_RADIO_LOSS){
		supervisorData.ledState = LED_RADIO_LOSS;                       	//遥控器丢失工作灯
	}
	else{
		supervisorData.ledState = LED_WORKINGORDER;                     	//正常工作灯
	}
}

void supervisorRadioLoss(void){																				//检测丢控
	remoteControlData.rcError.intervalNum = remoteControlData.rcError.errorCount - remoteControlData.rcError.lastErrorCount;
	remoteControlData.rcError.lastErrorCount = remoteControlData.rcError.errorCount;
	if(!remoteControlData.rcError.intervalNum){
		supervisorStateSwitch(STATE_RADIO_LOSS,ENABLE);                  	//检测到丢控
		supervisorData.beepState = MUSIC_RADIO_LOSS;
	}
	else{
		supervisorStateSwitch(STATE_RADIO_LOSS,DISABLE);                  //标记无丢控
		digitalLo(&controlTransData.otherRcReadly);
	}
}

void supervisorJudgeError(void){																			//检测裁判系统
	judgeData.intervalNum = judgeData.judgeErrorCount - judgeData.judgeLastErrorCount;
	judgeData.judgeLastErrorCount = judgeData.judgeErrorCount;
	if(!judgeData.intervalNum && robotConfigData.typeOfRobot != NO_ID){
		supervisorStateSwitch(STATE_JUDGE_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_JUDGE_ERROR,DISABLE); 
	}
}

void supervisorVisionError(void){																			//检测视觉端
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

void supervisorSlaveError(void){																			//检测云台板是否有数据反馈
	slaveSensorData.intervalNum = slaveSensorData.slaveErrorCount - slaveSensorData.slaveLastErrorCount;
	slaveSensorData.slaveLastErrorCount =slaveSensorData.slaveErrorCount;
	if(!slaveSensorData.intervalNum && robotConfigData.typeOfRobot != NO_ID){
		supervisorStateSwitch(STATE_SENSOR_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_SENSOR_ERROR,DISABLE); 
	}
}

void supervisorCurrentError(void){																		//检测功率板
	chassisData.currentError.intervalNum = chassisData.currentError.errorCount - chassisData.currentError.lastErrorCount;
	chassisData.currentError.lastErrorCount = chassisData.currentError.errorCount;
	if(!chassisData.currentError.intervalNum && robotConfigData.typeOfRobot != NO_ID){
		supervisorStateSwitch(STATE_CURRENT_ERROR,ENABLE); 
	}
	else{
		supervisorStateSwitch(STATE_CURRENT_ERROR,DISABLE); 
	}	
}

void supervisorMotorError(void){																			//检测基础动力是否正常
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
			wheelNum = 0;                                                     //没有ID时轮子数为0
			break;
	}
	//底盘电机检错
	for(uint8_t index = 0;index < wheelNum;index++){
		chassisData.wheelError[index].intervalNum = chassisData.wheelError[index].errorCount - chassisData.wheelError[index].lastErrorCount;
		chassisData.wheelError[index].lastErrorCount = chassisData.wheelError[index].errorCount;
		if(!chassisData.wheelError[index].intervalNum && robotConfigData.typeOfRobot != NO_ID)
			scanErrorNum++;
	}
	//云台电机检错
	for(uint8_t index = 0;index < 2;index++){
		gimbalData.gimbalError[index].intervalNum = gimbalData.gimbalError[index].errorCount - gimbalData.gimbalError[index].lastErrorCount;
		gimbalData.gimbalError[index].lastErrorCount = gimbalData.gimbalError[index].errorCount;
		if(!gimbalData.gimbalError[index].intervalNum && robotConfigData.typeOfRobot != NO_ID)
			scanErrorNum++;
	}
	//拨弹电机检错
	shootData.pokeError.intervalNum = shootData.pokeError.errorCount - shootData.pokeError.lastErrorCount;
	shootData.pokeError.lastErrorCount = shootData.pokeError.errorCount;
	if(!shootData.pokeError.intervalNum && robotConfigData.typeOfRobot != NO_ID)
		scanErrorNum++;
	//弹仓盖电机检错
	shootData.lidError.intervalNum = shootData.lidError.errorCount - shootData.lidError.lastErrorCount;
	shootData.lidError.lastErrorCount = shootData.lidError.errorCount;
	if(!shootData.lidError.intervalNum && robotConfigData.typeOfRobot != NO_ID)
		scanErrorNum++;
	//摩擦轮电机检错
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

void supervisorFlash(void){	   																				//Flash储存
	if(supervisorData.flashSave){                                       //如果想要对flash进行操作，直接将supervisorData.flashSave拉高一次即可
		supervisorData.beepState = MUSIC_PARAMCALI;												//检测到需要存入Flash		参数保存提示音
		configFlashWrite();                                             	//写入flash
		parameterWriteDataFormFlash(robotConfigData.typeOfRobot);					//写入TF卡PID参数
		digitalLo(&supervisorData.flashSave);                             	
	} 	
}

void supervisorGimbalCali(void){																			//云台码盘值校准
	parameter[PITCH_CENTER] = gimbal_chooseData(CODEBOARD_VALUE,&pitchMotorData);
	parameter[YAW_CENTER] = gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData);
	chassisData.yawCenterSave = parameter[YAW_CENTER];
	digitalHi(&supervisorData.flashSave);
}

void supervisorDefineRbot(void){																			//机器人类型校准
	parameter[ROBOT_TYPE] = NO_ID;
	robotDistinguish();
}

void supervisorDefineID(void){
	localIdDistinguish();
}

void supervisorUpdateTask(void *Parameters){
	uint16_t Loops[CONFIG_LIST] = {0,0,0,0,0,0};
	uint8_t Digital[CONFIG_LIST] = {0,0,0,0,0,0};
	TickType_t xLastWakeTime = xTaskGetTickCount();											//获取任务系统运行的时钟节拍数
	while(true){
		vTaskDelayUntil(&xLastWakeTime,SUPER_STACK_PERIOD);								//绝对延时（周期性延时函数）
		supervisorTaskCheck();																						//检测任务是否正常
		supervisorFlash();                                                //Flash写入检测
		tFCardUpdate();																										//tf卡检测	
		warningUpdate();																									//灯条、RGB与蜂鸣器更新
			
    /*-- 在解锁状态下的状态机监控 --*/
		if(supervisorData.state & STATE_ARMED){														//在解锁状态下
			if(RC_MODE == RCSW_BOTTOM){		  																//控制权检测		如果在底部就上锁
				supervisorArmedSwitch(DISABLE);																//将解锁状态切换成上锁状态
				supervisorData.beepState = MUSIC_DISARMED;										//上锁的提示音
			}
		}
		/*-- 在上锁状态下的状态机监控 --*/
		else if(supervisorData.state & STATE_DISARMED){										//supervisorData.state和STATE_DISARMED相同
			if(!(supervisorData.state & STATE_MAGCALI) \
				&& !(supervisorData.state & STATE_IMUCALI) \
				&& (robotConfigData.distinguishState != ROBOT_BEING_IDENTIFIED) \
				&& (supervisorData.gimbalCaliReset != ENABLE)
				&& (robotConfigData.typeOfRobot != NO_ID)\
				&& !localIdData.distinguishState){
				supervisorData.busyState = false;															//执行各种校准必须处于不忙碌且有ID的状态
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
					//IMU校准
					if(Loops[IMU] >= WAITLOOPS && Digital[IMU]!= High){
						supervisorImuCali(DISABLE);																				//遥控器所进行的校准，是不带加速度计校准的		
						supervisorData.beepState = MUSIC_IMUCALI;
						digitalHi(&Digital[IMU]);
						digitalClan(&Loops[IMU]);
					}
					else if(wirelessData.imuCalibrate){
						supervisorImuCali(ENABLE);                                        //IMU校准
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
			
			
#if UAV_SBUS																														//RM版
			if(((RC_ROLL < -CALI_RC_VALUE) && (RC_PITCH > CALI_RC_VALUE)) \
				|| (wirelessData.magCalibrate)){
#else
			if((((RC_TRANSVERSE < -CALI_RC_VALUE) && (RC_LONGITUDINAL > CALI_RC_VALUE)) \
				|| (wirelessData.magCalibrate))){
#endif
				if(!supervisorData.busyState){	
					//MAG校准
					if(Loops[MAG] >= WAITLOOPS && Digital[MAG]!= High){
						supervisorMagCali();																				//MAG校准指令
						supervisorData.beepState = MUSIC_MAGCALI;
						digitalHi(&Digital[MAG]);
						digitalClan(&Loops[MAG]);
					}
					else if(wirelessData.magCalibrate){
						supervisorMagCali();                                        //磁力计校准
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
						digitalHi(&supervisorData.flashSave);												//储存flash
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
						supervisorDefineRbot();																			//机器类型识别
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
						supervisorDefineID();																				//主控板ID设置
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
						digitalHi(&supervisorData.gimbalCaliReset);									//云台码盘值校准
						digitalHi(&Digital[GIMBAL_CALI]);
						digitalClan(&Loops[GIMBAL_CALI]);
					}
				}
			}
			else{
				if(supervisorData.gimbalCaliReset){
					supervisorGimbalCali();																				//只有在松开摇杆后才会执行校准云台码盘
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
				&& abs(RC_RUDD) < 10 && abs(RC_THROT) < 10))){								//校准从所有摇杆完全松开才开始
#else
			if((Digital[IMU] != Low \
				|| Digital[MAG] != Low \
				|| Digital[FLASH_OPERATION] != Low \
				|| Digital[DEFINE_ROBOT] != Low \
				|| Digital[YAW_CALI] != Low \
				|| Digital[GIMBAL_CALI] !=Low\
				|| Digital[DEFINE_ID] !=Low)
				&& (abs(RC_LONGITUDINAL) < 10 && abs(RC_TRANSVERSE) < 10 \
				&& abs(RC_PITCH) < 10 && abs(RC_RUDD) < 10)){										//校准从所有摇杆完全松开才开始
#endif
				digitalLo(&Digital[IMU]);
				digitalLo(&Digital[MAG]);
				digitalLo(&Digital[FLASH_OPERATION]);
				digitalLo(&Digital[DEFINE_ROBOT]);
				digitalLo(&Digital[YAW_CALI]);
				digitalLo(&Digital[GIMBAL_CALI]);
			  digitalLo(&Digital[DEFINE_ID]);
			}

			if(RC_MODE == RCSW_MID || RC_MODE== RCSW_TOP){									//解锁命令扫描
				supervisorArmedSwitch(ENABLE);
				supervisorData.beepState=MUSIC_ARMED;												  //解锁提示音
				supervisorData.ArmSwitch=ENABLE;
				digitalClan(&supervisorData.armTime);
			}
		}
		
		if(robotConfigData.typeOfRobot == NO_ID){
			supervisorData.beepState = MUSIC_NO_ID;													//没有ID会一直响
		}
		
		supervisorLedSwitch();																						//板载三色LED状态切换	设置灯的监控指示
		if(!(supervisorData.loops % 10)){
			supervisorRadioLoss();																					//丢控状态检测，1Hz
			supervisorMotorError();
			supervisorJudgeError();																					//裁判系统检测
			if(robotConfigData.typeOfRobot == INFANTRY_ID || robotConfigData.typeOfRobot == TANK_ID){
				supervisorSlaveError();
			}
			if(robotConfigData.robotDeviceList & DEVICE_VISION)
				supervisorVisionError();																			//视觉端检测
			if(robotConfigData.robotDeviceList & DEVICE_CURRENT)
				supervisorCurrentError();																			//功率板检测
		}
		digitalIncreasing(&supervisorData.loops);
	}
}

void supervisorInit(void){
	supervisorArmedSwitch(DISABLE);
	beepConfig();																												//蜂鸣器初始化，和LED一起属于监管状态机
	supervisorData.taskEvent[SUPERVISOR_TASK] = xTaskCreate(supervisorUpdateTask,"SUPE",SPUER_STACK_SIZE,NULL,SPUER_PRIORITY,&supervisorData.xHandleTask);
}
