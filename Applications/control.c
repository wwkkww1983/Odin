#include "application.h"
#include "driver.h"
#include "vision.h"

controlStruct_t controlData;
robotModeStruct_t robotMode = MODE_RELAX;			//解除控置
robotModeStruct_t lastRobotMode = MODE_RELAX;
uint8_t ControlMode(void){
	return RC_MODE;
}

void getGlobalMode(){													//此函数用于获取全局控制模式信息
#ifndef SHIELD_JUDGE
//	if((supervisorData.state & STATE_MOTOR_ERROR)&& (supervisorData.state & STATE_CURRENT_ERROR) 
//		&& parameter[LOCAL_ID] == 0x101 && supervisorData.state & STATE_ARMED){				
//		robotMode = MODE_RELAX;										//主控模式、并且有血量的情况下
//	}
	if(judgeData.extGameRobotState.remain_HP == 0){
		robotMode = MODE_RELAX;
		//死亡的时候开启小陀螺
		if(infantryAutoData.rotateFlag){
			//关闭小陀螺
			infantryAutoData.closeRotate = true;
		}
	}
#endif
	if(supervisorData.state & STATE_RADIO_LOSS){
		robotMode = MODE_STOP;               		  //上电恒温好后丢控就STOP模式
	}
	else{
		switch(robotMode){
			case MODE_INIT :{ 										  //如果当前为MODE_INIT
				if(RC_MODE == RCSW_BOTTOM){					  //如果SW2打到最低档时
					robotMode = MODE_RELAX;             //设置MODE_RELAX
				}
				else if(gimbalData.initFinishFlag){	  //如果云台回中完成 可以模式切换
					digitalLo(&gimbalData.initFinishFlag);
					robotMode = (robotModeStruct_t)ControlMode(); 	//读取遥控器sw2的状态		
				}
			}break;
			
			case MODE_KM :        								  //如果当前为键盘模式 																						
			case MODE_RC :{ 											  //如果当前为摇杆模式			
				if(RC_MODE == RCSW_BOTTOM){					  //如果SW2打到最低档时
					robotMode = MODE_RELAX;             //设置模式为RELAX模式	
				}
				else {
					robotMode = (robotModeStruct_t)ControlMode();//否则可以进行模式切换	读取SW2的状态
				}
			}break;
			
			case MODE_STOP :{ 										  //如果当前检测到模式为丢控模式，说明又检测到遥控器，不然程序运行不到这里，没有丢控
				robotMode = MODE_INIT;        			  //检测到遥控器切换到初始化模式
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
  if(lastmode != robotmode){                        		//如果机器人有模式切换
		LASER_OFF;
	  digitalLo(&shootData.fricMotorFlag);								//摩擦轮关闭，气动发射准备关闭
		digitalLo(&P_HERO_42_LID);									//关闭大弹仓
		digitalLo(&P_HERO_17_LID);									//关闭小弹仓
	}
	else{                             
	  if(robotmode == MODE_RC||robotmode == MODE_KM){			//在摇杆或键鼠模式下
			if(switch_now == RCSW_BOTTOM){               			//SW1打到最低档时摩擦轮关闭
				LASER_OFF;																			//激光关
				digitalLo(&shootData.fricMotorFlag);						//摩擦轮关闭
			}
			else if(lastswitch == RCSW_BOTTOM && switch_now == RCSW_MID){
				LASER_ON;																				//激光开
				digitalHi(&shootData.fricMotorFlag);           	//只有SW1从最低档打到中间档时摩擦轮开启
			}
		}
		else{
			LASER_OFF;
		  digitalLo(&shootData.fricMotorFlag);              //除摇杆或者键鼠模式外摩擦轮都关闭
		}
	}
	lastmode = robotmode;																	//已执行的机器人模式看作上次的模式
	lastswitch = switch_now;															//已执行的SW1看作上次的SW1		RC_GEAR	
}

void getShootMode(void){
	static uint8_t lastswitch = 0;
	static uint8_t KB_TYPY_SHOOT_LAST = 0;
//	static uint8_t KB_TYPY_HEAT_PROTECT_LAST = 0;
	
	fricWheelSwitch(robotMode,RC_GEAR);
	if(robotMode == MODE_RELAX){
		shootData.shootMode = MANUAL_SINGLE;			//释放控制权后回到单发模式
		digitalHi(&shootData.ballisticFill);			//开始弹道填充
		digitalClan(&shootData.loadStep);					//重置气动发射流程
	}	
	if(shootData.fricMotorFlag){									//摩擦轮开启状态才可以发射子弹
		if(robotMode == MODE_RC){										//遥控器控制模式
			//RC模式且拨轮拨动步兵进入清弹模式
			if(RC_ROTATE < -100 && ROBOT == INFANTRY_ID){	//步兵场间快速清弹
				shootData.shootMode = AUTO_CONTINUOUS;
				shootData.clearBulletFlag = true;
			}
			//RC模式下只有单发
			else{
				shootData.shootMode = MANUAL_SINGLE;
				shootData.clearBulletFlag = false;
			}
			if(lastswitch == RCSW_MID && RC_GEAR == RCSW_TOP){
				digitalHi(&shootData.fireFlag_17mm);
				digitalHi(&shootData.fireFlag_42mm);
			}
			else if(RC_GEAR != RCSW_TOP){							//从MID到BOTTOM
				shootDataReset();												//射击参数重置
			}
		}	
		else if(robotMode == MODE_KM){							//键盘控制模式
			//键鼠没有清弹功能
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
			
			//安全模式和危险模式之间切换
			else if(KB_TYPY_HEAT_PROTECT)						 //无警告模式
				shootData.shootStatusMode = DANGEROUS;
			else 
				shootData.shootStatusMode = SAFE;
			 //已关闭步兵自瞄
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
			//已关闭英雄自瞄
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

//初始化与更新函数
void controlDeviceConfirm(uint16_t deviceFlag,DeviceActivation_t *deviceFunction){
	if(robotConfigData.robotDeviceList & deviceFlag){
		deviceFunction();
	}
}

void congtrolGlobalInit(void){
	controlDeviceConfirm(DEVICE_GIMBAL,gimbalInit);													//云台初始化
	controlDeviceConfirm(DEVICE_CHASSIS,chassisInit);												//底盘初始化
	controlDeviceConfirm(DEVICE_CANSEND,canSendInit);												//CAN发送初始化										
	controlDeviceConfirm(DEVICE_DEFORMING,mechaDeformingInit);							//机甲变形初始化 
	controlDeviceConfirm(DEVICE_SHOOT,shootInit);														//发射机构初始化
	controlDeviceConfirm(DEVICE_SUPPLY,supplyInit);													//补给机构初始化
	adcInit();                                                              //ADC采集初始化
	visionSendDataInit();                           												//视觉发送信息初始化
	pneumaticInit();                                                        //气动初始化
}

void controlUpdateTask(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(true){	
		vTaskDelayUntil(&xLastWakeTime,CONTROL_PERIOD);
		
		if(remoteControlData.initFlag)
			rcUpdateTask();																											//遥控器数据更新
		if(!(controlData.loops % 2)){	
			if(judgeData.initFlag)
				judgeTask();																											//裁判系统数据更新
		}
		if(supervisorData.state & STATE_SENSOR_ERROR)													//imu传感器错误则不继续执行
			continue;
		if(!supervisorData.taskEvent[SUPERVISOR_TASK])
			continue;		
		//必须在成功读取到机器人信息后才能执行初始化
		if(robotConfigData.distinguishState == ROBOT_COMPLETE_IDENTIFIED \
			|| localIdData.distinguishState == ID_COMPLETE_IDENTIFIED){					
			congtrolGlobalInit();																								//所有控制全部初始化，机器人有的结构进行初始化
				
			robotConfigData.distinguishState = ROBOT_NO_NEED_TO_IDENTIFY;				//拉低防止重复初始化，防止重复给ID
			localIdData.distinguishState = ID_NO_NEED_TO_IDENTIFY;
			autoTaskInit();																											//自动任务初始化
			digitalHi(&controlData.dataInitFlag);
		}
		//必须具有机器人ID才能控制机器人，并且自动任务初始化完成
		if(robotConfigData.typeOfRobot != NO_ID && controlData.dataInitFlag){	
			getGlobalMode();                    																//获取机器人模式
			getShootMode();											 																//获取摩擦轮模式
			if(slaveSensorData.initFlag){
				moduleCommandUpload(SLAVE_SENSOR_USARTX);													//发送控制指令到云台板
				if(robotConfigData.typeOfRobot == TANK_ID)
					controlDataUpload();
			}
			controlDeviceConfirm(DEVICE_DEFORMING,mechaDeformingUpdate);				//机甲变形更新		
			controlDeviceConfirm(DEVICE_GIMBAL,gimbalUpdate);										//云台更新
			controlDeviceConfirm(DEVICE_CHASSIS,chassisUpdate);									//底盘更新,底盘控制频率必须和云台相同
			controlDeviceConfirm(DEVICE_SHOOT,shootUpdate);											//发射机构更新
			if(!(controlData.loops % 2)){																				//4ms一次的控制
				controlDeviceConfirm(DEVICE_SUPPLY,supplyUpdate);									//补给机构更新
        lastRobotMode = robotMode;	
        gimbalData.lastCtrlMode =	gimbalData.ctrlMode	;					
			}
			controlDeviceConfirm(DEVICE_CANSEND,canSendUpdate);								  //CAN发送更新	
			autoTaskUpdate();										 																//自动控制执行
		}
		digitalIncreasing(&controlData.loops);
	}
}

void controlInit(void){
	supervisorData.taskEvent[CONTROL_TASK] = xTaskCreate(controlUpdateTask,"CONTROL",CONTROL_STACK_SIZE,NULL,CONTROL_PRIORITY,&controlData.xHandleTask);
}



