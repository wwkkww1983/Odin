#include "warning.h"
#include "config.h"
#include "supervisor.h"
#include "shoot.h"
#include "auto_task.h"
#include "type_robot.h"
#include "local_id.h"
#include "imu.h"
#include "cali.h"
#include "rc.h"
#include "judge.h"
#include "vision.h"
#include "vision_ukf.h"
#include "power.h"

warningStruct_t warningData;

void lightBarsReportErrors(void){																			//用于报错
	uint8_t index = 0;
	uint8_t nowFaultBit = 0; 
	uint16_t remainFaultSate; 
	if(warningData.lightBarsState.u16_temp != 0){
		remainFaultSate = warningData.lightBarsState.u16_temp;
		while(index < SK6812_LED_STRIP_LENGTH && !warningData.highestFaultFlag){		//用于检测最高一级的错误的
			nowFaultBit = remainFaultSate % 2; 
			if(nowFaultBit){
				warningData.highestFault = index;
				digitalHi(&warningData.highestFaultFlag);
			}
			remainFaultSate = remainFaultSate >> 1;
			index ++;
		}
		digitalLo(&warningData.highestFaultFlag);
	}
	index = 0;
	if(warningData.highestFault != 0){																	//在第一位和最高错误位间来一个渐变效果，用于帮助计数
		while(index <= warningData.highestFault){
			if(index >= warningData.highestFaultLoop)
				setOneLedHsv(index,&SK6812_DARK);
			else
				setOneLedHsv(index,&SK6812_GREEN);
			index ++;
		}
	}
	index = warningData.highestFault;
	remainFaultSate = warningData.lightBarsState.s16_temp >> warningData.highestFault;
	while(index < SK6812_LED_STRIP_LENGTH){
		nowFaultBit = remainFaultSate % 2;
		if(nowFaultBit){
			setOneLedHsv(index,&SK6812_RED);
		}
		else{
			setOneLedHsv(index,&SK6812_GREEN);
		}
		remainFaultSate = remainFaultSate >> 1;
		index ++;
	}
	if(!(warningData.loops % 5)){					//500ms
		digitalIncreasing(&warningData.highestFaultLoop);
		if(warningData.highestFaultLoop > warningData.highestFault)
			digitalClan(&warningData.highestFaultLoop);
	}
	else{
		warningData.highestFault = 0;
	}
}

void lightBarsOfContrl(uint8_t contrlMode,uint8_t safeMode){					//用于控制状态
	uint8_t index = 0;
	uint8_t invertFreq = 0;
	static uint8_t lightBarsSwitch = ENABLE;
	switch(contrlMode){																									//控制模式
		case MANUAL_SINGLE:
			warningData.displayNumber = 1;
			break;
		case MANUAL_CONTINUOUS:
			warningData.displayNumber = 2;
			break;
		case AUTO_CONTINUOUS:
			warningData.displayNumber = 3;
			break;
	}
	switch(safeMode){																										//识别安保等级
		case SAFE:
			warningData.blinkFrequency = 1;
			warningData.displayColor = SK6812_GREEN;
			break;
		/**********无警告模式屏蔽此处*****************
		case WARNING:
			warningData.blinkFrequency = 2;
			warningData.displayColor = SK6812_YELLOW;
			break;
		********************************************/
		case DANGEROUS:
			warningData.blinkFrequency = 5;
			warningData.displayColor = SK6812_RED;
			break;
	}
	invertFreq = (1000 / WARNING_STACK_PERIOD) / warningData.blinkFrequency;
	if(!(warningData.loops % invertFreq)){
		lightBarsSwitch = !lightBarsSwitch;
		if(invertFreq == 10)//安全模式长亮
			lightBarsSwitch = ENABLE;
	}
	if(!lightBarsSwitch){
		warningData.displayColor = SK6812_DARK;
	}
	
	//从左到右显示射频单发，三连发，连发
	while(index < SK6812_LED_STRIP_LENGTH){
		if(index < warningData.displayNumber){
			setOneLedHsv(index,&warningData.displayColor);
		}
		else{
			setOneLedHsv(index,&SK6812_DARK);
		}
		index ++;
	}
	//右侧第三个灯显示扭腰,小陀螺
	if(autoTaskData->aviodFlag)
		warningData.displayColor = SK6812_RED;
	else if(infantryAutoData.rotateFlag)
		warningData.displayColor = SK6812_GREEN;
	else
		warningData.displayColor = SK6812_DARK;
	setOneLedHsv(SK6812_LED_STRIP_LENGTH - 3,&warningData.displayColor);
	//右侧第二个灯珠用于显示任务状态
	switch(autoTaskData->currentTask){																	//最左侧和最右侧的灯珠用于显示任务状态
		case R_TASK:
			warningData.displayColor = SK6812_RED;													
			break;	
		case V_TASK:
			warningData.displayColor = SK6812_GREEN;
			break;
		case Z_TASK:
			warningData.displayColor = SK6812_YELLOW;
			break;
		default:
			warningData.displayColor = SK6812_DARK;
			break;
	}
	setOneLedHsv(SK6812_LED_STRIP_LENGTH - 2,&warningData.displayColor);
	//最右边的灯显示电容状态
	//电容充电
	bool CapLight = false;
	if(powerData.chargeFlag){
		if(CAP_SOC > 95.0f){
			//满电长亮
			warningData.capSoc = 0x01;
			warningData.displayColor = SK6812_GREEN;
		}
		else{
			static uint8_t index = 0;
			warningData.capSoc = 0x00;
			if(!(warningData.loops % 2)){
				if(index){
					warningData.displayColor = SK6812_GREEN;
					index = 0;
				}
				else{
					warningData.displayColor = SK6812_DARK;
					index = 1;
				}	
			}
		}
		CapLight = true;
	}
	//电容放电
	else{
		if(CAP_SOC < 10.0f){
			//电容即将没电，警告
			warningData.capSoc = 0x02;
			warningData.displayColor = SK6812_RED;
		}
		else{
			//安全电量
			warningData.capSoc = 0x03;
			warningData.displayColor = SK6812_GREEN;
		}
		CapLight = true;
	}
	//连底盘时灯不亮
	if(!powerData.linkCapFlag){
		warningData.displayColor = SK6812_DARK;
		CapLight = true;
	}
	if(CapLight)
		setOneLedHsv(SK6812_LED_STRIP_LENGTH - 1,&warningData.displayColor);
	if(robotMode == MODE_KM && !shootData.fricMotorFlag){
		setAllLedColors(&SK6812_RED);
	}
}

void lightBarsStateSwitch(uint16_t state,uint8_t valve){							//用于切换灯条状态机状态的																														
	if (valve)
		warningData.lightBarsState.u16_temp |= state;											//把对应位置1
	else
		warningData.lightBarsState.u16_temp &= ~state;										//把对应位清0	
}

void lightBarsErrorCheck(void){																				//用于检查当前错误状态
	if(supervisorData.state & STATE_RADIO_LOSS){												//(1)检测遥控器是否丢失  0x0001
		lightBarsStateSwitch(RC_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(RC_FAULT,DISABLE);
	}
	if(supervisorData.state & STATE_JUDGE_ERROR){												//(2)检测裁判系统是否故障  0x0002
		lightBarsStateSwitch(JUDGE_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(JUDGE_FAULT,DISABLE);
	}
	if(supervisorData.state & STATE_MOTOR_ERROR){												//(3)检测动力系统和云台电机是否故障  0x0004
		lightBarsStateSwitch(MOTOR_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(MOTOR_FAULT,DISABLE);
	}
	if(supervisorData.state & STATE_SENSOR_ERROR){											//(4)检测云台板是否故障  0x0008
		lightBarsStateSwitch(SENSOR_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(SENSOR_FAULT,DISABLE);
	}
	if(supervisorData.state & STATE_VISION_ERROR){											//(5)检测TF卡是否插入  0x0010
		lightBarsStateSwitch(VISION_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(VISION_FAULT,DISABLE);
	}
	if(robotConfigData.robotDeviceList & DEVICE_CURRENT \
		&& powerData.CapError){																						//(6)检测电容板是否故障  0x0020
		lightBarsStateSwitch(POWER_LIMIT_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(POWER_LIMIT_FAULT,DISABLE);
	}
}

void lightBarsUpdate(void){																						//灯带状态更新
	lightBarsErrorCheck();																							//检查运行状态
	if(!(supervisorData.state & STATE_RADIO_LOSS)){											//如果处于有遥控的状态	没有丢控	
		if(supervisorData.state & STATE_DISARMED || remoteControlData.dt7Value.keyBoard.bit.CTRL){
			lightBarsReportErrors();																				//没有解锁或按下Z显示车身状态
			warningData.reportError = 0x00;
		}
		else{																															//如果在拥有控制权的情况下
			lightBarsStateUpdata();
			warningData.reportError = 0x01;
			switch(robotConfigData.typeOfRobot){
				case INFANTRY_ID:
					lightBarsOfContrl(shootData.shootMode,shootData.shootStatusMode); 
					break;
				case TANK_ID:
					lightBarsOfContrl(shootData.shootMode,shootData.shootStatusMode);
					break;
				case AUXILIARY_ID:																						//工程车的人机界面还包括登岛界面		
					break;
				case SENTRY_ID:																								//哨兵不用灯带
					break;
				case UAV_ID:
					lightBarsOfContrl(shootData.shootMode,shootData.shootStatusMode);
					break;
			}
		}
	}
	else{
		warningData.reportError = 0x00;
		lightBarsReportErrors();
	}
//	SK6812UpdateStrip();
}

void lightBarsStateUpdata(void){
	if(shootData.shootStatusMode == SAFE){
		lightBarsStateSwitch(SAFE_MODE,ENABLE);
	}
	else{
		lightBarsStateSwitch(SAFE_MODE,DISABLE);
	}
	if(shootData.shootStatusMode == DANGEROUS){
		lightBarsStateSwitch(DANGEROUS_MODE,ENABLE);
	}
	else{
		lightBarsStateSwitch(DANGEROUS_MODE,DISABLE);
	}
	if(autoTaskData->aviodFlag){
		lightBarsStateSwitch(AVIOD_STAE,ENABLE);
	}
	else{
		lightBarsStateSwitch(AVIOD_STAE,DISABLE);
	}
	if(infantryAutoData.rotateFlag){
		lightBarsStateSwitch(ROTATE_STAE,ENABLE);
	}
	else{
		lightBarsStateSwitch(ROTATE_STAE,DISABLE);
	}
	if(powerData.chargeFlag){
		lightBarsStateSwitch(CAP_CHARGE_STATE,ENABLE);
	}
	else{
		lightBarsStateSwitch(CAP_CHARGE_STATE,DISABLE);
	}
	if(powerData.linkCapFlag){
		lightBarsStateSwitch(LINK_CAP_STATE,ENABLE);
	}
	else{
		lightBarsStateSwitch(LINK_CAP_STATE,DISABLE);
	}
	if(autoTaskData->currentTask == R_TASK){
		lightBarsStateSwitch(R_TASK_STATE,ENABLE);
	}
	else{
		lightBarsStateSwitch(R_TASK_STATE,DISABLE);
	}
	if(autoTaskData->currentTask == V_TASK){
		lightBarsStateSwitch(V_TASK_STATE,ENABLE);
	}
	else{
		lightBarsStateSwitch(V_TASK_STATE,DISABLE);
	}
	if(autoTaskData->currentTask == Z_TASK){
		lightBarsStateSwitch(Z_TASK_STATE,ENABLE);
	}
	else{
		lightBarsStateSwitch(Z_TASK_STATE,DISABLE);
	}
}

void lightBarsJudgeUpdate(void){
	if(visionData.captureFlag&&visionData.cailSuccess)
		judgeData.extShowData.data1 = UKF_POS_Z;
	else
		judgeData.extShowData.data1 = 0.0;
	
	//电容电量百分比，17v为保护电压
		judgeData.extShowData.data2 = (float)(100 * (powerData.capVol - 17) / (powerData.capVolMax - 17));
	//没开启摩擦轮灯不亮
	if(!shootData.fricMotorFlag)	
		//红灯全亮		
		judgeData.extShowData.mask = 0x00;									
	else{
		if(shootData.shootMode == MANUAL_SINGLE){	
			//单发1颗灯
			judgeData.extShowData.mask |= 0x01;
			judgeData.extShowData.mask &= 0xF9;
		}
		else if(shootData.shootMode == MANUAL_CONTINUOUS){
			//三连发2颗灯
			judgeData.extShowData.mask |= 0x03;
			judgeData.extShowData.mask &= 0xFB;
		}
		else if(shootData.shootMode == AUTO_CONTINUOUS){
			//连发三颗灯
			judgeData.extShowData.mask |= 0x07;
		}
		if(infantryAutoData.rotateFlag)
      //小陀螺模式			
			judgeData.extShowData.mask |= 0x08;
		else
			judgeData.extShowData.mask &= 0xF7;
		
		if(infantryAutoData.aviodFlag)
			//扭腰模式			
			judgeData.extShowData.mask |= 0x10;
		else
			judgeData.extShowData.mask &= 0xEF;
			//开电容放电
		if(openCap)							   													
			judgeData.extShowData.mask |= 0x20;
		else
			judgeData.extShowData.mask &= 0xDF;
	}
}

void warningUpdate(void){
	colorStdInit();
  lightBarsUpdate();																									//sk6812 更新
	lightBarsJudgeUpdate();																							//裁判系统发送数据更新
	ledUpdateTask(supervisorData.ledState);                           	//LED闪动		主控板上3色LED闪动指示状态
	beepUpdateTask(supervisorData.beepState);												 		//蜂鸣器正常响起
	digitalClan(&supervisorData.beepState);													  	//状态清零
	digitalIncreasing(&warningData.loops);
}
