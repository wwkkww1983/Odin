#include "type_robot.h"
#include "config.h"
#include "supervisor.h"
#include "Util.h"
#include "rc.h"
#include "Driver_Beep.h"
#include "control.h"
#include "tf_card_parameter.h"
#include "local_id.h"
#include "power.h"

robotConfigStruct_t robotConfigData; 

void currentRobotParameterConfig(void){
	switch(robotConfigData.typeOfRobot){
		case INFANTRY_ID:{							//步兵ID
			parameter[WEAPON_TYPE] = SMALL_LAUNCHER;												//武器种类
			parameter[CHASSIS_CURRENT] = INFANTR_CHASSIS_CURRENT;						//底盘最大电流限制
			parameter[PITCH_MIN_RANGE] = INFANTRY_PITCH_MIN_RANGE;					//最小俯仰角(云台抬起为负数)
			parameter[PITCH_MAX_RANGE] = INFANTRY_PITCH_MAX_RANGE;					//最大俯仰角
			parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//获取云台安装方向
			parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN];      
			pitchMotorData.motorID = parameter[PITCH_TYPE];    //获取pitch轴电机ID
      yawMotorData.motorID = parameter[YAW_TYPE];        //获取taw轴电机ID
			//步兵功率限制参数配置
			powerData.powerLimit = INFANTR_POWER_LIMIT;
			powerData.warningPower = INFANTR_WARNING_POWER;
			powerData.WarningPowerBuff = INFANTR_WARNING_POWER_BUFF;
			powerData.noJudgeTotalCurrentLimit = INFANTR_NO_JUDGE_TOTAL_CURRENT_LIMIT;
			powerData.judgeTotalCurrentLimit = INFANTR_JUDGE_TOTAL_CURRENT_LIMIT;
			powerData.addPowerCurrent = INFANTR_ADD_POWER_CURRENT;
			robotConfigData.robotDeviceList = INFANTRY_DEVICE_LIST;
		}break;

		case TANK_ID:{									//英雄车ID
				parameter[WEAPON_TYPE] = DOUBLE_LAUNCHER;
				parameter[CHASSIS_CURRENT] = TANK_CHASSIS_CURRENT;	
				parameter[PITCH_MIN_RANGE] = TANK_PITCH_MIN_RANGE;
				parameter[PITCH_MAX_RANGE] = TANK_PITCH_MAX_RANGE;	
				parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//获取云台安装方向
				parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN]; 
				pitchMotorData.motorID = parameter[PITCH_TYPE];    //获取pitch轴电机ID
        yawMotorData.motorID = parameter[YAW_TYPE];        //获取taw轴电机ID
				//英雄功率限制参数配置
			  powerData.powerLimit = TANK_POWER_LIMIT;
				powerData.warningPower = TANK_WARNING_POWER;
				powerData.WarningPowerBuff = TANK_WARNING_POWER_BUFF;
				powerData.noJudgeTotalCurrentLimit = TANK_NO_JUDGE_TOTAL_CURRENT_LIMIT;
				powerData.judgeTotalCurrentLimit = TANK_JUDGE_TOTAL_CURRENT_LIMIT;
				powerData.addPowerCurrent = TANK_ADD_POWER_CURRENT;
				robotConfigData.robotDeviceList = TANK_DEVICE_LIST;
		}break;
		
		case AUXILIARY_ID:{							//工程车ID
			parameter[WEAPON_TYPE] = NO_WEAPONS;
			parameter[CHASSIS_CURRENT] = CHASSIS_NO_LIMIT;	
			parameter[PITCH_MIN_RANGE] = AUXILIARY_PITCH_MIN_RANGE;
			parameter[PITCH_MAX_RANGE] = AUXILIARY_PITCH_MAX_RANGE;		
			parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//获取云台安装方向
			parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN];	
			pitchMotorData.motorID = parameter[PITCH_TYPE];    //获取pitch轴电机ID
      yawMotorData.motorID = parameter[YAW_TYPE];        //获取taw轴电机ID
			robotConfigData.robotDeviceList = AUXILIARY_DEVICE_LIST;
		}break;
		
		case SENTRY_ID:{								//哨兵ID
			parameter[WEAPON_TYPE] = SMALL_LAUNCHER;
			parameter[CHASSIS_CURRENT] = SENTRY_CHASSIS_CURRENT;	
			parameter[PITCH_MIN_RANGE] = SENTRY_PITCH_MIN_RANGE;
			parameter[PITCH_MAX_RANGE] = SENTRY_PITCH_MAX_RANGE;	
			parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//获取云台安装方向
			parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN];
			pitchMotorData.motorID = parameter[PITCH_TYPE];    //获取pitch轴电机ID
      yawMotorData.motorID = parameter[YAW_TYPE];        //获取taw轴电机ID
			//哨兵功率限制参数配置
			powerData.powerLimit = SENTRY_POWER_LIMIT;
			powerData.warningPower = SENTRY_WARNING_POWER;
			powerData.WarningPowerBuff = SENTRY_WARNING_POWER_BUFF;
			powerData.noJudgeTotalCurrentLimit = SENTRY_NO_JUDGE_TOTAL_CURRENT_LIMIT;
			powerData.judgeTotalCurrentLimit = SENTRY_JUDGE_TOTAL_CURRENT_LIMIT;
			powerData.addPowerCurrent = SENTRY_ADD_POWER_CURRENT;
			robotConfigData.robotDeviceList = SENTRY_DEVICE_LIST;
		}break;
		
		case UAV_ID:{										//无人机ID
			parameter[WEAPON_TYPE] = SMALL_LAUNCHER;
			parameter[CHASSIS_CURRENT] = CHASSIS_NO_LIMIT;	
			parameter[PITCH_MIN_RANGE] = UAV_PITCH_MIN_RANGE;
			parameter[PITCH_MAX_RANGE] = UAV_PITCH_MAX_RANGE;	
			parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//获取云台安装方向
			parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN];	
			pitchMotorData.motorID = parameter[PITCH_TYPE];    //获取pitch轴电机ID
      yawMotorData.motorID = parameter[YAW_TYPE];        //获取taw轴电机ID
			robotConfigData.robotDeviceList = UAV_DEVICE_LIST;
		}break;
		
		case SMALLGIMBAL_ID:{										//小云台ID
			parameter[WEAPON_TYPE] = DOUBLE_LAUNCHER;
				parameter[CHASSIS_CURRENT] = CHASSIS_NO_LIMIT;
				parameter[PITCH_MIN_RANGE] = SMALLGIMBAL_PITCH_MIN_RANGE;
				parameter[PITCH_MAX_RANGE] = SMALLGIMBAL_PITCH_MAX_RANGE;	
				parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//获取云台安装方向
				parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN];
				pitchMotorData.motorID = parameter[PITCH_TYPE];    //获取pitch轴电机ID
        yawMotorData.motorID = parameter[YAW_TYPE];        //获取taw轴电机ID
				robotConfigData.robotDeviceList = SMALLGIMBAL_LIST;
		}break;
		
		default: break;
	}
	parameterReadDataFromTFCard(robotConfigData.typeOfRobot);					//从TF卡中读取对应数据
	robotConfigData.distinguishState = ROBOT_COMPLETE_IDENTIFIED;			//表示识别完毕
}

static void identifyTypeRobotBeepUpdate(uint8_t typeRobot){
	switch(typeRobot){
		case INFANTRY_ID: supervisorData.beepState = MUSIC_TYPE_INFANTRY; break;
		case TANK_ID:	supervisorData.beepState = MUSIC_TYPE_TANK; break;
		case AUXILIARY_ID:	supervisorData.beepState = MUSIC_TYPE_AUXILIARY; break;
		case SENTRY_ID:	supervisorData.beepState = MUSIC_TYPE_SENTRY; break;
		case UAV_ID: 	supervisorData.beepState = MUSIC_TYPE_UAV; break;
		case SMALLGIMBAL_ID: 	supervisorData.beepState = MUSIC_TYPE_SMALLGIMBAL; break;
		
	}
}

void identifyTheCurrentRobot(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1){
		vTaskDelayUntil(&xLastWakeTime,TYPE_ROBOT_PERIOD);																//10Hz
		if(RC_PITCH > 490 && parameterRunData.TFInsertState == TFCARD_INSERT){						//必须插入TF卡才能进行机器人识别																			//pitch轴摇杆必须保持拉高
			robotConfigData.distinguishLever = RC_GEAR;
			if(robotConfigData.distinguishLastLever != robotConfigData.distinguishLever){		//当摇杆不同于上次，计数一次
				digitalIncreasing(&robotConfigData.typeOfRobot);							//机器人种类往上加一
				if(robotConfigData.typeOfRobot > SMALLGIMBAL_ID)      				//大于小云台的ID就重新计数
					robotConfigData.typeOfRobot = INFANTRY_ID;
			identifyTypeRobotBeepUpdate(robotConfigData.typeOfRobot);				//机器人种类报数
			}
			robotConfigData.distinguishLastLever = RC_GEAR;
		}
		else if(RC_PITCH < -490){																					//如果下拉摇杆却没有ID，就重新再来
			if(robotConfigData.typeOfRobot != NO_ID){
				parameter[ROBOT_TYPE] = robotConfigData.typeOfRobot;					//储存机器人种类
				robotConfigData.distinguishState = ROBOT_COMPLETE_IDENTIFIED;	//表示识别完毕
				parameterReadDataFromTFCard(parameter[ROBOT_TYPE]);           //从TF卡里读PID参数
        motorMessageReadDataFromTFCard(parameter[ROBOT_TYPE]);        //从TF卡里面读电机配置参数
	      readMotormessage();
				currentRobotParameterConfig();																//配置不同机器人所对应的默认参数
				digitalHi(&supervisorData.flashSave);													//开启储存到flash
				vTaskDelete(robotConfigData.xHandleTask);											//删除本任务
			}
		}
		else{
			robotConfigData.distinguishLastLever = RC_GEAR;									//其他情况不记录
		}
		digitalIncreasing(&robotConfigData.loops);
	}
}

void robotDistinguish(void){
	digitalLo(&controlData.dataInitFlag);
	robotConfigData.typeOfRobot = parameter[ROBOT_TYPE];								//从flash内拉取种类信息
	if(robotConfigData.typeOfRobot == NO_ID){														//没有ID什么都不干，直接进识别任务
		robotConfigData.distinguishState = ROBOT_BEING_IDENTIFIED;				//表示正在识别
		robotConfigData.distinguishLastLever = RC_GEAR;										//储存一次摇杆值
		supervisorData.taskEvent[TYPE_ROBOT_TASK] = xTaskCreate(identifyTheCurrentRobot,"TYPE_ROBOT",TYPE_ROBOT_STACK_SIZE,NULL,TYPE_ROBOT_PRIORITY,&robotConfigData.xHandleTask);
	}
	else{																																//如果存在ID则直接配置
		currentRobotParameterConfig();
		robotConfigData.distinguishState = ROBOT_COMPLETE_IDENTIFIED;			//表示识别完毕
	}
}

