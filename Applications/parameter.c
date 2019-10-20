#include "bsp.h"
#include "supervisor.h"
#include "parameter.h"
#include "config.h"
#include "type_robot.h"
#include "tf_card_parameter.h"

parameterStruct_t parameterRunData;

void tFCardUpdate(void){
	parameterRunData.TFInsertState = TFCARD_INSERT_IO;
	if(parameterRunData.TFInsertLastState != parameterRunData.TFInsertState)
		parameterRunData.TFInsertFlag = ENABLE;
	if(parameterRunData.TFInsertState == TFCARD_INSERT){								//插入的情况下
		supervisorData.tfState = ENABLE;				
		if(parameterRunData.TFInsertFlag){																//刚插入的情况下
			parameterRunData.TFError = tfFATFS_Init();											//检测是否出错
			parameterRunData.TFInsertFlag = DISABLE;
			parameterReadDataFromTFCard(robotConfigData.typeOfRobot);				//读出数据到parameter	
		}
	}
	else{
		supervisorData.tfState = DISABLE;
		if(parameterRunData.TFInsertFlag)
			parameterRunData.TFError = DISABLE;															//出错检测归零
	}
	parameterRunData.TFInsertLastState = parameterRunData.TFInsertState;
}

uint8_t parameterWriteDataFormFlash(uint8_t robotId){               //把flash中的PID参数写入TF卡中的参数文件
	uint8_t answer;
	if(robotId != NO_ID){
		answer = tfOverwrite(robotId,configParameterStrings,parameter,TFCARD_NUM_LIST);
		return answer;
	}
	else
		return 1;																													//写入失败
}

uint8_t parameterReadDataFromTFCard(uint8_t robotId){               //从TF卡中读取PID参数
	uint8_t answer;
	if(robotId != NO_ID){
		answer = tfOverread(robotId,configParameterStrings,parameter,TFCARD_NUM_LIST);
		return answer;
	}
	else
		return 1;																													//写入失败
}

uint8_t motorMessageReadDataFromTFCard(uint8_t robotId){             //从TF卡里读出电机配置信息      
	uint8_t answer;                                                     
	if(robotId != NO_ID){
		answer = tfMotorread(robotId,configMotorStrings,motormessage,ABOVE_TOTAL);
		return answer;
	}
	else
		return 1;																													//写入失败
}

uint8_t motorMessageWriteDataFormFlash(uint8_t robotId){             //把flash的电机配置参数写入TF卡中的电机配置文件
	uint8_t answer;                                                      
	if(robotId != NO_ID){
		answer = tfMotorwrite(robotId,configMotorStrings,motormessage,ABOVE_TOTAL);
		return answer;
	}
	else
		return 1;																													//写入失败
}

void writeMotormessage(void){                                        //将从flash里面读出来数据写到TF卡的数组里面
	motormessage[ROBOT_ID] = parameter[ROBOT_TYPE];
  motormessage[NATIVE_ID] = parameter[LOCAL_ID];											
	motormessage[ARMS_TYPE] = parameter[WEAPON_TYPE];								
	motormessage[PITCH_ORIENT] = parameter[PITCH_INSTALL];				
	motormessage[YAW_ORIENT] = parameter[YAW_INSTALL];			
	motormessage[RETURN_TIME] = parameter[BACK_CENTER_TIME];			
	motormessage[CHASSIS_POWER] = parameter[CHASSIS_CURRENT];				
	motormessage[RC_RATIO] = parameter[RC_RESOLUTION];						
	motormessage[YAW_MIDDLE] = parameter[YAW_CENTER];								
	motormessage[PITCH_MIDDLE] = parameter[PITCH_CENTER];														
	motormessage[PITCH_MIN_SEAT] = parameter[PITCH_MIN_RANGE];				
	motormessage[PITCH_MAX_SEAT] = parameter[PITCH_MAX_RANGE];
  motormessage[YAW_ID] = parameter[YAW_TYPE];
	motormessage[PITCH_ID] = parameter[PITCH_TYPE]; 
	motormessage[YAW_FASTEN] = parameter[YAW_FIX];
	motormessage[YAW_SPIN] = parameter[YAW_TURN];
	motormessage[PITCH_FASTEN] = parameter[PITCH_FIX]; 
	motormessage[PITCH_SPIN] = parameter[PITCH_TURN];
	motormessage[IMU_ACC_INIT_X] = parameter[IMU_ACC_BIAS_X];					
	motormessage[IMU_ACC_INIT_Y] = parameter[IMU_ACC_BIAS_Y];					
	motormessage[IMU_ACC_INIT_Z] = parameter[IMU_ACC_BIAS_Z];					
	motormessage[IMU_MAG_INIT_X] = parameter[IMU_MAG_BIAS_X];					
	motormessage[IMU_MAG_INIT_Y] = parameter[IMU_MAG_BIAS_Y];					
	motormessage[IMU_MAG_INIT_Z] = parameter[IMU_MAG_BIAS_Z];					
	motormessage[IMU_GYO_INIT_X] = parameter[IMU_GYO_BIAS_X];					
	motormessage[IMU_GYO_INIT_Y] = parameter[IMU_GYO_BIAS_Y];					
	motormessage[IMU_GYO_INIT_Z] = parameter[IMU_GYO_BIAS_Z];	
	
}

void readMotormessage(void){                                        //将从TF卡里面读出来数据写到flash的数组里面
	parameter[ROBOT_TYPE] = motormessage[ROBOT_ID];
  parameter[LOCAL_ID] = motormessage[NATIVE_ID];											
	parameter[WEAPON_TYPE] = motormessage[ARMS_TYPE];								
	parameter[PITCH_INSTALL] = motormessage[PITCH_ORIENT];				
	parameter[YAW_INSTALL] = motormessage[YAW_ORIENT];			
	parameter[BACK_CENTER_TIME] = motormessage[RETURN_TIME];			
	parameter[CHASSIS_CURRENT] = motormessage[CHASSIS_POWER];				
	parameter[RC_RESOLUTION] = motormessage[RC_RATIO];						
	parameter[YAW_CENTER] = motormessage[YAW_MIDDLE];								
	parameter[PITCH_CENTER] = motormessage[PITCH_MIDDLE];														
	parameter[PITCH_MIN_RANGE] = motormessage[PITCH_MIN_SEAT];				
	parameter[PITCH_MAX_RANGE] = motormessage[PITCH_MAX_SEAT];
  parameter[YAW_TYPE] = motormessage[YAW_ID];
	parameter[PITCH_TYPE] = motormessage[PITCH_ID]; 
	parameter[YAW_FIX] = motormessage[YAW_FASTEN];
	parameter[YAW_TURN] = motormessage[YAW_SPIN];
	parameter[PITCH_FIX] = motormessage[PITCH_FASTEN]; 
	parameter[PITCH_TURN] = motormessage[PITCH_SPIN];
	parameter[IMU_ACC_BIAS_X] = motormessage[IMU_ACC_INIT_X];					
	parameter[IMU_ACC_BIAS_Y] = motormessage[IMU_ACC_INIT_Y];					
	parameter[IMU_ACC_BIAS_Z] = motormessage[IMU_ACC_INIT_Z];					
	parameter[IMU_MAG_BIAS_X] = motormessage[IMU_MAG_INIT_X];					
	parameter[IMU_MAG_BIAS_Y] = motormessage[IMU_MAG_INIT_Y];					
	parameter[IMU_MAG_BIAS_Z] = motormessage[IMU_MAG_INIT_Z];					
	parameter[IMU_GYO_BIAS_X] = motormessage[IMU_GYO_INIT_X];					
	parameter[IMU_GYO_BIAS_Y] = motormessage[IMU_GYO_INIT_Y];					
	parameter[IMU_GYO_BIAS_Z] = motormessage[IMU_GYO_INIT_Z];	
	
}

