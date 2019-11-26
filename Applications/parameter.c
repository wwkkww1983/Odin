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

