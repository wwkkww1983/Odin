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
	if(parameterRunData.TFInsertState == TFCARD_INSERT){								//����������
		supervisorData.tfState = ENABLE;				
		if(parameterRunData.TFInsertFlag){																//�ղ���������
			parameterRunData.TFError = tfFATFS_Init();											//����Ƿ����
			parameterRunData.TFInsertFlag = DISABLE;
			parameterReadDataFromTFCard(robotConfigData.typeOfRobot);				//�������ݵ�parameter	
		}
	}
	else{
		supervisorData.tfState = DISABLE;
		if(parameterRunData.TFInsertFlag)
			parameterRunData.TFError = DISABLE;															//���������
	}
	parameterRunData.TFInsertLastState = parameterRunData.TFInsertState;
}

uint8_t parameterWriteDataFormFlash(uint8_t robotId){               //��flash�е�PID����д��TF���еĲ����ļ�
	uint8_t answer;
	if(robotId != NO_ID){
		answer = tfOverwrite(robotId,configParameterStrings,parameter,TFCARD_NUM_LIST);
		return answer;
	}
	else
		return 1;																													//д��ʧ��
}

uint8_t parameterReadDataFromTFCard(uint8_t robotId){               //��TF���ж�ȡPID����
	uint8_t answer;
	if(robotId != NO_ID){
		answer = tfOverread(robotId,configParameterStrings,parameter,TFCARD_NUM_LIST);
		return answer;
	}
	else
		return 1;																													//д��ʧ��
}

