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
		case INFANTRY_ID:{							//����ID
			parameter[WEAPON_TYPE] = SMALL_LAUNCHER;												//��������
			parameter[CHASSIS_CURRENT] = INFANTR_CHASSIS_CURRENT;						//��������������
			parameter[PITCH_MIN_RANGE] = INFANTRY_PITCH_MIN_RANGE;					//��С������(��̨̧��Ϊ����)
			parameter[PITCH_MAX_RANGE] = INFANTRY_PITCH_MAX_RANGE;					//�������
			parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//��ȡ��̨��װ����
			parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN];      
			pitchMotorData.motorID = parameter[PITCH_TYPE];    //��ȡpitch����ID
      yawMotorData.motorID = parameter[YAW_TYPE];        //��ȡtaw����ID
			//�����������Ʋ�������
			powerData.powerLimit = INFANTR_POWER_LIMIT;
			powerData.warningPower = INFANTR_WARNING_POWER;
			powerData.WarningPowerBuff = INFANTR_WARNING_POWER_BUFF;
			powerData.noJudgeTotalCurrentLimit = INFANTR_NO_JUDGE_TOTAL_CURRENT_LIMIT;
			powerData.judgeTotalCurrentLimit = INFANTR_JUDGE_TOTAL_CURRENT_LIMIT;
			powerData.addPowerCurrent = INFANTR_ADD_POWER_CURRENT;
			robotConfigData.robotDeviceList = INFANTRY_DEVICE_LIST;
		}break;

		case TANK_ID:{									//Ӣ�۳�ID
				parameter[WEAPON_TYPE] = DOUBLE_LAUNCHER;
				parameter[CHASSIS_CURRENT] = TANK_CHASSIS_CURRENT;	
				parameter[PITCH_MIN_RANGE] = TANK_PITCH_MIN_RANGE;
				parameter[PITCH_MAX_RANGE] = TANK_PITCH_MAX_RANGE;	
				parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//��ȡ��̨��װ����
				parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN]; 
				pitchMotorData.motorID = parameter[PITCH_TYPE];    //��ȡpitch����ID
        yawMotorData.motorID = parameter[YAW_TYPE];        //��ȡtaw����ID
				//Ӣ�۹������Ʋ�������
			  powerData.powerLimit = TANK_POWER_LIMIT;
				powerData.warningPower = TANK_WARNING_POWER;
				powerData.WarningPowerBuff = TANK_WARNING_POWER_BUFF;
				powerData.noJudgeTotalCurrentLimit = TANK_NO_JUDGE_TOTAL_CURRENT_LIMIT;
				powerData.judgeTotalCurrentLimit = TANK_JUDGE_TOTAL_CURRENT_LIMIT;
				powerData.addPowerCurrent = TANK_ADD_POWER_CURRENT;
				robotConfigData.robotDeviceList = TANK_DEVICE_LIST;
		}break;
		
		case AUXILIARY_ID:{							//���̳�ID
			parameter[WEAPON_TYPE] = NO_WEAPONS;
			parameter[CHASSIS_CURRENT] = CHASSIS_NO_LIMIT;	
			parameter[PITCH_MIN_RANGE] = AUXILIARY_PITCH_MIN_RANGE;
			parameter[PITCH_MAX_RANGE] = AUXILIARY_PITCH_MAX_RANGE;		
			parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//��ȡ��̨��װ����
			parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN];	
			pitchMotorData.motorID = parameter[PITCH_TYPE];    //��ȡpitch����ID
      yawMotorData.motorID = parameter[YAW_TYPE];        //��ȡtaw����ID
			robotConfigData.robotDeviceList = AUXILIARY_DEVICE_LIST;
		}break;
		
		case SENTRY_ID:{								//�ڱ�ID
			parameter[WEAPON_TYPE] = SMALL_LAUNCHER;
			parameter[CHASSIS_CURRENT] = SENTRY_CHASSIS_CURRENT;	
			parameter[PITCH_MIN_RANGE] = SENTRY_PITCH_MIN_RANGE;
			parameter[PITCH_MAX_RANGE] = SENTRY_PITCH_MAX_RANGE;	
			parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//��ȡ��̨��װ����
			parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN];
			pitchMotorData.motorID = parameter[PITCH_TYPE];    //��ȡpitch����ID
      yawMotorData.motorID = parameter[YAW_TYPE];        //��ȡtaw����ID
			//�ڱ��������Ʋ�������
			powerData.powerLimit = SENTRY_POWER_LIMIT;
			powerData.warningPower = SENTRY_WARNING_POWER;
			powerData.WarningPowerBuff = SENTRY_WARNING_POWER_BUFF;
			powerData.noJudgeTotalCurrentLimit = SENTRY_NO_JUDGE_TOTAL_CURRENT_LIMIT;
			powerData.judgeTotalCurrentLimit = SENTRY_JUDGE_TOTAL_CURRENT_LIMIT;
			powerData.addPowerCurrent = SENTRY_ADD_POWER_CURRENT;
			robotConfigData.robotDeviceList = SENTRY_DEVICE_LIST;
		}break;
		
		case UAV_ID:{										//���˻�ID
			parameter[WEAPON_TYPE] = SMALL_LAUNCHER;
			parameter[CHASSIS_CURRENT] = CHASSIS_NO_LIMIT;	
			parameter[PITCH_MIN_RANGE] = UAV_PITCH_MIN_RANGE;
			parameter[PITCH_MAX_RANGE] = UAV_PITCH_MAX_RANGE;	
			parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//��ȡ��̨��װ����
			parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN];	
			pitchMotorData.motorID = parameter[PITCH_TYPE];    //��ȡpitch����ID
      yawMotorData.motorID = parameter[YAW_TYPE];        //��ȡtaw����ID
			robotConfigData.robotDeviceList = UAV_DEVICE_LIST;
		}break;
		
		case SMALLGIMBAL_ID:{										//С��̨ID
			parameter[WEAPON_TYPE] = DOUBLE_LAUNCHER;
				parameter[CHASSIS_CURRENT] = CHASSIS_NO_LIMIT;
				parameter[PITCH_MIN_RANGE] = SMALLGIMBAL_PITCH_MIN_RANGE;
				parameter[PITCH_MAX_RANGE] = SMALLGIMBAL_PITCH_MAX_RANGE;	
				parameter[PITCH_INSTALL] = parameter[PITCH_FIX] + parameter[PITCH_TURN];		//��ȡ��̨��װ����
				parameter[YAW_INSTALL] = parameter[YAW_FIX] + parameter[YAW_TURN];
				pitchMotorData.motorID = parameter[PITCH_TYPE];    //��ȡpitch����ID
        yawMotorData.motorID = parameter[YAW_TYPE];        //��ȡtaw����ID
				robotConfigData.robotDeviceList = SMALLGIMBAL_LIST;
		}break;
		
		default: break;
	}
	parameterReadDataFromTFCard(robotConfigData.typeOfRobot);					//��TF���ж�ȡ��Ӧ����
	robotConfigData.distinguishState = ROBOT_COMPLETE_IDENTIFIED;			//��ʾʶ�����
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
		if(RC_PITCH > 490 && parameterRunData.TFInsertState == TFCARD_INSERT){						//�������TF�����ܽ��л�����ʶ��																			//pitch��ҡ�˱��뱣������
			robotConfigData.distinguishLever = RC_GEAR;
			if(robotConfigData.distinguishLastLever != robotConfigData.distinguishLever){		//��ҡ�˲�ͬ���ϴΣ�����һ��
				digitalIncreasing(&robotConfigData.typeOfRobot);							//�������������ϼ�һ
				if(robotConfigData.typeOfRobot > SMALLGIMBAL_ID)      				//����С��̨��ID�����¼���
					robotConfigData.typeOfRobot = INFANTRY_ID;
			identifyTypeRobotBeepUpdate(robotConfigData.typeOfRobot);				//���������౨��
			}
			robotConfigData.distinguishLastLever = RC_GEAR;
		}
		else if(RC_PITCH < -490){																					//�������ҡ��ȴû��ID������������
			if(robotConfigData.typeOfRobot != NO_ID){
				parameter[ROBOT_TYPE] = robotConfigData.typeOfRobot;					//�������������
				robotConfigData.distinguishState = ROBOT_COMPLETE_IDENTIFIED;	//��ʾʶ�����
				parameterReadDataFromTFCard(parameter[ROBOT_TYPE]);           //��TF�����PID����
        motorMessageReadDataFromTFCard(parameter[ROBOT_TYPE]);        //��TF�������������ò���
	      readMotormessage();
				currentRobotParameterConfig();																//���ò�ͬ����������Ӧ��Ĭ�ϲ���
				digitalHi(&supervisorData.flashSave);													//�������浽flash
				vTaskDelete(robotConfigData.xHandleTask);											//ɾ��������
			}
		}
		else{
			robotConfigData.distinguishLastLever = RC_GEAR;									//�����������¼
		}
		digitalIncreasing(&robotConfigData.loops);
	}
}

void robotDistinguish(void){
	digitalLo(&controlData.dataInitFlag);
	robotConfigData.typeOfRobot = parameter[ROBOT_TYPE];								//��flash����ȡ������Ϣ
	if(robotConfigData.typeOfRobot == NO_ID){														//û��IDʲô�����ɣ�ֱ�ӽ�ʶ������
		robotConfigData.distinguishState = ROBOT_BEING_IDENTIFIED;				//��ʾ����ʶ��
		robotConfigData.distinguishLastLever = RC_GEAR;										//����һ��ҡ��ֵ
		supervisorData.taskEvent[TYPE_ROBOT_TASK] = xTaskCreate(identifyTheCurrentRobot,"TYPE_ROBOT",TYPE_ROBOT_STACK_SIZE,NULL,TYPE_ROBOT_PRIORITY,&robotConfigData.xHandleTask);
	}
	else{																																//�������ID��ֱ������
		currentRobotParameterConfig();
		robotConfigData.distinguishState = ROBOT_COMPLETE_IDENTIFIED;			//��ʾʶ�����
	}
}

