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

void lightBarsReportErrors(void){																			//���ڱ���
	uint8_t index = 0;
	uint8_t nowFaultBit = 0; 
	uint16_t remainFaultSate; 
	if(warningData.lightBarsState.u16_temp != 0){
		remainFaultSate = warningData.lightBarsState.u16_temp;
		while(index < SK6812_LED_STRIP_LENGTH && !warningData.highestFaultFlag){		//���ڼ�����һ���Ĵ����
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
	if(warningData.highestFault != 0){																	//�ڵ�һλ����ߴ���λ����һ������Ч�������ڰ�������
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

void lightBarsOfContrl(uint8_t contrlMode,uint8_t safeMode){					//���ڿ���״̬
	uint8_t index = 0;
	uint8_t invertFreq = 0;
	static uint8_t lightBarsSwitch = ENABLE;
	switch(contrlMode){																									//����ģʽ
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
	switch(safeMode){																										//ʶ�𰲱��ȼ�
		case SAFE:
			warningData.blinkFrequency = 1;
			warningData.displayColor = SK6812_GREEN;
			break;
		/**********�޾���ģʽ���δ˴�*****************
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
		if(invertFreq == 10)//��ȫģʽ����
			lightBarsSwitch = ENABLE;
	}
	if(!lightBarsSwitch){
		warningData.displayColor = SK6812_DARK;
	}
	
	//��������ʾ��Ƶ������������������
	while(index < SK6812_LED_STRIP_LENGTH){
		if(index < warningData.displayNumber){
			setOneLedHsv(index,&warningData.displayColor);
		}
		else{
			setOneLedHsv(index,&SK6812_DARK);
		}
		index ++;
	}
	//�Ҳ����������ʾŤ��,С����
	if(autoTaskData->aviodFlag)
		warningData.displayColor = SK6812_RED;
	else if(infantryAutoData.rotateFlag)
		warningData.displayColor = SK6812_GREEN;
	else
		warningData.displayColor = SK6812_DARK;
	setOneLedHsv(SK6812_LED_STRIP_LENGTH - 3,&warningData.displayColor);
	//�Ҳ�ڶ�������������ʾ����״̬
	switch(autoTaskData->currentTask){																	//���������Ҳ�ĵ���������ʾ����״̬
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
	//���ұߵĵ���ʾ����״̬
	//���ݳ��
	bool CapLight = false;
	if(powerData.chargeFlag){
		if(CAP_SOC > 95.0f){
			//���糤��
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
	//���ݷŵ�
	else{
		if(CAP_SOC < 10.0f){
			//���ݼ���û�磬����
			warningData.capSoc = 0x02;
			warningData.displayColor = SK6812_RED;
		}
		else{
			//��ȫ����
			warningData.capSoc = 0x03;
			warningData.displayColor = SK6812_GREEN;
		}
		CapLight = true;
	}
	//������ʱ�Ʋ���
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

void lightBarsStateSwitch(uint16_t state,uint8_t valve){							//�����л�����״̬��״̬��																														
	if (valve)
		warningData.lightBarsState.u16_temp |= state;											//�Ѷ�Ӧλ��1
	else
		warningData.lightBarsState.u16_temp &= ~state;										//�Ѷ�Ӧλ��0	
}

void lightBarsErrorCheck(void){																				//���ڼ�鵱ǰ����״̬
	if(supervisorData.state & STATE_RADIO_LOSS){												//(1)���ң�����Ƿ�ʧ  0x0001
		lightBarsStateSwitch(RC_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(RC_FAULT,DISABLE);
	}
	if(supervisorData.state & STATE_JUDGE_ERROR){												//(2)������ϵͳ�Ƿ����  0x0002
		lightBarsStateSwitch(JUDGE_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(JUDGE_FAULT,DISABLE);
	}
	if(supervisorData.state & STATE_MOTOR_ERROR){												//(3)��⶯��ϵͳ����̨����Ƿ����  0x0004
		lightBarsStateSwitch(MOTOR_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(MOTOR_FAULT,DISABLE);
	}
	if(supervisorData.state & STATE_SENSOR_ERROR){											//(4)�����̨���Ƿ����  0x0008
		lightBarsStateSwitch(SENSOR_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(SENSOR_FAULT,DISABLE);
	}
	if(supervisorData.state & STATE_VISION_ERROR){											//(5)���TF���Ƿ����  0x0010
		lightBarsStateSwitch(VISION_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(VISION_FAULT,DISABLE);
	}
	if(robotConfigData.robotDeviceList & DEVICE_CURRENT \
		&& powerData.CapError){																						//(6)�����ݰ��Ƿ����  0x0020
		lightBarsStateSwitch(POWER_LIMIT_FAULT,ENABLE);
	}
	else{
		lightBarsStateSwitch(POWER_LIMIT_FAULT,DISABLE);
	}
}

void lightBarsUpdate(void){																						//�ƴ�״̬����
	lightBarsErrorCheck();																							//�������״̬
	if(!(supervisorData.state & STATE_RADIO_LOSS)){											//���������ң�ص�״̬	û�ж���	
		if(supervisorData.state & STATE_DISARMED || remoteControlData.dt7Value.keyBoard.bit.CTRL){
			lightBarsReportErrors();																				//û�н�������Z��ʾ����״̬
			warningData.reportError = 0x00;
		}
		else{																															//�����ӵ�п���Ȩ�������
			lightBarsStateUpdata();
			warningData.reportError = 0x01;
			switch(robotConfigData.typeOfRobot){
				case INFANTRY_ID:
					lightBarsOfContrl(shootData.shootMode,shootData.shootStatusMode); 
					break;
				case TANK_ID:
					lightBarsOfContrl(shootData.shootMode,shootData.shootStatusMode);
					break;
				case AUXILIARY_ID:																						//���̳����˻����滹�����ǵ�����		
					break;
				case SENTRY_ID:																								//�ڱ����õƴ�
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
	
	//���ݵ����ٷֱȣ�17vΪ������ѹ
		judgeData.extShowData.data2 = (float)(100 * (powerData.capVol - 17) / (powerData.capVolMax - 17));
	//û����Ħ���ֵƲ���
	if(!shootData.fricMotorFlag)	
		//���ȫ��		
		judgeData.extShowData.mask = 0x00;									
	else{
		if(shootData.shootMode == MANUAL_SINGLE){	
			//����1�ŵ�
			judgeData.extShowData.mask |= 0x01;
			judgeData.extShowData.mask &= 0xF9;
		}
		else if(shootData.shootMode == MANUAL_CONTINUOUS){
			//������2�ŵ�
			judgeData.extShowData.mask |= 0x03;
			judgeData.extShowData.mask &= 0xFB;
		}
		else if(shootData.shootMode == AUTO_CONTINUOUS){
			//�������ŵ�
			judgeData.extShowData.mask |= 0x07;
		}
		if(infantryAutoData.rotateFlag)
      //С����ģʽ			
			judgeData.extShowData.mask |= 0x08;
		else
			judgeData.extShowData.mask &= 0xF7;
		
		if(infantryAutoData.aviodFlag)
			//Ť��ģʽ			
			judgeData.extShowData.mask |= 0x10;
		else
			judgeData.extShowData.mask &= 0xEF;
			//�����ݷŵ�
		if(openCap)							   													
			judgeData.extShowData.mask |= 0x20;
		else
			judgeData.extShowData.mask &= 0xDF;
	}
}

void warningUpdate(void){
	colorStdInit();
  lightBarsUpdate();																									//sk6812 ����
	lightBarsJudgeUpdate();																							//����ϵͳ�������ݸ���
	ledUpdateTask(supervisorData.ledState);                           	//LED����		���ذ���3ɫLED����ָʾ״̬
	beepUpdateTask(supervisorData.beepState);												 		//��������������
	digitalClan(&supervisorData.beepState);													  	//״̬����
	digitalIncreasing(&warningData.loops);
}
