#include "config.h"
#include "control.h"
#include "rc.h"
#include "imu.h"
#include "keyboard.h"
#include "gimbal.h"
#include "vision.h"
#include "chassis.h"
#include "supervisor.h"
#include "Driver_RMMotor.h"

remoteControlStruct_t remoteControlData;

void rcSbusScale(sbusStruct_t *raw,sbusStruct_t *real){					//�Ѿ���ֵ��660ת����500�ĺ���
	uint8_t i;
	float ch[CH_NUMBER];
	for(i = 0;i < CH_NUMBER;i++){  
		ch[i] = (float)*(&(raw->CH0)+i) / 660 * 500;
		ch[i] = constrainFloat(ch[i],-500,500);
		*(&(real->CH0) + i) = (int16_t)ch[i];
	}
}

void rcDt7Scale(dt7Struct_t *raw,dt7Struct_t *real){						//�Ѿ���ֵ��660ת����500�ĺ���
	uint8_t i;
	float ch[CH_NUMBER];
	for(i = 0;i < CH_NUMBER;i++){  
		ch[i] = (float)*(&(raw->CH0)+i) / 660 * 500;		
		ch[i] = constrainFloat(ch[i],-500.0f,500.0f);
		*(&(real->CH0) + i) = (int16_t)ch[i];
	}
	real->S1 = raw->S1;
	real->S2 = raw->S2;
}

static void chassisOperationFunc(int16_t forwardBack, int16_t leftRight, int16_t rotate){
	if(robotMode == MODE_RC){
		//�����ٶȸ�ֵ,ҡ�����ֵ��Ӧ�����ٶ����ֵ		
		remoteControlData.chassisSpeedTarget.x = leftRight / parameter[RC_RESOLUTION] * \
												 parameter[CHASSIS_RC_SPEED] * REAL_MOTOR_SPEED_SCALE;
		remoteControlData.chassisSpeedTarget.y = forwardBack / parameter[RC_RESOLUTION] *\
												 parameter[CHASSIS_RC_SPEED] * REAL_MOTOR_SPEED_SCALE;
		remoteControlData.chassisSpeedTarget.z = rotate / parameter[RC_RESOLUTION] * \
												 parameter[CHASSIS_RC_SPEED] * REAL_MOTOR_SPEED_SCALE;
	}
	else{
		remoteControlData.chassisSpeedTarget.x = 0;
		remoteControlData.chassisSpeedTarget.y = 0;
		remoteControlData.chassisSpeedTarget.z = 0;	
	}
}

void remoteCtrlChassisHook(void){
  chassisOperationFunc(RC_LONGITUDINAL, RC_TRANSVERSE, RC_RUDD);
}

static void gimbalOperationFunc(int16_t pitCtrl, int16_t yawCtrl){//��̨�ٶ�ҡ�˸�ֵ
  remoteControlData.pitchGyroTarget =  pitCtrl * parameter[GIMBAL_CTR_SCALE] * 2;
  remoteControlData.yawGyroTarget   =  yawCtrl * parameter[GIMBAL_CTR_SCALE] * 2;
}

void remoteCtrlGimbalHook(void){
  gimbalOperationFunc(RC_PITCH, RC_RUDD);
}

void getGimbalCtrlDate(void){										//��ȡ��̨�������� 
	if(pitchMotorData.motorID < MOTOR_RL7015)
		gimbalData.pitchMotorAngle = ENCODER_ANGLE_RATIO13 * getRelativePos(gimbal_chooseData(CODEBOARD_VALUE,&pitchMotorData),parameter[PITCH_CENTER],&pitchMotorData);
	else
		gimbalData.pitchMotorAngle = ENCODER_ANGLE_RATIO14 * getRelativePos(gimbal_chooseData(CODEBOARD_VALUE,&pitchMotorData),parameter[PITCH_CENTER],&pitchMotorData);//��̨����ֵ��������
	if(yawMotorData.motorID < MOTOR_RL7015)
		gimbalData.yawMotorAngle = ENCODER_ANGLE_RATIO13 * getRelativePos(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData),parameter[YAW_CENTER],&yawMotorData);
	else
		gimbalData.yawMotorAngle = ENCODER_ANGLE_RATIO14 * getRelativePos(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData),parameter[YAW_CENTER],&yawMotorData);
	gimbalData.pitchGyroAngle = AQ_PITCH;         //pitch��Ƕ�
	gimbalData.yawGyroAngle   = AQ_YAW;		 			  //yaw��Ƕ�
	keyboardGimbalHook();                 				//��̨���̲������ݴ���
	remoteCtrlGimbalHook();	              				//��̨ҡ�˲������ݴ���
}

void getChassisCtrlDate(void){									//��ȡ���̿�������
  keyboardChassisHook();                        //���̼��̲������ݴ���
  remoteCtrlChassisHook();                      //����ҡ�˲������ݴ���
}

void rcUpdateTask(void){
#if UAV_SBUS 
	if(remoteControlData.rcIspReady){
		Driver_SBUS_Decode_RemoteData(&remoteControlData.sbusValue.rcRawData,Array_USART1_RX);
		digitalLo(&remoteControlData.rcIspReady);
	}
	rcSbusScale(&remoteControlData.sbusValue.rcRawData,&remoteControlData.sbusValue.rcRealData);
#else
	if(remoteControlData.rcIspReady){
		Driver_RMDT7_Decode_RemoteData(&remoteControlData.dt7Value,Array_USART1_RX);
		digitalLo(&remoteControlData.rcIspReady);
	}
	rcDt7Scale(&remoteControlData.dt7Value.rcRawData,&remoteControlData.dt7Value.rcRealData);
#endif
	keyBoardCtrlData.leftKeyState  = leftKeyDriver();
	keyBoardCtrlData.rightKeyState = rightKeyDriver();		
	getKeyboardMouseState();
	getChassisCtrlDate();		
	digitalIncreasing(&remoteControlData.loops);
}
void rcInit(void){
	Driver_RMDT7_Init(SBUS_USARTX,SBUS_USARTX_RX_PIN,SBUS_USART_PRE_PRIORITY,SBUS_USART_SUB_PRIORITY);
	remoteControlData.initFlag = true;
}
 



