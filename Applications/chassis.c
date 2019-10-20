#include "chassis.h"
#include "gimbal.h"
#include "rc.h"
#include "keyboard.h"
#include "config.h"
#include "Driver_RMMotor.h"
#include "judge.h"
#include "cansend.h"
#include "control.h"
#include "auto_auxiliary.h"
#include "auto_infantry.h"
#include "imu.h"

//#define CHASSIS_SPEED_ADRC
#define ANTI_SLIP
#define CHASSIS_SPEED_PID
#define ADD_IMU_RATE_PID

chassisStruct_t chassisData;
/*
***************************************************
�� �� ����	chassisDataUpdate
��		�ܣ������������
��ڲ�����	chassisData.autoSpeedTarget ������ת�˶��ٶ�Ŀ��(�ṹ�壬�������и�ֵx,y,z)
					chassisData.autoMode���л��Զ������־
�� �� ֵ��	��
Ӧ�÷�Χ��	�ⲿ����
��		ע��
**************************************************
*/
void chassisUpdate(void){
	if(imuSensorOfChassisData.initFlag){
		//��ȡ��������������
		imuChassisUpdate();																									
	}
//	powerRealData(&powerRawDate,&powerRealDate);
	//���³���ʵ������ٶ��޷�
	chassisData.speedLimitFloor = chassisData.speedLimit * parameter[CHASSIS_RC_SPEED]*REAL_MOTOR_SPEED_SCALE;
	for (uint8_t i = 0; i < 4; i++) {
		//�����ٶȷ�����ֵ
		chassisData.speedFbd[i] = (float)wheelData[i].speed * REAL_MOTOR_SPEED_SCALE;
		//ɨ�������������ٶ�
		if(fabs(chassisData.speedFbd[i]) >= chassisData.speedFbdMax)
			chassisData.speedFbdMax = fabs(chassisData.speedFbd[i]);
	}
	
#ifdef ANTI_SLIP
	for(uint8_t index = 0; index < NUMBER_OF_WHEEL; index++){	
		//����ʱ��Ϊ����
		if(fabs(chassisData.speedRef[index]) < 0.2f)
			chassisData.scale[index] = 1.0f;
		else
			//�õ���ǰ�������Ӵ�ϵ������ֵԽ��򻬳̶�Խ��    
			chassisData.scale[index] = (float)fabs(chassisData.speedFbd[index] / chassisData.speedRef[index]);
	}
	chassisData.averageScale = (float)(chassisData.scale[0] + chassisData.scale[1] + \
	chassisData.scale[2] + chassisData.scale[3]) / 4;
	for(uint8_t index = 0; index < NUMBER_OF_WHEEL; index++){	
		chassisData.scale[index] = (float)chassisData.averageScale / chassisData.scale[index];
		if(chassisData.averageScale < 0.25f){ //����ʱ���ֵ���Ȩ�ؽ�һ���Ŵ�
//			chassisData.scale[0] = chassisData.scale[1] = 0.5f;
//			chassisData.scale[2] = chassisData.scale[3] = 2.0f;
		}
		chassisData.scale[index] = chassisData.scale[index] > 2.0f? 2.0f : chassisData.scale[index];
		chassisData.scale[index] = chassisData.scale[index] < 0.05f? 0.05f : chassisData.scale[index];
	}
#else
	for(uint8_t index = 0; index < NUMBER_OF_WHEEL; index++){	
		chassisData.scale[index] = 1.0f;
	}
#endif
	
	chassisData.time[0] = getClockCount();
	chassisData.intervalTime = (float)(chassisData.time[0] - chassisData.time[1]);
	chassisData.time[1] = chassisData.time[0];
	
	switch(robotMode){
		//������̨ģʽ
		case MODE_RC : 
		case MODE_KM :{
			//���̸�����̨
			if(chassisData.ctrlMode == CHASSIS_FOLLOW_GIMBAL){
				followGimbalHandle();
			}
			//������̨����
			else if(chassisData.ctrlMode == CHASSIS_SEPARATE_GIMBAL){
				separateGimbalHandle();
			}
			//����
			else if(chassisData.ctrlMode == CHASSIS_STOP||chassisData.ctrlMode == CHASSIS_INIT){
				chassisStopHandle();
			}
		}break;
		case MODE_INIT:
		//����ֹͣģʽ
		case MODE_STOP:
			chassisStopHandle();			
			break;
		//�������Ȩģʽ	
		case MODE_RELAX:						
			chassisRelaxHandle();			
			break;		
		default :                                             
			break;
	}
	
	if(chassisData.autoMode){																																							//�Զ����񼤻�ʱ,ƽ���˶����������Զ����
		chassisData.autoSpeedTarget.x = pidUpdate(chassisData.posPID, chassisData.posRef.x, \
												  chassisData.posFbd.x, chassisData.intervalTime);
		chassisData.autoSpeedTarget.y = -pidUpdate(chassisData.posPID, chassisData.posRef.y, \
												   chassisData.posFbd.y, chassisData.intervalTime);
		chassisData.autoSpeedTarget.z = pidUpdate(chassisData.posPID, chassisData.posRef.z, \
												  chassisData.posFbd.z, chassisData.intervalTime);
		mecanumCalculate(chassisData.autoSpeedTarget.x + chassisData.landingSpeedx,
						 chassisData.autoSpeedTarget.y + chassisData.landingSpeedy, 
		                 chassisData.autoSpeedTarget.z + chassisData.landingSpeedz,
						 chassisData.speedLimit * parameter[CHASSIS_RC_SPEED]*REAL_MOTOR_SPEED_SCALE, chassisData.speedRef);        //���ֽ���õ��ĸ����ӵ������ٶ�
	}
	else{
		mecanumCalculate(chassisData.direction * (chassisData.manualSpeedTarget.x + chassisData.landingSpeedx), 
						 chassisData.direction * (chassisData.manualSpeedTarget.y + chassisData.landingSpeedy), 
		                 chassisData.direction * (chassisData.manualSpeedTarget.z + chassisData.landingSpeedz),
						 chassisData.speedLimit * parameter[CHASSIS_RC_SPEED] * REAL_MOTOR_SPEED_SCALE, chassisData.speedRef);        //���ֽ���õ��ĸ����ӵ������ٶ�
		if(chassisData.direction == -1.0f){//�˶�����Ϊ����
			mecanumCalculate(chassisData.direction * (chassisData.manualSpeedTarget.x + chassisData.landingSpeedx), 
							 chassisData.direction * (chassisData.manualSpeedTarget.y + chassisData.landingSpeedy), 
							 chassisData.manualSpeedTarget.z + chassisData.landingSpeedz,
							 chassisData.speedLimit * parameter[CHASSIS_RC_SPEED] * REAL_MOTOR_SPEED_SCALE, chassisData.speedRef);    //���ֽ���õ��ĸ����ӵ������ٶ�	
		}
		//����г��������С�����ٶ�
		if(chassisData.speedFbdMax > chassisData.speedLimitFloor){
			chassisData.DecelerateRatio = chassisData.speedLimitFloor/chassisData.speedFbdMax;			
			for(uint8_t i = 0; i < NUMBER_OF_WHEEL; i++){
				chassisData.speedRef[i] = chassisData.DecelerateRatio*chassisData.speedRef[i];
			}
		}
	}
	
	//�ٶȻ�����																																		
	for(uint8_t i = 0; i < NUMBER_OF_WHEEL; i++){
#ifdef CHASSIS_SPEED_PID
		chassisData.current[i] = pidUpdate(chassisData.speedPID[i],chassisData.speedRef[i],chassisData.speedFbd[i],chassisData.intervalTime);
#endif
#ifdef CHASSIS_SPEED_ADRC
		chassisData.current[i] = adrcUpdate(chassisData.speedADRC[i],chassisData.speedRef[i],chassisData.speedFbd[i]);
#endif
	}
	//�ٶȻ����
	for(uint8_t i = 0; i < NUMBER_OF_WHEEL; i++){
		chassisData.powerCurrent[i] = chassisData.current[i];						
	}
	
	//���̳����̲��޹��ʣ����豸�б��в�Ӧ�и��豸
	if(robotConfigData.robotDeviceList & DEVICE_CURRENT){
		/******C620��������������ܺã����Բ��յ�����******
		//�ٶȻ������������
		for(uint8_t i = 0; i < NUMBER_OF_WHEEL; i++){
			chassisData.powerCurrent[i] += pidUpdate(chassisData.currentPID[i],chassisData.current[i],wheelData[i].currunt,chassisData.intervalTime);//������PID����
		}
		**********************************************/
		//��������ʽ�������
		powerLimitHandle();
	}

	if(robotMode == MODE_RELAX){
		memset((void *)&chassisData.powerCurrent, 0, sizeof(chassisData.powerCurrent));
		//�Ͽ�ʱ��������̨�н�Ϊ0
		chassisData.chaseRef = 0.0f; 
		//ǰ���ķ���Ϊ����
		chassisData.direction = 1.0f;
		//�Ͽظ�λʱ ʼ����һ��λ��	
		parameter[YAW_CENTER] = chassisData.yawCenterSave;      
	}
}

static void chassisStopHandle(void){
	//����ֹͣģʽ�����ٶ�ȫΪ��
  chassisData.manualSpeedTarget.y = 0;
  chassisData.manualSpeedTarget.x = 0;
  chassisData.manualSpeedTarget.z = 0;
}

//�������Ȩģʽ��λpid
static void chassisRelaxHandle(void){
	pidZeroState(chassisData.chasePID);
	for(uint8_t index = 0;index < NUMBER_OF_WHEEL;index++){
		pidZeroState(chassisData.speedPID[index]);
	}
}

//������̨ģʽ
static void followGimbalHandle(void){
	float angleError;
	float sinAngle;
	float cosAngle;

	gimbalData.yawMotorAngle   = ENCODER_ANGLE_RATIO * getRelativePos(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData),parameter[YAW_CENTER]);
	//���̸��淴����ֵ	
	chassisData.chaseFbd = gimbalData.yawMotorAngle;     
	
#if	USE_CHANGE_HEAD	
	if(chassisData.changeHeadOrder && robotConfigData.typeOfRobot==INFANTRY_ID){
		chassisData.chaseFbd = 0.0f;
		chassisData.chaseRef = chassisData.chaseFbd;      		
	}
	if(robotMode==MODE_RC && (chassisData.changeHeadSchedule==2 || chassisData.changeChassisSchedule==2) && robotConfigData.typeOfRobot==INFANTRY_ID){												//��ң������ͷʱ ������Ϊ��ͷ��� ǿ����RCģʽ										
		parameter[YAW_CENTER] =  (parameter[YAW_CENTER] + 4096) > 8192?  (parameter[YAW_CENTER] - 4096) : (parameter[YAW_CENTER] + 4096);
		gimbalData.yawMotorAngle = ENCODER_ANGLE_RATIO * getRelativePos(motorYawData.encoderAngle,parameter[YAW_CENTER]); 
		//��ʼ����
		chassisData.changeHeadOrder = 0.0f; 
		//�ı�ǰ���ķ���
		chassisData.direction = -chassisData.direction; 
		chassisData.changeHeadSchedule = 0.0f;
		chassisData.changeChassisSchedule = 0.0f;
		chassisData.chaseRef = 0.0f;
	}
#endif
	
	angleError = chassisData.chaseFbd * DEG_TO_RAD;
	sinAngle = sinf(angleError);
	cosAngle = cosf(angleError);
	
	chassisData.manualSpeedTarget.x = (remoteControlData.chassisSpeedTarget.x + keyBoardCtrlData.chassisSpeedTarget.x) * cosAngle \
							          - (remoteControlData.chassisSpeedTarget.y + keyBoardCtrlData.chassisSpeedTarget.y) * sinAngle;
	
	chassisData.manualSpeedTarget.y = (remoteControlData.chassisSpeedTarget.x + keyBoardCtrlData.chassisSpeedTarget.x) * sinAngle \
									  + (remoteControlData.chassisSpeedTarget.y + keyBoardCtrlData.chassisSpeedTarget.y) * cosAngle;
#ifdef ADD_IMU_RATE_PID
	chassisData.chaseAngleOut = -getInstallDirect(parameter[YAW_INSTALL], INSTALL_TURN) \
								* pidUpdate(chassisData.chasePID, chassisData.chaseFbd, \
											chassisData.chaseRef, chassisData.intervalTime);
	chassisData.chaseSpeedRef = chassisData.chaseAngleOut;
	if(ROBOT == INFANTRY_ID && powerData.rotateFast && infantryAutoData.rotateFlag)
		chassisData.chaseSpeedRef *= 1.5f;
	else
		chassisData.chaseSpeedRef *= 1.0f;
	chassisData.chaseSpeedFbd = IMU_CHASE_RATEZ;
	chassisData.manualSpeedTarget.z = pidUpdate(chassisData.chaseSpeedPID, chassisData.chaseSpeedFbd, \
												chassisData.chaseSpeedRef, chassisData.intervalTime);
	chassisData.manualSpeedTarget.z = chassisData.manualSpeedTarget.z * REAL_MOTOR_SPEED_SCALE;		 									
#else
	chassisData.manualSpeedTarget.z = getInstallDirect(parameter[YAW_INSTALL],INSTALL_TURN) \
									* pidUpdate(chassisData.chasePID, chassisData.chaseFbd, \
												chassisData.chaseRef, chassisData.intervalTime);
#endif
	
}

//���̷�����̨ģʽ
static void separateGimbalHandle(void){
	//�ڱ�ֻ��y���ٶ�
	if(robotConfigData.typeOfRobot == SENTRY_ID)
		chassisData.manualSpeedTarget.x = 0;
	else
		chassisData.manualSpeedTarget.x = (remoteControlData.chassisSpeedTarget.x + keyBoardCtrlData.chassisSpeedTarget.x);
	
	chassisData.manualSpeedTarget.y = (remoteControlData.chassisSpeedTarget.y + keyBoardCtrlData.chassisSpeedTarget.y);

	if(robotConfigData.typeOfRobot == SENTRY_ID)
		chassisData.manualSpeedTarget.z = 0;
	else	
		chassisData.manualSpeedTarget.z = - getInstallDirect(parameter[YAW_INSTALL], INSTALL_TURN) \
										  * (remoteControlData.chassisSpeedTarget.z + keyBoardCtrlData.chassisSpeedTarget.z);
}

static void powerLimitHandle(void){
	float totalCurrent,totalCurrentLimit;
	//����ϵͳ��������δ��װ����ϵͳ
	if((judgeData.extPowerHeatData.chassis_volt == 0) || (judgeData.extPowerHeatData.chassis_power == 0)){
		//���̹��ʲ������ƣ��ĸ�������ܹ�����������
			totalCurrentLimit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
	}
	else{
		//�����������С��50J
		if(judgeData.extPowerHeatData.chassis_power_buffer < WARNING_POWER_BUFF){
			float powerScale = 0.0f;
			if(judgeData.extPowerHeatData.chassis_power_buffer > 5.0f){
				powerScale = judgeData.extPowerHeatData.chassis_power_buffer / WARNING_POWER_BUFF;
			}
			else{
				powerScale = 5.0f / WARNING_POWER_BUFF;  //���ϵ�������Ƶ���Ϊ80w����
			}
			totalCurrentLimit = powerScale * JUDGE_TOTAL_CURRENT_LIMIT;
		}
		//���������������50J
		else{
			//��ǰ���ʴ���40W
			if(judgeData.extPowerHeatData.chassis_power > WARNING_POWER){
				float powerScale = 0.0f;
				//������80Wʱ
				if(judgeData.extPowerHeatData.chassis_power < POWER_LIMIT){
					powerScale = (POWER_LIMIT - judgeData.extPowerHeatData.chassis_power) / (POWER_LIMIT - WARNING_POWER);
				}
				//����80W�޵����ӳ�
				else{
					powerScale = 0.0f;
				}
				totalCurrentLimit = ADD_POWER_CURRENT * powerScale + JUDGE_TOTAL_CURRENT_LIMIT;
			}
			//��ǰ����С��40W
			else{
				totalCurrentLimit = ADD_POWER_CURRENT + JUDGE_TOTAL_CURRENT_LIMIT;
			}
		}
	}
	totalCurrent = 0.0f;
	//��������ĸ�����������������С
	for(uint8_t i = 0; i < NUMBER_OF_WHEEL; i++){
		totalCurrent += fabs(chassisData.powerCurrent[i]);
	}
	//��������Ƿ񳬹���ǰ����ֵ�������򰴱����޷�
	if(totalCurrent > totalCurrentLimit){
		float current_scale = totalCurrentLimit / totalCurrent;
		chassisData.powerCurrent[0] *= current_scale * chassisData.scale[0];
		chassisData.powerCurrent[1] *= current_scale * chassisData.scale[1];
		chassisData.powerCurrent[2] *= current_scale * chassisData.scale[2];
		chassisData.powerCurrent[3] *= current_scale * chassisData.scale[3];
	}
}

//���̳�ʼ��
void chassisInit(void){							
	chassisData.ctrlMode      = CHASSIS_RELAX;
	chassisData.lastCtrlMode  = CHASSIS_STOP;
	//FirstOrderFilter
	powerInitFirstOrderFilter();					
	initFirstOrderFilter();
	chassisData.posPID = pidInit(&parameter[CHASSIS_POS_P], &parameter[CHASSIS_POS_I], &parameter[CHASSIS_POS_D], &parameter[CHASSIS_POS_F],	\
												 	  &parameter[CHASSIS_POS_PM], &parameter[CHASSIS_POS_IM], &parameter[CHASSIS_POS_DM], &parameter[CHASSIS_POS_OM],	\
													  NULL, NULL, NULL, NULL); 
	for(uint8_t index = 0;index < NUMBER_OF_WHEEL;index++){						
#ifdef CHASSIS_SPEED_PID
		chassisData.speedPID[index] = pidInit(&parameter[CHASSIS_SPEED_P], &parameter[CHASSIS_SPEED_I], &parameter[CHASSIS_SPEED_D], &parameter[CHASSIS_SPEED_F],	\
																&parameter[CHASSIS_SPEED_PM], &parameter[CHASSIS_SPEED_IM], &parameter[CHASSIS_SPEED_DM], &parameter[CHASSIS_SPEED_OM],	\
																NULL, NULL, NULL, NULL);
#endif
#ifdef CHASSIS_SPEED_ADRC
		chassisData.speedADRC[index] = adrcInit(&parameter[ADRC_R],&parameter[ADRC_H],&parameter[ADRC_N0],&parameter[ADRC_BETA01],&parameter[ADRC_BETA02],	\
																						&parameter[ADRC_BETA03],&parameter[ADRC_B0],&parameter[ADRC_BETA0],&parameter[ADRC_BETA1],&parameter[ADRC_BETA2],	\
																						&parameter[ADRC_N1],&parameter[ADRC_C],&parameter[ADRC_ALPHA1],&parameter[ADRC_ALPHA2],&parameter[ADRC_ZETA],	\
																						&parameter[ADRC_B],&parameter[ADRC_OMAX]);
#endif

		if(robotConfigData.robotDeviceList & DEVICE_CURRENT){
			chassisData.currentPID[index] = pidInit(&parameter[POWER_LIMIT_P], &parameter[POWER_LIMIT_I], &parameter[POWER_LIMIT_D], &parameter[POWER_LIMIT_F],	\
																&parameter[POWER_LIMIT_PM], &parameter[POWER_LIMIT_IM], &parameter[POWER_LIMIT_DM], &parameter[POWER_LIMIT_OM],	\
																NULL, NULL, NULL, NULL);  
		}
	}
	chassisData.chasePID = pidInit(&parameter[CHASSIS_CHASE_P], &parameter[CHASSIS_CHASE_I], &parameter[CHASSIS_CHASE_D], &parameter[CHASSIS_CHASE_F],	\
													&parameter[CHASSIS_CHASE_PM], &parameter[CHASSIS_CHASE_IM], &parameter[CHASSIS_CHASE_DM], &parameter[CHASSIS_CHASE_OM],	\
													NULL, NULL, NULL, NULL);	
	chassisData.chaseSpeedPID = pidInit(&parameter[CHASSIS_RATE_P], &parameter[CHASSIS_RATE_I], &parameter[CHASSIS_RATE_D], &parameter[CHASSIS_RATE_F],	\
													&parameter[CHASSIS_RATE_PM], &parameter[CHASSIS_RATE_IM], &parameter[CHASSIS_RATE_DM], &parameter[CHASSIS_RATE_OM],	\
													NULL, NULL, NULL, NULL);	
	chassisData.speedLimit = 1.0f;
	chassisData.direction = 1.0f;
	chassisData.landingSpeedx = 0.0f;
	chassisData.landingSpeedy = 0.0f;
	chassisData.landingSpeedz = 0.0f;
	chassisData.changeHeadOrder = 0.0f;
	chassisData.changeHeadSchedule = 0.0f;
	chassisData.changeChassisSchedule = 0.0f;
	//�洢У׼�õ���ֵ
	chassisData.yawCenterSave = parameter[YAW_CENTER]; 						  
} 
