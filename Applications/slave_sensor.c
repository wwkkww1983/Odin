#include "Driver_Slave_Sensor.h"
#include "vision.h"
#include "slave_sensor.h"
#include "Driver_judge.h"
#include "processing.h"
#include "shoot.h"
#include "warning.h"
#include "Driver_DMMotor.h"
#include "supervisor.h"
#include "config.h"
#include "judge.h"
#include "rc.h"

slaveSensorStruct_t slaveSensorData;
controlTransStruct_t controlTransData;
//��ȡ��̨���ϵ���Ϣ
void modularDataDecoding(u8 array){
	shootData.bulletMonitorFlag = array & 0x01;
	supervisorStateSwitch(STATE_SENSOR_ERROR, array & 0x02);
	supervisorStateSwitch(STATE_IMUCALI, array & 0x04);
}

//��ȡ�Ӿ�������Ϣ
static void  visionDataDecoding(u8 array){
	visionData.distingushState = array;
	visionData.captureFlag = array & 0x01;
	visionData.sameTargetFlag = array & 0x02;
	visionData.buffOfDirRota = (buffOfDirRota_e)((array & 0x0C)>>2);
}

//��ȡIMU����
static void slaveImuDataRead(uint8_t *array){
	uint8_t index_ptr = 4;
	uint8_t index = 0;
	modularDataDecoding(array[index_ptr++]);
	for(uint8_t j = 0; j < 3; j++){
		for(index = 0; index < 2; index++){
			imuData.gyo[j].u8_temp[index] = array[index_ptr++];
		}
	}

	if(array[1] & TRANS_ADD_ANGLE){
		for(index=0;index < 4;index++)
			imuData.pitch.u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			imuData.roll.u8_temp[index] = array[index_ptr++];
		for(index=0;index < 4;index++)
			imuData.yaw.u8_temp[index] = array[index_ptr++];
		for(index=0;index < 2;index++)
			imuData.CNTR.u8_temp[index] = array[index_ptr++];
		visionStoreHistoricalData(imuData.pitch.float_temp, imuData.yaw.float_temp, imuData.CNTR.u16_temp);
	}
	if(array[1] & TRANS_ADD_VISION){
		visionDataDecoding(array[index_ptr++]);		
		for(uint8_t j = 0; j < 3; j++){
			for(index = 0; index < 2; index++){
				 visionData.coordinateBase[j].u8_temp[index] = array[index_ptr++];
			}
		}
		for(uint8_t j = 0; j < 3; j++){
			for(index = 0; index < 2; index++){
				 visionData.coordinate[j].u8_temp[index] = array[index_ptr++];
			}
		}
		for(index = 0; index < 2; index++){
			visionData.CNTR.u8_temp[index] = array[index_ptr++];
		}		
	}
	slaveSensorData.canForwardIndexPtr = index_ptr;
}

//��ȡ��̨ת�����������
static void slaveGimbalDataRead(uint8_t *array){
	uint8_t index_ptr = slaveSensorData.canForwardIndexPtr;
	uint8_t index = 0;
//	for(uint8_t j=0;j < 3;j++)
//		for(index=0;index < 2;index++)
//			slaveSensorData.yawRecv[j].u8_temp[index] = arraySlaveGimbal[index_ptr++];
	for(uint8_t j=0;j < 3;j++)
		for(index=0;index < 2;index++)
			slaveSensorData.pitchRecv[j].u8_temp[index] = array[index_ptr++];
//	gimbal_readSlaveData(CODEBOARD_VALUE,slaveSensorData.pitchRecv[0].s16_temp,&yawMotorData);
//	gimbal_readSlaveData(REALTORQUE_CURRENT,slaveSensorData.pitchRecv[1].s16_temp,&yawMotorData);
//	gimbal_readSlaveData(TORQUE_CURRENT,slaveSensorData.pitchRecv[2].s16_temp,&yawMotorData);	

	for(uint8_t i=0;i < 2;i++){
		for(uint8_t j=0;j < 3;j++){
			for(index=0;index < 2;index++){
				slaveSensorData.fricWheelRecv[i][j].u8_temp[index] = array[index_ptr++];
			}
		}
	}
	
	gimbal_readSlaveData(CODEBOARD_VALUE,slaveSensorData.pitchRecv[0].s16_temp,&pitchMotorData);
	gimbal_readSlaveData(REALTORQUE_CURRENT,slaveSensorData.pitchRecv[1].s16_temp,&pitchMotorData);
	gimbal_readSlaveData(TORQUE_CURRENT,slaveSensorData.pitchRecv[2].s16_temp,&pitchMotorData);
	
	fricWheelData[0].rawangle = slaveSensorData.fricWheelRecv[0][0].s16_temp;
	fricWheelData[0].speed = slaveSensorData.fricWheelRecv[0][1].s16_temp;
	fricWheelData[0].currunt = slaveSensorData.fricWheelRecv[0][2].s16_temp;
	
	fricWheelData[1].rawangle = slaveSensorData.fricWheelRecv[1][0].s16_temp;
	fricWheelData[1].speed = slaveSensorData.fricWheelRecv[1][1].s16_temp;
	fricWheelData[1].currunt = slaveSensorData.fricWheelRecv[1][2].s16_temp;
}

//��ȡ��̨�巢����������
void slaveSensorRead(uint8_t *array){
	if(array[0] == MAIN_CONTROL_BEGIN && (array[1] & MAIN_CONTROL_ADDRESS) ){
		if(!Verify_CRC8_Check_Sum(array, 4) && !Verify_CRC16_Check_Sum(array, array[2])){					
			// У��ʧ���򲻵�������
			//��ʱ�����в���
		}
		else{
			//��ȡ�ӻ�imu����
			slaveImuDataRead(array);
			//�����Ҫת���������
			if(canSendData.canForward)
				slaveGimbalDataRead(array);
		}
	}
}

//��϶���̨��Ŀ���ָ��
void mixModularCommand(uint8_t *array){
	*array = 0x00;
	*array |= robotMode << 0;
	*array |= supervisorData.imuCali << 3;
	digitalClan(&supervisorData.imuCali);
	*array |= shootData.fricMotorFlag << 5;
	*array |= canSendData.canForward << 6;
}
uint8_t aa = 0;
//��϶��Ӿ���Ŀ���ָ��
void mixVisionCommand(uint8_t *array){
	*array = 0x00;
	*array |= visionData.workMode << 0;
	*array |= visionData.bullet << 2;
	*array |= visionData.enemyType << 3;
	*array |= shootData.shootMode << 4;
	*array |= visionData.rBiasMode << 6;
	aa = *array;
}

void mixSk6812Command(uint8_t *array){
	*array = 0x00;
	*array |= robotConfigData.typeOfRobot << 0;
	*array |= warningData.reportError << 3;
	*array |= warningData.capSoc << 4;
}

void motorCmdForward(void){
//	slaveSensorData.yawCmd.s16_temp = canSendData.can1_0x2FF.currunt1;
	slaveSensorData.pitchCmd.s16_temp = canSendData.can1_0x2FF.currunt2;
	slaveSensorData.fricWheelCmd[0].s16_temp = canSendData.can1_0x1FF.currunt1;
	slaveSensorData.fricWheelCmd[1].s16_temp = canSendData.can1_0x1FF.currunt2;
}

//����ָ�����
void moduleCommandUpload(USART_TypeDef *USARTx){
	uint8_t *array = (uint8_t *)USART_TO_ArrayTX(USARTx);
	uint8_t index_ptr = 0;
	uint8_t index = 0;
	array[index_ptr++] = IMU_MODULAR_BEGIN;
	array[index_ptr++] = IMU_MODULAR_ADDRESS;
	array[index_ptr++] = 0x00;
	array[index_ptr++] = 0x00;
	//��װ��������
	mixModularCommand(&array[index_ptr++]);
	for(index = 0; index < 2; index++){
		array[index_ptr++] = warningData.lightBarsState.u8_temp[index];
	}
	//��װ��tx2ָ��
	identifyCamp();																											
	mixVisionCommand(&array[index_ptr++]);
	//��װ��������ָ��
	mixSk6812Command(&array[index_ptr++]);
	//�����Ҫת����̨CAN����
	if(canSendData.canForward){
		motorCmdForward();
		for(index = 0; index < 2; index++)
			Array_USART3_TX[index_ptr++] = slaveSensorData.pitchCmd.u8_temp[index];
		for(index = 0; index < 2; index++)
			Array_USART3_TX[index_ptr++] = slaveSensorData.fricWheelCmd[0].u8_temp[index];
		for(index = 0; index < 2; index++)
			Array_USART3_TX[index_ptr++] = slaveSensorData.fricWheelCmd[1].u8_temp[index];
	}

	//��װУ��λ
	array[2] = index_ptr + 2;
	Append_CRC8_Check_Sum(array, 4);
	Append_CRC16_Check_Sum(array, array[2]);
	BSP_USART_DMA_SendData(USARTx, array, (index_ptr + 2));
}

void otherControDataDecoding(uint8_t array){
	controlTransData.otherMode = array & 0x07;
	controlTransData.otherSameTargetFlag = array & 0x08;
	controlTransData.otherEnemyType = array & 0x10;
	controlTransData.otherfricMotorFlag = array & 0x20;
	controlTransData.otherAutoMaticFlag = array & 0x40;
	controlTransData.otherPatrolMode = array & 0x80;
}

//��һ���������ݵĽ���
static void otherControlDataRead(uint8_t *arrayOtherControl){
	uint8_t index_ptr = 7;
	uint8_t index = 0;
	otherControDataDecoding(arrayOtherControl[index_ptr++]);
	if(parameter[ROBOT_TYPE] == SMALLGIMBAL_ID){
			controlTransData.bullet_type = arrayOtherControl[index_ptr++];
		for(index = 0; index < 4; index++)
			controlTransData.shootSpeed.u8_temp[index] = arrayOtherControl[index_ptr++];
		for(index = 0; index < 2; index++)
			controlTransData.shooter_heat0.u8_temp[index] = arrayOtherControl[index_ptr++];
		for(index = 0; index < 2; index++)
			controlTransData.maxHP.u8_temp[index] = arrayOtherControl[index_ptr++];		
		for(index = 0; index < 2; index++)
			controlTransData.remainHP.u8_temp[index] = arrayOtherControl[index_ptr++];	
		//װ��ң��������
		for(index = 0; index < 18; index++){
			controlTransData.otherRcValue[index] = arrayOtherControl[index_ptr++];
		}
		controlTransData.otherRcReadly = arrayOtherControl[index_ptr++];
			
		for(index = 0; index < 4; index++)
			controlTransData.masterPitchMotorAngle.u8_temp[index] = arrayOtherControl[index_ptr++];		
		for(index = 0; index < 4; index++)
			controlTransData.masterYawMotorAngle.u8_temp[index] = arrayOtherControl[index_ptr++];
		for(index = 0; index < 4; index++)
			controlTransData.masterPitchAngleRef.u8_temp[index] = arrayOtherControl[index_ptr++];	
		for(index = 0; index < 4; index++)
			controlTransData.masterYawAngleRef.u8_temp[index] = arrayOtherControl[index_ptr++];

		//����ͬ��
		judgeData.extShootData.bullet_type = controlTransData.bullet_type;
		judgeData.extShootData.bullet_speed = controlTransData.shootSpeed.float_temp;
		judgeData.extPowerHeatData.shooter_heat0 = controlTransData.shooter_heat0.u16_temp;
		judgeData.extGameRobotState.max_HP = controlTransData.maxHP.u16_temp;	
		judgeData.extGameRobotState.remain_HP = controlTransData.remainHP.u16_temp;
		for(index = 0; index < 18; index++){
			Array_USART1_RX[index] = controlTransData.otherRcValue[index];
		}	
		remoteControlData.rcIspReady = controlTransData.otherRcReadly;
		if(controlTransData.otherRcReadly){
			digitalIncreasing(&remoteControlData.rcError.errorCount);
		}		
	}
}

//��ȡ��һ�����ط�����������
void otherControlRead(u8 *arrayOtherControl){
	if(arrayOtherControl[0] == 0xAD && arrayOtherControl[1] == 0x4C){
		if(!Verify_CRC8_Check_Sum(arrayOtherControl,5) && !Verify_CRC16_Check_Sum(arrayOtherControl,arrayOtherControl[2])){					
			//У��ʧ���򲻵�������
			//��ʱ�����в���
		}
		else{
			controlTransData.seq = arrayOtherControl[3];
			otherControlDataRead(arrayOtherControl);
		}
	}
}

//��Ͽ��ư�Ŀ���ָ��
void mixControlCommand(u8* array){
	*array = 0x00;
	*array |= robotMode << 0;
	*array |= visionData.sameTargetFlag << 3;
	*array |= visionData.enemyType << 4;
	*array |= shootData.fricMotorFlag << 5;
	if(parameter[ROBOT_TYPE] != SMALLGIMBAL_ID)	
		*array |= (ENABLE_AUTOMATIC_AIM_42MM) << 6;
	else 
		*array |= visionData.miniPTZEnableAim << 6;
	*array |= gimbalData.followLock << 7;
	digitalClan(&supervisorData.imuCali);
}

//���͵�ǰ���ư�ָ��
void controlDataUpload(void){
	static unsigned char Seq=0;
	u8 index_ptr = 0;
	u8 index = 0;
	Array_USART6_TX[index_ptr++] = 0xAD;
	Array_USART6_TX[index_ptr++] = 0x4C;
	Array_USART6_TX[index_ptr++] = 0x00;
	Array_USART6_TX[index_ptr++] = Seq;
	Array_USART6_TX[index_ptr++] = 0x00;
	Array_USART6_TX[index_ptr++] = 0x00;
	Array_USART6_TX[index_ptr++] = 0x01;
	//��װ������Ϣ
	mixControlCommand(&Array_USART6_TX[index_ptr++]);
	if(parameter[ROBOT_TYPE] != SMALLGIMBAL_ID){
		//��װ����ϵͳ��Ϣ
		Array_USART6_TX[index_ptr++] = judgeData.extShootData.bullet_type;
		controlTransData.shootSpeed.float_temp = judgeData.extShootData.bullet_speed;
		controlTransData.shooter_heat0.u16_temp = judgeData.extPowerHeatData.shooter_heat0;
		controlTransData.maxHP.u16_temp = judgeData.extGameRobotState.max_HP;
		controlTransData.remainHP.u16_temp = judgeData.extGameRobotState.remain_HP;
		for(index=0;index < 4;index++){
			Array_USART6_TX[index_ptr++] = controlTransData.shootSpeed.u8_temp[index];
		}
		for(index=0;index < 2;index++){
			Array_USART6_TX[index_ptr++] = controlTransData.shooter_heat0.u8_temp[index];
		}
		for(index=0;index < 2;index++){
			Array_USART6_TX[index_ptr++] = controlTransData.maxHP.u8_temp[index];
		}
		for(index=0;index < 2;index++){
			Array_USART6_TX[index_ptr++] = controlTransData.remainHP.u8_temp[index];
		}
		
		//װ��ң��������
		for(index=0;index < 18;index++){
			controlTransData.otherRcValue[index] = Array_USART1_RX[index];
		}
		for(index=0;index < 18;index++){
			Array_USART6_TX[index_ptr++] = controlTransData.otherRcValue[index];
		}
		Array_USART6_TX[index_ptr++] = controlTransData.otherRcReadly;
			
		//��װ��ǰ��̨�����̽Ƕ�
		controlTransData.masterPitchMotorAngle.float_temp = gimbalData.pitchMotorAngle;
		controlTransData.masterYawMotorAngle.float_temp = gimbalData.yawMotorAngle;
		for(index=0;index < 4;index++){
			Array_USART6_TX[index_ptr++] = controlTransData.masterPitchMotorAngle.u8_temp[index];
		}
		for(index=0;index < 4;index++){
			Array_USART6_TX[index_ptr++] = controlTransData.masterYawMotorAngle.u8_temp[index];
		}
		//��װ��ǰ��̨�ĽǶ�����
		controlTransData.masterPitchAngleRef.float_temp = gimbalData.pitchAngleRef;
		controlTransData.masterYawAngleRef.float_temp = gimbalData.yawAngleRef;
		for(index=0;index < 4;index++){
			Array_USART6_TX[index_ptr++] = controlTransData.masterPitchAngleRef.u8_temp[index];
		}
		for(index=0;index < 4;index++){
			Array_USART6_TX[index_ptr++] = controlTransData.masterYawAngleRef.u8_temp[index];
		}
	}
	
	//��װУ��λ
	Array_USART6_TX[2] = index_ptr + 2;
	Append_CRC8_Check_Sum(Array_USART6_TX,5);
	Append_CRC16_Check_Sum(Array_USART6_TX,(index_ptr + 2));
	Seq++;
	BSP_USART6_DMA_SendData(Array_USART6_TX,(index_ptr + 2));
}

void slaveSensorConfig(void){
	driver_slaveSensorInit(SLAVE_SENSOR_USARTX, SLAVE_SENSOR_USARTX_RX_PIN, SLAVE_SENSOR_USARTX_TX_PIN, \
						   SLAVE_SENSOR_USART_BOUND, SLAVE_SENSOR_USART_PRE, SLAVE_SENSOR_USART_SUB);
	driver_slaveSensorInit(MAIN_OR_SLAVE_CONTROL_USARTX, MAIN_OR_SLAVE_CONTROL_USARTX_RX_PIN, MAIN_OR_SLAVE_CONTROL_USARTX_TX_PIN, \
						   MIAN_OR_SLAVE_CONTROL_USARTS_BOUND, MAIN_OR_SLAVE_CONTROL_USARTX_PRE, MAIN_OR_SLAVE_CONTROL_USARTX_SUB);
	slaveSensorData.initFlag = true;  	 	 
}
