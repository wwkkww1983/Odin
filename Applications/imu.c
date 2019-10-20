#include "imu.h"
#include "supervisor.h"
#include "app_Init.h"
#include "Driver_MPU6500.h"
#include "processing.h"

imuStruct_t imuData;
imusensorStruct_t imuSensorOfChassisData;

bool moduleUpdateCheck(void){
	bool res;
	if(imuData.CNTR.u16_temp != imuData.lastCNTR)
		res = true;
	else
		res = false;
	imuData.lastCNTR = imuData.CNTR.u16_temp;
	return res;
}

void imuChassisUpdate(void){
	Read_MPU6500();
	sensorProcessUpdate(&imuSensorOfChassisData,NULL,&mpu6500Data.gyro,NULL,mpu6500Data.tempreature);
	imuTempControl(imuSensorOfChassisData.expTemp);									//���¿���,pid����
	digitalIncreasing(&imuSensorOfChassisData.loops); 
}

void imuInit(void){
	memset((void *)&imuData, 0, sizeof(imuData));
	sensorProcessInit();
	initLowPassFilter();	
	Driver_MPU6500_Init();
	imuSensorOfChassisData.expTemp = CONSTANT_TEMP_VAULE;						//��ֵ�¶�Ŀ��ֵ
	imuSensorOfChassisData.initFlag = true;
}


