
#ifndef __IMU_H
#define __IMU_H

#include "imu.h"
#include "parameter.h"
#include "stdbool.h"
#include "NavAndData.h"

enum{
	IMU_CAIL_NO_NEED = 0,
	IMU_CAIL_START ,
	IMU_CAIL_BEING ,
	IMU_CAIL_FINISH ,
};

typedef struct{
  float x;
	float y;
	float z;
}coordinateFloat_t;

typedef struct{
  s16 x;
	s16 y;
	s16 z;
}coordinateInteger_t;

typedef struct{
	formatTrans16Struct_t x;
	formatTrans16Struct_t y;
	formatTrans16Struct_t z;
} coordinateUnion_t;

/* UNION Data 	   --------------------------------*/
typedef union{
	int16_t value;
	uint8_t bytes[2];
} int16AndUint8_t;

typedef union{
	int32_t value;
	uint8_t bytes[4];
} int32AndUint8_t;

typedef union{
	uint16_t value;
	uint8_t bytes[2];
} uint16AndUint8_t;

typedef struct {
	int16AndUint8_t originalAccel[3];
	int16AndUint8_t originalGyro[3];
	int16AndUint8_t originalMag[3];
	int16AndUint8_t originalTemperature;
	float rawAcc[3];
	float rawGyo[3];
	float rawMag[3];
	volatile float acc[3];
	volatile float temp;
	volatile float gyo[3];
	volatile float mag[3];
	float accBIAS[3];
	float gyoBIAS[3];
	float magBIAS[3];
	float expTemp;
	volatile uint8_t accTare;
	uint32_t imuTareLoop;
	uint8_t state;
	double time[2];
	float intervalTime;
	bool initFlag;
	uint32_t loops;
}imusensorStruct_t;

typedef struct {
	formatTrans32Struct_t pitch,yaw,roll;
	formatTrans16Struct_t gyo[3];
	formatTrans16Struct_t CNTR;
	float lastCNTR;
	float sinRot, cosRot;
	uint32_t fullUpdates;
	uint32_t halfUpdates;
} imuStruct_t;

#define AQ_YAW			imuData.yaw.float_temp
#define AQ_PITCH		imuData.pitch.float_temp
#define AQ_ROLL			imuData.roll.float_temp

#define IMU_RATEX		(float)imuData.gyo[0].s16_temp / 1000
#define IMU_RATEY		(float)imuData.gyo[1].s16_temp / 1000
#define IMU_RATEZ		(float)imuData.gyo[2].s16_temp / 1000

#define IMU_CHASE_RATEX imuSensorOfChassisData.gyo[0]
#define IMU_CHASE_RATEY imuSensorOfChassisData.gyo[1]
#define IMU_CHASE_RATEZ imuSensorOfChassisData.gyo[2]

extern imuStruct_t imuData;
extern imusensorStruct_t imuSensorOfChassisData;

void imuChassisUpdate(void);
void imuSensorReady(void);
void imuInit(void);

#endif




