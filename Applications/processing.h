#ifndef __PROCESSING_H
#define __PROCESSING_H

#include "FreeRTOS_board.h"
#include "imu.h"
#include "parameter.h"
#include "NavAndData.h"
 
#define	GYO_SCALE  1.0f/((1<<16)/(2000*2.0f))*DEG_TO_RAD
#define	ACC_SCALE  1.0f/((1<<16)/(16*2.0f))*GRAVITY
#define	MAG_SCALE  1.0f / 187.88f

enum {
	ACCEL_X_500HZ_LOWPASS = 0,
	ACCEL_Y_500HZ_LOWPASS,
	ACCEL_Z_500HZ_LOWPASS,
};

typedef struct LowPassFilterData {
	float   gx1;
	float   gx2;
	float   gx3;
	float   previousInput;
	float   previousOutput;
} lowPassFilterStruct_t;

typedef struct {
	int16_t accRotate[3];
	int16_t gyoRotate[3];
	int16_t magRotate[3];
	int16_t accOrient[3];
  int16_t gyoOrient[3];
	int16_t magOrient[3];
	int16_t tempOrient[1];
	lowPassFilterStruct_t lowPassFilterData[6];
} sensorStruct_t;

extern sensorStruct_t sensorRotate;
typedef void SENSORUpdate_t(float *in,float *out,uint8_t t,float translate);
void sensorProcessUpdate(imusensorStruct_t *imuSensor,coordinateFloat_t *accdata,coordinateFloat_t *gyodata,coordinateFloat_t *magdata,s16 tempreature);
void sensorProcessInit(void);
float meanRecursiveFilter(s16 newVuale,u8 axis);
float LowPass_Filter(float input, struct LowPassFilterData *filterParameters);
void initLowPassFilter(void);
void imuQuasiStatic(int n);

#endif 



