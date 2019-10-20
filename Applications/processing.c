#include "processing.h"
#include "imu.h"
#include "Driver_MPU6500.h"
#include "slave_sensor.h"

sensorStruct_t sensorRotate;
void sensorCalibProcess(float *in,volatile float *out,float *bias,uint8_t t,int16_t *rotate,float *p){
	float a, b, c;
	float *r = p; 
	
	a = rotate[0] * (in[0] + bias[0]);
	b = rotate[1] * (in[1] + bias[1]);
	c = rotate[2] * (in[2] + bias[2]);

	out[0] = a + b*r[0] + c*r[1];
	out[1] = a*r[2] + b + c*r[3];
	out[2] = a*r[4] + b*r[5] + c;
}

void sensorScaleProcess(float *in,float *out,uint8_t t,float coefficient,int16_t *orient){
	uint8_t i =0;
	for(i=0; i<t ;i++)
		out[i] = in[i] * coefficient * orient[i];
}

void sensorTempScaleProcess(float *in,volatile float *out,float coefficient){
	if(coefficient != NULL)
		out[0] = coefficient * in[0];
	else
		out[0]=21.0f+((float)in[0])/333.87f;
}
	
void sensorProcessUpdate(imusensorStruct_t *imuSensor,coordinateFloat_t *accdata,coordinateFloat_t *gyodata,coordinateFloat_t *magdata,s16 tempreature){
	static float temp[1],acc[3],gyo[3],mag[3];
	if(accdata != NULL){
		acc[0] = accdata->y;
		acc[1] = accdata->x;
		acc[2] = accdata->z;
		sensorScaleProcess(acc,imuSensor->rawAcc,3,ACC_SCALE,sensorRotate.accOrient);
		sensorCalibProcess(imuSensor->rawAcc,imuSensor->acc,imuSensor->accBIAS,3,sensorRotate.accRotate,NULL);
	}
	if(gyodata != NULL){
		gyo[0] = gyodata->y;
		gyo[1] = gyodata->x;
		gyo[2] = gyodata->z;
		sensorScaleProcess(gyo,imuSensor->rawGyo,3,GYO_SCALE,sensorRotate.gyoOrient);
		sensorCalibProcess(imuSensor->rawGyo,imuSensor->gyo,imuSensor->gyoBIAS,3,sensorRotate.gyoRotate,NULL);
	}
	if(magdata != NULL){
		mag[0] = magdata->y;
		mag[1] = magdata->x;
		mag[2] = magdata->z;
		sensorScaleProcess(mag,imuSensor->rawMag,3,MAG_SCALE,sensorRotate.magOrient);
		sensorCalibProcess(imuSensor->rawMag,imuSensor->mag,imuSensor->magBIAS,3,sensorRotate.magRotate,NULL);
	}
	if(tempreature != NULL){
		temp[0] = tempreature;
		sensorTempScaleProcess(temp,&imuSensor->temp,NULL);
	}
}

void sensorProcessInit(void){
	sensorRotate.accRotate[0] = -1;sensorRotate.gyoRotate[0] = 1;sensorRotate.magRotate[0] = 1;
	sensorRotate.accRotate[1] = 1;sensorRotate.gyoRotate[1] = -1;sensorRotate.magRotate[1] = -1;
	sensorRotate.accRotate[2] = -1;sensorRotate.gyoRotate[2] = 1;sensorRotate.magRotate[2] = -1;
	
//	sensorRotate.accRotate[0] = -1;sensorRotate.gyoRotate[0] = 1;sensorRotate.magRotate[0] = 1;
//	sensorRotate.accRotate[1] = -1;sensorRotate.gyoRotate[1] = 1;sensorRotate.magRotate[1] = -1;
//	sensorRotate.accRotate[2] = 1;sensorRotate.gyoRotate[2] = -1;sensorRotate.magRotate[2] = -1;
	
	sensorRotate.accOrient[0] = 1;sensorRotate.gyoOrient[0] = 1;sensorRotate.magOrient[0] = 1;
	sensorRotate.accOrient[1] = 1;sensorRotate.gyoOrient[1] = 1;sensorRotate.magOrient[1] = 1;
	sensorRotate.accOrient[2] = 1;sensorRotate.gyoOrient[2] = 1;sensorRotate.magOrient[2] = 1;
//	sensorProcessUpdate(&imuSensorData,&mpu6500Data.acc,&mpu6500Data.gyro,&mpu6500Data.mag,mpu6500Data.tempreature);
}

float meanRecursiveFilter(s16 newVuale,u8 axis){
	static float gyoSum[12];
	static float gyoVaule[12][10];
	float outVaule;
	static uint8_t index[12] = {0,0,0,0,0,0};
	gyoSum[axis] -= gyoVaule[axis][index[axis]];
	gyoVaule[axis][index[axis]] = newVuale;
	gyoSum[axis] += gyoVaule[axis][index[axis]];
	index[axis] = (index[axis] + 1) % 10;
	outVaule = (float)gyoSum[axis] / 10;
	return outVaule;
}	

void initLowPassFilter(void){
    float a;

    a = 2.0f * 0.03f * 500.0f;

    sensorRotate.lowPassFilterData[ACCEL_X_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    sensorRotate.lowPassFilterData[ACCEL_X_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    sensorRotate.lowPassFilterData[ACCEL_X_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    sensorRotate.lowPassFilterData[ACCEL_X_500HZ_LOWPASS].previousInput  = 0.0f;
    sensorRotate.lowPassFilterData[ACCEL_X_500HZ_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    a = 2.0f * 0.03f * 500.0f;

    sensorRotate.lowPassFilterData[ACCEL_Y_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    sensorRotate.lowPassFilterData[ACCEL_Y_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    sensorRotate.lowPassFilterData[ACCEL_Y_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    sensorRotate.lowPassFilterData[ACCEL_Y_500HZ_LOWPASS].previousInput  = 0.0f;
    sensorRotate.lowPassFilterData[ACCEL_Y_500HZ_LOWPASS].previousOutput = 0.0f;

    ///////////////////////////////////

    a = 2.0f * 0.03f * 500.0f;

    sensorRotate.lowPassFilterData[ACCEL_Z_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
    sensorRotate.lowPassFilterData[ACCEL_Z_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
    sensorRotate.lowPassFilterData[ACCEL_Z_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
    sensorRotate.lowPassFilterData[ACCEL_Z_500HZ_LOWPASS].previousInput  = -9.8065f;
    sensorRotate.lowPassFilterData[ACCEL_Z_500HZ_LOWPASS].previousOutput = -9.8065f;

}


float LowPass_Filter(float input, struct LowPassFilterData *filterParameters){
    float output;

    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}
