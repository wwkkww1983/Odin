#ifndef __DIFFERENTIAL_CALCULATION_H
#define __DIFFERENTIAL_CALCULATION_H

#include "util.h"

typedef struct
{
	s16 inputBuffer;
	s16	outputBuffer;
}dataAcquisitionBufStruct_t;

typedef struct
{
	float *yCoefficient;
	float *xCoefficient;
}coefficientStruct_t;

typedef struct
{
	u8 			differentialLength;
	float 	*inputData;
	float 	*outputData;
	u8			initFlag;
	coefficientStruct_t *coefficient;
}differentialDataStruct_t;


float differentialCal(differentialDataStruct_t *data,float currentData);
differentialDataStruct_t *differentialInit(float *coefficientData,u8 length);
void dataAcquisition(float *inputData,float outputData,float amplitude);
void dataAcquisitionInit(void);

extern float transferData[8];

#endif
