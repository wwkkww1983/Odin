#ifndef __RAMP_H
#define __RAMP_H

#include "arm_math.h"
//#define PI					3.14159265358979f

typedef struct __Ramp_t
{
	int count;
	int XSCALE;
	float out;
	void (*Init)(struct __Ramp_t *ramp, int XSCALE);
	float (*Calc)(struct __Ramp_t *ramp,int add);
	void (*SetCounter)(struct __Ramp_t *ramp, int count);
	void (*ResetCounter)(struct __Ramp_t *ramp);
	void (*SetScale)(struct __Ramp_t *ramp, int scale);
	unsigned char (*IsOverflow)(struct __Ramp_t *ramp);
}ramp_t;

void RampInit(ramp_t *ramp, int XSCALE);
float LinearRampCalc(ramp_t *ramp,int add);			//����б�¼���
float SinRampCalc(ramp_t *ramp,int add);				//����б�¼���
float QuadraticRampCalc(ramp_t *ramp,int add);	//���κ���б�¼���
void RampSetCounter(ramp_t *ramp, int count);
void RampResetCounter(ramp_t *ramp);
void RampSetScale(ramp_t *ramp, int scale);
unsigned char RampIsOverflow(ramp_t *ramp);
#define RAMP_GEN_DAFAULT \
{ \
		.count = 0, \
		.XSCALE = 0, \
		.out = 0, \
		.Init = &RampInit, \
		.Calc = &LinearRampCalc, \
		.SetCounter = &RampSetCounter, \
		.ResetCounter = &RampResetCounter, \
		.SetScale = &RampSetScale, \
		.IsOverflow = &RampIsOverflow, \
} \


#endif
