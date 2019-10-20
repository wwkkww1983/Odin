#ifndef __RAMP_H
#define __RAMP_H

#include "arm_math.h"
#define PI					3.14159265358979f

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
float LinearRampCalc(ramp_t *ramp,int add);			//线性斜坡计算
float SinRampCalc(ramp_t *ramp,int add);				//正弦斜坡计算
float QuadraticRampCalc(ramp_t *ramp,int add);	//二次函数斜坡计算
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
