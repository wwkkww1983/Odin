#include "ramp.h"

/****************************外部调用函数*******************************/
void RampInit(ramp_t *ramp, int XSCALE);
float LinearRampCalc(ramp_t *ramp,int add);			//线性斜坡计算
float SinRampCalc(ramp_t *ramp,int add);				//正弦斜坡计算
float QuadraticRampCalc(ramp_t *ramp,int add);	//二次函数斜坡计算
void RampSetCounter(ramp_t *ramp, int count);
void RampResetCounter(ramp_t *ramp);
void RampSetScale(ramp_t *ramp, int scale);
unsigned char RampIsOverflow(ramp_t *ramp);
/**********************************************************************/

//斜坡初始化
void RampInit(ramp_t *ramp, int XSCALE)
{
	ramp->XSCALE = XSCALE;
	ramp->out = 0;
	ramp->count = 0;
}

//线性斜坡计算
float LinearRampCalc(ramp_t *ramp,int add)
{
	float result;
	ramp->count += add;
	if(ramp->count >= ramp->XSCALE)
	{
		ramp->count = ramp->XSCALE;
		ramp->out = 1;
		result = 1;
	}
	else
	{
		result = (float)ramp->count / (float)ramp->XSCALE;
	}
	return result;
}

//正弦斜坡计算
float SinRampCalc(ramp_t *ramp,int add)
{
	float result;
	ramp->count += add;
	if(ramp->count >= ramp->XSCALE)
	{
		ramp->count = ramp->XSCALE;
		ramp->out = 1;
		result = 1;
	}
	else
	{
		result = arm_sin_f32(2*PI*((float)ramp->count / (float)ramp->XSCALE));
	}
	return result;
}

//二次函数斜坡计算
float QuadraticRampCalc(ramp_t *ramp,int add)
{
	float result;
	ramp->count += add;
	if(ramp->count >= ramp->XSCALE)
	{
		ramp->count = ramp->XSCALE;
		ramp->out = 1;
		result = 1;
	}
	else
	{
		result = 1 - 2 * ((ramp->XSCALE - ramp->count) * (ramp->XSCALE - ramp->count)) / (ramp->XSCALE * ramp->XSCALE);
	}
	return result;
}

void RampSetCounter(ramp_t *ramp, int count)
{
	ramp->count = count;
}

void RampResetCounter(ramp_t *ramp)
{
	ramp->count = 0;
}

void RampSetScale(ramp_t *ramp, int scale)
{
	ramp->XSCALE = scale;
}

unsigned char RampIsOverflow(ramp_t *ramp)
{
	if(ramp->count >= ramp->XSCALE)
	{
		ramp->count = ramp->XSCALE;
		ramp->out = 1;
	}
	return ramp->out;
}

