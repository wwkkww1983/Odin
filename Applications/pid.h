#ifndef __PID_H
#define __PID_H

#include "util.h"
#include "differential_calculation.h"

typedef struct {
	float setPoint;		// Last setpoint			最后的设置值
	float dState;		// Last position input	最后的状态输入
	float iState;		// Integrator state   	积分状态
	float *iGain;		// integral gain				积分增益
	float *pGain;		// proportional gain		比例增益
	float *dGain;		// derivative gain			微分增益
	float *fGain;		// low pass filter factor (1 - pole) for derivative gain			用于微分增益的低通滤波器因子

	float *pMax, *iMax, *dMax, *oMax;
	int16_t *pTrim, *iTrim, *dTrim, *fTrim;	// pointers to radio trim channels (or NULL)
	float pv_1, pv_2;
	float co_1;
	float pTerm_1;
	float iTerm_1;
	float dTerm_1;
	float sp_1;
	
	differentialDataStruct_t *differential;
} pidStruct_t;

pidStruct_t *pidInit(float *p, float *i, float *d, float *f, float *pMax, float *iMax, float *dMax, float *oMax, int16_t *pTrim, int16_t *iTrim, int16_t *dTrim, int16_t *fTrim);
float pidUpdate(pidStruct_t *pid, float setpoint, float position,float Dt);
void pidZeroIntegral(pidStruct_t *pid, float pv, float iState);
void pidZeroState(pidStruct_t *pid);

#endif


