#include "parameter.h"
#include "pid.h"

pidStruct_t *pidInit(float *p, float *i, float *d, float *f, float *pMax, float *iMax, float *dMax, float *oMax, int16_t *pTrim, int16_t *iTrim, int16_t *dTrim, int16_t *fTrim){
	pidStruct_t *pid;

	pid = (pidStruct_t *)aqDataCalloc(1, sizeof(pidStruct_t));

	pid->pMax = pMax;
	pid->iMax = iMax;
	pid->dMax = dMax;
	pid->oMax = oMax;
	pid->pGain = p;
	pid->iGain = i;
	pid->dGain = d;
	pid->fGain = f;
	pid->pTrim = pTrim;
	pid->iTrim = iTrim;
	pid->dTrim = dTrim;
	pid->fTrim = fTrim;

	return pid;
}

float pidUpdate(pidStruct_t *pid, float setpoint, float position,float Dt){
	float error;
	float p = *pid->pGain;
	float i = *pid->iGain;
	float d = (pid->dGain) ? *pid->dGain : 0.0f;
	float f = (pid->fGain) ? *pid->fGain : 1.0f;															//如果地址存在输入，则取地址内的值，没有则取1
	
	if(pid->differential)																											//如果已经加入差分系统
		setpoint = differentialCal(pid->differential,setpoint);									//差分方程计算

	error = setpoint - position;

	// calculate the proportional term                                        //计算比例项
	pid->pTerm_1 = p * error;
	if (pid->pTerm_1 > *pid->pMax) {
		pid->pTerm_1 = *pid->pMax;
	}
	else if (pid->pTerm_1 < -*pid->pMax) {
		pid->pTerm_1 = -*pid->pMax;
	}																																					//用适当的极限计算积分状态
	pid->iState += error;
	pid->iTerm_1 = i * pid->iState * Dt;
	if (pid->iTerm_1 > *pid->iMax) {
		pid->iTerm_1 = *pid->iMax;
		pid->iState = pid->iTerm_1 / i;
	}
	else if (pid->iTerm_1 < -*pid->iMax) {
		pid->iTerm_1 = -*pid->iMax;
		pid->iState = pid->iTerm_1 / i;
	}

	// derivative																															//微分
	if (pid->dGain) {																													//如果存在微分项
		error = -position;

		pid->dTerm_1 = (d * f) * (error - pid->dState);													//在此处去除时间系数
		pid->dState += f * (error - pid->dState);
		if (pid->dTerm_1 > *pid->dMax) {
				pid->dTerm_1 = *pid->dMax;
		}
		else if (pid->dTerm_1 < -*pid->dMax) {
				pid->dTerm_1 = -*pid->dMax;
		}
	}
	else {                         
		pid->dTerm_1 = 0.0f;																										//不存在则微分输出取0
	}

	pid->pv_1 = position;
	pid->sp_1 = setpoint;
	pid->co_1 = pid->pTerm_1 + pid->iTerm_1 + pid->dTerm_1;

	if (pid->co_1 > *pid->oMax) {																							//给PID输出限幅
		pid->co_1 = *pid->oMax;
	}
	else if (pid->co_1 < -*pid->oMax) {
		pid->co_1 = -*pid->oMax;
	}

	return pid->co_1;
}

void pidZeroIntegral(pidStruct_t *pid, float pv, float iState){
	if (*pid->iGain != 0.0f)
		pid->iState = iState / *pid->iGain;
	pid->dState = -pv;
	pid->sp_1 = pv;
	pid->co_1 = 0.0f;
	pid->pv_1 = pv;
	pid->pv_2 = pv;
}

void pidZeroState(pidStruct_t *pid){
	pid->setPoint = 0.0f;
	pid->dState = 0.0f;
	pid->iState = 0.0f;
	pid->co_1 = 0.0f;
	pid->pv_1 = 0.0f;
	pid->sp_1 = 0.0f;
	pid->pTerm_1 = 0.0f;
	pid->iTerm_1 = 0.0f;
	pid->dTerm_1 = 0.0f;
}


