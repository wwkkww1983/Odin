#ifndef __AUTO_SENTRY_H
#define __AUTO_SENTRY_H

#include "auto_task.h"
#include "Driver_Coder.h"

#define PATROLSPEED_MAX 4000.0f
#define SPEEDHIGH 6500 
#define SPEEDLOW 2000
#define X_RANGE 150
#define Y_RANGE 120

enum{
	SENTRY_MANUAL = 0,					//手动
	SENTRY_PATROL,							//巡逻模式
	SENTRY_ATTACK,							//攻击模式
	SENTRY_UNTER_ATTACK					//躲避模式
};

extern AutoTaskStruct_t sentryAutoData;

void sentryAutoTaskUpdate(void);

#endif
