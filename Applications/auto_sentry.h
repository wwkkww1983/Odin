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
	SENTRY_MANUAL = 0,					//�ֶ�
	SENTRY_PATROL,							//Ѳ��ģʽ
	SENTRY_ATTACK,							//����ģʽ
	SENTRY_UNTER_ATTACK					//���ģʽ
};

extern AutoTaskStruct_t sentryAutoData;

void sentryAutoTaskUpdate(void);

#endif
