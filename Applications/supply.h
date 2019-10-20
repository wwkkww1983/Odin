#ifndef __SUPPLY_H
#define __SUPPLY_H

#include "BSP.h"
#include "Util.h"
#include "pid.h"
#include "Driver_RMMotor.h"
#include "control.h"

typedef struct
{
	pidStruct_t *speedPID;
	float supplySpeedRef;  		  //�������ٶ�����ֵ
  float supplySpeedOut; 		  //������PID���ֵ
	
	float intervalTime;
	double time[2];
}supplyStruct_t;

extern supplyStruct_t supplyData;

void supplyUpdate(void);
void supplyInit(void);

#endif

