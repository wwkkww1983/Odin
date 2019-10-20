#ifndef __CLOCKCOUNT_H
#define __CLOCKCOUNT_H

#include "Driver_ClockCount.h"
#include "FreeRTOS.h"

//#define TIME_US  

typedef struct{
	u32 lastClockTick;
	u32 saveTimer;
	double clockTick;
	u8  index;
} clockCountStruct_t;

extern clockCountStruct_t  clockCountData;

double getClockCount(void);

#endif
