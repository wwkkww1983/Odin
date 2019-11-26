#ifndef __APP_INIT_H
#define __APP_INIT_H

#include "FreeRTOS_board.h"

#define INIT_PRIORITIES 2
#define INIT_SIZE 1600

typedef struct {
	TaskHandle_t xHandleTask;	
	EventGroupHandle_t eventGroups;
}taskInit_t;

void appInit(void *Parameters);

extern taskInit_t taskInitData;

#endif 





