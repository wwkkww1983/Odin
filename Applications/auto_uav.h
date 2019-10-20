#ifndef __AUTO_UAV_H
#define __AUTO_UAV_H

#include "auto_task.h"

enum{
	UAV_MANUAL = 0,															//云台手手动任务
	UAV_AUTOMATIC_AIM = 5,													//V：辅助瞄准
};

extern AutoTaskStruct_t uavAutoData;

void uavAutoTaskUpdate(void);

#endif
