#ifndef __AUTO_UAV_H
#define __AUTO_UAV_H

#include "auto_task.h"

enum{
	UAV_MANUAL = 0,															//��̨���ֶ�����
	UAV_AUTOMATIC_AIM = 5,													//V��������׼
};

extern AutoTaskStruct_t uavAutoData;

void uavAutoTaskUpdate(void);

#endif
