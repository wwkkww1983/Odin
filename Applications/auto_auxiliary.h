#ifndef __AUTO_AUXILIARY_H
#define __AUTO_AUXILIARY_H

#include "auto_task.h"

#define PRESS_W (remoteControlData.dt7Value.keyBoard.bit.W)
#define PRESS_A (remoteControlData.dt7Value.keyBoard.bit.A)
#define PRESS_S (remoteControlData.dt7Value.keyBoard.bit.S)
#define PRESS_D (remoteControlData.dt7Value.keyBoard.bit.D)

enum{
	AUXILIARY_MANUAL = 0,											//手动
	AUXILIARY_GRAB,													  //X：抓取弹药箱	
	AUXILIARY_RESCUE = 4,											//Z：救援
	AUXILIARY_BULLET_TRANSFER = 5							//V：交互
	
};

extern AutoTaskStruct_t auxiliaryAutoData;

void auxiliaryAutoTaskUpdate(void);

#endif
