#ifndef __AUTO_INFANTRY_H
#define __AUTO_INFANTRY_H

#include "auto_task.h"
#include "keyboard.h"

#define AUTOMATIC_AIM_DEVICE	0x05
#define ACTIVE_BUFF_DEVICE	0x05
#define BULLET_TRANSFER_DEVICE	0x03
#define AVIOD_DEVICE	0x02
#define USE_CHANGE_HEAD 0

enum{
	INFANTRY_MANUAL = 0,										//手动
	INFANTRY_BULLET_TRANSFER = 3,						//R:子弹交接
	INFANTRY_ACTIVE_BUFF = 4,								//Z:击打神符
	INFANTRY_AUTOMATIC_AIM = 5,							//V:辅助瞄准
};

extern AutoTaskStruct_t infantryAutoData;

void infantryAutoTaskUpdate(void);

#endif

