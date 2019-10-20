#ifndef __AUTO_TANK_H
#define __AUTO_TANK_H

#include "auto_task.h"

enum{
	TANK_MANUAL = 0,											//手动									
	TANK_BULLET_TRANSFER,									//X:弹药交接
	TANK_MORTAR,													//CTRL:迫击炮模式
	TANK_BULLET_SUPPLY,										//R:补给站交互	
	TANK_ACTIVE_BUFF,											//Z:击打神符
	TANK_AUTOMATIC_AIM,										//V:辅助瞄准
	TANK_TURN_AROUND											//Q:快速掉头
																				//G:
};

extern AutoTaskStruct_t tankAutoData;

void tankAutoTaskUpdate(void);

#endif

