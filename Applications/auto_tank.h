#ifndef __AUTO_TANK_H
#define __AUTO_TANK_H

#include "auto_task.h"

enum{
	TANK_MANUAL = 0,											//�ֶ�									
	TANK_BULLET_TRANSFER,									//X:��ҩ����
	TANK_MORTAR,													//CTRL:�Ȼ���ģʽ
	TANK_BULLET_SUPPLY,										//R:����վ����	
	TANK_ACTIVE_BUFF,											//Z:�������
	TANK_AUTOMATIC_AIM,										//V:������׼
	TANK_TURN_AROUND											//Q:���ٵ�ͷ
																				//G:
};

extern AutoTaskStruct_t tankAutoData;

void tankAutoTaskUpdate(void);

#endif

