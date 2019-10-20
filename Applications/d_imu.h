
#ifndef __D_IMU_H
#define __D_IMU_H

#include "stm32f4xx.h"
#include "FreeRTOS_board.h"
#include "bsp.h"
#include "Util.h"
#include "stdbool.h"




extern imusensorStruct_t imuSensorOfChassisData;
void dimuUpdateTask(void);
void dimuInit(void);
void dIMUTare(void); 

#endif




