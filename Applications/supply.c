#include "supply.h"
#include "rc.h"
#include "config.h"
#include "Driver_RMMotor.h"
#include "cansend.h"

supplyStruct_t supplyData;

void supplyUpdate(void){
	supplyData.time[0] = getClockCount();
	supplyData.intervalTime = (float)(supplyData.time[0] - supplyData.time[1]);
	supplyData.time[1] = supplyData.time[0];
	supplyData.supplySpeedOut = pidUpdate(supplyData.speedPID,supplyData.supplySpeedRef,lidData.speed,supplyData.intervalTime);
}

void supplyInit(void){
	supplyData.speedPID = pidInit(&parameter[LOADED_SPEED_P], &parameter[LOADED_SPEED_I], &parameter[LOADED_SPEED_D], &parameter[LOADED_SPEED_F],	\
													&parameter[LOADED_SPEED_PM], &parameter[LOADED_SPEED_IM], &parameter[LOADED_SPEED_DM], &parameter[LOADED_SPEED_OM],	\
													NULL, NULL, NULL, NULL);
	supplyData.supplySpeedRef = 80;
}
