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
	supplyData.speedPID = pidInit(&parameter[SHOOT_DIAL_S_P], &parameter[SHOOT_DIAL_S_I], &parameter[SHOOT_DIAL_S_D], &parameter[SHOOT_DIAL_S_F],	\
													&parameter[SHOOT_DIAL_S_PM], &parameter[SHOOT_DIAL_S_IM], &parameter[SHOOT_DIAL_S_DM], &parameter[SHOOT_DIAL_S_OM],	\
													NULL, NULL, NULL, NULL);
	supplyData.supplySpeedRef = 80;
}
