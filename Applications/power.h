#ifndef __POWER_H
#define __POWER_H

#include "bsp.h"
#include "Driver_powerContol.h"
#include "stdbool.h"

#define openCap   (remoteControlData.dt7Value.keyBoard.bit.SHIFT)
#define chargeVol (remoteControlData.dt7Value.keyBoard.bit.X)

#define LINK_CAP	    PDout(14)
#define LINK_CHASSIS	PDout(15)

#define autocharge

#define average_times_cap   5     //≤…µÁ»›µÁ—π
#define capChannel 					1

enum linkChassisState{
	CHASSIS_ON = 0,
	CHASSIS_OFF = 1
};

enum linkCapState{
	CAP_OFF = 0,
	CAP_ON = 1
};

typedef enum{
	linkCap = 0,
	linkCha 
}linkWay_e;

typedef struct{
  linkWay_e SwitchPolicy;
	float 	capVolMax;
	float 	capVol;
	float 	standardVol;
	float   powerLimit;
	float   warningPower;
	float   WarningPowerBuff;
	float   noJudgeTotalCurrentLimit;
	float   judgeTotalCurrentLimit;
	float   addPowerCurrent;
	float   volPIn;
  float   volCap;
	uint8_t initFlag;
	uint32_t loops;
	uint8_t step;
	bool    linkCapFlag;
	bool    chargeFlag;
	bool		CapError;
	bool    rotateFast;
}powerstruct_t;	
	
extern powerstruct_t powerData;

void sendPowerDataInit(void);
void powerDataUpdata(void);
void linkAimUpdata(void);
void powerLinkInit(void);
void adcInit_cap(void);
void powerDataReceive(u8 *array);
void powerDataTransportTask(void *Parameters);

#endif
