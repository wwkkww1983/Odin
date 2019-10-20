#ifndef __DRIVER_POWER_H
#define __DRIVER_POWER_H
#include "DRIVER_VL53L0X.h"
#include "Driver_RMMotor.h"
#include "BSP.h"

#define POWER_SCALE 4.0283203f

typedef struct{
	vs16 current[4];
	vs16 voltage;
	uint8_t	powerRank;
}powerRawDateStruct_t;

typedef struct{
	float current[4];
	float voltage;
	uint8_t	powerRank;
}powerRealDateStruct_t;

typedef struct{
	double raw_value;
	double xbuf[18];
	double ybuf[18];
	double filtered_value;
}Filter_t;

extern powerRawDateStruct_t powerRawDate;
extern powerRealDateStruct_t powerRealDate;
extern firstOrderFilterData_t powerFirstOrderFilters[4];
extern double NUM[18];
extern double DEN[18];

void powerRealData(powerRawDateStruct_t *rawdata,powerRealDateStruct_t *realdata);
void powerRawData(CanRxMsg *CAN_RX_Msg,powerRawDateStruct_t *rawdata,motorCanDataRecv_t *chassisData);
void powerInitFirstOrderFilter(void);
double Chebyshev50HzLowPassFilter(Filter_t *F);

#endif
