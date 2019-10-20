#ifndef __UPPERMONITOR_H
#define __UPPERMONITOR_H

#include "Driver_DTU.h"
#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct{
	TaskHandle_t xHandleTask;
	u8 Receive_data[4];
	uint8_t flag_pitch;
	uint8_t flag_yaw;
	uint8_t flag_power;
	uint8_t flag_chassis;
	uint8_t flag_chase;
	uint8_t flag_poke;
	uint8_t flag_firewheel;
	uint8_t flag_read;
	uint32_t read_schedules;
	uint32_t loops;
}upperStruct_t;

#define UPPERMONITOR_PRIORITY	    7
#define UPPERMONITOR_STACK_SIZE	  256
#define UPPERMONITOR_PERIOD				6

void GetupperMonitorReceiveData(u8 *array);
void upperMonitorUpdateTask(void *Parameters);   //上位机实时数据更新
void UpperMonitorReceiveData(u8 *array);  //接收数据处理
void upperMonitorInit(void);

#endif

