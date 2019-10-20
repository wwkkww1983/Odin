#ifndef __INDOOR_H
#define __INDOOR_H

#include "bsp.h"
#include "BSP_GPIO.h"

typedef struct
{                
	float err;
	float output;
	float old;
	float P;
	float I;
	float D;
}parts_PID;

typedef struct
{
	float kp;
	float kd;
	float ki;

}_st_pos_pid;

typedef struct 
{
	parts_PID front;
	parts_PID left;
	parts_PID pitch;
	parts_PID roll;
}position_parts;

typedef struct 
{
	position_parts distance;
	position_parts speed;
	position_parts angle;
}position_expect;

#define INDOOR_STACK_SIZE 512
#define INDOOR_PRIORITY 9

void flowPOSCtrl(float T,float exp_POS_x,float exp_POS_y,float current_POS_x,float current_POS_y);
void flowSpeedCtrl(float T,float exp_speed_x,float exp_speed_y,float current_speed_x,float current_speed_y);
void indoorPOSTask(void *Parameters);
void indoorInit(void);

#endif

