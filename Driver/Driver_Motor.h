#ifndef __DRIVER_MOTOR_H
#define __DRIVER_MOTOR_H

#include "bsp.h"
#include "BSP_GPIO.h"

#define INIT_DUTY 4000
#define MOTORS_NUM 8
#define MOTORS_SCALE 1000
#define MOTORS_AXIS 4

// Custom mixer data per motor
typedef struct motorMixer_s {
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

typedef struct {
	float CH[MOTORS_NUM];
	float throttle, rudd, pitch, roll;
	float throtlimit;
	float oldValues[MOTORS_NUM];
	float coefficient[MOTORS_NUM * MOTORS_AXIS];
}Motor_TypeDef;
 
extern Motor_TypeDef MotorData;

void motorPWMOutput(Motor_TypeDef *motor,float throt,float rudd,float pitch,float roll);
void motorOff(void);
void motorInit(void);
void motorPWMOutput(Motor_TypeDef *motor,float throt,float rudd,float pitch,float roll);

#endif


