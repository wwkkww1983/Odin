#ifndef __DRIVER_PITCHSTEERINGGEAR_H
#define __DRIVER_PITCHSTEERINGGEAR_H

#include "stdbool.h"
#include "stm32f4xx.h"

#define SERVO_LENGTH 10000
#define SERVO_PRESCALER 168

#define SERVO_ANGLE_MIN 250
#define SERVO_ANGLE_MAX 1000

#define SERVO_MOTOR_NO1	TIM4->CCR3
#define SERVO_MOTOR_NO2 TIM4->CCR4

void pitchServoConfig(void);
void pitchServoAdjust(uint16_t angle);

#endif
