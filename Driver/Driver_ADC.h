#ifndef __DRIVER_ADC_H
#define __DRIVER_ADC_H
#include "BSP.h"
#define ADC_1             0
#define ADC_2             1
#define ADC_3             2
#define ADC_4             3

#define SPEED_1             0
#define SPEED_2             1
#define SPEED_3             2
#define SPEED_4             3
typedef struct adcFilterData
{
    float   gx1;
    float   gx2;
    float   gx3;
    float   previousInput;
    float   previousOutput;
}adcFilterData_t;
extern __IO uint16_t adc_raw_value[10][4]; 
extern __IO float average_dis[4];
extern float temp_speed[4];
extern __IO float averageOLED_AD[4];
void adcInit(void);
void getadc_average(__IO float *real);
void adcLowPassFilter(float *distence, float *speed);
void adcLowPassFilterInit(void);
float adcFilter(float input, struct adcFilterData *filterParameters);
void getOLEDAdc_average(__IO float *real);

#endif
