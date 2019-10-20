#ifndef __CALI_H
#define __CALI_H

#include "NavAndData.h"

#define CALIB_SAMPLES       20      // per octant											//Ã¿45¶È
#define CALIB_MIN_ANGLE     25      // degrees
#define CALIB_EMPTY_SLOT    100.0f
#define CALIB_SCALE	    2.0f

typedef struct {
    float32_t *calibSamples;
    float32_t lastVec[3];
    float32_t min[3];
    float32_t max[3];
    float32_t bias[3];
    float32_t U[10];
    float32_t percentComplete;
} calibStruct_t;

extern calibStruct_t calibData;

extern void calibInit(void);
extern void calibDeinit(void);
extern void calibrate(float * mag);

#endif



