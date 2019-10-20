#ifndef __VISION_UKF_H
#define __VISION_UKF_H

#include "srcdkf.h"

#define VISION_S                   6		// states
#define VISION_M                   3		// max measurements
#define VISION_V                   6		// process noise
#define VISION_N                   3		// max observation noise

#define UKF_STATE_VEL_X 0
#define UKF_STATE_VEL_Y 1
#define UKF_STATE_VEL_Z 2
#define UKF_STATE_POS_X 3	
#define UKF_STATE_POS_Y 4
#define UKF_STATE_POS_Z 5	

#define UKF_V_VEL_X 0
#define UKF_V_VEL_Y 1
#define UKF_V_VEL_Z 2
#define UKF_V_POS_X 3
#define UKF_V_POS_Y 4
#define UKF_V_POS_Z 5

#define UKF_VEL_X visionUkfData.x[UKF_STATE_VEL_X]
#define UKF_VEL_Y visionUkfData.x[UKF_STATE_VEL_Y]
#define UKF_VEL_Z visionUkfData.x[UKF_STATE_VEL_Z]
#define UKF_POS_X visionUkfData.x[UKF_STATE_POS_X]
#define UKF_POS_Y visionUkfData.x[UKF_STATE_POS_Y]
#define UKF_POS_Z visionUkfData.x[UKF_STATE_POS_Z]

typedef struct {
	srcdkf_t *kf;
	int histIndex;
	float *x;			
	double time[2];
	float intervalTime;
} visionUkfStruct_t;

extern visionUkfStruct_t visionUkfData;

void visionUkfDoPosUpdate(float pos_x,float pos_y,float pos_z);
void visionUkfDoVelUpdate(float vel_x,float vel_y,float vel_z);
void visionUkfProcess(void);
void visionUkfStateInit(float pos_x,float pos_y,float pos_z);
void visionUkfInit(void);

#endif
