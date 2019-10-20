#ifndef __DRIVER_ITV_H
#define __DRIVER_ITV_H
#include "BSP.h"

#define ITV_R_F_TOP  		TIM3->CCR1
#define ITV_R_F_BOTTOM  TIM3->CCR2
#define ITV_L_F_TOP  		TIM3->CCR3
#define ITV_L_F_BOTTOM  TIM3->CCR4

#define ITV_L_B_TOP  		TIM4->CCR1
#define ITV_L_B_BOTTOM  TIM4->CCR2
#define ITV_R_B_TOP 		TIM4->CCR3
#define ITV_R_B_BOTTOM	TIM4->CCR4

void Driver_ITV_Init(void);





#endif
