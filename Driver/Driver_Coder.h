#ifndef __DRIVER_CODER_H
#define __DRIVER_CODER_H

#include "bsp.h"

#define CODER_AHEAD_PIN						  BSP_GPIOD14	//±àÂëÆ÷Ç°
#define CODER_BACK_PIN						  BSP_GPIOE13	//±àÂëÆ÷ºó
#define CODER_AHEAD_PTY             0x01
#define CODER_AHEAD_STY             0x01
#define CODER_BACK_PTY              0x01
#define CODER_BACK_STY              0x02

typedef struct {
	int32_t coderAhead;
	int32_t coderBack;
	int32_t xLocation;
} coderStruct_t;

extern coderStruct_t coderData;

void CoderInit(void);

#endif


