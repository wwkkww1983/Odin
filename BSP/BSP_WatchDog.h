#ifndef __BSP_WATCHDOG_H
#define __BSP_WATCHDOG_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdio.h"

void BSP_IWDG_Init(u8 prer,u16 rlr);
void BSP_IWDG_Feed(void);
void BSP_WWDG_Init(u8 tr,u8 wr,u32 fprer,u8 PreemptionPriority,u8 SubPriority);

#endif
