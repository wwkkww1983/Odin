#ifndef __DRIVER_FLASH_H
#define __DRIVER_FLASH_H

#include "stm32f4xx.h"

#define FLASH_START_ADDR   ((uint32_t)0x080E0000)
#define FLASH_END_ADDR     ((uint32_t)0x080FFFFF)
#define FLASH_RETRIES      3



extern int flashAddress(uint32_t startAddr, uint32_t *data, uint32_t len);
extern int flashErase(uint32_t startAddr, uint32_t len);
extern uint32_t flashStartAddr(void);
extern uint32_t flashSerno(uint8_t n);

#endif

