#ifndef __TF_CARD_PARAMETER_H
#define __TF_CARD_PARAMETER_H

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "ff.h"
#include "diskio.h"

#define TFCARD_INT BSP_GPIOD10

uint8_t tfFATFS_Init(void);
uint8_t tFCardConfig(void);
uint8_t tfWriteOneParameter(uint8_t id, const uint8_t* strPara, float parameter);
uint8_t tfReadOneParameter(uint8_t id, const uint8_t* strPara, float* parameter);
uint8_t tfOverwrite(uint8_t id, const char ** strPara, float* parameter, uint16_t amount);
uint8_t tfOverread(uint8_t id, const char ** strPara, float* parameter, uint16_t amount);
uint8_t tfMotorread(uint8_t id, const char ** strPara, float* parameter, uint16_t amount);
uint8_t tfMotorwrite(uint8_t id, const char ** strPara, float* parameter, uint16_t amount);

#endif
