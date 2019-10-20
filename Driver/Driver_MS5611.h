#ifndef _MS5611_H
#define _MS5611_H

#include "stm32f4xx.h"
#include "Util.h"

#define MS5611_ADDR             0x77   //0xee //

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR							0x08	//CMD_ADC_4096

#define MS5611_CS 				PBout(9)		//MS5611µÄÆ¬Ñ¡Òý½Å
#define MS5611_SPI_DEFAULT \
{\
	.SPIx = SPI2,\
	.SPI_NSS = BSP_GPIOB9,\
	.SPI_SCK = BSP_GPIOD3,\
	.SPI_MISO = BSP_GPIOB14,\
	.SPI_MOSI = BSP_GPIOB15\
}

#define IMU_ROOM_TEMP		20.0f
#define DIMU_TEMP_TAU	    5.0f
#define MS5611_SLOTS		    8				// ~13 Hz

typedef struct {
	utilFilter_t tempFilter;
	volatile uint8_t slot;
	volatile uint32_t d1[MS5611_SLOTS];
	volatile uint32_t d2[MS5611_SLOTS];
	uint16_t p[8];
	uint8_t step;
	float rawTemp;
	volatile float temp;
	volatile float pres;
} ms5611Struct_t;

void MS5611_SPI_Update(void);
void MS5611_SPI_Init(void);
void MS5611_SPI_Decode(void);

extern ms5611Struct_t ms5611Data;
#endif
