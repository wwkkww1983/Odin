
#include "Driver_MS5611.h"
#include "FreeRTOS_board.h"
#include "math.h"
#include "util.h"
#include "bsp.h"
//#include "filter.h"

ms5611Struct_t ms5611Data;

#define BARO_CAL_CNT 200

BSP_SPI_TypeDef MS5611_SPI_Base = MS5611_SPI_DEFAULT;
BSP_SPI_TypeDef *MS5611_SPI = &MS5611_SPI_Base;


void MS5611_SPI_Write(uint8_t command)
{
	MS5611_CS = 0;
	BSP_SPI_ReadWriteByte(MS5611_SPI,command);
	MS5611_CS = 1;
}

uint8_t MS5611_SPI_Read(uint8_t command)
{
	uint8_t value;
	MS5611_CS = 0;
	value = BSP_SPI_ReadWriteByte(MS5611_SPI,command);
	MS5611_CS = 1;
	return value;
}

uint32_t MS5611_SPI_Reads(uint8_t command,uint8_t n)
{
	uint32_t value;
	MS5611_CS = 0;
	for (uint8_t i = 0; i < n; i++)
		value = (value<<8)|BSP_SPI_ReadWriteByte(MS5611_SPI,command);
	MS5611_CS = 1;
	return value; 
}

static uint32_t MS5611_Read_SPI_Adc()
{
	uint8_t rxbuf[3];
	MS5611_CS = 0;
	for(uint8_t i;i<3;i++)
		rxbuf[i]=BSP_SPI_ReadWriteByte(MS5611_SPI,CMD_ADC_READ);
	MS5611_CS = 1;
	return (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
}

void MS5611_SPI_Reset()
{
	MS5611_SPI_Write(CMD_RESET);
	vTaskDelay(10);
}

static void MS5611_Start_SPI_Ut()
{
	MS5611_SPI_Write(CMD_ADC_CONV+CMD_ADC_D2+CMD_ADC_4096);
}

static void MS5611_Get_SPI_Ut()
{
	ms5611Data.d2[ms5611Data.slot]=MS5611_Read_SPI_Adc();
}

static void MS5611_Start_SPI_Up()
{
	MS5611_SPI_Write(CMD_ADC_CONV+CMD_ADC_D1+CMD_ADC_4096);
}

static void MS5611_Get_SPI_Up()
{
	ms5611Data.d1[ms5611Data.slot]=MS5611_Read_SPI_Adc();
 }

void MS5611_SPI_Update()
{
	switch (ms5611Data.step) {
	  case 0:
			MS5611_Start_SPI_Ut();
			break;
		case 1:
			ms5611Data.slot = (ms5611Data.slot + 1) % MS5611_SLOTS;
			break;
		case 2:
			MS5611_Get_SPI_Ut();
			break;
		case 3:
			MS5611_Start_SPI_Up();
			break;
		case 4:
			break;
		case 5:
			MS5611_Get_SPI_Up();
			break;
	}
	ms5611Data.step = (ms5611Data.step + 1) % 6;
}

u8 ms5611_ok;

void MS5611_SPI_Init(void)
{
  BSP_SPI_Init(MS5611_SPI);
	BSP_SPIx_SetSpeed(MS5611_SPI,SPI_BaudRatePrescaler_16);
	MS5611_SPI_Reset();
	for(uint8_t i;i<8;i++){
		ms5611Data.p[i]=MS5611_SPI_Reads(CMD_PROM_RD + i*2,2);
		vTaskDelay(2);
	}
	utilFilterInit(&ms5611Data.tempFilter, (1.0f / 13.0f), DIMU_TEMP_TAU, IMU_ROOM_TEMP);
}

void MS5611_SPI_Decode()
{
	uint32_t rawTemp, rawPres;
	uint8_t *ptr;
	int32_t dT;
	int32_t temp;
	int64_t off;
	int64_t sens;
	int divisor;
	int i;

	rawTemp = 0;
	rawPres = 0;
	divisor = MS5611_SLOTS;
	for (i = 0; i < MS5611_SLOTS; i++) {
			if (i == ms5611Data.slot) {
				divisor--;
			}
			else {
				ptr = (uint8_t *)&ms5611Data.d2[i];
				rawTemp += (ptr[1]<<16 | ptr[2]<<8 | ptr[3]);

				ptr = (uint8_t *)&ms5611Data.d1[i];
				rawPres += (ptr[1]<<16 | ptr[2]<<8 | ptr[3]);
			}
	}
	// temperature
	dT = rawTemp / divisor - (ms5611Data.p[5]<<8);
	temp = (int64_t)dT * ms5611Data.p[6] / (1<<23) + 2000;
	ms5611Data.rawTemp = temp / 100.0f;
	ms5611Data.temp = utilFilter(&ms5611Data.tempFilter, ms5611Data.rawTemp);
	// pressure
	off = ((int64_t)ms5611Data.p[2]<<16) + (((int64_t)dT * ms5611Data.p[4])>>7);
	if (off < -8589672450)
		off = -8589672450;
	else if (off > 12884705280)
		off = 12884705280;
	sens = ((int64_t)ms5611Data.p[1]<<15) + (((int64_t)dT * ms5611Data.p[3])>>8);
	if (sens < -4294836225)
		sens = -4294836225;
	else if (sens > 6442352640)
		sens = 6442352640;
	ms5611Data.pres = ((int64_t)rawPres * sens / (1<<21) / divisor - off) * (1.0f / (1<<15));
}


