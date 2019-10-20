#ifndef __BSP_SPI_HW_H
#define __BSP_SPI_HW_H

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdio.h"

#include "BSP_GPIO.h"

typedef struct
{
	SPI_TypeDef *SPIx;								//SPI��
	BSP_GPIOSource_TypeDef *SPI_NSS;	//SPI_NSS ����
	BSP_GPIOSource_TypeDef *SPI_SCK;	//SPI_SCK ����
	BSP_GPIOSource_TypeDef *SPI_MISO;	//SPI_MISO����
	BSP_GPIOSource_TypeDef *SPI_MOSI;	//SPI_MOSI����
}BSP_SPI_TypeDef;	

void BSP_SPI_Init(BSP_SPI_TypeDef* BSP_SPIx);
void BSP_SPIx_SetSpeed(BSP_SPI_TypeDef* BSP_SPIx,u8 SPI_BaudRatePrescaler);
u8	 BSP_SPI_ReadWriteByte(BSP_SPI_TypeDef* BSP_SPIx,u8 TxData);

/**************����SPI����*******************/
extern BSP_SPI_TypeDef BSP_SPI1;
extern BSP_SPI_TypeDef BSP_SPI2;
extern BSP_SPI_TypeDef BSP_SPI3;


#endif
