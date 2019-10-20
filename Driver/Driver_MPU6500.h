#ifndef __DRIVER_MPU6500_H
#define __DRIVER_MPU6500_H

#include "bsp.h"
#include "RegMap_MPU6500.h"
#include "imu.h"
#include "pid.h"

typedef struct {
  coordinateInteger_t accInteger;
	coordinateInteger_t gyroInteger;
	coordinateInteger_t magInteger;

	coordinateFloat_t acc;
	coordinateFloat_t gyro;
	coordinateFloat_t mag;
	
	s16 tempreature;
	float tempLpf;
	int16_t ak8963ASA[3];
	pidStruct_t *tempPID;
}mpu6500Data_t;

#define CONSTANT_TEMP_LENGTH	10000
#define CONSTANT_TEMP_PRES		45
#define CONSTANT_TEMP_VAULE		60.0f

/************************* 配置定义 *********************/
#define SMPLRT_DIV 				0
#define MPU6500_SPIx_ADDR 0x00
#define MPU6500_CS 				PAout(4)		//MPU6500的片选引脚
#define MPU6500_INT				BSP_GPIOE10	//MPU6500的中断信号引脚
#define MPU6500_SPI_DEFAULT \
{\
	.SPIx = SPI1,\
	.SPI_NSS = BSP_GPIOA4,\
	.SPI_SCK = BSP_GPIOA5,\
	.SPI_MISO = BSP_GPIOA6,\
	.SPI_MOSI = BSP_GPIOA7\
}

//#define MPU6500_SPI_DEFAULT \
//{\
//	.SPIx = SPI2,\
//	.SPI_NSS = BSP_GPIOB12,\
//	.SPI_SCK = BSP_GPIOB13,\
//	.SPI_MISO = BSP_GPIOC2,\
//	.SPI_MOSI = BSP_GPIOC3\
//}

extern mpu6500Data_t mpu6500Data;

void Driver_MPU6500_SPI_Write(u8 regAddr,u8 dat);
u8 Driver_MPU6500_SPI_Read(u8 regAddr);
u8 Driver_MPU6500_SPI_Reads(u8 regAddr,u8 len,u8* dat);
void Driver_MPU6500_Init(void);
void Read_MPU6500(void);
void imuTempControl(float expTemp);

#endif
