#include "Driver_MPU6500.h"
#include "imu.h"
#include "processing.h"
#include "Util.h"

#define MPU6500_Write(regAddr,dat) 						Driver_MPU6500_SPI_Write(regAddr,dat)
#define MPU6500_Read(regAddr) 									Driver_MPU6500_SPI_Read(regAddr)
#define MPU6500_Reads(regAddr,len,dat)					Driver_MPU6500_SPI_Reads(regAddr,len,dat)
#define MPU9250_AK8963_Write(regAddr,dat) 			Driver_MPU9250_AK8963_SPI_Write(regAddr,dat)
#define MPU9250_AK8963_Reads(regAddr,len,dat) 	Driver_MPU9250_AK8963_SPI_Reads(regAddr,len,dat)


BSP_SPI_TypeDef MPU6500_SPI_Base = MPU6500_SPI_DEFAULT;
BSP_SPI_TypeDef *MPU6500_SPI = &MPU6500_SPI_Base;

mpu6500Data_t mpu6500Data;
/*
***************************************************
函数名：Driver_MPU6500_SPI_Write
功能：SPI方式向MPU6500的指定寄存器写入一个字节数据
入口参数：	regAddr：MPU6500的寄存器地址
					dat：要写入的数据
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void Driver_MPU6500_SPI_Write(u8 regAddr,u8 dat){
	MPU6500_CS = 0;
	BSP_SPI_ReadWriteByte(MPU6500_SPI,regAddr);
	BSP_SPI_ReadWriteByte(MPU6500_SPI,dat);
	MPU6500_CS = 1;
}

/*
***************************************************
函数名：Driver_MPU6500_SPI_Read
功能：SPI方式从MPU6500的指定寄存器读出一个字节数据
入口参数：	regAddr：MPU6500的寄存器地址
返回值：dat：读出的数据
应用范围：外部调用
备注：
***************************************************
*/
u8 Driver_MPU6500_SPI_Read(u8 regAddr){
	u8 dummy = 0;	//假写数据
	u8 dat = 0;		//读取的数据
	MPU6500_CS = 0;
	dat = BSP_SPI_ReadWriteByte(MPU6500_SPI,dummy);
	MPU6500_CS = 1;
	return dat;
}

/*
***************************************************
函数名：Driver_MPU6500_SPI_Reads
功能：SPI方式从MPU6500指定寄存器读出指定长度字节数据
入口参数：	regAddr：MPU6500的寄存器地址
					len：读出长度
					dat：要读出的数据的指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
u8 Driver_MPU6500_SPI_Reads(u8 regAddr,u8 len,u8* dat){
	u32 i = 0;
	u8 dummy = 0x00;
	MPU6500_CS = 0;
	BSP_SPI_ReadWriteByte(MPU6500_SPI,MPU6500_I2C_READ | regAddr);
	while(i < len){
		dat[i++] = BSP_SPI_ReadWriteByte(MPU6500_SPI,dummy);
	}
	MPU6500_CS = 1;
	return 0;
}

/*
***************************************************
函数名：Driver_MPU9250_AK8963_SPI_Write
功能：SPI方式向MPU9250内部AK8963的指定寄存器写入一个字节数据
入口参数：	regAddr：AK8963的寄存器地址
					dat：要写入的数据
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
u8 Driver_MPU9250_AK8963_SPI_Write(u8 regAddr, u8 dat){
	u16 timeout = 0;
	u8 status = 0;

	Driver_MPU6500_SPI_Write(MPU6500_I2C_SLV4_ADDR,MPU9250_AK8963_I2C_ADDR);
	vTaskDelay(2);
	Driver_MPU6500_SPI_Write(MPU6500_I2C_SLV4_REG,regAddr);
	vTaskDelay(2);
	Driver_MPU6500_SPI_Write(MPU6500_I2C_SLV4_DO,dat);
	vTaskDelay(2);
	Driver_MPU6500_SPI_Write(MPU6500_I2C_SLV4_CTRL,MPU6500_I2C_SLV4_EN);
	vTaskDelay(2);
	
	while((status & MPU6500_I2C_SLV4_DONE) == 0){
		status = Driver_MPU6500_SPI_Read(MPU6500_I2C_MST_STATUS);
		if (timeout++ > 50)
			return 2;
		vTaskDelay(2);
	}
	
	if (status & MPU6500_I2C_SLV4_NACK)
		return 3;
	return 0;
}

/*
***************************************************
函数名：Driver_MPU9250_AK8963_SPI_Reads
功能：SPI方式从MPU9250内部AK8963的指定寄存器读出指定长度字节数据
入口参数：	regAddr：AK8963的寄存器地址
					len：读出长度
					dat：要读出的数据的指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
u8 Driver_MPU9250_AK8963_SPI_Reads(u8 regAddr, u8 len, u8* dat){
	volatile u8 index = 0;
	volatile u8 status = 0;
	u16 timeout = 0;

	Driver_MPU6500_SPI_Write(MPU6500_I2C_SLV4_ADDR,MPU9250_AK8963_I2C_ADDR | MPU6500_I2C_READ);
	vTaskDelay(2);
	while(index < len){
		Driver_MPU6500_SPI_Write(MPU6500_I2C_SLV4_REG,regAddr + index);
		vTaskDelay(2);
		Driver_MPU6500_SPI_Write(MPU6500_I2C_SLV4_CTRL,MPU6500_I2C_SLV4_EN);
		vTaskDelay(2);
		
		while((status & MPU9250_I2C_SLV4_DONE) == 0){
			status = Driver_MPU6500_SPI_Read(MPU6500_I2C_MST_STATUS);
			if (timeout++ > 50)
				return 2;
			vTaskDelay(2);
		}

		dat[index] = Driver_MPU6500_SPI_Read(MPU6500_I2C_SLV4_DI);
		vTaskDelay(2);
		index++;
	}
	return 0;
}

/*
***************************************************
函数名：Driver_MPU6500_Init
功能：MPU6500初始化配置
入口参数：	无
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
float mpu6500_PidConfig[]={
1500,190,0,0.25f,7000,4000,4000,7000
};


void Driver_MPU6500_Init(void){
	uint8_t data = 0, state = 0;
	uint8_t response[3] = {0, 0, 0};
	
	/****************** 单片机内部资源设置 ********************/
	BSP_SPI_Init(MPU6500_SPI);
	BSP_SPIx_SetSpeed(MPU6500_SPI,SPI_BaudRatePrescaler_16);
//	BSP_GPIO_EXIT_Init(MPU6500_INT,EXTI_Trigger_Rising,1,0);
	//恒温定时器初始化
//	BSP_TIM_PWM_Init(TIM4,CONSTANT_TEMP_LENGTH-1,CONSTANT_TEMP_PRES-1,BSP_GPIOD12,NULL,NULL,NULL);  						  
	TIM4->CCR1 = 0;	
	mpu6500Data.tempPID = pidInit(&mpu6500_PidConfig[0],&mpu6500_PidConfig[1],&mpu6500_PidConfig[2],&mpu6500_PidConfig[3],
														&mpu6500_PidConfig[4],&mpu6500_PidConfig[5],&mpu6500_PidConfig[6],&mpu6500_PidConfig[7],
														NULL, NULL, NULL, NULL);
	
	/****************** MPU6500初始化设置 ********************/
	//MPU6500 Reset
	
	MPU6500_Write(MPU6500_PWR_MGMT_1, MPU6500_RESET);
	vTaskDelay(100);
	//MPU6500 Set Clock Source
	MPU6500_Write(MPU6500_PWR_MGMT_1,  MPU6500_CLOCK_PLLGYROZ);
	vTaskDelay(10);
	//MPU6500 Set Interrupt
	MPU6500_Write(MPU6500_INT_PIN_CFG,  MPU6500_INT_ANYRD_2CLEAR);
	vTaskDelay(10);
	MPU6500_Write(MPU6500_INT_ENABLE, ENABLE);
	vTaskDelay(10);
	//MPU6500 Set Sensors
	MPU6500_Write(MPU6500_PWR_MGMT_2, MPU6500_XYZ_GYRO & MPU6500_XYZ_ACCEL);
	vTaskDelay(10);
	//MPU6500 Set SampleRate
	//SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
	MPU6500_Write(MPU6500_SMPLRT_DIV, SMPLRT_DIV);
	vTaskDelay(10);
	//MPU6500 Set Full Scale Gyro Range
	//Fchoice_b[1:0] = [00] enable DLPF
	MPU6500_Write(MPU6500_GYRO_CONFIG, (MPU6500_FSR_2000DPS << 3));
	vTaskDelay(10);
	//MPU6500 Set Full Scale Accel Range PS:2G
	MPU6500_Write(MPU6500_ACCEL_CONFIG, (MPU6500_FSR_16G << 3));
	vTaskDelay(10);
	//MPU6500 Set Accel DLPF
	data = MPU6500_Read(MPU6500_ACCEL_CONFIG2);
	data |= MPU6500_ACCEL_DLPF_41HZ;
	vTaskDelay(10);
	MPU6500_Write(MPU6500_ACCEL_CONFIG2, data);
	vTaskDelay(10);
	//MPU6500 Set Gyro DLPF
	MPU6500_Write(MPU6500_CONFIG, MPU6500_GYRO_DLPF_41HZ);
	vTaskDelay(10);
#ifdef MAG9250_ENABLE
	//MPU6500 Set SPI Mode
	state = MPU6500_Read(MPU6500_USER_CTRL);
	vTaskDelay(10);
	MPU6500_Write(MPU6500_USER_CTRL, state | MPU6500_I2C_IF_DIS);
	vTaskDelay(10);
	state = MPU6500_Read(MPU6500_USER_CTRL);
	vTaskDelay(10);
	MPU6500_Write(MPU6500_USER_CTRL, state | MPU6500_I2C_MST_EN);
	vTaskDelay(10);
	
	/****************** AK8963初始化设置 ********************/
	//reset AK8963
	MPU9250_AK8963_Write(MPU9250_AK8963_CNTL2, MPU9250_AK8963_CNTL2_SRST);
	vTaskDelay(20);

	MPU9250_AK8963_Write(MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	vTaskDelay(10);
	MPU9250_AK8963_Write(MPU9250_AK8963_CNTL, MPU9250_AK8963_FUSE_ROM_ACCESS);
	vTaskDelay(10);
	
	//AK8963 get calibration data
	MPU9250_AK8963_Reads(MPU9250_AK8963_ASAX, 3, response);
	//AK8963_SENSITIVITY_SCALE_FACTOR
	//AK8963_ASA[i++] = (s16)((data - 128.0f) / 256.0f + 1.0f) ;
	mpu6500Data.ak8963ASA[0] = (s16)(response[0]) + 128;
	mpu6500Data.ak8963ASA[1] = (s16)(response[1]) + 128;
	mpu6500Data.ak8963ASA[2] = (s16)(response[2]) + 128;
	vTaskDelay(10);
	MPU9250_AK8963_Write(MPU9250_AK8963_CNTL, MPU9250_AK8963_POWER_DOWN);
	vTaskDelay(10);
	//
	MPU6500_Write(MPU6500_I2C_MST_CTRL, 0x5D);
	vTaskDelay(10);
	MPU6500_Write(MPU6500_I2C_SLV0_ADDR, MPU9250_AK8963_I2C_ADDR | MPU6500_I2C_READ);
	vTaskDelay(10);
	MPU6500_Write(MPU6500_I2C_SLV0_REG, MPU9250_AK8963_ST1);
	vTaskDelay(10);
	MPU6500_Write(MPU6500_I2C_SLV0_CTRL, 0x88);
	vTaskDelay(10);
	//
	MPU9250_AK8963_Write(MPU9250_AK8963_CNTL, MPU9250_AK8963_CONTINUOUS_MEASUREMENT);
	vTaskDelay(10);

	//
	MPU6500_Write(MPU6500_I2C_SLV4_CTRL, 0x09);
	vTaskDelay(10);
	//
	MPU6500_Write(MPU6500_I2C_MST_DELAY_CTRL, 0x81);
	vTaskDelay(100);
#endif
}

void Read_MPU6500(void){
	uint8_t data[22] = {0};
	uint8_t list;
#ifdef MAG9250_ENABLE
	list = 22;
#else
	list = 14;
#endif
	MPU6500_Reads(MPU6500_ACCEL_XOUT_H, list, data);	//14 or 22
	
	mpu6500Data.accInteger.x = (data[0] << 8) | data[1];
	mpu6500Data.accInteger.y = (data[2] << 8) | data[3];
	mpu6500Data.accInteger.z = (data[4] << 8) | data[5];
	mpu6500Data.tempreature = (data[6] << 8) | data[7];
	mpu6500Data.gyroInteger.x = (data[8]  << 8) | data[9];
	mpu6500Data.gyroInteger.y = (data[10] << 8) | data[11];
	mpu6500Data.gyroInteger.z = (data[12] << 8) | data[13];
	
	mpu6500Data.acc.x = (float)LowPass_Filter(mpu6500Data.accInteger.x,&sensorRotate.lowPassFilterData[ACCEL_X_500HZ_LOWPASS]);
	mpu6500Data.acc.y = (float)LowPass_Filter(mpu6500Data.accInteger.y,&sensorRotate.lowPassFilterData[ACCEL_Y_500HZ_LOWPASS]);
	mpu6500Data.acc.z = (float)LowPass_Filter(mpu6500Data.accInteger.z,&sensorRotate.lowPassFilterData[ACCEL_Z_500HZ_LOWPASS]);
	
	mpu6500Data.gyro.x = meanRecursiveFilter(mpu6500Data.gyroInteger.x,0);
	mpu6500Data.gyro.y = meanRecursiveFilter(mpu6500Data.gyroInteger.y,1);
	mpu6500Data.gyro.z = meanRecursiveFilter(mpu6500Data.gyroInteger.z,2);

#ifdef MAG9250_ENABLE
	mpu6500Data.magInteger.x = (data[16] << 8) | data[15];
	mpu6500Data.magInteger.y = (data[18] << 8) | data[17];
	mpu6500Data.magInteger.z = (data[20] << 8) | data[19];
	
	mpu6500Data.mag.x = ((long)mpu6500Data.magInteger.x * mpu6500Data.ak8963ASA[0]) >> 8;
	mpu6500Data.mag.y = ((long)mpu6500Data.magInteger.y * mpu6500Data.ak8963ASA[1]) >> 8;
	mpu6500Data.mag.z = ((long)mpu6500Data.magInteger.z * mpu6500Data.ak8963ASA[2]) >> 8;
#endif
}


void imuTempControl(float expTemp){
	float tempOut;
	tempOut = pidUpdate(mpu6500Data.tempPID,expTemp,imuSensorOfChassisData.temp,0.002f);
	tempOut = constrainFloat(tempOut,0,7000);	
	TIM4->CCR1 = tempOut;
}	









