#ifndef __SLAVE_SENSOR_H
#define __SLAVE_SENSOR_H

#include "Util.h"

#define IMU_MODULAR_BEGIN					0xAD
#define IMU_MODULAR_ADDRESS				0x40

#define MAIN_CONTROL_BEGIN				0x3F
#define MAIN_CONTROL_ADDRESS			0x10

#define VISION_BEGIN					0xD4
#define VISION_ADDRESS					0x20

enum {
	TRANS_ONLY_INS = 0x00,
	TRANS_ADD_ANGLE = 0x01,
	TRANS_ADD_VISION = 0x02
};

typedef struct{
	uint32_t slaveErrorCount;
	uint32_t slaveLastErrorCount;
	uint32_t intervalNum;	
	bool initFlag;
	uint8_t canForwardIndexPtr;
	
	formatTrans16Struct_t pitchCmd;
	formatTrans16Struct_t yawCmd;
	formatTrans16Struct_t pitchRecv[3];
	formatTrans16Struct_t yawRecv[3];
	
	formatTrans16Struct_t fricWheelCmd[2];
	formatTrans16Struct_t fricWheelRecv[2][3];
	
} slaveSensorStruct_t;

typedef struct{
	uint8_t seq;
	uint8_t bullet_type;
	formatTrans32Struct_t shootSpeed;
	formatTrans16Struct_t shooter_heat0;
	formatTrans16Struct_t shooter_heat1;
	formatTrans16Struct_t maxHP;
	formatTrans16Struct_t remainHP;
	formatTrans32Struct_t masterPitchMotorAngle;
	formatTrans32Struct_t masterYawMotorAngle;
	formatTrans32Struct_t masterPitchAngleRef;
	formatTrans32Struct_t masterYawAngleRef;
	uint8_t otherRcValue[18];
	uint8_t otherRcReadly;
	uint8_t otherMode;
	bool otherSameTargetFlag;
	bool otherEnemyType;
	bool otherfricMotorFlag;
	bool otherAutoMaticFlag;
	bool otherPatrolMode;
} controlTransStruct_t;

extern slaveSensorStruct_t slaveSensorData;
extern controlTransStruct_t controlTransData;

void adis16470DataUpdate(void);
void slaveSensorRead(u8 *arraySlaveSensor);
void moduleCommandUpload(USART_TypeDef *USARTx);
void otherControlRead(u8 *arrayOtherControl);
void controlDataUpload(void);
void slaveSensorConfig(void);

#endif
