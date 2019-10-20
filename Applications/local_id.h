#ifndef __LOCAL_ID_H
#define __LOCAL_ID_H

#include "NavAndData.h"
#include "bsp.h"
#include "BSP_GPIO.h"
#include "Util.h"

#define LOCAL_ID_PRIORITY	12
#define LOCAL_ID_STACK_SIZE 128
#define LOCAL_ID_PERIOD	10

#define MASTER_ROBOT_MODE	masterData.dataList[0].u8_temp[0]

enum localDistinguishSate{
	ID_NO_NEED_TO_IDENTIFY = 0,
	ID_BEING_IDENTIFIED,
	ID_COMPLETE_IDENTIFIED
};

typedef struct
{
	uint8_t flag[8];
	uint8_t byte[3];
	uint16_t halfWord[2];
}canUserData_t;

typedef struct{
	TaskHandle_t xHandleTask;
	uint16_t nowAddress;
	uint8_t distinguishState;
	uint8_t distinguishLever;
	uint8_t distinguishLastLever;
	uint32_t loops;
}localIdStruct_t;

typedef struct
{
	FormatTrans dataList[2]; 
} masterSlaveSendStruct_t;

extern localIdStruct_t localIdData;
extern masterSlaveSendStruct_t masterData;
extern masterSlaveSendStruct_t slaveData;

void masterSlaveSend(CAN_TypeDef *CANx, u32 ID_CAN, masterSlaveSendStruct_t *CanData);
void masterSlaveDataUpdate(CanRxMsg *canRxMsg, masterSlaveSendStruct_t *canData);
void localIdDistinguish(void);
void user_senddata(CAN_TypeDef *CANx, u32 ID_CAN, canUserData_t *userCanData);
void user_receiveData( CanRxMsg *CAN_RX_Msg, canUserData_t *canUserData );

#endif
