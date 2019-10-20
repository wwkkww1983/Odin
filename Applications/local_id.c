#include "local_id.h"
#include "config.h"
#include "supervisor.h"
#include "control.h"
#include "Driver_Beep.h"
#include "rc.h"
#include "cansend.h"

localIdStruct_t localIdData;
masterSlaveSendStruct_t masterData;
masterSlaveSendStruct_t slaveData;

void masterSlaveSend(CAN_TypeDef *CANx, u32 ID_CAN, masterSlaveSendStruct_t *CanData){
	u8 mbox;
  volatile u16 i = 0;
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = (uint8_t)(CanData->dataList[0].u8_temp[0]);
	txMessage.Data[1] = (uint8_t)(CanData->dataList[0].u8_temp[1]);
	txMessage.Data[2] = (uint8_t)(CanData->dataList[0].u8_temp[2]);
	txMessage.Data[3] = (uint8_t)(CanData->dataList[0].u8_temp[3]);
	
	txMessage.Data[4] = (uint8_t)(CanData->dataList[1].u8_temp[0]);
	txMessage.Data[5] = (uint8_t)(CanData->dataList[1].u8_temp[1]);
	txMessage.Data[6] = (uint8_t)(CanData->dataList[1].u8_temp[2]);
	txMessage.Data[7] = (uint8_t)(CanData->dataList[1].u8_temp[3]);
	        
  mbox= CAN_Transmit(CANx, &txMessage);
	//等待发送结束
  while(CAN_TransmitStatus(CANx, mbox) == CAN_TxStatus_Failed){
		i++;	
		if(i >= 0xFFF)
		break;
	}
}

void user_senddata(CAN_TypeDef *CANx, u32 ID_CAN, canUserData_t *userCanData){
	u8 mbox;
  volatile u16 i = 0;
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = (uint8_t)(userCanData->flag[0]<<7 | userCanData->flag[1]<<6 |
																userCanData->flag[2]<<5 | userCanData->flag[3]<<4 |
																userCanData->flag[4]<<3 | userCanData->flag[5]<<2 |
																userCanData->flag[6]<<1 | userCanData->flag[7]<<0 );
	txMessage.Data[1] = (uint8_t) userCanData->byte[0];
	txMessage.Data[2] = (uint8_t) userCanData->byte[1];
	txMessage.Data[3] = (uint8_t) userCanData->byte[2];

	txMessage.Data[4] = (uint8_t)(userCanData->halfWord[0]>>8);
	txMessage.Data[5] = (uint8_t) userCanData->halfWord[0];
	txMessage.Data[6] = (uint8_t)(userCanData->halfWord[1]>>8);
	txMessage.Data[7] = (uint8_t) userCanData->halfWord[1];
  mbox= CAN_Transmit(CANx, &txMessage);   
	
	//等待发送结束
  while(CAN_TransmitStatus(CANx, mbox)==CAN_TxStatus_Failed){
		i++;	
		if(i >= 0xFFF)
		break;
	}
}

void user_receiveData( CanRxMsg *CAN_RX_Msg, canUserData_t *canUserData ){
	canUserData -> flag[0] = (CAN_RX_Msg -> Data[0] >> 7) & 0x01;
	canUserData -> flag[1] = (CAN_RX_Msg -> Data[0] >> 6) & 0x01;
	canUserData -> flag[2] = (CAN_RX_Msg -> Data[0] >> 5) & 0x01;
	canUserData -> flag[3] = (CAN_RX_Msg -> Data[0] >> 4) & 0x01;
	canUserData -> flag[4] = (CAN_RX_Msg -> Data[0] >> 3) & 0x01;
	canUserData -> flag[5] = (CAN_RX_Msg -> Data[0] >> 2) & 0x01;
	canUserData -> flag[6] = (CAN_RX_Msg -> Data[0] >> 1) & 0x01;
	canUserData -> flag[7] = (CAN_RX_Msg -> Data[0] >> 0) & 0x01;
	canUserData -> byte[0] = CAN_RX_Msg -> Data[1];
	canUserData -> byte[1] = CAN_RX_Msg -> Data[2];
	canUserData -> byte[2] = CAN_RX_Msg -> Data[3];
	canUserData -> halfWord[0] = (uint16_t)((CAN_RX_Msg -> Data[4] << 8) | CAN_RX_Msg -> Data[5]);
	canUserData -> halfWord[1] = (uint16_t)((CAN_RX_Msg -> Data[6] << 8) | CAN_RX_Msg -> Data[7]);
}

static void identifyLocalIdBeepUpdate(uint16_t localId){
	switch(localId){
		case 0: supervisorData.beepState = ID_0x0101; break;
		case 1:	supervisorData.beepState = ID_0x0102; break;
		case 2:	supervisorData.beepState = ID_0x0103; break;
		case 3:	supervisorData.beepState = ID_0x0104; break;
		case 4: supervisorData.beepState = ID_0x0105; break;
	}
}

void identifyLocalId(void *Parameters){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1){
		vTaskDelayUntil(&xLastWakeTime,LOCAL_ID_PERIOD);				//10Hz
		if(RC_PITCH > 490){
			localIdData.distinguishLever = RC_GEAR;
			if(localIdData.distinguishLastLever != localIdData.distinguishLever){		//当摇杆不同于上次，计数一次
				digitalIncreasing(&localIdData.nowAddress);			
				if(localIdData.nowAddress > 4)											//最多5个ID可用
					localIdData.nowAddress = 0;
				identifyLocalIdBeepUpdate(localIdData.nowAddress);
			}
			localIdData.distinguishLastLever = RC_GEAR;
		}
		else if(RC_PITCH < -490){
			parameter[LOCAL_ID] = localIdData.nowAddress + 0x0101;
			localIdData.distinguishState = ID_COMPLETE_IDENTIFIED;
			currentRobotParameterConfig();
			digitalHi(&supervisorData.flashSave);									//开启储存到flash
			vTaskDelete(localIdData.xHandleTask);									//删除本任务
		}
		else{
			robotConfigData.distinguishLastLever = RC_GEAR;				//其他情况不记录
		}
		digitalIncreasing(&localIdData.loops);
	}
}

void localIdDistinguish(void){
	digitalLo(&controlData.dataInitFlag);
	localIdData.distinguishState = ID_BEING_IDENTIFIED;
	identifyLocalIdBeepUpdate(localIdData.nowAddress);
	localIdData.distinguishLastLever = RC_GEAR;								//储存一次摇杆值
	supervisorData.taskEvent[TYPE_LOCAL_ID_TASK] = xTaskCreate(identifyLocalId,"LOCAL_ID",LOCAL_ID_STACK_SIZE,NULL,LOCAL_ID_PRIORITY,&localIdData.xHandleTask);
}
