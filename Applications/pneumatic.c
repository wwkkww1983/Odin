#include "pneumatic.h"
#include "shoot.h"

pneumatic_state_data_t pneumaticData;

void pneumaticInit(void){				//暂无
	
}

/* CAN1气动发送 */
void pneumatic_can1_sentData(u32 ID_CAN,uint8_t *pneumatic_state){
	u8 mbox;                                   
  volatile u16 i=0;
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = 0xAC;/*******校验*********/
	txMessage.Data[1] = 0xAD;/*******校验*********/
	
	txMessage.Data[2] = (uint8_t)pneumatic_state[0];
	txMessage.Data[3] = (uint8_t)pneumatic_state[1];
	txMessage.Data[4] = (uint8_t)pneumatic_state[2];
	txMessage.Data[5] = (uint8_t)pneumatic_state[3];
	txMessage.Data[6] = (uint8_t)pneumatic_state[4];
	txMessage.Data[7] = (uint8_t)pneumatic_state[5];
	
	mbox= CAN_Transmit(CAN1, &txMessage);   
	
	//等待发送结束
  while(CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)
	{
		i++;	
		if(i>=0xFFF)
		break;
	}
}

/* CAN2气动发送 */
void pneumatic_can2_sentData(u32 ID_CAN,uint8_t *pneumatic_state){
	u8 mbox;                                   
  volatile u16 i=0;                           //工程车气动发送
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = 0xAC;/*******校验*********/
	txMessage.Data[1] = 0xAD;/*******校验*********/
	
	txMessage.Data[2] = (uint8_t)pneumatic_state[0];
	txMessage.Data[3] = (uint8_t)pneumatic_state[1];
	txMessage.Data[4] = (uint8_t)pneumatic_state[2];
	txMessage.Data[5] = (uint8_t)pneumatic_state[3];
	txMessage.Data[6] = (uint8_t)pneumatic_state[4];
	txMessage.Data[7] = (uint8_t)pneumatic_state[5];
	
	mbox= CAN_Transmit(CAN2, &txMessage);   
	
	//等待发送结束
  while(CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)
	{
		i++;	
		if(i>=0xFFF)
		break;
	}
}

void pneumatic_readData(CanRxMsg *can_rx_msg){
	pneumaticData.read1[0] = can_rx_msg->Data[0];
	pneumaticData.read1[1] = can_rx_msg->Data[1];
	pneumaticData.read1[2] = can_rx_msg->Data[2];
	pneumaticData.read1[3] = can_rx_msg->Data[3];
	pneumaticData.read1[4] = can_rx_msg->Data[4];
	pneumaticData.read1[5] = can_rx_msg->Data[5];
}
