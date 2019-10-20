#include "DRIVER_VL53L0X.h"
vl53l0x_raw_data_t vl53l0x_raw_data[4];
vl53l0x_real_data_t vl53l0x_real_data[4];
firstOrderFilterData_t firstOrderFilters[8];
vl53l0x_state_data_t  tofSendDate;
vl53l0x_state_data_t  tofReadDate;

void read_vl53l0x_raw_data(CanRxMsg *can_rx_msg,vl53l0x_raw_data_t *rawdata){
	rawdata->distance = ((can_rx_msg->Data[2]<<8|can_rx_msg->Data[1]));		 
}
//////////////////////////////////////////////////////////////////////////
void initFirstOrderFilter(void){
	float a;
/**********************  TOF1 *******************************/

	a = 2.0f * 0.1f * 100.0f;

	firstOrderFilters[TOF1].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[TOF1].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[TOF1].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[TOF1].previousInput  = 0.0f;
	firstOrderFilters[TOF1].previousOutput = 0.0f;
		
/**********************  TOF2 *******************************/
	a = 2.0f * 0.1f * 100.0f;

	firstOrderFilters[TOF2].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[TOF2].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[TOF2].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[TOF2].previousInput  = 0.0f;
	firstOrderFilters[TOF2].previousOutput = 0.0f;		
/**********************  TOF3 *******************************/
	a = 2.0f * 0.1f * 100.0f;

	firstOrderFilters[TOF3].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[TOF3].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[TOF3].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[TOF3].previousInput  = 0.0f;
	firstOrderFilters[TOF3].previousOutput = 0.0f;		
	
/**********************  TOF4 *******************************/
	a = 2.0f * 0.1f * 100.0f;

	firstOrderFilters[TOF4].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[TOF4].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[TOF4].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[TOF4].previousInput  = 0.0f;
	firstOrderFilters[TOF4].previousOutput = 0.0f;		
	
/**********************  TOFS1 *******************************/
	a = 2.0f * 0.03f * 100.0f;

	firstOrderFilters[TOFS1].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[TOFS1].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[TOFS1].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[TOFS1].previousInput  = 0.0f;
	firstOrderFilters[TOFS1].previousOutput = 0.0f;	

/**********************  TOFS2 *******************************/
	a = 2.0f * 0.03f * 100.0f;

	firstOrderFilters[TOFS2].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[TOFS2].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[TOFS2].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[TOFS2].previousInput  = 0.0f;
	firstOrderFilters[TOFS2].previousOutput = 0.0f;	
	
/**********************  TOFS3 *******************************/
	a = 2.0f * 0.03f * 100.0f;

	firstOrderFilters[TOFS3].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[TOFS3].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[TOFS3].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[TOFS3].previousInput  = 0.0f;
	firstOrderFilters[TOFS3].previousOutput = 0.0f;			
	
/**********************  TOFS4 *******************************/
	a = 2.0f * 0.03f * 100.0f;

	firstOrderFilters[TOFS4].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[TOFS4].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[TOFS4].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[TOFS4].previousInput  = 0.0f;
	firstOrderFilters[TOFS4].previousOutput = 0.0f;		
}

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters){
	float output;

	output = filterParameters->gx1 * input +
					 filterParameters->gx2 * filterParameters->previousInput -
					 filterParameters->gx3 * filterParameters->previousOutput;

	filterParameters->previousInput  = input;
	filterParameters->previousOutput = output;

	return output;
}

void read_vl53l0x_real_data(vl53l0x_raw_data_t *rawdata,vl53l0x_real_data_t *realdata){
  for(int i = 0; i<4; i++){
	  realdata[i].distance = firstOrderFilter(rawdata[i].distance , &firstOrderFilters[i]);
		rawdata[i].speed = realdata[i].distance - realdata[i].last_dis;
		realdata[i].speed = firstOrderFilter(rawdata[i].speed , &firstOrderFilters[i+4]);
		realdata[i].last_dis = realdata[i].distance;
	}
}

void vl5310_senddata(CAN_TypeDef *CANx, u32 ID_CAN, vl53l0x_state_data_t *vl53l0x_state){
	u8 mbox;
  u16 i=0;
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	for(uint8_t j = 0;j<8;j++){
	  txMessage.Data[j] = (uint8_t)vl53l0x_state->state[i];
	}      
  mbox= CAN_Transmit(CANx, &txMessage);   
	
	//µÈ´ı·¢ËÍ½áÊø
  while(CAN_TransmitStatus(CANx, mbox)==CAN_TxStatus_Failed){
		i++;	
		if(i>=0xFFF)
		break;
	}
}
