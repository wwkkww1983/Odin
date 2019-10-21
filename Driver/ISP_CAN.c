#include "board.h"
/*
***************************************************
函数名：CAN1_RX0_IRQHandler
功能：CAN1接收中断
备注：y:700 p:3180
				
***************************************************
*/
void CAN1_RX0_IRQHandler(void){
	CanRxMsg can1_rx_msg;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET){
		CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);		
		CAN_Receive(CAN1, CAN_FIFO0, &can1_rx_msg);
		switch(can1_rx_msg.StdId){ 
			case ID_RMMOTOR_RIGHT_FRONT: 
			case ID_RMMOTOR_LEFT_FRONT: 
			case ID_RMMOTOR_LEFT_BACK: 
			case ID_RMMOTOR_RIGHT_BACK:{ 
					static uint8_t i; 
					i = can1_rx_msg.StdId - ID_RMMOTOR_RIGHT_FRONT; 
					rmmotor_readdata(&can1_rx_msg,&wheelData[i]); 
					digitalIncreasing(&chassisData.wheelError[i].errorCount);
			}break; 
			
			case ID_FIRE_TNF_R_OR_YAW :
				//工程车小云台
				if(ROBOT == AUXILIARY_ID)
					gimbal_readData(&can1_rx_msg,&yawMotorData);	
				//英雄车小云台和步兵的3508摩擦轮
				if(ROBOT == INFANTRY_ID || ROBOT == SMALLGIMBAL_ID || ROBOT == UAV_ID){
					rmmotor_readdata(&can1_rx_msg,&fricWheelData[0]); 			
					digitalIncreasing(&shootData.fricWheelError[0].errorCount);
				}					
				break;
			case ID_FIRE_TNF_L_OR_PITCH : 
				//工程车小云台
				if(ROBOT == AUXILIARY_ID)
					gimbal_readData(&can1_rx_msg,&pitchMotorData);	
				//英雄车小云台和步兵的3508摩擦轮
				if(ROBOT == INFANTRY_ID || ROBOT == SMALLGIMBAL_ID || ROBOT == UAV_ID){
					rmmotor_readdata(&can1_rx_msg,&fricWheelData[1]);
					digitalIncreasing(&shootData.fricWheelError[1].errorCount);
				}					
				break;
			case ID_PITCH_INF : 					
				gimbal_readData(&can1_rx_msg,&pitchMotorData);
				digitalIncreasing(&gimbalData.gimbalError[1].errorCount); 
			break;
			case ID_YAW_INF :							
				gimbal_readData(&can1_rx_msg,&yawMotorData); 
				digitalIncreasing(&gimbalData.gimbalError[0].errorCount); 
			break;
			case ID_POKEMOTOR:
				  pokeMoterReadData(&can1_rx_msg, &pokeData );
					digitalIncreasing(&shootData.pokeError.errorCount);			
				break;
			case ID_SUPPLY:
					pokeMoterReadData(&can1_rx_msg,&lidData);
					digitalIncreasing(&shootData.lidError.errorCount);
				break;
				/*********不用检流设备，暂时屏蔽***************/
//			case ID_CURRENT:							powerRawData(&can1_rx_msg, &powerRawDate, wheelData);	
//																		digitalIncreasing(&chassisData.currentError.errorCount);		
//																																													break;
				//用该ID来接受电容电压数据
			case ID_CURRENT:							
				powerDataReceive(&can1_rx_msg.Data[0]);
				digitalIncreasing(&chassisData.currentError.errorCount);		
				break;
			default:	break;		
		}
	}
}

/*
***************************************************
函数名：CAN1_TX_IRQHandler
功能：CAN1发送中断
备注：
***************************************************
*/
void CAN1_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);

		/*********以下是自定义部分**********/
	}
}

/*
***************************************************
函数名：CAN2_RX0_IRQHandler
功能：CAN2接收中断
备注：
***************************************************
*/	
void CAN2_RX0_IRQHandler(void){
   CanRxMsg can2_rx_msg;
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET){
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_Receive(CAN2, CAN_FIFO0, &can2_rx_msg);
		  switch(can2_rx_msg.StdId){
				case ID_RMMOTOR_RIGHT_FRONT:
				case ID_RMMOTOR_LEFT_FRONT:{	
					static uint8_t i;
					i = can2_rx_msg.StdId - ID_RMMOTOR_RIGHT_FRONT;
					rmmotor_readdata(&can2_rx_msg,&fricWheelData[i]);
					break;
				}
				case ID_TURNTABLE:{
				  rmmotor_readdata(&can2_rx_msg,&turntableData);
					break;
				}
				case ID_BIGPOKE:{
				  rmmotor_readdata(&can2_rx_msg,&bigPokeData);
					break;	
				}
				case ID_PAW_L:{
					rmmotor_readdata(&can2_rx_msg,&pawMotorData[0]);
					break;
				}
				case ID_PAW_R:{
					rmmotor_readdata(&can2_rx_msg,&pawMotorData[1]);
					break;
				}
				case ID_OPTOLECTONIC_SWITCH:{
					readOpSwitch602_data(&can2_rx_msg); 
				}break;
				case ID_PNEUMATIC:{
					pneumatic_readData(&can2_rx_msg);
				}break;
				
				default:						
					break;				
		 }
	}
}

/*
***************************************************
函数名：CAN2_TX_IRQHandler
功能：CAN2发送中断
备注：
***************************************************
*/
void CAN2_TX_IRQHandler(void){
	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET){
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
		
		/*********以下是自定义部分**********/
	}
}
