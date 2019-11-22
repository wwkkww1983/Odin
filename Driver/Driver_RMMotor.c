#include "Driver_RMMotor.h"

#define CHASSIS_EC60

/*******************************外部使用的变量*********************************/
motorCanDataRecv_t	 fricWheelData[2];											//英雄摩擦轮
motorCanDataRecv_t	 wheelData[4];													//底盘电机
motorCanDataRecv_t	 pokeData;															//拨弹
motorCanDataRecv_t	 lidData;																//补给
motorCanDataRecv_t	 turntableData;  											  //英雄拨弹转盘
motorCanDataRecv_t	 bigPokeData;     										  //大发射机构拨弹
rmmotorCanDataRecv_t pitchMotorData,yawMotorData; 
currentCanDataRecv_t currentDate;
motorMeasureData_t   motorViewPitData,motorViewYawData;
motorMeasureData_t   motorGM3510PitData;

/****************************机器人类型电机变量*********************************/
motorCanDataRecv_t	 pawMotorData[2];												//工程爪子
motorCanDataRecv_t	 deformMotorData[2];										//工程横向变形机构
motorCanDataRecv_t	 liftMotorData[2];											//工程抬升机构
rmmotorTarData_t 	 pawMotorTarData[2];
/*****************************************************************************/                 

/****************************本文件使用的静态变量******************************/
static BSP_CAN_TypeDef can1;
static BSP_CAN_TypeDef can2;
/*****************************************************************************/
/*
***************************************************
函数名：driver_can1_init
功能：RM电机CAN1初始化
入口参数：	rm_canx：使用的CAN通道
					rm_canx_rx：RM CAN接收引脚
					rm_canx_tx：RM CAN发送引脚
					Preemption：CAN中断抢占优先级
					Sub：CAN中断次优先级
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void driver_can1_init(CAN_TypeDef* rm_canx,BSP_GPIOSource_TypeDef *rm_canx_rx,BSP_GPIOSource_TypeDef *rm_canx_tx,u8 Preemption,u8 Sub){
	can1.CANx = rm_canx;
	can1.CANx_RX = rm_canx_rx;
	can1.CANx_TX = rm_canx_tx;
	if(rm_canx == CAN1){
		can1.CAN_FilterInitStructure = CAN1_FilterInitStructure;
	}
	else if(rm_canx == CAN2){
		can1.CAN_FilterInitStructure = CAN2_FilterInitStructure;
	}
	BSP_CAN_Mode_Init(&can1,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal,Preemption,Sub);
} 
/*
***************************************************
函数名：driver_can1_init
功能：RM电机CAN2初始化
入口参数：	rm_canx：使用的CAN通道
					rm_canx_rx：RM CAN接收引脚
					rm_canx_tx：RM CAN发送引脚
					Preemption：CAN中断抢占优先级
					Sub：CAN中断次优先级
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void driver_can2_init(CAN_TypeDef* rm_canx,BSP_GPIOSource_TypeDef *rm_canx_rx,BSP_GPIOSource_TypeDef *rm_canx_tx,u8 Preemption,u8 Sub){
	can2.CANx = rm_canx;
	can2.CANx_RX = rm_canx_rx;
	can2.CANx_TX = rm_canx_tx;
	if(rm_canx == CAN1){
		can2.CAN_FilterInitStructure = CAN1_FilterInitStructure;
	}
	else if(rm_canx == CAN2){
		can2.CAN_FilterInitStructure = CAN2_FilterInitStructure;
	}
	BSP_CAN_Mode_Init(&can2,CAN_SJW_1tq,CAN_BS2_5tq,CAN_BS1_9tq,3,CAN_Mode_Normal,Preemption,Sub);
}
/*
***************************************************
函数名：driver_gimbal_readdata
功能：读取云台电机数据
入口参数：	can_rx_msg：CAN接收数据指针变量
					gimbal_data：存放云台电机接收到数据的指针变量
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void gimbal_readdata(CanRxMsg *can_rx_msg,gimbalCanDataRecv_t *gimbal_data){
	gimbal_data->encoderAngle = ((uint16_t)can_rx_msg->Data[0] << 8) | can_rx_msg->Data[1];
	gimbal_data->realcurrent 	= (( int16_t)can_rx_msg->Data[2] << 8) | can_rx_msg->Data[3];
	gimbal_data->current = (int16_t)(can_rx_msg->Data[4] << 8) | can_rx_msg->Data[5];
}
/*
***************************************************
函数名：chassis_motor_readdata
功能：读取地盘电机数据
入口参数：	can_rx_msg：CAN接收数据指针变量
					wheelData：存放地盘电机接收到数据的指针变量
返回值：无
应用范围：外部调用
备注：
***************************************************
*/ 
void rmmotor_readdata(CanRxMsg *can_rx_msg,motorCanDataRecv_t *motorData){
	motorData->rawangle = ((uint16_t)can_rx_msg->Data[0] << 8) | can_rx_msg->Data[1];
	motorData->speed 	 = (( int16_t)can_rx_msg->Data[2] << 8) | can_rx_msg->Data[3];
	motorData->currunt  = (( int16_t)can_rx_msg->Data[4] << 8) | can_rx_msg->Data[5];
	motorData->temperature = ( int8_t)can_rx_msg->Data[6];
}

void gimbal_motor6020_readdata(CanRxMsg *can_rx_msg,motorCanDataRecv_t *motor6020Data){
	motor6020Data->rawangle = ((uint16_t)can_rx_msg->Data[0] << 8) | can_rx_msg->Data[1];
	motor6020Data->speed 	 = (( int16_t)can_rx_msg->Data[2] << 8) | can_rx_msg->Data[3];
	motor6020Data->currunt  = (( int16_t)can_rx_msg->Data[4] << 8) | can_rx_msg->Data[5];
	motor6020Data->temperature = ( int8_t)can_rx_msg->Data[6];
}

void rmmotor_senddata(CAN_TypeDef *CANx, u32 ID_CAN, motorSerialNumber_t *rmmotorCanData){
	u8 mbox;
  u16 i=0;
	
	CanTxMsg txMessage;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	txMessage.Data[0] = (uint8_t)(rmmotorCanData->currunt1>>8);
	txMessage.Data[1] = (uint8_t) rmmotorCanData->currunt1;
	txMessage.Data[2] = (uint8_t)(rmmotorCanData->currunt2>>8);
	txMessage.Data[3] = (uint8_t) rmmotorCanData->currunt2;
	txMessage.Data[4] = (uint8_t)(rmmotorCanData->currunt3>>8);
	txMessage.Data[5] = (uint8_t) rmmotorCanData->currunt3;
	txMessage.Data[6] = (uint8_t)(rmmotorCanData->currunt4>>8);
	txMessage.Data[7] = (uint8_t) rmmotorCanData->currunt4;
	
	        
  mbox= CAN_Transmit(CANx, &txMessage);   
	
	//等待发送结束
  while(CAN_TransmitStatus(CANx, mbox)==CAN_TxStatus_Failed){
		i++;	
		if(i>=0xFFF)
		break;
	}
}

int16_t getRelativePos(int16_t rawEcd, int16_t centerOffset,rmmotorCanDataRecv_t *gimbalMotor){
  int16_t tmp = 0;
  if(gimbalMotor->motorID < MOTOR_RL7015){
	  if (centerOffset >= 4096){
		if (rawEcd > centerOffset - 4096)
		  tmp = rawEcd - centerOffset;
		else
		  tmp = rawEcd + 8192 - centerOffset;
	  }
	  else{
		if (rawEcd > centerOffset + 4096)
		  tmp = rawEcd - 8192 - centerOffset;
		else
		  tmp = rawEcd - centerOffset;
	  }
  }
  else{
	  if (centerOffset >= 8192){
		if (rawEcd > centerOffset - 8192)
		  tmp = rawEcd - centerOffset;
		else
		  tmp = rawEcd + 16384 - centerOffset;
	  }
	  else{
		if (rawEcd > centerOffset + 8192)
		  tmp = rawEcd - 16384 - centerOffset;
		else
		  tmp = rawEcd - centerOffset;
	  }
  }	  
  return tmp;
}

/*
***************************************************
函数名：driver_rm6623_calibration
功能：校准RM6623电机
入口参数：	version：电机版本
             canx：can通道
#include <stm32f4xx.h>
						VERSION_RM2016：2016年的RM6623电机
						VERSION_RM2017：2017年的RM6623电机
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void driver_rm6623_calibration(CAN_TypeDef *canx, unsigned char version){ //没用到
	u8 mbox;
  u16 i=0;
	CanTxMsg txMessage;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	if(version == VERSION_RM2016){
		txMessage.StdId = 0x1FF;
		txMessage.Data[0] = 0;
		txMessage.Data[1] = 0;
		txMessage.Data[2] = 0;
		txMessage.Data[3] = 0;
		txMessage.Data[4] = 0;
		txMessage.Data[5] = 0;
		txMessage.Data[6] = 0x04;
		txMessage.Data[7] = 0;
	}
	else if(version == VERSION_RM2017){
		txMessage.StdId = 0x3F0;
		txMessage.Data[0] = 'c';
		txMessage.Data[1] = 0;
		txMessage.Data[2] = 0;
		txMessage.Data[3] = 0;
		txMessage.Data[4] = 0;
		txMessage.Data[5] = 0;
		txMessage.Data[6] = 0;
		txMessage.Data[7] = 0;
	}
	
	mbox= CAN_Transmit(canx, &txMessage);   
	
	//等待发送结束
  while(CAN_TransmitStatus(canx, mbox)==CAN_TxStatus_Failed){
		i++;	
		if(i>=0xFFF)
			break;
	}
}
/*
***************************************************
函数名：mecanumCalculate
功能：麦克纳姆轮速度计算
入口参数：	V_X：左右速度（右为正）
					V_Y：前后速度（前为正）
					V_Rotate：旋转速度（顺时针为正）
					MaxWheelSpeed：最大轮子速度限制（绝对值）
					Wheel_Speed：度指针变量（逆时针为正）

返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void mecanumCalculate(float V_X,float V_Y,float V_Rotate,float MaxWheelSpeed,float* Wheel_Speed){
	float Buffer[4], Param, MaxSpeed;
	unsigned char index;
	Buffer[0] = V_Rotate + V_X - V_Y ;
	Buffer[1] = V_Rotate + V_X + V_Y ;
	Buffer[2] = V_Rotate - V_X + V_Y ;
	Buffer[3] = V_Rotate - V_X - V_Y ;

	//限幅
	for(index = 0, MaxSpeed = 0; index < 4; index++){
		if((Buffer[index] > 0 ? Buffer[index] : -Buffer[index]) > MaxSpeed){
			MaxSpeed = (Buffer[index] > 0 ? Buffer[index] : -Buffer[index]);
		}
	}
	if(MaxWheelSpeed < MaxSpeed){
		Param = MaxWheelSpeed / MaxSpeed;
		Wheel_Speed[0] = Buffer[0] * Param;
		Wheel_Speed[1] = Buffer[1] * Param;
		Wheel_Speed[2] = Buffer[2] * Param;
		Wheel_Speed[3] = Buffer[3] * Param; 
	}
	else{
		Wheel_Speed[0] = Buffer[0];
		Wheel_Speed[1] = Buffer[1];
		Wheel_Speed[2] = Buffer[2];
		Wheel_Speed[3] = Buffer[3];
	}
}
/*
***************************************************
函数名：gimbal_readData
功能：读取云台电机数据
入口参数：	can_rx_msg：CAN接收数据指针变量
					gimbal_data：存放云台电机接收到数据的指针变量
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void gimbal_readData(CanRxMsg *can_rx_msg,rmmotorCanDataRecv_t *gimbal_data){                                       
	switch(gimbal_data -> motorID){
		case MOTOR_6623: {
			gimbal_data -> M6623Data . encoderAngle = ((uint16_t)can_rx_msg->Data[0] << 8) | can_rx_msg->Data[1];
			gimbal_data -> M6623Data . realcurrent = (( int16_t)can_rx_msg->Data[2] << 8) | can_rx_msg->Data[3];
			gimbal_data -> M6623Data . current = (int16_t)(can_rx_msg->Data[4] << 8) | can_rx_msg->Data[5];
		}break;
		case MOTOR_6020: {
			gimbal_data -> GM6020Data . rawangle = ((uint16_t)can_rx_msg->Data[0] << 8) | can_rx_msg->Data[1];
			gimbal_data -> GM6020Data . speed 	 = (( int16_t)can_rx_msg->Data[2] << 8) | can_rx_msg->Data[3];
			gimbal_data -> GM6020Data . currunt  = (( int16_t)can_rx_msg->Data[4] << 8) | can_rx_msg->Data[5];
			gimbal_data -> GM6020Data . temperature = ( int8_t)can_rx_msg->Data[6];
		}break;
		case MOTOR_3510: {
			gimbal_data -> GM3510Data . encoderAngle = ((uint16_t)can_rx_msg->Data[0] << 8) | can_rx_msg->Data[1];
			gimbal_data -> GM3510Data . realcurrent 	= (( int16_t)can_rx_msg->Data[2] << 8) | can_rx_msg->Data[3];
			gimbal_data -> GM3510Data . current = (int16_t)(can_rx_msg->Data[4] << 8) | can_rx_msg->Data[5];
		}break;
		case MOTOR_RL7015:{
			gimbal_data->RMD_L_7015Data.molotovDataSheet.motorEncoder = ((uint16_t)can_rx_msg->Data[7] << 8) | can_rx_msg->Data[6];
			gimbal_data->RMD_L_7015Data.molotovDataSheet.motorSpeed   = ((int16_t)can_rx_msg->Data[5] << 8) | can_rx_msg->Data[4];
			gimbal_data->RMD_L_7015Data.molotovDataSheet.temperature  = (int8_t)can_rx_msg->Data[1];
			gimbal_data->RMD_L_7015Data.molotovDataSheet.iq			  = ((int16_t)can_rx_msg->Data[3] << 8) | can_rx_msg->Data[2];
		}break;
		case MOTOR_RL9015:{
			gimbal_data->RMD_L_9015Data.molotovDataSheet.motorEncoder = ((uint16_t)can_rx_msg->Data[7] << 8) | can_rx_msg->Data[6];
			gimbal_data->RMD_L_9015Data.molotovDataSheet.motorSpeed   = ((int16_t)can_rx_msg->Data[5] << 8) | can_rx_msg->Data[4];
			gimbal_data->RMD_L_9015Data.molotovDataSheet.temperature  = (int8_t)can_rx_msg->Data[1];
			gimbal_data->RMD_L_9015Data.molotovDataSheet.iq			  = ((int16_t)can_rx_msg->Data[3] << 8) | can_rx_msg->Data[2];
		}break;
	  default: break;
	}

}

/*
***************************************************
函数名：gimbal_readSlaveData
功能：向云台电机数据结构体中写入数据
入口参数：	dataNumber : 数据对应的序号（具体查看头文件）
          writeData：要写入的数据
					gimbal_data：存放云台电机接收到数据的指针变量
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void gimbal_readSlaveData(uint8_t dataNumber,vs16 writeData,rmmotorCanDataRecv_t *gimbal_data){                                       
	switch(gimbal_data -> motorID){
		case MOTOR_6623: {
			switch(dataNumber){
				case CODEBOARD_VALUE:    gimbal_data -> M6623Data . encoderAngle = writeData;break;  
				case REALTORQUE_CURRENT: gimbal_data -> M6623Data . realcurrent = writeData;break;
				case TORQUE_CURRENT:     gimbal_data -> M6623Data . current = writeData;break;
			}
		}break;
		case MOTOR_6020: {
			switch(dataNumber){
				case CODEBOARD_VALUE:    gimbal_data -> GM6020Data . rawangle = writeData;break;
				case REALTORQUE_CURRENT: gimbal_data -> GM6020Data . currunt = writeData;break;
				case TURN_SPEED:         gimbal_data -> GM6020Data . speed = writeData;break;
				case TEMPER:             gimbal_data -> GM6020Data . temperature = writeData;break;
			}
		}break;
		case MOTOR_3510: {
			switch(dataNumber){
				case CODEBOARD_VALUE:    gimbal_data -> GM3510Data . encoderAngle = writeData;break;
				case REALTORQUE_CURRENT: gimbal_data -> GM3510Data . realcurrent = writeData;break;
				case TORQUE_CURRENT:     gimbal_data -> GM3510Data . current = writeData;break;
			}
		}break;
		case MOTOR_9015: {
			switch(dataNumber){
				case CODEBOARD_VALUE:    gimbal_data -> DM9015Data . encoderAngle = writeData;break;
			}
		}break;
		case MOTOR_RL7015:{
			switch(dataNumber){
				case CODEBOARD_VALUE: gimbal_data->RMD_L_7015Data.molotovDataSheet.motorEncoder = writeData;break;
				case TORQUE_CURRENT: gimbal_data->RMD_L_7015Data.molotovDataSheet.iq = writeData;break;
			}
		}break;
		case MOTOR_RL9015:{
			switch(dataNumber){
				case CODEBOARD_VALUE: gimbal_data->RMD_L_9015Data.molotovDataSheet.motorEncoder = writeData;break;
				case TORQUE_CURRENT: gimbal_data->RMD_L_9015Data.molotovDataSheet.iq = writeData;break;	
			}
		}break;			
	  default: break;
	}

}

/*
***************************************************
gimbal_chooseData
功能：云台电机数据选择
入口参数：	dataNumber : 数据对应的序号（具体查看头文件）
					gimbal_data：存放云台电机接收到数据的指针变量
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
vs16 gimbal_chooseData(uint8_t dataNumber,rmmotorCanDataRecv_t *gimbal_data){                                       
	vs16 dataChoose;
	switch(gimbal_data -> motorID){
		case MOTOR_6623: {
			switch(dataNumber){
				case CODEBOARD_VALUE:    dataChoose = gimbal_data -> M6623Data . encoderAngle;break;  
				case REALTORQUE_CURRENT: dataChoose = gimbal_data -> M6623Data . realcurrent;break;
				case TORQUE_CURRENT:     dataChoose = gimbal_data -> M6623Data . current;break;
			}
		}break;
		case MOTOR_6020: {
			switch(dataNumber){
				case CODEBOARD_VALUE:    dataChoose = gimbal_data -> GM6020Data . rawangle;break;
				case REALTORQUE_CURRENT: dataChoose = gimbal_data -> GM6020Data . currunt;break;
				case TURN_SPEED:         dataChoose = gimbal_data -> GM6020Data . speed;break;
				case TEMPER:             dataChoose = gimbal_data -> GM6020Data . temperature;break;
			}
	}break;
		case MOTOR_3510: {
			switch(dataNumber){
				case CODEBOARD_VALUE:    dataChoose = gimbal_data -> GM3510Data . encoderAngle;break;
				case REALTORQUE_CURRENT: dataChoose = gimbal_data -> GM3510Data . realcurrent;break;
				case TORQUE_CURRENT:     dataChoose = gimbal_data -> GM3510Data . current;break;
			}
	}break;
		case MOTOR_9015: {
			switch(dataNumber){
				case CODEBOARD_VALUE:   dataChoose = gimbal_data -> DM9015Data . encoderAngle;break;
			}
	}break;
		case MOTOR_RL7015:{
			switch(dataNumber){
				case CODEBOARD_VALUE: dataChoose = gimbal_data->RMD_L_7015Data.molotovDataSheet.motorEncoder;break;
				case TORQUE_CURRENT: dataChoose = gimbal_data->RMD_L_7015Data.molotovDataSheet.iq;break;
			}
		}break;
		case MOTOR_RL9015:{
			switch(dataNumber){
				case CODEBOARD_VALUE: dataChoose = gimbal_data->RMD_L_9015Data.molotovDataSheet.motorEncoder;break;
				case TORQUE_CURRENT: dataChoose = gimbal_data->RMD_L_9015Data.molotovDataSheet.iq;break;	
			}
		}break;
	  default: break;
	}
    return  dataChoose;
}











