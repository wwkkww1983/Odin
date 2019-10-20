#include "config.h"
#include "cansend.h"
#include "control.h"
#include "deforming.h"
#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"
#include "supply.h"
#include "supervisor.h"
#include "type_robot.h"
#include "DRIVER_VL53L0X.h"
#include "Driver_DMMotor.h"
#include "pneumatic.h"
#include "judge.h"
#include "util.h"

canSendStruct_t canSendData;

void canSendChassisUpdate(void){
	if(robotConfigData.robotDeviceList & DEVICE_CHASSIS){														 		  //如果有底盘，则使用CAN1的ID1到ID4进行发送
		canSendData.can1_0x200.currunt1 = (short)chassisData.powerCurrent[R_F_WHEEL];			  //右前轮，CAN1_01
		canSendData.can1_0x200.currunt2 = (short)chassisData.powerCurrent[L_F_WHEEL];			  //左前轮，CAN1_02
		canSendData.can1_0x200.currunt3 = (short)chassisData.powerCurrent[L_B_WHEEL];			  //左后轮，CAN1_03
		canSendData.can1_0x200.currunt4 = (short)chassisData.powerCurrent[R_B_WHEEL];		  	//右后轮，CAN1_04
	}
}

void canSendGimbalUpdate(void){
	if(robotConfigData.robotDeviceList & DEVICE_GIMBAL){														 		  //如果有云台，则使用CAN1的ID5到ID6进行发送
		switch(ROBOT){
			case INFANTRY_ID:
			case TANK_ID:{
				canSendData.can1_0x2FF.currunt2 = (short)gimbalData.pitchSpeedOut;
				break;
			}	
			case SENTRY_ID:
			case SMALLGIMBAL_ID:
			case UAV_ID:{
				canSendData.can1_0x2FF.currunt1 = (short)gimbalData.yawSpeedOut;
				canSendData.can1_0x2FF.currunt2 = (short)gimbalData.pitchSpeedOut;		     
				break;
			}
			case AUXILIARY_ID:{
				canSendData.can1_0x1FF.currunt1 = (short)gimbalData.yawSpeedOut;
				canSendData.can1_0x1FF.currunt2 = (short)gimbalData.pitchSpeedOut;		     
				break;
			}
		}
	}
}

void canSendShootUpdate(void){
	switch((uint8_t)parameter[WEAPON_TYPE]){
		case NO_WEAPONS: break;
		case SMALL_LAUNCHER:{																													     	//小发射机构，则使用CAN1的ID7到ID8进行发送
			canSendData.can1_0x1FF.currunt1 = (short)shootData.fricWheelSpeedOut[0];					//17mm摩擦轮,   	CAN1_05
			canSendData.can1_0x1FF.currunt2 = (short)shootData.fricWheelSpeedOut[1];					//17mm摩擦轮,	  CAN1_06
			canSendData.can1_0x1FF.currunt3 = (short)shootData.smallPokeSpeedOut;							//17mm拨弹电机， CAN1_07
			canSendData.can1_0x1FF.currunt4 = (short)supplyData.supplySpeedOut;								//17mm弹仓电机， CAN1_08
			break;
		}
		case BIG_LAUNCHER:{																															    //大发射机构
			canSendData.can2_0x200.currunt3 = (short)shootData.turntableOut;									//筛弹电机，CAN2_03
			canSendData.can2_0x200.currunt4 = (short)shootData.bigPokeSpeedOut;								//42mm拨弹电机，CAN2_04
			break;
		}
		case DOUBLE_LAUNCHER:{																													    //双发射机构
			if(parameter[ROBOT_TYPE] == SMALLGIMBAL_ID){
				canSendData.can1_0x1FF.currunt1 = (short)shootData.fricWheelSpeedOut[0];					//17mm摩擦轮,   CAN1_05
				canSendData.can1_0x1FF.currunt2 = (short)shootData.fricWheelSpeedOut[1];					//17mm摩擦轮,	 CAN1_06
				canSendData.can1_0x1FF.currunt3 = (short)shootData.smallPokeSpeedOut;							//17mm拨弹电机，CAN1_07
				canSendData.can1_0x1FF.currunt4 = (short)supplyData.supplySpeedOut;								//17mm弹仓电机，CAN1_08
			}
			else if(parameter[ROBOT_TYPE] == TANK_ID){
				canSendData.can2_0x200.currunt3 = (short)shootData.turntableOut;									//筛弹电机，CAN2_03
				canSendData.can2_0x200.currunt4 = (short)shootData.bigPokeSpeedOut;								//42mm拨弹电机，CAN2_04
			}
			break;
		}
	}
}

void canSendDeformingUpdate(void){
	if(robotConfigData.robotDeviceList & DEVICE_DEFORMING){													  		//判断是否有变形机构
		switch(robotConfigData.typeOfRobot){
			case TANK_ID:{																																		//英雄车		
				break;
			}
			case AUXILIARY_ID:{																														    //工程车
				
				break;
			}
		}
	}
}
void canSendTofUpdate(void){
	for(uint8_t i;i<8;i++){
		canSendData.can1_0x501.state[i] = tofSendDate.state[i];
	}
}

void canSendRelax(void){
	memset((void *)&canSendData.can1_0x200, 0, sizeof(canSendData.can1_0x200));
	memset((void *)&canSendData.can2_0x200, 0, sizeof(canSendData.can2_0x200));
	memset((void *)&canSendData.can1_0x1FF, 0, sizeof(canSendData.can1_0x1FF));
	memset((void *)&canSendData.can2_0x1FF, 0, sizeof(canSendData.can2_0x1FF));
}

void currentSendData(CAN_TypeDef *CANx, u32 ID_CAN, float current){
	u8 mbox;
  u16 i=0;
	static u8 Array[4] = {0};
	FormatTrans dataTrans;
	CanTxMsg txMessage;
	
	txMessage.StdId = ID_CAN;
	txMessage.IDE = CAN_Id_Standard;
	txMessage.RTR = CAN_RTR_Data;
	txMessage.DLC = 0x08;
	
	dataTrans.float_temp = current;
	for(i=0;i<4;i++){
		Array[i] = dataTrans.u8_temp[i];
	}
	txMessage.Data[0] = (uint8_t)0xAA;
	txMessage.Data[1] = (uint8_t)Array[0];
	txMessage.Data[2] = (uint8_t)Array[1];
	txMessage.Data[3] = (uint8_t)Array[2];
	txMessage.Data[4] = (uint8_t)Array[3];
	txMessage.Data[5] = (uint8_t)0x00;
	txMessage.Data[6] = (uint8_t)0x00;
	txMessage.Data[7] = (uint8_t)0xAA;     
  mbox= CAN_Transmit(CANx, &txMessage);   
	
	//等待发送结束
  while(CAN_TransmitStatus(CANx, mbox)==CAN_TxStatus_Failed){
		i++;	
		if(i>=0xFFF)
		break;
	}
}
void canSendUpToRobot(){
	switch(ROBOT){
		case NO_ID:{	
			break;
		}
		case INFANTRY_ID:{
			rs485SendGimbalUpdata(YAW);
			break;
		}
		case TANK_ID:{
			if(parameter[ROBOT_TYPE] != SMALLGIMBAL_ID){
				if(!((canSendData.loops + 1) % 2)){
					rs485SendGimbalUpdata(YAW);
				}
				else{
					pneumatic_can2_sentData(0x401,pneumaticData.send4);
				}
				if(!((canSendData.loops + 2) % 2)){
					rs485SendGimbalUpdata(PITCH);
				}
			}
			break;
		}
		case AUXILIARY_ID:{
			rmmotor_senddata(CAN1, 0x1FF,&canSendData.can1_0x1FF);					//除了工程外其余带枪管的云台ID全用0x2FF
			if((canSendData.loops + 1) % 2){
				pneumatic_can2_sentData(0x401,pneumaticData.send1);					 //工程车气动发送   
				pneumatic_can2_sentData(0x403,pneumaticData.send2);
				pneumatic_can1_sentData(0x405,pneumaticData.send3);		
			}
			break;
		}
		case SENTRY_ID:{
			break;
		}
		case UAV_ID:{
			break;
		}
		case SMALLGIMBAL_ID:{
			break;
		}
	}
	
	
	rmmotor_senddata(CAN1, 0x200,&canSendData.can1_0x200);	              								//地盘
	if(ROBOT != AUXILIARY_ID)
		rmmotor_senddata(CAN1, 0x2FF,&canSendData.can1_0x2FF);
	if(parameter[ROBOT_TYPE] != SMALLGIMBAL_ID)
		rmmotor_senddata(CAN2, 0x1FF,&canSendData.can2_0x1FF);		
	if(!((canSendData.loops + 1) % 2)){																										//250Hz  轮循发送　
		rmmotor_senddata(CAN1, 0x1FF,&canSendData.can1_0x1FF);															//步兵弹仓盖、拨弹盘、17mm摩擦轮、工程抓取机构 
		if((parameter[ROBOT_TYPE] != SMALLGIMBAL_ID))																				//主控才发can2电机数据，不然会副控can2数据跟主主控数据冲突
			rmmotor_senddata(CAN2, 0x200,&canSendData.can2_0x200);														//（英雄）42mm摩擦轮 筛弹电机 拨弹电机
	}
	
	
}
void canSendUpdate(void){
	canSendChassisUpdate();																																//底盘CAN发送更新
	canSendGimbalUpdate();																																//云台CAN发送更新
	canSendShootUpdate();																																	//发射机构CAN发送更新
	canSendDeformingUpdate();																															//变形机构CAN发送更新
	canSendTofUpdate();                                                                   //RM2018激光测距模块  没有 暂时屏蔽
	canSendUpToRobot();																																		//不同兵种can发送函数
	if(robotMode == MODE_RELAX)																														//处于RELAX模式，输出全清零	
		canSendRelax();
	digitalIncreasing(&canSendData.loops);
}

void canSendInit(void){								//CAN发送初始化
  driver_can1_init(CAN1,BSP_GPIOD0,BSP_GPIOD1,4,0);
	driver_can2_init(CAN2,BSP_GPIOB12,BSP_GPIOB13,3,0);
	rs485DeviceInit();
}
