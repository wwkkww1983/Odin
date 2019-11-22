#include "Driver_RMDMotor.h"
/*PID参数初始化*/
RMDLpidUnionStuct_t pidCommit = {
	.angle.KP = 100,
	.angle.KI = 100,
	.rate.KP = 	100,
	.rate.KI = 	100,
	.iq.KP = 	100,
	.iq.KI = 	100,
};

RMDLCANSendStruct_t CANSendingTo70;//测试7015发送数据
RMDLCANSendStruct_t CANSendingTo90;
RMDLCanDataRecv_t givemeZu;//测试7015接收数据
RMDLCanDataRecv_t shaying;//测试5015接收数据
RMDLCANMultiplyStruct_t wdnmd;//测试多电机发送

/*电机CAN发送*/
void RMDLMolotovSendData(CAN_TypeDef *CANx, uint32_t ID_CAN, RMDLCANSendStruct_t* CanSendData) {
	CanTxMsg txMessage;
	uint8_t mbox;
	uint8_t count;
	uint16_t i = 0;
	txMessage.StdId = ID_CAN;
	txMessage.IDE = 0x00;
	txMessage.RTR = 0x00;
	txMessage.DLC = 0x08;

	txMessage.Data[0] = (uint8_t)CanSendData->cmd;
	for (count = 0; count < 7; count++) {
		txMessage.Data[count+1] = (uint8_t)CanSendData->data[count];
	}
	mbox = CAN_Transmit(CANx, &txMessage);
	while (CAN_TransmitStatus(CANx,mbox) == 0x00) {
		i++;
		if (i >= 0xFFF)break;
	}
}
/*读取命令发送*/
void RMDLMotorReadState(uint8_t cmd,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData) {
	uint8_t i;
	CanSendData->cmd = cmd;
	for (i = 0; i < 7; i++) {
		CanSendData->data[i] = 0x00;
	}
	RMDLMolotovSendData(CANx,ID_CAN,CanSendData);
}
/*PID类*/
void RMDLMotorPIDctrl(uint8_t cmd,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData) {
	CanSendData->cmd = cmd;
	CanSendData->data[0] = 0x00;
	CanSendData->data[1] = (uint8_t)pidCommit.angle.KI;
	CanSendData->data[2] = (uint8_t)pidCommit.rate.KP;
	CanSendData->data[3] = (uint8_t)pidCommit.rate.KI;
	CanSendData->data[4] = (uint8_t)pidCommit.iq.KP;
	CanSendData->data[5] = (uint8_t)pidCommit.iq.KI;
	CanSendData->data[6] = (uint8_t)pidCommit.angle.KP;
	RMDLMolotovSendData(CANx,ID_CAN,CanSendData);
}
/*加速度类*/
void RMDLMotorACCctrl(uint8_t cmd,int32_t accControl,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData){
	uint8_t i;
	CanSendData->cmd = cmd;
	for (i = 0; i < 7; i++) {
		CanSendData->data[i] = 0x00;
	}
	for (i = 3; i < 7; i++) {
		CanSendData->data[i] = (uint8_t)accControl & 0xFF;
		accControl = accControl >> 8;
	}
	RMDLMolotovSendData(CANx,ID_CAN,CanSendData);
}
/*编码器类*/
void RMDLMotorEncoder(uint8_t cmd,uint16_t encoderOffSet,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData) {
	uint8_t i;
	CanSendData->cmd = cmd;
	if(cmd == RMDL_W_ANGLE_ZERO_ROM) encoderOffSet = 0x00;
	for (i = 0; i < 7; i++) {
		CanSendData->data[i] = 0x00;
	}
	CanSendData->data[5] = (uint8_t)(encoderOffSet & 0xFF);
	CanSendData->data[6] = (uint8_t)((encoderOffSet >> 8) & 0xFF);
	RMDLMolotovSendData(CANx,ID_CAN,CanSendData);
}
/*闭环类*/
/*转矩闭环*/
void RMDLMotorTorqueCloseLoop(uint8_t cmd,int16_t iqControl,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData) {
	uint8_t i;
	CanSendData->cmd = cmd;
	for (i = 0; i < 7; i++) {
		CanSendData->data[i] = 0x00;
	}
	if (iqControl > 2000)iqControl = 2000;
	if (iqControl < -2000)iqControl = -2000;
	CanSendData->data[3] = (uint8_t)(iqControl & 0xFF);
	CanSendData->data[4] = (uint8_t)((iqControl >> 8) & 0xFF);
	RMDLMolotovSendData(CANx,ID_CAN,CanSendData);
}
/*角速度闭环*/
void RMDLMotorSpeedCloseLoop(uint8_t cmd,int32_t speedControl,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData) {
	uint8_t i;
	CanSendData->cmd = cmd;
	for (i = 0; i < 7; i++) {
		CanSendData->data[i] = 0x00;
	}
	for (i = 3; i < 7; i++) {
		CanSendData->data[i] = (uint8_t)speedControl & 0xFF;
		speedControl = speedControl >> 8;
	}
	RMDLMolotovSendData(CANx,ID_CAN,CanSendData);
}
/*角度闭环1和2*/
void RMDLMotorAngleCloseLoop1_2(uint8_t cmd,int32_t angleControl,uint16_t maxSpeed,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData) {
	uint8_t i;
	CanSendData->cmd = cmd;
	if (cmd != RMDL_ANGLE_CMD2)maxSpeed = 0x00;
	for (i = 0; i < 7; i++) {
		CanSendData->data[i] = 0x00;
	}
	for (i = 1; i < 3; i++) {
		CanSendData->data[i] = (uint8_t)maxSpeed & 0xFF;
		maxSpeed = maxSpeed >> 8;
	}
	for (i = 3; i < 7; i++) {
		CanSendData->data[i] = (uint8_t)angleControl & 0xFF;
		angleControl = angleControl >> 8;
	}
	RMDLMolotovSendData(CANx,ID_CAN,CanSendData);
}
/*角度闭环3和4*/
void RMDLMotorAngleCloseLoop3_4(uint8_t cmd,uint16_t angleControl,uint16_t maxSpeed,uint8_t spinDirection,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData) {
	uint8_t i;
	CanSendData->cmd = cmd;
	if (cmd != RMDL_ANGLE_CMD4)maxSpeed = 0x00;
	for (i = 0; i < 7; i++) {
		CanSendData->data[i] = 0x00;
	}
	CanSendData->data[0] = (uint8_t)spinDirection;
	for (i = 1; i < 3; i++) {
		CanSendData->data[i] = (uint8_t)maxSpeed & 0xFF;
		maxSpeed = maxSpeed >> 8;
	}
	for (i = 3; i < 5; i++) {
		CanSendData->data[i] = (uint8_t)angleControl & 0xFF;
		angleControl = angleControl >> 8;
	}
	RMDLMolotovSendData(CANx,ID_CAN,CanSendData);
}
/*读取CAN数据*/
void RMDLMotorReadData(CanRxMsg* CanRevData,RMDLCanDataRecv_t* Spetsnaz) {
	uint8_t cmd;
	uint8_t count;
	cmd = CanRevData->Data[0];
	/*命令分流*/
	switch (cmd) {
	case RMDL_R_PID: {//读取数据PID
		Spetsnaz->pidData.angle.KP = CanRevData->Data[2];
		Spetsnaz->pidData.angle.KI = CanRevData->Data[3];
		Spetsnaz->pidData.rate.KP = CanRevData->Data[4];
		Spetsnaz->pidData.rate.KI = CanRevData->Data[5];
		Spetsnaz->pidData.iq.KP = CanRevData->Data[6];
		Spetsnaz->pidData.iq.KI = CanRevData->Data[7];
	}break;
	case RMDL_R_ACC:{
		Spetsnaz->accel = (CanRevData->Data[7]<<24)|(CanRevData->Data[6]<<16)|(CanRevData->Data[5]<<8)|CanRevData->Data[4];
	}break;
	case RMDL_R_CODE: {//读取编码器数据
		Spetsnaz->encoderData.encoder = (CanRevData->Data[3] << 8) | CanRevData->Data[2];		//当前编码器值
		Spetsnaz->encoderData.encoderRaw = (CanRevData->Data[5] << 8) | CanRevData->Data[4];	//编码器真实值
		Spetsnaz->encoderData.encoderOffSet = (CanRevData->Data[7] << 8) | CanRevData->Data[6];	//编码器零偏值
	}break;
	case RMDL_R_MORE_ANGLE: {//读取多圈角度
		Spetsnaz->angleData.motorAngle = 0;
		for (count = 7; count > 0; count--) {
			Spetsnaz->angleData.motorAngle |= CanRevData->Data[count];
			Spetsnaz->angleData.motorAngle = Spetsnaz->angleData.motorAngle << 8;
		}
	}break;
	case RMDL_R_ONE_ANGLE: {//读取单圈角度
		Spetsnaz->angleData.circleAngle = (CanRevData->Data[7] << 8) | CanRevData->Data[6];
	}break;
	case RMDL_R_STATE1_ERRORCMD://获取状态1和错误标志命令
	case RMDL_CLEAR_ERRORCMD:{//清除错误标志命令
		Spetsnaz->molotovDataSheet.temperature = CanRevData->Data[1];
		Spetsnaz->molotovDataSheet.voltage = (CanRevData->Data[4] << 8) | CanRevData->Data[3];
		Spetsnaz->molotovDataSheet.errorState = CanRevData->Data[7];
	}break;
	case RMDL_R_STATE2: {//获取状态2
		Spetsnaz->molotovDataSheet.temperature = CanRevData->Data[1];
		Spetsnaz->molotovDataSheet.iq = (CanRevData->Data[3] << 8) | CanRevData->Data[2];
		Spetsnaz->molotovDataSheet.motorSpeed = (CanRevData->Data[5] << 8) | CanRevData->Data[4];
		Spetsnaz->molotovDataSheet.motorEncoder = (CanRevData->Data[7] << 8) | CanRevData->Data[6];
	}break;
	case RMDL_R_STATE3: {//获取状态3
		Spetsnaz->molotovDataSheet.temperature = CanRevData->Data[1];
		Spetsnaz->molotovDataSheet.iA = (CanRevData->Data[3] << 8) | CanRevData->Data[2];
		Spetsnaz->molotovDataSheet.iB = (CanRevData->Data[5] << 8) | CanRevData->Data[4];
		Spetsnaz->molotovDataSheet.iC = (CanRevData->Data[7] << 8) | CanRevData->Data[6];
	}break;
	case RMDL_IQ_CMD:		//转矩电流闭环控制
	case RMDL_RATE_CMD:		//角速度闭环控制
	case RMDL_ANGLE_CMD1:	//角度闭环控制1
	case RMDL_ANGLE_CMD2:	//角度闭环控制2
	case RMDL_ANGLE_CMD3:	//角度闭环控制3
	case RMDL_ANGLE_CMD4:{	//角度闭环控制4
		Spetsnaz->molotovDataSheet.temperature = CanRevData->Data[1];
		Spetsnaz->molotovDataSheet.iq = (CanRevData->Data[3] << 8) | CanRevData->Data[2];
		Spetsnaz->molotovDataSheet.motorSpeed = (CanRevData->Data[5] << 8) | CanRevData->Data[4];
		Spetsnaz->molotovDataSheet.motorEncoder = (CanRevData->Data[7] << 8) | CanRevData->Data[6];
	}break;
	default:break;
	}
}
/*多电机控制命令(仅用于ID1~4)*/
/*转矩电流数据发送转换*/
void RMDLMotorMultiplyTorque(RMDLCANMultiplyStruct_t* CanSendData,int16_t iqcRaw1,int16_t iqcRaw2,int16_t iqcRaw3,int16_t iqcRaw4) {
	CanSendData->iqc1[0] = iqcRaw1 & 0xFF;
	CanSendData->iqc1[1] = (iqcRaw1 >> 8) & 0xFF;
	CanSendData->iqc2[0] = iqcRaw2 & 0xFF;
	CanSendData->iqc2[1] = (iqcRaw2 >> 8) & 0xFF;
	CanSendData->iqc3[0] = iqcRaw3 & 0xFF;
	CanSendData->iqc3[1] = (iqcRaw3 >> 8) & 0xFF;
	CanSendData->iqc4[0] = iqcRaw4 & 0xFF;
	CanSendData->iqc4[1] = (iqcRaw4 >> 8) & 0xFF;
}
/*多电机转矩闭环控制发送*/
void RMDLMotorMultiplyTorqueCloseLoop(CAN_TypeDef *CANx,RMDLCANMultiplyStruct_t* CanSendData) {
	CanTxMsg txMessage;
	uint8_t mbox;
	uint16_t i = 0;
	txMessage.StdId = 0x280;
	txMessage.IDE = 0x00;
	txMessage.RTR = 0x00;
	txMessage.DLC = 0x08;
	txMessage.Data[0] = (uint8_t)CanSendData->iqc1[0];
	txMessage.Data[1] = (uint8_t)CanSendData->iqc1[1];
	txMessage.Data[2] = (uint8_t)CanSendData->iqc2[0];
	txMessage.Data[3] = (uint8_t)CanSendData->iqc2[1];
	txMessage.Data[4] = (uint8_t)CanSendData->iqc3[0];
	txMessage.Data[5] = (uint8_t)CanSendData->iqc3[1];
	txMessage.Data[6] = (uint8_t)CanSendData->iqc4[0];
	txMessage.Data[7] = (uint8_t)CanSendData->iqc4[1];
	mbox = CAN_Transmit(CANx,&txMessage);
	while (CAN_TransmitStatus(CANx,mbox) == 0x00) {
		i++;
		if (i >= 0xFFF)break;
	}
}
