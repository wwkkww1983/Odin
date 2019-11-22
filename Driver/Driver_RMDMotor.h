#ifndef __DRIVER_RMDMOTOR_H
#define __DRIVER_RMDMOTOR_H

#include "stm32f4xx.h"
#include "Driver_RMDMotorID.h"
#include "Util.h"
/*PID类*/
#define RMDL_R_PID 				0x30	//读PID参数
#define RMDL_W_PID_RAM 			0X31	//写入PID参数到电机RAM
#define RMDL_W_PID_ROM 			0X32	//写入PID参数到电机ROM
/*加速度类*/
#define RMDL_R_ACC 				0X33	//读加速度参数
#define RMDL_W_ACC_RAM			0X34	//写入加速度参数到电机RAM
/*编码器类*/
#define RMDL_R_CODE 			0x90	//读编码器参数
#define RMDL_W_CODE_ZERO_ROM 	0x91	//写入编码器零点值到电机ROM
#define RMDL_W_ANGLE_ZERO_ROM 	0x19	//写入点击角度零点到电机ROM
/*角度类*/
#define RMDL_R_MORE_ANGLE 		0x92	//读取多圈角度
#define RMDL_R_ONE_ANGLE 		0x94	//读取单圈角度
#define RMDL_SET_FIRST_ANGLE 	0x95	//清除电机角度数据,并将当前位置设为角度0点
/*电机状态类*/
#define RMDL_R_STATE1_ERRORCMD 	0x9A	//读取电机状态1数据和错误数据
#define RMDL_CLEAR_ERRORCMD 	0x9B	//清空错误数据
#define RMDL_R_STATE2 			0x9C	//读取电机状态2
#define RMDL_R_STATE3 			0x9D	//读取电机状态3
/*控制类*/
#define RMDL_POWER_OFF  			0x80	//关闭电机电源
#define RMDL_POWER_STOP 			0x81	//电机停止
#define RMDL_POWER_ON 			0x88	//开启电机电源
/*闭环类*/
#define RMDL_IQ_CMD 				0xA1	//转矩电流控制命令
#define RMDL_RATE_CMD 			0xA2	//角速度控制命令
#define RMDL_ANGLE_CMD1 			0xA3	//角度控制命令1
#define RMDL_ANGLE_CMD2 			0xA4	//角度控制命令2
#define RMDL_ANGLE_CMD3 			0xA5	//角度控制命令3
#define RMDL_ANGLE_CMD4 			0xA6	//角度控制命令4
//旋转方向
#define TURN_UP 0x00	
#define TURN_ON 0x01	
/*RMD-L电机CAN发送结构体*/
typedef struct {
	uint8_t cmd;
	uint8_t data[7];
}RMDLCANSendStruct_t;
/*RMD-L多电机发送数据包*/
typedef struct {
	uint8_t iqc1[2];
	uint8_t iqc2[2];
	uint8_t iqc3[2];
	uint8_t iqc4[2];
}RMDLCANMultiplyStruct_t;
/*RMD-L电机PID参数结构体*/
typedef struct {
	float KP;
	float KI;
}RMDLpidUnitStruct_t;
/*RMD-L电机控制模式PID参数结构体*/
typedef struct {
	RMDLpidUnitStruct_t angle;
	RMDLpidUnitStruct_t rate;
	RMDLpidUnitStruct_t iq;
}RMDLpidUnionStuct_t;
/*RMD-L电机编码器状态结构体*/
typedef struct {
	uint16_t encoder;
	uint16_t encoderRaw;
	uint16_t encoderOffSet;
}RMDLencoderStandStruct_t;
/*RMD-L电机角度状态结构体*/
typedef struct {
	int64_t motorAngle;
	uint16_t circleAngle;
}RMDLangleForgeData;
/*RMD-L电机状态反馈结构体*/
typedef struct {
	int8_t temperature;
	uint16_t voltage;
	uint8_t errorState;
	int16_t iq;
	int16_t motorSpeed;
	uint16_t motorEncoder;
	int16_t iA,iB,iC;
}RMDLmotorStateStruct_t;
/*RMD-L电机CAN接收结构体*/
typedef struct {   
	RMDLpidUnionStuct_t pidData;
	int32_t accel;
	RMDLencoderStandStruct_t encoderData;
	RMDLangleForgeData angleData;
	RMDLmotorStateStruct_t molotovDataSheet;
	errorScanStruct_t fuze;
}RMDLCanDataRecv_t;
/*相关测试结构体声明*/
extern RMDLpidUnionStuct_t pidCommit;
extern RMDLCanDataRecv_t givemeZu;
extern RMDLCanDataRecv_t shaying;
extern RMDLCANMultiplyStruct_t wdnmd;
extern RMDLCANSendStruct_t CANSendingTo70;
extern RMDLCANSendStruct_t CANSendingTo90;
void RMDLMolotovSendData(CAN_TypeDef *CANx, uint32_t ID_CAN, RMDLCANSendStruct_t* CanSendData);
void RMDLMotorReadState(uint8_t cmd,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData);
void RMDLMotorPIDctrl(uint8_t cmd,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData);
void RMDLMotorEncoder(uint8_t cmd,uint16_t encoderOffSet,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData);
void RMDLMotorTorqueCloseLoop(uint8_t cmd,int16_t iqControl,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData);
void RMDLMotorSpeedCloseLoop(uint8_t cmd,int32_t speedControl,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData);
void RMDLMotorAngleCloseLoop1_2(uint8_t cmd,int32_t angleControl,uint16_t maxSpeed,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData);
void RMDLMotorAngleCloseLoop3_4(uint8_t cmd,uint16_t angleControl,uint16_t maxSpeed,uint8_t spinDirection,CAN_TypeDef *CANx,uint32_t ID_CAN,RMDLCANSendStruct_t* CanSendData);
void RMDLMotorReadData(CanRxMsg* CanRevData,RMDLCanDataRecv_t* Spetsnaz);
void RMDLMotorMultiplyTorque(RMDLCANMultiplyStruct_t* CanSendData,int16_t iqcRaw1,int16_t iqcRaw2,int16_t iqcRaw3,int16_t iqcRaw4);
void RMDLMotorMultiplyTorqueCloseLoop(CAN_TypeDef *CANx,RMDLCANMultiplyStruct_t* CanSendData);

#endif
