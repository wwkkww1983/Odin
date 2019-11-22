#ifndef __DRIVER_RMDMOTOR_H
#define __DRIVER_RMDMOTOR_H

#include "stm32f4xx.h"
#include "Driver_RMDMotorID.h"
#include "Util.h"
/*PID��*/
#define RMDL_R_PID 				0x30	//��PID����
#define RMDL_W_PID_RAM 			0X31	//д��PID���������RAM
#define RMDL_W_PID_ROM 			0X32	//д��PID���������ROM
/*���ٶ���*/
#define RMDL_R_ACC 				0X33	//�����ٶȲ���
#define RMDL_W_ACC_RAM			0X34	//д����ٶȲ��������RAM
/*��������*/
#define RMDL_R_CODE 			0x90	//������������
#define RMDL_W_CODE_ZERO_ROM 	0x91	//д����������ֵ�����ROM
#define RMDL_W_ANGLE_ZERO_ROM 	0x19	//д�����Ƕ���㵽���ROM
/*�Ƕ���*/
#define RMDL_R_MORE_ANGLE 		0x92	//��ȡ��Ȧ�Ƕ�
#define RMDL_R_ONE_ANGLE 		0x94	//��ȡ��Ȧ�Ƕ�
#define RMDL_SET_FIRST_ANGLE 	0x95	//�������Ƕ�����,������ǰλ����Ϊ�Ƕ�0��
/*���״̬��*/
#define RMDL_R_STATE1_ERRORCMD 	0x9A	//��ȡ���״̬1���ݺʹ�������
#define RMDL_CLEAR_ERRORCMD 	0x9B	//��մ�������
#define RMDL_R_STATE2 			0x9C	//��ȡ���״̬2
#define RMDL_R_STATE3 			0x9D	//��ȡ���״̬3
/*������*/
#define RMDL_POWER_OFF  			0x80	//�رյ����Դ
#define RMDL_POWER_STOP 			0x81	//���ֹͣ
#define RMDL_POWER_ON 			0x88	//���������Դ
/*�ջ���*/
#define RMDL_IQ_CMD 				0xA1	//ת�ص�����������
#define RMDL_RATE_CMD 			0xA2	//���ٶȿ�������
#define RMDL_ANGLE_CMD1 			0xA3	//�Ƕȿ�������1
#define RMDL_ANGLE_CMD2 			0xA4	//�Ƕȿ�������2
#define RMDL_ANGLE_CMD3 			0xA5	//�Ƕȿ�������3
#define RMDL_ANGLE_CMD4 			0xA6	//�Ƕȿ�������4
//��ת����
#define TURN_UP 0x00	
#define TURN_ON 0x01	
/*RMD-L���CAN���ͽṹ��*/
typedef struct {
	uint8_t cmd;
	uint8_t data[7];
}RMDLCANSendStruct_t;
/*RMD-L�����������ݰ�*/
typedef struct {
	uint8_t iqc1[2];
	uint8_t iqc2[2];
	uint8_t iqc3[2];
	uint8_t iqc4[2];
}RMDLCANMultiplyStruct_t;
/*RMD-L���PID�����ṹ��*/
typedef struct {
	float KP;
	float KI;
}RMDLpidUnitStruct_t;
/*RMD-L�������ģʽPID�����ṹ��*/
typedef struct {
	RMDLpidUnitStruct_t angle;
	RMDLpidUnitStruct_t rate;
	RMDLpidUnitStruct_t iq;
}RMDLpidUnionStuct_t;
/*RMD-L���������״̬�ṹ��*/
typedef struct {
	uint16_t encoder;
	uint16_t encoderRaw;
	uint16_t encoderOffSet;
}RMDLencoderStandStruct_t;
/*RMD-L����Ƕ�״̬�ṹ��*/
typedef struct {
	int64_t motorAngle;
	uint16_t circleAngle;
}RMDLangleForgeData;
/*RMD-L���״̬�����ṹ��*/
typedef struct {
	int8_t temperature;
	uint16_t voltage;
	uint8_t errorState;
	int16_t iq;
	int16_t motorSpeed;
	uint16_t motorEncoder;
	int16_t iA,iB,iC;
}RMDLmotorStateStruct_t;
/*RMD-L���CAN���սṹ��*/
typedef struct {   
	RMDLpidUnionStuct_t pidData;
	int32_t accel;
	RMDLencoderStandStruct_t encoderData;
	RMDLangleForgeData angleData;
	RMDLmotorStateStruct_t molotovDataSheet;
	errorScanStruct_t fuze;
}RMDLCanDataRecv_t;
/*��ز��Խṹ������*/
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
