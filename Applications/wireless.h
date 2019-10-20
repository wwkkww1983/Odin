#ifndef __WIRELESS_H
#define __WIRELESS_H

#include "Driver_DTU.h"
#include "FreeRTOS_board.h"
#include "BSP.h"
typedef struct{
	u8 sendCheck;
	u8 sendVersion;
	u8 sendStatus;
	u8 sendSenser;
	u8 sendSenser2;
	u8 sendPid1;
	u8 sendPid2;
	u8 sendPid3;
	u8 sendPid4;
	u8 sendPid5;
	u8 sendPid6;
	u8 sendRcData;
	u8 sendOffSet;
	u8 sendMotoPwm;
	u8 sendPower;
	u8 sendUser;
	u8 sendSpeed;
	u8 sendLocation;
}dt_flag_t;

typedef struct{
	TaskHandle_t xHandleTask;
	u8 imuCalibrate;
	u8 magCalibrate;
	uint32_t loops;
	uint8_t dataNeedSend[50];
	uint8_t checkDataNeedSend;
	uint8_t checkSumNeedSend;
}wirelessStruct_t;

extern dt_flag_t f;
extern wirelessStruct_t wirelessData;

#define WIRELESS_PRIORITY	    6
#define WIRELESS_STACK_SIZE	  256
#define WIRELESS_PERIOD				5

extern TaskHandle_t xHandleTaskWireless;

void ANO_DT_Data_Exchange(void);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_DT_Send_Senser2(s32 bar_alt,u16 csb_alt);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_User(void);
void ANO_DT_Send_Speed(float x_s,float y_s,float z_s);
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
void ANO_DT_Data_Receive_Prepare(u8 data);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void dataSendToGroundStation(void);
void dataTransportTask(void *Parameters);
void wirelessInit(void);

#endif


