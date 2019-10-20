#ifndef __WARNING_H
#define __WARNING_H

#include "Util.h"
#include "Driver_SK6812.h"
#include "Driver_Beep.h"

#define WARNING_PRIORITY 8
#define WARNING_STACK_SIZE 64
#define WARNING_STACK_PERIOD 100

#define SK6812_GREEN  sk6812Data.colorStd[COLOR_GREEN]
#define SK6812_RED	  sk6812Data.colorStd[COLOR_RED]
#define SK6812_YELLOW sk6812Data.colorStd[COLOR_YELLOW]
#define SK6812_DARK		sk6812Data.colorStd[COLOR_DARK]
#define SK6812_BLUE		sk6812Data.colorStd[COLOR_BLUE]
#define SK6812_PINK		sk6812Data.colorStd[COLOR_PINK]
#define SK6812_WHITE	sk6812Data.colorStd[COLOR_WHITE]

#define CAP_SOC       ((float)(100 * (powerData.capVol - 17) / (powerData.capVolMax - 17)))

enum{
	RC_FAULT = 0x0001,									//遥控器故障或丢失
	JUDGE_FAULT = 0x0002,								//裁判系统异常
	MOTOR_FAULT = 0x0004,								//电机是否有异常	
	SENSOR_FAULT = 0x0008,							//云台板是否故障
	VISION_FAULT = 0x0010,							//TF卡未插入或异常
	POWER_LIMIT_FAULT = 0x0020,					//电容控制板故障	
//	IMU_TEMP_FAULT = 0x0040,						//恒温未完成
	SAFE_MODE = 0x0080,
	DANGEROUS_MODE = 0x0100,
	AVIOD_STAE = 0x0200,
	ROTATE_STAE = 0x0400,
	CAP_CHARGE_STATE = 0x0800,
	LINK_CAP_STATE = 0x1000,
	R_TASK_STATE = 0x2000,
	V_TASK_STATE = 0x4000,
	Z_TASK_STATE = 0x8000,
};

typedef struct{
	formatTrans16Struct_t lightBarsState;
	uint8_t highestFault;
	uint8_t highestFaultFlag;
	uint8_t highestFaultLoop;
	hsvColor_t caliLightColour;
	hsvColor_t displayColor;
	uint8_t displayNumber;
	uint8_t blinkFrequency;
	uint8_t reportError;
	uint8_t capSoc;
	float progressBar;
	uint32_t loops;
}warningStruct_t;

extern warningStruct_t warningData;

void warningUpdate(void);
void lightBarsStateUpdata(void);

#endif

