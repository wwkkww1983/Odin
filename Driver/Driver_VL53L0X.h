#ifndef __DRIVER_VL53L0X_H
#define __DRIVER_VL53L0X_H
#include "BSP.h"
#define ID_VL53L0X_1 		0x401
#define ID_VL53L0X_2 		0x402
#define ID_VL53L0X_3 		0x403
#define ID_VL53L0X_4 		0x404

#define ID_VL53L0X_SEND 		0x501
#define ID_VL53L0X_READ 		0x502

#define TOF1             0
#define TOF2             1
#define TOF3             2
#define TOF4             3

#define TOFS1             4
#define TOFS2             5
#define TOFS3             6
#define TOFS4             7

enum{
	TOF_CAIL_NO_NEED = 0,
	TOF_CAIL_START ,
	TOF_CAIL_BEING ,
	TOF_CAIL_FINISH ,
};

typedef struct 
{
  vs16 dis_basic;
	vs16 distance;
	float speed;
}vl53l0x_raw_data_t;

typedef struct 
{
	float distance;
	float last_dis;
	float speed;
}vl53l0x_real_data_t;

typedef struct 
{
	uint8_t state[8];
}vl53l0x_state_data_t;

typedef struct firstOrderFilterData
{
    float   gx1;
    float   gx2;
    float   gx3;
    float   previousInput;
    float   previousOutput;
}firstOrderFilterData_t; 
void initFirstOrderFilter(void);
float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters);
void read_vl53l0x_raw_data(CanRxMsg *can_rx_msg,vl53l0x_raw_data_t *rawdata);
void read_vl53l0x_real_data(vl53l0x_raw_data_t *rawdata,vl53l0x_real_data_t *realdata);
void vl5310_senddata(CAN_TypeDef *CANx, u32 ID_CAN, vl53l0x_state_data_t *vl53l0x_state);
extern vl53l0x_raw_data_t vl53l0x_raw_data[4];
extern vl53l0x_real_data_t vl53l0x_real_data[4];
extern firstOrderFilterData_t firstOrderFilters[8];
extern vl53l0x_state_data_t  tofSendDate;
extern vl53l0x_state_data_t  tofReadDate;

#endif




