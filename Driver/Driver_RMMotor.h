#ifndef __DRIVER_RMMOTOR_H
#define __DRIVER_RMMOTOR_H

#include "bsp.h"
#include "BSP_GPIO.h"
#define ENCODER_ANGLE_RATIO    (360.0f/8192.0f)
#define FILTER_BUF 3
/************************* ID定义 *********************/
#define ID_MASTER_BOARD						0x101
#define ID_SLAVE_BOARD						0x102

#define ID_RMMOTOR_RIGHT_FRONT 		0x201
#define ID_RMMOTOR_LEFT_FRONT 		0x202
#define ID_RMMOTOR_LEFT_BACK 			0x203
#define ID_RMMOTOR_RIGHT_BACK 		0x204
#define ID_RM6623_YAW		          0x205
#define ID_RM6623_PITCH	          0x206
#define ID_POKEMOTOR		          0x207
#define ID_PAW_L									0x207
#define ID_PAW_R									0x208
#define ID_SUPPLY				          0x208
#define ID_FIRE_TNF_R_OR_YAW      0x205
#define ID_FIRE_TNF_L_OR_PITCH    0x206
#define ID_YAW_INF					 			0x209
#define ID_PITCH_INF         			0x20A
#define ID_TURNTABLE              0x203     //筛弹电机
#define ID_BIGPOKE                0x204     //大发射机构拨弹电机
#define ID_CURRENT                0x301

enum{
	VERSION_RM2016 = 0,
	VERSION_RM2017
};

enum{                         //电机列表
	MOTOR_NO = 0,								//没有ID不会工作  
	MOTOR_6623,		   						//6623电机
	MOTOR_6020,									//GM6020电机
	MOTOR_3510,								  //GM3510电机
	MOTOR_9015,                 //DM9015电机
};

enum{                         //电信返回信息列表 
	CODEBOARD_VALUE = 0,				//码盘值  
	REALTORQUE_CURRENT,		   		//实际转矩电流
	TORQUE_CURRENT,							//给定转矩电流
	TURN_SPEED,								  //转速
	TEMPER,                     //温度
};


typedef struct{
	vs16 currunt1;		//右前方
	vs16 currunt2;   	//左前方
	vs16 currunt3;    //左后方
	vs16 currunt4;   	//右后方
}motorSerialNumber_t;

typedef struct{
	vs16 	encoderAngle;	//电机角度编码器值  
	vs16	realcurrent;	//实际转矩电流测量值				
}dm9015DataRecv_t;

typedef struct{
	vs16 	rawangle;	//电机角度编码器值
	vs16 	speed;		//电机转速
	vs16  currunt;  //转矩电流
	uint8_t  temperature;	//电机温度
}motorCanDataRecv_t;

typedef struct{
	vs16 fdbPosition;        //电机的编码器反馈值
	vs16 last_fdbPosition;   //电机上次的编码器反馈值
	vs16 bias_position;      //机器人初始状态电机位置环设定值
	vs16 fdbSpeed;           //电机反馈的转速/rpm
	vs16 round;              //电机转过的圈数
	vs32 real_position;      //过零处理后的电机转子位置
}rmmotorTarData_t;

typedef struct{
	vs16 	encoderAngle;	//电机角度编码器值  
	vs16	realcurrent;	//实际转矩电流测量值
	vs16	current;			//转矩电流给定值
	float realAngle;		//转换成角度单位						
}gimbalCanDataRecv_t;

typedef struct{                         //通用电机列表              
	gimbalCanDataRecv_t 	M6623Data;	    //6623电机反馈信息          
	gimbalCanDataRecv_t 	GM3510Data;    //GM3510电机反馈                      
	motorCanDataRecv_t    GM6020Data;    //GM6020电机反馈           
  dm9015DataRecv_t      DM9015Data;	   //DM9015电机反馈
	uint8_t  motorID;                     //电机种类ID               
}rmmotorCanDataRecv_t;

typedef struct{
  uint16_t ecd;
  float  rotationalSpeed;
  int16_t  current;
  int16_t  temp;
	
	uint16_t lastEcd;
  int32_t  roundCnt;
  int32_t  totalEcd;
  int32_t  totalAngle;
  
  uint16_t offsetEcd;
  uint32_t msgCnt;
  
  int32_t  ecdRawRate;
  int32_t  rateBuf[FILTER_BUF];
  uint8_t  bufCut;
  int32_t  filterRate;
} motorMeasureData_t;

typedef struct{
	vs16 currentAll;		//id为总电机的电流值	
	vs16 current201;		//id为0x201电机的电流值				
	vs16 current202;		//id为0x202电机的电流值				
	vs16 current203;		//id为0x203电机的电流值				
	vs16 current204;		//id为0x204电机的电流值
	vs16 powerRank ;    //超功率等级
}currentCanDataRecv_t;

/*******************************************************************/
void driver_can1_init(CAN_TypeDef* rm_canx,BSP_GPIOSource_TypeDef *rm_canx_rx,BSP_GPIOSource_TypeDef *rm_canx_tx,u8 Preemption,u8 Sub);
void driver_can2_init(CAN_TypeDef* rm_canx,BSP_GPIOSource_TypeDef *rm_canx_rx,BSP_GPIOSource_TypeDef *rm_canx_tx,u8 Preemption,u8 Sub);
void gimbal_readdata(CanRxMsg *can_rx_msg,gimbalCanDataRecv_t *gimbal_data);
void rmmotor_readdata(CanRxMsg *can_rx_msg,motorCanDataRecv_t *chassisData);
void rmmotor_senddata(CAN_TypeDef *CANx, u32 ID_CAN, motorSerialNumber_t *rmmotor_cansend);
int16_t getRelativePos(int16_t rawEcd, int16_t centerOffset);
void driver_rm6623_calibration(CAN_TypeDef *canx, unsigned char version);
void mecanumCalculate(float V_X,float V_Y,float V_Rotate,float MaxWheelSpeed,float* Wheel_Speed);
void gimbal_motor6020_readdata(CanRxMsg *can_rx_msg,motorCanDataRecv_t *motor6020Data);

void gimbal_readData(CanRxMsg *can_rx_msg,rmmotorCanDataRecv_t *gimbal_data); 
vs16 gimbal_chooseData(uint8_t dataNumber,rmmotorCanDataRecv_t *gimbal_data);
void gimbal_readSlaveData(uint8_t dataNumber,vs16 writeData,rmmotorCanDataRecv_t *gimbal_data);

extern motorCanDataRecv_t	 		fricWheelData[2];
extern motorCanDataRecv_t	 		pokeData;
extern motorCanDataRecv_t	 		lidData;
extern motorCanDataRecv_t  		turntableData;
extern motorCanDataRecv_t	 		bigPokeData;      
extern motorCanDataRecv_t	 		wheelData[4];
extern rmmotorCanDataRecv_t 	pitchMotorData,yawMotorData; 
extern motorMeasureData_t   	motorViewPitData,motorViewYawData;
extern currentCanDataRecv_t 	currentDate;
extern motorMeasureData_t   	motorGM3510PitData;
extern motorCanDataRecv_t	 		pawMotorData[2];												//工程爪子
extern motorCanDataRecv_t	 		deformMotorData[2];										//工程横向变形机构
extern motorCanDataRecv_t	 		liftMotorData[2];											//工程抬升机构
extern rmmotorTarData_t 	 		pawMotorTarData[2];		
#endif


