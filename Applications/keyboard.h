#ifndef __KEYBOARD_H
#define __KEYBOARD_H
#include "stm32f4xx.h"
#include "Driver_RMDT7.h"
#include "imu.h"
#include "auto_task.h"


#define FORWARD    (remoteControlData.dt7Value.keyBoard.bit.W)
#define BACK       (remoteControlData.dt7Value.keyBoard.bit.S)
#define LEFT       (remoteControlData.dt7Value.keyBoard.bit.A)
#define RIGHT      (remoteControlData.dt7Value.keyBoard.bit.D)
//      speed      key
#define SLOW_SPD   (remoteControlData.dt7Value.keyBoard.bit.SHIFT)
#define FAST_SPD   (autoTaskData->fastSeed)
//#define SpLOW_SPD   (remoteControlData.dt7Value.keyBoard.bit.CTRL)
//      function   key or mouse operate
#define TWIST_CTRL (remoteControlData.dt7Value.keyBoard.bit.E)
#define BUFF_CTRL  (remoteControlData.dt7Value.keyBoard.bit.F)
#define TRACK_CTRL (km.rk_sta == KEY_PRESS_LONG)
//      shoot mode and shoot type
#define KB_17MM_SHOOT   (keyBoardCtrlData.lkSta == KEY_PRESS_ONCE)
#define KB_17MM_SHOOT_CONTINUOUS  (remoteControlData.dt7Value.mouse.Press_L)
#define KB_42MM_SHOOT   (keyBoardCtrlData.lkSta == KEY_PRESS_ONCE)
#define KB_42MM_SHOOT_CONTINUOUS  (remoteControlData.dt7Value.mouse.Press_L)
//#define KB_TYPY_SHOOT 	(remoteControlData.dt7Value.keyBoard.bit.B && keyBoardCtrlData.lkSta == KEY_PRESS_ONCE)
#define KB_TYPY_SHOOT   (remoteControlData.dt7Value.keyBoard.bit.B)
//#define KB_TYPY_HEAT_PROTECT	(remoteControlData.dt7Value.keyBoard.bit.B && keyBoardCtrlData.rkSta == KEY_PRESS_ONCE)	
#define KB_TYPY_HEAT_PROTECT  (remoteControlData.dt7Value.keyBoard.bit.C)
#define KB_NO_PJEJUDGMENT  (keyBoardCtrlData.lkSta)
#define KB_PJEJUDGMENT     (keyBoardCtrlData.rkSta)

/**********************************************************************************
 * bit      :15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 * keyboard : V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 **********************************************************************************/
//#define W 			0x0001		//bit 0
//#define S 			0x0002
//#define A 			0x0004
//#define D 			0x0008
//#define SHIFT 	0x0010
//#define CTRL 		0x0020
//#define Q 			0x0040
//#define E				0x0080
//#define R 			0x0100
//#define F 			0x0200
//#define G 			0x0400
//#define Z 			0x0800
//#define X 			0x1000
//#define C 			0x2000
//#define V 			0x4000		//bit 15
//#define B				0x8000
/******************************************************/

#define KEY_STATE_0         0       // 按键状态
#define KEY_STATE_1         1
#define KEY_STATE_2         2
#define KEY_STATE_3         3

#define SINGLE_PRESS_TIME     30      //单击时间ms 
#define LONG_PRESS_TIME       500	    //长按时间ms
/* key acceleration time */
#define KEY_ACC_TIME     10000  //ms
#define KEY_DEC_TIME     4000  	//ms

typedef enum{
  NORMAL_MODE = 0,  //底盘普通速度
  FAST_MODE,        //底盘快速
  SLOW_MODE,        //底盘慢速
} keyBoardMove_e;

typedef enum{
  KEY_RELEASE = 0,
  KEY_WAIT_EFFECTIVE,
  KEY_PRESS_ONCE,
  KEY_PRESS_DOWN,
  KEY_PRESS_LONG,
} keyBoardState_e;

typedef struct{
	coordinateFloat_t chassisSpeedTarget;  
  float pitchGyroTarget;
  float yawGyroTarget;
	
	float pitchSpeedTarget;
	float yawSpeedTarget;
	
  uint16_t lk_cnt;
  uint16_t rk_cnt;
  
  keyBoardState_e lkSta;
  keyBoardState_e rkSta;

  keyBoardState_e leftKeyState;
	keyBoardState_e rightKeyState; 
	
  uint16_t xSpeedLimit;
  uint16_t ySpeedLimit;
} keyBoardCtrlStruct_t;

extern keyBoardCtrlStruct_t keyBoardCtrlData;

void getKeyboardMouseState(void);
void keyboardChassisHook(void);
void keyboardGimbalHook(void);
void keyboardShootHook(void);
keyBoardState_e leftKeyDriver(void) ;
keyBoardState_e rightKeyDriver(void);

#endif


