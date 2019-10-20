#ifndef __DRIVER_BEEP_H
#define __DRIVER_BEEP_H

#include "stdbool.h"
#include "stm32f4xx.h"

/* 主控型号选择，同时只能定义一个 */
#define USE_2019_A			//2019第一版主控，全接口
//#define USE_2019_B			//2019第二版主控，串口与CAN为焊盘，新增ADC接口

#ifdef USE_2019_A
#define BEEP_LENGTH 50
#define BEEP_MAX_NOISE 50
#define BEEP_TIM TIM4
#define BEEP_PORT BEEP_TIM->CCR2
#endif

#ifdef USE_2019_B
#define BEEP_LENGTH 100
#define BEEP_MAX_NOISE 100
#define BEEP_TIM TIM10
#define BEEP_PORT BEEP_TIM->CCR1
#endif

#define BEEP_PRESCALER 180
#define BEEP_MIN_NOISE 0
#define BEEP_NOTE_MIN 10
#define BEEP_NOTE_MAX 32768 
#define BEEP_PSC BEEP_TIM->PSC
#define MUSIC_MAX_LENGHT 48

#define LED_SLOW 10
#define LED_NORMAL 4
#define LED_FAST 1
#define LED_LIST 10

#define LED_R	PDout(5)
#define LED_G PDout(4)
#define LED_B PDout(6)

#define	ID_0x0101	7
#define ID_0x0102 8
#define ID_0x0103 9
#define ID_0x0104 10
#define ID_0x0105 11

enum{			//声音种类
	QUIET=0,
	MUSIC_ARMED,					//解锁提示音
	MUSIC_DISARMED,				//上锁提示音
	MUSIC_IMUCALI,				//IMU校准提示音
	MUSIC_PARAMCALI,			//参数保存提示音
	MUSIC_MAGCALI,				//MAG校准
	MUSIC_RADIO_LOSS,			//失控
	MUSIC_TYPE_INFANTRY,	//步兵
	MUSIC_TYPE_TANK,			//英雄
	MUSIC_TYPE_AUXILIARY,	//工程车
	MUSIC_TYPE_SENTRY,		//哨兵
	MUSIC_TYPE_UAV,				//无人机
	MUSIC_TYPE_SMALLGIMBAL,//小云台
	MUSIC_NO_ID,					//无ID
	MUSIC_LOWPOWER,				//低电压
	MUSIC_LIST
};

enum{     //音阶
	BASS_DO=0,
	BASS_RE,
	BASS_MI,
	BASS_FA,
	BASS_SOL,
	BASS_LA,
	BASS_SI,
	ALTO_DO,
	ALTO_RE,
	ALTO_MI,
	ALTO_FA,
	ALTO_SOL,
	ALTO_LA,
	ALTO_SI,
	HIGH_DO,
	HIGH_RE,
	HIGH_MI,
	HIGH_FA,
	HIGH_SOL,
	HIGH_LA,
	HIGH_SI,
};

enum{
	LED_HARDWORK_FAIL = 1,
	LED_RADIO_LOSS,
	LED_MAG_CALI,
	LED_WORKINGORDER,
	LED_RESERVE_1,
	LED_IMU_CALI,
	LED_GIMBAL_CALI,
	LED_RESERVE_2,
	LED_DEFINE_ID
};

enum{
	LED_DISWORK=0,
	LED_WORK_RED,
	LED_WORK_GREEN,
	LED_WORK_BLUE
};
enum{
	LED_ENABLE=0,
	LED_DISABLE	
};
     //音量
#define	N_NOISE 0
#define L_NOISE 50
#define	H_NOISE 100

typedef struct {
	uint16_t note[MUSIC_MAX_LENGHT];
	uint16_t volume[MUSIC_MAX_LENGHT];
	uint16_t lenght;
} BeepSound_t;

typedef struct {
	uint16_t colour;
	uint16_t frequency;
} LedData_t;

void beepUpdate(uint16_t beepSound,uint16_t note);
void ledUpdateTask(uint16_t commandState);
void beepUpdateTask(uint16_t commandSate);
void beepConfig(void);

#endif

