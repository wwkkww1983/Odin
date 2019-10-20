#ifndef __DRIVER_BEEP_H
#define __DRIVER_BEEP_H

#include "stdbool.h"
#include "stm32f4xx.h"

/* �����ͺ�ѡ��ͬʱֻ�ܶ���һ�� */
#define USE_2019_A			//2019��һ�����أ�ȫ�ӿ�
//#define USE_2019_B			//2019�ڶ������أ�������CANΪ���̣�����ADC�ӿ�

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

enum{			//��������
	QUIET=0,
	MUSIC_ARMED,					//������ʾ��
	MUSIC_DISARMED,				//������ʾ��
	MUSIC_IMUCALI,				//IMUУ׼��ʾ��
	MUSIC_PARAMCALI,			//����������ʾ��
	MUSIC_MAGCALI,				//MAGУ׼
	MUSIC_RADIO_LOSS,			//ʧ��
	MUSIC_TYPE_INFANTRY,	//����
	MUSIC_TYPE_TANK,			//Ӣ��
	MUSIC_TYPE_AUXILIARY,	//���̳�
	MUSIC_TYPE_SENTRY,		//�ڱ�
	MUSIC_TYPE_UAV,				//���˻�
	MUSIC_TYPE_SMALLGIMBAL,//С��̨
	MUSIC_NO_ID,					//��ID
	MUSIC_LOWPOWER,				//�͵�ѹ
	MUSIC_LIST
};

enum{     //����
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
     //����
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

