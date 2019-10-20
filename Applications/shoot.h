#ifndef __SHOOT_H
#define __SHOOT_H
#include "BSP.h"
#include "Util.h"
#include "pid.h"
#include "Driver_RMMotor.h"
#include "control.h"

/* �������ѡ��ͬʱֻ�ܶ���һ�� */
#define USE_PNEUMATIC								//ʹ������
//#define USE_FRICWHEEL								//ʹ��Ħ����

#define FricMotor_L TIM9->CCR1
#define FricMotor_R TIM9->CCR2

#define BULLET_MONITOR_FLAG					shootData.bulletMonitorFlag

#define LASER_ON										PBout(5)=1//����
#define LASER_OFF										PBout(5)=0//����

#define SHOOT_HEAT(shootSpeed)			(float)shootSpeed
#define FIRE_FLAG 			         		shootData.fireFlag_17mm	//�����ź� 
#define AIRDROP_FLAG								shootData.airdropFlag		//�յ������ź�			��ע�����������ֵ.
	
#define SHOOT_TIMES_CONTINUOUS			 3											//������������·����ӵ���Ŀ
#define SMALL_BULLET_HARM						 10											//С�ӵ��˺�
#define BIG_BULLET_HARM							 100										//���ӵ��˺�
#define BULLET_SPEED_DEFAULT    		 26.0f			    				//С�ӵ�Ĭ������

#define MIN_SPEED                    0
#define MAX_SPEED                    5000
#define SNAIL_MOTOR                  DISABLE

#define protectHeartTank             100										//Ӣ�۱�������

typedef enum{
	MANUAL_SINGLE = 0,
	MANUAL_CONTINUOUS,
	AUTO_CONTINUOUS,
}shootMode_e;      						//���ģʽ

typedef enum{
	SAFE = 0,
//	WARNING,
	DANGEROUS,
}shootStatus_e;								//���״̬

typedef enum{
	MOTOR_REVERSAL = -1,
	MOTOR_STOP = 0,
	MOTOR_FOWARD = 1,
}motorStatus_e;								//���״̬

typedef struct{
	TickType_t xLastWakeTime;	  //ʱ����
	shootMode_e shootMode;		  //���ģʽ
	shootStatus_e shootStatus;  //���״̬
	shootStatus_e shootStatusMode;//�����ȫģʽ
  TaskHandle_t xHandleTaskshoot;	
  pidStruct_t *speedPID;
	pidStruct_t *testSpeedPID;
	pidStruct_t *fricWheelSpeedPID[2];
	pidStruct_t *turntablePID;
	pidStruct_t *loadedSpeedPID;
	errorScanStruct_t fricWheelError[2];
	errorScanStruct_t lidError;
	errorScanStruct_t pokeError;
	bool  clearBulletFlag;      //�嵯��־λ
	float smallPokeSpeedRef;  	//�������ٶ�����ֵ
  float smallPokeSpeedOut; 	  //������PID���ֵ
	float bigPokeSpeedRef;  		//�������ٶ�����ֵ
  float bigPokeSpeedOut; 	  	//������PID���ֵ
	uint8_t autoMode;
	uint8_t airdropFlag;			  //���յ��ӵ���־
	float shootSpeedLast;				//�ϴ��ӵ�����
	uint16_t fricSpeedSet_42mm; //42mmĦ����ת���趨
	uint16_t fricSpeedSet_17mm;	//17mmĦ����ת���趨
	float fricWheelSpeedRef[2];	//Ӣ�۳��ӵ���������ֵ
	float	fricWheelSpeedOut[2];	//Ӣ�۳��ӵ�����PID���ֵ
	uint8_t bulletMonitor[3]; 	//ǰ3�ι�紫����״̬
	float OutputGain;						//���ݴ�С���Ƿ񼤻����ж��������
	uint16_t bulletRemain;		  //ʣ���ӵ���
	uint16_t shootManualNumber; //ʣ���ֶ��������
	uint8_t fireDataInit;				//�������flag
	uint8_t shootTrigger; 			//�������ָ��
	float shootHPRef;   		  	//���Ԥ���˺�
	float shooterJudgeLastHeat_17mm;
	float shooterJudgeHeat_17mm;
	float shooterHeat;			  	//ǹ������
	float shooterHeatCoolingPs;	//ǹ������ÿ0.1����ȴֵ
	float shooterHeatMax;	  		//ǹ���������ֵ
	float shooterHeatRemain;  	//ǹ����������ֵ
	TickType_t shootWaitTime;
	uint8_t monitorFlag;
	uint8_t fricMotorFlag;		  //Ħ���ֿ�����־
	uint8_t fireFlag_42mm;		  //42mm�ӵ������־
	uint8_t fireFlag_17mm;		  //17mm�ӵ������־
	uint8_t leval;						  //�����˵ȼ�
	
	shootMode_e shootModeTank;		  //���ģʽ
	shootStatus_e shootStatusTank;  //���״̬
	shootStatus_e shootStatusModeTank;//�����ȫģʽ
	
	uint8_t loadStep;									//װ���
	bool ballisticClean;							//���������־
	bool ballisticFill;								//��������־
	bool bulletRelax;									//װ���־
	bool bulletReady;									//���ű�־
	bool bulletExist;									//���ű�־
  int8_t turntableStall;						//��Ͳ��ת��־
	uint8_t suicideFireFlag;					//��ɱ�����־
	
  int8_t pokeStall;                //������תλ
	uint8_t pokeFlag;             //�����־
	uint8_t shootTriggerTank;   
  uint8_t shootOutNumber;       //�ѷ����ӵ���
  uint8_t shootFillNumber;      //����ӵ���
	uint16_t shootTimesTank;      //Ԥ���ӵ���
	uint16_t shooterTimesOutTank; //�ѷ����ӵ���
	uint16_t shooterTimesLimitTank; //�ɷ����ӵ���
	float shooterHeatCoolingPsTank;
	float turntableOut;
	float turntableRef;
	float turnSpeed;                //��Ͳת��
	float lastShooterHeat1;  
	float shootSpeedLastTank;				//�ϴ��ӵ�����	
	float shooterHeatTank;			  	//ǹ������
	float shooterHeatMaxTank;	  		//ǹ���������ֵ
	float shooterHeatRemainTank;  	//ǹ����������ֵ
	uint8_t bulletMonitorFlag;
	float intervalTime;
	double time[2];
	uint32_t loops;
}shootStruct_t;

void shootDataReset(void);
void shooterHeatIncreasing(void);
void shooterHeatCooling(void);
void shooterHeatAbjust(void);
void pokeMoterReadData( CanRxMsg *can_rx_msg, motorCanDataRecv_t *pokeMoterData );
void shootUpdate(void);
void shootInit(void);
void shooterHeatIncreasingTank(void);
void TankShootHeatAbjust(void);	
void smallPokeSpeedRefChange(void);
void shootProtectTank(void);

extern shootStruct_t shootData;

#endif
