#ifndef __DEFORMING_H
#define __DEFORMING_H
#include "BSP.h"
#include "Util.h"
#include "pid.h"
#include "chassis.h"

/************
Ӣ�۳�
*************/
#define TANK_NORMAL_DISTANCE  70.0f
#define TANK_MAX_DISTANCE     540.0f
#define TANK_MIN_DISTANCE     30.0f
#define TANK_GRAB_DISTANCE    185.0f

#define TANK_GRAB_MAX_ANGLE	335		//�Ƕȴ�����ֵ��ץ�������ֵ470
#define TANK_GRAB_MIN_ANGLE	230		//�Ƕȴ�����ֵ��ץ�������ֵ
#define TANK_GRAB_THROW_ANGLE	315		//�Ƕȴ�����ֵ��ץ�Ӷ���ҩ��ֵ

#define LANDING_1 PEin(2)
#define LANDING_2 PEin(13)

#define LANDING_3 PAin(1)
#define LANDING_4 PEin(10)

#define HOLDPILLAR_SENSOR PCin(5)
#define GRABLEFTSENSOR  PEin(14)
#define GRABRIGHTSENSOR PEin(4)

#define GRAB_LED  PDout(13)
#define GRAB_LED_ON 	GRAB_LED=1;
#define GRAB_LED_OFF 	GRAB_LED=0;

#define GRAB PBout(1)
#define GRAB_ON 	GRAB=1;
#define GRAB_OFF 	GRAB=0;

#define HOLDPILLAR PBout(0)
#define HOLDPILLAR_ON  HOLDPILLAR=1;
#define HOLDPILLAR_OFF HOLDPILLAR=0;

#define RIGHT_BYPASS_MOTOR TIM3->CCR2
#define RIGHT_BYPASS_MOTOR_ON   (RIGHT_BYPASS_MOTOR = 1750)
#define RIGHT_BYPASS_MOTOR_OFF  (RIGHT_BYPASS_MOTOR = 1000)

#define LEFT_BYPASS_MOTOR  TIM3->CCR1
#define LEFT_BYPASS_MOTOR_ON   (LEFT_BYPASS_MOTOR = 1750)
#define LEFT_BYPASS_MOTOR_OFF  (LEFT_BYPASS_MOTOR = 1000)

/************
���̳��ǵ�
*************/
#define ID_LINE_DISTANCE         0x601     //��λ�ƴ�����  ����������ٶ�
#define ID_OPTOLECTONIC_SWITCH   0x602     //�ƹ��ź�
#define ID_SENSORDATA            0x704     //ץ������������
#define ID_TOF1_4DATE            0x401     //tof1-4����
#define ID_OPTOLECTONIC1_SWITCH  0x604
#define ID_OPTOLECTONIC2_SWITCH  0x606
#define MELT_DATA                0x602
#define LIFTDISTANCEMAX       	440    		//�������λ��468
#define LIFTDISTANCEMIN       	20    		//����������Сλ��
#define BULLETTRANSFERDISTANCE	330				//�����߶�
#define GRAB_DISTANCE         	270     	//ץ���߶�
#define INITDISTANCE          	50      	//�����ʼλ��
#define LANDINGDISTANCE       	36

#define HOLD_PILLAR_SPEED     7500
#define holdPillarDistanceMax 0
#define holdPillarDistanceMin 0
#define HOLDPILLARON          1
#define HOLDPILLAROFF         0
#define DUCTEDFANLEFT_ON			0
#define DUCTDEFANLEFT_OFF			1
#define DUCTDEFANRIGHT_ON			0
#define DUCTDEFANRIGHT_OFF		0
#define Op_INITIAL_DISTANCE   0       //2.3�Ź�翪�س�ʼ��װ�ݵ����λ��
#define GRAB_LIFT_DISTANCE    0       //ץ�����������߶�
#define GRAB_MEASURE_DISTANCE 0       //ץ��ʱ��ǽ�ľ���
#define GRAB_DISTANCE_FOR_WALL_MIN     0       //ץ����λʱ��ǽ��С����
#define GRAB_DISTANCE_FOR_CAISSSON_MIN     0   //ץ��ʱ���һ�ŵ�ҩ���λ��
#define GRAB_DISTANCE_FOR_CAISSSON_MAX     0   //ץ��ʱ��ڶ��ŵ�ҩ���λ��
#define DOWN_HOLDPILLAR_POSITION   0     //�½�����ʱ�������λ��
#define HOLD_PILLAR_ROTATE_REF 1900/4     //ÿ����50Ȧ
#define HOLD_PILLAR_LOOSEN pneumaticData.send1[2]=0x01 //�����ɿ�
#define HOLD_PILLAR_TIGHT  pneumaticData.send1[2]=0x00  //�������� 

/************
���̳�ץ��
*************/
/* 
���¿����Ƕ�Ӧʵ�ﶯ�������Ƕ�Ӧ���׵Ŀ���
צ�ӵĴ򿪶�Ӧ���׵Ĺر�
צ�ӵĹرն�Ӧ���׵Ĵ�
*/
#define EJECT_OPEN    				pneumaticData.send1[0]=1  	//�����
#define EJECT_CLOSE   				pneumaticData.send1[0]=0  	//����ر�
#define SMALLMAGAZINE_OPEN    pneumaticData.send1[1]=1  	//С���ֿ�
#define SMALLMAGAZINE_CLOSE   pneumaticData.send1[1]=0  	//С���ֹ�
#define PAW_OPEN              pneumaticData.send1[2]=1  	//��צ��
#define PAW_CLOSE             pneumaticData.send1[2]=0  	//�н�צ��
#define PAWGO_FORWARD         pneumaticData.send1[3]=1  	//צ��ǰ��
#define PAWGO_BACK         		pneumaticData.send1[3]=0  	//צ�Ӻ���
#define PAWTURN_OPEN          pneumaticData.send1[4]=1  	//צ�ӷ�ת��
#define PAWTURN_CLOSE         pneumaticData.send1[4]=0  	//צ�ӷ�ת�ر�

#define PAWGOLEFT_OPEN        pneumaticData.send2[0]=1  	//צ�����ƴ�
#define PAWGOLEFT_CLOSE       pneumaticData.send2[0]=0  	//צ�����ƹر�
#define PAWGORIGHT_OPEN       pneumaticData.send2[1]=1  	//צ�����ƴ�
#define PAWGORIGHT_CLOSE      pneumaticData.send2[1]=0		//צ�����ƹر�

#define TOP_RIGHT							pneumaticData.send3[0]=1		//���ƿ�
#define TOP_LEFT							pneumaticData.send3[0]=0		//���ƹ�
#define TAIL_OPEN							pneumaticData.send3[1]=1		//β��֧�ſ�
#define TAIL_CLOSE						pneumaticData.send3[1]=0		//β��֧�Ź�
#define UP_OPEN								pneumaticData.send3[2]=1		//̧����
#define UP_CLOSE							pneumaticData.send3[2]=0		//̧����

#define ONE_LEFT              allAutoUpDate.optoelectronicRecive[0]
#define ONE_RIGHT             allAutoUpDate.optoelectronicRecive[1]
#define TWO_LEFT              allAutoUpDate.optoelectronicRecive[2]
#define PAWTURN_SENSOR        allAutoUpDate.magSwitch  		//��תצ�Ӵ����� 0Ϊ��⵽�ջ�צ��

/************
���̳���Ԯ
*************/
#define RESCUE_L			  (keyBoardCtrlData.lkSta == KEY_PRESS_ONCE)
#define RESCUE_R   			(keyBoardCtrlData.rkSta == KEY_PRESS_ONCE)
#define RESCUE_TIGHT  pneumaticData.send3[3]=1		//��Ԯ��������
#define RESCUE_LOOSEN pneumaticData.send3[3]=0		//��Ԯ����̧��
 
/************
���̳��ӵ�����
*************/
#define MAGAZINE_OPEN  pneumaticData.send3[4]=1	//���ָǴ�
#define MAGAZINE_CLOSE pneumaticData.send3[4]=0	//���ָǹر�


typedef struct 
{
//	uint8_t auxiliarySchdule;
	uint8_t schedule1;
	uint8_t deformingCount;
	uint8_t holdPillarPidFlag[2];
	
	formatTrans16Struct_t tofdate[2];      								//tof����
	uint8_t selfdeformingComplete;    										//�ǵ�������ɱ�־
	uint8_t grabBulletComplete;
	uint8_t downIslandComplete;
	uint8_t liftSchedule;     														//���̳������������
	uint8_t grabSchedule;   														  //ץ���������
	uint8_t dowmIslandSchedule;  													//�µ��������
	uint8_t optoelectronicRecive[8];						 					//��翪�ط����ź�
	uint8_t magSwitch;																		//���״���������
	uint8_t solenoidvalueContralSignal[8];    						//������ŷ������ź�  //0��������ŷ�  1:
	uint8_t grabOpContralSignal[8];   										//צ�ӵ�ŷ������ź�
	uint8_t ductedFan[2];            											//�������ӦductedFan[0]
	
	
	formatTrans32Struct_t liftSpeed[2];										//���̳���ǰ�����ٶ�
	formatTrans32Struct_t liftDistance[2];                //���̳���ǰ��������    ������ʽλ�ƴ���������
	float speedRef[2];                 								    //��������
	float speedOut[2];
	float positionRef[2];
	float positionOut[2];
	float checkPositionDistence[6];      									//1�������λ���룬23�������ת��ݵ��̵ľ���,4.5Ϊץ��ʱ����ǽ��λ��	
	pidStruct_t *liftSpeedPID[2];
	pidStruct_t *liftPositionPID[2];
	float revolveFbd[2];
	float revolveRef[2];
  float revolveOut[2];
	pidStruct_t *imuRevolvePID[2];
	float pawmotorSpeedFbd[2];
	float pawmotorSpeedRef[2];
  float pawmotorSpeedOut[2];
	pidStruct_t *pawmotorSpeedPID[2];
	float intervalTime;
//	float adc_test[8];
	
}allAutoUpStruct_t;         														//��������(�����ǵ�ץ���µ�ȫ����)

extern allAutoUpStruct_t allAutoUpDate;  

void mechaDeformingUpdate(void);
void mechaDeformingInit(void);
void grabAmmunitionUpdate(void);
uint8_t tankGrabForward(void);
uint8_t tankGrabBack(void);
uint8_t tankGrabThrow(void);
uint8_t supplyGrabForward(void);				//ǰ��

/********
���̳�
********/
void readLineDistance_data(CanRxMsg *can_rx_msg,allAutoUpStruct_t *allAutoUpDate);   //��ȡ��λ�ƴ���������ֵ
void readOpSwitch602_data(CanRxMsg *can_rx_msg);   //��翪��
void SensorDataReceive(CanRxMsg *can_rx_msg);
void readmeltSwitch608_data(CanRxMsg *can_rx_msg);    //�ƹ��ź� 01�ǵ����ź� 23��ץ����λ
void tof_readdata(CanRxMsg *can_rx_msg);
void tof_readdata2(CanRxMsg *can_rx_msg);
void liftUpdate(void);
void chassisChaseSwitch(u8 isEnable);
void grabFirstMiddle(void);
void grabZeroState(void);
void resSetPress(void);
#endif
