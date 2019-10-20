#include "deforming.h"
#include "rc.h"
#include "keyboard.h"
#include "config.h"
#include "ramp.h"
#include "DRIVER_VL53L0X.h"
#include "Driver_ADC.h"
#include "Driver_ITV.h"
#include "judge.h"
#include "chassis.h"
#include "gimbal.h"
#include "cansend.h"
#include "control.h"
#include "auto_tank.h"
#include "auto_auxiliary.h"
#include "imu.h"

allAutoUpStruct_t allAutoUpDate; 

/*********************************/
/*******���մ������������*********/
/*********************************/
void readLineDistance_data(CanRxMsg *can_rx_msg,allAutoUpStruct_t *allAutoUpDate)    //��ȡ��λ�ƴ����������Լ����������ٶ�ֵ
{
	
}

void readOpSwitch602_data(CanRxMsg *can_rx_msg)    //
{
	for(int i = 0 ; i < 8 ; i++ ){
		allAutoUpDate.optoelectronicRecive[i] = can_rx_msg->Data[i];
	}
}

void readmeltSwitch608_data(CanRxMsg *can_rx_msg)    //
{
		allAutoUpDate.magSwitch = can_rx_msg->Data[0];
}

void tof_readdata(CanRxMsg *can_rx_msg)
{
	if((can_rx_msg->Data[0] == 0XAC)&&(can_rx_msg->Data[1] == 0XAD)){
	 allAutoUpDate.tofdate[0].u8_temp[0] = can_rx_msg->Data[2];
	 allAutoUpDate.tofdate[0].u8_temp[1] = can_rx_msg->Data[3];
	}
}

void tof_readdata2(CanRxMsg *can_rx_msg)
{
	if((can_rx_msg->Data[0] == 0XAC)&&(can_rx_msg->Data[1] == 0XAD)){
	 allAutoUpDate.tofdate[1].u8_temp[0] = can_rx_msg->Data[2];
	 allAutoUpDate.tofdate[1].u8_temp[1] = can_rx_msg->Data[3];
	}
}

void SensorDataReceive(CanRxMsg *can_rx_msg)					//���������պ���
{
	
}
/*************************
���̳��߼�����
**************************/
/* ���λ�����ݹ��㴦���������λ��ͻ������ */	
void motorRawAngle(rmmotorTarData_t* tarMotor,motorCanDataRecv_t* srcMotor){

	tarMotor->last_fdbPosition = tarMotor->fdbPosition;
	tarMotor->fdbPosition = srcMotor->rawangle;
	if(tarMotor->fdbPosition - tarMotor->last_fdbPosition > 4096)
	{
		tarMotor->round --;
	}
	else if(tarMotor -> fdbPosition - tarMotor->last_fdbPosition < -4096)
	{
		tarMotor->round ++;
	}
	tarMotor->real_position = tarMotor->fdbPosition + tarMotor->round * 8192;
}

void chassisChaseSwitch(u8 isEnable){
	if(isEnable){
		chassisData.chaseRef = 0;													//�������̸���
	}
	else{
		chassisData.chaseRef = chassisData.chaseFbd;			//�رյ��̸���
	}
}

void grabZeroState(void){  														//�ָ�����״̬
	
}

void resSetPress(void){

}

static ramp_t liftRamp = RAMP_GEN_DAFAULT;	//����б��
float liftRampForBeter(float liftHightRef,float rampBegin,float liftHighFdb){	//�߶�Ŀ�ꡢ��ʼ����б�¸߶�=liftHightRef-rampBegin*5���߶ȵ�ǰֵ
	static float hightRefOut;
	static int8_t liftHightLastRef,direction,inLiftRampAgain;		//��һ�θ߶�Ŀ�꣬���򣬸�Ŀ��ֵ����ʱ
	if(liftHightLastRef != liftHightRef){		//���Ŀ��ֵ�ı䣬����б��
		liftHightLastRef = liftHightRef;
		RampResetCounter(&liftRamp);
		inLiftRampAgain = 1;
	}
	if(inLiftRampAgain == 1){
		inLiftRampAgain = 0;
		if(liftHightRef > liftHighFdb)
			direction = 1;
		else if(liftHightRef < liftHighFdb)
			direction = -1;	
	}

	if(direction > 0){			//����
		if(liftHighFdb >= (liftHightRef - rampBegin * 5)){
			hightRefOut = (liftHightRef - rampBegin * 5) + rampBegin * 5 * LinearRampCalc(&liftRamp,1);
			return hightRefOut;
		}
		else{
			hightRefOut = liftHightRef + rampBegin * 5;
			return hightRefOut;
		}
	}
	else if(direction < 0){			//����
		if(liftHighFdb <= (liftHightRef + rampBegin * 5)){
			hightRefOut = (liftHightRef + rampBegin * 5) - rampBegin * 5 * LinearRampCalc(&liftRamp,1);
			return hightRefOut;
		}
		else{
			hightRefOut = liftHightRef - rampBegin * 5;
			return hightRefOut;
		}
	}
	else{			//�����£����ص�ǰֵΪĿ��ֵ
		RampResetCounter(&liftRamp);
		return liftHightRef;
	}
}

void liftUpdate(void){    															//������������

}

/*************************
Ӣ�۳��߼�����
**************************/


/*************************
�������θ���
**************************/
void mechaDeformingUpdate(void)								//��������εĿ���
{  
	u8 i;
	if(robotConfigData.typeOfRobot == TANK_ID){
		
	}
	else if(robotConfigData.typeOfRobot == AUXILIARY_ID){
		for(i = 0;i<2;i++)
			motorRawAngle(&pawMotorTarData[i],&pawMotorData[i]);
    liftUpdate();
	}
}


/*************************
���̳���ʼ��
**************************/
static void deformationLiftInit(void)    /*  ͬ��������pid��ʼ��  */
{
	RampInit(&liftRamp, 2000);
	allAutoUpDate.schedule1=1;
	allAutoUpDate.deformingCount=1;
	for(int i=0;i<2;i++){
		allAutoUpDate.positionRef[i]=INITDISTANCE;
		/***/
	}
}

/************************
������ʼ��
**************************/
void mechaDeformingInit(void)
{
	if(robotConfigData.typeOfRobot == TANK_ID){

		
	}
	else if(robotConfigData.typeOfRobot == AUXILIARY_ID){
		deformationLiftInit();
		grabZeroState();
	}
}











