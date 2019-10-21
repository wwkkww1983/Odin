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

auxiliaryStruct_t auxiliaryData; 

/*********************************/
/*******���մ������������*********/
/*********************************/

void readOpSwitch602_data(CanRxMsg *can_rx_msg)    //
{
	for(int i = 0 ; i < 8 ; i++ ){
		auxiliaryData.optoelectronicRecive[i] = can_rx_msg->Data[i];
	}
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
		motorRawAngle(&pawMotorTarData[0],&pawMotorData[0]);
		auxiliaryData.pawangle[0].dataFbd = pawMotorTarData[0].real_position;
		auxiliaryData.pawangle[0].dataOut = pidUpdate(auxiliaryData.pawmotorAnglePID[0],auxiliaryData.pawangle[0].dataRef,auxiliaryData.pawangle[0].dataFbd,auxiliaryData.intervalTime);
		auxiliaryData.pawmotor[0].dataRef = -auxiliaryData.pawangle[0].dataOut;
		auxiliaryData.pawmotor[1].dataRef = auxiliaryData.pawangle[0].dataOut;	
		for(i = 0;i<2;i++){
			auxiliaryData.pawmotor[i].dataOut= pidUpdate(auxiliaryData.pawmotorSpeedPID[i],auxiliaryData.pawmotor[i].dataRef,auxiliaryData.pawmotor[i].dataFbd,auxiliaryData.intervalTime);
		}
		liftUpdate();
	}
}


/*************************
���̳���ʼ��
**************************/
static void deformationLiftInit(void)    /*  ͬ��������pid��ʼ��  */
{
	RampInit(&liftRamp, 2000);
	for(uint8_t index = 0;index < 2;index++){			
		auxiliaryData.pawmotorSpeedPID[index] = pidInit(&parameter[CHASSIS_POS_P], &parameter[CHASSIS_POS_I], &parameter[CHASSIS_POS_D], &parameter[CHASSIS_POS_F],	\
												 	  &parameter[CHASSIS_POS_PM], &parameter[CHASSIS_POS_IM], &parameter[CHASSIS_POS_DM], &parameter[CHASSIS_POS_OM],	\
													  NULL, NULL, NULL, NULL); 
		auxiliaryData.pawmotorAnglePID[index] = pidInit(&parameter[CHASSIS_POS_P], &parameter[CHASSIS_POS_I], &parameter[CHASSIS_POS_D], &parameter[CHASSIS_POS_F],	\
												 	  &parameter[CHASSIS_POS_PM], &parameter[CHASSIS_POS_IM], &parameter[CHASSIS_POS_DM], &parameter[CHASSIS_POS_OM],	\
													  NULL, NULL, NULL, NULL); 
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











