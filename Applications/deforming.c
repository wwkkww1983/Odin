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
/*******接收传感器板的数据*********/
/*********************************/
void readLineDistance_data(CanRxMsg *can_rx_msg,allAutoUpStruct_t *allAutoUpDate)    //读取线位移传感器距离以及升降机构速度值
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

void SensorDataReceive(CanRxMsg *can_rx_msg)					//传感器接收函数
{
	
}
/*************************
工程车逻辑程序
**************************/
/* 电机位置数据过零处理，避免出现位置突变的情况 */	
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
		chassisData.chaseRef = 0;													//开启底盘跟随
	}
	else{
		chassisData.chaseRef = chassisData.chaseFbd;			//关闭地盘跟随
	}
}

void grabZeroState(void){  														//恢复基础状态
	
}

void resSetPress(void){

}

static ramp_t liftRamp = RAMP_GEN_DAFAULT;	//升降斜坡
float liftRampForBeter(float liftHightRef,float rampBegin,float liftHighFdb){	//高度目标、开始进入斜坡高度=liftHightRef-rampBegin*5、高度当前值
	static float hightRefOut;
	static int8_t liftHightLastRef,direction,inLiftRampAgain;		//上一次高度目标，方向，改目标值进入时
	if(liftHightLastRef != liftHightRef){		//如果目标值改变，重置斜坡
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

	if(direction > 0){			//向上
		if(liftHighFdb >= (liftHightRef - rampBegin * 5)){
			hightRefOut = (liftHightRef - rampBegin * 5) + rampBegin * 5 * LinearRampCalc(&liftRamp,1);
			return hightRefOut;
		}
		else{
			hightRefOut = liftHightRef + rampBegin * 5;
			return hightRefOut;
		}
	}
	else if(direction < 0){			//向下
		if(liftHighFdb <= (liftHightRef + rampBegin * 5)){
			hightRefOut = (liftHightRef + rampBegin * 5) - rampBegin * 5 * LinearRampCalc(&liftRamp,1);
			return hightRefOut;
		}
		else{
			hightRefOut = liftHightRef - rampBegin * 5;
			return hightRefOut;
		}
	}
	else{			//非上下，返回当前值为目标值
		RampResetCounter(&liftRamp);
		return liftHightRef;
	}
}

void liftUpdate(void){    															//升降机构更新

}

/*************************
英雄车逻辑程序
**************************/


/*************************
公共变形更新
**************************/
void mechaDeformingUpdate(void)								//负责车身变形的控制
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
工程车初始化
**************************/
static void deformationLiftInit(void)    /*  同步带升降pid初始化  */
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
公共初始化
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











