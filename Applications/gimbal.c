#include "gimbal.h"
#include "slave_sensor.h"
#include "keyboard.h"
#include "rc.h"
#include "ramp.h"
#include "config.h"
#include "Driver_RMMotor.h"
#include "imu.h"
#include "chassis.h"
#include "cansend.h"
#include "judge.h"
#include "control.h"
#include "vision.h"
#include "SEGGER_RTT.h"
#include "math.h"
#include "slave_sensor.h"

/*
***************************************************
函 数 名：	gimbalUpdate
功		能：云台任务更新
入口参数：	gimbalData.visionYawCmd				横向坐标系反馈
					gimbalData.visionPitchCmd			纵向坐标系反馈
					gimbalData.autoMode		切换自动任务标志
返 回 值：	无
应用范围：	外部调用
备		注：
***************************************************
*********************************************************/

gimbalStruct_t gimbalData;
ramp_t yawVisionRamp = RAMP_GEN_DAFAULT;											//云台斜坡初始化
ramp_t pitchVisionRamp = RAMP_GEN_DAFAULT;

static ramp_t yawRamp = RAMP_GEN_DAFAULT;											//云台斜坡初始化
static ramp_t pitchRamp = RAMP_GEN_DAFAULT;

int8_t getInstallDirect(uint8_t installPara,bool type){
	int8_t res;
	if(type)
		res = (installPara & 0x02)?-1:1;
	else
		res = (installPara & 0x01)?-1:1;
	return res;
}

u8 acc;
void gimbalUpdate(void){
	getGimbalCtrlDate();															
	switch (robotMode){
		case MODE_INIT:           
			gimbalInitHandle();   
			break;//初始化模式
		case MODE_RC: 			     
		case MODE_KM:{
			if(gimbalData.ctrlMode == GIMBAL_INIT){
				gimbalInitHandle();
			}
			else if(gimbalData.ctrlMode == GIMBAL_NORMAL){
				gimbalFollowHandle();
			}
			else if(gimbalData.ctrlMode == GIMBAL_STOP){
				gimbalStopHandle();
			}
			break;
		}
		case MODE_STOP:{	 
			gimbalStopHandle();	 
			break;//丢控模式
		}
		case MODE_RELAX:{        
			gimbalRelaxHandle();  
			break;//解除控制权
		}
		default:                                        
			break;
	}

	gimbalData.time[0] = getClockCount();
	gimbalData.intervalTime = (float)(gimbalData.time[0] - gimbalData.time[1]);
	gimbalData.time[1] = gimbalData.time[0];
	
	//视觉模式云台自动
	if(gimbalData.autoMode && visionData.captureFlag && visionData.cailSuccess){
		gimbalData.pitchAngleRef = visionData.pitchCmd;				 //自动模式下同步当前值到角度期望
		gimbalData.yawAngleRef = visionData.yawCmd;	
		//如果是自瞄状态云台角度反馈取陀螺仪角度（不加偏置）
		gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;
		gimbalData.yawAngleFbd = gimbalData.yawGyroAngle - gimbalData.yawAngleSave;
	}
	
	//云台迫击炮模式
	if(gimbalData.autoMode && (ROBOT == TANK_ID)&&(autoTaskData->currentTask == TANK_MORTAR)){
		gimbalData.pitchAngleRef = visionData.mortarPitCmd;				 //自动模式下同步当前值到角度期望
		gimbalData.yawAngleRef = visionData.mortarYawCmd;	
		//如果是自瞄状态云台角度反馈取陀螺仪角度（不加偏置）
		gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;
		gimbalData.yawAngleFbd = gimbalData.yawGyroAngle - gimbalData.yawAngleSave;
	}
	
	if(isnan(gimbalData.yawAngleOut)||isnan(gimbalData.pitchAngleOut)){
		pidZeroState(gimbalData.pitchAnglePID);
		pidZeroState(gimbalData.pitchSpeedPID);
		pidZeroState(gimbalData.yawAnglePID);
		pidZeroState(gimbalData.yawSpeedPID);
		digitalClan(&gimbalData.yawAngleOut);
		digitalClan(&gimbalData.pitchAngleOut);
		digitalClan(&gimbalData.yawSpeedOut);
		digitalClan(&gimbalData.pitchSpeedOut);		
		
		gimbalData.pitchAngleRef = gimbalData.pitchAngleFbd;				
		gimbalData.yawAngleRef = gimbalData.yawAngleFbd;	
	}
	
	//只有鼠标给云台期望时才执行
	if(robotMode == MODE_KM && !(gimbalData.autoMode && visionData.captureFlag && visionData.cailSuccess) \
		&& !infantryAutoData.rotateFlag && ROBOT == INFANTRY_ID){
		//鼠标移动时只闭速度环
		gimbalData.yawSpeedRef = keyBoardCtrlData.yawSpeedTarget;
		//该轴没有速度期望时闭角度环
		if(!keyBoardCtrlData.yawSpeedTarget){
			if(gimbalData.angleCycleStep == 2 || gimbalData.angleCycleStep == 0){
				gimbalData.yawAngleOut = pidUpdate(gimbalData.yawAnglePID, gimbalData.yawAngleRef, gimbalData.yawAngleFbd,gimbalData.intervalTime); 
				gimbalData.yawSpeedRef = -gimbalData.yawAngleOut;																		
			}
		}
	}
	else{
		gimbalData.yawAngleOut   = pidUpdate(gimbalData.yawAnglePID, gimbalData.yawAngleRef, gimbalData.yawAngleFbd,gimbalData.intervalTime); 
		gimbalData.yawSpeedRef   = -gimbalData.yawAngleOut;
	}

	gimbalData.pitchAngleOut = pidUpdate(gimbalData.pitchAnglePID, gimbalData.pitchAngleRef, gimbalData.pitchAngleFbd,gimbalData.intervalTime);
	gimbalData.pitchSpeedRef = -gimbalData.pitchAngleOut;
	
	gimbalData.pitchSpeedFbd = IMU_RATEY;                    // 速度环反馈用imu角速度速度
	gimbalData.yawSpeedFbd   = IMU_RATEZ;
	
	gimbalData.pitchSpeedOut = -getInstallDirect(parameter[PITCH_INSTALL],INSTALL_TURN) * pidUpdate(gimbalData.pitchSpeedPID, gimbalData.pitchSpeedRef, gimbalData.pitchSpeedFbd,gimbalData.intervalTime);
	gimbalData.yawSpeedOut   = getInstallDirect(parameter[YAW_INSTALL],INSTALL_TURN) * pidUpdate(gimbalData.yawSpeedPID, gimbalData.yawSpeedRef, gimbalData.yawSpeedFbd,gimbalData.intervalTime);
	if(robotMode == MODE_RELAX){
		digitalClan(&gimbalData.yawSpeedOut);
		digitalClan(&gimbalData.pitchSpeedOut);
	}
}

static void gimbalPatrolHandle(void){
	static uint8_t shootSchedule = 1;
	static uint32_t lostCNT = 0,mode = 0;
	static bool captureFlag = 0,init = 0;
	
	float yawBias = 0.0f;
	float pitchBias = 0.0f;
	
	captureFlag = visionData.captureFlag;
	if(!captureFlag){	//没有识别到目标则重置自瞄状态
		lostCNT ++;
	}
	else{
		lostCNT = 0;
		mode = 1;
	}
	
	//大炮管自瞄时先跟随		
	if(controlTransData.otherAutoMaticFlag && !init){
		mode = 2;
	}
	else if(!controlTransData.otherAutoMaticFlag){
		init = false;
	}
	
	if(controlTransData.otherPatrolMode){
		mode = 0;
	}
	
	switch(mode){
		//未捕获目标巡逻模式
		case 0:{
			gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;//设置pitch轴反馈
			gimbalData.yawAngleFbd = getInstallDirect(parameter[YAW_INSTALL],INSTALL_ENCODER) * gimbalData.yawMotorAngle;   //设置yaw轴反馈
			gimbalData.yawAngleRef = -controlTransData.masterYawMotorAngle.float_temp;
			gimbalData.pitchAngleRef = controlTransData.masterPitchMotorAngle.float_temp;
		}break;
		//捕获目标跟随
		case 1:{		
			if(lostCNT > 250){	
				mode = 0;
				shootSchedule = 1;				
				visionData.prejudgFlag = false;
				visionData.fireFbd = false;
				shootDataReset();
				visionData.miniPTZEnableAim = false;
				visionData.workMode = TX2_STOP;
			}		
			else{
				mode = 1;
			}
			visionData.miniPTZEnableAim = true;		
			miniPTZAutomaticAimUpdate(&shootSchedule);	
			if(fabs(gimbalData.yawAngleFbd-(gimbalData.yawGyroAngle - gimbalData.yawAngleSave))<1.0f){
				gimbalData.pitchAngleRef = visionData.pitchCmd;				 //自动模式下同步当前值到角度期望
				gimbalData.yawAngleRef = visionData.yawCmd;	
			}
			//如果是自瞄状态云台角度反馈取陀螺仪角度（不加偏置）
			gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;
			gimbalData.yawAngleFbd = gimbalData.yawGyroAngle - gimbalData.yawAngleSave;
		}break;
		case 2:{
			gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;//设置pitch轴反馈
			gimbalData.yawAngleFbd = getInstallDirect(parameter[YAW_INSTALL],INSTALL_ENCODER) * gimbalData.yawMotorAngle;   //设置yaw轴反馈
			gimbalData.yawAngleRef = -controlTransData.masterYawMotorAngle.float_temp;
			gimbalData.pitchAngleRef = controlTransData.masterPitchMotorAngle.float_temp;
			yawBias = gimbalData.yawAngleRef - gimbalData.yawAngleFbd;
			pitchBias = gimbalData.pitchAngleRef - gimbalData.pitchAngleFbd;
			if((fabsf(yawBias) < 2.0f)&&(fabsf(pitchBias) < 2.0f)){
				init = true;
			}			
		}break;
	}
}

float shiftAngle = 0;
static void gimbalFollowHandle(void){
	static float lastYawSpeedRef;
	digitalClan(&gimbalData.initFinishFlag);
	digitalClan(&gimbalData.motorFlag);
	if(parameter[ROBOT_TYPE] == SMALLGIMBAL_ID){
		gimbalPatrolHandle();
	}
	else{
		gimbalData.pitchAngleFbd = gimbalData.pitchGyroAngle;
		gimbalData.yawAngleFbd = gimbalData.yawGyroAngle - gimbalData.yawAngleSave;
		gimbalData.yawAngleRef += remoteControlData.yawGyroTarget + keyBoardCtrlData.yawGyroTarget;
		gimbalData.pitchAngleRef += remoteControlData.pitchGyroTarget + keyBoardCtrlData.pitchGyroTarget;
		if(robotMode == MODE_KM && !(gimbalData.autoMode && visionData.captureFlag) \
			&& !infantryAutoData.rotateFlag && ROBOT == INFANTRY_ID){
			//鼠标从运动到停止，不能以此时角度闭环
			if(!keyBoardCtrlData.yawSpeedTarget && lastYawSpeedRef){
				gimbalData.angleCycleStep = 1;
			}
			//等到云台速度接近为0时，开始闭角度环
			if((fabs(IMU_RATEZ) < 0.08f) && (gimbalData.angleCycleStep == 1)){
				gimbalData.yawAngleRef = gimbalData.yawAngleFbd;
				gimbalData.angleCycleStep = 2;
			}
			lastYawSpeedRef = keyBoardCtrlData.yawSpeedTarget;
		}
	}
	if(robotConfigData.typeOfRobot == AUXILIARY_ID){
	  gimbalData.yawAngleFbd  = getInstallDirect(parameter[YAW_INSTALL],INSTALL_ENCODER) * gimbalData.yawMotorAngle;
	  gimbalData.yawAngleRef = 0.0f + shiftAngle;
	}
	//pitch轴角度限幅
	gimbalData.pitchAngleRef = constrainFloat(gimbalData.pitchAngleRef, parameter[PITCH_MIN_RANGE], parameter[PITCH_MAX_RANGE]); 
}


static void gimbalInitHandle(void){
	static float pitchRampAngle;
	static float yawRampAngle;
	static TickType_t xLastWakeTime = 0;
	if(lastRobotMode != robotMode||gimbalData.ctrlMode != gimbalData.lastCtrlMode){																			//每一次进入初始化模式都获取当前角度并初始化斜坡函数
		xLastWakeTime = xTaskGetTickCount();
		gimbalRampInit();
		pitchRampAngle = getInstallDirect(parameter[PITCH_INSTALL],INSTALL_ENCODER) * gimbalData.pitchMotorAngle;  				//获取pitch轴imu当前角度
		yawRampAngle   = getInstallDirect(parameter[YAW_INSTALL],INSTALL_ENCODER) * gimbalData.yawMotorAngle;   					//获取yaw码盘当前角度
	}
	gimbalData.pitchAngleFbd = getInstallDirect(parameter[PITCH_INSTALL],INSTALL_ENCODER) * gimbalData.pitchMotorAngle;	//设置pitch轴反馈
	gimbalData.pitchAngleRef = pitchRampAngle * (1 - LinearRampCalc(&pitchRamp,2));																			//pitch轴回中

	gimbalData.yawAngleFbd = getInstallDirect(parameter[YAW_INSTALL],INSTALL_ENCODER) * gimbalData.yawMotorAngle;   		//设置yaw轴反馈
	gimbalData.yawAngleRef = yawRampAngle * (1 - LinearRampCalc(&yawRamp,2));       																		//yaw轴回中
	
  	//等待pitch、yaw轴回中
	if(((gimbalData.yawMotorAngle < 2.0f && gimbalData.yawMotorAngle > -2.0)\
	&&(gimbalData.pitchMotorAngle < 2.0f && gimbalData.pitchMotorAngle > -2.0f))\
	|| xTaskGetTickCount() - xLastWakeTime > parameter[BACK_CENTER_TIME] * 3 ){
		gimbalData.ctrlMode  = GIMBAL_NORMAL;       							//设置云台模式为跟随模式
		if ((robotConfigData.typeOfRobot == SENTRY_ID)
			||(robotConfigData.typeOfRobot == UAV_ID)
			||(robotConfigData.typeOfRobot == AUXILIARY_ID))
			chassisData.ctrlMode = CHASSIS_SEPARATE_GIMBAL;					//设置底盘模式为分离模式																																											
		else
			chassisData.ctrlMode = CHASSIS_FOLLOW_GIMBAL;						//设置底盘模式为跟随模式
		gimbalData.pitchAngleSave = gimbalData.pitchGyroAngle;		//保存从初始化模式到跟随模式的陀螺仪角度
		gimbalData.yawAngleSave = gimbalData.yawGyroAngle;
		digitalClan(&gimbalData.yawAngleRef);                    	//期望值清零
		digitalClan(&gimbalData.pitchAngleRef);
		digitalHi(&gimbalData.initFinishFlag);                    //云台回中完成标志置一
	}
}

static void gimbalStopHandle(void){
	static float pitchRampAngle;
	static float yawRampAngle;
	static TickType_t xLastWakeTime = 0;

	if(lastRobotMode != robotMode || gimbalData.ctrlMode != gimbalData.lastCtrlMode){																						//每一次进入初始化模式都获取当前角度并初始化斜坡函数
		xLastWakeTime = xTaskGetTickCount();
		gimbalRampInit();
	  pitchRampAngle = getInstallDirect(parameter[PITCH_INSTALL],INSTALL_ENCODER) * gimbalData.pitchMotorAngle;  								//获取pitch码盘当前角度
		yawRampAngle   = getInstallDirect(parameter[YAW_INSTALL],INSTALL_ENCODER) * gimbalData.yawMotorAngle;    									//获取yaw码盘当前角度
	}
  gimbalData.pitchAngleFbd = getInstallDirect(parameter[PITCH_INSTALL],INSTALL_ENCODER) * gimbalData.pitchMotorAngle;					//设置pitch轴反馈
	gimbalData.yawAngleFbd   = getInstallDirect(parameter[YAW_INSTALL],INSTALL_ENCODER) * gimbalData.yawMotorAngle;  						//设置yaw轴反馈
	
	if(gimbalData.motorFlag){
		gimbalData.pitchAngleRef = pitchRampAngle + (gimbalData.pitchAngleStop - pitchRampAngle) * LinearRampCalc(&pitchRamp,1);	//pitch轴回中	  
		gimbalData.yawAngleRef = yawRampAngle + (gimbalData.yawAngleStop - yawRampAngle) * LinearRampCalc(&yawRamp,1);						//yaw轴回中	
	}
	else{
		digitalClan(&gimbalData.pitchAngleStop);
		digitalClan(&gimbalData.yawAngleStop);
		gimbalData.pitchAngleRef = pitchRampAngle * (1 - LinearRampCalc(&pitchRamp,2));//pitch轴回中
		gimbalData.yawAngleRef = yawRampAngle * (1 - LinearRampCalc(&yawRamp,2));       //yaw轴回中
	}

  	//等待pitch、yaw轴回中
	if(((gimbalData.yawMotorAngle < gimbalData.yawAngleStop + 2.0f && gimbalData.yawMotorAngle > gimbalData.yawAngleStop - 2.0f)\
		&& (gimbalData.pitchMotorAngle < gimbalData.pitchAngleStop + 2.0f && gimbalData.pitchMotorAngle > gimbalData.pitchAngleStop - 2.0f))\
		|| xTaskGetTickCount() - xLastWakeTime > parameter[BACK_CENTER_TIME] * 2 ){
		gimbalData.pitchAngleSave = gimbalData.pitchGyroAngle;//保存从初始化模式到跟随模式的陀螺仪角度
		gimbalData.yawAngleSave = gimbalData.yawGyroAngle;
		if(gimbalData.motorFlag){
			
		}
		else{
			digitalClan(&gimbalData.yawAngleRef);                    //期望值清零
			digitalClan(&gimbalData.pitchAngleRef);
		}
	}
}

static void gimbalRelaxHandle(void){
	 digitalLo(&gimbalData.initFinishFlag);
	 digitalLo(&gimbalData.motorFlag);
   pidZeroState(gimbalData.pitchAnglePID);
	 pidZeroState(gimbalData.pitchSpeedPID);
	
	 pidZeroState(gimbalData.yawAnglePID);
	 pidZeroState(gimbalData.yawSpeedPID);
}

void gimbalStopSwitch(uint8_t active){									//云台闭角度环
	if(active){
		digitalHi(&gimbalData.motorFlag);																					
		gimbalData.yawAngleStop = gimbalData.yawAngleStopSet;
		gimbalData.pitchAngleStop = gimbalData.pitchAngleStopSet;
		gimbalData.ctrlMode = GIMBAL_STOP;
		chassisData.ctrlMode = CHASSIS_SEPARATE_GIMBAL;
	}
	else{
		gimbalData.ctrlMode = GIMBAL_INIT;
		digitalLo(&gimbalData.motorFlag);
	}	
}

void gimbalRampInit(void){ 													  	//斜坡函数初始化
  RampInit(&pitchRamp, parameter[BACK_CENTER_TIME]/2);
  RampInit(&yawRamp, parameter[BACK_CENTER_TIME]/2);
	
	RampInit(&pitchVisionRamp, 100);
  RampInit(&yawVisionRamp, 100);
}

void gimbalInit(void){
	gimbalData.ctrlMode  = GIMBAL_RELAX;
	gimbalRampInit();
	
	gimbalData.pitchSpeedPID = pidInit(&parameter[TILT_RATE_P], &parameter[TILT_RATE_I], &parameter[TILT_RATE_D], &parameter[TILT_RATE_F],	\
																&parameter[TILT_RATE_PM], &parameter[TILT_RATE_IM], &parameter[TILT_RATE_DM], &parameter[TILT_RATE_OM],	\
																NULL, NULL, NULL, NULL);

	gimbalData.pitchAnglePID = pidInit(&parameter[TILT_ANG_P], &parameter[TILT_ANG_I], &parameter[TILT_ANG_D], &parameter[TILT_ANG_F],	\
																 &parameter[TILT_ANG_PM], &parameter[TILT_ANG_IM], &parameter[TILT_ANG_DM], &parameter[TILT_ANG_OM],	\
																 NULL, NULL, NULL, NULL);

	gimbalData.yawSpeedPID = pidInit(&parameter[YAW_RATE_P], &parameter[YAW_RATE_I], &parameter[YAW_RATE_D], &parameter[YAW_RATE_F],	\
																&parameter[YAW_RATE_PM], &parameter[YAW_RATE_IM], &parameter[YAW_RATE_DM], &parameter[YAW_RATE_OM],	\
																NULL, NULL, NULL, NULL);

	gimbalData.yawAnglePID = pidInit(&parameter[YAW_ANG_P], &parameter[YAW_ANG_I], &parameter[YAW_ANG_D], &parameter[YAW_ANG_F],	\
																 &parameter[YAW_ANG_PM], &parameter[YAW_ANG_IM], &parameter[YAW_ANG_DM], &parameter[YAW_ANG_OM],	\
																 NULL, NULL, NULL, NULL);
}
