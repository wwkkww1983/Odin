#include "chassis.h"
#include "gimbal.h"
#include "rc.h"
#include "keyboard.h"
#include "config.h"
#include "Driver_RMMotor.h"
#include "judge.h"
#include "cansend.h"
#include "control.h"
#include "auto_auxiliary.h"
#include "auto_infantry.h"
#include "imu.h"

//#define CHASSIS_SPEED_ADRC
#define ANTI_SLIP
#define CHASSIS_SPEED_PID
#define ADD_IMU_RATE_PID

chassisStruct_t chassisData;
/*
***************************************************
函 数 名：	chassisDataUpdate
功		能：底盘任务更新
入口参数：	chassisData.autoSpeedTarget 底盘旋转运动速度目标(结构体，在任务中赋值x,y,z)
					chassisData.autoMode：切换自动任务标志
返 回 值：	无
应用范围：	外部调用
备		注：
**************************************************
*/
void chassisUpdate(void){
	if(imuSensorOfChassisData.initFlag){
		//读取底盘陀螺仪数据
		imuChassisUpdate();																									
	}
//	powerRealData(&powerRawDate,&powerRealDate);
	//更新车身实际最大速度限幅
	chassisData.speedLimitFloor = chassisData.speedLimit * parameter[CHASSIS_RC_SPEED]*REAL_MOTOR_SPEED_SCALE;
	for (uint8_t i = 0; i < 4; i++) {
		//底盘速度反馈赋值
		chassisData.speedFbd[i] = (float)wheelData[i].speed * REAL_MOTOR_SPEED_SCALE;
		//扫描底盘轮子最大速度
		if(fabs(chassisData.speedFbd[i]) >= chassisData.speedFbdMax)
			chassisData.speedFbdMax = fabs(chassisData.speedFbd[i]);
	}
	
#ifdef ANTI_SLIP
	for(uint8_t index = 0; index < NUMBER_OF_WHEEL; index++){	
		//低速时认为不打滑
		if(fabs(chassisData.speedRef[index]) < 0.2f)
			chassisData.scale[index] = 1.0f;
		else
			//得到当前电流轮子打滑系数，该值越大打滑程度越大    
			chassisData.scale[index] = (float)fabs(chassisData.speedFbd[index] / chassisData.speedRef[index]);
	}
	chassisData.averageScale = (float)(chassisData.scale[0] + chassisData.scale[1] + \
	chassisData.scale[2] + chassisData.scale[3]) / 4;
	for(uint8_t index = 0; index < NUMBER_OF_WHEEL; index++){	
		chassisData.scale[index] = (float)chassisData.averageScale / chassisData.scale[index];
		if(chassisData.averageScale < 0.25f){ //上坡时后轮电流权重进一步放大
//			chassisData.scale[0] = chassisData.scale[1] = 0.5f;
//			chassisData.scale[2] = chassisData.scale[3] = 2.0f;
		}
		chassisData.scale[index] = chassisData.scale[index] > 2.0f? 2.0f : chassisData.scale[index];
		chassisData.scale[index] = chassisData.scale[index] < 0.05f? 0.05f : chassisData.scale[index];
	}
#else
	for(uint8_t index = 0; index < NUMBER_OF_WHEEL; index++){	
		chassisData.scale[index] = 1.0f;
	}
#endif
	
	chassisData.time[0] = getClockCount();
	chassisData.intervalTime = (float)(chassisData.time[0] - chassisData.time[1]);
	chassisData.time[1] = chassisData.time[0];
	
	switch(robotMode){
		//跟随云台模式
		case MODE_RC : 
		case MODE_KM :{
			//底盘跟随云台
			if(chassisData.ctrlMode == CHASSIS_FOLLOW_GIMBAL){
				followGimbalHandle();
			}
			//底盘云台分离
			else if(chassisData.ctrlMode == CHASSIS_SEPARATE_GIMBAL){
				separateGimbalHandle();
			}
			//丢控
			else if(chassisData.ctrlMode == CHASSIS_STOP||chassisData.ctrlMode == CHASSIS_INIT){
				chassisStopHandle();
			}
		}break;
		case MODE_INIT:
		//丢控停止模式
		case MODE_STOP:
			chassisStopHandle();			
			break;
		//解除控制权模式	
		case MODE_RELAX:						
			chassisRelaxHandle();			
			break;		
		default :                                             
			break;
	}
	
	if(chassisData.autoMode){																																							//自动任务激活时,平移运动交给任务自动完成
		chassisData.autoSpeedTarget.x = pidUpdate(chassisData.posPID, chassisData.posRef.x, \
												  chassisData.posFbd.x, chassisData.intervalTime);
		chassisData.autoSpeedTarget.y = -pidUpdate(chassisData.posPID, chassisData.posRef.y, \
												   chassisData.posFbd.y, chassisData.intervalTime);
		chassisData.autoSpeedTarget.z = pidUpdate(chassisData.posPID, chassisData.posRef.z, \
												  chassisData.posFbd.z, chassisData.intervalTime);
		mecanumCalculate(chassisData.autoSpeedTarget.x + chassisData.landingSpeedx,
						 chassisData.autoSpeedTarget.y + chassisData.landingSpeedy, 
		                 chassisData.autoSpeedTarget.z + chassisData.landingSpeedz,
						 chassisData.speedLimit * parameter[CHASSIS_RC_SPEED]*REAL_MOTOR_SPEED_SCALE, chassisData.speedRef);        //麦轮解算得到四个轮子的期望速度
	}
	else{
		mecanumCalculate(chassisData.direction * (chassisData.manualSpeedTarget.x + chassisData.landingSpeedx), 
						 chassisData.direction * (chassisData.manualSpeedTarget.y + chassisData.landingSpeedy), 
		                 chassisData.direction * (chassisData.manualSpeedTarget.z + chassisData.landingSpeedz),
						 chassisData.speedLimit * parameter[CHASSIS_RC_SPEED] * REAL_MOTOR_SPEED_SCALE, chassisData.speedRef);        //麦轮解算得到四个轮子的期望速度
		if(chassisData.direction == -1.0f){//运动方向为负向
			mecanumCalculate(chassisData.direction * (chassisData.manualSpeedTarget.x + chassisData.landingSpeedx), 
							 chassisData.direction * (chassisData.manualSpeedTarget.y + chassisData.landingSpeedy), 
							 chassisData.manualSpeedTarget.z + chassisData.landingSpeedz,
							 chassisData.speedLimit * parameter[CHASSIS_RC_SPEED] * REAL_MOTOR_SPEED_SCALE, chassisData.speedRef);    //麦轮解算得到四个轮子的期望速度	
		}
		//如果有超机动则减小麦轮速度
		if(chassisData.speedFbdMax > chassisData.speedLimitFloor){
			chassisData.DecelerateRatio = chassisData.speedLimitFloor/chassisData.speedFbdMax;			
			for(uint8_t i = 0; i < NUMBER_OF_WHEEL; i++){
				chassisData.speedRef[i] = chassisData.DecelerateRatio*chassisData.speedRef[i];
			}
		}
	}
	
	//速度环更新																																		
	for(uint8_t i = 0; i < NUMBER_OF_WHEEL; i++){
#ifdef CHASSIS_SPEED_PID
		chassisData.current[i] = pidUpdate(chassisData.speedPID[i],chassisData.speedRef[i],chassisData.speedFbd[i],chassisData.intervalTime);
#endif
#ifdef CHASSIS_SPEED_ADRC
		chassisData.current[i] = adrcUpdate(chassisData.speedADRC[i],chassisData.speedRef[i],chassisData.speedFbd[i]);
#endif
	}
	//速度环输出
	for(uint8_t i = 0; i < NUMBER_OF_WHEEL; i++){
		chassisData.powerCurrent[i] = chassisData.current[i];						
	}
	
	//工程车底盘不限功率，则设备列表中不应有该设备
	if(robotConfigData.robotDeviceList & DEVICE_CURRENT){
		/******C620电调电流跟随性能好，可以不闭电流环******
		//速度环串电流环输出
		for(uint8_t i = 0; i < NUMBER_OF_WHEEL; i++){
			chassisData.powerCurrent[i] += pidUpdate(chassisData.currentPID[i],chassisData.current[i],wheelData[i].currunt,chassisData.intervalTime);//电流环PID更新
		}
		**********************************************/
		//对输出功率进行限制
		powerLimitHandle();
	}

	if(robotMode == MODE_RELAX){
		memset((void *)&chassisData.powerCurrent, 0, sizeof(chassisData.powerCurrent));
		//上控时底盘与云台夹角为0
		chassisData.chaseRef = 0.0f; 
		//前进的方向为正向
		chassisData.direction = 1.0f;
		//上控复位时 始终在一个位置	
		parameter[YAW_CENTER] = chassisData.yawCenterSave;      
	}
}

static void chassisStopHandle(void){
	//丢控停止模式期望速度全为零
  chassisData.manualSpeedTarget.y = 0;
  chassisData.manualSpeedTarget.x = 0;
  chassisData.manualSpeedTarget.z = 0;
}

//解除控制权模式复位pid
static void chassisRelaxHandle(void){
	pidZeroState(chassisData.chasePID);
	for(uint8_t index = 0;index < NUMBER_OF_WHEEL;index++){
		pidZeroState(chassisData.speedPID[index]);
	}
}

//跟随云台模式
static void followGimbalHandle(void){
	float angleError;
	float sinAngle;
	float cosAngle;

	gimbalData.yawMotorAngle   = ENCODER_ANGLE_RATIO * getRelativePos(gimbal_chooseData(CODEBOARD_VALUE,&yawMotorData),parameter[YAW_CENTER]);
	//底盘跟随反馈赋值	
	chassisData.chaseFbd = gimbalData.yawMotorAngle;     
	
#if	USE_CHANGE_HEAD	
	if(chassisData.changeHeadOrder && robotConfigData.typeOfRobot==INFANTRY_ID){
		chassisData.chaseFbd = 0.0f;
		chassisData.chaseRef = chassisData.chaseFbd;      		
	}
	if(robotMode==MODE_RC && (chassisData.changeHeadSchedule==2 || chassisData.changeChassisSchedule==2) && robotConfigData.typeOfRobot==INFANTRY_ID){												//用遥控器掉头时 人眼认为掉头完成 强制切RC模式										
		parameter[YAW_CENTER] =  (parameter[YAW_CENTER] + 4096) > 8192?  (parameter[YAW_CENTER] - 4096) : (parameter[YAW_CENTER] + 4096);
		gimbalData.yawMotorAngle = ENCODER_ANGLE_RATIO * getRelativePos(motorYawData.encoderAngle,parameter[YAW_CENTER]); 
		//开始跟随
		chassisData.changeHeadOrder = 0.0f; 
		//改变前进的方向
		chassisData.direction = -chassisData.direction; 
		chassisData.changeHeadSchedule = 0.0f;
		chassisData.changeChassisSchedule = 0.0f;
		chassisData.chaseRef = 0.0f;
	}
#endif
	
	angleError = chassisData.chaseFbd * DEG_TO_RAD;
	sinAngle = sinf(angleError);
	cosAngle = cosf(angleError);
	
	chassisData.manualSpeedTarget.x = (remoteControlData.chassisSpeedTarget.x + keyBoardCtrlData.chassisSpeedTarget.x) * cosAngle \
							          - (remoteControlData.chassisSpeedTarget.y + keyBoardCtrlData.chassisSpeedTarget.y) * sinAngle;
	
	chassisData.manualSpeedTarget.y = (remoteControlData.chassisSpeedTarget.x + keyBoardCtrlData.chassisSpeedTarget.x) * sinAngle \
									  + (remoteControlData.chassisSpeedTarget.y + keyBoardCtrlData.chassisSpeedTarget.y) * cosAngle;
#ifdef ADD_IMU_RATE_PID
	chassisData.chaseAngleOut = -getInstallDirect(parameter[YAW_INSTALL], INSTALL_TURN) \
								* pidUpdate(chassisData.chasePID, chassisData.chaseFbd, \
											chassisData.chaseRef, chassisData.intervalTime);
	chassisData.chaseSpeedRef = chassisData.chaseAngleOut;
	if(ROBOT == INFANTRY_ID && powerData.rotateFast && infantryAutoData.rotateFlag)
		chassisData.chaseSpeedRef *= 1.5f;
	else
		chassisData.chaseSpeedRef *= 1.0f;
	chassisData.chaseSpeedFbd = IMU_CHASE_RATEZ;
	chassisData.manualSpeedTarget.z = pidUpdate(chassisData.chaseSpeedPID, chassisData.chaseSpeedFbd, \
												chassisData.chaseSpeedRef, chassisData.intervalTime);
	chassisData.manualSpeedTarget.z = chassisData.manualSpeedTarget.z * REAL_MOTOR_SPEED_SCALE;		 									
#else
	chassisData.manualSpeedTarget.z = getInstallDirect(parameter[YAW_INSTALL],INSTALL_TURN) \
									* pidUpdate(chassisData.chasePID, chassisData.chaseFbd, \
												chassisData.chaseRef, chassisData.intervalTime);
#endif
	
}

//底盘分离云台模式
static void separateGimbalHandle(void){
	//哨兵只有y轴速度
	if(robotConfigData.typeOfRobot == SENTRY_ID)
		chassisData.manualSpeedTarget.x = 0;
	else
		chassisData.manualSpeedTarget.x = (remoteControlData.chassisSpeedTarget.x + keyBoardCtrlData.chassisSpeedTarget.x);
	
	chassisData.manualSpeedTarget.y = (remoteControlData.chassisSpeedTarget.y + keyBoardCtrlData.chassisSpeedTarget.y);

	if(robotConfigData.typeOfRobot == SENTRY_ID)
		chassisData.manualSpeedTarget.z = 0;
	else	
		chassisData.manualSpeedTarget.z = - getInstallDirect(parameter[YAW_INSTALL], INSTALL_TURN) \
										  * (remoteControlData.chassisSpeedTarget.z + keyBoardCtrlData.chassisSpeedTarget.z);
}

static void powerLimitHandle(void){
	float totalCurrent,totalCurrentLimit;
	//裁判系统检测出或者未安装裁判系统
	if((judgeData.extPowerHeatData.chassis_volt == 0) || (judgeData.extPowerHeatData.chassis_power == 0)){
		//底盘功率不受限制，四个电机均能工作到最大电流
			totalCurrentLimit = NO_JUDGE_TOTAL_CURRENT_LIMIT;
	}
	else{
		//如果缓冲能量小于50J
		if(judgeData.extPowerHeatData.chassis_power_buffer < WARNING_POWER_BUFF){
			float powerScale = 0.0f;
			if(judgeData.extPowerHeatData.chassis_power_buffer > 5.0f){
				powerScale = judgeData.extPowerHeatData.chassis_power_buffer / WARNING_POWER_BUFF;
			}
			else{
				powerScale = 5.0f / WARNING_POWER_BUFF;  //这个系数乘限制电流为80w左右
			}
			totalCurrentLimit = powerScale * JUDGE_TOTAL_CURRENT_LIMIT;
		}
		//如果缓冲能量多于50J
		else{
			//当前功率大于40W
			if(judgeData.extPowerHeatData.chassis_power > WARNING_POWER){
				float powerScale = 0.0f;
				//不超过80W时
				if(judgeData.extPowerHeatData.chassis_power < POWER_LIMIT){
					powerScale = (POWER_LIMIT - judgeData.extPowerHeatData.chassis_power) / (POWER_LIMIT - WARNING_POWER);
				}
				//超过80W无电流加成
				else{
					powerScale = 0.0f;
				}
				totalCurrentLimit = ADD_POWER_CURRENT * powerScale + JUDGE_TOTAL_CURRENT_LIMIT;
			}
			//当前功率小于40W
			else{
				totalCurrentLimit = ADD_POWER_CURRENT + JUDGE_TOTAL_CURRENT_LIMIT;
			}
		}
	}
	totalCurrent = 0.0f;
	//计算底盘四个轮子总期望电流大小
	for(uint8_t i = 0; i < NUMBER_OF_WHEEL; i++){
		totalCurrent += fabs(chassisData.powerCurrent[i]);
	}
	//输出电流是否超过当前限制值，超出则按比例限幅
	if(totalCurrent > totalCurrentLimit){
		float current_scale = totalCurrentLimit / totalCurrent;
		chassisData.powerCurrent[0] *= current_scale * chassisData.scale[0];
		chassisData.powerCurrent[1] *= current_scale * chassisData.scale[1];
		chassisData.powerCurrent[2] *= current_scale * chassisData.scale[2];
		chassisData.powerCurrent[3] *= current_scale * chassisData.scale[3];
	}
}

//底盘初始化
void chassisInit(void){							
	chassisData.ctrlMode      = CHASSIS_RELAX;
	chassisData.lastCtrlMode  = CHASSIS_STOP;
	//FirstOrderFilter
	powerInitFirstOrderFilter();					
	initFirstOrderFilter();
	chassisData.posPID = pidInit(&parameter[CHASSIS_POS_P], &parameter[CHASSIS_POS_I], &parameter[CHASSIS_POS_D], &parameter[CHASSIS_POS_F],	\
												 	  &parameter[CHASSIS_POS_PM], &parameter[CHASSIS_POS_IM], &parameter[CHASSIS_POS_DM], &parameter[CHASSIS_POS_OM],	\
													  NULL, NULL, NULL, NULL); 
	for(uint8_t index = 0;index < NUMBER_OF_WHEEL;index++){						
#ifdef CHASSIS_SPEED_PID
		chassisData.speedPID[index] = pidInit(&parameter[CHASSIS_SPEED_P], &parameter[CHASSIS_SPEED_I], &parameter[CHASSIS_SPEED_D], &parameter[CHASSIS_SPEED_F],	\
																&parameter[CHASSIS_SPEED_PM], &parameter[CHASSIS_SPEED_IM], &parameter[CHASSIS_SPEED_DM], &parameter[CHASSIS_SPEED_OM],	\
																NULL, NULL, NULL, NULL);
#endif
#ifdef CHASSIS_SPEED_ADRC
		chassisData.speedADRC[index] = adrcInit(&parameter[ADRC_R],&parameter[ADRC_H],&parameter[ADRC_N0],&parameter[ADRC_BETA01],&parameter[ADRC_BETA02],	\
																						&parameter[ADRC_BETA03],&parameter[ADRC_B0],&parameter[ADRC_BETA0],&parameter[ADRC_BETA1],&parameter[ADRC_BETA2],	\
																						&parameter[ADRC_N1],&parameter[ADRC_C],&parameter[ADRC_ALPHA1],&parameter[ADRC_ALPHA2],&parameter[ADRC_ZETA],	\
																						&parameter[ADRC_B],&parameter[ADRC_OMAX]);
#endif

		if(robotConfigData.robotDeviceList & DEVICE_CURRENT){
			chassisData.currentPID[index] = pidInit(&parameter[POWER_LIMIT_P], &parameter[POWER_LIMIT_I], &parameter[POWER_LIMIT_D], &parameter[POWER_LIMIT_F],	\
																&parameter[POWER_LIMIT_PM], &parameter[POWER_LIMIT_IM], &parameter[POWER_LIMIT_DM], &parameter[POWER_LIMIT_OM],	\
																NULL, NULL, NULL, NULL);  
		}
	}
	chassisData.chasePID = pidInit(&parameter[CHASSIS_CHASE_P], &parameter[CHASSIS_CHASE_I], &parameter[CHASSIS_CHASE_D], &parameter[CHASSIS_CHASE_F],	\
													&parameter[CHASSIS_CHASE_PM], &parameter[CHASSIS_CHASE_IM], &parameter[CHASSIS_CHASE_DM], &parameter[CHASSIS_CHASE_OM],	\
													NULL, NULL, NULL, NULL);	
	chassisData.chaseSpeedPID = pidInit(&parameter[CHASSIS_RATE_P], &parameter[CHASSIS_RATE_I], &parameter[CHASSIS_RATE_D], &parameter[CHASSIS_RATE_F],	\
													&parameter[CHASSIS_RATE_PM], &parameter[CHASSIS_RATE_IM], &parameter[CHASSIS_RATE_DM], &parameter[CHASSIS_RATE_OM],	\
													NULL, NULL, NULL, NULL);	
	chassisData.speedLimit = 1.0f;
	chassisData.direction = 1.0f;
	chassisData.landingSpeedx = 0.0f;
	chassisData.landingSpeedy = 0.0f;
	chassisData.landingSpeedz = 0.0f;
	chassisData.changeHeadOrder = 0.0f;
	chassisData.changeHeadSchedule = 0.0f;
	chassisData.changeChassisSchedule = 0.0f;
	//存储校准好的中值
	chassisData.yawCenterSave = parameter[YAW_CENTER]; 						  
} 
