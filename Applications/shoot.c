#include "shoot.h"
#include "rc.h"
#include "config.h"
#include "Driver_RMMotor.h"
#include "judge.h"
#include "cansend.h"
#include "keyboard.h"
#include "control.h"
#include "pneumatic.h"

#define POKE_MOTER_SINGLE						 		(shootData.smallPokeSpeedRef = 6000)																													//17mm拨弹盘单发转速
#define POKE_MOTER_TRIPLE							  (shootData.smallPokeSpeedRef = 5800)																													//17mm拨弹盘三连发转速
#define POKE_MOTOR_BURST						  	(shootData.smallPokeSpeedRef = 5500)																													//17mm拨弹盘连发转速
#define POKE_MOTOR_CLEAR								(shootData.smallPokeSpeedRef = 2400)																													//17mm拨弹盘退弹转速
#define POKE_MOTER_OFF							    (shootData.smallPokeSpeedRef = 0)
#define FIRC_42mm_WHEEL_ON							(shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = shootData.fricSpeedSet_42mm)		//42mm摩擦轮转速
#define FIRC_42mm_WHEEL_OFF							(shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0)
#define FIRC_17MM_WHEEL_SINGLE					(shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 1.000f*shootData.fricSpeedSet_17mm)  //17mm摩擦轮转速，单发
#define FIRC_17MM_WHEEL_TRIPLE					(shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0.800f*shootData.fricSpeedSet_17mm)  //三连发
#define FIRC_17MM_WHEEL_BURST						(shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0.7000f*shootData.fricSpeedSet_17mm) //连发
#define FIRC_17MM_CLEAR_BULLET          (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0.450f*shootData.fricSpeedSet_17mm)  //退弹
#define FIRC_17MM_WHEEL_OFF             (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0)
#define SHOOT_PWM_VARIABLE_QUANTITY			2

#define BULLET_READY										shootData.bulletMonitorFlag
#define GUN_READY												!pneumaticData.read1[0]
#define GUN_AWAIT												!pneumaticData.read1[1]
#define GIMBAL_READY										gimbalData.initFinishFlag

shootStruct_t shootData;

/* 射击参数重置 */
void shootDataReset(void){
	digitalLo(&shootData.airdropFlag);
	digitalLo(&shootData.fireFlag_42mm);
	digitalLo(&shootData.fireFlag_17mm);
	digitalLo(&shootData.shootManualNumber);
	digitalLo(&shootData.fireDataInit);
	digitalLo(&shootData.shootTrigger);
}
/* 数弹光电传感器初始化 */
static void bulletMonitorInit(void){		//暂无
	
}

#if SNAIL_MOTOR 
	/* 摩擦轮更新(10ms) */
	static void shootFrictiongearUpdate( uint8_t FricONflag ){
		static uint16_t shootPwm = 1000;
		static uint16_t maxShootPwm;

		if(shootData.shootMode == MANUAL_SINGLE){
			maxShootPwm = parameter[SHOOT_HIGH_PWM];
		}
		else{
			maxShootPwm = parameter[SHOOT_LOW_PWM];
		}
		if(FricONflag){
			if(shootPwm > maxShootPwm){
				shootPwm -= SHOOT_PWM_VARIABLE_QUANTITY;
				if(shootPwm < maxShootPwm)
					shootPwm = maxShootPwm;
			}
			if(shootPwm < maxShootPwm){
				shootPwm += SHOOT_PWM_VARIABLE_QUANTITY;
				if( shootPwm > maxShootPwm ) 
					shootPwm = maxShootPwm;																	//单发连发模式大概26m/s射速
			}
			FricMotor_L = FricMotor_R = shootPwm;
		}
		else{
			FricMotor_L = FricMotor_R = 1000;
		}
	}
#endif
static void shootSmallFrictiongearUpdate(uint8_t FricONflag){
	if(FricONflag){
		//单发
		if(shootData.shootMode == MANUAL_SINGLE)
			FIRC_17MM_WHEEL_SINGLE;	
		//三连发
		else if(shootData.shootMode == MANUAL_CONTINUOUS)
			FIRC_17MM_WHEEL_TRIPLE;
		//连发
		else if(shootData.shootMode == AUTO_CONTINUOUS){
			if(ROBOT == SENTRY_ID){
					FIRC_17MM_WHEEL_SINGLE;//17mm摩擦轮转速默认值
			}
			else if(ROBOT == SMALLGIMBAL_ID){
					FIRC_17MM_WHEEL_TRIPLE;
			}
			else{
					FIRC_17MM_WHEEL_BURST;
			}
		}
		//退弹
		else if(shootData.clearBulletFlag)
			FIRC_17MM_CLEAR_BULLET;			
	}
	else{
		FIRC_17MM_WHEEL_OFF;
	}
}

/* 拨弹盘更新(2ms) */
static void shootpokeMoterUpdate( uint8_t FricONflag ){
	if(shootData.shootTrigger){
		switch(shootData.shootMode){
			//单发
			case MANUAL_SINGLE: 
				POKE_MOTER_SINGLE;
				break;
			//三连发
			case MANUAL_CONTINUOUS: 
				POKE_MOTER_TRIPLE;
				break;
			//连发
			case AUTO_CONTINUOUS:
				if(ROBOT == SENTRY_ID){
					POKE_MOTER_SINGLE;
				}
				else{ 
					if(shootData.clearBulletFlag)
						//场间退弹模式
						POKE_MOTOR_CLEAR;
					else
						//机枪模式
						POKE_MOTOR_BURST;					
				}				
				break;
		}
		controlDeviceConfirm(DEVICE_BuiltInPokeMotor,smallPokeSpeedRefChange);     //步兵有内置拨弹盘设备
	}
	else{
	  POKE_MOTER_OFF;
	}
	if( !FricONflag ){
	  POKE_MOTER_OFF;
	}			
	shootData.smallPokeSpeedOut = pidUpdate( shootData.speedPID, shootData.smallPokeSpeedRef*shootData.pokeStall, pokeData.speed, 0.002f );			
}

void  smallPokeSpeedRefChange(void){
	shootData.smallPokeSpeedRef = -shootData.smallPokeSpeedRef;
}

/****去除射击警告模式后屏蔽此处消除编译警告*******
static uint16_t shootHurtCount( uint8_t speed,uint16_t leval ){          
	uint16_t excessHeat;						//计算超出的热量
	uint16_t countResult;
	excessHeat = SHOOT_HEAT(speed) - shootData.shooterHeatRemain;
	if(excessHeat > 0){
		countResult = 2 * (judgeData.extGameRobotState.max_HP * 0.1f * excessHeat) / 250;	
	}
	return countResult;
}

static void shootGetJudgeBufferGain(void){
	shootData.OutputGain = 2.0f;																					//神符伤害加成
	shootData.shootHPRef = SMALL_BULLET_HARM * shootData.OutputGain;			//计算预计打出的伤害
}
************************************************/

/* 手动单发模式 */
static shootStatus_e shootModeManualSingle(void){
	if(SHOOT_HEAT(BULLET_SPEED_DEFAULT) < shootData.shooterHeatRemain){
		return SAFE;																							//返回射击状态  安全
	}
	/***去掉警告模式后屏蔽此处*******
	else{
		shootGetJudgeBufferGain();																//计算神符加成
		if(shootData.shootHPRef > shootHurtCount(BULLET_SPEED_DEFAULT,shootData.leval)){
			return WARNING;																					//计算掉血量并和预计打出的伤害对比,得>失返回状射击状态警告
		}
		else return DANGEROUS;																		//得不偿失返回射击状态危险
	}
	******************************/
	else return DANGEROUS;
}

/* 子弹剩余数计算 */
static void bulletRemainCount(void){
	//落弹传感器检测到子弹，子弹数量加50
	if(AIRDROP_FLAG){
		shootData.bulletRemain += 50;
		digitalLo(&AIRDROP_FLAG);
	}
	//没有检测到子弹
	if(BULLET_MONITOR_FLAG){																		
		if(shootData.shootTrigger){
			digitalHi(&shootData.monitorFlag);
			shootData.shootWaitTime = xTaskGetTickCount();
		}
	}
	else{																												
		if(!BULLET_MONITOR_FLAG && shootData.monitorFlag){
			digitalLo(&shootData.shootTrigger);
			digitalLo(&shootData.monitorFlag);
		}
	}
	if(shootData.clearBulletFlag && (shootData.shooterHeatRemain > 40.0f))
		digitalHi(&shootData.shootTrigger);//允许发射
}

/* 计算枪口热量信息 */
static void shooterHeatDataCount( void ){
	if(ROBOT == INFANTRY_ID){
		switch(judgeData.extGameRobotState.max_HP){
			case 200	 : shootData.leval = 1, shootData.shooterHeatMax = 240.0f, shootData.shooterHeatCoolingPs = 4.0f;		break;
			case 250	 : shootData.leval = 2, shootData.shooterHeatMax = 360.0f, shootData.shooterHeatCoolingPs = 6.0f;		break;
			case 300   : shootData.leval = 3, shootData.shooterHeatMax = 480.0f, shootData.shooterHeatCoolingPs = 8.0f;		break;
			case 65535 : shootData.leval = 3, shootData.shooterHeatMax = 480.0f, shootData.shooterHeatCoolingPs = 480.0f;		break;	 //无限血量模式
			default    : shootData.leval = 3, shootData.shooterHeatMax = 480.0f, shootData.shooterHeatCoolingPs = 480.0f;		break;
		}
		shootData.shooterHeatRemain = shootData.shooterHeatMax - shootData.shooterHeat;			 //获取枪口热量剩余值以便计算发射的危险性
	}
	else if(ROBOT == UAV_ID){
		//空中机器人没有等级机制，没有热量限制
		shootData.leval = 3;
		shootData.shooterHeatMax = 65535;
		shootData.shooterHeatCoolingPs = 65535;
		shootData.shooterHeatRemain = shootData.shooterHeatMax - shootData.shooterHeat;			 //获取枪口热量剩余值以便计算发射的危险性
	}
	else if(ROBOT == SENTRY_ID){
		//哨兵机器人不分等级
		shootData.leval = 3;
		shootData.shooterHeatMax = 480.0f;
		shootData.shooterHeatCoolingPs = 16.0f;
		shootData.shooterHeatRemain = shootData.shooterHeatMax - shootData.shooterHeat;			 //获取枪口热量剩余值以便计算发射的危险性
	}
}

/* 计算应打出的射击次数和射击状态 */
static void shootTimesCount(void){
	shootData.shootStatus = shootModeManualSingle();
	shootData.xLastWakeTime = xTaskGetTickCount();
}

/* 超时检测函数 */
static void shootOverTimeCheck(void){
	switch (shootData.shootMode){
		case MANUAL_SINGLE :{
			if((xTaskGetTickCount() - shootData.xLastWakeTime) > 100)  //如果0.2s还没打完单发，就停止射击
				digitalLo(&shootData.shootTrigger);																 
		}break;
		case MANUAL_CONTINUOUS :{
			if((xTaskGetTickCount() - shootData.xLastWakeTime) > 300)  //如果0.6s还没打完三连发，就停止射击
				digitalLo(&shootData.shootTrigger);																 
		}break;		
		case AUTO_CONTINUOUS :		break;
	}
}

/* 射击保护函数 */
static void shootProtect(void){
	if(shootData.shootMode == AUTO_CONTINUOUS){		//机枪模式
		if(shootData.shootStatusMode != DANGEROUS){	//除危险模式之外
			if(shootData.shooterHeatRemain < SHOOT_HEAT(BULLET_SPEED_DEFAULT)){//预留一发子弹的热量
				digitalLo(&shootData.shootTrigger); 		//机枪模式下为了预防卡弹现象导致的超热量掉血，预留一发子弹的热量(安全模式)
			}		
		}		
	}
	if(shootData.shootStatus > shootData.shootStatusMode)
		digitalLo(&shootData.shootTrigger);	 				//安全性不满足射击条件就停止射击
	if((shootData.shooterHeatMax - judgeData.extPowerHeatData.shooter_heat0) < SHOOT_HEAT(BULLET_SPEED_DEFAULT) \
		&& shootData.shootStatusMode == SAFE){
			digitalLo(&shootData.shootTrigger);
	}
}

/* 枪口热量控制函数 */
void shooterHeatControl(void){
  shootData.shooterHeat = shootData.shooterHeat < 0 ? 0 : shootData.shooterHeat;
	shootData.shooterHeat = shootData.shooterHeat > 2.0f * shootData.shooterHeatMax ? 2.0f * shootData.shooterHeatMax : shootData.shooterHeat;	
}

/* 枪口热量冷却函数 */
void shooterHeatCooling(void){
	if(!(controlData.loops % 50)){								//100ms结算一次  10HZ
		shootData.shooterHeat -= shootData.shooterHeatCoolingPs;
	}
	shooterHeatControl();
}

/* 枪口热量增加函数 */
void shooterHeatIncreasing(void){
	if((judgeData.extShootData.bullet_speed != shootData.shootSpeedLast)&&(judgeData.extShootData.bullet_type == 1)){					//如果射速不相同，则证明打出了一发子弹
		shootData.shooterHeat += SHOOT_HEAT(judgeData.extShootData.bullet_speed); 	//检测到子弹射出就以裁判系统返回的射速信息计算						
		shooterHeatControl();
		if(shootData.bulletRemain > 0)
			digitalDecline(&shootData.bulletRemain);//剩余子弹数和剩余发射数-1
	}	
	shootData.shootSpeedLast = judgeData.extShootData.bullet_speed;
}

/* 枪口热量矫正函数 */
void shooterHeatAbjust(void){
	/*裁判系统返回枪口热量数据后就矫正计算的枪口热量 */
	shootData.shooterJudgeHeat_17mm = judgeData.extPowerHeatData.shooter_heat0;
	if(shootData.shooterJudgeHeat_17mm != shootData.shooterJudgeLastHeat_17mm){					
		shootData.shooterJudgeLastHeat_17mm = shootData.shooterJudgeHeat_17mm;
	}
	if(shootData.shooterJudgeHeat_17mm >= shootData.shooterHeat){
		shootData.shooterHeat = shootData.shooterJudgeHeat_17mm;
	}
}

/* 读取拨弹盘电机数据 */
void pokeMoterReadData( CanRxMsg *can_rx_msg, motorCanDataRecv_t *pokeMoterData ){
	pokeMoterData -> rawangle = ( (uint16_t)can_rx_msg -> Data[0] << 8 ) | can_rx_msg -> Data[1];
	pokeMoterData -> speed = ( (int16_t)can_rx_msg -> Data[2] << 8 ) | can_rx_msg -> Data[3];
	pokeMoterData -> currunt = ( (int16_t)can_rx_msg -> Data[4] << 8 ) | can_rx_msg -> Data[5];
}

/* 单发，三连发强制结算函数 */
void shootManualUpdate(void){
	static uint16_t waitTime;
	if(!shootData.fireDataInit){					//如果单次发射初始化没有拉高	射击参数初始化拉低
		switch(shootData.shootMode){
			case MANUAL_SINGLE :{
				shootData.shootManualNumber = 1;
				waitTime = 25;
				break;
			}
			case MANUAL_CONTINUOUS :{
				shootData.shootManualNumber = 3;
				waitTime = 50;
				break;
			}
			case AUTO_CONTINUOUS :{
				shootData.shootManualNumber = 1;
				waitTime = 1;
				break;
			}
		}
		digitalClan(&shootData.loops);			//清除循环数，可以立即发射
	}
	if(!(shootData.loops % waitTime)){
		digitalHi(&shootData.fireDataInit);	//拉高单次初始化标志，防止重复击发
		if(shootData.shootManualNumber > 0){				//如果剩余发射数量大于0
			digitalHi(&shootData.shootTrigger);				//允许发射
			if(shootData.shootMode != AUTO_CONTINUOUS){			
				digitalDecline(&shootData.shootManualNumber);		//如果不是连发，则等待发射次数减一
			}
			else if((!KB_17MM_SHOOT_CONTINUOUS)&&(ROBOT != SENTRY_ID)&&(ROBOT != SMALLGIMBAL_ID)){
				digitalLo(&shootData.fireDataInit);
				digitalClan(&FIRE_FLAG);	//清开火标志	
				//立马停住拨弹盘
				digitalLo(&shootData.shootTrigger);		
			}
		}
		else{
			digitalLo(&shootData.fireDataInit);
			digitalClan(&FIRE_FLAG);	//清开火标志										
		}
	}
}

//拨叉堵转检测
static void pokeStallCheck(void){
	static uint16_t stallCount = 0;
	static TickType_t xLastWakeTime = 0;
	
	if(shootData.pokeStall == MOTOR_FOWARD){
		if(pokeData.currunt > 9000){   //如果堵转转矩电流大于9000持续500毫秒，说明堵转
			digitalIncreasing(&stallCount);
			if(stallCount > 40){
				if(pokeData.currunt > 9000){
					xLastWakeTime = xTaskGetTickCount();
					digitalClan(&stallCount);
					shootData.pokeStall = MOTOR_REVERSAL;
				}
				else{
				  digitalClan(&stallCount);
				}					
			}
		}
		else{
		  digitalClan(&stallCount);
		}
	}
	else if(shootData.pokeStall == MOTOR_REVERSAL){
		if(xTaskGetTickCount() - xLastWakeTime > 50){//反转100毫秒
			 shootData.pokeStall = MOTOR_FOWARD;                            //然后正转
		}
	}
}
/**************************************************************************************************/
/* 枪口热量增加函数 */
void shooterHeatIncreasingTank(void){
	if(judgeData.extShootData.bullet_type == 2 && shootData.shootSpeedLastTank != judgeData.extShootData.bullet_speed)				//用这一发的射速和上一发的做对比
	{
		shootData.shooterHeatTank += 100; 	//检测到子弹射出就以裁判系统返回的射速信息计算						
		shootData.shooterHeatTank = shootData.shooterHeatTank < 0 ? 0 : shootData.shooterHeatTank;
		shootData.shooterHeatTank = shootData.shooterHeatTank > 2.0f * shootData.shooterHeatMaxTank ? 2.0f * shootData.shooterHeatMaxTank : shootData.shooterHeatTank;	
		
	  shootData.shootSpeedLastTank = judgeData.extShootData.bullet_speed;	
	}
}
/* 计算枪口热量信息 */
static void shooterHeatDataCountTank( void ){
	//判断英雄当前的枪口热量上限，等级，枪口热量每秒冷却值
	if(ROBOT == TANK_ID /*&& parameter[LOCAL_ID] == 0x0101*/){
		switch(judgeData.extGameRobotState.max_HP){
			case 300 : {
				shootData.leval = 1; shootData.shooterHeatMaxTank = 200.0f , shootData.shooterHeatCoolingPsTank = 2.0f;	                     
			}break;
			case 500 : {
				shootData.leval = 2; shootData.shooterHeatMaxTank = 300.0f, shootData.shooterHeatCoolingPsTank = 4.0f;                   			
			}break;
			case 700 : {
				shootData.leval = 3; shootData.shooterHeatMaxTank = 400.0f, shootData.shooterHeatCoolingPsTank = 6.0f;	                      		
			}break;
			case 65535 : {
				shootData.leval = 3; shootData.shooterHeatMaxTank = 400.0f, shootData.shooterHeatCoolingPsTank = 400.0f;	                    	
			}break;	//无限血量模式
			default    : 	break;
		}
		shootData.shooterHeatRemainTank = shootData.shooterHeatMaxTank - shootData.shooterHeatTank;			 //获取枪口热量剩余值以便计算发射的危险性	
	}
	else if(ROBOT == SMALLGIMBAL_ID){
		switch(judgeData.extGameRobotState.max_HP){
			case 300 : {
				shootData.leval = 1; shootData.shooterHeatMax = 240.0f, shootData.shooterHeatCoolingPs  = 4.0f;	                     
			}break;
			case 500 : {
				shootData.leval = 2; shootData.shooterHeatMax = 360.0f, shootData.shooterHeatCoolingPs  = 6.0f;                   			
			}break;
			case 700 : {
				shootData.leval = 3; shootData.shooterHeatMax = 480.0f, shootData.shooterHeatCoolingPs  = 8.0f;	                      		
			}break;
			case 65535 : {
				shootData.leval = 3; shootData.shooterHeatMax = 480.0f, shootData.shooterHeatCoolingPs  = 480.0f;	                    	
			}break;	//无限血量模式
			default    : 	break;
		}
	}
	shootData.shooterHeatRemain = shootData.shooterHeatMax - shootData.shooterHeat;			 //获取枪口热量剩余值以便计算发射的危险性	
}

//英雄热量保护函数
void shootProtectTank(void){
	if((shootData.shooterHeatRemainTank > protectHeartTank) || shootData.suicideFireFlag){
		digitalHi(&shootData.shootTriggerTank);	//开火增加的热量不会超上限或自杀开火模式时允许发射	
	}
	else{
		digitalLo(&shootData.shootTriggerTank);																//禁止发射
	}
}

void TankShootHeatAbjust(void){
	shootData.shooterHeatTank = judgeData.extPowerHeatData.shooter_heat1;
}

static float turntableMove(float temp){
	return temp;
}

/*******************英雄只有单发模式*************************************
static void BulletsNumberSet(void){
	if(shootData.fireFlag_42mm){
		switch(shootData.shootMode)	//判断当前射击模式，判断射击状态
		{
			case MANUAL_SINGLE :			shootData.shootTimesTank = 1;	break;//手动单发模式
		 	case MANUAL_CONTINUOUS :  shootData.shootTimesTank = 3;	break;//三连发模式		
			case AUTO_CONTINUOUS :    shootData.shootTimesTank = 1; break;
			default :								 	            									break;
		}
		digitalLo(&shootData.fireFlag_42mm);
	}
	if(shootData.shootMode != AUTO_CONTINUOUS){
		if(shootData.shootTimesTank>shootData.shooterTimesLimitTank){
			shootData.shootTimesTank = shootData.shooterTimesLimitTank;
		}
		if(shootData.shooterTimesOutTank >= shootData.shooterTimesLimitTank){
			digitalClan(&shootData.shootTimesTank);
			digitalLo(&shootData.shootTriggerTank);
		}
	}
}
****************************************************************************/

/* 英雄开火 */
static void heroFire(void){
	if(shootData.shootTriggerTank && shootData.bulletReady){
			if(shootData.fireFlag_42mm){			
				digitalHi(&P_HERO_FIRE);															//开火
				digitalLo(&shootData.fireFlag_42mm);
				shootData.loadStep = 3;
			}
	}
	else{
		digitalLo(&P_HERO_FIRE);
		digitalLo(&shootData.fireFlag_42mm);
	}
}

/* 英雄自动装填 */
static void heroAutoLoad(void){										//施工中
	static bool LAST_BULLET_READY;
	static bool LAST_GUN_READY;
	static uint16_t LAST_HEAT;
	
	static uint8_t fireWakeTime;									//开火时间监测
	static uint8_t relaxWakeTime;									//卸力时间监测
	static uint8_t cleanWakeTime;									//清理弹道时间监测
	static uint8_t cleanTimes;										//清理弹道次数
	static uint16_t loadWakeTime;									//装填时间监测
	static uint16_t turnStallCount = 0;
	static TickType_t turnWakeTime = 0;
	
	
#ifdef USE_PNEUMATIC
	switch(shootData.loadStep){
		//发射流程初始化
		case 0:{
			digitalLo(&P_HERO_LOAD);														//退镗
			digitalLo(&shootData.bulletReady);									//禁止发射
			if(shootData.ballisticClean){
				if(robotMode == MODE_RC || robotMode == MODE_KM){
					if(shootData.bulletExist || (BULLET_READY && GUN_AWAIT)){
						digitalHi(&P_HERO_LOAD);													//上膛
						shootData.loadStep = 2;
					}
					else
						digitalIncreasing(&shootData.loadStep);
				}
			}
			else{
				shootData.loadStep = 5;
			}
		} break;
		//弹丸推入炮膛
		case 1:{
			digitalLo(&P_HERO_LOAD);														//退镗
			if((!BULLET_READY) && GUN_AWAIT){									//退镗且无弹丸时装填
				shootData.turnSpeed = 1500;
				digitalIncreasing(&loadWakeTime);
//				if(loadWakeTime > 2500){													//5000ms没检测到弹丸入镗则说明弹丸用尽
//					shootData.loadStep = 4;
//					digitalLo(&shootData.ballisticFill);
//					digitalClan(&loadWakeTime);
//				}
			}
			else if((!LAST_BULLET_READY) && BULLET_READY && GUN_AWAIT){					//退镗且弹丸到位时停止装填
				shootData.turnSpeed = 0;
				digitalHi(&P_HERO_LOAD);													//上膛
				digitalIncreasing(&shootData.loadStep);
			}
		} break;
		//装填完成后工作
		case 2:{
			if((!LAST_GUN_READY) && GUN_READY){
				digitalHi(&shootData.bulletRelax);					//弹链卸力
				digitalHi(&shootData.bulletReady);					//上膛完成时允许发射
				digitalHi(&shootData.bulletExist);					//弹丸存在
			}
		} break;
		//发射后处理
		case 3:{
			if(shootData.shooterHeatTank > LAST_HEAT){
				digitalLo(&P_HERO_FIRE);														//关闭快排阀
				digitalLo(&shootData.bulletReady);									//禁止发射
				digitalLo(&shootData.bulletExist);									//弹丸不存在
				shootData.loadStep = 1;
			}
			else{
				digitalIncreasing(&fireWakeTime);
				if(fireWakeTime > 150){																//300ms超时结算
					digitalLo(&P_HERO_FIRE);														//关闭快排阀
					digitalLo(&shootData.bulletReady);									//禁止发射
					digitalLo(&shootData.bulletExist);									//弹丸不存在
					shootData.loadStep = 1;
					digitalClan(&fireWakeTime);
				}
			}
		} break;
		//弹药用尽等待补给
		case 4:{
			if(shootData.ballisticFill){
				shootData.loadStep = 1;
			}
			else{
				shootData.turnSpeed = 0;
			}
		}  break;
		//换气瓶后清理弹道，复位前只执行一次
		case 5:{
			if(cleanTimes < 6){													//重复三次
				shootData.turnSpeed = -500;
				if(cleanWakeTime > 250){									//500mm反转一次气缸
					digitalTogg(&P_HERO_LOAD);
					digitalClan(&cleanWakeTime);
					digitalIncreasing(&cleanTimes);
				}
				else{
					digitalIncreasing(&cleanWakeTime);
				}
			}
			else{
				shootData.turnSpeed = 0;
				digitalHi(&shootData.ballisticClean);			//清理完成
				digitalClan(&shootData.loadStep);					//重置气动发射流程
			}
		}  break;
	}
#endif
	
#ifdef USE_FRICWHEEL
	
#endif
	//装填完成后弹链卸力
	if(shootData.bulletRelax){
		shootData.turnSpeed = -1000;
		digitalIncreasing(&relaxWakeTime);
		if(relaxWakeTime > 50){
			shootData.turnSpeed = 0;
			digitalLo(&shootData.bulletRelax);
			digitalClan(&relaxWakeTime);
		}
	}
	
	//滚筒堵转保护
	if(shootData.turntableStall == MOTOR_FOWARD){
		if(turntableData.currunt > 7500){																		//如果堵转转矩电流大于7500持续500毫秒，说明堵转
			digitalIncreasing(&turnStallCount);
			if(turnStallCount > 250){
				if(turntableData.currunt > 7500){
					turnWakeTime = xTaskGetTickCount();
					digitalClan(&turnStallCount);
					shootData.turntableStall = MOTOR_REVERSAL;
					if(shootData.loadStep == 5){																	//若正在清理弹道则停转
						shootData.turnSpeed = 0;
					}
				}
				else{
				  digitalClan(&turnStallCount);
				}					
			}
		}
		else{
		  digitalClan(&turnStallCount);
		}
	}
	else if(shootData.turntableStall == MOTOR_REVERSAL){
		if(xTaskGetTickCount() - turnWakeTime > 100){								//反转200毫秒
			 shootData.turntableStall = MOTOR_FOWARD;											//然后正转
		}
	}
	
	LAST_BULLET_READY = BULLET_READY;
	LAST_GUN_READY = GUN_READY;
	LAST_HEAT = shootData.shooterHeatTank;
}

static void shootBigFrictiongearUpdate( uint8_t FricONflag ){
  if(FricONflag){
	  FIRC_42mm_WHEEL_ON;
	}
	else{
		digitalLo(&shootData.shootTriggerTank);
		digitalLo(&shootData.pokeFlag);
		digitalLo(&shootData.suicideFireFlag);
		digitalClan(&shootData.shootTimesTank);
		digitalClan(&shootData.shootOutNumber);
	  FIRC_42mm_WHEEL_OFF;
	}
}

/* 射击函数更新 */
void shootUpdate(void){
	shooterHeatDataCount();		//计算剩余枪口热量	
	if(FIRE_FLAG){						//如果接收到开火信号，根据当前的射击模式判断应发射的子弹数，并计算安全性
		shootManualUpdate();
		shootTimesCount();	
	}	  
	shootOverTimeCheck();			//射击超时检测,根据收到开火信号的时间确定射击命令是否超时，如果超时则清零射击次数
	shootProtect();						//射击保护,根据发射的安全性确定是否清零射击次数
	bulletRemainCount();			//计算剩余子弹,并判断是否停止拨弹盘
#if SNAIL_MOTOR 
	if(!(controlData.loops % 5)){
		shootFrictiongearUpdate(shootData.fricMotorFlag);	//摩擦轮电机输出10ms一次
	}
#else
	if(ROBOT == INFANTRY_ID || ROBOT == UAV_ID || ROBOT == SENTRY_ID)
		shootSmallFrictiongearUpdate(shootData.fricMotorFlag);//17mm摩擦轮输出
#endif
	pokeStallCheck();         //拨叉电机堵转检测
	shootpokeMoterUpdate(shootData.fricMotorFlag);//拨弹盘电机输出,最终开火指令
	shooterHeatCooling();			//热量冷却
	digitalIncreasing(&shootData.loops);	
	
	shooterHeatDataCountTank();		//计算剩余枪口热量
	if(ROBOT == SMALLGIMBAL_ID){
		shooterHeatAbjust();
		shooterHeatIncreasing(); //英雄副控,在此进行热量计算
	}
	shootProtectTank();
	heroFire();                //打弹函数
	heroAutoLoad();						//英雄自动装填
	shootData.turntableRef = turntableMove(shootData.turnSpeed);  //高尔夫滚筒电机速度设置
	if(ROBOT == TANK_ID)
		shootBigFrictiongearUpdate(shootData.fricMotorFlag);//42mm摩擦轮输出
  if(ROBOT == SMALLGIMBAL_ID)	
		shootSmallFrictiongearUpdate(shootData.fricMotorFlag);//17mm摩擦轮输出

	shootData.time[0] = getClockCount();
	shootData.intervalTime = (float)(shootData.time[0] - shootData.time[1]);
	shootData.time[1] = shootData.time[0];
	//摩擦轮PID计算
	shootData.fricWheelSpeedOut[0] = pidUpdate( shootData.fricWheelSpeedPID[0], shootData.fricWheelSpeedRef[0], fricWheelData[0].speed, shootData.intervalTime);		
	shootData.fricWheelSpeedOut[1] = -pidUpdate( shootData.fricWheelSpeedPID[1], shootData.fricWheelSpeedRef[1], -fricWheelData[1].speed, shootData.intervalTime);			
	shootData.turntableOut = pidUpdate( shootData.turntablePID, shootData.turntableRef*shootData.turntableStall , turntableData.speed, shootData.intervalTime);
	//拨弹盘PID计算

	if(robotMode == MODE_RELAX||robotMode == MODE_STOP){																				//如果是释放控制权模式或停止模式
	  shootData.fricWheelSpeedOut[0] = shootData.fricWheelSpeedOut[1] = 0;											//摩擦轮不开，拨弹电机也不开
		shootData.smallPokeSpeedOut = shootData.turntableOut = shootData.bigPokeSpeedOut = 0;
	}		
}

/* 发射机构初始化 */
void shootInit(void){		
#if SNAIL_MOTOR 	
	//两个无刷电机,频率400Hz																								
	BSP_TIM_PWM_Init( TIM9, 2500-1, 168-1, BSP_GPIOE5, BSP_GPIOE6, NULL,NULL );	
	FricMotor_L = FricMotor_R = 1000;
#endif
	bulletMonitorInit();																										//数弹光电传感器初始化
	
	shootData.speedPID = pidInit(&parameter[SHOOT_DIAL_S_P], &parameter[SHOOT_DIAL_S_I], &parameter[SHOOT_DIAL_S_D], &parameter[SHOOT_DIAL_S_F],	\
													&parameter[SHOOT_DIAL_S_PM], &parameter[SHOOT_DIAL_S_IM], &parameter[SHOOT_DIAL_S_DM], &parameter[SHOOT_DIAL_S_OM],	\
													NULL, NULL, NULL, NULL);
	shootData.fricWheelSpeedPID[0]  = pidInit(&parameter[SHOOT_FRIC_P], &parameter[SHOOT_FRIC_I], &parameter[SHOOT_FRIC_D], &parameter[SHOOT_FRIC_F],	\
													      &parameter[SHOOT_FRIC_PM], &parameter[SHOOT_FRIC_IM], &parameter[SHOOT_FRIC_DM], &parameter[SHOOT_FRIC_OM],	\
													      NULL, NULL, NULL, NULL); 
	shootData.fricWheelSpeedPID[1]  = pidInit(&parameter[SHOOT_FRIC_P], &parameter[SHOOT_FRIC_I], &parameter[SHOOT_FRIC_D], &parameter[SHOOT_FRIC_F],	\
													      &parameter[SHOOT_FRIC_PM], &parameter[SHOOT_FRIC_IM], &parameter[SHOOT_FRIC_DM], &parameter[SHOOT_FRIC_OM],	\
													      NULL, NULL, NULL, NULL); 
	shootData.turntablePID = 	pidInit(&parameter[SHOOT_DIAL_L_P], &parameter[SHOOT_DIAL_L_I], &parameter[SHOOT_DIAL_L_D], &parameter[SHOOT_DIAL_L_F],	\
													      &parameter[SHOOT_DIAL_L_PM], &parameter[SHOOT_DIAL_L_IM], &parameter[SHOOT_DIAL_L_DM], &parameter[SHOOT_DIAL_L_OM],	\
													      NULL, NULL, NULL, NULL);	
	
	digitalClan(&shootData.bulletRemain);
	shootDataReset();
	shootData.turntableStall = MOTOR_FOWARD;
  shootData.pokeStall = MOTOR_FOWARD;
	shootData.shootMode = MANUAL_SINGLE;			//默认单发
	shootData.shootModeTank = MANUAL_SINGLE;
	shootData.shootStatusMode = SAFE;  //默认不开火 不开摩擦轮 没有收到补给标志 单发安全模式
	if(ROBOT == SENTRY_ID){
		shootData.shootMode = AUTO_CONTINUOUS;	
		shootData.fricSpeedSet_17mm = 7700;//17mm摩擦轮转速默认值
	}
	else{
		shootData.fricSpeedSet_17mm = 6750;//17mm摩擦轮转速默认值
	}
	shootData.fricSpeedSet_42mm = 5950;																			//42mm摩擦轮转速默认值
}
