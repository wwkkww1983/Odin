#include "shoot.h"
#include "rc.h"
#include "config.h"
#include "Driver_RMMotor.h"
#include "judge.h"
#include "cansend.h"
#include "keyboard.h"
#include "control.h"
#include "pneumatic.h"

#define POKE_MOTER_SINGLE						 		(shootData.smallPokeSpeedRef = 6000)																													//17mm�����̵���ת��
#define POKE_MOTER_TRIPLE							  (shootData.smallPokeSpeedRef = 5800)																													//17mm������������ת��
#define POKE_MOTOR_BURST						  	(shootData.smallPokeSpeedRef = 5500)																													//17mm����������ת��
#define POKE_MOTOR_CLEAR								(shootData.smallPokeSpeedRef = 2400)																													//17mm�������˵�ת��
#define POKE_MOTER_OFF							    (shootData.smallPokeSpeedRef = 0)
#define FIRC_42mm_WHEEL_ON							(shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = shootData.fricSpeedSet_42mm)		//42mmĦ����ת��
#define FIRC_42mm_WHEEL_OFF							(shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0)
#define FIRC_17MM_WHEEL_SINGLE					(shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 1.000f*shootData.fricSpeedSet_17mm)  //17mmĦ����ת�٣�����
#define FIRC_17MM_WHEEL_TRIPLE					(shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0.800f*shootData.fricSpeedSet_17mm)  //������
#define FIRC_17MM_WHEEL_BURST						(shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0.7000f*shootData.fricSpeedSet_17mm) //����
#define FIRC_17MM_CLEAR_BULLET          (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0.450f*shootData.fricSpeedSet_17mm)  //�˵�
#define FIRC_17MM_WHEEL_OFF             (shootData.fricWheelSpeedRef[0] = shootData.fricWheelSpeedRef[1] = 0)
#define SHOOT_PWM_VARIABLE_QUANTITY			2

#define BULLET_READY										shootData.bulletMonitorFlag
#define GUN_READY												!pneumaticData.read1[0]
#define GUN_AWAIT												!pneumaticData.read1[1]
#define GIMBAL_READY										gimbalData.initFinishFlag

shootStruct_t shootData;

/* ����������� */
void shootDataReset(void){
	digitalLo(&shootData.airdropFlag);
	digitalLo(&shootData.fireFlag_42mm);
	digitalLo(&shootData.fireFlag_17mm);
	digitalLo(&shootData.shootManualNumber);
	digitalLo(&shootData.fireDataInit);
	digitalLo(&shootData.shootTrigger);
}
/* ������紫������ʼ�� */
static void bulletMonitorInit(void){		//����
	
}

#if SNAIL_MOTOR 
	/* Ħ���ָ���(10ms) */
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
					shootPwm = maxShootPwm;																	//��������ģʽ���26m/s����
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
		//����
		if(shootData.shootMode == MANUAL_SINGLE)
			FIRC_17MM_WHEEL_SINGLE;	
		//������
		else if(shootData.shootMode == MANUAL_CONTINUOUS)
			FIRC_17MM_WHEEL_TRIPLE;
		//����
		else if(shootData.shootMode == AUTO_CONTINUOUS){
			if(ROBOT == SENTRY_ID){
					FIRC_17MM_WHEEL_SINGLE;//17mmĦ����ת��Ĭ��ֵ
			}
			else if(ROBOT == SMALLGIMBAL_ID){
					FIRC_17MM_WHEEL_TRIPLE;
			}
			else{
					FIRC_17MM_WHEEL_BURST;
			}
		}
		//�˵�
		else if(shootData.clearBulletFlag)
			FIRC_17MM_CLEAR_BULLET;			
	}
	else{
		FIRC_17MM_WHEEL_OFF;
	}
}

/* �����̸���(2ms) */
static void shootpokeMoterUpdate( uint8_t FricONflag ){
	if(shootData.shootTrigger){
		switch(shootData.shootMode){
			//����
			case MANUAL_SINGLE: 
				POKE_MOTER_SINGLE;
				break;
			//������
			case MANUAL_CONTINUOUS: 
				POKE_MOTER_TRIPLE;
				break;
			//����
			case AUTO_CONTINUOUS:
				if(ROBOT == SENTRY_ID){
					POKE_MOTER_SINGLE;
				}
				else{ 
					if(shootData.clearBulletFlag)
						//�����˵�ģʽ
						POKE_MOTOR_CLEAR;
					else
						//��ǹģʽ
						POKE_MOTOR_BURST;					
				}				
				break;
		}
		controlDeviceConfirm(DEVICE_BuiltInPokeMotor,smallPokeSpeedRefChange);     //���������ò������豸
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

/****ȥ���������ģʽ�����δ˴��������뾯��*******
static uint16_t shootHurtCount( uint8_t speed,uint16_t leval ){          
	uint16_t excessHeat;						//���㳬��������
	uint16_t countResult;
	excessHeat = SHOOT_HEAT(speed) - shootData.shooterHeatRemain;
	if(excessHeat > 0){
		countResult = 2 * (judgeData.extGameRobotState.max_HP * 0.1f * excessHeat) / 250;	
	}
	return countResult;
}

static void shootGetJudgeBufferGain(void){
	shootData.OutputGain = 2.0f;																					//����˺��ӳ�
	shootData.shootHPRef = SMALL_BULLET_HARM * shootData.OutputGain;			//����Ԥ�ƴ�����˺�
}
************************************************/

/* �ֶ�����ģʽ */
static shootStatus_e shootModeManualSingle(void){
	if(SHOOT_HEAT(BULLET_SPEED_DEFAULT) < shootData.shooterHeatRemain){
		return SAFE;																							//�������״̬  ��ȫ
	}
	/***ȥ������ģʽ�����δ˴�*******
	else{
		shootGetJudgeBufferGain();																//��������ӳ�
		if(shootData.shootHPRef > shootHurtCount(BULLET_SPEED_DEFAULT,shootData.leval)){
			return WARNING;																					//�����Ѫ������Ԥ�ƴ�����˺��Ա�,��>ʧ����״���״̬����
		}
		else return DANGEROUS;																		//�ò���ʧ�������״̬Σ��
	}
	******************************/
	else return DANGEROUS;
}

/* �ӵ�ʣ�������� */
static void bulletRemainCount(void){
	//�䵯��������⵽�ӵ����ӵ�������50
	if(AIRDROP_FLAG){
		shootData.bulletRemain += 50;
		digitalLo(&AIRDROP_FLAG);
	}
	//û�м�⵽�ӵ�
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
		digitalHi(&shootData.shootTrigger);//������
}

/* ����ǹ��������Ϣ */
static void shooterHeatDataCount( void ){
	if(ROBOT == INFANTRY_ID){
		switch(judgeData.extGameRobotState.max_HP){
			case 200	 : shootData.leval = 1, shootData.shooterHeatMax = 240.0f, shootData.shooterHeatCoolingPs = 4.0f;		break;
			case 250	 : shootData.leval = 2, shootData.shooterHeatMax = 360.0f, shootData.shooterHeatCoolingPs = 6.0f;		break;
			case 300   : shootData.leval = 3, shootData.shooterHeatMax = 480.0f, shootData.shooterHeatCoolingPs = 8.0f;		break;
			case 65535 : shootData.leval = 3, shootData.shooterHeatMax = 480.0f, shootData.shooterHeatCoolingPs = 480.0f;		break;	 //����Ѫ��ģʽ
			default    : shootData.leval = 3, shootData.shooterHeatMax = 480.0f, shootData.shooterHeatCoolingPs = 480.0f;		break;
		}
		shootData.shooterHeatRemain = shootData.shooterHeatMax - shootData.shooterHeat;			 //��ȡǹ������ʣ��ֵ�Ա���㷢���Σ����
	}
	else if(ROBOT == UAV_ID){
		//���л�����û�еȼ����ƣ�û����������
		shootData.leval = 3;
		shootData.shooterHeatMax = 65535;
		shootData.shooterHeatCoolingPs = 65535;
		shootData.shooterHeatRemain = shootData.shooterHeatMax - shootData.shooterHeat;			 //��ȡǹ������ʣ��ֵ�Ա���㷢���Σ����
	}
	else if(ROBOT == SENTRY_ID){
		//�ڱ������˲��ֵȼ�
		shootData.leval = 3;
		shootData.shooterHeatMax = 480.0f;
		shootData.shooterHeatCoolingPs = 16.0f;
		shootData.shooterHeatRemain = shootData.shooterHeatMax - shootData.shooterHeat;			 //��ȡǹ������ʣ��ֵ�Ա���㷢���Σ����
	}
}

/* ����Ӧ�����������������״̬ */
static void shootTimesCount(void){
	shootData.shootStatus = shootModeManualSingle();
	shootData.xLastWakeTime = xTaskGetTickCount();
}

/* ��ʱ��⺯�� */
static void shootOverTimeCheck(void){
	switch (shootData.shootMode){
		case MANUAL_SINGLE :{
			if((xTaskGetTickCount() - shootData.xLastWakeTime) > 100)  //���0.2s��û���굥������ֹͣ���
				digitalLo(&shootData.shootTrigger);																 
		}break;
		case MANUAL_CONTINUOUS :{
			if((xTaskGetTickCount() - shootData.xLastWakeTime) > 300)  //���0.6s��û��������������ֹͣ���
				digitalLo(&shootData.shootTrigger);																 
		}break;		
		case AUTO_CONTINUOUS :		break;
	}
}

/* ����������� */
static void shootProtect(void){
	if(shootData.shootMode == AUTO_CONTINUOUS){		//��ǹģʽ
		if(shootData.shootStatusMode != DANGEROUS){	//��Σ��ģʽ֮��
			if(shootData.shooterHeatRemain < SHOOT_HEAT(BULLET_SPEED_DEFAULT)){//Ԥ��һ���ӵ�������
				digitalLo(&shootData.shootTrigger); 		//��ǹģʽ��Ϊ��Ԥ�����������µĳ�������Ѫ��Ԥ��һ���ӵ�������(��ȫģʽ)
			}		
		}		
	}
	if(shootData.shootStatus > shootData.shootStatusMode)
		digitalLo(&shootData.shootTrigger);	 				//��ȫ�Բ��������������ֹͣ���
	if((shootData.shooterHeatMax - judgeData.extPowerHeatData.shooter_heat0) < SHOOT_HEAT(BULLET_SPEED_DEFAULT) \
		&& shootData.shootStatusMode == SAFE){
			digitalLo(&shootData.shootTrigger);
	}
}

/* ǹ���������ƺ��� */
void shooterHeatControl(void){
  shootData.shooterHeat = shootData.shooterHeat < 0 ? 0 : shootData.shooterHeat;
	shootData.shooterHeat = shootData.shooterHeat > 2.0f * shootData.shooterHeatMax ? 2.0f * shootData.shooterHeatMax : shootData.shooterHeat;	
}

/* ǹ��������ȴ���� */
void shooterHeatCooling(void){
	if(!(controlData.loops % 50)){								//100ms����һ��  10HZ
		shootData.shooterHeat -= shootData.shooterHeatCoolingPs;
	}
	shooterHeatControl();
}

/* ǹ���������Ӻ��� */
void shooterHeatIncreasing(void){
	if((judgeData.extShootData.bullet_speed != shootData.shootSpeedLast)&&(judgeData.extShootData.bullet_type == 1)){					//������ٲ���ͬ����֤�������һ���ӵ�
		shootData.shooterHeat += SHOOT_HEAT(judgeData.extShootData.bullet_speed); 	//��⵽�ӵ�������Բ���ϵͳ���ص�������Ϣ����						
		shooterHeatControl();
		if(shootData.bulletRemain > 0)
			digitalDecline(&shootData.bulletRemain);//ʣ���ӵ�����ʣ�෢����-1
	}	
	shootData.shootSpeedLast = judgeData.extShootData.bullet_speed;
}

/* ǹ�������������� */
void shooterHeatAbjust(void){
	/*����ϵͳ����ǹ���������ݺ�ͽ��������ǹ������ */
	shootData.shooterJudgeHeat_17mm = judgeData.extPowerHeatData.shooter_heat0;
	if(shootData.shooterJudgeHeat_17mm != shootData.shooterJudgeLastHeat_17mm){					
		shootData.shooterJudgeLastHeat_17mm = shootData.shooterJudgeHeat_17mm;
	}
	if(shootData.shooterJudgeHeat_17mm >= shootData.shooterHeat){
		shootData.shooterHeat = shootData.shooterJudgeHeat_17mm;
	}
}

/* ��ȡ�����̵������ */
void pokeMoterReadData( CanRxMsg *can_rx_msg, motorCanDataRecv_t *pokeMoterData ){
	pokeMoterData -> rawangle = ( (uint16_t)can_rx_msg -> Data[0] << 8 ) | can_rx_msg -> Data[1];
	pokeMoterData -> speed = ( (int16_t)can_rx_msg -> Data[2] << 8 ) | can_rx_msg -> Data[3];
	pokeMoterData -> currunt = ( (int16_t)can_rx_msg -> Data[4] << 8 ) | can_rx_msg -> Data[5];
}

/* ������������ǿ�ƽ��㺯�� */
void shootManualUpdate(void){
	static uint16_t waitTime;
	if(!shootData.fireDataInit){					//������η����ʼ��û������	���������ʼ������
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
		digitalClan(&shootData.loops);			//���ѭ������������������
	}
	if(!(shootData.loops % waitTime)){
		digitalHi(&shootData.fireDataInit);	//���ߵ��γ�ʼ����־����ֹ�ظ�����
		if(shootData.shootManualNumber > 0){				//���ʣ�෢����������0
			digitalHi(&shootData.shootTrigger);				//������
			if(shootData.shootMode != AUTO_CONTINUOUS){			
				digitalDecline(&shootData.shootManualNumber);		//���������������ȴ����������һ
			}
			else if((!KB_17MM_SHOOT_CONTINUOUS)&&(ROBOT != SENTRY_ID)&&(ROBOT != SMALLGIMBAL_ID)){
				digitalLo(&shootData.fireDataInit);
				digitalClan(&FIRE_FLAG);	//�忪���־	
				//����ͣס������
				digitalLo(&shootData.shootTrigger);		
			}
		}
		else{
			digitalLo(&shootData.fireDataInit);
			digitalClan(&FIRE_FLAG);	//�忪���־										
		}
	}
}

//�����ת���
static void pokeStallCheck(void){
	static uint16_t stallCount = 0;
	static TickType_t xLastWakeTime = 0;
	
	if(shootData.pokeStall == MOTOR_FOWARD){
		if(pokeData.currunt > 9000){   //�����תת�ص�������9000����500���룬˵����ת
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
		if(xTaskGetTickCount() - xLastWakeTime > 50){//��ת100����
			 shootData.pokeStall = MOTOR_FOWARD;                            //Ȼ����ת
		}
	}
}
/**************************************************************************************************/
/* ǹ���������Ӻ��� */
void shooterHeatIncreasingTank(void){
	if(judgeData.extShootData.bullet_type == 2 && shootData.shootSpeedLastTank != judgeData.extShootData.bullet_speed)				//����һ�������ٺ���һ�������Ա�
	{
		shootData.shooterHeatTank += 100; 	//��⵽�ӵ�������Բ���ϵͳ���ص�������Ϣ����						
		shootData.shooterHeatTank = shootData.shooterHeatTank < 0 ? 0 : shootData.shooterHeatTank;
		shootData.shooterHeatTank = shootData.shooterHeatTank > 2.0f * shootData.shooterHeatMaxTank ? 2.0f * shootData.shooterHeatMaxTank : shootData.shooterHeatTank;	
		
	  shootData.shootSpeedLastTank = judgeData.extShootData.bullet_speed;	
	}
}
/* ����ǹ��������Ϣ */
static void shooterHeatDataCountTank( void ){
	//�ж�Ӣ�۵�ǰ��ǹ���������ޣ��ȼ���ǹ������ÿ����ȴֵ
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
			}break;	//����Ѫ��ģʽ
			default    : 	break;
		}
		shootData.shooterHeatRemainTank = shootData.shooterHeatMaxTank - shootData.shooterHeatTank;			 //��ȡǹ������ʣ��ֵ�Ա���㷢���Σ����	
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
			}break;	//����Ѫ��ģʽ
			default    : 	break;
		}
	}
	shootData.shooterHeatRemain = shootData.shooterHeatMax - shootData.shooterHeat;			 //��ȡǹ������ʣ��ֵ�Ա���㷢���Σ����	
}

//Ӣ��������������
void shootProtectTank(void){
	if((shootData.shooterHeatRemainTank > protectHeartTank) || shootData.suicideFireFlag){
		digitalHi(&shootData.shootTriggerTank);	//�������ӵ��������ᳬ���޻���ɱ����ģʽʱ������	
	}
	else{
		digitalLo(&shootData.shootTriggerTank);																//��ֹ����
	}
}

void TankShootHeatAbjust(void){
	shootData.shooterHeatTank = judgeData.extPowerHeatData.shooter_heat1;
}

static float turntableMove(float temp){
	return temp;
}

/*******************Ӣ��ֻ�е���ģʽ*************************************
static void BulletsNumberSet(void){
	if(shootData.fireFlag_42mm){
		switch(shootData.shootMode)	//�жϵ�ǰ���ģʽ���ж����״̬
		{
			case MANUAL_SINGLE :			shootData.shootTimesTank = 1;	break;//�ֶ�����ģʽ
		 	case MANUAL_CONTINUOUS :  shootData.shootTimesTank = 3;	break;//������ģʽ		
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

/* Ӣ�ۿ��� */
static void heroFire(void){
	if(shootData.shootTriggerTank && shootData.bulletReady){
			if(shootData.fireFlag_42mm){			
				digitalHi(&P_HERO_FIRE);															//����
				digitalLo(&shootData.fireFlag_42mm);
				shootData.loadStep = 3;
			}
	}
	else{
		digitalLo(&P_HERO_FIRE);
		digitalLo(&shootData.fireFlag_42mm);
	}
}

/* Ӣ���Զ�װ�� */
static void heroAutoLoad(void){										//ʩ����
	static bool LAST_BULLET_READY;
	static bool LAST_GUN_READY;
	static uint16_t LAST_HEAT;
	
	static uint8_t fireWakeTime;									//����ʱ����
	static uint8_t relaxWakeTime;									//ж��ʱ����
	static uint8_t cleanWakeTime;									//������ʱ����
	static uint8_t cleanTimes;										//����������
	static uint16_t loadWakeTime;									//װ��ʱ����
	static uint16_t turnStallCount = 0;
	static TickType_t turnWakeTime = 0;
	
	
#ifdef USE_PNEUMATIC
	switch(shootData.loadStep){
		//�������̳�ʼ��
		case 0:{
			digitalLo(&P_HERO_LOAD);														//����
			digitalLo(&shootData.bulletReady);									//��ֹ����
			if(shootData.ballisticClean){
				if(robotMode == MODE_RC || robotMode == MODE_KM){
					if(shootData.bulletExist || (BULLET_READY && GUN_AWAIT)){
						digitalHi(&P_HERO_LOAD);													//����
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
		//������������
		case 1:{
			digitalLo(&P_HERO_LOAD);														//����
			if((!BULLET_READY) && GUN_AWAIT){									//�������޵���ʱװ��
				shootData.turnSpeed = 1500;
				digitalIncreasing(&loadWakeTime);
//				if(loadWakeTime > 2500){													//5000msû��⵽����������˵�������þ�
//					shootData.loadStep = 4;
//					digitalLo(&shootData.ballisticFill);
//					digitalClan(&loadWakeTime);
//				}
			}
			else if((!LAST_BULLET_READY) && BULLET_READY && GUN_AWAIT){					//�����ҵ��赽λʱֹͣװ��
				shootData.turnSpeed = 0;
				digitalHi(&P_HERO_LOAD);													//����
				digitalIncreasing(&shootData.loadStep);
			}
		} break;
		//װ����ɺ���
		case 2:{
			if((!LAST_GUN_READY) && GUN_READY){
				digitalHi(&shootData.bulletRelax);					//����ж��
				digitalHi(&shootData.bulletReady);					//�������ʱ������
				digitalHi(&shootData.bulletExist);					//�������
			}
		} break;
		//�������
		case 3:{
			if(shootData.shooterHeatTank > LAST_HEAT){
				digitalLo(&P_HERO_FIRE);														//�رտ��ŷ�
				digitalLo(&shootData.bulletReady);									//��ֹ����
				digitalLo(&shootData.bulletExist);									//���費����
				shootData.loadStep = 1;
			}
			else{
				digitalIncreasing(&fireWakeTime);
				if(fireWakeTime > 150){																//300ms��ʱ����
					digitalLo(&P_HERO_FIRE);														//�رտ��ŷ�
					digitalLo(&shootData.bulletReady);									//��ֹ����
					digitalLo(&shootData.bulletExist);									//���費����
					shootData.loadStep = 1;
					digitalClan(&fireWakeTime);
				}
			}
		} break;
		//��ҩ�þ��ȴ�����
		case 4:{
			if(shootData.ballisticFill){
				shootData.loadStep = 1;
			}
			else{
				shootData.turnSpeed = 0;
			}
		}  break;
		//����ƿ������������λǰִֻ��һ��
		case 5:{
			if(cleanTimes < 6){													//�ظ�����
				shootData.turnSpeed = -500;
				if(cleanWakeTime > 250){									//500mm��תһ������
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
				digitalHi(&shootData.ballisticClean);			//�������
				digitalClan(&shootData.loadStep);					//����������������
			}
		}  break;
	}
#endif
	
#ifdef USE_FRICWHEEL
	
#endif
	//װ����ɺ���ж��
	if(shootData.bulletRelax){
		shootData.turnSpeed = -1000;
		digitalIncreasing(&relaxWakeTime);
		if(relaxWakeTime > 50){
			shootData.turnSpeed = 0;
			digitalLo(&shootData.bulletRelax);
			digitalClan(&relaxWakeTime);
		}
	}
	
	//��Ͳ��ת����
	if(shootData.turntableStall == MOTOR_FOWARD){
		if(turntableData.currunt > 7500){																		//�����תת�ص�������7500����500���룬˵����ת
			digitalIncreasing(&turnStallCount);
			if(turnStallCount > 250){
				if(turntableData.currunt > 7500){
					turnWakeTime = xTaskGetTickCount();
					digitalClan(&turnStallCount);
					shootData.turntableStall = MOTOR_REVERSAL;
					if(shootData.loadStep == 5){																	//��������������ͣת
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
		if(xTaskGetTickCount() - turnWakeTime > 100){								//��ת200����
			 shootData.turntableStall = MOTOR_FOWARD;											//Ȼ����ת
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

/* ����������� */
void shootUpdate(void){
	shooterHeatDataCount();		//����ʣ��ǹ������	
	if(FIRE_FLAG){						//������յ������źţ����ݵ�ǰ�����ģʽ�ж�Ӧ������ӵ����������㰲ȫ��
		shootManualUpdate();
		shootTimesCount();	
	}	  
	shootOverTimeCheck();			//�����ʱ���,�����յ������źŵ�ʱ��ȷ����������Ƿ�ʱ�������ʱ�������������
	shootProtect();						//�������,���ݷ���İ�ȫ��ȷ���Ƿ������������
	bulletRemainCount();			//����ʣ���ӵ�,���ж��Ƿ�ֹͣ������
#if SNAIL_MOTOR 
	if(!(controlData.loops % 5)){
		shootFrictiongearUpdate(shootData.fricMotorFlag);	//Ħ���ֵ�����10msһ��
	}
#else
	if(ROBOT == INFANTRY_ID || ROBOT == UAV_ID || ROBOT == SENTRY_ID)
		shootSmallFrictiongearUpdate(shootData.fricMotorFlag);//17mmĦ�������
#endif
	pokeStallCheck();         //��������ת���
	shootpokeMoterUpdate(shootData.fricMotorFlag);//�����̵�����,���տ���ָ��
	shooterHeatCooling();			//������ȴ
	digitalIncreasing(&shootData.loops);	
	
	shooterHeatDataCountTank();		//����ʣ��ǹ������
	if(ROBOT == SMALLGIMBAL_ID){
		shooterHeatAbjust();
		shooterHeatIncreasing(); //Ӣ�۸���,�ڴ˽�����������
	}
	shootProtectTank();
	heroFire();                //�򵯺���
	heroAutoLoad();						//Ӣ���Զ�װ��
	shootData.turntableRef = turntableMove(shootData.turnSpeed);  //�߶����Ͳ����ٶ�����
	if(ROBOT == TANK_ID)
		shootBigFrictiongearUpdate(shootData.fricMotorFlag);//42mmĦ�������
  if(ROBOT == SMALLGIMBAL_ID)	
		shootSmallFrictiongearUpdate(shootData.fricMotorFlag);//17mmĦ�������

	shootData.time[0] = getClockCount();
	shootData.intervalTime = (float)(shootData.time[0] - shootData.time[1]);
	shootData.time[1] = shootData.time[0];
	//Ħ����PID����
	shootData.fricWheelSpeedOut[0] = pidUpdate( shootData.fricWheelSpeedPID[0], shootData.fricWheelSpeedRef[0], fricWheelData[0].speed, shootData.intervalTime);		
	shootData.fricWheelSpeedOut[1] = -pidUpdate( shootData.fricWheelSpeedPID[1], shootData.fricWheelSpeedRef[1], -fricWheelData[1].speed, shootData.intervalTime);			
	shootData.turntableOut = pidUpdate( shootData.turntablePID, shootData.turntableRef*shootData.turntableStall , turntableData.speed, shootData.intervalTime);
	//������PID����

	if(robotMode == MODE_RELAX||robotMode == MODE_STOP){																				//������ͷſ���Ȩģʽ��ֹͣģʽ
	  shootData.fricWheelSpeedOut[0] = shootData.fricWheelSpeedOut[1] = 0;											//Ħ���ֲ������������Ҳ����
		shootData.smallPokeSpeedOut = shootData.turntableOut = shootData.bigPokeSpeedOut = 0;
	}		
}

/* ���������ʼ�� */
void shootInit(void){		
#if SNAIL_MOTOR 	
	//������ˢ���,Ƶ��400Hz																								
	BSP_TIM_PWM_Init( TIM9, 2500-1, 168-1, BSP_GPIOE5, BSP_GPIOE6, NULL,NULL );	
	FricMotor_L = FricMotor_R = 1000;
#endif
	bulletMonitorInit();																										//������紫������ʼ��
	
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
	shootData.shootMode = MANUAL_SINGLE;			//Ĭ�ϵ���
	shootData.shootModeTank = MANUAL_SINGLE;
	shootData.shootStatusMode = SAFE;  //Ĭ�ϲ����� ����Ħ���� û���յ�������־ ������ȫģʽ
	if(ROBOT == SENTRY_ID){
		shootData.shootMode = AUTO_CONTINUOUS;	
		shootData.fricSpeedSet_17mm = 7700;//17mmĦ����ת��Ĭ��ֵ
	}
	else{
		shootData.fricSpeedSet_17mm = 6750;//17mmĦ����ת��Ĭ��ֵ
	}
	shootData.fricSpeedSet_42mm = 5950;																			//42mmĦ����ת��Ĭ��ֵ
}
