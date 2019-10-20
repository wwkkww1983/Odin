
#include "board.h"
#include "Driver_Motor.h"

Motor_TypeDef MotorData;

motorMixer_t mixerOctoX8[] = {
    { 1.0f, -1.0f,  1.0f, -1.0f },          // REAR_R      		后右
    { 1.0f, -1.0f, -1.0f,  1.0f },          // FRONT_R	      前右
    { 1.0f,  1.0f,  1.0f,  1.0f },          // REAR_L	        后左
    { 1.0f,  1.0f, -1.0f, -1.0f },          // FRONT_L     		前左
    { 1.0f, -1.0f,  1.0f,  1.0f },          // UNDER_REAR_R   下后右
    { 1.0f, -1.0f, -1.0f, -1.0f },          // UNDER_FRONT_R  下前右
    { 1.0f,  1.0f,  1.0f, -1.0f },          // UNDER_REAR_L   下后左
    { 1.0f,  1.0f, -1.0f,  1.0f },          // UNDER_FRONT_L  下前右
};

void axisCoefficientInit(void)
{
	uint8_t i;
	float d[MOTORS_NUM * MOTORS_AXIS] = {
		1.0f , -1.0f , 1.0f  , 1.0f  ,          //前左
		1.0f , 1.0f  , 1.0f  , -1.0f ,					//前右
		1.0f , 1.0f  , -1.0f , 1.0f  ,          //后左
		1.0f , -1.0f , -1.0f , -1.0f ,          //后右
		1.0f , -1.0f , 1.0f  , 1.0f  ,
		1.0f , 1.0f  , 1.0f  , -1.0f ,
		1.0f , 1.0f  , -1.0f , 1.0f  ,
		1.0f , -1.0f , -1.0f , -1.0f 
	};
	memset(MotorData.coefficient,0,MOTORS_NUM * MOTORS_AXIS *sizeof(int8_t));
	for(i = 0;i < MOTORS_NUM * MOTORS_AXIS;i++)
		MotorData.coefficient[i] = d[i];
}

void motorPWMOutput(Motor_TypeDef *motor,float throt,float rudd,float pitch,float roll)
{
	uint8_t i;
	float value;
	motor->throttle = constrainFloat(throt + DSHOT_MIN_THROTTLE,0.0f,DSHOT_3D_DEADBAND_LOW);						//限幅0~1000
	motor->rudd = rudd;
	motor->pitch = pitch;
	motor->roll = roll;
	for(i = 0;i < MOTORS_NUM;i++){
		uint8_t j;
		value = 0;
		for(j = 0;j < MOTORS_AXIS;j++)
			value += (int32_t)(*(&(motor->throttle) + j) * motor->coefficient[i*MOTORS_AXIS+j]);
#ifdef DSHOT_USE	
		motor->CH[i] = constrainFloat(value , DSHOT_MIN_THROTTLE , DSHOT_3D_DEADBAND_LOW);
#else		
		motor->CH[i] = constrainFloat(value ,0 ,MOTORS_SCALE) + INIT_DUTY;
#endif
	}
#ifdef DSHOT_USE
	dshotWriteMotors(motor->CH);
#else	
	ARM_1 = (uint16_t)motor->CH[RightFrontTop];
	ARM_2 = (uint16_t)motor->CH[LeftFrontTop];
	ARM_3 = (uint16_t)motor->CH[LeftRearTop];
	ARM_4 = (uint16_t)motor->CH[RightRearTop];
//	ARM_5 = motor->CH[RightFrontBottom];
//	ARM_6 = motor->CH[LeftFrontBottom];
//	ARM_7 = motor->CH[LeftRearBottom];
//	ARM_8 = motor->CH[RightRearBottom];
#endif
}

uint16_t StopNum=0;
void motorOff(void)
{
//	motorPWMOutput(&MotorData,0,0,0,0);
	motorPWMOutput(&MotorData,StopNum,0,0,0);
}

void motorOutputInit(void)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		MotorData.CH[i] = INIT_DUTY;
	}
	ARM_1 = MotorData.CH[RightFrontTop];
	ARM_2 = MotorData.CH[LeftFrontTop];
	ARM_3 = MotorData.CH[LeftRearTop];
	ARM_4 = MotorData.CH[RightRearTop];
	ARM_5 = MotorData.CH[RightFrontBottom];
	ARM_6 = MotorData.CH[LeftFrontBottom];
	ARM_7 = MotorData.CH[LeftRearBottom];
	ARM_8 = MotorData.CH[RightRearBottom];
}

void motorInit(void)
{
	axisCoefficientInit();											//初始化输出系数
#if (CONTROLLER == DAGGER)	
#ifdef DSHOT_USE
	pitchServoConfig();
	dshotMotorConfig(PWM_TYPE_DSHOT600);
#else
	BSP_TIM_PWM_Init(TIM3,MOTOR_PER,MOTOR_PRES,BSP_GPIOA6,BSP_GPIOA7,BSP_GPIOB0,BSP_GPIOB1);
	BSP_TIM_PWM_Init(TIM4,MOTOR_PER,MOTOR_PRES,BSP_GPIOD12,BSP_GPIOD13,BSP_GPIOD14,BSP_GPIOD15);
	motorOutputInit();
#endif
#else																					
	BSP_TIM_PWM_Init(TIM5,MOTOR_PER,MOTOR_PRES,NULL,NULL,BSP_GPIOA2,BSP_GPIOA3);
	BSP_TIM_PWM_Init(TIM1,MOTOR_PER,MOTOR_PRES_S,BSP_GPIOE9,BSP_GPIOE11,BSP_GPIOE13,BSP_GPIOE14);
	BSP_TIM_PWM_Init(TIM8,MOTOR_PER,MOTOR_PRES_S,NULL,NULL,BSP_GPIOC8,BSP_GPIOC9);
	motorOutputInit();
#endif
}














