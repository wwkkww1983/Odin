#include "FreeRTOS_board.h"
#include "indoor.h"
#include "Driver_Ultrasonic.h"
#include "Util.h"
#include "nav.h"
#include "Driver_Flow.h"

position_expect Expect;

_st_pos_pid  flowPOS;
void flowPOS_PIDInit(void)
{
	flowPOS.kp = 1.50f;
	flowPOS.kd = 0.0f;
	flowPOS.ki = 0.0f;
}

void flowPOSCtrl(float T,float exp_POS_x,float exp_POS_y,float current_POS_x,float current_POS_y)					 //输入的是cm
{
	Expect.distance.pitch.err = exp_POS_x - current_POS_x;
	Expect.distance.pitch.P = flowPOS.kp * Expect.distance.pitch.err;
	Expect.distance.pitch.I += flowPOS.ki * Expect.distance.pitch.err * T;
	Expect.distance.pitch.I = constrainFloat(Expect.distance.pitch.I,-100.0f,100.0f);										//允许1米的积分
	Expect.distance.pitch.D = flowPOS.kd * (Expect.distance.pitch.err - Expect.distance.pitch.old);
	Expect.distance.pitch.output = Expect.distance.pitch.P + Expect.distance.pitch.I + Expect.distance.pitch.D;
	Expect.distance.pitch.output = constrainFloat(Expect.distance.pitch.output,-400.0f,400.0f);								//最多允许4m/s
	navData.holdSpeedPitch = Expect.distance.pitch.output;
	Expect.distance.pitch.old = Expect.distance.pitch.err;
	
	Expect.distance.roll.err = exp_POS_y - current_POS_y;
	Expect.distance.roll.P = flowPOS.kp * Expect.distance.roll.err;
	Expect.distance.roll.I += flowPOS.ki * Expect.distance.roll.err * T;
	Expect.distance.roll.I = constrainFloat(Expect.distance.roll.I,-100.0f,100.0f);										//允许1米的积分
	Expect.distance.roll.D = flowPOS.kd * (Expect.distance.roll.err - Expect.distance.roll.old);
	Expect.distance.roll.output = Expect.distance.roll.P + Expect.distance.roll.I + Expect.distance.roll.D;
	Expect.distance.roll.output = constrainFloat(Expect.distance.roll.output,-300.0f,300.0f);								//最多允许3m/s
	navData.holdSpeedRoll = Expect.distance.roll.output;
	Expect.distance.roll.old = Expect.distance.roll.err;
}

_st_pos_pid flowSpeed;
void flowSpeed_PIDInit(void)
{
	flowSpeed.kp = 0.12f;
	flowSpeed.kd = 0.00f;
	flowSpeed.ki = 0.0010f;
}
void flowSpeedCtrl(float T,float exp_speed_x,float exp_speed_y,float current_speed_x,float current_speed_y)			//输入的是cm/s
{
//	if(navData.taskMode == TAKE_OFF)
//	{
//		flowSpeed.kp = 0.27f;									//起飞的时候比例增倍
//		flowSpeed.kd = 0.00f;
//		flowSpeed.ki = 0.0010f;
//	}
//	else
//	{
//		flowSpeed.kp = 0.12f;
//		flowSpeed.kd = 0.00f;
//		flowSpeed.ki = 0.0010f;
//	}
	Expect.speed.pitch.err = exp_speed_x - current_speed_x;
	Expect.speed.pitch.P = flowSpeed.kp * Expect.speed.pitch.err; 
	Expect.speed.pitch.I += flowSpeed.ki * Expect.speed.pitch.err * T;
	Expect.speed.pitch.I = constrainFloat(Expect.speed.pitch.I,-5.0f,5.0f);
//	Expect.speed.pitch.D = flowSpeed.kd * (0.6f * (-north_speed) + 0.4f * (Expect.speed.pitch.err - Expect.speed.pitch.old));
	Expect.speed.pitch.D = flowSpeed.kd * (Expect.speed.pitch.err - Expect.speed.pitch.old);
	Expect.speed.pitch.output = Expect.speed.pitch.P + Expect.speed.pitch.I + Expect.speed.pitch.D;
	navData.holdTiltPitch = -constrainFloat(Expect.speed.pitch.output,-25.0f,25.0f);
	Expect.speed.pitch.old = Expect.speed.pitch.err;
	
	Expect.speed.roll.err = exp_speed_y - current_speed_y;
	Expect.speed.roll.P = flowSpeed.kp * Expect.speed.roll.err; 
	Expect.speed.roll.I += flowSpeed.ki * Expect.speed.roll.err * T;
	Expect.speed.roll.I = constrainFloat(Expect.speed.roll.I,-5.0f,5.0f);
//	Expect.speed.roll.D = flowSpeed.kd * (0.6f * (-west_speed) + 0.4f * (Expect.speed.roll.err - Expect.speed.roll.old));
	Expect.speed.roll.D = flowSpeed.kd * (Expect.speed.roll.err - Expect.speed.roll.old);
	Expect.speed.roll.output = Expect.speed.roll.P + Expect.speed.roll.I + Expect.speed.roll.D;
	navData.holdTiltRoll = -constrainFloat(Expect.speed.roll.output,-25.0f,25.0f);	
	Expect.speed.roll.old = Expect.speed.roll.err;
}



void indoorPOSTask(void *Parameters)
{
	TickType_t xLastWakeTime;
	uint32_t loops = 0;
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime,20);
//		Ultra_Duty();
		loops++;
	}
}

BaseType_t INDOOREvent;
TaskHandle_t xHandleTaskIndoor;
void indoorInit(void)
{
	flowSpeed_PIDInit();
	flowPOS_PIDInit();
//	Driver_Ultrasonic_Init(INDOOR_USARTX,INDOOR_USARTX_RX_PIN,INDOOR_USARTX_TX_PIN, \
//									 INDOOR_USART_PreemptionPriority,INDOOR_USART_SubPriority);
	INDOOREvent = xTaskCreate(indoorPOSTask,"INDOOR",INDOOR_STACK_SIZE,NULL,INDOOR_PRIORITY,xHandleTaskIndoor);
}



