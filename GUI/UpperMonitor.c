#include "UpperMonitor.h"

upperStruct_t upperData;
BSP_USART_TypeDef upperMonitor;
float data;

//#define UART7_SEND  //串口7发送（波特率为115200）
#define USBVCP_SEND  //虚拟串口发送（波特率为500000）

void upperMonitorUpdateTask(void *Parameters)   //上位机实时数据更新
  {
		static unsigned char Seq2=0;
		TickType_t xLastWakeTime = xTaskGetTickCount();
		while(1){
			vTaskDelayUntil(&xLastWakeTime,UPPERMONITOR_PERIOD);
			Array_UART7_TX[0] = 0xAD;
	    Array_UART7_TX[1] = 0x2C;
	    Array_UART7_TX[2] = 0x00;
	    Array_UART7_TX[3] = Seq2;
	    Array_UART7_TX[5] = 0x00;
	    Array_UART7_TX[6] = 0x01;
			if(upperData.flag_pitch)
			{
				//pitch
				Array_UART7_TX[7] = 6000/256;
				Array_UART7_TX[8] = 6000%256;
				Array_UART7_TX[9] = 4000/256;
				Array_UART7_TX[10] = 4000%256;
				Array_UART7_TX[11] = 1000/256;
				Array_UART7_TX[12] = 1000%256;
				Array_UART7_TX[13] = 800/256;
				Array_UART7_TX[14] = 800%256;
			}
			if(upperData.flag_yaw)
			{
				//yaw
				Array_UART7_TX[7] = (gimbalData.yawAngleRef)/256;
				Array_UART7_TX[8] = ((int)(gimbalData.yawAngleRef))%256;
				Array_UART7_TX[9] = (gimbalData.yawAngleFbd)/256;
				Array_UART7_TX[10] = ((int)(gimbalData.yawAngleFbd))%256;
				Array_UART7_TX[11] = (gimbalData.yawSpeedRef)/256;
				Array_UART7_TX[12] = ((int)(gimbalData.yawSpeedRef))%256;
				Array_UART7_TX[13] = (gimbalData.yawSpeedFbd)/256;
				Array_UART7_TX[14] = ((int)(gimbalData.yawSpeedFbd))%256;
			}
			if(upperData.flag_chassis)
			{
				//底盘
				Array_UART7_TX[7] = (chassisData.speedRef[0])/256;
				Array_UART7_TX[8] = ((int)(chassisData.speedRef[0]))%256;
				Array_UART7_TX[9] = (chassisData.speedFbd[0])/256;
				Array_UART7_TX[10] = ((int)(chassisData.speedFbd[0]))%256;
			}
			if(upperData.flag_chase)
			{
				//跟随
				Array_UART7_TX[7] = (chassisData.chaseRef*100)/256;
				Array_UART7_TX[8] = ((int)(chassisData.chaseRef*100))%256;
				Array_UART7_TX[9] = (chassisData.chaseFbd*100)/256;
				Array_UART7_TX[10] = ((int)(chassisData.chaseFbd*100))%256;
			}
			if(upperData.flag_power)
			{
				//功率
				Array_UART7_TX[7] = (judgeData.extPowerHeatData.chassis_power)/256;
				Array_UART7_TX[8] = ((int)(judgeData.extPowerHeatData.chassis_power))%256;

			}
			if(upperData.flag_firewheel)
			{
				//摩擦轮
				Array_UART7_TX[7] = (shootData.fricWheelSpeedRef[0])/256;
				Array_UART7_TX[8] = ((int)(shootData.fricWheelSpeedRef[0]))%256;
				Array_UART7_TX[9] = (shootData.fricWheelSpeedOut[0])/256;
				Array_UART7_TX[10] = ((int)(shootData.fricWheelSpeedOut[0]))%256;
			}
			if(upperData.flag_poke)
			{
				//拨弹盘
				Array_UART7_TX[7] = (shootData.bigPokeSpeedRef)/256;
				Array_UART7_TX[8] = ((int)(shootData.bigPokeSpeedRef))%256;
				Array_UART7_TX[9] = (shootData.bigPokeSpeedOut)/256;
				Array_UART7_TX[10] = ((int)(shootData.bigPokeSpeedOut))%256;
			}
			
			if(upperData.flag_read)
				{
					Array_UART7_TX[11] = parameter[ROBOT_TYPE];
			    Array_UART7_TX[12] = parameter[LOCAL_ID];
				  Array_UART7_TX[13] = parameter[PITCH_TYPE];
					Array_UART7_TX[14] = parameter[YAW_TYPE];
			    Array_UART7_TX[15] = parameter[PITCH_FIX];
				  Array_UART7_TX[16] = parameter[YAW_FIX];
					Array_UART7_TX[17] = parameter[PITCH_TURN];
			    Array_UART7_TX[18] = parameter[YAW_TURN];
					digitalIncreasing(&upperData.read_schedules);
					if((upperData.read_schedules%5)){digitalLo(&upperData.flag_read);}
				}
			Append_CRC8_Check_Sum(Array_UART7_TX,5);
			#ifdef UART7_SEND
			upperMonitor.USARTx = UART7;
			if(upperData.flag_read)
			BSP_USART_SendData(&upperMonitor,Array_UART7_TX,18);
			else
			BSP_USART_SendData(&upperMonitor,Array_UART7_TX,15);
			#endif
			#ifdef USBVCP_SEND
			if(upperData.flag_read)
			usbVCP_SendBuffer(Array_UART7_TX,18);
			else
			usbVCP_SendBuffer(Array_UART7_TX,15);
			#endif
			Seq2++;
			UpperMonitorReceiveData(upperData.Receive_data);
		  digitalIncreasing(&wirelessData.loops);
		}
  }

	//接收数据处理
void UpperMonitorReceiveData(u8 *array)
	{
		int i = 0;
		data = array[1]*256+array[2]+(float)array[3]/200; //接收数据
	  if(array[0] == 0x61)  //步兵
	 {
		 robotConfigData.typeOfRobot = INFANTRY_ID;
	
	 }
	 else if(array[0] == 0x62)  //英雄
	 {
		 robotConfigData.typeOfRobot = TANK_ID;

	 }
	  else if(array[0] == 0x63)  //工程
	 {
		 robotConfigData.typeOfRobot = AUXILIARY_ID;

	 }
	  else if(array[0] == 0x64)  //哨兵
	 {
		 robotConfigData.typeOfRobot = SENTRY_ID;

	 }
	  else if(array[0] == 0x65)  //飞机
	 {
		 robotConfigData.typeOfRobot = UAV_ID;
	
	 }
	 else if(array[0] == 0x67)
	 {
	  digitalLo(&upperData.flag_yaw);
		digitalLo(&upperData.flag_power);
		digitalLo(&upperData.flag_chassis);
		digitalLo(&upperData.flag_chase);
		digitalLo(&upperData.flag_poke);
		digitalLo(&upperData.flag_firewheel);
		digitalHi(&upperData.flag_pitch);
	 }
	 else if(array[0] == 0x68)
	 {
	  digitalLo(&upperData.flag_pitch);
		digitalLo(&upperData.flag_power);
		digitalLo(&upperData.flag_chassis);
		digitalLo(&upperData.flag_chase);
		digitalLo(&upperData.flag_poke);
		digitalLo(&upperData.flag_firewheel);
		digitalHi(&upperData.flag_yaw);
	 }
	 else if(array[0] == 0x69)
	 {
	  digitalLo(&upperData.flag_pitch);
		digitalLo(&upperData.flag_power);
		digitalLo(&upperData.flag_yaw);
		digitalLo(&upperData.flag_chase);
		digitalLo(&upperData.flag_poke);
		digitalLo(&upperData.flag_firewheel);
		digitalHi(&upperData.flag_chassis);
	 }
	 else if(array[0] == 0x6A)
	 {
	  digitalLo(&upperData.flag_pitch);
		digitalLo(&upperData.flag_power);
		digitalLo(&upperData.flag_chassis);
		digitalLo(&upperData.flag_yaw);
		digitalLo(&upperData.flag_poke);
		digitalLo(&upperData.flag_firewheel);
		digitalHi(&upperData.flag_chase);
	 }
	 else if(array[0] == 0x6B)
	 {
	  digitalLo(&upperData.flag_pitch);
		digitalLo(&upperData.flag_yaw);
		digitalLo(&upperData.flag_chassis);
		digitalLo(&upperData.flag_chase);
		digitalLo(&upperData.flag_poke);
		digitalLo(&upperData.flag_firewheel);
		digitalHi(&upperData.flag_power);
	 }
	 else if(array[0] == 0x6C)
	 {
	  digitalLo(&upperData.flag_pitch);
		digitalLo(&upperData.flag_power);
		digitalLo(&upperData.flag_chassis);
		digitalLo(&upperData.flag_chase);
		digitalLo(&upperData.flag_poke);
		digitalLo(&upperData.flag_yaw);
		digitalHi(&upperData.flag_firewheel);
	 }
	 else if(array[0] == 0x6D)
	 {
	  digitalLo(&upperData.flag_pitch);
		digitalLo(&upperData.flag_power);
		digitalLo(&upperData.flag_chassis);
		digitalLo(&upperData.flag_chase);
		digitalLo(&upperData.flag_yaw);
		digitalLo(&upperData.flag_firewheel);
		digitalHi(&upperData.flag_poke);
	 }
	 else if(array[0] == 0x71)      //读取参数
	 {
	   digitalHi(&upperData.flag_read);
	 }
	 else if(array[0] == 0x72)
	 {
	   parameter[LOCAL_ID] = 0x101; //主控
	 }
	 else if(array[0] == 0x73)
	 {
	   parameter[LOCAL_ID] = 0x102; //副控
	 }
	 else if(array[0] == 0x74)
	 {
	   parameter[PITCH_TYPE] = MOTOR_6623; //pitch轴6623电机
	 }
	 else if(array[0] == 0x75)
	 {
	   parameter[PITCH_TYPE] = MOTOR_6020; //6020电机
	 }
	 else if(array[0] == 0x76)
	 {
	   parameter[PITCH_TYPE] = MOTOR_3510; //3510电机
	 }
	 else if(array[0] == 0x77)
	 {
	   parameter[YAW_TYPE] = MOTOR_6623; //yaw轴电机6623电机
	 }
	 else if(array[0] == 0x78)
	 {
	   parameter[YAW_TYPE] = MOTOR_6020; //6020电机
	 }
	 else if(array[0] == 0x79)
	 {
	   parameter[YAW_TYPE] = MOTOR_3510; //3510电机
	 }
	 else if(array[0] == 0x7A)
	 {
	   parameter[PITCH_FIX] = CCW_INSTALL; //pitch轴安装方向 左边
	 }
	 else if(array[0] == 0x7B)
	 {
	   parameter[PITCH_FIX] = CW_INSTALL; //右边
	 }
	 else if(array[0] == 0x7C)
	 {
	   parameter[YAW_FIX] = CCW_INSTALL; //yaw轴安装方向 反向
	 }
	 else if(array[0] == 0x7D)
	 {
	   parameter[YAW_FIX] = CW_INSTALL; //正向
	 }
	 else if(array[0] == 0x7E)
	 {
	   parameter[PITCH_TURN] = CCW_TURN; //pitch轴旋转方向 逆时针
	 }
	 else if(array[0] == 0x7F)
	 {
	   parameter[PITCH_TURN] = CW_TURN; //顺时针
	 }
	 else if(array[0] == 0x80)
	 {
	   parameter[YAW_TURN] = CCW_TURN; //yaw轴旋转方向 逆时针
	 }
	 else if(array[0] == 0x81)
	 {
	   parameter[YAW_TURN] = CW_TURN; //顺时针
	 }
	 else if(array[0] == 0xFF)  //保存参数
	 {
		 digitalHi(&supervisorData.flashSave);	
	 }
	 for(i = 0;i <= 3;i++)
	 upperData.Receive_data[i] = 0;
	}
	
void GetupperMonitorReceiveData(u8 *array)  //接收上位机发过来的数据
	{
		if(array[4] == 0XAD)
		{
		  upperData.Receive_data[0] = array[0];
	    upperData.Receive_data[1] = array[1];
	   	upperData.Receive_data[2] = array[2];
	    upperData.Receive_data[3] = array[3];
		}
		else {}
	}
	
	
void upperMonitorInit(void)   //初始化
  { 
		#ifdef UART7_SEND
  	Driver_Upper_Init(WIRELESS_USARTX,WIRELESS_USARTX_RX_PIN,WIRELESS_USARTX_TX_PIN, \
									 WIRELESS_USART_PreemptionPriority,WIRELESS_USART_SubPriority);
		#endif
		#ifdef USBVCP_SEND
	  usbVCP_Init(WIRELESS_VCP_PreemptionPriority,WIRELESS_VCP_SubPriority);
		#endif
		supervisorData.taskEvent[WIRELESS_TASK] = xTaskCreate(upperMonitorUpdateTask,"UPPERMONITOR",UPPERMONITOR_STACK_SIZE,NULL,UPPERMONITOR_PRIORITY,&upperData.xHandleTask);
		
		digitalLo(&upperData.flag_read);
		digitalLo(&upperData.flag_pitch);
		digitalLo(&upperData.flag_power);
		digitalLo(&upperData.flag_chassis);
		digitalLo(&upperData.flag_chase);
		digitalLo(&upperData.flag_poke);
		digitalLo(&upperData.flag_firewheel);
		digitalHi(&upperData.flag_yaw);
  }
