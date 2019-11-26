#include "wireless.h"
#include "keyboard.h"
#include "board.h"
#include "judge.h"
#include "shoot.h"

BSP_USART_TypeDef WIRELESS;
wirelessStruct_t wirelessData;
/* 两个宏定义只能定义一个 */
//#define ANO_DT_USE_USB_HID		//使用USB
#define ANO_DT_USE_UART7			//使用串口
/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;					//需要发送数据的标志

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(u8 *dataToSend , u8 length){
#ifdef ANO_DT_USE_USB_HID
	usbVCP_SendBuffer(dataToSend,length);
#endif
#ifdef ANO_DT_USE_UART7
	WIRELESS.USARTx = WIRELESS_USARTX;
	BSP_USART_SendData( &WIRELESS, dataToSend, length);
#endif
}
static void ANO_DT_Send_Check(u8 head, u8 check_sum){
	wirelessData.dataNeedSend[0]=0xAA;
	wirelessData.dataNeedSend[1]=0xAA;
	wirelessData.dataNeedSend[2]=0xEF;
	wirelessData.dataNeedSend[3]=2;
	wirelessData.dataNeedSend[4]=head;
	wirelessData.dataNeedSend[5]=check_sum;
	
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[6]=sum;

	ANO_DT_Send_Data(wirelessData.dataNeedSend, 7);
}


void ANO_DT_Data_Exchange(void){
	static u8 cnt = 0;
	static u8 senser_cnt 	= 10;
	static u8 senser2_cnt = 50;
	static u8 user_cnt 	  = 10;
	static u8 status_cnt 	= 15;
	static u8 rcdata_cnt 	= 20;
	static u8 motopwm_cnt	= 20;
	static u8 power_cnt		=	50;
	static u8 speed_cnt   = 50;
	static u8 location_cnt   = 200;
	static u8 fly_ready = 0;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.sendSenser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		f.sendSenser2 = 1;	

	if((cnt % user_cnt) == (user_cnt-2))
		f.sendUser = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.sendStatus = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.sendRcData = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-2))
		f.sendMotoPwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-2))
		f.sendPower = 1;		
	
	if((cnt % speed_cnt) == (speed_cnt-3))
		f.sendSpeed = 1;		
	
	if((cnt % location_cnt) == (location_cnt-3))
	{
		f.sendLocation += 1;		
	}
	
	if(++cnt>200) cnt = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.sendCheck)
	{
		f.sendCheck = 0;
		ANO_DT_Send_Check(wirelessData.checkDataNeedSend,wirelessData.checkSumNeedSend);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendVersion)
	{
		f.sendVersion = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendStatus)
	{
		f.sendStatus = 0;
		fly_ready = supervisorData.state & STATE_ARMED;
		ANO_DT_Send_Status(AQ_ROLL,AQ_PITCH,AQ_YAW,0,0,fly_ready);
	}	
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendSpeed)
	{
		f.sendSpeed = 0;
		ANO_DT_Send_Speed(0,0,0);
	}
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendUser)
	{
		f.sendUser = 0;
		ANO_DT_Send_User();
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendSenser)
	{
		f.sendSenser = 0;
//		ANO_DT_sendSenser(temp_speed[0],landingActionData.speed[0],landingActionData.speedTarget[0],
//											 average_dis[0],landingAIMU_RATEYctionData.distance[0],landingActionData.positionTarget[0],
//											 mpu6500Data.mag.x,mpu6500Data.mag.y,mpu6500Data.mag.z);     
		ANO_DT_Send_Senser(gimbalData.pitchAngleFbd*100,gimbalData.pitchAngleRef*100,gimbalData.yawAngleFbd*100,
											 gimbalData.yawAngleRef*100,chassisData.chaseFbd*100,0,
											 gimbalData.yawSpeedFbd*100,0,0); 
//		
//		ANO_DT_Send_Senser(visionData.armorRate[0]* 1000, UKF_VEL_X * 1000, visionData.armorRate[1]* 1000,
//											 UKF_VEL_Y * 1000, (visionData.intervalTime-0.04f) * 100000, UKF_VEL_Z * 1000,	
//											 visionData.armorCoordinate[0].float_temp , UKF_POS_X * 1000, visionData.armorCoordinatePredict[1].float_temp * 1000);
//		ANO_DT_Send_Senser(visionData.angleBias[0] * 100, visionData.angleBias[1] * 100, visionData.barrelToArmor.float_temp,
//											 UKF_POS_Z * 1000, judgeData.extPowerHeatData.chassis_power, 80,	
//											chassisData.current[0] , powerRealDate.current[0], chassisData.chaseSpeedRef);
//			ANO_DT_Send_Senser(chassisData.chaseRef,0, chassisData.chaseFbd*100,
//												 gimbalData.pitchAngleFbd*100, visionData.armorRate[2] * 1000, UKF_VEL_Z * 1000,
//			  								 visionData.armorCoordinate[0].float_temp , UKF_POS_X * 1000, (visionData.yawData.float_temp - gimbalData.yawAngleSave) * 10);
//		//弹丸射速发送
//		static float bulletLimit = 30.0f;
//		switch(judgeData.extShootData.bullet_type){		
//			case 1 : bulletLimit = 30.0f; break;
//			case 2 : bulletLimit = 16.5f; break;
//			default: bulletLimit = 0.0f ; break;
//		}
//		ANO_DT_Send_Senser(0,judgeData.extShootData.bullet_speed*100,0,
//											 16*100,0,0,
//											 0,shootData.fricWheelSpeedRef[0],fricWheelData[0].speed);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendSenser2)
	{
		f.sendSenser2 = 0;
		ANO_DT_Send_Senser2(0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendRcData)
	{
		f.sendRcData = 0;
		ANO_DT_Send_RCData(remoteControlData.dt7Value.rcRawData.CH2+1500,remoteControlData.dt7Value.rcRawData.CH3+1500,\
											 remoteControlData.dt7Value.rcRawData.CH0+1500,remoteControlData.dt7Value.rcRawData.CH1+1500,\
											 0,0,0,0,0,0);
											 
	}	
///////////////////////////////////////////////////////////////////////////////////////	
	else if(f.sendMotoPwm)
	{
		f.sendMotoPwm = 0;
		ANO_DT_Send_MotoPWM(0,0,0,0,0,0,0,0);
	}	
///////////////////////////////////////////////////////////////////////////////////////
//	else if(f.sendPower)
//	{
//		f.sendPower = 0;
//		ANO_DT_sendPower(123,456);
//	}
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendPid1)
	{
		f.sendPid1 = 0;
		ANO_DT_Send_PID(1,0.001f*parameter[PITCH_RATE_P],0.001f*parameter[PITCH_RATE_I],0.001f*parameter[PITCH_RATE_D],
											0.001f*parameter[PITCH_RATE_P],0.001f*parameter[PITCH_RATE_I],0.001f*parameter[PITCH_RATE_D],
											0.001f*parameter[YAW_RATE_P],0.001f*parameter[YAW_RATE_I],0.001f*parameter[YAW_RATE_D]);
	}	
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendPid2)
	{
		f.sendPid2 = 0;
		ANO_DT_Send_PID(2,parameter[PITCH_ANG_P],parameter[PITCH_ANG_I],parameter[PITCH_ANG_D],
											parameter[PITCH_ANG_P],parameter[PITCH_ANG_I],parameter[PITCH_ANG_D],
											parameter[YAW_ANG_P],parameter[YAW_ANG_I],parameter[YAW_ANG_D]);
	}
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.sendPid3)
	{
		f.sendPid3 = 0;
		ANO_DT_Send_PID(3,0.1f*parameter[CHASSIS_SPEED_P],0.1f*parameter[CHASSIS_SPEED_I],0.1f*parameter[CHASSIS_SPEED_D],
											0.01f*parameter[CHASSIS_CHASE_P],0.01f*parameter[CHASSIS_CHASE_I],0.01f*parameter[CHASSIS_CHASE_D],
											0,0,0);
	}
	else if(f.sendPid4)
	{
		f.sendPid4 = 0;
		ANO_DT_Send_PID(4,0,0,0,
											parameter[LOADED_SPEED_P],parameter[LOADED_SPEED_I],parameter[LOADED_SPEED_D],
											0,0,0);
	}
//	else if(f.sendLocation == 2)
//	{
//		
//		f.sendLocation = 0;
//		ANO_DT_sendLocation(gpsx.fixmode,gpsx.svnum,gpsx.longitude*10000000,gpsx.latitude*10000000,0);
//		
//	}
//	Usb_Hid_Send();					
/////////////////////////////////////////////////////////////////////////////////////
}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed){
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x01;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	wirelessData.dataNeedSend[_cnt++]=BYTE3(_temp2);
	wirelessData.dataNeedSend[_cnt++]=BYTE2(_temp2);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp2);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp2);
	
	wirelessData.dataNeedSend[_cnt++] = fly_model;
	
	wirelessData.dataNeedSend[_cnt++] = armed;
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[_cnt++]=sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z){
	u8 _cnt=0;
	vs16 _temp;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x02;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	_temp = a_x;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
/////////////////////////////////////////
	_temp = 0;	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);	
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[_cnt++] = sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}

void ANO_DT_Send_Senser2(s32 bar_alt,u16 csb_alt){
	u8 _cnt=0;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x07;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	wirelessData.dataNeedSend[_cnt++]=BYTE3(bar_alt);
	wirelessData.dataNeedSend[_cnt++]=BYTE2(bar_alt);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(bar_alt);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(bar_alt);

	wirelessData.dataNeedSend[_cnt++]=BYTE1(csb_alt);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(csb_alt);
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[_cnt++] = sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6){
	u8 _cnt=0;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x03;
	wirelessData.dataNeedSend[_cnt++]=0;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(thr);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(thr);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(yaw);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(yaw);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(rol);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(rol);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(pit);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(pit);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux1);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux1);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux2);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux2);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux3);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux3);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux4);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux4);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux5);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux5);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(aux6);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(aux6);

	wirelessData.dataNeedSend[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	
	wirelessData.dataNeedSend[_cnt++]=sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8){
	u8 _cnt=0;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x06;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_1);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_1);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_2);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_2);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_3);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_3);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_4);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_4);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_5);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_5);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_6);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_6);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_7);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_7);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(m_8);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(m_8);
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	
	wirelessData.dataNeedSend[_cnt++]=sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d){
	u8 _cnt=0;
	vs16 _temp;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x10+group-1;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	
	wirelessData.dataNeedSend[_cnt++]=sum;

	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_User(void){
	u8 _cnt=0;
	vs16 _temp;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA; 
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xf1; //用户数据
	wirelessData.dataNeedSend[_cnt++]=0;
	
	
//	_temp = (s16)baro_measure;            //1
	_temp = 0;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);

	_temp = (s16)0;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	_temp = (s16)0;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);	
	
	_temp = (s16)0;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
  _temp = (s16)0;              //5
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);

	

	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	
	wirelessData.dataNeedSend[_cnt++]=sum;

	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
void ANO_DT_Send_Speed(float x_s,float y_s,float z_s){
	u8 _cnt=0;
	vs16 _temp;
	
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x0B;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	_temp = (int)(0.1f *x_s);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *y_s);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	_temp = (int)(0.1f *z_s);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(_temp);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(_temp);
	
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[_cnt++]=sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);

}
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver){
	u8 _cnt=0;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0xAA;
	wirelessData.dataNeedSend[_cnt++]=0x00;
	wirelessData.dataNeedSend[_cnt++]=0;
	
	wirelessData.dataNeedSend[_cnt++]=hardware_type;
	wirelessData.dataNeedSend[_cnt++]=BYTE1(hardware_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(hardware_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(software_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(software_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(protocol_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(protocol_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE1(bootloader_ver);
	wirelessData.dataNeedSend[_cnt++]=BYTE0(bootloader_ver);
	
	wirelessData.dataNeedSend[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += wirelessData.dataNeedSend[i];
	wirelessData.dataNeedSend[_cnt++]=sum;
	
	ANO_DT_Send_Data(wirelessData.dataNeedSend, _cnt);
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data){
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
u16 RX_CH[9];
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num){
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
		{
			digitalHi(&wirelessData.imuCalibrate);
		}
		else if(*(data_buf+4)==0X02)
			digitalHi(&wirelessData.imuCalibrate);
		else if(*(data_buf+4)==0X03)
		{
			digitalHi(&wirelessData.imuCalibrate);			
		}
		else if(*(data_buf+4)==0X04)
		{
			digitalHi(&wirelessData.magCalibrate);
		}
		else if((*(data_buf+4)>=0X021)&&(*(data_buf+4)<=0X26))
		{
			//acc_3d_calibrate_f = 1;
		}
		else if(*(data_buf+4)==0X20)
		{
			//acc_3d_step = 0; //退出，6面校准步清0
		}
	}
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.sendPid1 = 1;
			f.sendPid2 = 1;
			f.sendPid3 = 1;
			f.sendPid4 = 1;
			f.sendPid5 = 1;
			f.sendPid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.sendVersion = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
//			Para_ResetToFactorySetup();
		}
	}
	if(*(data_buf+2)==0X10)								//PID1
	{
		parameter[PITCH_RATE_P]  = 1*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		parameter[PITCH_RATE_I]  = 1*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		parameter[PITCH_RATE_D]  = 1*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		parameter[PITCH_RATE_P] = 1*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		parameter[PITCH_RATE_I] = 1*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		parameter[PITCH_RATE_D] = 1*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		parameter[YAW_RATE_P] 	= 1*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		parameter[YAW_RATE_I] 	= 1*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		parameter[YAW_RATE_D] 	= 1*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.sendCheck == 0)
		{
			f.sendCheck = 1;
			wirelessData.checkDataNeedSend = *(data_buf+2);
			wirelessData.checkSumNeedSend = sum;
		}
//		pidZeroState(controlData.rollRatePID);
//		pidZeroState(controlData.pitchRatePID);
//		pidZeroState(controlData.yawRatePID);
//		
//		controlData.rollRatePID = pidInit(&parameter[PITCH_RATE_P], &parameter[PITCH_RATE_I], &parameter[PITCH_RATE_D], &parameter[PITCH_RATE_F],	\
//																		 &parameter[PITCH_RATE_PM], &parameter[PITCH_RATE_IM], &parameter[PITCH_RATE_PM], &parameter[PITCH_RATE_OM],	\
//																		 0, 0, 0, 0);
//		controlData.pitchRatePID = controlData.rollRatePID;
//		controlData.yawRatePID = 		pidInit(&parameter[YAW_RATE_P], &parameter[YAW_RATE_I], &parameter[YAW_RATE_D], &parameter[YAW_RATE_F],	\
//																	 &parameter[YAW_RATE_PM], &parameter[YAW_RATE_IM], &parameter[YAW_RATE_PM], &parameter[YAW_RATE_OM],	\
//																	 0, 0, 0, 0);
		digitalHi(&supervisorData.flashSave);
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
		parameter[PITCH_ANG_P]  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		parameter[PITCH_ANG_I]  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		parameter[PITCH_ANG_D]  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
		parameter[PITCH_ANG_P] = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		parameter[PITCH_ANG_I] = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		parameter[PITCH_ANG_D] = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
		parameter[YAW_ANG_P] 	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
		parameter[YAW_ANG_I] 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
		parameter[YAW_ANG_D] 	= 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.sendCheck == 0)
		{
			f.sendCheck = 1;
			wirelessData.checkDataNeedSend = *(data_buf+2);
			wirelessData.checkSumNeedSend = sum;                                                                 
		}
//		pidZeroState(controlData.rollAnglePID);
//		pidZeroState(controlData.pitchAnglePID);
//		pidZeroState(controlData.yawAnglePID);
//		
//		controlData.rollAnglePID = 	pidInit(&parameter[PITCH_ANG_P], &parameter[PITCH_ANG_I], &parameter[PITCH_ANG_D], &parameter[PITCH_ANG_F],	\
//																		 &parameter[PITCH_ANG_PM], &parameter[PITCH_ANG_IM], &parameter[PITCH_ANG_PM], &parameter[PITCH_ANG_OM],	\
//																		 0, 0, 0, 0);
//		controlData.pitchAnglePID = controlData.rollAnglePID;
//		controlData.yawAnglePID =	  pidInit(&parameter[YAW_ANG_P], &parameter[YAW_ANG_I], &parameter[YAW_ANG_D], &parameter[YAW_ANG_F],	\
//																		&parameter[YAW_ANG_PM], &parameter[YAW_ANG_IM], &parameter[YAW_ANG_PM], &parameter[YAW_ANG_OM],	\
//																		0, 0, 0, 0);
		digitalHi(&supervisorData.flashSave);
	}
	if(*(data_buf+2)==0X12)								//PID3
	{	
		parameter[CHASSIS_SPEED_P]  = 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
		parameter[CHASSIS_SPEED_I]  = 0.01*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
		parameter[CHASSIS_SPEED_D]  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
	
		parameter[CHASSIS_CHASE_P] = 0.1*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		parameter[CHASSIS_CHASE_I] = 0.1*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		parameter[CHASSIS_CHASE_D] = 0.1*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
	
//		parameter[PROPORTIONAL_SPEED_P] 	= 0.1*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//		parameter[PROPORTIONAL_SPEED_I] 	= 0.1*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//		parameter[PROPORTIONAL_SPEED_D] 	= 0.1*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
		if(f.sendCheck == 0)
		{
			f.sendCheck = 1;
			wirelessData.checkDataNeedSend = *(data_buf+2);
			wirelessData.checkSumNeedSend = sum;
		}
		pidZeroState(chassisData.chasePID);
		for(uint8_t index = 0;index < NUMBER_OF_WHEEL;index++){
			pidZeroState(chassisData.speedPID[index]);	
			chassisData.speedPID[index] = pidInit(&parameter[CHASSIS_SPEED_P],&parameter[CHASSIS_SPEED_I],&parameter[CHASSIS_SPEED_D],&parameter[CHASSIS_SPEED_F],
																&parameter[CHASSIS_SPEED_PM],&parameter[CHASSIS_SPEED_IM],&parameter[CHASSIS_SPEED_DM],&parameter[CHASSIS_SPEED_OM],
																0,0,0,0);
		}
		chassisData.chasePID = pidInit(&parameter[CHASSIS_CHASE_P],&parameter[CHASSIS_CHASE_I],&parameter[CHASSIS_CHASE_D],&parameter[CHASSIS_CHASE_F],
																&parameter[CHASSIS_CHASE_PM],&parameter[CHASSIS_CHASE_IM],&parameter[CHASSIS_CHASE_DM],&parameter[CHASSIS_CHASE_OM],
																0,0,0,0);
		digitalHi(&supervisorData.flashSave);
	}
	if(*(data_buf+2)==0X13)								//PID4
	{
//		parameter[PROPORTIONAL_POS_P]  = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//		parameter[PROPORTIONAL_POS_I]  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//		parameter[PROPORTIONAL_POS_D]  = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
	
		parameter[LOADED_SPEED_P]  = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
		parameter[LOADED_SPEED_I]  = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
		parameter[LOADED_SPEED_D]  = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );	
			
		if(f.sendCheck == 0)
		{
			f.sendCheck = 1;
			wirelessData.checkDataNeedSend = *(data_buf+2);
			wirelessData.checkSumNeedSend = sum;
		}
//		pidZeroState(gimbalData.visionYawPID);
//		pidZeroState(gimbalData.visionPitchPID);
//		gimbalData.visionYawPID = pidInit(&parameter[VISION_POS_P], &parameter[VISION_POS_I], &parameter[VISION_POS_D], &parameter[VISION_POS_F],	\
//																&parameter[VISION_POS_PM], &parameter[VISION_POS_IM], &parameter[VISION_POS_DM], &parameter[VISION_POS_OM],	\
//																NULL, NULL, NULL, NULL);

//		gimbalData.visionPitchPID = pidInit(&parameter[VISION_POS_P], &parameter[VISION_POS_I], &parameter[VISION_POS_D], &parameter[VISION_POS_F],	\
//																&parameter[VISION_POS_PM], &parameter[VISION_POS_IM], &parameter[VISION_POS_DM], &parameter[VISION_POS_OM],	\
//																NULL, NULL, NULL, NULL);
		digitalHi(&supervisorData.flashSave);
	}
}

void dataSendToGroundStation(void){

#if (CONTROLLER == DAGGER)
	ANO_DT_Data_Exchange();
#else
	ANO_DT_Data_Exchange();
#endif
}


void dataTransportTask(void *Parameters){
	while(1)
	{
		vTaskDelay(WIRELESS_PERIOD);
		dataSendToGroundStation();
		digitalIncreasing(&wirelessData.loops);             
	}
	
}
void wirelessInit(void){
#ifdef ANO_DT_USE_USB_HID
	usbVCP_Init(WIRELESS_VCP_PreemptionPriority,WIRELESS_VCP_SubPriority);
#endif
#ifdef ANO_DT_USE_UART7
	Driver_DTU_Init(WIRELESS_USARTX,WIRELESS_USARTX_RX_PIN,WIRELESS_USARTX_TX_PIN, \
									 WIRELESS_USART_PreemptionPriority,WIRELESS_USART_SubPriority);
#endif
	supervisorData.taskEvent[WIRELESS_TASK] = xTaskCreate(dataTransportTask,"DATATRANS",WIRELESS_STACK_SIZE,NULL,WIRELESS_PRIORITY,&wirelessData.xHandleTask);
}
