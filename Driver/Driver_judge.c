#include "Driver_Judge.h"
#include "judge.h"

#define POSITION_DEFINE 	1	//是否读取position数据
#define READBULLET_DEFINE 1	//是否读取子弹数据
#define READGOLF_DEFINE 	1	//是否读取高尔夫数据

buffefLoop_t bufferLoop = {
	.header = 0,
	.tail		= 0,
	.buffer	= {0},
};

/*
***************************************************
函数名：Driver_Judge_Init
功能：裁判系统串口初始化
入口参数：	Judge_USARTx：裁判系统串口号
					Judge_USART_TX：串口发送引脚号
					Judge_USART_RX：串口接收引脚号
					PreemptionPriority：抢占优先级
					SubPriority：次优先级
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void Driver_Judge_Init(USART_TypeDef *Judge_USARTx,
												BSP_GPIOSource_TypeDef *Judge_USART_TX,
												BSP_GPIOSource_TypeDef *Judge_USART_RX,
												u8 PreemptionPriority,u8 SubPriority){
	BSP_USART_TypeDef Judge_USART = {
		.USARTx = Judge_USARTx,				//串口号
		.USART_RX = Judge_USART_RX,		//引脚
		.USART_TX = Judge_USART_TX,
		.USART_InitStructure = {
			.USART_BaudRate = 115200,										/*波特率设置*/					
			.USART_WordLength = USART_WordLength_8b,		/*字长为8位数据格式*/	
			.USART_StopBits = USART_StopBits_1,					/*一个停止位*/					
			.USART_Parity = USART_Parity_No,						/*无校验位*/						
			.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,/*接收和发送模式*/						
			.USART_HardwareFlowControl = USART_HardwareFlowControl_None,	/*无硬件数据流控制*/	
		}
	};
	BSP_USART_Init(&Judge_USART,PreemptionPriority,SubPriority);
	/*- 暂时屏蔽，有冲突 -*/
	BSP_USART_TX_DMA_Init(&Judge_USART);
	BSP_USART_RX_DMA_Init(&Judge_USART);
}

/*
***************************************************
函数名：judgeExtGameStateInfo
功能：获取裁判系统比赛状态数据(0x0001)
入口参数：	Array：数据段数组指针
					extGameState：比赛状态数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameStateInfo(u8 *Array,ext_game_state_t *extGameState){
	extGameState->game_type    = (uint8_t)Array[0]&0x0f;
	extGameState->game_progress   = ((uint8_t)Array[0]&0xf0)>>4;
	extGameState->stage_remain_time = (uint16_t)Array[2]<<8 | Array[1];
}

/*
***************************************************
函数名：judgeExtGameResultInfo
功能：获取裁判系统比赛结果数据 (0x0002)
入口参数：	Array：数据段数组指针
					extGameResult：比赛结果数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameResultInfo(u8 *Array,ext_game_result_t *extGameResult){
   extGameResult->winner = (uint8_t)Array[0];
}

/*
***************************************************
函数名：judgeExtGameRobotSurvivorsInfo
功能：获取裁判系统比赛机器人血量数据 1HZ (0x0003)
入口参数：	Array：数据段数组指针
					extGameRobotSurvivors：机器人存活数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameRobotHpInfo(u8 *Array,ext_game_robot_HP_t *extGameRobotHp){	
	extGameRobotHp->red_1_robot_HP = (uint16_t)Array[1]<<8 | Array[0];
	extGameRobotHp->red_2_robot_HP = (uint16_t)Array[3]<<8 | Array[2];
	extGameRobotHp->red_3_robot_HP = (uint16_t)Array[5]<<8 | Array[4];
	extGameRobotHp->red_4_robot_HP = (uint16_t)Array[7]<<8 | Array[6];
	extGameRobotHp->red_5_robot_HP = (uint16_t)Array[9]<<8 | Array[8];
	extGameRobotHp->red_7_robot_HP = (uint16_t)Array[11]<<8 | Array[10];
	extGameRobotHp->red_base_HP    = (uint16_t)Array[13]<<8 | Array[12];
	extGameRobotHp->blue_1_robot_HP = (uint16_t)Array[15]<<8 | Array[14];
	extGameRobotHp->blue_2_robot_HP = (uint16_t)Array[17]<<8 | Array[16];
	extGameRobotHp->blue_3_robot_HP = (uint16_t)Array[19]<<8 | Array[18];
	extGameRobotHp->blue_4_robot_HP = (uint16_t)Array[21]<<8 | Array[20];
	extGameRobotHp->blue_5_robot_HP = (uint16_t)Array[23]<<8 | Array[22];
	extGameRobotHp->blue_7_robot_HP = (uint16_t)Array[25]<<8 | Array[24];
	extGameRobotHp->blue_base_HP    = (uint16_t)Array[27]<<8 | Array[26];
}
/*
***************************************************
函数名：judgeExtEventDataInfo
功能：获取裁判系统场地事件数据 (0x0101)
入口参数：	Array：数据段数组指针
					extEventData：场地事件数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtEventDataInfo(u8 *Array,ext_event_data_t *extEventData){
	FormatTrans dataTrans;
	dataTrans.u8_temp[0] = Array[0];
	dataTrans.u8_temp[1] = Array[1];
	dataTrans.u8_temp[2] = Array[2];
	dataTrans.u8_temp[3] = Array[3];
	extEventData->event_type = dataTrans.u32_temp;
}

/*
***************************************************
函数名：judgeExtSupplyProjectileActionInfo
功能：获取裁判系统场地补给站动作标识数据(0x0102)
入口参数：	Array：数据段数组指针
					extSupplyProjectileAction：场地补给站动作标识数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtSupplyProjectileActionInfo(u8 *Array,ext_supply_projectile_action_t *extSupplyProjectileAction){
  extSupplyProjectileAction->supply_projectile_id =  (uint8_t)Array[0];
	extSupplyProjectileAction->supply_robot_id = (uint8_t)Array[1];
	extSupplyProjectileAction->supply_projectile_step = (uint8_t)Array[2];
	extSupplyProjectileAction->supply_projectile_step = (uint8_t)Array[3];
}
/*
***************************************************
函数名：judgeExtSupplyProjectileBookingInfo
功能：请求补给站补弹数据(0x0103) 由参赛队发送
入口参数：	Array：数据段数组指针
					extSupplyProjectileBooking：请求补给站补弹数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtSupplyProjectileBookingInfo(u8 *Array,ext_supply_projectile_booking_t *extSupplyProjectileBooking){
  extSupplyProjectileBooking->supply_projectile_id = (uint8_t)Array[0];
	extSupplyProjectileBooking->supply_robot_id = (uint8_t)Array[1];
	extSupplyProjectileBooking->supply_num = (uint8_t)Array[2];
}
/*
***************************************************
函数名：judgeExtSupplyProjectileBookingInfo
功能：获取裁判系统警告数据(0x0104)
入口参数：	Array：数据段数组指针
					extRefereeWarning：裁判系统警告数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtRefereeWarningInfo(u8 *Array,ext_referee_warning_t *extRefereeWarning){
	extRefereeWarning->level = (uint8_t)Array[0];
	extRefereeWarning->foul_robot_id = (uint8_t)Array[1];
}
/*
***************************************************
函数名：judgeExtGameRobotStateInfo
功能：获取裁判系统机器人状态数据（0x0201）
入口参数：	Array：数据段数组指针
					extGameRobotState：机器人状态数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameRobotStateInfo(u8 *Array,ext_game_robot_state_t *extGameRobotState){
  extGameRobotState->robot_id = (uint8_t)Array[0];
	extGameRobotState->robot_level = (uint8_t)Array[1];
	extGameRobotState->remain_HP = (uint16_t)Array[3]<<8 | Array[2];
	extGameRobotState->max_HP = (uint16_t)Array[5]<<8 | Array[4];
	extGameRobotState->shooter_heat0_cooling_rate = (uint16_t)Array[7]<<8 | Array[6];
	extGameRobotState->shooter_heat0_cooling_limit = (uint16_t)Array[9]<<8 | Array[8];
	extGameRobotState->shooter_heat1_cooling_rate = (uint16_t)Array[11]<<8 | Array[10];
	extGameRobotState->shooter_heat1_cooling_limit = (uint16_t)Array[13]<<8 | Array[12];
	extGameRobotState->mains_power_gimbal_output = (uint8_t)Array[14]&0x01;
	extGameRobotState->mains_power_chassis_output = ((uint8_t)Array[14]&0x02)>>1;
	extGameRobotState->mains_power_shooter_output = ((uint8_t)Array[14]&0x04)>>2;
}
/*
***************************************************
函数名：judgeExtPowerHeatDataInfo
功能：获取裁判系统实时功率热量数据数据（0x0202）
入口参数：	Array：数据段数组指针
					extPowerHeatData：实时功率热量数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtPowerHeatDataInfo(u8 *Array,ext_power_heat_data_t *extPowerHeatData){
	FormatTrans dataTrans;
	extPowerHeatData->chassis_volt = (uint16_t)Array[1]<<8 | Array[0];
	extPowerHeatData->chassis_current = (uint16_t)Array[3]<<8 | Array[2];
	dataTrans.u8_temp[0] = Array[4];
	dataTrans.u8_temp[1] = Array[5];
	dataTrans.u8_temp[2] = Array[6];
	dataTrans.u8_temp[3] = Array[7];
	extPowerHeatData->chassis_power = dataTrans.float_temp;
	extPowerHeatData->chassis_power_buffer = (uint16_t)Array[9]<<8 | Array[8];
	extPowerHeatData->shooter_heat0 = (uint16_t)Array[11]<<8 | Array[10];
	extPowerHeatData->shooter_heat1 = (uint16_t)Array[13]<<8 | Array[12];
}

/*
***************************************************
函数名：judgeExtGameRobotPosInfo
功能：获取机器人位置朝向信息(0x0203)
入口参数：	Array：数据段数组指针
					  extGameRobotPos 位置朝向信息数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtGameRobotPosInfo(u8 *Array,ext_game_robot_pos_t *extGameRobotPos ){
#if POSITION_DEFINE
	FormatTrans dataTrans;
	dataTrans.u8_temp[0] = Array[0];
	dataTrans.u8_temp[1] = Array[1];
	dataTrans.u8_temp[2] = Array[2];
	dataTrans.u8_temp[3] = Array[3];
	extGameRobotPos -> x = dataTrans.float_temp;
	
	dataTrans.u8_temp[0] = Array[4];
	dataTrans.u8_temp[1] = Array[5];
	dataTrans.u8_temp[2] = Array[6];
	dataTrans.u8_temp[3] = Array[7];
	extGameRobotPos -> y = dataTrans.float_temp;
	
	dataTrans.u8_temp[0] = Array[8];
	dataTrans.u8_temp[1] = Array[9];
	dataTrans.u8_temp[2] = Array[10];
	dataTrans.u8_temp[3] = Array[11];
	extGameRobotPos -> z = dataTrans.float_temp;	
	
	dataTrans.u8_temp[0] = Array[12];
	dataTrans.u8_temp[1] = Array[13];
	dataTrans.u8_temp[2] = Array[14];
	dataTrans.u8_temp[3] = Array[15];
	extGameRobotPos -> yaw = dataTrans.float_temp;
	
#endif
}
/*
***************************************************
函数名：judgeExtBuffMuskInfo
功能：获取裁判系统机器人增益数据（0x0204）
入口参数：	Array：数据段数组指针
					extBuffMusk：机器人增益数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtBuffMuskInfo(u8 *Array,ext_buff_musk_t *extBuffMusk ){
	extBuffMusk->power_rune_buff = (uint8_t)Array[0];
}
/*
***************************************************
函数名：judgeaerialRobotEnergyInfo
功能：获取裁判系统空中机器人能量状态数据（0x0205）
入口参数：	Array：数据段数组指针
					aerialRobotEnergy：空中机器人能量状态数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeaerialRobotEnergyInfo(u8 *Array,aerial_robot_energy_t *aerialRobotEnergy ){
	aerialRobotEnergy->energy_point = (uint8_t)Array[0];
	aerialRobotEnergy->attack_time = (uint8_t)Array[1];
}
/*
***************************************************
函数名：judgeExtRobotHurtInfo
功能：获取裁判系统伤害状态数据（0x0206）
入口参数：	Array：数据段数组指针
					extRobotHurt：伤害状态数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtRobotHurtInfo(u8 *Array,ext_robot_hurt_t *extRobotHurt ){
	extRobotHurt->armor_id = (uint8_t)Array[0]&0x0f;
	extRobotHurt->hurt_type = ((uint8_t)Array[0]&0xf0)>>4;
}
/*
***************************************************
函数名：judgeExtShootDataInfo
功能：获取裁判系统实时射击数据（0x0207）
入口参数：	Array：数据段数组指针
					extShootData：实时射击数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtShootDataInfo(u8 *Array,ext_shoot_data_t *extShootData ){
	FormatTrans dataTrans;
  extShootData->bullet_type = (uint8_t)Array[0];
	extShootData->bullet_freq = (uint8_t)Array[1];
	dataTrans.u8_temp[0] = Array[2];
	dataTrans.u8_temp[1] = Array[3];
	dataTrans.u8_temp[2] = Array[4];
	dataTrans.u8_temp[3] = Array[5];
	extShootData->bullet_speed = dataTrans.float_temp;
}
/*
***************************************************
函数名：judgeExtShootDataInfo
功能：获取裁判系统子弹剩余发射数数据（0x0208）
入口参数：	Array：数据段数组指针
					extShootData：实时射击数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtBulletRemainingInfo(u8 *Array,ext_bullet_remaining_t *extBulletRemaining ){
	extBulletRemaining->bullet_remaining_num = (uint16_t)Array[1]<<8 | Array[0];
}
/*
***************************************************
函数名：judgeOtherRobotSendDataInfo
功能：获取其他机器人发送来数据（0x0301）
入口参数：	Array：数据段数组指针
					extReceiveData：接收数据指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeOtherRobotSendDataInfo(u8 *Array,ext_receive_data_t *extReceiveData){	
	u8 index = 0;
	for(index = 0; index < dataLen; index ++) 
	extReceiveData->data[index] = Array[index];
}

/*
***************************************************
函数名：Driver_Judge_SendData
功能：向裁判系统发送数据
入口参数：	senddata：数据段数组指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void judgeExtShowData(extShowData_t *extShowData,ext_game_robot_state_t *extGameRobotState){
	static unsigned char Seq=0;
	FormatTrans dataTrans;
	u8 i;
	Array_USART2_TX[0] = 0xA5;
	Array_USART2_TX[1] = 0x13;				//data长度包含ID在内 长度固定
	Array_USART2_TX[2] = 0x00;
	Array_USART2_TX[3] = Seq;
	Array_USART2_TX[5] = 0x01;
	Array_USART2_TX[6] = 0x03;
	/***内容ID****/
	Array_USART2_TX[7] = 0x80;
	Array_USART2_TX[8] = 0xD1;
	/***发送者ID**/
	Array_USART2_TX[9] = (uint8_t)extGameRobotState->robot_id;
	Array_USART2_TX[10] = 0X00;
	/***客户端ID**/
	if(extGameRobotState->robot_id > 7){        //说明是蓝方
		Array_USART2_TX[11] = (uint8_t)(extGameRobotState->robot_id + 6);
		Array_USART2_TX[12] = 0X01;
	}
	else{
		Array_USART2_TX[11] = (uint8_t)extGameRobotState->robot_id;
		Array_USART2_TX[12] = 0X01;
	}
	
	dataTrans.float_temp = extShowData->data1;
	for(i=0;i<4;i++){
		Array_USART2_TX[13+i] = dataTrans.u8_temp[i];
	}
	
	dataTrans.float_temp = extShowData->data2;
	for(i=0;i<4;i++){
		Array_USART2_TX[17+i] = dataTrans.u8_temp[i];
	}
	
	dataTrans.float_temp = extShowData->data3;
	for(i=0;i<4;i++){
		Array_USART2_TX[21+i] = dataTrans.u8_temp[i];
	}
	
	Array_USART2_TX[25] = extShowData->mask;
	Append_CRC8_Check_Sum(Array_USART2_TX,5);
	Append_CRC16_Check_Sum(Array_USART2_TX,28);
	Seq++;
	BSP_USART2_DMA_SendData(Array_USART2_TX,28);
}

/*
***************************************************
函数名：Driver_Judge_SendData
功能：机器人间交互数据
入口参数：	senddata：数据段数组指针
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void robotExchangeData(ext_student_interacttive_header_data_t *extStudentData,ext_game_robot_state_t *extGameRobotState,uint8_t robotID){
	static unsigned char Seq=0;
	u8 count = 0;
	extStudentData->data_cmd_id = 0x0200; // 0x0200 ~ 0x02FF
	extStudentData->send_id = extGameRobotState->robot_id;
	if(extGameRobotState->robot_id > 7) //己方是蓝方
		//只能发送给己方
		extStudentData->receive_id = robotID + 0x0a;
	else
		//己方是红方
		extStudentData->receive_id = robotID;
	//如果是自己发送给自己
	if(extGameRobotState->robot_id == extStudentData->receive_id ){
		//发给自己什么都不做
	}
	else{
		//填装数据长度小于113 个字节
		Array_USART2_TX[0] = 0xA5;
		Array_USART2_TX[2] = 0x00;
		Array_USART2_TX[3] = Seq;
		Array_USART2_TX[5] = 0x01;
		Array_USART2_TX[6] = 0x03;
		/****内容ID****/
		Array_USART2_TX[7+(count++)] = (uint8_t)(extStudentData->data_cmd_id & 0x00ff);
		Array_USART2_TX[7+(count++)] = (uint8_t)((extStudentData->data_cmd_id & 0xff00)>>8);

		/***发送者ID***/
		Array_USART2_TX[7+(count++)] = extStudentData->send_id;
		Array_USART2_TX[7+(count++)] = 0x00;
		
		/***接收者ID***/
		Array_USART2_TX[7+(count++)] = extStudentData->receive_id;
		Array_USART2_TX[7+(count++)] = 0x00;
		
		/**数据段***/
		Array_USART2_TX[7+(count++)] = 0x00;   // 数据填装  数据带宽最大为112byte  测试数据“1”正式填装将其覆盖
		/*..........*/
		/**********/
		
		Array_USART2_TX[1] = count;						//数据段长度
		
		Append_CRC8_Check_Sum(Array_USART2_TX,5);
		Append_CRC16_Check_Sum(Array_USART2_TX,count+9);
		Seq++;
		BSP_USART2_DMA_SendData(Array_USART2_TX,count+9);	
	}
}

//crc8 generator polynomial:G(x)=x8+x5+x4+1
static const unsigned char CRC8_INIT = 0xff;
static const unsigned char CRC8_TAB[256] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8){
	unsigned char ucIndex;
	while (dwLength--){
		ucIndex = ucCRC8^(*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}
	return(ucCRC8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength){
	unsigned char ucExpected = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) 
		return 0;
	ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT);
	return ( ucExpected == pchMessage[dwLength-1] );
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength){
	unsigned char ucCRC = 0;
	if ((pchMessage == 0) || (dwLength <= 2)) 
		return;
	ucCRC = Get_CRC8_Check_Sum ( (unsigned char *)pchMessage, dwLength-1, CRC8_INIT);
	pchMessage[dwLength-1] = ucCRC;
}

uint16_t CRC_INIT = 0xffff;
const uint16_t wCRC_Table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC){
	uint8_t chData;
	if (pchMessage == NULL){
		return 0xFFFF;
	}
	while(dwLength--){
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^
		(uint16_t)(chData)) & 0x00ff];
	}
	return wCRC;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength){
	uint16_t wExpected = 0;
	if ((pchMessage == NULL) || (dwLength <= 2)){
		return 0;
	}
	wExpected = Get_CRC16_Check_Sum ( pchMessage, dwLength - 2, CRC_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void Append_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength){
	uint16_t wCRC = 0;
	if ((pchMessage == NULL) || (dwLength <= 2))
		return;
	wCRC = Get_CRC16_Check_Sum ( (u8 *)pchMessage, dwLength-2, CRC_INIT );
	pchMessage[dwLength-2] = (u8)(wCRC & 0x00ff);
	pchMessage[dwLength-1] = (u8)((wCRC >> 8)& 0x00ff);
}

