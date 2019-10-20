#ifndef __DRIVER_JUDGE_H
#define __DRIVER_JUDGE_H

#include "BSP.h"
#include "Util.h"
#define JUDGE_USARTX							USART2			//接收机串口号
#define JUDGE_USARTX_TX_PIN			  BSP_GPIOA2	//接收机发送引脚
#define JUDGE_USARTX_RX_PIN			  BSP_GPIOA3	//接收机接收引脚
#define JUDGE_USART_PRE_PRIORITY 3						//SBUS_USART中断抢占优先级
#define JUDGE_USART_SUB_PRIORITY 0						//SBUS_USART中断响应优先级

#define Armor_0 0
#define Armor_1 1
#define Armor_2 2
#define Armor_3 3
#define Armor_4 4
#define Armor_5 5

#define Armor_Front 	Armor_0
#define Armor_Left 		Armor_1
#define Armor_Behind 	Armor_2
#define Armor_Right		Armor_3
#define Armor_UP1 		Armor_4
#define Armor_UP2 		Armor_5

#define Damage_Armor 			0x00
#define Damage_OverSpeed 	0x01
#define Damage_OverFreq 	0x02
#define Damage_OverPower 	0x03
#define Damage_Offline 		0x04
#define Damage_Foul  			0x06
#define Heal_Park 				0x0A
#define Heal_Engineer 		0x0B

#define LEN           118
#define dataLen       112

//比赛机器人状态 (0x0001)
typedef __packed struct 
{
	uint8_t game_type : 4;   
	uint8_t game_progress : 4;   
	uint16_t stage_remain_time; 
}ext_game_state_t; 
 
//比赛结果数据 (0x0002)
typedef __packed struct 
{    
	uint8_t winner; 
}ext_game_result_t; 

//机器人血量数据 (0x0003)
typedef __packed struct{ 
uint16_t red_1_robot_HP;  
uint16_t red_2_robot_HP;   
uint16_t red_3_robot_HP;    
uint16_t red_4_robot_HP;   
uint16_t red_5_robot_HP;   
uint16_t red_7_robot_HP;   
uint16_t red_base_HP;  
uint16_t blue_1_robot_HP;  
uint16_t blue_2_robot_HP;   
uint16_t blue_3_robot_HP;  
uint16_t blue_4_robot_HP;   
uint16_t blue_5_robot_HP;  
uint16_t blue_7_robot_HP;   
uint16_t blue_base_HP;
}ext_game_robot_HP_t; 

//场地事件数据 (0x0101)
typedef __packed struct
{
	uint32_t event_type;
}ext_event_data_t;

//补给站动作标识（0x0102）
typedef __packed struct 
{   
	uint8_t supply_projectile_id;    
	uint8_t supply_robot_id;    
	uint8_t supply_projectile_step;  
	uint8_t supply_projectile_num;
}ext_supply_projectile_action_t;

//补给站预约子弹（0x0103）
typedef __packed struct 
{   
  uint8_t supply_projectile_id;
  uint8_t supply_robot_id; 	
	uint8_t supply_num;  
}ext_supply_projectile_booking_t; 

//裁判系统警告信息
typedef __packed struct 
{
uint8_t level;
uint8_t foul_robot_id;  
}ext_referee_warning_t; 

//比赛机器人状态（0x0201）
typedef __packed struct 
{   
	uint8_t  robot_id;   
	uint8_t  robot_level;   
	uint16_t remain_HP;   
	uint16_t max_HP;   
	uint16_t shooter_heat0_cooling_rate;   
	uint16_t shooter_heat0_cooling_limit;   
	uint16_t shooter_heat1_cooling_rate;   
	uint16_t shooter_heat1_cooling_limit;   
	uint8_t  mains_power_gimbal_output : 1;  
	uint8_t  mains_power_chassis_output : 1;   
	uint8_t  mains_power_shooter_output : 1; 
}ext_game_robot_state_t; 

//实时功率热量数据(0x0202)
typedef __packed struct 
{   
	uint16_t chassis_volt;    
	uint16_t chassis_current;    
	float 	 chassis_power;    
	uint16_t chassis_power_buffer;    
	uint16_t shooter_heat0;    
	uint16_t shooter_heat1;  
}ext_power_heat_data_t; 

//机器人位置(0x0203)
typedef __packed struct
{
	float x;
	float y;
	float z;
	float yaw;
}ext_game_robot_pos_t;

//机器人增益(0x0204)
typedef __packed struct
{
	uint8_t power_rune_buff;	
}ext_buff_musk_t;

//空中机器人的状态(0x0205)
typedef __packed struct
{
	uint8_t energy_point;
	uint8_t attack_time;
}aerial_robot_energy_t;

//伤害状态(0x0206)
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t hurt_type : 4;
}ext_robot_hurt_t;

//实时射击信息(0x0207)
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t bullet_freq;
	float bullet_speed;	
}ext_shoot_data_t;

//子弹剩余发射数 (0x0208 空中机器人及哨兵机器人主控发送)
typedef __packed struct 
{   
	uint16_t bullet_remaining_num;   
}ext_bullet_remaining_t; 

//参赛队自定义数据（0x0301）
typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}extShowData_t;

//机器人交互 内容ID 发送者ID 接收者ID
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t send_id;
	uint16_t receive_id;
}ext_student_interacttive_header_data_t;

typedef __packed struct
{
	uint8_t data[dataLen];
}ext_receive_data_t;

typedef struct 
{
	unsigned char header;
	unsigned char	tail;
	unsigned char buffer[256];
}buffefLoop_t;

extern buffefLoop_t bufferLoop;

void Driver_Judge_Init(USART_TypeDef *Judge_USARTx,BSP_GPIOSource_TypeDef *Judge_USART_TX,BSP_GPIOSource_TypeDef *Judge_USART_RX,u8 PreemptionPriority,u8 SubPriority);
void judgeExtGameStateInfo(u8 *Array,ext_game_state_t *extGameState);
void judgeExtGameResultInfo(u8 *Array,ext_game_result_t *extGameResult);
void judgeExtGameRobotHpInfo(u8 *Array,ext_game_robot_HP_t *extgameRobotHP);
void judgeExtEventDataInfo(u8 *Array,ext_event_data_t *extEventData);   
void judgeExtSupplyProjectileActionInfo(u8 *Array,ext_supply_projectile_action_t *extSupplyProjectileAction);
void judgeExtSupplyProjectileBookingInfo(u8 *Array,ext_supply_projectile_booking_t *extSupplyProjectileBooking);
void judgeExtRefereeWarningInfo(u8 *Array,ext_referee_warning_t *extRefereeWarning);
void judgeExtGameRobotStateInfo(u8 *Array,ext_game_robot_state_t *extGameRobotState); 
void judgeExtPowerHeatDataInfo(u8 *Array,ext_power_heat_data_t *extPowerHeatData);
void judgeExtGameRobotPosInfo(u8 *Array,ext_game_robot_pos_t *extGameRobotPos );
void judgeExtBuffMuskInfo(u8 *Array,ext_buff_musk_t *extBuffMusk );
void judgeaerialRobotEnergyInfo(u8 *Array,aerial_robot_energy_t *aerialRobotEnergy );
void judgeExtRobotHurtInfo(u8 *Array,ext_robot_hurt_t *extRobotHurt );
void judgeExtShootDataInfo(u8 *Array,ext_shoot_data_t *extShootData );
void judgeExtBulletRemainingInfo(u8 *Array,ext_bullet_remaining_t *extBulletRemaining );//
void judgeOtherRobotSendDataInfo(u8 *Array,ext_receive_data_t *extReceiveData);
void judgeExtShowData(extShowData_t *extShowData,ext_game_robot_state_t *extGameRobotState);
void robotExchangeData(ext_student_interacttive_header_data_t *extStudentData,ext_game_robot_state_t *extGameRobotState,uint8_t robotID);
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength);

#endif
