#ifndef __TASK_JUDGE_H
#define __TASK_JUDGE_H
#include "Driver_Judge.h"
#include "Util.h"
#include "stdbool.h"

#define send_to_robot 7         //英雄 1  工程 2  步兵 3 4 5  空中 6  哨兵 7

enum{
	BUFFER_FIELD_RECOVERY = 0x0001,											//场地回血
	BUFFER_AUXILIARY_RECOVERY	= 0x0002,									//工程车自身回血
	BUFFER_RFID_RECOVERY = 0x0004,											//回血卡回血
	BUFFER_RESOURSE_ISLAND_DEFENSE = 0x0008,						//资源岛防御
	BUFFER_OUR_LARGE_ENERGY = 0X0010,										//我方大能量机关
	BUFFER_ENEMY_LARGE_ENERGY =0x0020,									//敌方大能量机关
	BUFFER_OUR_SMALL_ENERGY = 0x0040,										//我方小能量机关
	BUFFER_ENEMY_SMALL_ENERGY = 0x0080,									//敌方小能量机关
	BUFFER_FAST_COOLING = 0x0100,												//快速冷却缩减
	BUFFER_BLOCKHOUSE_DEFENSE = 0x0200,									//碉堡防御加成
	BUFFER_IGNORE_ATTACK = 0x0400,											//百分百防御加成
	BUFFER_WITOUT_SENTRY_DEFENSE = 0x0800,							//无哨兵基地防御
	BUFFER_SENTRY_ALIVE_DEFENSE	= 0x1000								//有哨兵基地防御
};

enum{
	BUFFER_ROBOT_RECOVERY = 0x01,	                    //机器人血量补血状态
	BUFFER_GUN_COOLING = 0x02,												//枪口热量冷却加速
	BUFFER_ROBOT_DEFENSEBONUS = 0x04,                 //机器人防御加成
	BUFFER_ROBOT_ATTACKADD = 0x08 								    //机器人攻击加成
};

typedef struct{
  ext_game_state_t                extGameState;
	ext_game_result_t               extGameResult;
	ext_game_robot_HP_t      				extGameRobotHp;
	ext_event_data_t                extEventData;
	ext_supply_projectile_action_t  extSupplyProjectileAction;
	ext_supply_projectile_booking_t extSupplyProjectileBooking;
	ext_referee_warning_t           extRefereeWarning;
	ext_game_robot_state_t          extGameRobotState;
	ext_power_heat_data_t           extPowerHeatData;
	ext_game_robot_pos_t            extGameRobotPos;
	ext_buff_musk_t                 extBuffMusk;
	aerial_robot_energy_t           aerialRobotEnergy;
	ext_robot_hurt_t                extRobotHurt;
	ext_shoot_data_t                extShootData;
	ext_bullet_remaining_t					extBulletRemaining;
	extShowData_t										extShowData;
	ext_student_interacttive_header_data_t	extStudentData;
	ext_receive_data_t              extReceiveData;
	uint32_t judgeErrorCount;
	uint32_t judgeLastErrorCount;
	uint32_t intervalNum;
	bool initFlag;
} judgeStruct_t;

extern judgeStruct_t judgeData;

void judgeTask(void);
void jugdeInit(void);
#endif
