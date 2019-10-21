#ifndef __DEFORMING_H
#define __DEFORMING_H
#include "BSP.h"
#include "Util.h"
#include "pid.h"
#include "chassis.h"

/************
英雄车
*************/
#define TANK_NORMAL_DISTANCE  70.0f
#define TANK_MAX_DISTANCE     540.0f
#define TANK_MIN_DISTANCE     30.0f
#define TANK_GRAB_DISTANCE    185.0f

#define TANK_GRAB_MAX_ANGLE	335		//角度传感器值，抓子伸最高值470
#define TANK_GRAB_MIN_ANGLE	230		//角度传感器值，抓子伸最低值
#define TANK_GRAB_THROW_ANGLE	315		//角度传感器值，抓子丢弹药箱值

#define LANDING_1 PEin(2)
#define LANDING_2 PEin(13)

#define LANDING_3 PAin(1)
#define LANDING_4 PEin(10)

#define HOLDPILLAR_SENSOR PCin(5)
#define GRABLEFTSENSOR  PEin(14)
#define GRABRIGHTSENSOR PEin(4)

#define GRAB_LED  PDout(13)
#define GRAB_LED_ON 	GRAB_LED=1;
#define GRAB_LED_OFF 	GRAB_LED=0;

#define GRAB PBout(1)
#define GRAB_ON 	GRAB=1;
#define GRAB_OFF 	GRAB=0;

#define HOLDPILLAR PBout(0)
#define HOLDPILLAR_ON  HOLDPILLAR=1;
#define HOLDPILLAR_OFF HOLDPILLAR=0;

#define RIGHT_BYPASS_MOTOR TIM3->CCR2
#define RIGHT_BYPASS_MOTOR_ON   (RIGHT_BYPASS_MOTOR = 1750)
#define RIGHT_BYPASS_MOTOR_OFF  (RIGHT_BYPASS_MOTOR = 1000)

#define LEFT_BYPASS_MOTOR  TIM3->CCR1
#define LEFT_BYPASS_MOTOR_ON   (LEFT_BYPASS_MOTOR = 1750)
#define LEFT_BYPASS_MOTOR_OFF  (LEFT_BYPASS_MOTOR = 1000)

/************
工程车登岛
*************/
#define ID_OPTOLECTONIC_SWITCH   0x602     //黄管信号
#define ID_SENSORDATA            0x704     //抓弹传感器数据
#define ID_OPTOLECTONIC1_SWITCH  0x604
#define ID_OPTOLECTONIC2_SWITCH  0x606
#define MELT_DATA                0x602
#define LIFTDISTANCEMAX       	440    		//抱柱最高位置468
#define LIFTDISTANCEMIN       	20    		//车体拉升最小位置
#define BULLETTRANSFERDISTANCE	330				//补单高度
#define GRAB_DISTANCE         	270     	//抓弹高度
#define INITDISTANCE          	50      	//车身初始位置
#define LANDINGDISTANCE       	36

#define HOLD_PILLAR_SPEED     7500
#define holdPillarDistanceMax 0
#define holdPillarDistanceMin 0
#define HOLDPILLARON          1
#define HOLDPILLAROFF         0
#define DUCTEDFANLEFT_ON			0
#define DUCTDEFANLEFT_OFF			1
#define DUCTDEFANRIGHT_ON			0
#define DUCTDEFANRIGHT_OFF		0
#define Op_INITIAL_DISTANCE   0       //2.3号光电开关初始安装据地面的位置
#define GRAB_LIFT_DISTANCE    0       //抓弹车架上升高度
#define GRAB_MEASURE_DISTANCE 0       //抓弹时离墙的距离
#define GRAB_DISTANCE_FOR_WALL_MIN     0       //抓弹对位时离墙最小距离
#define GRAB_DISTANCE_FOR_CAISSSON_MIN     0   //抓弹时离第一排弹药箱的位置
#define GRAB_DISTANCE_FOR_CAISSSON_MAX     0   //抓弹时离第二排弹药箱的位置
#define DOWN_HOLDPILLAR_POSITION   0     //下降抱柱时车升起的位置
#define HOLD_PILLAR_ROTATE_REF 1900/4     //每分钟50圈
#define HOLD_PILLAR_LOOSEN pneumaticData.send1[2]=0x01 //抱柱松开
#define HOLD_PILLAR_TIGHT  pneumaticData.send1[2]=0x00  //抱柱抱紧 

/************
工程车抓弹
*************/
/* 
以下开关是对应实物动作、不是对应气缸的开关
爪子的打开对应气缸的关闭
爪子的关闭对应气缸的打开
*/
#define EJECT_OPEN    				pneumaticData.send1[0]=1  	//弹射打开
#define EJECT_CLOSE   				pneumaticData.send1[0]=0  	//弹射关闭
#define SMALLMAGAZINE_OPEN    pneumaticData.send1[1]=1  	//小弹仓开
#define SMALLMAGAZINE_CLOSE   pneumaticData.send1[1]=0  	//小弹仓关
#define PAW_OPEN              pneumaticData.send1[2]=1  	//打开爪子
#define PAW_CLOSE             pneumaticData.send1[2]=0  	//夹紧爪子
#define PAWGO_FORWARD         pneumaticData.send1[3]=1  	//爪子前进
#define PAWGO_BACK         		pneumaticData.send1[3]=0  	//爪子后退
#define PAWTURN_OPEN          pneumaticData.send1[4]=1  	//爪子翻转打开
#define PAWTURN_CLOSE         pneumaticData.send1[4]=0  	//爪子翻转关闭

#define PAWGOLEFT_OPEN        pneumaticData.send2[0]=1  	//爪子左移打开
#define PAWGOLEFT_CLOSE       pneumaticData.send2[0]=0  	//爪子左移关闭
#define PAWGORIGHT_OPEN       pneumaticData.send2[1]=1  	//爪子右移打开
#define PAWGORIGHT_CLOSE      pneumaticData.send2[1]=0		//爪子右移关闭

#define TOP_RIGHT							pneumaticData.send3[0]=1		//右移开
#define TOP_LEFT							pneumaticData.send3[0]=0		//右移关
#define TAIL_OPEN							pneumaticData.send3[1]=1		//尾巴支撑开
#define TAIL_CLOSE						pneumaticData.send3[1]=0		//尾巴支撑关
#define UP_OPEN								pneumaticData.send3[2]=1		//抬升开
#define UP_CLOSE							pneumaticData.send3[2]=0		//抬升关

#define ONE_LEFT              auxiliaryData.optoelectronicRecive[0]
#define ONE_RIGHT             auxiliaryData.optoelectronicRecive[1]
#define TWO_LEFT              auxiliaryData.optoelectronicRecive[2]

/************
工程车救援
*************/
#define RESCUE_L			  (keyBoardCtrlData.lkSta == KEY_PRESS_ONCE)
#define RESCUE_R   			(keyBoardCtrlData.rkSta == KEY_PRESS_ONCE)
#define RESCUE_TIGHT  pneumaticData.send3[3]=1		//救援机构落下
#define RESCUE_LOOSEN pneumaticData.send3[3]=0		//救援机构抬起
 
/************
工程车子弹交接
*************/
#define MAGAZINE_OPEN  pneumaticData.send3[4]=1	//弹仓盖打开
#define MAGAZINE_CLOSE pneumaticData.send3[4]=0	//弹仓盖关闭


typedef struct 
{
	uint8_t optoelectronicRecive[8];						 					//光电开关返回信号			.
	pidData_t pawangle[2];
	pidData_t pawmotor[2];
	pidStruct_t *pawmotorSpeedPID[2];
	pidStruct_t *pawmotorAnglePID[2];
	float intervalTime;
	
}auxiliaryStruct_t;         														//整车上升(包含登岛抓弹下岛全数据)

extern auxiliaryStruct_t auxiliaryData;  

void mechaDeformingUpdate(void);
void mechaDeformingInit(void);
void grabAmmunitionUpdate(void);
uint8_t tankGrabForward(void);
uint8_t tankGrabBack(void);
uint8_t tankGrabThrow(void);
uint8_t supplyGrabForward(void);				//前伸

/********
工程车
********/
void readLineDistance_data(CanRxMsg *can_rx_msg,auxiliaryStruct_t *auxiliaryData);   //读取线位移传感器距离值
void readOpSwitch602_data(CanRxMsg *can_rx_msg);   //光电开关
void SensorDataReceive(CanRxMsg *can_rx_msg);
void readmeltSwitch608_data(CanRxMsg *can_rx_msg);    //黄管信号 01是底盘信号 23是抓蛋对位
void tof_readdata(CanRxMsg *can_rx_msg);
void tof_readdata2(CanRxMsg *can_rx_msg);
void liftUpdate(void);
void chassisChaseSwitch(u8 isEnable);
void grabFirstMiddle(void);
void grabZeroState(void);
void resSetPress(void);
#endif
