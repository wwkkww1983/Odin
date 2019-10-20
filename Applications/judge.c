#include "judge.h"
#include "shoot.h"
#include "cansend.h"
#include "type_robot.h"

unsigned char bufferdecode[256];
judgeStruct_t judgeData;
void judgeTask(void){
	u8 i;
	u8 bufferlength;
	u8 distance;
	u16 cmdid;
	//如果环形数组的头地址和尾地址一致说明当前环形数组无处理数据
	if(bufferLoop.header != bufferLoop.tail){
		//计算数组头尾之间相差几个字节
		if(bufferLoop.tail<bufferLoop.header){
			distance = bufferLoop.tail + 256 - bufferLoop.header;
		}
		else{
			distance = bufferLoop.tail - bufferLoop.header;
		}
		//如果当前数组头的值不为0xA5则向前寻找到0xA5
		while((bufferLoop.header != bufferLoop.tail) && (bufferLoop.buffer[bufferLoop.header] != 0xA5)){
			digitalIncreasing(&bufferLoop.header);
		}
		if(bufferLoop.header != bufferLoop.tail){
			bufferlength = bufferLoop.buffer[(u8)(bufferLoop.header+1)];
			bufferlength += 9;
			for(i=0;i<bufferlength;i++){
				bufferdecode[i] = bufferLoop.buffer[(u8)(bufferLoop.header+i)];
			}
			if(Verify_CRC8_Check_Sum(bufferdecode,5)){
				if(Verify_CRC16_Check_Sum(bufferdecode,bufferlength)){
					bufferLoop.header += bufferlength;
					
					cmdid = (bufferdecode[5]) | ((u16)bufferdecode[6]<<8);
					switch(cmdid){
						case 0x0001:
								judgeExtGameStateInfo(bufferdecode + 7, &judgeData.extGameState); break;
						case 0x0002:
								judgeExtGameResultInfo(bufferdecode + 7, &judgeData.extGameResult); break;								
						case 0x0003:
								judgeExtGameRobotHpInfo(bufferdecode + 7, &judgeData.extGameRobotHp); break;			            
						case 0x0101:
								judgeExtEventDataInfo(bufferdecode + 7, &judgeData.extEventData); break;	                                                 									
						case 0x0102:
								judgeExtSupplyProjectileActionInfo(bufferdecode + 7, &judgeData.extSupplyProjectileAction); break;									
						case 0x0103:
								judgeExtSupplyProjectileBookingInfo(bufferdecode + 7, &judgeData.extSupplyProjectileBooking); break;
						case 0x0104:
								judgeExtRefereeWarningInfo(bufferdecode + 7, &judgeData.extRefereeWarning); break;
						case 0x0201:
								judgeExtGameRobotStateInfo(bufferdecode + 7, &judgeData.extGameRobotState); break;			
						case 0x0202:
								judgeExtPowerHeatDataInfo(bufferdecode + 7, &judgeData.extPowerHeatData); 
								shooterHeatAbjust();
								TankShootHeatAbjust(); break;	
						case 0x0203:
								judgeExtGameRobotPosInfo(bufferdecode + 7, &judgeData.extGameRobotPos ); break; 		
						case 0x0204:		
								judgeExtBuffMuskInfo(bufferdecode + 7, &judgeData.extBuffMusk ); break;
						case 0x0205:
								judgeaerialRobotEnergyInfo(bufferdecode + 7, &judgeData.aerialRobotEnergy ); break;
						case 0x0206:
								judgeExtRobotHurtInfo(bufferdecode + 7, &judgeData.extRobotHurt ); break;
						case 0x0207:
								judgeExtShootDataInfo(bufferdecode + 7, &judgeData.extShootData );
								shooterHeatIncreasingTank();
								shooterHeatIncreasing(); break;
						case 0x0208:
								judgeExtBulletRemainingInfo(bufferdecode + 7, &judgeData.extBulletRemaining ); break;
						case 0x0301:
								//接收其他机器人（己方）发送过来的数据
								judgeOtherRobotSendDataInfo(bufferdecode + 13,&judgeData.extReceiveData); break;
						default : break;								
					}
				}
				else{
					if(distance>bufferlength){
						digitalIncreasing(&bufferLoop.header);
					}
				}
			}
			else{
				if(distance>bufferlength){
					digitalIncreasing(&bufferLoop.header);
				}
			}
		}				
	}
	for(i = 0;i < 255;i++){
		bufferdecode[i] = 0;
	}
	if(!(controlData.loops % 500)){
		//1000ms 1HZ 最大5台车同时显示客户端，cmd_id为0x0301频率最大10HZ
		if(ROBOT == INFANTRY_ID || ROBOT == TANK_ID || ROBOT == AUXILIARY_ID){		
			//客户端显示
			judgeExtShowData(&judgeData.extShowData,&judgeData.extGameRobotState); 
		}			
	}
	//1000ms 1HZ 最大5台车同时发出数据
	if(!((controlData.loops + 2)% 500)){
		if(ROBOT == UAV_ID){
			//机器人之间通信			
			robotExchangeData(&judgeData.extStudentData,&judgeData.extGameRobotState,send_to_robot);
		}
	}
}

void jugdeInit(void){
  Driver_Judge_Init(JUDGE_USARTX, JUDGE_USARTX_TX_PIN, JUDGE_USARTX_RX_PIN, JUDGE_USART_PRE_PRIORITY,JUDGE_USART_SUB_PRIORITY);
	judgeData.initFlag = true;
}
