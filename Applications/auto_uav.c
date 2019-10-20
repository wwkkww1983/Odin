#include "auto_uav.h"
#include "gimbal.h"
#include "shoot.h"
#include "vision.h"
#include "rc.h"
#include "config.h"
#include "imu.h"
AutoTaskStruct_t uavAutoData;

void uavTaskBegin(void){
	uavAutoData.taskState = EXECUTIONG;											//标记为进行中
	digitalIncreasing(&uavAutoData.schedule);								//给执行序列加一
}

void uavAutomaticAimUpdate(void){													//辅助打击/自瞄任务更新
	switch(uavAutoData.schedule){														//用schedule来增加、减少和判断任务进度
		case 1: gimbalSwitch(DISABLE);												
						chassisSwitch(DISABLE);
						//配置TX2数据
						visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);							
						digitalIncreasing(&(uavAutoData.schedule));	
						break;
		case 2: gimbalSwitch(ENABLE);	
						shootVisionInit();
						//自瞄模式决策
						if(KB_PJEJUDGMENT){																//右键 带预判打击
							visionData.prejudgFlag = true;	
							visionData.manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE * 0.5f;
							visionData.manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
						}
						else{																							//左键 不带预判打击
							visionData.prejudgFlag = false;											
							digitalClan(&visionData.manualPitchBias);
							digitalClan(&visionData.manualYawBias);
						}
						//自瞄射击决策
						if(!visionData.prejudgFlag){
							shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_17mm);
						}
						else{
							shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_17mm);	
						}										
						if(uavAutoData.breakSign){
							uavAutoData.schedule = 99;									//如果有打断则结束任务
							visionData.prejudgFlag = false;
							visionData.fireFbd = false;
							shootDataReset();	
						}			
						break;
		case 99: uavAutoData.taskState = END_OF_EXECUTION; break;						//只有调到99时才能退出
		default: uavAutoData.taskState = EXECUTION_ERROR; break;						//如果到达列表中没有进度，则任务出错
	}
	uavAutoData.taskDuration += AUTO_TASK_DURATION;
}

void uavAutoTaskUpdate(void){
	if(uavAutoData.currentTask != UAV_MANUAL){							//必须有可执行任务
		if(uavAutoData.taskState == UNEXECUTED){							//如果任务刚刚开始执行
			uavTaskBegin();																			//执行开始序列
		}
		else if(uavAutoData.taskState == EXECUTIONG){					//如果在执行中
			switch(uavAutoData.currentTask){
				case UAV_AUTOMATIC_AIM: {
					uavAutomaticAimUpdate();
					break;
				}
				default: {																				//如果是其他命令，则直接重新初始化结构体
					autoDataInit(&uavAutoData); 
					break;
				}
			}
		}
		else{																									//如果执行完毕或执行错误，则重新
			autoDataInit(&uavAutoData);
		}
	}
}



