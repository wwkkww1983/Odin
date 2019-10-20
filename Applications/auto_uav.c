#include "auto_uav.h"
#include "gimbal.h"
#include "shoot.h"
#include "vision.h"
#include "rc.h"
#include "config.h"
#include "imu.h"
AutoTaskStruct_t uavAutoData;

void uavTaskBegin(void){
	uavAutoData.taskState = EXECUTIONG;											//���Ϊ������
	digitalIncreasing(&uavAutoData.schedule);								//��ִ�����м�һ
}

void uavAutomaticAimUpdate(void){													//�������/�����������
	switch(uavAutoData.schedule){														//��schedule�����ӡ����ٺ��ж��������
		case 1: gimbalSwitch(DISABLE);												
						chassisSwitch(DISABLE);
						//����TX2����
						visionSendDataUpdate(TX2_DISTINGUISH_ARMOR,SMALL_BULLET);							
						digitalIncreasing(&(uavAutoData.schedule));	
						break;
		case 2: gimbalSwitch(ENABLE);	
						shootVisionInit();
						//����ģʽ����
						if(KB_PJEJUDGMENT){																//�Ҽ� ��Ԥ�д��
							visionData.prejudgFlag = true;	
							visionData.manualPitchBias += keyBoardCtrlData.pitchGyroTarget * MANUAL_PREJUDG_SCALE * 0.5f;
							visionData.manualYawBias += keyBoardCtrlData.yawGyroTarget * MANUAL_PREJUDG_SCALE;
						}
						else{																							//��� ����Ԥ�д��
							visionData.prejudgFlag = false;											
							digitalClan(&visionData.manualPitchBias);
							digitalClan(&visionData.manualYawBias);
						}
						//�����������
						if(!visionData.prejudgFlag){
							shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_17mm);
						}
						else{
							shootVisionUpdate(KB_NO_PJEJUDGMENT,TX2_DISTINGUISH_ARMOR,&shootData.fireFlag_17mm);	
						}										
						if(uavAutoData.breakSign){
							uavAutoData.schedule = 99;									//����д�����������
							visionData.prejudgFlag = false;
							visionData.fireFbd = false;
							shootDataReset();	
						}			
						break;
		case 99: uavAutoData.taskState = END_OF_EXECUTION; break;						//ֻ�е���99ʱ�����˳�
		default: uavAutoData.taskState = EXECUTION_ERROR; break;						//��������б���û�н��ȣ����������
	}
	uavAutoData.taskDuration += AUTO_TASK_DURATION;
}

void uavAutoTaskUpdate(void){
	if(uavAutoData.currentTask != UAV_MANUAL){							//�����п�ִ������
		if(uavAutoData.taskState == UNEXECUTED){							//�������ոտ�ʼִ��
			uavTaskBegin();																			//ִ�п�ʼ����
		}
		else if(uavAutoData.taskState == EXECUTIONG){					//�����ִ����
			switch(uavAutoData.currentTask){
				case UAV_AUTOMATIC_AIM: {
					uavAutomaticAimUpdate();
					break;
				}
				default: {																				//��������������ֱ�����³�ʼ���ṹ��
					autoDataInit(&uavAutoData); 
					break;
				}
			}
		}
		else{																									//���ִ����ϻ�ִ�д���������
			autoDataInit(&uavAutoData);
		}
	}
}



