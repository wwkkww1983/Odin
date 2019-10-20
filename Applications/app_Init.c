#include "board.h"

taskInit_t taskInitData;
void appInit(void *Parameters){
	taskInitData.eventGroups = NULL;	//事件标志组清零
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	taskInitData.eventGroups = xEventGroupCreate();
	configInit();               			//加载默认参数
	clockCountInit();									//定时器5初始化,用于精确时间节点计算
	controlInit();										//总控制状态机的初始化
	rcInit();                   			//人机控制交互的初始化
	jugdeInit();											//裁判系统初始化
	robotDistinguish();								//识别机器并对关键数据进行初始化
  datatransmissionInit();           //数据传输初始化
	slaveSensorConfig();							//从机传感器数据
	visionInit();											//视觉任务
	imuInit();                  			//imu传感器的初始化
	supervisorInit();           			//监控状态机的初始化
  vTaskDelete(NULL);								//删除当前任务
}




















