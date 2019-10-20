#include "board.h"

taskInit_t taskInitData;
void appInit(void *Parameters){
	taskInitData.eventGroups = NULL;	//�¼���־������
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	taskInitData.eventGroups = xEventGroupCreate();
	configInit();               			//����Ĭ�ϲ���
	clockCountInit();									//��ʱ��5��ʼ��,���ھ�ȷʱ��ڵ����
	controlInit();										//�ܿ���״̬���ĳ�ʼ��
	rcInit();                   			//�˻����ƽ����ĳ�ʼ��
	jugdeInit();											//����ϵͳ��ʼ��
	robotDistinguish();								//ʶ��������Թؼ����ݽ��г�ʼ��
  datatransmissionInit();           //���ݴ����ʼ��
	slaveSensorConfig();							//�ӻ�����������
	visionInit();											//�Ӿ�����
	imuInit();                  			//imu�������ĳ�ʼ��
	supervisorInit();           			//���״̬���ĳ�ʼ��
  vTaskDelete(NULL);								//ɾ����ǰ����
}




















