#include "data_transmission.h"
#include "Driver_powerContol.h"
#include "power.h"

void datatransmissionInit(void){
#ifdef USE_WIRELESS
	wirelessInit();             			//���������ĳ�ʼ��
#endif
	
#ifdef USE_UPPER
	upperMonitorInit();               //��λ�����շ��ͳ�ʼ��
#endif
	
#ifdef USE_POWER_LIMIT
	sendPowerDataInit();
#endif
}





