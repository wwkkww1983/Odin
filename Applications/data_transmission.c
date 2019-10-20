#include "data_transmission.h"
#include "Driver_powerContol.h"
#include "power.h"

void datatransmissionInit(void){
#ifdef USE_WIRELESS
	wirelessInit();             			//无线数传的初始化
#endif
	
#ifdef USE_UPPER
	upperMonitorInit();               //上位机接收发送初始化
#endif
	
#ifdef USE_POWER_LIMIT
	sendPowerDataInit();
#endif
}





