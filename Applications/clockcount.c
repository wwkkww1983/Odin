#include "clockcount.h"

clockCountStruct_t  clockCountData;

double getClockCount(void){
	clockCountData.clockTick  = (double)(clockCountData.saveTimer*1000000 + TIM5->CNT)*1e-6f;
	return clockCountData.clockTick;	
}
