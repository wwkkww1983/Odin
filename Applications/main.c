#include "board.h"

volatile uint32_t ulHighFrequencyTimerTicks = 0UL;
int main(void){
	xTaskCreate(appInit,"Init",INIT_SIZE,NULL,INIT_PRIORITIES,NULL);
	vTaskStartScheduler();
	printf("Task run failed.");
	return 0;
}
