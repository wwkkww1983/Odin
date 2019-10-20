#include "cpu_utils.h"

void vApplicationIdleHook(void)
{
}


void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for(;;);
}


void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
