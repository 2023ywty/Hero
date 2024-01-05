#include "test_task.h"
#include "led.h"
#include "buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "oscillography.h"
void led_task(void *pvParameters)
{
	while(1)
	{
		//led_green_toggle();
		vTaskDelay(500); 
	}
}  
void buzzer_task(void *pvParameters)
{
	while(1)
	{
		//buzzer_on(oscillographyData[0], oscillographyData[1]);
		vTaskDelay(500); 
		//buzzer_off();
		vTaskDelay(500);
	}
}
