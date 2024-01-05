#ifndef DELAY_H
#define DELAY_H

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "main.h"
#include "stm32f4xx.h"

void delay_init(void);
void delay_us(uint16_t nus);
void delay_ms(uint16_t nms);
void smartDelayMs(uint16_t ms);

#endif
