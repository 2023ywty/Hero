#ifndef BUZZER_H
#define BUZZER_H
#include "main.h"
extern void buzzer_init(uint16_t arr, uint16_t psc);
void buzzer_on(uint16_t pwm);
extern void buzzer_off(void);

void TIM8_Init(uint16_t arr, uint16_t psc);

#endif
