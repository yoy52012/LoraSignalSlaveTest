#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

#define CLOCK_SECOND  1000     //1ms

void SysTick_Init(void);
void Dealy_us(uint32_t nTime);
#endif
