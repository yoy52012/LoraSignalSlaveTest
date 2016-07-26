#ifndef __ADC_H
#define	__ADC_H


#include "stm32f10x.h"
#include <stdlib.h>

void ADC1_GPIO_Config(void);
void ADC1_Mode_Config(void);
void ADC1_Init(void);

uint16_t Get_ADC_RandomSeed(void);
uint16_t Get_ADC_Random(void);

#endif /* __ADC_H */
