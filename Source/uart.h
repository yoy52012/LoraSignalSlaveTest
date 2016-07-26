#ifndef __USART_H
#define	__USART_H

#include "stm32f10x.h"
#include <stdio.h>

#define USART1_DR_Base  0x40013804
#define SENDBUFF_SIZE 128

void USART_Config(void);
void DMA_Config(void);

int fputc(int ch, FILE *f);
void USART_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
void UART_Send_Array(USART_TypeDef* USARTx,uint8_t * send_array,unsigned char array_size);
#endif /* __USART_H */
