#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f10x.h"

#include <stdio.h>
#include <stdlib.h>
//#define LORA                                        1         // [0: OFF, 1: ON] 

#ifdef  DEBUG_ENABLED
#define DEBUG(x) UART_PutStr(USART1, x);
#else   
#define DEBUG(x)  
#endif 

#define LEDG_OFF()			GPIO_SetBits(GPIOA, GPIO_Pin_0)
#define LEDG_ON()				GPIO_ResetBits(GPIOA, GPIO_Pin_0)
#define LEDG_INVERSE()	GPIO_WriteBit(GPIOA, GPIO_Pin_0, (BitAction)(!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_0)))

#define LEDR1_OFF()			GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define LEDR1_ON()			GPIO_ResetBits(GPIOA, GPIO_Pin_1)
#define LEDR1_INVERSE()	GPIO_WriteBit(GPIOA, GPIO_Pin_1, (BitAction)(!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1)))

#define LEDR2_OFF()			GPIO_SetBits(GPIOA, GPIO_Pin_2)
#define LEDR2_ON()			GPIO_ResetBits(GPIOA, GPIO_Pin_2)
#define LEDR2_INVERSE()	GPIO_WriteBit(GPIOA, GPIO_Pin_2, (BitAction)(!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_2)))

#define KEY1_INPUT			GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)
#define KEY2_INPUT			GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)

#define sce0   GPIO_ResetBits(GPIOA, GPIO_Pin_5)  		//

#define res0   GPIO_ResetBits(GPIOA, GPIO_Pin_3)  		//
#define res1   GPIO_SetBits(GPIOA, GPIO_Pin_3)

#define dc0    GPIO_ResetBits(GPIOA, GPIO_Pin_7)   		//
#define dc1    GPIO_SetBits(GPIOA, GPIO_Pin_7)

#define sdin0  GPIO_ResetBits(GPIOB, GPIO_Pin_1)   		//
#define sdin1  GPIO_SetBits(GPIOB, GPIO_Pin_1)

#define sclk0  GPIO_ResetBits(GPIOB, GPIO_Pin_11)   	//
#define sclk1  GPIO_SetBits(GPIOB, GPIO_Pin_11)

#define backled0  GPIO_ResetBits(GPIOB, GPIO_Pin_10)  //
#define backled1  GPIO_SetBits(GPIOB, GPIO_Pin_10)

#define FLASH_PAGE_SIZE ((uint16_t)0x400)

void appTxInit(void);
void appTransmitter(void);
void appReceiver(void);
void MCU_Init(void);
void GPIO_Configuration(void);
void USART_Configuration(void);
void USART_SendByte(USART_TypeDef* USARTx, u8 data);
void UART_PutStr(USART_TypeDef* USARTx, uint8_t *str);
void SPI_Configuration(void);
void RCC_Configuration(void);
void EXTI_Configuration(void);
void NVIC_Configuration(void);
void TIM2_Configuration(void);
void IWDG_Configuration(void);
void DrvSYS_Delay(u32 us);
void SysTick_Init(void);
void Delay(uint32_t delay);
void LongDelay(uint8_t delay);
void OnMaster(void);
void OnSlave(void);
void GetChipId(void);
void USART1_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
int Flash_Read(uint32_t iAddress, uint8_t *buf, uint32_t iNbrToRead) ;
int Flash_Write(uint32_t iAddress, uint8_t *buf, uint32_t iNbrToWrite);
void Print_Array(uint8_t *buf,uint32_t size,const char* s);
void Config_Handler(uint8_t *buf);
void Config_Handler1(uint8_t *buf);
void Print_Config(void);
void Send_Config(void);
void Send_Data(void);
char Get_Snr();
void Get_Rssi(char* buf);
#endif
