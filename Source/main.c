#include "main.h"
#include "sx1276.h"
#include "sx1276-Hal.h"
#include "spi.h"
#include "radio.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "sx1276-LoRa.h"
#include "sx1276-LoRaMisc.h"
#include "uart.h"
#include "adc.h"

volatile uint32_t TickCounter = 0;  //系统时钟计数

tRadioDriver *Radio = NULL;

#define TX_PERIOD							50            //发送时间间隔
#define PAYLOD_LEN						128	          //发送数据包的长度

static uint8_t TxBuf[PAYLOD_LEN ];				
static uint16_t TxBufSize = PAYLOD_LEN ;

bool receive_flag = false;
static uint8_t send_flag = 1;

uint16_t num=0;

extern uint8_t SendBuff[SENDBUFF_SIZE];
extern uint8_t ReceiveBuff[SENDBUFF_SIZE];

//uint8_t mac[6]={0xFF,0xCA,0xFE,0xBA,0xBE,0xFF};  //网关mac地址
//uint8_t mac[6]={0x01,0x02,0x03,0x04,0x05,0x07};  //节点mac地址


uint8_t gateway_mac[6]={0x0};
uint8_t net_id[2]={0x0};

const uint8_t PingMsg[]="PING";
const uint8_t PongMsg[]="PONG";

const	char *s1="GATEWAY_MAC : ";
const	char *s2="NET_ID : ";

extern tLoRaSettings LoRaSettings;
int main(void)
{				
	
	MCU_Init();
	LEDG_OFF();
	LEDR1_OFF();
	LEDR2_OFF();
	memset( TxBuf, 0, ( size_t )TxBufSize);
  //GetChipId();    //获取芯片ID
	
	//Flash_Read((uint32_t)0x0800FFF0,gateway_mac,6);
	//Flash_Read((uint32_t)0x0800FFE0,net_id,2);
	
	//Print_Array(gateway_mac,6,s1);
	//Print_Array(net_id,2,s2);
	
	//初始化Radio驱动
	Radio = RadioDriverInit( ); 
  Radio->Init( );
	Radio->StartRx( );
	//printf("The radio have been init!!\r\n");
	
	while(1)
	{	
			OnSlave();
			if(receive_flag==true)
			{
				receive_flag=false;
				Config_Handler(ReceiveBuff);
				
			}
       
//			IWDG_ReloadCounter();  
	
	
// SignalBw [0: 7.8kHz, 1: 10.4 kHz, 2: 15.6 kHz, 3: 20.8 kHz, 4: 31.2 kHz,5: 41.6 kHz, 6: 62.5 kHz, 7: 125 kHz, 8: 250 kHz, 9: 500 kHz, other: Reserved]
	//SX1276LoRaSetSignalBandwidth(8);
//SpreadingFactor [6: 64, 7: 128, 8: 256, 9: 512, 10: 1024, 11: 2048, 12: 4096  chips]
//			SX1276LoRaSetSpreadingFactor( uint8_t factor );
// ErrorCoding [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
//			SX1276LoRaSetErrorCoding( uint8_t value );

	
////	  DMA_Cmd(DMA1_Channel4,ENABLE);
////		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	

	}

}

void Config_Handler(uint8_t *buf)
{
	uint8_t bw=buf[0];
	uint8_t sp=buf[1];
	uint8_t ec=buf[2];
	
	if(bw>=1&&bw<=9)
	{
		SX1276LoRaSetSignalBandwidth(bw);
	}
	
	if(sp>=7&&sp<=12)
	{
		SX1276LoRaSetSpreadingFactor(sp);
	}
	
	if(ec>=1&&ec<=4)
	{
		SX1276LoRaSetErrorCoding(ec);	
	}
	
	//Send_Data();
}


void Config_Handler1(uint8_t *buf)
{
	uint8_t command=buf[0];
	
	switch(command)
	{
		case 0x01:
					Config_Handler(buf);
					break;
		case 0x02:
					Send_Config();
					break;
		case 0x03:
					Send_Data();
					break;
		default:
					break;
	}
	
	Print_Config();
}

void Print_Config(void)
{
	printf("LoRaSettings.SignalBw=%d\r\n",LoRaSettings.SignalBw);
	printf("LoRaSettings.SpreadingFactor=%d\r\n",LoRaSettings.SpreadingFactor);
	printf("LoRaSettings.ErrorCoding=%d\r\n",LoRaSettings.ErrorCoding);
	printf("\r\n");
}

/*
 * uint32_t SX1276LoRaGetRFFrequency()
 * int8_t SX1276LoRaGetRFPower()
 * uint8_t SX1276LoRaGetSignalBandwidth()
 * uint8_t SX1276LoRaGetSpreadingFactor()
 * uint8_t SX1276LoRaGetErrorCoding()
 * BOOL SX1276LoRaGetPacketCrcOn()
 * BOOL SX1276LoRaGetImplicitHeaderOn()
 * BOOL SX1276LoRaGetRxSingleOn()
 * BOOL SX1276LoRaGetFreqHopOn()
 * uint8_t SX1276LoRaGetHopPeriod()
 * uint32_t SX1276LoRaGetTxPacketTimeout()
 * uint32_t SX1276LoRaGetRxPacketTimeout()
 * uint8_t SX1276LoRaGetPayloadLength()
 * BOOL SX1276LoRaGetPa20dBm()
 *
 *
 * uint8_t SX1276LoRaGetPacketRxGain()
 * int8_t  SX1276LoRaGetPacketSnr() [dB]  ：信噪比（接收信号质量）
 * double  SX1276LoRaGetPacketRssi() [dBm] : 接收信号强度 
*/
void Send_Config(void)
{
	uint8_t send_buf[4];
	//memset( send_buf, 0, ( size_t )send_buf);
	send_buf[0]=0xf2;
	send_buf[1]=SX1276LoRaGetSignalBandwidth();
	send_buf[2]=SX1276LoRaGetSpreadingFactor();
	send_buf[3]=SX1276LoRaGetErrorCoding();
	//printf("%s",send_buf);
	UART_Send_Array(USART1,send_buf,4);
}

void Send_Data(void)
{
	
	char *rssi;
	Get_Rssi(rssi);
	
	char snr=Get_Snr();
	
	char send_buf[12];
	//send_buf[0]=0xf3;
	send_buf[0]=snr;
	memcpy(send_buf+1,rssi,8);
	send_buf[9]=SX1276LoRaGetSignalBandwidth();
	send_buf[10]=SX1276LoRaGetSpreadingFactor();
	send_buf[11]=SX1276LoRaGetErrorCoding();
//	send_buf[10]=0xff;
//	send_buf[11]=0x0d;
	
	//UART_Send_Array(USART1,send_buf,11);
	printf("%s",send_buf);
	
}

char Get_Snr(void)
{
	char snr=SX1276LoRaGetPacketSnr();
	 
	return snr; 
	
}

void Get_Rssi(char* buf)
{
	double rssi=SX1276LoRaGetPacketRssi();
	sprintf(buf,"%f",rssi);
	
}

void MCU_Init(void)
{
	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	//TIM2_Configuration();
	SysTick_Init();
	SpiInit();
	USART_Config();
	ADC1_Init();
//	IWDG_Configuration();
}	
	
/*
 * GPIO引脚配置
 */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/* OUTPUT for GPIOA */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_8|GPIO_Pin_15|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;																
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* INPUT for GPIOB */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* OUTPUT for GPIOB */	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3|GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;																
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinRemapConfig( GPIO_Remap_SWJ_JTAGDisable, ENABLE );
}


/*
 * RCC时钟频率引脚配置
 */
void RCC_Configuration(void)
{
	//定义错误状态变量
	ErrorStatus HSEStartUpStatus;
	
	//将RCC寄存器重新设置为默认值
	RCC_DeInit();
	
	//打开外部高速时钟晶振
	RCC_HSEConfig(RCC_HSE_ON);
	
	//等待外部高速时钟晶振工作
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	if(HSEStartUpStatus != SUCCESS) return;
	
	//使能预取指缓存
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
	
	//设置FLASH代码延时
	FLASH_SetLatency(FLASH_Latency_2);

	//设置AHB时钟(HCLK)为系统时钟
	RCC_HCLKConfig(RCC_SYSCLK_Div1); 

	//设置高速AHB时钟(APB2)为HCLK时钟
	RCC_PCLK2Config(RCC_HCLK_Div1); 

  //设置低速AHB时钟(APB1)为HCLK的2分频
	RCC_PCLK1Config(RCC_HCLK_Div2);	

  //设置PLL时钟，为HSE的12倍频 PLLCLK = 12MHz/2 * 12 = 72 MHz
	RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_12);

	//使能PLL
	RCC_PLLCmd(ENABLE);

	//等待PLL准备就绪
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

	//设置PLL为系统时钟源
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	//判断PLL是否是系统时钟
	while(RCC_GetSYSCLKSource() != 0x08);
}

/*
 * 中断配置
 */
void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
#ifdef  VECT_TAB_RAM 
	/* Set the Vector Table base location at 0x20000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */ 
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	
//#ifdef TX_DEV	
//	
//	//选择中断分组1
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
//	
//	//选择TIM2的中断通道
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//	
//	//抢占式中断优先级设置为0
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
//	
//	//响应式中断优先级设置为0
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
//	
//	//使能中断
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);	
//#endif

/* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
 
   /* 配置DMA发送通道中断 */
	 NVIC_InitStructure.NVIC_IRQChannel= DMA1_Channel4_IRQn;   
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;   
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority= 1;   
	 NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;   
	 NVIC_Init(&NVIC_InitStructure); 
 
   /* 配置串口中断 */
	 NVIC_InitStructure.NVIC_IRQChannel= USART1_IRQn;  
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 1;  
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority= 0;  
	 NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE;  
	 NVIC_Init(&NVIC_InitStructure);     
}


void TIM2_Configuration(void)																								
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;

	//允许TIM2时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//重新将Timer设置为缺省值
	TIM_DeInit(TIM2);	
	
	//设置预分频系数 计数器时钟为72MHz/7200 = 1kHz
	TIM_TimeBaseStructure.TIM_Prescaler		 	= 7200 - 1;
	
	//设置定时周期0-99；
	TIM_TimeBaseStructure.TIM_Period			= 100 - 1; 
	
	//设置计数器模式：向上计数
	TIM_TimeBaseStructure.TIM_CounterMode		= TIM_CounterMode_Up;
	
	//设置时钟分割
	TIM_TimeBaseStructure.TIM_ClockDivision	 	= TIM_CKD_DIV1;

  //将配置应用到TIM2中	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	 //清除溢出中断标志
	TIM_ClearFlag(TIM2, TIM_FLAG_Update); 
	
	//开启TIM2的中断
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
	
	TIM_Cmd(TIM2, ENABLE);																			
}

/*
 *独立看门狗初始化
 */
void IWDG_Configuration(void)
{
	 //启动寄存器读写
	 IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	 
	 //40K时钟32分频
   IWDG_SetPrescaler(IWDG_Prescaler_64);
	 
	 //计数器数值
   IWDG_SetReload(1875);                
	 
	 //重启计数器
   IWDG_ReloadCounter();            
	 
	 //启动看门狗
   IWDG_Enable();                       
}


void SysTick_Handler(void)
{
  TickCounter++;
}


void SysTick_Init(void)
{
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    while (1);
  }
}

/* delay  毫秒 ms */
void Delay(uint32_t delay)
{
    // Wait delay ms
    uint32_t startTick = TickCounter;
    while( ( TickCounter - startTick ) < delay );   
}


/* delay  秒 s*/
void LongDelay(uint8_t delay)
{
    uint32_t longDelay;
    uint32_t startTick;
    longDelay = delay * 1000;

    // Wait delay s
    startTick = TickCounter;
    while( ( TickCounter - startTick ) < longDelay );   
}

void OnMaster(void)
{
	switch(Radio->Process())
	{
		case RF_RX_TIMEOUT:
				 LEDR2_INVERSE();
				 printf("time_out!\r\n");
				 break;
		case RF_RX_DONE:
				 Radio->GetRxPacket(TxBuf,TxBufSize);
				 LEDR1_INVERSE();
				 break;
		case RF_TX_DONE:
				 LEDG_INVERSE();
				 Delay(500);
				 Radio->SetTxPacket(TxBuf,TxBufSize);
				 break;
		 default: 
				 break;
		}

}

void OnSlave(void)
{
	switch(Radio->Process())
	{
		case RF_RX_DONE:
				 Radio->GetRxPacket(TxBuf,TxBufSize);
				 LEDR1_INVERSE();
				 Send_Data();
				 //printf("Rsn:  %d dB\r\n",SX1276LoRaGetPacketSnr());
		     //printf("Rsii: %f dBm\r\n",SX1276LoRaGetPacketRssi());
//				 for(uint8_t i=0;i<TxBufSize;i++)
//				 {
//						printf("%x  ",TxBuf[i]);
//				 }
//				 printf("\r\n");
				 break;
		case RF_TX_DONE:
				 LEDG_INVERSE();
				 memset( TxBuf, 0, ( size_t )TxBufSize);
				 Radio->StartRx();
				 break;
		default:
				 break;
	}
}

/*
 *获取芯片唯一Id
 */
void GetChipId(void)
{
    u32 ChipUniqueID[3];
		ChipUniqueID[2] = *(__IO u32 *)(0X1FFFF7E8);  // 低字节
		ChipUniqueID[1] = *(__IO u32 *)(0X1FFFF7EC); // 
		ChipUniqueID[0] = *(__IO u32 *)(0X1FFFF7F0); // 高字节
		printf("######## The ID of the Chip is : %x-%x-%x\r\n",ChipUniqueID[0],ChipUniqueID[1],ChipUniqueID[2]);
	
//		uint8_t data[4]={0xb1,0xb2,0xb4,0xb5};
//	  Flash_Write((uint32_t)0x0800FFF0,data,4);
//	  uint8_t data1[4];
//		Flash_Read((uint32_t)0x0800FFF0,data1,4);
//		for(int i=0;i<4;i++)
//		{
//			printf("%x",data1[i]);
//		}
}

void USART1_IRQHandler(void)
{
  
    if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET)
    {
       num = USART1->SR;
       num = USART1->DR; //清USART_IT_IDLE标志 关键的一点，就是要读取SR,DR，将USART_IT_IDLE标志给清掉
			
       DMA_Cmd(DMA1_Channel5,DISABLE);    //关闭DMA通道
			 DMA_ClearFlag(DMA1_FLAG_GL5);     //清除DMA标志位
       num = 128 -  DMA_GetCurrDataCounter(DMA1_Channel5);      //得到真正接收数据个数  
       DMA1_Channel5->CNDTR=128;       //重新设置接收数据个数   
       DMA_Cmd(DMA1_Channel5,ENABLE);  //开启DMA
       receive_flag = true;           //接收数据标志位置1
			
			 LEDR2_INVERSE();
				 
    }
}

void DMA1_Channel4_IRQHandler(void)
{	
//判断是否为DMA发送完成中断
   if(DMA_GetFlagStatus(DMA1_FLAG_TC4)==SET) 
   {  
   //LED关闭  
		LEDG_INVERSE();	
	 
		DMA_Cmd(DMA1_Channel4,DISABLE);    //关闭DMA通道
		DMA_ClearFlag(DMA1_FLAG_TC4|DMA1_FLAG_GL4); 		//清除DMA标志位
		send_flag = 0;
	}	
}


/*************************************************************************************************************/

uint16_t Flash_Write_Without_check(uint32_t iAddress, uint8_t *buf, uint16_t iNumByteToWrite)
{
    uint16_t i;
    volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
    i = 0;

    // FLASH_UnlockBank1();
    while((i < iNumByteToWrite) && (FLASHStatus == FLASH_COMPLETE))
    {
        FLASHStatus = FLASH_ProgramHalfWord(iAddress, *(uint16_t*)buf);
        i = i+2;
        iAddress = iAddress + 2;
        buf = buf + 2;
    }

    return iNumByteToWrite;
}

/**
* @brief Programs a half word at a specified Option Byte Data address.
* @note This function can be used for all STM32F10x devices.
* @param Address: specifies the address to be programmed.
* @param buf: specifies the data to be programmed.
* @param iNbrToWrite: the number to write into flash
* @retval if success return the number to write, -1 if error
*
*/
int Flash_Write(uint32_t iAddress, uint8_t *buf, uint32_t iNbrToWrite)
{
    /* Unlock the Flash Bank1 Program Erase controller */
    uint32_t secpos;
    uint32_t iNumByteToWrite = iNbrToWrite;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i = 0;
    uint8_t tmp[FLASH_PAGE_SIZE];

    FLASH_UnlockBank1();
    secpos=iAddress & (~(FLASH_PAGE_SIZE -1 )) ;//扇区地址
    secoff=iAddress & (FLASH_PAGE_SIZE -1); //在扇区内的偏移
    secremain=FLASH_PAGE_SIZE-secoff; //扇区剩余空间大小
    volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;

    if(iNumByteToWrite<=secremain) secremain = iNumByteToWrite;//不大于4096个字节

    while( 1 )
    {
        Flash_Read(secpos, tmp, FLASH_PAGE_SIZE); //读出整个扇区
        for(i=0;i<secremain;i++)
        { //校验数据
            if(tmp[secoff+i]!=0XFF)break; //需要擦除
        }
        if(i<secremain)
        { //需要擦除
            FLASHStatus = FLASH_ErasePage(secpos); //擦除这个扇区
            if(FLASHStatus != FLASH_COMPLETE)
            return -1;
            for(i=0;i<secremain;i++) //复制
            {
                tmp[i+secoff]=buf[i];
            }
            Flash_Write_Without_check(secpos ,tmp ,FLASH_PAGE_SIZE);//写入整个扇区
        }
        else
        {
            Flash_Write_Without_check(iAddress,buf,secremain);//写已经擦除了的,直接写入扇区剩余区间.
        }

        if(iNumByteToWrite==secremain) //写入结束了
        {
            break;
        }
        else
        {
            secpos += FLASH_PAGE_SIZE;
            secoff = 0;//偏移位置为0
            buf += secremain; //指针偏移
            iAddress += secremain;//写地址偏移
            iNumByteToWrite -= secremain; //字节数递减
            if(iNumByteToWrite>FLASH_PAGE_SIZE) secremain=FLASH_PAGE_SIZE;//下一个扇区还是写不完
            else secremain = iNumByteToWrite; //下一个扇区可以写完了
        }
    }
    FLASH_LockBank1();
    return iNbrToWrite;
}


/**
* @brief Programs a half word at a specified Option Byte Data address.
* @note This function can be used for all STM32F10x devices.
* @param Address: specifies the address to be programmed.
* @param buf: specifies the data to be programmed.
* @param iNbrToWrite: the byte number to read from flash
* @retval if success return the number to read, without error
*
*/
int Flash_Read(uint32_t iAddress, uint8_t *buf, uint32_t iNbrToRead)
{
    int i = 0;
    while(i < iNbrToRead)
    {
        *(buf + i) = *(__IO uint8_t*) iAddress++;
        i++;
    }
    return i;
}

void Print_Array(uint8_t *buf,uint32_t size,const char* s)
{
	printf("%s",s);
	for (int i = 0;i < size;i++)
	{
		printf("%x ",buf[i]);
	}
	printf("\r\n");
}

//#ifdef TX_DEV
//void TIM2_IRQHandler(void)
//{
//	static uint16_t cnt = 0;

//	//检测是否发生溢出更新事件
//	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
//  {
//		//清除TIM2的中断待处理位
//		TIM_ClearITPendingBit(TIM2 , TIM_IT_Update);
//		
//		//  50/1000=0.05s
//		if (++cnt == TX_PERIOD )
//		{
//			Radio->SetTxPacket( TxBuf, TxBufSize );
//			LEDG_INVERSE();
//			cnt = 0;
//		}
//	}
//}
//#endif	

