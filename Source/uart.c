/******************** (C) COPYRIGHT 2012 WildFire Team ***************************
 * 文件名  ：uart.c
 * 描述    ：将printf函数重定向到USART。这样就可以用printf函数将单片机的数据
 *           打印到PC上的超级终端或串口调试助手。         
 * 硬件连接：------------------------
 *          | PA9  - USART1(Tx)      |
 *          | PA10 - USART1(Rx)      |
 *           ------------------------
 * 库版本  ：ST3.5.0
 * 作者    ：yuan chao
**********************************************************************************/
#include "uart.h"
#include <stdarg.h>

uint8_t SendBuff[SENDBUFF_SIZE];
uint8_t ReceiveBuff[SENDBUFF_SIZE];

/*
 * 函数名：USART_Config
 * 描述  ：USART GPIO 配置,工作模式配置。115200 8-N-1
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	DMA_Config();
	
	/* 配置GPIO和USART时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/* 配置GPIO引脚 */
	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	/* 初始化串口参数 */
	USART_InitStructure.USART_BaudRate = 115200;   //波特率115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //数据位8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;       //停止位1位
	USART_InitStructure.USART_Parity = USART_Parity_No ;         //无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;		//打开Rx接收和Tx发送功能
	
	
	USART_Init(USART1, &USART_InitStructure); 			//初始化串口
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  // 开启 串口空闲IDEL 中断  TXE发送中断,TC传输完成中断,RXNE接收中断,PE奇偶错误中断,可以是多个   

	USART_Cmd(USART1, ENABLE);											//串口使能
	
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); // 开启串口DMA接收
}

/*
 * 函数名：DMA_Config
 * 描述  ：DMA 串口的初始化配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

	/*开启DMA时钟*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	
	
	
	/**********配置DMA发送串口*************/

	/*关DMA通道*/
		DMA_Cmd(DMA1_Channel4, DISABLE);     

	/*恢复缺省值*/
    DMA_DeInit(DMA1_Channel4);                                 

 	/*设置DMA源：内存地址&串口数据寄存器地址*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);;	   

	/*内存地址(要传输的变量的指针)  设置发送缓冲区首地址*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)SendBuff;
	
	/*设置数据传输方向：从内存到外设  内存缓冲区 -> 外设寄存器*/		
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	
	
	/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
    DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;
	
	/*外设地址不增*/	    
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	
	/*内存地址自增*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	
	/*外设数据单位 8bit，一个字节*/	
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	/*内存数据单位 8bit，一个字节*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
	
	/*DMA模式：一次传输，循环*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;	 
	
	/*优先级：中*/	
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
	
	/*禁止内存到内存的传输	*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	/*配置DMA1的4通道*/		   
    DMA_Init(DMA1_Channel4, &DMA_InitStructure); 	
		
	/* 清除DMA所有标志*/
	DMA_ClearFlag(DMA1_FLAG_GL4);                                 
		
	/*使能DMA发送通道*/
	DMA_Cmd(DMA1_Channel4,DISABLE);

  /*配置DMA发送完成后产生中断*/
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);  
	
	
	
	/**********配置DMA接收串口*************/
	
	/*关DMA通道*/
		DMA_Cmd(DMA1_Channel5, DISABLE);     

	/*恢复缺省值*/
    DMA_DeInit(DMA1_Channel5);                                 

 	/*设置DMA源：内存地址&串口数据寄存器地址*/
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);;	   

	/*内存地址(要传输的变量的指针)  设置接收缓冲区首地址*/
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ReceiveBuff;
	
	/*设置数据传输方向： 外设寄存器 ->  内存缓冲区 */		
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	
	
	/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
    DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE;
	
	/*外设地址不增*/	    
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
	
	/*内存地址自增*/
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
	
	/*外设数据单位 8bit，一个字节*/	
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	
	/*内存数据单位 8bit，一个字节*/
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	 
	
	/*DMA模式：一次传输，循环*/
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;	 
	
	/*优先级：中*/	
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
	
	/*禁止内存到内存的传输	*/
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	/*配置DMA1的5通道*/		   
    DMA_Init(DMA1_Channel5, &DMA_InitStructure); 	
		
	/* 清除DMA所有标志*/
	DMA_ClearFlag(DMA1_FLAG_GL5);                                 
		
	/*使能DMA接收通道*/
	DMA_Cmd(DMA1_Channel5,ENABLE);


}

/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到USART1 */
	USART_SendData(USART1, (uint8_t) ch);
//	while (!(USART1->SR & USART_FLAG_TXE));
	/* 等待发送完成 */
	while( USART_GetFlagStatus(USART1,USART_FLAG_TXE)== RESET);	
	return (ch);
}

/*
 *
 *
 */
void UART_Send_Array(USART_TypeDef* USARTx,uint8_t * send_array,unsigned char array_size)  
{
    
   uint8_t i;  
	 USART_ClearFlag(USART1, USART_FLAG_TC);//清除传输完成标志位
        for(i=0;i<array_size;i++)   //i肯定小于num 是正确  就执行
        {      
								USART_SendData(USARTx,*send_array++);        //通过库函数  发送数据
                while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);//等待发送完毕 
								
        }
        
}

/*
 * 函数名：itoa
 * 描述  ：将整形数据转换成字符串
 * 输入  ：-radix =10 表示10进制，其他结果为0
 *         -value 要转换的整形数
 *         -buf 转换后的字符串
 *         -radix = 10
 * 输出  ：无
 * 返回  ：无
 * 调用  ：被USART1_printf()调用
 */
static char *itoa(int value, char *string, int radix)
{
	int     i, d;
	int     flag = 0;
	char    *ptr = string;
	
	/* This implementation only works for decimal numbers. */
	if (radix != 10)
	{
	    *ptr = 0;
	    return string;
	}
	
	if (!value)
	{
	    *ptr++ = 0x30;
	    *ptr = 0;
	    return string;
	}
	
	/* if this is a negative value insert the minus sign. */
	if (value < 0)
	{
	    *ptr++ = '-';
	
	    /* Make the value positive. */
	    value *= -1;
	}
	
	for (i = 10000; i > 0; i /= 10)
	{
	    d = value / i;
	
	    if (d || flag)
	    {
	        *ptr++ = (char)(d + 0x30);
	        value -= (d * i);
	        flag = 1;
	    }
	}
	
	/* Null terminate the string. */
	*ptr = 0;
	
	return string;

} /* NCL_Itoa */

/*
 * 函数名：USART_printf
 * 描述  ：格式化输出，类似于C库中的printf，但这里没有用到C库
 * 输入  ：-USARTx 串口通道，这里只用到了串口1，即USART1
 *		     -Data   要发送到串口的内容的指针
 *			   -...    其他参数
 * 输出  ：无
 * 返回  ：无 
 * 调用  ：外部调用
 *         典型应用USART_printf( USART1, "\r\n this is a demo \r\n" );
 *            		 USART_printf( USART1, "\r\n %d \r\n", i );
 *            		 USART_printf( USART1, "\r\n %s \r\n", j );
 */
void USART_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
	int d;   
	char buf[16];
	
	va_list ap;
	va_start(ap, Data);
	
	while ( *Data != 0)     // 判断是否到达字符串结束符
	{				                          
		if ( *Data == 0x5c )  //'\'
	{									  
	switch ( *++Data )
	{
		case 'r':							          //回车符
			USART_SendData(USARTx, 0x0d);
			Data ++;
		break;
		
		case 'n':							          //换行符
			USART_SendData(USARTx, 0x0a);	
			Data ++;
		break;
		
		default:
			Data ++;
		break;
	}			 
	}
	else if ( *Data == '%')
	{									  //
	switch ( *++Data )
	{				
		case 's':										  //字符串
			s = va_arg(ap, const char *);
	for ( ; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
		Data++;
		break;
	
	case 'd':										//十进制
	d = va_arg(ap, int);
	itoa(d, buf, 10);
	for (s = buf; *s; s++) 
	{
		USART_SendData(USARTx,*s);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
	Data++;
	break;
		 default:
				Data++;
		    break;
	}		 
	} /* end of else if */
	else USART_SendData(USARTx, *Data++);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/

