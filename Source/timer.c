#include "timer.h"

void SysTick_Init(void)
{
	/*两个中断之间的脉冲数=SystemCoreClock / CLOCK_SECOND  脉冲时间周期为1/72us   SystemCoreClock=7200 0000  定时时间即中断时间=T=ticks×(1/f)
	 *SystemCoreClock / 1000     1ms中断一次
	 *SystemCoreClock / 100000   10us中断一次
	 *SystemCoreClock / 1000000  1us中断一次
	 */
	if (SysTick_Config(SystemCoreClock / CLOCK_SECOND))
    {
        while (1);
	}
	// 关闭滴答定时器
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;
}

volatile uint32_t TickCounter = 0;//1ms
static uint32_t TimingDelay = 0;

/*
 * 函数名：Delay_us
 * 描述  ：us延时程序,1ms为一个单位
 * 输入  ：- nTime
 * 输出  ：无
 * 调用  ：Delay_us( 1 ) 则实现的延时为 1 * 10ms = 10ms
 *       ：外部调用 
 */
void Delay_us(uint32_t nTime)
{ 
	TimingDelay = nTime;	

	// 使能滴答定时器  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;

	while(TimingDelay != 0);
}

void SysTick_Handler(void)
{
	TickCounter++;
	if(TimingDelay!=0x00)
	{
    TimingDelay--;
    //tx_delay++;
	}
}

