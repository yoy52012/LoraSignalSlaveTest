#include "adc.h"

uint16_t ADC_ConvertedValue;

/*
 * 函数名：ADC1_GPIO_Config
 * 描述  ：使能ADC1和DMA1的时钟，初始化PC.01
 * 输入  : 无
 * 输出  ：无
 * 调用  ：内部调用
 */
 
 void ADC1_GPIO_Config(void)
 {
 		GPIO_InitTypeDef GPIO_InitStructure;
 		
 		/* Enable ADC1 and GPIOC clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB, ENABLE);
 		
 		/* Configure PC.01  as analog input */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
		GPIO_Init(GPIOB, &GPIO_InitStructure);				// PB0,输入时不用设置速率
 }
 
 /* 函数名：ADC1_Mode_Config
 * 描述  ：配置ADC1的工作模式为MDA模式
 * 输入  : 无
 * 输出  ：无
 * 调用  ：内部调用
 */
 void ADC1_Mode_Config(void)
 {
 		ADC_InitTypeDef ADC_InitStructure;
 		
 		/* ADC1 configuration */
	
		ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//独立ADC模式
		ADC_InitStructure.ADC_ScanConvMode = DISABLE ; 	 //禁止扫描模式，扫描模式用于多通道采集
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//开启连续转换模式，即不停地进行ADC转换
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
		ADC_InitStructure.ADC_NbrOfChannel = 1;	 	//要转换的通道数目1
		ADC_Init(ADC1, &ADC_InitStructure);
 	
 		/*配置ADC时钟，为PCLK2的8分频，即9Hz*/
		RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
		/*配置ADC1的通道11为55.	5个采样周期，序列为1 */ 
		ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_1Cycles5);
		/* Enable ADC1 */
		ADC_Cmd(ADC1, ENABLE);
		
		/*复位校准寄存器 */   
		ADC_ResetCalibration(ADC1);
		/*等待校准寄存器复位完成 */
		while(ADC_GetResetCalibrationStatus(ADC1));
	
		/* ADC校准 */
		ADC_StartCalibration(ADC1);
		/* 等待校准完成*/
		while(ADC_GetCalibrationStatus(ADC1));
	
		/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ 
		//ADC_SoftwareStartConvCmd(ADC1, ENABLE);
 }
 
 /*
 * 函数名：ADC1_Init
 * 描述  ：无
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
 void ADC1_Init(void)
 {
	 ADC1_GPIO_Config();
	 ADC1_Mode_Config();
 }
 
 
 /*
 * 函数名 : Get_ADC_RandomSeed
 * 描述   : 获取ADC悬空引脚输入作为随机数种子
 * 输入   :	无
 * 输出   : 随机数种子
 */
 uint16_t Get_ADC_RandomSeed(void)
 {
 	uint8_t Count;
 	uint16_t ADC_RandomSeed = 0;
 	
 	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
 	for(Count = 0;Count<4;Count++)
 	{
 		while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
 		ADC_RandomSeed <<=4;
 		ADC_RandomSeed += ADC_GetConversionValue(ADC1) & 0x00ff;
 		
 	}
 	ADC_SoftwareStartConvCmd(ADC1, DISABLE);
 	return ADC_RandomSeed;
 }
 
 
  /*
 * 函数名 : Get_ADC_Random
 * 描述   : 获取随机数
 * 输入   :	无
 * 输出   : 随机数
 */
 uint16_t Get_ADC_Random(void)
 {
 	srand(Get_ADC_RandomSeed());
 	return rand();
 }
 
 
