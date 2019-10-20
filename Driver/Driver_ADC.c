#include "Driver_ADC.h"

__IO uint16_t adc_raw_value[10][4];
adcFilterData_t adcFilters[4];
adcFilterData_t speedFilters[4];
__IO float average_dis[4];
__IO float averageOLED_AD[4];

void adcInit(void){
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStrcture;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_DMA2, ENABLE);//使能GPIOC时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)adc_raw_value;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//（外设）地址到DMA储存器
	DMA_InitStructure.DMA_BufferSize = 4 * 10;//
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//DMA非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//DMA存储增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//DMA存储半个字节
	DMA_InitStructure.DMA_MemoryDataSize =DMA_PeripheralDataSize_HalfWord;//储存器数据长度16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式；
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//优先级 （高）
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);//初始化DMA Stream0
	DMA_Cmd(DMA2_Stream0, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//不带上下拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	ADC_CommonInitStrcture.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStrcture.ADC_DMAAccessMode = ADC_DMAAccessMode_1;//???? DMA模式1
	ADC_CommonInitStrcture.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4（频率84/4M）
	ADC_CommonInitStrcture.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;//两个采样时间延时20个时钟
	
	ADC_CommonInit(&ADC_CommonInitStrcture);
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//  分辨率12位
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = 4;//四个通道

	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_144Cycles);
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);	
	ADC_DMACmd(ADC1,ENABLE);
	
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器
	ADC_SoftwareStartConv(ADC1);//软件开启ADC
}

void adcLowPassFilterInit(void){
		float a;
/**********************  ADC_1 *******************************/

    a = 2.0f * 0.3f * 1000.0f;

    adcFilters[ADC_1].gx1 = 1.0f / (1.0f + a);
    adcFilters[ADC_1].gx2 = 1.0f / (1.0f + a);
    adcFilters[ADC_1].gx3 = (1.0f - a) / (1.0f + a);
    adcFilters[ADC_1].previousInput  = 0.0f;
    adcFilters[ADC_1].previousOutput = 0.0f;
		
/**********************  ADC_2 *******************************/
    a = 2.0f * 0.3f * 1000.0f;

    adcFilters[ADC_2].gx1 = 1.0f / (1.0f + a);
    adcFilters[ADC_2].gx2 = 1.0f / (1.0f + a);
    adcFilters[ADC_2].gx3 = (1.0f - a) / (1.0f + a);
    adcFilters[ADC_2].previousInput  = 0.0f;
    adcFilters[ADC_2].previousOutput = 0.0f;		
/**********************  ADC_3 *******************************/
    a = 2.0f * 0.3f * 1000.0f;

    adcFilters[ADC_3].gx1 = 1.0f / (1.0f + a);
    adcFilters[ADC_3].gx2 = 1.0f / (1.0f + a);
    adcFilters[ADC_3].gx3 = (1.0f - a) / (1.0f + a);
    adcFilters[ADC_3].previousInput  = 0.0f;
    adcFilters[ADC_3].previousOutput = 0.0f;		
		
/**********************  ADC_4 *******************************/
    a = 2.0f * 0.3f * 1000.0f;

    adcFilters[ADC_4].gx1 = 1.0f / (1.0f + a);
    adcFilters[ADC_4].gx2 = 1.0f / (1.0f + a);
    adcFilters[ADC_4].gx3 = (1.0f - a) / (1.0f + a);
    adcFilters[ADC_4].previousInput  = 0.0f;
    adcFilters[ADC_4].previousOutput = 0.0f;		
		
		
		/**********************  SPEED_1 *******************************/

    a = 2.0f * 0.05f * 1000.0f;

    speedFilters[SPEED_1].gx1 = 1.0f / (1.0f + a);
    speedFilters[SPEED_1].gx2 = 1.0f / (1.0f + a);
    speedFilters[SPEED_1].gx3 = (1.0f - a) / (1.0f + a);
    speedFilters[SPEED_1].previousInput  = 0.0f;
    speedFilters[SPEED_1].previousOutput = 0.0f;
		
/**********************  SPEED_2 *******************************/
    a = 2.0f * 0.05f * 1000.0f;

    speedFilters[SPEED_2].gx1 = 1.0f / (1.0f + a);
    speedFilters[SPEED_2].gx2 = 1.0f / (1.0f + a);
    speedFilters[SPEED_2].gx3 = (1.0f - a) / (1.0f + a);
    speedFilters[SPEED_2].previousInput  = 0.0f;
    speedFilters[SPEED_2].previousOutput = 0.0f;		
/**********************  SPEED_3 *******************************/
    a = 2.0f * 0.05f * 1000.0f;

    speedFilters[SPEED_3].gx1 = 1.0f / (1.0f + a);
    speedFilters[SPEED_3].gx2 = 1.0f / (1.0f + a);
    speedFilters[SPEED_3].gx3 = (1.0f - a) / (1.0f + a);
    speedFilters[SPEED_3].previousInput  = 0.0f;
    speedFilters[SPEED_3].previousOutput = 0.0f;		
		
/**********************  SPEED_4 *******************************/
    a = 2.0f * 0.05f * 1000.0f;

    speedFilters[SPEED_4].gx1 = 1.0f / (1.0f + a);
    speedFilters[SPEED_4].gx2 = 1.0f / (1.0f + a);
    speedFilters[SPEED_4].gx3 = (1.0f - a) / (1.0f + a);
    speedFilters[SPEED_4].previousInput  = 0.0f;
    speedFilters[SPEED_4].previousOutput = 0.0f;		
}

float adcFilter(float input, struct adcFilterData *filterParameters){
    float output;

    output = filterParameters->gx1 * input +
             filterParameters->gx2 * filterParameters->previousInput -
             filterParameters->gx3 * filterParameters->previousOutput;

    filterParameters->previousInput  = input;
    filterParameters->previousOutput = output;

    return output;
}

void getadc_average(__IO float *real){
	float sum =0;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 10;j++)
		{
			sum += adc_raw_value[j][i];
		}
		real[i] = sum * 0.1f*0.0732421875f;//mm
		sum=0;
  }
}

float last_dis[4];
float temp_speed[4];

void adcLowPassFilter(float *distence, float *speed){
  getadc_average(average_dis);
	
  for(int i = 0; i<4; i++){
	  distence[i] = (adcFilter(average_dis[i] , &adcFilters[i]));//mm
		temp_speed[i]    = (distence[i] - last_dis[i])*1000;
		last_dis[i] = distence[i];
	}
	 for(int i = 0; i<4; i++){
	  speed[i] = (adcFilter(temp_speed[i] , &speedFilters[i]));//mm
	}
	
	
}

void getOLEDAdc_average(__IO float *real){
	float sum =0;
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 10;j++){
			sum += adc_raw_value[j][i];
		}
		real[i] = sum * 0.1f;//mm
		sum=0;
  }
}













