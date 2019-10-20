#include "Driver_SK6812.h"
#include "Driver_Motor_Dshot.h"
#include "BSP.h"

uint16_t BIT_COMPARE_1 = 0;
uint16_t BIT_COMPARE_0 = 0;

SK6812Struct_t sk6812Data;

void getLedHsv(uint16_t index, hsvColor_t *color){
	*color = sk6812Data.ledColorBuffer[index];
}

rgbColor24bpp_t* hsvToRgb24(const hsvColor_t* c){
	static rgbColor24bpp_t r;
	uint16_t val = c->v;
	uint16_t sat = 255 - c->s;
	uint32_t base;
	uint16_t hue = c->h;

	if (sat == 0) { // Acromatic color (gray). Hue doesn't mind.
		r.rgb.r = val;
		r.rgb.g = val;
		r.rgb.b = val;
	} 
	else {
		base = ((255 - sat) * val) >> 8;
		switch (hue / 60) {
			case 0:
			r.rgb.r = val;
			r.rgb.g = (((val - base) * hue) / 60) + base;
			r.rgb.b = base;
			break;
			case 1:
			r.rgb.r = (((val - base) * (60 - (hue % 60))) / 60) + base;
			r.rgb.g = val;
			r.rgb.b = base;
			break;

			case 2:
			r.rgb.r = base;
			r.rgb.g = val;
			r.rgb.b = (((val - base) * (hue % 60)) / 60) + base;
			break;

			case 3:
			r.rgb.r = base;
			r.rgb.g = (((val - base) * (60 - (hue % 60))) / 60) + base;
			r.rgb.b = val;
			break;

			case 4:
			r.rgb.r = (((val - base) * (hue % 60)) / 60) + base;
			r.rgb.g = base;
			r.rgb.b = val;
			break;

			case 5:
			r.rgb.r = val;
			r.rgb.g = base;
			r.rgb.b = (((val - base) * (60 - (hue % 60))) / 60) + base;
			break;
		}
	}
	return &r;
}

//此处应用的DMA通道和Dshot是相互冲突的
void SK6812Config(void){
	uint16_t typePrescaler;
	uint16_t typePeriod;
	typePrescaler = (uint16_t)((MHZ_TO_HZ(84) / MHZ_TO_HZ(SK6812_TIMER_MHZ)) - 1);
	typePeriod  = (uint16_t)((MHZ_TO_HZ(84) / (typePrescaler + 1)) / SK6812_CARRIER_HZ);
	BIT_COMPARE_1 = (uint16_t)typePeriod / 11 * 6;
  BIT_COMPARE_0 = (uint16_t)typePeriod / 11 * 3;
	BSP_TIM_PWM_Init(TIM4,typePeriod,typePrescaler,NULL,NULL,BSP_GPIOD14,NULL);	
	pwmDshotDmaIrqnConfig(DMA1_Stream7_IRQn,1,0);
	BSP_DMA_SK6812_Init(&BSP_DMA_TIM4_CH3,(uint32_t)&TIM4->CCR3,(uint32_t)sk6812Data.ledStripDMABuffer,SK6812_DMA_BUFFER_SIZE);
}

static void fastUpdateLEDDMABuffer(rgbColor24bpp_t *color){
	uint32_t grb = (color->rgb.g << 16) | (color->rgb.r << 8) | (color->rgb.b);

	for (int8_t index = 23; index >= 0; index--) {
		sk6812Data.ledStripDMABuffer[sk6812Data.dmaBufferOffset++] = (grb & (1 << index)) ? BIT_COMPARE_1 : BIT_COMPARE_0;
	}
}

void SK6812_SendData(void){		
	DMA_SetCurrDataCounter(BSP_DMA_TIM4_CH3.DMA_Streamx,SK6812_DMA_BUFFER_SIZE);	
	TIM_SetCounter(TIM4, 0);				
	TIM_DMACmd(TIM4,TIM_DMA_CC3, ENABLE);
	DMA_Cmd(BSP_DMA_TIM4_CH3.DMA_Streamx, ENABLE);
}

void setOneLedHsv(uint16_t index, const hsvColor_t *color){
	sk6812Data.ledColorBuffer[index] = *color;
}

void setColor(hsvColor_t *color,uint16_t h,uint8_t s,uint8_t v){
	color->h = h;
	color->s = s;
	color->v = v;
}

void colorStdInit(){
	setColor(&sk6812Data.colorStd[COLOR_GREEN],120,0,2);
	setColor(&sk6812Data.colorStd[COLOR_RED],0,0,5);
	setColor(&sk6812Data.colorStd[COLOR_YELLOW],15,15,10);
	setColor(&sk6812Data.colorStd[COLOR_DARK],0,0,0);
	setColor(&sk6812Data.colorStd[COLOR_BLUE],220,0,2);
	setColor(&sk6812Data.colorStd[COLOR_PINK],305,0,2);
	setColor(&sk6812Data.colorStd[COLOR_WHITE],0,255,3);
}

void setAllLedColors(hsvColor_t *color){
	for(uint16_t index = 0;index < SK6812_LED_STRIP_LENGTH;index++)
		sk6812Data.ledColorBuffer[index] = *(color);
}

void SK6812UpdateStrip(void){
	static rgbColor24bpp_t *rgb24;
	static int16_t ledIndex;
	sk6812Data.dmaBufferOffset = 0;     // reset buffer memory index				重置缓冲存储器索引
	ledIndex = 0;                       // reset led index							重置LED的指数	
	// fill transmit buffer with correct compare values to achieve			用正确的比较值填充传输缓冲区来实现
	while (ledIndex < SK6812_LED_STRIP_LENGTH){
		rgb24 = hsvToRgb24(&sk6812Data.ledColorBuffer[ledIndex]);						//这句是把hsv转换成RGB的关键句
		fastUpdateLEDDMABuffer(rgb24);
 		ledIndex++;
	}
	SK6812_SendData();
}

void DMA1_Stream7_IRQHandler(void){																	//DMA1-7
	if(DMA_GetITStatus(DMA1_Stream7,DMA_IT_TCIF7)){
		DMA_Cmd(DMA1_Stream7, DISABLE);																	//失能
		TIM_DMACmd(TIM3,TIM_DMA_CC3, DISABLE);													//取消dma更新tim比较
		DMA_ClearITPendingBit(DMA1_Stream7, DMA_IT_TCIF7);							//清除IT_FLAG
	}
}

