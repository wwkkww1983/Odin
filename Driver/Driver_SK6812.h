#ifndef __DRIVER_SK6812_H
#define __DRIVER_SK6812_H

#include "BSP_PWM.h"
#include "BSP_DMA.h"

#define SK6812_LED_STRIP_LENGTH    6			//LEDÊýÁ¿
#define SK6812_BITS_PER_LED        24
// for 50us delay
#define SK6812_DELAY_BUFFER_LENGTH 42

#define SK6812_DATA_BUFFER_SIZE    (SK6812_BITS_PER_LED * SK6812_LED_STRIP_LENGTH)
// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes)
#define SK6812_DMA_BUFFER_SIZE     (SK6812_DATA_BUFFER_SIZE + SK6812_DELAY_BUFFER_LENGTH)

#define SK6812_TIMER_MHZ           84
#define SK6812_CARRIER_HZ          800000

typedef enum {
	RGB_RED = 0,
	RGB_GREEN,
	RGB_BLUE,
	RGB_COLOR_COMPONENT_COUNT
} rgbColorComponent_e;

struct rgbColor24bpp_s {
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

typedef union {
	struct rgbColor24bpp_s rgb;
	uint8_t raw[RGB_COLOR_COMPONENT_COUNT];
} rgbColor24bpp_t;

#define HSV_HUE_MAX 359
#define HSV_SATURATION_MAX 255
#define HSV_VALUE_MAX 255

typedef enum {
	HSV_HUE = 0,
	HSV_SATURATION,
	HSV_VALUE
} hsvColorComponent_e;

typedef struct hsvColor_s {
	uint16_t h; // 0 - 359
	uint8_t s; // 0 - 255
	uint8_t v; // 0 - 255
} hsvColor_t;

enum {
	COLOR_DARK = 0,
	COLOR_RED,
	COLOR_YELLOW,
	COLOR_GREEN,
	COLOR_BLUE,
	COLOR_PINK,
	COLOR_WHITE,
	COLOR_LIST				//the nunmber of colors
};

typedef struct{
	uint16_t dmaBufferOffset;
	hsvColor_t ledColorBuffer[SK6812_LED_STRIP_LENGTH];
	uint16_t ledStripDMABuffer[SK6812_DMA_BUFFER_SIZE];
	hsvColor_t colorStd[COLOR_LIST];
}SK6812Struct_t;

void setColor(hsvColor_t *color,uint16_t h,uint8_t s,uint8_t v);
void colorStdInit(void);
void setOneLedHsv(uint16_t index, const hsvColor_t *color);
void setAllLedColors(hsvColor_t *color);
void SK6812Config(void);
void SK6812UpdateStrip(void);
void SK6812_SendData(void);

extern SK6812Struct_t sk6812Data;

#endif
