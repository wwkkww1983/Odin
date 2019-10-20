#ifndef __DRIVER_RMOLED_H
#define __DRIVER_RMOLED_H

#include "bsp.h"
#include "BSP_GPIO.h"
#include "stm32f4xx.h"
#include "FreeRTOS_board.h"

/************************OLED驱动*********************************************************/
//OLED模式设置

#define SIZE 12
#define XLevelL		0x00
#define XLevelH		0x10
#define Max_Column	128
#define Max_Row		64
#define	Brightness	0xFF 
#define X_WIDTH 	128
#define Y_WIDTH 	64	    						  
//-----------------OLED端口定义----------------  					   

#define OLED_RST_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_15)//RES
#define OLED_RST_Set() GPIO_SetBits(GPIOB,GPIO_Pin_15)

#define OLED_DC_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_14)//DC
#define OLED_DC_Set() GPIO_SetBits(GPIOB,GPIO_Pin_14)

#define OLED_SCLK_Clr() GPIO_ResetBits(GPIOD,GPIO_Pin_3)//CLK
#define OLED_SCLK_Set() GPIO_SetBits(GPIOD,GPIO_Pin_3)

#define OLED_SDIN_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_9)//DIN
#define OLED_SDIN_Set() GPIO_SetBits(GPIOB,GPIO_Pin_9)

 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

enum{
	OLED_NULL	= 0,                               //没按
	OLED_ENTER,															//确认键
	OLED_UP,																			//上键
	OLED_DOWN,																		//下键
	OLED_BACK,																			//返回键
	OLED_SAVE,                                    //保存键
	OLED_KEY_LIST,
	OLED_ALL_NEED,                        //用以清零等待时间
};

typedef struct
{
	uint8_t init;
	uint8_t keyValue;
	float keyOriginalValue;
	
}oled_t;

extern oled_t oled;

//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y, u8 *p);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
void OLED_ADKeyScan(void);
void OLED_ShowNumFloat(uint8_t x,uint8_t y,float num,uint8_t precision,uint8_t size);

#endif
