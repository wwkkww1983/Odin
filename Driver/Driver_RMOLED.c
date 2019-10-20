#include "Driver_RMOLED.h"
#include "Driver_ADC.h"
#include "stdlib.h"
#include "GUI_OLEDFont.h"
#include "GUI_OLEDBmp.h"

oled_t oled;
//OLED���Դ�
//��Ÿ�ʽ����.
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 			   
  	    
//��SSD1106д��һ���ֽڡ�
//dat:Ҫд�������/����
//cmd:����/�����־ 0,��ʾ����;1,��ʾ����;
void OLED_WR_Byte(u8 dat,u8 cmd){	
	u8 i;			  
	if(cmd)
	  OLED_DC_Set();
	else 
	  OLED_DC_Clr();		  
	for(i=0;i<8;i++){			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_DC_Set();   	  
}   	

void OLED_Set_Pos(unsigned char x, unsigned char y) { 
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD); 
} 

//����OLED��ʾ    
void OLED_Display_On(void){
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}

//�ر�OLED��ʾ     
void OLED_Display_Off(void){
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC����
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		

//��������,������,������Ļ�Ǻ�ɫ��!��û����һ��!!!	  
void OLED_Clear(void)  {  
	uint8_t i,n;		    
	for(i=0;i<8;i++){  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //����ҳ��ַ��0~7��
		OLED_WR_Byte (0x00,OLED_CMD);      //������ʾλ�á��е͵�ַ
		OLED_WR_Byte (0x10,OLED_CMD);      //������ʾλ�á��иߵ�ַ   
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} //������ʾ
}

//��ָ��λ����ʾһ���ַ�,���������ַ�
//x:0~127
//y:0~63
//mode:0,������ʾ;1,������ʾ				 
//size:ѡ������ 16/12 
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr){      	
	unsigned char c=0,i=0;	
	c=chr-' ';//�õ�ƫ�ƺ��ֵ			
	if(x>Max_Column-1){
		x=0;y=y+2;
	}
	if(SIZE ==16){
		OLED_Set_Pos(x,y);	
		for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
		OLED_Set_Pos(x,y+1);
		for(i=0;i<8;i++)
			OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
	}
	else {	
		OLED_Set_Pos(x,y+1);
		for(i=0;i<6;i++)
		OLED_WR_Byte(F6x8[c][i],OLED_DATA);	
	}
}

//m^n����
u32 oled_pow(uint8_t m,uint8_t n){
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}		

//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);	 		  
void OLED_ShowNum(uint8_t x,uint8_t y,u32 num,uint8_t len,uint8_t size){         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++){
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1)){
			if(temp==0){
				OLED_ShowChar(x+(size/2)*t,y,' ');
				continue;
			}
			else enshow=1; 	 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0'); 
	}
}

//��ʾ2������
//x,y :�������	 
//len :���ֵ�λ��
//size:�����С
//mode:ģʽ	0,���ģʽ;1,����ģʽ
//num:��ֵ(0~4294967295);
void OLED_ShowNum2(uint8_t x,uint8_t y,u32 num,uint8_t len,uint8_t size){         	
	uint8_t t,temp;
	uint8_t enshow=0;						   
	for(t=0;t<len;t++){
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1)){
			if(temp==0){
				OLED_ShowChar(x+(size/2)*t,y,'0');
				continue;
			}else enshow=1;  
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0'); 
	}
}

uint8_t OLED_autoGetLen(uint32_t num,uint8_t lenMax){
  uint8_t i = 1,j = 1,len = 0;
	uint32_t numMax = 1;
	for(i = 0;i <= lenMax;i++){
		numMax *= 10;
	}
	for(j = 1;j <= numMax;j *= 10){
		if(num < 1){
			len = 1;
			j = numMax + 1;
		}
		else{
			if(num / j){
			len++;
			}
			else{
				j = numMax + 1;
			}
		}
	}
	return len;
}

void OLED_ShowNumFloat(uint8_t x,uint8_t y,float num,uint8_t precision,uint8_t size){         	
	uint32_t  integer = 0,decimals = 0,decimalsLenMax = 1;
	uint8_t	integerLen = 0,decimalsLen = 0,i = 0;
	integer = (uint32_t)num;
	integerLen = 5;
	//OLED_autoGetLen(integer,9);
	decimalsLen = precision;
	for(i = 0;i < decimalsLen;i ++){
		decimalsLenMax *= 10;
	}
	decimals = (uint32_t)((num - (float)integer)*decimalsLenMax);
	
	OLED_ShowNum(x,y,integer,integerLen,size);
	OLED_ShowChar(x+(size/2)*integerLen,y,'.');
	OLED_ShowNum2(x+(size/2)*(integerLen+1),y,decimals,decimalsLen,size);
} 
//��ʾһ���ַ��Ŵ�
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr){
	uint8_t j=0;
	while (chr[j]!='\0'){		
		OLED_ShowChar(x,y,chr[j]);
			x+=8;
		if(x>120){x=0;y+=2;}
			j++;
	}
}

//��ʾ����
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no){      			    
	uint8_t t,adder=0;
	OLED_Set_Pos(x,y);	
	for(t=0;t<16;t++){
			OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
			adder+=1;
	}	
	OLED_Set_Pos(x,y+1);	
	for(t=0;t<16;t++){	
		OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
		adder+=1;
	}					
}

/***********������������ʾ��ʾBMPͼƬ128��64��ʼ������(x,y),x�ķ�Χ0��127��yΪҳ�ķ�Χ0��7*****************/
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]){ 	
	unsigned int j=0;
	unsigned char x,y;
  
  if(y1%8==0) 
		y=y1/8;      
  else y=y1/8+1;
	for(y=y0;y<y1;y++){
		OLED_Set_Pos(x0,y);
    for(x=x0;x<x1;x++){      
	    OLED_WR_Byte(BMP[j++],OLED_DATA);	    	
	  }
	}
} 

void adcOLEDLowPassFilter(oled_t *oledADKey){
	getOLEDAdc_average(averageOLED_AD);
	oledADKey->keyOriginalValue = (float)averageOLED_AD[0];
}

void OLED_ADKeyScan(){
	adcOLEDLowPassFilter(&oled);
	if(oled.keyOriginalValue >= 0.00f && oled.keyOriginalValue < 300.00f){
		  oled.keyValue = OLED_SAVE;
	}	
	else if(oled.keyOriginalValue >= 500.00f && oled.keyOriginalValue < 1000.00f){
		oled.keyValue = OLED_BACK;
	}
	else if(oled.keyOriginalValue >= 1600.00f && oled.keyOriginalValue < 1900.00f){
		oled.keyValue = OLED_ENTER;
	}	
	else if(oled.keyOriginalValue >= 2100.00f && oled.keyOriginalValue < 2600.00f){
		oled.keyValue = OLED_UP;
	}		
	else if(oled.keyOriginalValue >= 3100.00f && oled.keyOriginalValue < 3400.0f){
		oled.keyValue = OLED_DOWN;
	}		
	else{
		oled.keyValue = NULL;
	} 			
}

//��ʼ��SSD1306					    
void OLED_Init(void){
	
	oled.keyOriginalValue = 0.0f;
	
	oled.keyValue = 0;
	
  BSP_GPIO_Init(BSP_GPIOB9,GPIO_Mode_Out_PP);
	BSP_GPIO_Init(BSP_GPIOB14,GPIO_Mode_Out_PP);
	BSP_GPIO_Init(BSP_GPIOB15,GPIO_Mode_Out_PP);
	BSP_GPIO_Init(BSP_GPIOD3,GPIO_Mode_Out_PP);

 	PBout(9) = 1;
	PBout(14) = 1;
	PBout(15) = 1;
	PDout(3) = 1;
 
  OLED_RST_Set();
	vTaskDelay(100);
	OLED_RST_Clr();
	vTaskDelay(100);
	OLED_RST_Set(); 
					  
	OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
	OLED_WR_Byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
	OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0���ҷ��� 0xa1����
	OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0���·��� 0xc8����
	OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_Byte(0x00,OLED_CMD);//-not offset
	OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
	OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
	OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
	OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_Byte(0x02,OLED_CMD);//
	OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
	OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
	OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
	
	OLED_WR_Byte(0xAF,OLED_CMD); /*display ON*/ 
	OLED_Clear();
	OLED_Set_Pos(0,0); 	
} 



