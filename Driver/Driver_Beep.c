#include "BSP.h"
#include "Util.h"
#include "Driver_beep.h"

int32_t BeepCode[]={									//C调
	6412,5714,5090,4813,4285,3818,3400, //低音
	3212,2906,2549,2406,2142,1909,1700,	//中音
	1606,1429,1274,1202,1071,954,850		//高音
};

void beepUpdate(uint16_t beepSound,uint16_t note){
	BEEP_PORT=constrainInt(beepSound,BEEP_MIN_NOISE,BEEP_MAX_NOISE);
	BEEP_PSC=constrainInt(note,BEEP_NOTE_MIN,BEEP_NOTE_MAX);
}

BeepSound_t beepNewSound[MUSIC_LIST]={
	{	
		{ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_RE,ALTO_RE,ALTO_RE,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI},    //ARMED
		{H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE,N_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE},16
	},
	{
		{ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_RE,ALTO_RE,ALTO_RE,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO},		//DISARMED
		{H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE,N_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE},16
	},
	{
		{HIGH_SOL,HIGH_SOL,ALTO_DO,ALTO_DO,HIGH_SOL,HIGH_SOL,ALTO_DO,ALTO_DO,HIGH_SOL},																												//IMU_CALI
		{H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE},9
	},	
	{
		{HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL},																										//FLASH_SAVE
		{H_NOISE,H_NOISE,H_NOISE,N_NOISE,N_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE},9	
	},
	{
		{HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL,HIGH_SOL},																					//MAG_CALI
		{H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE},10
	},
	{
		{ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO,ALTO_DO},																										//RADIO_LOSS
		{H_NOISE,H_NOISE,N_NOISE,N_NOISE,N_NOISE,H_NOISE,H_NOISE,N_NOISE,N_NOISE,N_NOISE},10
	},
	{
		{ALTO_MI,ALTO_MI},																																																										//TYPE_INFANTRY
		{H_NOISE,N_NOISE},2
	},
	{
		{ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI},																																																		//TYPE_TANK
		{H_NOISE,N_NOISE,H_NOISE,N_NOISE},4		
	},
	{
		{ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI},																																										//TYPE_AUXILIARY
		{H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE},6		
	},
	{
		{ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI},																																		//TYPE_SENTRY
		{H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE},8			
	},
	{
		{ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI},																										//TYPE_UAV
		{H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE},10		
	},
	{
		{ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI,ALTO_MI},																		//SMALLGIMBAL_ID
		{H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE,H_NOISE,N_NOISE},12		
	},
	{
		{BASS_RE,BASS_RE,BASS_RE,BASS_RE,BASS_RE,BASS_RE,BASS_RE,BASS_RE,BASS_RE,BASS_RE},																										//TYPE_NO_ID
		{H_NOISE,N_NOISE,H_NOISE,N_NOISE,N_NOISE,N_NOISE,H_NOISE,H_NOISE,H_NOISE,N_NOISE},10
	},
};

BeepSound_t frequencyCode;
void prepareBeepPacket(BeepSound_t *sound){
	uint16_t lenght=sound->lenght;
	for(uint16_t i=0;i<lenght;i++){
		frequencyCode.note[i]=BeepCode[sound->note[i]];
		frequencyCode.volume[i]=sound->volume[i];
	}
}

void beepUpdateTask(uint16_t commandState){
	static uint16_t musicNum=0;
	static BeepSound_t *musicSound;
	if(musicNum==0){
		if(commandState==QUIET)
			return;
		else{
			musicSound = &beepNewSound[commandState-1];
			prepareBeepPacket(musicSound);
			musicNum++;
		}
	}
	else{
		beepUpdate(frequencyCode.volume[musicNum-1],frequencyCode.note[musicNum-1]);
		musicNum++;
		if(musicNum > musicSound->lenght){
			musicNum=0;
		}
	}
}

LedData_t ledFuntionData[LED_LIST]={
	{LED_WORK_RED,LED_SLOW},         //硬件故障
	{LED_WORK_GREEN,LED_NORMAL},     //遥控器丢失
	{LED_WORK_RED,LED_FAST},         //磁力计校准
	{LED_WORK_GREEN,LED_SLOW},       //正常工作
	{LED_WORK_RED,LED_NORMAL},			 //开机温度等待、传感器异常
	{LED_WORK_GREEN,LED_FAST},       //IMU校准
	{LED_WORK_BLUE,LED_SLOW},				 //云台校准
	{LED_WORK_BLUE,LED_NORMAL},			 //YAW轴中值确认
	{LED_WORK_BLUE,LED_FAST},				 //机器人类型识别
};

void ledUpdateTask(uint16_t commandState){
	static uint16_t ledLoops=0;
	static uint8_t workFlag = 0;
	static LedData_t *ledWork;
	ledWork = &ledFuntionData[commandState-1];
	if(ledWork->frequency==0){
		LED_R = LED_G = LED_B = LED_DISABLE;
	}
	else if(!(ledLoops % ledWork->frequency)){
		workFlag =~ workFlag;
	}
	if(workFlag){
		if(ledWork->colour == LED_WORK_RED){
			LED_R = LED_ENABLE;
			LED_G = LED_B = LED_DISABLE;
		}
		else if(ledWork->colour == LED_WORK_GREEN){
			LED_G = LED_ENABLE;
			LED_R = LED_B = LED_DISABLE;
		}
		else if(ledWork->colour == LED_WORK_BLUE){
			LED_B = LED_ENABLE;
			LED_R = LED_G = LED_DISABLE;
		}
		else{
			LED_R = LED_G = LED_B = LED_DISABLE;
		}
	}
	else{
		LED_R = LED_G = LED_B = LED_DISABLE;
	}
	ledLoops++;
}

void rgbLedConfig(){
	BSP_GPIO_Init(BSP_GPIOD5,GPIO_Mode_Out_PP);				//R
	BSP_GPIO_Init(BSP_GPIOD4,GPIO_Mode_Out_PP);       //G
	BSP_GPIO_Init(BSP_GPIOD6,GPIO_Mode_Out_PP);       //B
	LED_R = LED_G = LED_B = LED_DISABLE;
}

void beepConfig(){
#ifdef USE_2019_A
	BSP_TIM_PWM_Init(TIM4,BEEP_LENGTH,BEEP_PRESCALER,NULL,BSP_GPIOD13,NULL,NULL);
#endif
#ifdef USE_2019_B
	BSP_TIM_PWM_Init(TIM10,BEEP_LENGTH,BEEP_PRESCALER,BSP_GPIOB8,NULL,NULL,NULL);
#endif
	beepUpdate(0,ALTO_DO);
	rgbLedConfig();
}

