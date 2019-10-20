#include "Driver_Power.h"
#include "Util.h"

firstOrderFilterData_t powerFirstOrderFilters[4];
powerRawDateStruct_t powerRawDate;
powerRealDateStruct_t powerRealDate;
static float currentBuffer[4][11];
static float currentSum[4];

double NUM[18] = {
  0.0001539807224874,-0.001633551718237, 0.008285871521075, -0.02655137141783,
    0.05976268341326, -0.09892004512411,   0.1209510267949,  -0.1024952672175,
    0.04044670396161,  0.04044670396161,  -0.1024952672175,   0.1209510267949,
   -0.09892004512411,  0.05976268341326, -0.02655137141783, 0.008285871521075,
  -0.001633551718237,0.0001539807224874
};

double DEN[18] = {
                   1,   -12.83384996104,    77.93650732675,   -297.3335694968,
      797.9129616887,   -1597.972651706,    2472.494945198,   -3018.357074163,
      2942.681165509,   -2303.842335957,    1448.206814835,   -726.6729407251,
      287.4544421962,   -87.75235536838,    19.96212762001,      -3.188300212,
     0.3191936459711, -0.01508036852769
};

void powerRealData(powerRawDateStruct_t *rawdata,powerRealDateStruct_t *realdata){
	volatile uint8_t i,j;
	for(i=0;i<4;i++){
		rawdata->current[i] = abs(rawdata->current[i]) < 48 ? 0 : rawdata->current[i]; //当轮子静止时，电流值直接当零
	}
	for(j=0;j<9;j++){
		for(i=0;i<4;i++)
			currentBuffer[i][j] = currentBuffer[i][j+1];
	}
	for(i=0;i<4;i++){
		currentBuffer[i][9] = rawdata->current[i];
	}
	for(i=0;i<4;i++){
		currentSum[i] = 0;
		for(j=0;j<10;j++)
			currentSum[i] += currentBuffer[i][j];
	}
	for(i=0;i<4;i++){
		currentBuffer[i][10] = currentSum[i] / 10.0f;
		realdata->current[i] = currentBuffer[i][10];
	}
//	realdata->Current1 = 4.0283203f*firstOrderFilter((float)rawdata->Current1 , &powerFirstOrderFilters[0]);//用低通会把电流最大值滤掉，得到的电流不真实
//	realdata->Current2 = 4.0283203f*firstOrderFilter((float)rawdata->Current2 , &powerFirstOrderFilters[1]);//如果用电机反馈的转矩电流可以用这个低通滤波	
//	realdata->Current3 = 4.0283203f*firstOrderFilter((float)rawdata->Current3 , &powerFirstOrderFilters[2]);	
//	realdata->Current4 = 4.0283203f*firstOrderFilter((float)rawdata->Current4 , &powerFirstOrderFilters[3]);	
	
}

void powerRawData(CanRxMsg *CAN_RX_Msg,powerRawDateStruct_t *rawdata,motorCanDataRecv_t *chassisData){	
	volatile uint8_t i;
	for(i=0;i<4;i++){
		rawdata->current[i] =  CAN_RX_Msg->Data[2 * i] | (CAN_RX_Msg->Data[2 * i + 1] << 8);
	}
	for(i=0;i<4;i++){	
		if(chassisData[i].speed < 0) 
			rawdata->current[i] = -rawdata->current[i];
	}
}

void powerInitFirstOrderFilter(void){
	float a;
/*********************************************************/

  a = 2.0f * 0.04f * 500.0f;

	powerFirstOrderFilters[0].gx1 = 1.0f / (1.0f + a);
	powerFirstOrderFilters[0].gx2 = 1.0f / (1.0f + a);
	powerFirstOrderFilters[0].gx3 = (1.0f - a) / (1.0f + a);
	powerFirstOrderFilters[0].previousInput  = 0.0f;
	powerFirstOrderFilters[0].previousOutput = 0.0f;
		
/******************************************************/
	a = 2.0f * 0.04f * 500.0f;

	powerFirstOrderFilters[1].gx1 = 1.0f / (1.0f + a);
	powerFirstOrderFilters[1].gx2 = 1.0f / (1.0f + a);
	powerFirstOrderFilters[1].gx3 = (1.0f - a) / (1.0f + a);
	powerFirstOrderFilters[1].previousInput  = 0.0f;
	powerFirstOrderFilters[1].previousOutput = 0.0f;		
/*****************************************************/
	a = 2.0f * 0.04f * 500.0f;

	powerFirstOrderFilters[2].gx1 = 1.0f / (1.0f + a);
	powerFirstOrderFilters[2].gx2 = 1.0f / (1.0f + a);
	powerFirstOrderFilters[2].gx3 = (1.0f - a) / (1.0f + a);
	powerFirstOrderFilters[2].previousInput  = 0.0f;
	powerFirstOrderFilters[2].previousOutput = 0.0f;		
		
/******************************************************/
	a = 2.0f * 0.04f * 500.0f;

	powerFirstOrderFilters[3].gx1 = 1.0f / (1.0f + a);
	powerFirstOrderFilters[3].gx2 = 1.0f / (1.0f + a);
	powerFirstOrderFilters[3].gx3 = (1.0f - a) / (1.0f + a);
	powerFirstOrderFilters[3].previousInput  = 0.0f;
	powerFirstOrderFilters[3].previousOutput = 0.0f;			
}
//切比雪夫低通滤波器 50HZ
double Chebyshev50HzLowPassFilter(Filter_t *F){     
	int i;
	for(i=17; i>0; i--)
	{
		F->ybuf[i] = F->ybuf[i-1]; 
		F->xbuf[i] = F->xbuf[i-1];
	}
	F->xbuf[0] = F->raw_value;
	F->ybuf[0] = NUM[0] * F->xbuf[0];
	for(i=1;i<18;i++)
	{
		F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
	}
	F->filtered_value = F->ybuf[0];
	return F->filtered_value;
}
