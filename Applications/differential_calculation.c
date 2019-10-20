#include "differential_calculation.h"
#include "math.h"
#include "NavAndData.h"
#include "SEGGER_RTT.h"

float frequency[70] = {0};
u8 rttUpBuffer[2048];
dataAcquisitionBufStruct_t buf;
float transferData[8] = 							//yaw云台速度环传函参数
{
1.0000,-0.7315,-0.4638,0.1953,
3.5904,-2.9756,-3.5828,2.9832
};

/*
@ param differential:	differential data
@ param currentData:	new data
@ return:							update data
*/
float differentialCal(differentialDataStruct_t *differential,float currentData){
	u8 loopNum = differential -> differentialLength;
	differential -> inputData[loopNum - 1] = currentData;
	
	/* H(z)计算更新 */
	differential -> outputData[loopNum - 1] = 0.0f;
	for(u8 i = 0;i < loopNum;i++){
		differential -> outputData[loopNum - 1] +=	\
		differential -> coefficient -> xCoefficient[i] * differential -> inputData[loopNum - 1 - i];
		
		if(i != loopNum - 1)
			differential -> outputData[loopNum - 1] -=	\
			differential -> coefficient -> yCoefficient[i + 1] * differential -> outputData[loopNum - 1 - i - 1];
	}

	/* x(n) 序列保存 */
	for(u8 i = 0;i < loopNum - 1;i++){
			differential->inputData[i] = differential -> inputData[i + 1];
	}
	
	/* y(n) 序列保存 */
	for(u8 i = 0;i < loopNum - 1;i++){
			differential->outputData[i] = differential -> outputData[i + 1];
	}
	return (differential->outputData[loopNum - 1]);
}

/*
@ param coefficientData:	coefficient data
@ param length:						length of coefficient data
@ return:									structure of differential data
*/
differentialDataStruct_t *differentialInit(float *coefficientData,u8 length){
	differentialDataStruct_t *differential;
	differential = (differentialDataStruct_t *)aqDataCalloc(1, sizeof(differentialDataStruct_t));
	differential -> differentialLength = length;
	
	/* differential coefficient initialization */
	differential -> coefficient = (coefficientStruct_t *)aqDataCalloc(1, sizeof(coefficientStruct_t));
	differential -> coefficient -> yCoefficient = coefficientData;
	differential -> coefficient -> xCoefficient = coefficientData + differential -> differentialLength;
	
	/* input and output data initialization */
	differential -> inputData = (float *)aqDataCalloc(differential -> differentialLength,sizeof(float));
	differential -> outputData = (float *)aqDataCalloc(differential -> differentialLength,sizeof(float));
	for(u8 i;i < differential -> differentialLength;i++){
		differential -> inputData[i] 	= 0.0f;
		differential -> outputData[i] = 0.0f;
	}
	
	digitalHi(&differential->initFlag);
	return differential;
}

/*
@ param inputData:	input data
@ param outputData:	feedback data
@ param amplitude:	amplitude of input data
@ return:						null
*/
void dataAcquisition(float *inputData,float outputData,float amplitude){
	static u8 TCounter = 0,stopFlag = 0;
	static u16 counter = 0,index = 0;
	if(!frequency[0])										//如果没初始化，则初始化频点
		dataAcquisitionInit();
	
	if(!stopFlag){
		if(counter++ < 500 / frequency[index]){
			*inputData = amplitude * (float)sin(2 * M_PI * frequency[index] * counter * 0.002f);
			buf.inputBuffer = *inputData * 1000;
			buf.outputBuffer = outputData * 1000;
			SEGGER_RTT_Write(1, &buf, sizeof(buf));
		}
		else{
			digitalClan(&counter);
			if(++TCounter == 20){						//20个周期到了
				digitalIncreasing(&index);		//索引下一个频点
				digitalClan(&TCounter);
			}
			if(!frequency[index])						//全部频点索引完毕
				digitalHi(&stopFlag);					//不在更新数据
		}
	}
	else{
		digitalClan(inputData);						//数据测试完毕
	}
}

void dataAcquisitionInit(void){
	u8 i = 0,cnt = 0;
	SEGGER_RTT_ConfigUpBuffer(1, "JScope_i2i2", rttUpBuffer, 2048, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	for(cnt = 0;frequency[i - 1] < 22;i++,cnt++){
		frequency[i] = 1 + 0.5 * cnt;
	}
	for(cnt = 0;frequency[i - 1] < 40;i++,cnt++){
		frequency[i] = 24 + 2 * cnt;
	}
	for(cnt = 0;frequency[i - 1] < 120;i++,cnt++){
		frequency[i] = 50 + 10 * cnt;
	}
	frequency[i++] = 200;
	frequency[i++] = 250;
	frequency[i++] = 0;
//	frequency[i++] = 333;
//	frequency[i++] = 500;
}
