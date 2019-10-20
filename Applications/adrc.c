#include "adrc.h"
#include <math.h>

float ADRC_Unit[4][17]=
{
/*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
/*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta    b*/
	{10000000 ,0.002 , 5,               500,      80000,      2000,      20,    0.8,   10,      0.0003,       200,    5,    0.99,   1.2,    0.03,    10,10000},
	{100000 ,0.005 , 5,               300,      7000,      1000,      20,    0.8,   1.5,      0.0003,       200,    5,    0.95,   1.2,    0.03,    0.1,0},
	{300000 ,0.005 , 3,               300,      4000,      10000,      100,   0.2,   1.2,      0.0010,       5,    5,    0.8,   1.5,    0.03,    0.05,0},
};

/* 浮点型限幅 */
static float fConstrain(float amt, float low, float high){
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/* 浮点型符号函数 */
static int16_t fSign(float input){
	int16_t output=0;
	if(input > 1E-6f){
		output=1;
	}
	else if(input < -1E-6f){
		output=-1;
	}
	else{ 
		output=0;
	}
	return output;
}

static int16_t fsg(float x,float d){
	int16_t output=0;
	output = (fSign(x + d) - fSign(x - d)) / 2;
	return output;
}

/* 原点附近有连线性段的连续幂次函数 */
static float fal(float e,float alpha,float zeta){
	uint16_t s = 0;
	float falOutput = 0;
	s = fsg(e,zeta);
	falOutput = e * s / (powf(zeta,1 - alpha)) + powf(ABS(e),alpha) * fSign(e) * (1 - s);
	return falOutput;
}

adrcFilterStruct_t ADRC_Div_LPF_Parameter = {
	//500---20hz
	1,    -1.6474599810769768,    0.7008967811884026,
	0.013359200027856505,   0.02671840005571301,  0.013359200027856505
};

float adrcLPF(float curr_inputer,adrcFilterTempStruct_t *Buffer,adrcFilterStruct_t *Parameter){
	/* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
	Buffer->Input_Butter[2] = curr_inputer;
	/* Butterworth滤波 */
	Buffer->Output_Butter[2] =
	Parameter->b[0] * Buffer->Input_Butter[2]
	+ Parameter->b[1] * Buffer->Input_Butter[1]
	+ Parameter->b[2] * Buffer->Input_Butter[0]
	- Parameter->a[1] * Buffer->Output_Butter[1]
	- Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) 序列保存 */
	Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
	Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
	Buffer->Output_Butter[0] = Buffer->Output_Butter[1];
	Buffer->Output_Butter[1] = Buffer->Output_Butter[2];
	return (Buffer->Output_Butter[2]);
}

adrcStruct_t *adrcInit(float *r, float *h, float *N0, float *beta_01, float *beta_02, float *beta_03, float *b0,\
											float *beta_0, float *beta_1, float *beta_2, float *N1, float *c, \
											float *alpha1, float *alpha2, float *zeta, float *b, float *oMax){
	adrcStruct_t *adrc;

	adrc = (adrcStruct_t *)aqDataCalloc(1, sizeof(adrcStruct_t));

	adrc->r = r;
	adrc->h = h;
	adrc->N0 = N0;
	adrc->beta_01 = beta_01;
	adrc->beta_02 = beta_02;
	adrc->beta_03 = beta_03;
	adrc->b0 = b0;
	adrc->beta_0 = beta_0;
	adrc->beta_1 = beta_1;
	adrc->beta_2 = beta_2;
	adrc->N1 = N1;
	adrc->c = c;
	adrc->alpha1 = alpha1;
	adrc->alpha2 = alpha2;
	adrc->zeta = zeta;
	adrc->b = b;
	adrc->oMax = oMax;

	return adrc;
}

//ADRC最速跟踪微分器TD，改进的算法fhan
static void fhanADRC(adrcStruct_t *adrc,float expect){		//安排ADRC过度过程
	float d = 0,a0 = 0,y = 0,a1 = 0,a2 = 0,a = 0;
	float x1_delta = 0;																			//ADRC状态跟踪误差项
	x1_delta = adrc->x1 - expect;														//用x1-v(k)替代x1得到离散更新公式
	adrc->h0 = *adrc->N0 * *adrc->h;													//用h0替代h，解决最速跟踪微分器速度超调问题
	d  = *adrc->r * adrc->h0 * adrc->h0;											//d=rh^2;
	a0 = adrc->h0 * adrc->x2;																//a0=h*x2
	y  = x1_delta + a0;																			//y=x1+a0
	a1 = sqrt(d * (d + 8 * ABS(y)));												//a1=sqrt(d*(d+8*ABS(y))])
	a2 = a0 + fSign(y) * (a1 - d)/2;												//a2=a0+sign(y)*(a1-d)/2;
	a  = (a0 + y) * fsg(y,d) + a2 * (1-fsg(y,d));
	adrc->fh = -*adrc->r * (a / d) * fsg(a,d)
						 -*adrc->r * fSign(a) * (1-fsg(a,d));					//得到最速微分加速度跟踪量
	adrc->x1 += *adrc->h * adrc->x2;													//跟新最速跟踪状态量x1
	adrc->x2 += *adrc->h * adrc->fh;													//跟新最速跟踪状态量微分x2
}

/************扩张状态观测器********************/
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
static void esoADRC(adrcStruct_t *adrc){
  adrc->e = adrc->z1 - adrc->y;														//状态误差

  adrc->fe 	= fal(adrc->e,0.50,*adrc->N1 * *adrc->h);				//非线性函数，提取跟踪状态与当前状态误差
  adrc->fe1 = fal(adrc->e,0.25,*adrc->N1 * *adrc->h);

  /*************扩展状态量更新**********/
  adrc->z1 += *adrc->h * (adrc->z2 - *adrc->beta_01 * adrc->e);
  adrc->z2 += *adrc->h * (adrc->z3 - *adrc->beta_02 * adrc->fe + *adrc->b * adrc->u);
	//ESO估计状态加速度信号，进行扰动补偿，传统MEMS陀螺仪漂移较大，估计会产生漂移
  adrc->z3 += *adrc->h * (-*adrc->beta_03 * adrc->fe1);
}

__weak void linearConbination(adrcStruct_t *adrc){				//此处效仿STM32库去除函数前的static防止编译器出现警告，并且设为弱函数
	adrc->u0 = *adrc->beta_0 * adrc->e0
					 + *adrc->beta_1 * adrc->e1
					 + *adrc->beta_2 * adrc->e2;
}

static void nolinearConbination(adrcStruct_t *adrc){
	adrc->e2 = fConstrain(adrc->e2_lpf,-15000,15000);
	adrc->u0 = *adrc->beta_1 * fal(adrc->e1,*adrc->alpha1,*adrc->zeta)
					 + *adrc->beta_2 * fal(adrc->e2,*adrc->alpha2,*adrc->zeta);
}

float adrcUpdate(adrcStruct_t *adrc,float expect,float feedback){
	/* 计算TD跟踪微分器参数 */
	adrc->Last_TD_Input = adrc->TD_Input;
	adrc->TD_Input 			= expect;
	adrc->TD_Input_Div 	= (adrc->TD_Input - adrc->Last_TD_Input) / *adrc->h;
	/* 计算ESO扩张状态观测器参数 */
	adrc->Last_ESO_Input = adrc->ESO_Input;
	adrc->ESO_Input 		 = feedback;
	adrc->ESO_Input_Div  = (adrc->ESO_Input - adrc->Last_ESO_Input) / *adrc->h;
	
	/*自抗扰控制器第1步*/
	/********
			**
			**
			**
			**
			**
	 ********/
	/*****
	安排过度过程，输入为期望给定，
	由TD跟踪微分器得到：
	过度期望信号x1，过度期望微分信号x2
	******/
	fhanADRC(adrc,expect);

	/*自抗扰控制器第2步*/
	/********
					*
					*
		 ****
	 *
	 *
	 ********/
	/************系统输出值为反馈量，状态反馈，ESO扩张状态观测器的输入*********/
	adrc->y = feedback;
	/*****
	扩张状态观测器，得到反馈信号的扩张状态：
	1、状态信号z1；
	2、状态速度信号z2；
	3、状态加速度信号z3。
	其中z1、z2用于作为状态反馈与TD微分跟踪器得到的x1,x2做差后，
	经过非线性函数映射，乘以beta系数后，
	组合得到未加入状态加速度估计扰动补偿的原始控制量u
	*********/
	esoADRC(adrc);//低成本MEMS会产生漂移，扩展出来的z3此项会漂移，目前暂时未想到办法解决，未用到z3
	/*自抗扰控制器第3步*/
	/********
				 **
			 **
		 **
			 **
				 **
	 ********/
	/********状态误差反馈率***/
	adrc->e0 += adrc->e1 * *adrc->h;																										//状态积分项
	adrc->e1 = adrc->x1 - adrc->z1;																										//状态偏差项
	adrc->e2 = adrc->x2 - adrc->z2;																										//状态微分项
	adrc->e2_lpf = adrcLPF(adrc->e2,&adrc->ADRC_LPF_Buffer,&ADRC_Div_LPF_Parameter);	//巴特沃斯低通后得到的微分项,20hz
	nolinearConbination(adrc);																												//非线性组合
//	if(feedback < 200 && feedback > -200)
//		adrc->b0 = adrc->b * 2;
//	else
//		adrc->b0 = adrc->b;
	/**********扰动补偿*******/
	adrc->u = adrc->u0 - adrc->z3 / *adrc->b0;
	//adrc->u+=Constrain_Float(adrc->beta_0*adrc->e0,-150,150);
	//由于MEMS传感器漂移比较严重，当beta_03取值比较大时，长时间z3漂移比较大，目前不加入扰动补偿控制量
	//adrc->u=Constrain_Float(adrc->u0,-450,450);不加入扰动补偿
	adrc->u = fConstrain(adrc->u,-*adrc->oMax,*adrc->oMax);															//加入扰动补偿后的输出
	return adrc->u;

}

void adrcIntegrateReset(adrcStruct_t *adrc){
	adrc->e0 = 0.0f;
}
