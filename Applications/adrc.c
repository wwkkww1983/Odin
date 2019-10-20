#include "adrc.h"
#include <math.h>

float ADRC_Unit[4][17]=
{
/*TD����΢����   �Ľ�����TD,h0=N*h      ����״̬�۲���ESO           �Ŷ�����     ���������*/
/*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta    b*/
	{10000000 ,0.002 , 5,               500,      80000,      2000,      20,    0.8,   10,      0.0003,       200,    5,    0.99,   1.2,    0.03,    10,10000},
	{100000 ,0.005 , 5,               300,      7000,      1000,      20,    0.8,   1.5,      0.0003,       200,    5,    0.95,   1.2,    0.03,    0.1,0},
	{300000 ,0.005 , 3,               300,      4000,      10000,      100,   0.2,   1.2,      0.0010,       5,    5,    0.8,   1.5,    0.03,    0.05,0},
};

/* �������޷� */
static float fConstrain(float amt, float low, float high){
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

/* �����ͷ��ź��� */
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

/* ԭ�㸽���������Զε������ݴκ��� */
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
	/* ���ٶȼ�Butterworth�˲� */
	/* ��ȡ����x(n) */
	Buffer->Input_Butter[2] = curr_inputer;
	/* Butterworth�˲� */
	Buffer->Output_Butter[2] =
	Parameter->b[0] * Buffer->Input_Butter[2]
	+ Parameter->b[1] * Buffer->Input_Butter[1]
	+ Parameter->b[2] * Buffer->Input_Butter[0]
	- Parameter->a[1] * Buffer->Output_Butter[1]
	- Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) ���б��� */
	Buffer->Input_Butter[0] = Buffer->Input_Butter[1];
	Buffer->Input_Butter[1] = Buffer->Input_Butter[2];
	/* y(n) ���б��� */
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

//ADRC���ٸ���΢����TD���Ľ����㷨fhan
static void fhanADRC(adrcStruct_t *adrc,float expect){		//����ADRC���ȹ���
	float d = 0,a0 = 0,y = 0,a1 = 0,a2 = 0,a = 0;
	float x1_delta = 0;																			//ADRC״̬���������
	x1_delta = adrc->x1 - expect;														//��x1-v(k)���x1�õ���ɢ���¹�ʽ
	adrc->h0 = *adrc->N0 * *adrc->h;													//��h0���h��������ٸ���΢�����ٶȳ�������
	d  = *adrc->r * adrc->h0 * adrc->h0;											//d=rh^2;
	a0 = adrc->h0 * adrc->x2;																//a0=h*x2
	y  = x1_delta + a0;																			//y=x1+a0
	a1 = sqrt(d * (d + 8 * ABS(y)));												//a1=sqrt(d*(d+8*ABS(y))])
	a2 = a0 + fSign(y) * (a1 - d)/2;												//a2=a0+sign(y)*(a1-d)/2;
	a  = (a0 + y) * fsg(y,d) + a2 * (1-fsg(y,d));
	adrc->fh = -*adrc->r * (a / d) * fsg(a,d)
						 -*adrc->r * fSign(a) * (1-fsg(a,d));					//�õ�����΢�ּ��ٶȸ�����
	adrc->x1 += *adrc->h * adrc->x2;													//�������ٸ���״̬��x1
	adrc->x2 += *adrc->h * adrc->fh;													//�������ٸ���״̬��΢��x2
}

/************����״̬�۲���********************/
//״̬�۲�������beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
static void esoADRC(adrcStruct_t *adrc){
  adrc->e = adrc->z1 - adrc->y;														//״̬���

  adrc->fe 	= fal(adrc->e,0.50,*adrc->N1 * *adrc->h);				//�����Ժ�������ȡ����״̬�뵱ǰ״̬���
  adrc->fe1 = fal(adrc->e,0.25,*adrc->N1 * *adrc->h);

  /*************��չ״̬������**********/
  adrc->z1 += *adrc->h * (adrc->z2 - *adrc->beta_01 * adrc->e);
  adrc->z2 += *adrc->h * (adrc->z3 - *adrc->beta_02 * adrc->fe + *adrc->b * adrc->u);
	//ESO����״̬���ٶ��źţ������Ŷ���������ͳMEMS������Ư�ƽϴ󣬹��ƻ����Ư��
  adrc->z3 += *adrc->h * (-*adrc->beta_03 * adrc->fe1);
}

__weak void linearConbination(adrcStruct_t *adrc){				//�˴�Ч��STM32��ȥ������ǰ��static��ֹ���������־��棬������Ϊ������
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
	/* ����TD����΢�������� */
	adrc->Last_TD_Input = adrc->TD_Input;
	adrc->TD_Input 			= expect;
	adrc->TD_Input_Div 	= (adrc->TD_Input - adrc->Last_TD_Input) / *adrc->h;
	/* ����ESO����״̬�۲������� */
	adrc->Last_ESO_Input = adrc->ESO_Input;
	adrc->ESO_Input 		 = feedback;
	adrc->ESO_Input_Div  = (adrc->ESO_Input - adrc->Last_ESO_Input) / *adrc->h;
	
	/*�Կ��ſ�������1��*/
	/********
			**
			**
			**
			**
			**
	 ********/
	/*****
	���Ź��ȹ��̣�����Ϊ����������
	��TD����΢�����õ���
	���������ź�x1����������΢���ź�x2
	******/
	fhanADRC(adrc,expect);

	/*�Կ��ſ�������2��*/
	/********
					*
					*
		 ****
	 *
	 *
	 ********/
	/************ϵͳ���ֵΪ��������״̬������ESO����״̬�۲���������*********/
	adrc->y = feedback;
	/*****
	����״̬�۲������õ������źŵ�����״̬��
	1��״̬�ź�z1��
	2��״̬�ٶ��ź�z2��
	3��״̬���ٶ��ź�z3��
	����z1��z2������Ϊ״̬������TD΢�ָ������õ���x1,x2�����
	���������Ժ���ӳ�䣬����betaϵ����
	��ϵõ�δ����״̬���ٶȹ����Ŷ�������ԭʼ������u
	*********/
	esoADRC(adrc);//�ͳɱ�MEMS�����Ư�ƣ���չ������z3�����Ư�ƣ�Ŀǰ��ʱδ�뵽�취�����δ�õ�z3
	/*�Կ��ſ�������3��*/
	/********
				 **
			 **
		 **
			 **
				 **
	 ********/
	/********״̬������***/
	adrc->e0 += adrc->e1 * *adrc->h;																										//״̬������
	adrc->e1 = adrc->x1 - adrc->z1;																										//״̬ƫ����
	adrc->e2 = adrc->x2 - adrc->z2;																										//״̬΢����
	adrc->e2_lpf = adrcLPF(adrc->e2,&adrc->ADRC_LPF_Buffer,&ADRC_Div_LPF_Parameter);	//������˹��ͨ��õ���΢����,20hz
	nolinearConbination(adrc);																												//���������
//	if(feedback < 200 && feedback > -200)
//		adrc->b0 = adrc->b * 2;
//	else
//		adrc->b0 = adrc->b;
	/**********�Ŷ�����*******/
	adrc->u = adrc->u0 - adrc->z3 / *adrc->b0;
	//adrc->u+=Constrain_Float(adrc->beta_0*adrc->e0,-150,150);
	//����MEMS������Ư�ƱȽ����أ���beta_03ȡֵ�Ƚϴ�ʱ����ʱ��z3Ư�ƱȽϴ�Ŀǰ�������Ŷ�����������
	//adrc->u=Constrain_Float(adrc->u0,-450,450);�������Ŷ�����
	adrc->u = fConstrain(adrc->u,-*adrc->oMax,*adrc->oMax);															//�����Ŷ�����������
	return adrc->u;

}

void adrcIntegrateReset(adrcStruct_t *adrc){
	adrc->e0 = 0.0f;
}
