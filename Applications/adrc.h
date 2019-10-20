#ifndef __ADRC_H_
#define __ADRC_H_
#include "util.h"

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}adrcFilterTempStruct_t;

typedef struct
{
 const float a[3];
 const float b[3];
}adrcFilterStruct_t;

typedef struct
{
	/*****���Ź��ȹ���*******/
	float x1;//����΢����״̬��
	float x2;//����΢����״̬��΢����
	float *r;//ʱ��߶�
	float *h;//ADRCϵͳ����ʱ��
	float *N0;//����΢��������ٶȳ���h0=N*h

	float h0;
	float fh;//����΢�ּ��ٶȸ�����
	/*****����״̬�۲���*******/
	/******��ϵͳ���y������u�����ٹ���ϵͳ״̬���Ŷ�*****/
	float z1;
	float z2;
	float z3;//���ݿ��ƶ����������������ȡ���Ŷ���Ϣ
	float e;//ϵͳ״̬���
	float y;//ϵͳ�����
	float fe;
	float fe1;
	float *beta_01;
	float *beta_02;
	float *beta_03;
	float *b;


	/**********ϵͳ״̬������*********/
	float e0;//״̬��������
	float e1;//״̬ƫ��
	float e2;//״̬��΢����
	float u0;//���������ϵͳ���
	float u;//���Ŷ�����������
	float *b0;//�Ŷ�����

	/*********��һ�������ʽ*********/
	float *beta_0;//����
	float *beta_1;//��������ϲ���
	float *beta_2;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);
	/*********�ڶ��������ʽ*********/
	float *alpha1;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)
	float *alpha2;//0<alpha1<1<alpha2
	float *zeta;//���Զε����䳤��
	/*********�����������ʽ*********/
	float h1;//u0=-fhan(e1,e2,r,h1);
	float *N1;//����΢��������ٶȳ���h0=N*h
	/*********�����������ʽ*********/
	float *c;//u0=-fhan(e1,c*e2*e2,r,h1);

	float e2_lpf;
	adrcFilterTempStruct_t ADRC_LPF_Buffer;//��������ͨ�����������

	float TD_Input;
	float Last_TD_Input;
	float TD_Input_Div;

	float ESO_Input;
	float Last_ESO_Input;
	float ESO_Input_Div;
	
	float *oMax;
}adrcStruct_t;


adrcStruct_t *adrcInit(float *r, float *h, float *N0, float *beta_01, float *beta_02, float *beta_03, float *b0, float *beta_0, float *beta_1, float *beta_2, float *N1, float *c, float *alpha1, float *alpha2, float *zeta, float *b, float *oMax);
float adrcUpdate(adrcStruct_t *adrc,float expect_ADRC,float feedback_ADRC);
void adrcIntegrateReset(adrcStruct_t *adrc);

extern float ADRC_Unit[4][17];

#endif

