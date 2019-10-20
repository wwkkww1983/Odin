/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright ? 2011-2014  Bill Nesbitt
*/

#include "srcdkf.h"
#include "aq_math.h"
#include "util.h"
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

/*-------------�õ�״̬��------------*/
float *srcdkfGetState(srcdkf_t *f) {
    return f->x.pData;
}

/*-------------���÷���-------------*/
void srcdkfSetVariance(srcdkf_t *f, float32_t *q, float32_t *v, float32_t *n, int nn) {
	float32_t *Sx = f->Sx.pData;
	float32_t *Sv = f->Sv.pData;
	float32_t *Sn = f->Sn.pData;
	int i;

	// state variance					//����״̬����
	if (q)
		for (i = 0; i < f->S; i++)
			Sx[i*f->S + i] = __sqrtf(fabsf(q[i]));		//����һ��Ԫ��Ϊ״̬����ĶԽǾ���

	// process noise					//�������� Q
	if (v)
		for (i = 0; i < f->V; i++)
			Sv[i*f->V + i] = __sqrtf(fabsf(v[i]));		//����һ��Ԫ��Ϊ������������ĶԽǾ���

	// observation noise				//�������� R
	if (n && nn) {
		// resize Sn
		f->Sn.numRows = nn;				//��������ֵ
		f->Sn.numCols = nn;				//��������ֵ

		for (i = 0; i < nn; i++)
			Sn[i*nn + i] = __sqrtf(fabsf(n[i]));		//����һ��Ԫ��Ϊ�۲���������ĶԽǾ���
	}
}
/*-------------�õ�����-------------*/
void srcdkfGetVariance(srcdkf_t *f, float32_t *q) {
	float32_t *Sx = f->Sx.pData;
	int i;

	// state variance
	if (q)
		for (i = 0; i < f->S; i++) {
			q[i] = Sx[i*f->S + i];
			q[i] = q[i]*q[i];
		}
}

// states, max observations, process noise, max observation noise								//״̬�����۲�ֵ���������������۲�����
srcdkf_t *srcdkfInit(int s, int m, int v, int n, SRCDKFTimeUpdate_t *timeUpdate) {
	srcdkf_t *f;
	int maxN = MAX(v, n);

	f = (srcdkf_t *)aqDataCalloc(1, sizeof(srcdkf_t));

	f->S = s;
	f->V = v;

	matrixInit(&f->Sx, s, s);														//����Ϊ17������Ϊ17
	matrixInit(&f->SxT, s, s);													//����Ϊ17������Ϊ17
	matrixInit(&f->Sv, v, v);														//����Ϊ12������Ϊ12
	matrixInit(&f->Sn, n, n);														//����Ϊ3������Ϊ3
	matrixInit(&f->x, s, 1);														//����Ϊ17������Ϊ1
	matrixInit(&f->Xa, s+maxN, 1+(s+maxN)*2);						//����Ϊ29������Ϊ59����Ϊ����Sigma����

	matrixInit(&f->qrTempS, s, (s+v)*2);								//����Ϊ17������Ϊ58
	matrixInit(&f->y, m, 1);														//����Ϊ3������Ϊ1
	matrixInit(&f->Y, m, 1+(s+n)*2);										//����Ϊ3������Ϊ41����Ϊ����Sigma����
	matrixInit(&f->qrTempM, m, (s+n)*2);								//����Ϊ3������Ϊ40
	matrixInit(&f->Sy, m, m);														//����Ϊ3������Ϊ3
	matrixInit(&f->SyT, m, m);													//����Ϊ3������Ϊ3
	matrixInit(&f->SyC, m, m);													//����Ϊ3������Ϊ3
	matrixInit(&f->Pxy, s, m);													//����Ϊ17������Ϊ3
	matrixInit(&f->C1, m, s);														//����Ϊ3������Ϊ17
	matrixInit(&f->C1T, s, m);													//����Ϊ17������Ϊ3
	matrixInit(&f->C2, m, n);														//����Ϊ3������Ϊ3
	matrixInit(&f->D, m, s+n);													//����Ϊ3������Ϊ20
	matrixInit(&f->K, s, m);														//����Ϊ17������Ϊ3
	matrixInit(&f->inov, m, 1);													//����Ϊ3������Ϊ1
	matrixInit(&f->xUpdate, s, 1);											//����Ϊ17������Ϊ1			
	matrixInit(&f->qrFinal, s, 2*s + 2*n);							//����Ϊ17������Ϊ40
	matrixInit(&f->Q, s, s+n);	// scratch							//����Ϊ17������Ϊ20
	matrixInit(&f->R, n, n);	// scratch								//����Ϊ3������Ϊ3
	matrixInit(&f->AQ, s, n);	// scratch								//����Ϊ17������Ϊ3

	f->xOut = (float32_t *)aqDataCalloc(s, sizeof(float32_t));
	f->xNoise = (float32_t *)aqDataCalloc(maxN, sizeof(float32_t));
	f->xIn = (float32_t *)aqDataCalloc(s, sizeof(float32_t));

	f->h = SRCDKF_H;           //3*sqrt(3)
	f->hh = f->h*f->h;		   //27
//	f->w0m = (f->hh - (float32_t)s) / f->hh;	// calculated in process
	f->wim = 1.0f / (2.0f * f->hh);							//0.018519 
	f->wic1 = __sqrtf(1.0f / (4.0f * f->hh));		//0.096225 
	f->wic2 = __sqrtf((f->hh - 1.0f) / (4.0f * f->hh*f->hh));		//0.094426 
        f->timeUpdate = timeUpdate;

	return f;
}

// given noise matrix
/*--------------- �������ܣ�����Sigma�� ------------------*/
/*-- ��ڲ�����SRCDKF�ṹ�壬�ṹ���еĹ���������۲����� --*/
static void srcdkfCalcSigmaPoints(srcdkf_t *f, arm_matrix_instance_f32 *Sn) {
	int S = f->S;			// number of states		״̬����������
	int N = Sn->numRows;	// number of noise variables ��������������
	int A = S+N;			// number of agumented states	����״̬������ = ״̬���� + ��������������
	int L = 1+A*2;			// number of sigma points	Sigma������� = 2*����״̬���� + 1
	float32_t *x = f->x.pData;	// state	״̬
	float32_t *Sx = f->Sx.pData;	// state covariance	״̬Э����
	float32_t *Xa = f->Xa.pData;	// augmented sigma points			Sigma���������
	int i, j;

	// set the number of sigma points		����Sigma��ĸ���
	f->L = L;													//��f->L��ֵΪSigma��ĸ���

	// resize output matrix  Sigma�������������Ϊ״̬��+������������Ϊ 2*��ǿ״̬���� + 1
	f->Xa.numRows = A;								//�����ΪA
	f->Xa.numCols = L;								//��ΪL

	//	-	   -
	// Sa =	| Sx	0  |
	//	| 0	Sn |
	//	-	   -
	// xa = [ x 	0  ]
	// Xa = [ xa  (xa + h*Sa)  (xa - h*Sa) ]
	//
	for (i = 0; i < A; i++) 																			//�������������ѭ��
	{
			int rOffset = i*L;																				//rOffset=��ǰ����*������
			float32_t base = (i < S) ? x[i] : 0.0f;										//����С��״̬����������ʱȡԭ״ֵ̬������ȡ0

			Xa[rOffset + 0] = base;																		//��һ��ȫ��ȡԭֵ

			for (j = 1; j <= A; j++) 																	//���������ѭ����ѭ������Sigma�㽻������һ��
			{
					float32_t t = 0.0f;
					
					if (i < S && j < S+1)																	//�������С��״̬���������� �� ����С��״̬������һʱ
						t = Sx[i*S + (j-1)]*f->h;														//��ʱֵtȡ ��ǰ��ǰһ�е�״̬Э���� �� 3sqrt(3)

					if (i >= S && j >= S+1)																//���������С��״̬���������� �� ������С��״̬������һʱ
						t = Sn->pData[(i-S)*N + (j-S-1)]*f->h;							//��ʱֵtȡ (i-S)N+(j-S-1) �۲�������ֵ
					//�����ߵ�Sigma��ȡֵ
					Xa[rOffset + j]     = base + t;												//�����״̬��ͬ�У����i�е�j��ȡ ״̬��+��Ȩֵ�����������ͬ�У���ֱ��ȡ��Ȩֵ
					Xa[rOffset + j + A] = base - t;												//�����״̬��ͬ�У����i�е�j+A��ȡ ״̬��-��Ȩֵ�����������ͬ�У���ֱ��ȡ����Ȩֵ
			}
	}
}
/*--------------- �������ܣ�SRCDKFʱ����� ------------------*/
/*--------- ���ڼ���Sigma��õ�����״̬��Ϊ�������� ----------*/
/*-------- ��ڲ�����SRCDKF�ṹ�壬״̬������ʱ������ --------*/
void srcdkfTimeUpdate(srcdkf_t *f, float32_t *u, float32_t dt) {
	int S = f->S;			// number of states״̬������
	int V = f->V;			// number of noise variables��������������
	int L;						// number of sigma points  Sigma��ĸ���
	float32_t *x = f->x.pData;			// state estimate״̬����
	float32_t *Xa = f->Xa.pData;		// augmented sigma points����Sigma����
//	float32_t *xIn = f->xIn;			// callback buffer
//	float32_t *xOut = f->xOut;		// callback buffer
//	float32_t *xNoise = f->xNoise;// callback buffer
	float32_t *qrTempS = f->qrTempS.pData;
	int i, j;

	srcdkfCalcSigmaPoints(f, &f->Sv);																							//�����ڹ���������Sigma��
	L = f->L;

	// Xa = f(Xx, Xv, u, dt)
//	for (i = 0; i < L; i++) {
//		for (j = 0; j < S; j++)
//			xIn[j] = Xa[j*L + i];
//
//		for (j = 0; j < V; j++)
//			xNoise[j] = Xa[(S+j)*L + i];
//
//		f->timeUpdate(xIn, xNoise, xOut, u, dt);
//
//		for (j = 0; j < S; j++)
//			Xa[j*L + i] = xOut[j];
//	}
//ԭ����ΪnavUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n)
	f->timeUpdate(&Xa[0], &Xa[S*L], &Xa[0], u, dt, L);														//������Xa���룬����󴫵ݵ�Xa													

	// sum weighted resultant sigma points to create estimated state							//�ܺͼ�Ȩ���ɵ�Sigma������������״̬
	f->w0m = (f->hh - (float32_t)(S+V)) / f->hh;																	//w0m = (h^2 - S - V) / h^2;
	
	for (i = 0; i < S; i++) 																											//Sigma���������ѭ��
	{
		int rOffset = i*L;																													//Ĭ��ֵΪ���ԽǾ����Ԫ��

		x[i] = Xa[rOffset + 0] * f->w0m;																						//��ÿ��״ֵ̬���ϳ�ʼֵ

		for (j = 1; j < L; j++)																											//Sigam���������ѭ��
			x[i] += Xa[rOffset + j] * f->wim;																					//���Ѽ��ϳ�ʼֵXa���м���Sigma���Ȩֵ
	}

	// update state covariance																										//����״̬Э����
	for (i = 0; i < S; i++) 																											//Sigma������ѭ��
	{
		int rOffset = i*(S+V)*2;																										//Ĭ��ֵΪ12�ı���

		for (j = 0; j < S+V; j++) 																									//ѭ������Ϊ����Sigma�����
		{
			qrTempS[rOffset + j] = (Xa[i*L + j + 1] - Xa[i*L + S+V + j + 1]) * f->wic1;																		//�����߾��󣩶ԳƵĸ�Sigma���ȥ��Sigma�㣬�õ�ƽ����Э������ƣ��ٳ���wic1
			qrTempS[rOffset + S+V + j] = (Xa[i*L + j + 1] + Xa[i*L + S+V + j + 1] - 2.0f*Xa[i*L + 0]) * f->wic2;					//���Ұ�߾��󣩶ԳƵĸ�Sigma�������Sigma���ټ�ȥ������ԭ״̬�����õ�ƽ����Э������ƣ�������wic2
		}
	}

	qrDecompositionT_f32(&f->qrTempS, NULL, &f->SxT);   // with transposition			//��λ
	arm_mat_trans_f32(&f->SxT, &f->Sx);																						//ת�ó�״̬Э����
}
/*--------------- �������ܣ�SRCDKF�������� ------------------*/
/*-------- ��ڲ�����SRCDKF�ṹ�壬״̬�������۲�����������������������������������º��� --------*/
void srcdkfMeasurementUpdate(srcdkf_t *f, float32_t *u, float32_t *ym, int M, int N, float32_t *noise, SRCDKFMeasurementUpdate_t *measurementUpdate) {
	int S = f->S;											// number of states				״̬������
	float32_t *Xa = f->Xa.pData;			// sigma points						Sigma�������
	float32_t *xIn = f->xIn;					// callback buffer			
	float32_t *xNoise = f->xNoise;		// callback buffer
	float32_t *xOut = f->xOut;				// callback buffer
	float32_t *Y = f->Y.pData;				// measurements from sigma points				����Sigma��Ĳ���
	float32_t *y = f->y.pData;				// measurement estimate									��������
	float32_t *Sn = f->Sn.pData;			// observation noise covariance					�۲�����Э����
	float32_t *qrTempM = f->qrTempM.pData;
	float32_t *C1 = f->C1.pData;
	float32_t *C1T = f->C1T.pData;
	float32_t *C2 = f->C2.pData;
	float32_t *D = f->D.pData;
	float32_t *inov = f->inov.pData;		// M x 1 matrix
	float32_t *xUpdate = f->xUpdate.pData;	// S x 1 matrix
	float32_t *x = f->x.pData;				// state estimate					״̬����
	float32_t *Sx = f->Sx.pData;
	float32_t *Q = f->Q.pData;
	float32_t *qrFinal = f->qrFinal.pData;
	int L;					// number of sigma points
	int i, j;

	// make measurement noise matrix if provided  ����ṩ�Ļ��������������
	if (noise) 
	{
		f->Sn.numRows = N;
		f->Sn.numCols = N;
		arm_fill_f32(0.0f, f->Sn.pData, N*N);																			//������ֵ���Ϊ��������
		for (i = 0; i < N; i++)
			arm_sqrt_f32(fabsf(noise[i]), &Sn[i*N + i]);														//�������ԽǾ�����п�����ֵ�������ԽǾ�����
	}

	// generate sigma points	����Sigma��
	srcdkfCalcSigmaPoints(f, &f->Sn);																						//���ɹ��ڹ۲�������Sigma��
	L = f->L;

	// resize all N and M based storage as they can change each iteration 			//��������N��MΪ�����Ĵ洢����Ϊ���ǿ��Ըı�ÿ�ε���
	// ��ֵ��MΪ����������Nδ��������
	f->y.numRows = M;
	f->Y.numRows = M;
	f->Y.numCols = L;
	f->qrTempM.numRows = M;
	f->qrTempM.numCols = (S+N)*2;
	f->Sy.numRows = M;
	f->Sy.numCols = M;
	f->SyT.numRows = M;
	f->SyT.numCols = M;
	f->SyC.numRows = M;
	f->SyC.numCols = M;
	f->Pxy.numCols = M;
	f->C1.numRows = M;
	f->C1T.numCols = M;
	f->C2.numRows = M;
	f->C2.numCols = N;
	f->D.numRows = M;
	f->D.numCols = S+N;
	f->K.numCols = M;
	f->inov.numRows = M;
	f->qrFinal.numCols = 2*S + 2*N;

	// Y = h(Xa, Xn)
	for (i = 0; i < L; i++) 																										//������ѭ��
	{
		for (j = 0; j < S; j++)																										//������ѭ��
			xIn[j] = Xa[j*L + i];																										//��״̬���������е����Խ�״̬Ԫ�ش��ݵ�xIn������

		for (j = 0; j < N; j++)
			xNoise[j] = Xa[(S+j)*L + i];																						//��״̬���������е����Խ�����Ԫ�ش��ݵ�xNoise������

		measurementUpdate(u, xIn, xNoise, xOut);																	//���¸��Ե�״̬��������

		for (j = 0; j < M; j++)
			Y[j*L + i] = xOut[j];																										//���۲����ֵ
	}

	// sum weighted resultant sigma points to create estimated measurement			//������Ȩ����Sigma����������ƵĲ���
	f->w0m = (f->hh - (float32_t)(S+N)) / f->hh;
	for (i = 0; i < M; i++) {
		int rOffset = i*L;

		y[i] = Y[rOffset + 0] * f->w0m;
		for (j = 1; j < L; j++)
			y[i] += Y[rOffset + j] * f->wim;
	}

	// calculate measurement covariance components															//�������Э�������
	for (i = 0; i < M; i++) {
		int rOffset = i*(S+N)*2;

		for (j = 0; j < S+N; j++) {
			float32_t c, d;

			c = (Y[i*L + j + 1] - Y[i*L + S+N + j + 1]) * f->wic1;
			d = (Y[i*L + j + 1] + Y[i*L + S+N + j + 1] - 2.0f*Y[i*L]) * f->wic2;

			qrTempM[rOffset + j] = c;
			qrTempM[rOffset + S+N + j] = d;

			// save fragments for future operations																	//Ϊδ����������Ƭ��
			if (j < S) {
				C1[i*S + j] = c;																											//����۲�ϵ��
				C1T[j*M + i] = c;
			}
			else {
				C2[i*N + (j-S)] = c;
			}
			D[i*(S+N) + j] = d;
		}
	}

	qrDecompositionT_f32(&f->qrTempM, NULL, &f->SyT);	// with transposition			//��λ

	arm_mat_trans_f32(&f->SyT, &f->Sy);
	arm_mat_trans_f32(&f->SyT, &f->SyC);		// make copy as later Div is destructive			���ƺ�Div���ƻ��Ե�

	// create Pxy																																//����״̬�����͹۲�������Э����
	arm_mat_mult_f32(&f->Sx, &f->C1T, &f->Pxy);

	// K = (Pxy / SyT) / Sy																											//���㿨��������
	matrixDiv_f32(&f->K, &f->Pxy, &f->SyT, &f->Q, &f->R, &f->AQ);
	matrixDiv_f32(&f->K, &f->K, &f->Sy, &f->Q, &f->R, &f->AQ);

	// x = x + k(ym - y)																												//�õ���ѹ���
	for (i = 0; i < M; i++)																											//��3��
		inov[i] = ym[i] - y[i];																										//inov = y()
	arm_mat_mult_f32(&f->K, &f->inov, &f->xUpdate);															//xUpdate = K * inov

	for (i = 0; i < S; i++)
			x[i] += xUpdate[i];																											//�ó�����ֵ
	

	// build final QR matrix																										//��������QR����
	//	rows = s																																//����
	//	cols = s + n + s + n																										//����
	//	use Q as temporary result storage																				//ʹ��Q��Ϊ��ʱ����洢

	f->Q.numRows = S;
	f->Q.numCols = S;
	arm_mat_mult_f32(&f->K, &f->C1, &f->Q);
	for (i = 0; i < S; i++) 
	{
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < S; j++)
			qrFinal[rOffset + j] = Sx[i*S + j] - Q[i*S + j];												//����һ�����Խǲ��־�������Ԫ��Ϊ Sx - K*C1
	}

	f->Q.numRows = S;
	f->Q.numCols = N;
	arm_mat_mult_f32(&f->K, &f->C2, &f->Q);
	for (i = 0; i < S; i++) 
	{
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < N; j++)
			qrFinal[rOffset + S+j] = Q[i*N + j];																		//����һ�����Խǲ��־�������Ԫ��Ϊ  K*C2
	}

	f->Q.numRows = S;
	f->Q.numCols = S+N;
	arm_mat_mult_f32(&f->K, &f->D, &f->Q);
	for (i = 0; i < S; i++) 
	{
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < S+N; j++)
			qrFinal[rOffset + S+N+j] = Q[i*(S+N) + j];															//����һ�����Խǲ��־�������Ԫ��Ϊ  K*D
	}

	// Sx = qr([Sx-K*C1 K*C2 K*D]')
	// this method is not susceptable to numeric instability like the Cholesky is	���ַ�����������ֵ���ȶ���Cholesky
	qrDecompositionT_f32(&f->qrFinal, NULL, &f->SxT);	// with transposition			//��λ
	arm_mat_trans_f32(&f->SxT, &f->Sx);																					//ת��Ϊ״̬Э�������
}



