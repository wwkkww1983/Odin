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

/*-------------得到状态量------------*/
float *srcdkfGetState(srcdkf_t *f) {
    return f->x.pData;
}

/*-------------设置方差-------------*/
void srcdkfSetVariance(srcdkf_t *f, float32_t *q, float32_t *v, float32_t *n, int nn) {
	float32_t *Sx = f->Sx.pData;
	float32_t *Sv = f->Sv.pData;
	float32_t *Sn = f->Sn.pData;
	int i;

	// state variance					//计算状态方差
	if (q)
		for (i = 0; i < f->S; i++)
			Sx[i*f->S + i] = __sqrtf(fabsf(q[i]));		//生成一个元素为状态方差的对角矩阵

	// process noise					//过程噪声 Q
	if (v)
		for (i = 0; i < f->V; i++)
			Sv[i*f->V + i] = __sqrtf(fabsf(v[i]));		//生成一个元素为过程噪声方差的对角矩阵

	// observation noise				//测量噪声 R
	if (n && nn) {
		// resize Sn
		f->Sn.numRows = nn;				//给行数赋值
		f->Sn.numCols = nn;				//给列数赋值

		for (i = 0; i < nn; i++)
			Sn[i*nn + i] = __sqrtf(fabsf(n[i]));		//生成一个元素为观测噪声方差的对角矩阵
	}
}
/*-------------得到方差-------------*/
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

// states, max observations, process noise, max observation noise								//状态，最大观测值，过程噪声，最大观测噪声
srcdkf_t *srcdkfInit(int s, int m, int v, int n, SRCDKFTimeUpdate_t *timeUpdate) {
	srcdkf_t *f;
	int maxN = MAX(v, n);

	f = (srcdkf_t *)aqDataCalloc(1, sizeof(srcdkf_t));

	f->S = s;
	f->V = v;

	matrixInit(&f->Sx, s, s);														//行数为17，列数为17
	matrixInit(&f->SxT, s, s);													//行数为17，列数为17
	matrixInit(&f->Sv, v, v);														//行数为12，列数为12
	matrixInit(&f->Sn, n, n);														//行数为3，列数为3
	matrixInit(&f->x, s, 1);														//行数为17，列数为1
	matrixInit(&f->Xa, s+maxN, 1+(s+maxN)*2);						//行数为29，列数为59，其为增广Sigma矩阵

	matrixInit(&f->qrTempS, s, (s+v)*2);								//行数为17，列数为58
	matrixInit(&f->y, m, 1);														//行数为3，列数为1
	matrixInit(&f->Y, m, 1+(s+n)*2);										//行数为3，列数为41，其为增广Sigma矩阵
	matrixInit(&f->qrTempM, m, (s+n)*2);								//行数为3，列数为40
	matrixInit(&f->Sy, m, m);														//行数为3，列数为3
	matrixInit(&f->SyT, m, m);													//行数为3，列数为3
	matrixInit(&f->SyC, m, m);													//行数为3，列数为3
	matrixInit(&f->Pxy, s, m);													//行数为17，列数为3
	matrixInit(&f->C1, m, s);														//行数为3，列数为17
	matrixInit(&f->C1T, s, m);													//行数为17，列数为3
	matrixInit(&f->C2, m, n);														//行数为3，列数为3
	matrixInit(&f->D, m, s+n);													//行数为3，列数为20
	matrixInit(&f->K, s, m);														//行数为17，列数为3
	matrixInit(&f->inov, m, 1);													//行数为3，列数为1
	matrixInit(&f->xUpdate, s, 1);											//行数为17，列数为1			
	matrixInit(&f->qrFinal, s, 2*s + 2*n);							//行数为17，列数为40
	matrixInit(&f->Q, s, s+n);	// scratch							//行数为17，列数为20
	matrixInit(&f->R, n, n);	// scratch								//行数为3，列数为3
	matrixInit(&f->AQ, s, n);	// scratch								//行数为17，列数为3

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
/*--------------- 函数功能：计算Sigma点 ------------------*/
/*-- 入口参数：SRCDKF结构体，结构体中的过程噪声或观测噪声 --*/
static void srcdkfCalcSigmaPoints(srcdkf_t *f, arm_matrix_instance_f32 *Sn) {
	int S = f->S;			// number of states		状态变量的数量
	int N = Sn->numRows;	// number of noise variables 噪声变量的数量
	int A = S+N;			// number of agumented states	增广状态的数量 = 状态数量 + 噪声变量的数量
	int L = 1+A*2;			// number of sigma points	Sigma点的数量 = 2*增广状态数量 + 1
	float32_t *x = f->x.pData;	// state	状态
	float32_t *Sx = f->Sx.pData;	// state covariance	状态协方差
	float32_t *Xa = f->Xa.pData;	// augmented sigma points			Sigma点增广矩阵
	int i, j;

	// set the number of sigma points		设置Sigma点的个数
	f->L = L;													//给f->L赋值为Sigma点的个数

	// resize output matrix  Sigma点输出矩阵，行数为状态数+噪声数，列数为 2*增强状态数量 + 1
	f->Xa.numRows = A;								//输出行为A
	f->Xa.numCols = L;								//列为L

	//	-	   -
	// Sa =	| Sx	0  |
	//	| 0	Sn |
	//	-	   -
	// xa = [ x 	0  ]
	// Xa = [ xa  (xa + h*Sa)  (xa - h*Sa) ]
	//
	for (i = 0; i < A; i++) 																			//增广矩阵行数大循环
	{
			int rOffset = i*L;																				//rOffset=当前行数*总列数
			float32_t base = (i < S) ? x[i] : 0.0f;										//行数小于状态变量的数量时取原状态值，否则取0

			Xa[rOffset + 0] = base;																		//第一列全部取原值

			for (j = 1; j <= A; j++) 																	//增广矩阵列循环，循环正负Sigma点交界的最后一列
			{
					float32_t t = 0.0f;
					
					if (i < S && j < S+1)																	//如果行数小于状态变量的数量 且 列数小于状态数量加一时
						t = Sx[i*S + (j-1)]*f->h;														//临时值t取 当前行前一列的状态协方差 乘 3sqrt(3)

					if (i >= S && j >= S+1)																//如果行数不小于状态变量的数量 且 列数不小于状态数量加一时
						t = Sn->pData[(i-S)*N + (j-S-1)]*f->h;							//临时值t取 (i-S)N+(j-S-1) 观测噪声的值
					//给两边的Sigma点取值
					Xa[rOffset + j]     = base + t;												//如果与状态量同行，则第i行第j列取 状态量+加权值，如果与噪声同行，则直接取加权值
					Xa[rOffset + j + A] = base - t;												//如果与状态量同行，则第i行第j+A列取 状态量-加权值，如果与噪声同行，则直接取负加权值
			}
	}
}
/*--------------- 函数功能：SRCDKF时间更新 ------------------*/
/*--------- 用于计算Sigma点得到估计状态，为先验运算 ----------*/
/*-------- 入口参数：SRCDKF结构体，状态变量，时间周期 --------*/
void srcdkfTimeUpdate(srcdkf_t *f, float32_t *u, float32_t dt) {
	int S = f->S;			// number of states状态的数量
	int V = f->V;			// number of noise variables噪声变量的数量
	int L;						// number of sigma points  Sigma点的个数
	float32_t *x = f->x.pData;			// state estimate状态估计
	float32_t *Xa = f->Xa.pData;		// augmented sigma points增广Sigma矩阵
//	float32_t *xIn = f->xIn;			// callback buffer
//	float32_t *xOut = f->xOut;		// callback buffer
//	float32_t *xNoise = f->xNoise;// callback buffer
	float32_t *qrTempS = f->qrTempS.pData;
	int i, j;

	srcdkfCalcSigmaPoints(f, &f->Sv);																							//做关于过程噪声的Sigma点
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
//原函数为navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n)
	f->timeUpdate(&Xa[0], &Xa[S*L], &Xa[0], u, dt, L);														//这里由Xa输入，计算后传递到Xa													

	// sum weighted resultant sigma points to create estimated state							//总和加权生成的Sigma点来创建估计状态
	f->w0m = (f->hh - (float32_t)(S+V)) / f->hh;																	//w0m = (h^2 - S - V) / h^2;
	
	for (i = 0; i < S; i++) 																											//Sigma增广矩阵行循环
	{
		int rOffset = i*L;																													//默认值为正对角矩阵的元素

		x[i] = Xa[rOffset + 0] * f->w0m;																						//给每个状态值加上初始值

		for (j = 1; j < L; j++)																											//Sigam增广矩阵列循环
			x[i] += Xa[rOffset + j] * f->wim;																					//给已加上初始值Xa数列加上Sigma点的权值
	}

	// update state covariance																										//更新状态协方差
	for (i = 0; i < S; i++) 																											//Sigma矩阵行循环
	{
		int rOffset = i*(S+V)*2;																										//默认值为12的倍数

		for (j = 0; j < S+V; j++) 																									//循环列数为单边Sigma点个数
		{
			qrTempS[rOffset + j] = (Xa[i*L + j + 1] - Xa[i*L + S+V + j + 1]) * f->wic1;																		//（左半边矩阵）对称的负Sigma点减去正Sigma点，得到平方根协方差估计，再乘上wic1
			qrTempS[rOffset + S+V + j] = (Xa[i*L + j + 1] + Xa[i*L + S+V + j + 1] - 2.0f*Xa[i*L + 0]) * f->wic2;					//（右半边矩阵）对称的负Sigma点加上正Sigma点再减去两倍的原状态量，得到平方根协方差估计，最后乘上wic2
		}
	}

	qrDecompositionT_f32(&f->qrTempS, NULL, &f->SxT);   // with transposition			//换位
	arm_mat_trans_f32(&f->SxT, &f->Sx);																						//转置成状态协方差
}
/*--------------- 函数功能：SRCDKF测量更新 ------------------*/
/*-------- 入口参数：SRCDKF结构体，状态变量，观测变量，行数，列数，噪声变量，测量更新函数 --------*/
void srcdkfMeasurementUpdate(srcdkf_t *f, float32_t *u, float32_t *ym, int M, int N, float32_t *noise, SRCDKFMeasurementUpdate_t *measurementUpdate) {
	int S = f->S;											// number of states				状态的数量
	float32_t *Xa = f->Xa.pData;			// sigma points						Sigma点的数量
	float32_t *xIn = f->xIn;					// callback buffer			
	float32_t *xNoise = f->xNoise;		// callback buffer
	float32_t *xOut = f->xOut;				// callback buffer
	float32_t *Y = f->Y.pData;				// measurements from sigma points				来自Sigma点的测量
	float32_t *y = f->y.pData;				// measurement estimate									测量估计
	float32_t *Sn = f->Sn.pData;			// observation noise covariance					观测噪声协方差
	float32_t *qrTempM = f->qrTempM.pData;
	float32_t *C1 = f->C1.pData;
	float32_t *C1T = f->C1T.pData;
	float32_t *C2 = f->C2.pData;
	float32_t *D = f->D.pData;
	float32_t *inov = f->inov.pData;		// M x 1 matrix
	float32_t *xUpdate = f->xUpdate.pData;	// S x 1 matrix
	float32_t *x = f->x.pData;				// state estimate					状态估计
	float32_t *Sx = f->Sx.pData;
	float32_t *Q = f->Q.pData;
	float32_t *qrFinal = f->qrFinal.pData;
	int L;					// number of sigma points
	int i, j;

	// make measurement noise matrix if provided  如果提供的话则测量噪声矩阵
	if (noise) 
	{
		f->Sn.numRows = N;
		f->Sn.numCols = N;
		arm_fill_f32(0.0f, f->Sn.pData, N*N);																			//将常量值填充为浮点向量
		for (i = 0; i < N; i++)
			arm_sqrt_f32(fabsf(noise[i]), &Sn[i*N + i]);														//对噪声对角矩阵进行开方，值放入主对角矩阵中
	}

	// generate sigma points	生成Sigma点
	srcdkfCalcSigmaPoints(f, &f->Sn);																						//生成关于观测噪声的Sigma点
	L = f->L;

	// resize all N and M based storage as they can change each iteration 			//调整所有N和M为基础的存储，因为它们可以改变每次迭代
	// 赋值：M为矩阵行数，N未矩阵列数
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
	for (i = 0; i < L; i++) 																										//列数大循环
	{
		for (j = 0; j < S; j++)																										//行数大循环
			xIn[j] = Xa[j*L + i];																										//将状态变量矩阵中的正对角状态元素传递到xIn数组中

		for (j = 0; j < N; j++)
			xNoise[j] = Xa[(S+j)*L + i];																						//将状态变量矩阵中的正对角噪声元素传递到xNoise数组中

		measurementUpdate(u, xIn, xNoise, xOut);																	//更新各自的状态变量函数

		for (j = 0; j < M; j++)
			Y[j*L + i] = xOut[j];																										//给观测矩阵赋值
	}

	// sum weighted resultant sigma points to create estimated measurement			//总数加权生成Sigma点来创造估计的测量
	f->w0m = (f->hh - (float32_t)(S+N)) / f->hh;
	for (i = 0; i < M; i++) {
		int rOffset = i*L;

		y[i] = Y[rOffset + 0] * f->w0m;
		for (j = 1; j < L; j++)
			y[i] += Y[rOffset + j] * f->wim;
	}

	// calculate measurement covariance components															//计算测量协方差分量
	for (i = 0; i < M; i++) {
		int rOffset = i*(S+N)*2;

		for (j = 0; j < S+N; j++) {
			float32_t c, d;

			c = (Y[i*L + j + 1] - Y[i*L + S+N + j + 1]) * f->wic1;
			d = (Y[i*L + j + 1] + Y[i*L + S+N + j + 1] - 2.0f*Y[i*L]) * f->wic2;

			qrTempM[rOffset + j] = c;
			qrTempM[rOffset + S+N + j] = d;

			// save fragments for future operations																	//为未来操作保存片段
			if (j < S) {
				C1[i*S + j] = c;																											//计算观测系数
				C1T[j*M + i] = c;
			}
			else {
				C2[i*N + (j-S)] = c;
			}
			D[i*(S+N) + j] = d;
		}
	}

	qrDecompositionT_f32(&f->qrTempM, NULL, &f->SyT);	// with transposition			//换位

	arm_mat_trans_f32(&f->SyT, &f->Sy);
	arm_mat_trans_f32(&f->SyT, &f->SyC);		// make copy as later Div is destructive			复制后Div是破坏性的

	// create Pxy																																//创建状态变量和观测变量间的协方差
	arm_mat_mult_f32(&f->Sx, &f->C1T, &f->Pxy);

	// K = (Pxy / SyT) / Sy																											//计算卡尔曼增益
	matrixDiv_f32(&f->K, &f->Pxy, &f->SyT, &f->Q, &f->R, &f->AQ);
	matrixDiv_f32(&f->K, &f->K, &f->Sy, &f->Q, &f->R, &f->AQ);

	// x = x + k(ym - y)																												//得到最佳估计
	for (i = 0; i < M; i++)																											//分3次
		inov[i] = ym[i] - y[i];																										//inov = y()
	arm_mat_mult_f32(&f->K, &f->inov, &f->xUpdate);															//xUpdate = K * inov

	for (i = 0; i < S; i++)
			x[i] += xUpdate[i];																											//得出最优值
	

	// build final QR matrix																										//建立最终QR矩阵
	//	rows = s																																//行数
	//	cols = s + n + s + n																										//列数
	//	use Q as temporary result storage																				//使用Q作为临时结果存储

	f->Q.numRows = S;
	f->Q.numCols = S;
	arm_mat_mult_f32(&f->K, &f->C1, &f->Q);
	for (i = 0; i < S; i++) 
	{
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < S; j++)
			qrFinal[rOffset + j] = Sx[i*S + j] - Q[i*S + j];												//生成一个主对角部分矩阵，其中元素为 Sx - K*C1
	}

	f->Q.numRows = S;
	f->Q.numCols = N;
	arm_mat_mult_f32(&f->K, &f->C2, &f->Q);
	for (i = 0; i < S; i++) 
	{
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < N; j++)
			qrFinal[rOffset + S+j] = Q[i*N + j];																		//生成一个主对角部分矩阵，其中元素为  K*C2
	}

	f->Q.numRows = S;
	f->Q.numCols = S+N;
	arm_mat_mult_f32(&f->K, &f->D, &f->Q);
	for (i = 0; i < S; i++) 
	{
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < S+N; j++)
			qrFinal[rOffset + S+N+j] = Q[i*(S+N) + j];															//生成一个主对角部分矩阵，其中元素为  K*D
	}

	// Sx = qr([Sx-K*C1 K*C2 K*D]')
	// this method is not susceptable to numeric instability like the Cholesky is	这种方法不容易数值不稳定如Cholesky
	qrDecompositionT_f32(&f->qrFinal, NULL, &f->SxT);	// with transposition			//换位
	arm_mat_trans_f32(&f->SxT, &f->Sx);																					//转化为状态协方差矩阵
}



