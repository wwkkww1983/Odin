#include "cali.h"
#include "imu.h"
#include "config.h"
#include "supervisor.h"

calibStruct_t calibData;

// solves U'*x = b for 3x3 matrix
void solveUT(float32_t *U, float32_t *b, float32_t *x) {
	float32_t t1 = b[1] - (U[1]*b[0]) / U[0];

	x[0] = b[0] / U[0];
	x[1] = t1 /  U[4];
	x[2] = -(U[5]*t1/U[4] - b[2] + U[2]*b[0]/U[0]) / U[8];
}

// solves U*-x = b for 3x3 matrix
void solveU(float32_t *U, float32_t *b, float32_t *x) {
	float32_t t1 = b[1] - (U[5]*b[2])/U[8];

	x[0] = +(U[1]*t1/U[4] - b[0] + U[2]*b[2]/U[8]) / U[0];
	x[1] = -(t1 / U[4]);
	x[2] = -(b[2] / U[8]);
}

//根据当前参数来反馈所处三维象限
int calibOctant(float32_t *vec) {
	float32_t x, y, z;

	x = vec[0] + calibData.bias[0];
	y = vec[1] + calibData.bias[1];
	z = vec[2] + calibData.bias[2];

	if (x > 0) {
		if (y > 0) {
			if (z > 0)
				return 0;								//000 return 0
			else
				return 1;								//001 return 1
		}
		else {
			if (z > 0)
				return 2;								//010 return 2
			else
				return 3;					   		//011 return 3
		}
	}
	else {
		if (y > 0) {
			if (z > 0)
				return 4;								//100 return 4
			else
				return 5;								//101 return 5
		}
		else {
			if (z > 0)
				return 6;								//110 return 6
			else
				return 7;								//111 return 7
		}
	}
}

// determine angle between two vectors																//确定两个向量之间的夹角
float32_t calibAngle(float *v1, float *v2) {
	float32_t v1b[3];
	float32_t v2b[3];
	float32_t t;
	int i;

	t = 0.0f;
	for (i = 0; i < 3; i++) {
			v1b[i] = v1[i] + calibData.bias[i];
			v2b[i] = v2[i] + calibData.bias[i];

			t += (v1b[i]*v2b[i]);
	}

	t /= __sqrtf(v1b[0]*v1b[0] + v1b[1]*v1b[1] + v1b[2]*v1b[2]);
	t /= __sqrtf(v2b[0]*v2b[0] + v2b[1]*v2b[1] + v2b[2]*v2b[2]);

	return (fabsf(t-1.0f) < 1e-6f) ? 0.0f : acosf(t);
}

void calibInsert(float32_t *v) {
	int idx;
	int i;

	idx = calibOctant(v) * CALIB_SAMPLES;								//象限序号*样本数
	for (i = 0; i < CALIB_SAMPLES; i++)									//循环判断20次
		if (calibData.calibSamples[(idx + i)*3 + 0] == CALIB_EMPTY_SLOT){		//当前样本第一个元素等于100.0f
			calibData.calibSamples[(idx + i)*3 + 0] = v[0];				//分别赋值
			calibData.calibSamples[(idx + i)*3 + 1] = v[1];
			calibData.calibSamples[(idx + i)*3 + 2] = v[2];
			return;														//退出循环
		}
}
//校准重新计算？
void calibRecalc() {
	int i, j, k;
	for (i = 0; i < 8; i++){													//行循环8次，8个三维象限
		for (j = 0; j < CALIB_SAMPLES; j++){					//列循环20次 
			k = i*CALIB_SAMPLES + j;
			if (calibData.calibSamples[k*3] != CALIB_EMPTY_SLOT){
				if (calibOctant(&calibData.calibSamples[k*3]) != i){//要求当前样本所处的象限不处于当前象限
					calibInsert(&calibData.calibSamples[k*3]);			//储存当前值
					calibData.calibSamples[k*3] = CALIB_EMPTY_SLOT;		
				}
			}
		}
 }
}

int calibCount(void) {
	int sum;
	int i, j;
	sum = 0;
	for (i = 0; i < 8; i++)
		for (j = 0; j < CALIB_SAMPLES; j++)
			sum += (calibData.calibSamples[(i*CALIB_SAMPLES + j)*3] != CALIB_EMPTY_SLOT);
	calibData.percentComplete = (float32_t)sum / (8*CALIB_SAMPLES) * 100.0f;
	return sum;
}
//校准初始化
void calibInit(void) {
	int i, j;
	calibData.calibSamples = (float32_t *)aqCalloc(CALIB_SAMPLES*8*3, sizeof(float32_t));		//给样本分配20*8*3个单位的地址
	for (i = 0; i < 3; i++) {
		calibData.min[i] = +999.9;
		calibData.max[i] = -999.9;
	}
	for (i = 0; i < 8; i++)
		for (j = 0; j < CALIB_SAMPLES; j++)
			calibData.calibSamples[(i*CALIB_SAMPLES + j)*3] = CALIB_EMPTY_SLOT;					//给样本的初始值赋100.0f
	calibData.lastVec[0] = CALIB_EMPTY_SLOT;
	calibData.percentComplete = 0.0f;
}
//释放内存空间
void calibFree(void) {
  if (calibData.calibSamples) {
		aqFree(calibData.calibSamples, CALIB_SAMPLES*8*3, sizeof(float32_t));
		calibData.calibSamples = 0;
  }
}
//删除当前函数
void calibDeinit(void) {
  calibFree();
}

//校准计算
void calibCalculate(void) {
	arm_matrix_instance_f32 D, R;
	float32_t v[3];
	float32_t d, s;
	float32_t *t;
	float sign;
	int i;

	t = (float32_t *)aqCalloc(10*8*CALIB_SAMPLES, sizeof(float32_t));
	if (t == 0) {
		goto calibFree;														//内存不足则释放内存空间
	}

	arm_mat_init_f32(&D, 10, 8*CALIB_SAMPLES, t);					//创建一个10行120列的矩阵

	t = calibData.calibSamples;														//给t赋样本的地址
	for (i = 0; i < 8*CALIB_SAMPLES; i++) 								//给160列依次赋值，将校准样本中的内容转移至矩阵中
	{	
		D.pData[0*8*CALIB_SAMPLES + i] = t[0]*t[0];					//x^2
		D.pData[1*8*CALIB_SAMPLES + i] = t[1]*t[1];					//y^2
		D.pData[2*8*CALIB_SAMPLES + i] = t[2]*t[2];					//z^2
		D.pData[3*8*CALIB_SAMPLES + i] = t[0]*t[1];					//x*y
		D.pData[4*8*CALIB_SAMPLES + i] = t[0]*t[2];					//x*z
		D.pData[5*8*CALIB_SAMPLES + i] = t[1]*t[2];					//y*z
		D.pData[6*8*CALIB_SAMPLES + i] = t[0];						//x
		D.pData[7*8*CALIB_SAMPLES + i] = t[1];						//y
		D.pData[8*8*CALIB_SAMPLES + i] = t[2];						//z
		D.pData[9*8*CALIB_SAMPLES + i] = 1.0f;						//1
		t += 3;
	}

	// free raw data
	calibFree();																//转移完毕释放内存空间

	t = (float32_t *)aqCalloc(20*10, sizeof(float32_t));
	if (t == 0) {
		goto calibFree;
	}

	arm_mat_init_f32(&R, 20, 10, t);
	arm_fill_f32(0, t, 20*10);
	
	if (qrDecompositionT_f32(&D, 0, &R) == 0) {
		goto calibFree;
	}

	svd(R.pData, calibData.U, 10);												//进行奇异值分解

	sign = (R.pData[10*10 + 9] < 0.0f) ? -1.0f : +1.0f;

	calibData.U[0] = sign * R.pData[(10+0)*10 + 9];
	calibData.U[1] = sign * R.pData[(10+3)*10 + 9] * 0.5f;
	calibData.U[2] = sign * R.pData[(10+4)*10 + 9] * 0.5f;
	calibData.U[3] = sign * R.pData[(10+3)*10 + 9] * 0.5f;
	calibData.U[4] = sign * R.pData[(10+1)*10 + 9];
	calibData.U[5] = sign * R.pData[(10+5)*10 + 9] * 0.5f;
	calibData.U[6] = sign * R.pData[(10+4)*10 + 9] * 0.5f;
	calibData.U[7] = sign * R.pData[(10+5)*10 + 9] * 0.5f;
	calibData.U[8] = sign * R.pData[(10+2)*10 + 9];

	calibData.bias[0] = sign * R.pData[(10+6)*10 + 9] * 0.5f;
	calibData.bias[1] = sign * R.pData[(10+7)*10 + 9] * 0.5f;
	calibData.bias[2] = sign * R.pData[(10+8)*10 + 9] * 0.5f;

	d = sign * R.pData[(10+9)*10 + 9];

	if (cholF(calibData.U) == 0) {							//判断矩阵是否正定
		goto calibFree;
	}

	aqFree(t,20*10, sizeof(float32_t));
	
	solveUT(calibData.U, calibData.bias, v);

	s = 1.0f / __sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] - d) * CALIB_SCALE;

	solveU(calibData.U, v, calibData.bias);

	for (i = 0; i < 9; i++)
	calibData.U[i] *= s;

	calibData.bias[0] = -calibData.bias[0];
	calibData.bias[1] = -calibData.bias[1];
	calibData.bias[2] = -calibData.bias[2];

	calibData.U[0] = 1.0f / calibData.U[0];
	calibData.U[4] = 1.0f / calibData.U[4];
	calibData.U[8] = 1.0f / calibData.U[8];

	// store config params
//	imuSensorData.magBIAS[0] = calibData.bias[0];
//	imuSensorData.magBIAS[1] = calibData.bias[1];
//	imuSensorData.magBIAS[2] = calibData.bias[2];
//    p[IMU_MAG_BIAS1_X] = 0.0f;
//    p[IMU_MAG_BIAS1_Y] = 0.0f;
//    p[IMU_MAG_BIAS1_Z] = 0.0f;
//    p[IMU_MAG_BIAS2_X] = 0.0f;
//    p[IMU_MAG_BIAS2_Y] = 0.0f;
//    p[IMU_MAG_BIAS2_Z] = 0.0f;
//    p[IMU_MAG_BIAS3_X] = 0.0f;
//    p[IMU_MAG_BIAS3_Y] = 0.0f;
//    p[IMU_MAG_BIAS3_Z] = 0.0f;
//    p[IMU_MAG_SCAL_X] = calibData.U[0];
//    p[IMU_MAG_SCAL_Y] = calibData.U[4];
//    p[IMU_MAG_SCAL_Z] = calibData.U[8];
//    p[IMU_MAG_SCAL1_X] = 0.0f;
//    p[IMU_MAG_SCAL1_Y] = 0.0f;
//    p[IMU_MAG_SCAL1_Z] = 0.0f;
//    p[IMU_MAG_SCAL2_X] = 0.0f;
//    p[IMU_MAG_SCAL2_Y] = 0.0f;
//    p[IMU_MAG_SCAL2_Z] = 0.0f;
//    p[IMU_MAG_SCAL3_X] = 0.0f;
//    p[IMU_MAG_SCAL3_Y] = 0.0f;
//    p[IMU_MAG_SCAL3_Z] = 0.0f;
//    parameter[IMU_MAG_ALGN_XY] = calibData.U[1];
//    parameter[IMU_MAG_ALGN_XZ] = calibData.U[2];
//    parameter[IMU_MAG_ALGN_YX] = calibData.U[3];
//    parameter[IMU_MAG_ALGN_YZ] = calibData.U[5];
//    parameter[IMU_MAG_ALGN_ZX] = calibData.U[6];
//    parameter[IMU_MAG_ALGN_ZY] = calibData.U[7];

	calibFree:

//    if (D.pData)
//	aqFree(D.pData, 10*8*CALIB_SAMPLES, sizeof(float32_t));
//    if (R.pData)
//	aqFree(R.pData, 20*10, sizeof(float32_t));
	for(uint8_t i = 0;i < 3;i++){
		parameter[IMU_MAG_BIAS_X + i] = calibData.bias[i];	
	}
}

void calibFinished(void) {
	calibCalculate();
	supervisorStateSwitch(STATE_MAGCALI,DISABLE);
	digitalHi(&supervisorData.flashSave);
}

/*--------------- 此函数为磁力计球状拟合校准 --------------*/
void calibrate(float * mag) 
{
  float32_t vec[3];
	int i;

	if (!(supervisorData.state & STATE_MAGCALI))
			return;

#ifdef USE_DIGITAL_IMU																									//赋值
	vec[0] = mag[0];
	vec[1] = mag[1];
	vec[2] = mag[2];
#else
		
#endif

	for (i = 0; i < 3; i++){																						//更新极值数据	
		if (vec[i] < calibData.min[i])																	
				calibData.min[i] = vec[i];																	//给最小值赋值
		if (vec[i] > calibData.max[i])
				calibData.max[i] = vec[i];																	//给最大值赋值

		calibData.bias[i] = -(calibData.min[i] + calibData.max[i]) * 0.5f;								//取中间值作为校准值
		calibRecalc();																									//重新计算
	}

	if (calibData.lastVec[0] == CALIB_EMPTY_SLOT){											//如果上一个值是100.0f，则赋当前值
		calibData.lastVec[0] = vec[0];
		calibData.lastVec[1] = vec[1];
		calibData.lastVec[2] = vec[2];
	}
	else if (calibAngle(calibData.lastVec, vec) * RAD_TO_DEG > CALIB_MIN_ANGLE){	//如果此时的向量和上一个向量间的夹角大于25度
		if (fabsf(calibData.min[0] - calibData.max[0]) > 1.0f &&
			fabsf(calibData.min[1] - calibData.max[1]) > 1.0f &&
			fabsf(calibData.min[2] - calibData.max[2]) > 1.0f){							//当所有最小值和最大值的绝对值大于1.0时结束校准
			calibInsert(vec);
			if (calibCount() == 8 * CALIB_SAMPLES)													//需要等于160
				calibFinished();																						//校准结束
			calibData.lastVec[0] = vec[0];
			calibData.lastVec[1] = vec[1];
			calibData.lastVec[2] = vec[2];
		}
	}
}
		
