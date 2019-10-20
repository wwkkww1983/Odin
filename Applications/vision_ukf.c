#include "vision_ukf.h"
#include "clockcount.h"

#define UKF_VEL_Q	+5.0e-4f
#define UKF_VEL_V	+5.0e-4f
#define UKF_VEL_N	+1.0e-5f

#define UKF_POS_Q	+5.0e-4f
#define UKF_POS_V	+5.0e-3f
#define UKF_POS_N	+2.0e-7f

#define UKF_POS_V_Z +5.0e-3f
#define UKF_POS_N_Z +6.0e-4f

visionUkfStruct_t visionUkfData;

void visionUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) {
	int i;
	out = in;
	for (i = 0; i < n; i++) {
		out[UKF_STATE_VEL_X*n + i] = in[UKF_STATE_VEL_X*n + i] + noise[UKF_V_VEL_X*n + i] * dt;
		out[UKF_STATE_VEL_Y*n + i] = in[UKF_STATE_VEL_Y*n + i] + noise[UKF_V_VEL_Y*n + i] * dt;
		out[UKF_STATE_VEL_Z*n + i] = in[UKF_STATE_VEL_Z*n + i] + noise[UKF_V_VEL_Z*n + i] * dt;

		out[UKF_STATE_POS_X*n + i] = in[UKF_STATE_POS_X*n + i] + in[UKF_STATE_VEL_X*n + i] * dt + noise[UKF_V_POS_X*n + i];
		out[UKF_STATE_POS_Y*n + i] = in[UKF_STATE_POS_Y*n + i] + in[UKF_STATE_VEL_Y*n + i] * dt + noise[UKF_V_POS_Y*n + i];
		out[UKF_STATE_POS_Z*n + i] = in[UKF_STATE_POS_Z*n + i] + in[UKF_STATE_VEL_Z*n + i] * dt + noise[UKF_V_POS_Z*n + i];
	}
}

void visionUkfPosUpdate(float *u, float *x, float *noise, float *y) {
	y[0] = x[UKF_STATE_POS_X] + noise[0];
	y[1] = x[UKF_STATE_POS_Y] + noise[1];
	y[2] = x[UKF_STATE_POS_Z] + noise[2];
}

void visionUkfVelUpdate(float *u, float *x, float *noise, float *y) {
	y[0] = x[UKF_STATE_VEL_X] + noise[0];
	y[1] = x[UKF_STATE_VEL_Y] + noise[1];
	y[2] = x[UKF_STATE_VEL_Z] + noise[2];
}

void visionUkfDoPosUpdate(float pos_x,float pos_y,float pos_z) {
	float noise[3];
	float y[3];
	//更新位置信息

	y[0] = pos_x;
	y[1] = pos_y;
	y[2] = pos_z;
	noise[0] = UKF_POS_N;
	noise[1] = UKF_POS_N;
	noise[2] = UKF_POS_N_Z;
	srcdkfMeasurementUpdate(visionUkfData.kf, 0, y, 3, 3, noise, visionUkfPosUpdate);
}

void visionUkfDoVelUpdate(float vel_x,float vel_y,float vel_z) {
	float noise[3];
	float y[3];
	//更新速度信息
	y[0] = vel_x;
	y[1] = vel_y;
	y[2] = vel_z;;
	noise[0] = UKF_VEL_N;
	noise[1] = UKF_VEL_N;
	noise[2] = UKF_VEL_N;
	srcdkfMeasurementUpdate(visionUkfData.kf, 0, y, 3, 3, noise, visionUkfVelUpdate);
}

void visionUkfProcess() {
	visionUkfData.time[0] = getClockCount();
	visionUkfData.intervalTime = (float)(visionUkfData.time[0] - visionUkfData.time[1]);
	visionUkfData.time[1] = visionUkfData.time[0];
	srcdkfTimeUpdate(visionUkfData.kf, 0, visionUkfData.intervalTime);
}

void visionUkfStateInit(float pos_x,float pos_y,float pos_z) {
	float Q[VISION_S];
	float V[VISION_V];
	
	Q[UKF_STATE_VEL_X] = UKF_VEL_Q;
	Q[UKF_STATE_VEL_Y] = UKF_VEL_Q;
	Q[UKF_STATE_VEL_Z] = UKF_VEL_Q;
	Q[UKF_STATE_POS_X] = UKF_POS_Q;
	Q[UKF_STATE_POS_Y] = UKF_POS_Q;
	Q[UKF_STATE_POS_Z] = UKF_POS_Q;

	V[UKF_V_VEL_X] = UKF_VEL_V;
	V[UKF_V_VEL_Y] = UKF_VEL_V;
	V[UKF_V_VEL_Z] = UKF_VEL_V;
	V[UKF_V_POS_X] = UKF_POS_V;
	V[UKF_V_POS_Y] = UKF_POS_V;
	V[UKF_V_POS_Z] = UKF_POS_V_Z;
	
	srcdkfSetVariance(visionUkfData.kf, Q, V, 0, 0);
	UKF_VEL_X = 0.0;
	UKF_VEL_Y = 0.0;
	UKF_VEL_Z = 0.0;
	UKF_POS_X = pos_x;
	UKF_POS_Y = pos_y;
	UKF_POS_Z = pos_z;
}

void visionUkfInit() {
	float Q[VISION_S];
	float V[VISION_V];
	memset((void *)&visionUkfData.kf, 0, sizeof(visionUkfData.kf));

	visionUkfData.kf = srcdkfInit(VISION_S, VISION_M, VISION_V, VISION_N, visionUkfTimeUpdate);
	visionUkfData.x = srcdkfGetState(visionUkfData.kf);

	Q[UKF_STATE_VEL_X] = UKF_VEL_Q;
	Q[UKF_STATE_VEL_Y] = UKF_VEL_Q;
	Q[UKF_STATE_VEL_Z] = UKF_VEL_Q;
	Q[UKF_STATE_POS_X] = UKF_POS_Q;
	Q[UKF_STATE_POS_Y] = UKF_POS_Q;
	Q[UKF_STATE_POS_Z] = UKF_POS_Q;

	V[UKF_V_VEL_X] = UKF_VEL_V;
	V[UKF_V_VEL_Y] = UKF_VEL_V;
	V[UKF_V_VEL_Z] = UKF_VEL_V;
	V[UKF_V_POS_X] = UKF_POS_V;
	V[UKF_V_POS_Y] = UKF_POS_V;
	V[UKF_V_POS_Z] = UKF_POS_V;

	srcdkfSetVariance(visionUkfData.kf, Q, V, 0, 0);
	UKF_VEL_X = 0.0f;
	UKF_VEL_Y = 0.0f;
	UKF_VEL_Z = 0.0f;
	UKF_POS_X = 0.0f;
	UKF_POS_Y = 0.0f;
	UKF_POS_Z = 0.0f;
}
