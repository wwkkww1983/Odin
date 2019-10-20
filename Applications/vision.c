#include "application.h"
#include "ramp.h"

ramp_t UKFXVisionRamp = RAMP_GEN_DAFAULT;											//云台斜坡初始化
ramp_t UKFYVisionRamp = RAMP_GEN_DAFAULT;
ramp_t UKFZVisionRamp = RAMP_GEN_DAFAULT;

#define USE_PREJUDG 1						//装甲板打击预判使能（测试中）
#define BUFF_SHOOT_INTERVAL_TIME 800  //BUFF_SHOOT_INTERVAL_TIME = 神符射击间隔时间（ms）

visionStruct_t visionData;

void visionStoreHistoricalData(float pitch, float yaw, uint16_t CNTR){
	visionData.pitchDataSet[visionData.storedIndex] = pitch;
	visionData.yawDataSet[visionData.storedIndex] = yaw;
	visionData.CNTR_DataSet[visionData.storedIndex] = CNTR;
	visionData.storedIndex = (visionData.storedIndex + 1) % VISION_STORED_LENGTH;
}

//返回50以内的数证明找到了对应角度或最接近的下标，返回255说明没有适配目标
uint8_t visionMatchAngle(void){
	uint16_t min = 65535;
	uint8_t r = 0;
	for(uint8_t index = 0; index < VISION_STORED_LENGTH; index++){
		//如果CNTR相同则直接返回下标
		if(visionData.CNTR_DataSet[index] == visionData.CNTR.u16_temp){
			return index;
		}
		if(abs(visionData.CNTR_DataSet[index] - visionData.CNTR.u16_temp) < min){
            min = abs(visionData.CNTR_DataSet[index] - visionData.CNTR.u16_temp);
            r = index;
        }
	}
	//如果没有找到相同的，为防止丢包，在+-1这两个数再找
	if(min < 2){
		return r;
	}
	else{
		return 255;
	}
}

//模式与子弹类型刷新
void visionSendDataUpdate(uint8_t workMode,uint8_t bullet){
	visionData.workMode = (visionWorkMode_e)workMode;
	visionData.bullet = (bulletType_e)bullet;
}
//视觉发送数据初始化
void visionSendDataInit(void){
	visionData.workMode = TX2_STOP;
	visionData.bullet = SMALL_BULLET;
}
//阵营更新
void identifyCamp(void){
	if(robotConfigData.typeOfRobot == SMALLGIMBAL_ID){
		visionData.enemyType = controlTransData.otherEnemyType;
	}
	else{
		if(ROBOT_ID > 0x07){									//如果己方是蓝方
			visionData.enemyType = ENEMY_RED;	  //敌军
		}							
		else{
			visionData.enemyType = ENEMY_BLUE;
		}	
	}	
}
//视觉射击初始化
void shootVisionInit(void){
	if(robotConfigData.typeOfRobot == TANK_ID){
			visionData.shootSpeed = BIG_SHOOT;
	}
	else if(robotConfigData.typeOfRobot == SMALLGIMBAL_ID){
	    shootData.shootMode = AUTO_CONTINUOUS;
			visionData.shootSpeed = SMALL_MANUAL_CONTINUOUS;
	}
	else if(robotConfigData.typeOfRobot == SENTRY_ID){
	    shootData.shootMode = AUTO_CONTINUOUS;
			visionData.shootSpeed = SMALL_MANUAL_SINGLE;
	}
	else{
		switch(shootData.shootMode){
			case MANUAL_SINGLE:
				visionData.shootSpeed = SMALL_MANUAL_SINGLE;
				break;
			case MANUAL_CONTINUOUS:
				visionData.shootSpeed = SMALL_MANUAL_CONTINUOUS;
				break;
			case AUTO_CONTINUOUS:
				visionData.shootSpeed = SMALL_AUTO_CONTINUOUS;
				break;
		}
	}
}
//视觉自动射击控制
void shootVisionControl(keyBoardState_e realKeyValue,uint8_t * shootFlag){	
	switch(shootData.shootMode){
		case MANUAL_SINGLE:											//单发
		case MANUAL_CONTINUOUS:{										
			if(realKeyValue == KEY_PRESS_ONCE){		//鼠标单击
				if(shootData.fricMotorFlag){
					digitalHi(shootFlag);   					//射击使能
				}	
			}
		}break;
		case AUTO_CONTINUOUS:{
			if(realKeyValue){											//鼠标按下
				if(shootData.fricMotorFlag){
					digitalHi(shootFlag);   					//射击使能
				}	
			}
			else{																	//鼠标松开
				digitalLo(shootFlag);								//射击失能
				digitalLo(&shootData.shootTrigger);	//拨盘失能
			}
		}break;
	}
}
//自瞄射击决策
float shootLimit = 0.0f;
uint32_t cnt;
float limitScale = 0.1f;
void visionFireFbdUpdata(uint8_t * shootFlag){	
//根据距离对自动射击准许偏差进行调整
	float scale = constrainFloat(5.0f - UKF_POS_Z, 0.0f, 5.0f) * 3;//0-1
	if((visionData.workMode ==TX2_DISTINGUISH_BUFF)&&(visionData.captureFlag == 1)&&(visionData.buffOfDirRota == STOP)){
		shootLimit = 0.4f* (1.0f + scale) + visionData.predictShootLimit;
	}
	else if((visionData.workMode ==TX2_DISTINGUISH_BUFF)&&(visionData.captureFlag == 1)&&(visionData.buffOfDirRota != UNKNOW)){
		shootLimit = 0.8f* (1.0f + scale) + visionData.predictShootLimit;
	}
	else{
		shootLimit = 0.4f* (1.0f + scale);
	}
	float shootLimit_left = shootLimit;
	float shootLimit_right = -shootLimit;
	float shootLimit_up = -shootLimit;
	float shootLimit_down = shootLimit;
	if(IMU_RATEZ > 0.1f)
		shootLimit_left *= 1.5f;
	else if(IMU_RATEZ < -0.1f)
		shootLimit_right *= 1.5f;
	if(IMU_RATEY > 0.1f)
		shootLimit_down *= 1.5f;
	else if(IMU_RATEY < -0.1f)
		shootLimit_up *= 1.5f;	
	//开枪判断
	if(visionData.workMode == TX2_DISTINGUISH_ARMOR){
		if((visionData.angleBias[0] > shootLimit_right && visionData.angleBias[0] < shootLimit_left)
			&& (visionData.angleBias[1] > shootLimit_up && visionData.angleBias[1] < shootLimit_down)){
			if((!(*shootFlag))&&visionData.captureFlag&&visionData.cailSuccess){
				visionData.fireFbd = true;	//开枪使能
			}
			else if((*shootFlag) && visionData.captureFlag&&visionData.cailSuccess&& (shootData.shootMode == AUTO_CONTINUOUS)){
				visionData.fireFbd = true;	//开枪使能
			}
			else{
				visionData.fireFbd = false;	//开枪失能
			}
			cnt = 0;
		}
		else{
			visionData.fireFbd = false;
		}
	}
	else{
		if((fabsf(visionData.angleBias[0]) < shootLimit)&&(fabsf(visionData.angleBias[1]) < shootLimit)){
			if((!(*shootFlag))&&visionData.captureFlag&&visionData.cailSuccess){
				visionData.fireFbd = true;	//开枪使能
			}
			else if((*shootFlag) && visionData.captureFlag&&visionData.cailSuccess&& (shootData.shootMode == AUTO_CONTINUOUS)){
				visionData.fireFbd = true;	//开枪使能
			}
			else{
				visionData.fireFbd = false;	//开枪失能
			}
			cnt = 0;
		}
		else{
			visionData.fireFbd = false;		//开枪失能
			cnt++;
		}
	}
}
//视觉射击状态更新
void shootVisionUpdate(keyBoardState_e realKeyValue,visionWorkMode_e mode,uint8_t * shootFlag){
	//static TickType_t buffFireLastWakeTime = 0;	  //时间监测
	visionFireFbdUpdata(shootFlag);
	//按键早已释放
	if(realKeyValue == KEY_RELEASE){
		visionData.fireFbd = false;		//开枪失能
	}
	if(mode ==  TX2_DISTINGUISH_ARMOR){
		if(!visionData.prejudgFlag){
			//射击模式决策
			if(visionData.fireFbd){
				shootVisionControl(realKeyValue,shootFlag);
			}					
		}
		else{
			shootVisionControl(realKeyValue,shootFlag);
		}	
	}
//	else if(mode ==  TX2_DISTINGUISH_BUFF){
//		//射击模式决策
//		if(visionData.fireFbd){
//			if(realKeyValue){		//鼠标按下			
//				if(xTaskGetTickCount() - buffFireLastWakeTime > BUFF_SHOOT_INTERVAL_TIME){
//					buffFireLastWakeTime = xTaskGetTickCount();
//					if(shootData.fricMotorFlag){
//						digitalHi(shootFlag);   //射击使能
//					}			
//				}			
//			}
//			else{								//鼠标松开
//				digitalLo(shootFlag);		//射击失能
//				digitalLo(&shootData.shootTrigger);	//拨盘失能
//			}
//		}	
//	}
}

//目标速度计算
void getVisionRate(formatTrans16Struct_t* pos){
	//计算得到时间间隔
	visionData.time[0] = getClockCount();
	visionData.intervalTime = (float)(visionData.time[0] - visionData.time[1]);
	visionData.time[1] = visionData.time[0];	
	for (int i = 0; i < 3; i++) {
		visionData.targeRate[i] = (MM_TO_M_CONVERSION(&pos[i].s16_temp) - visionData.lastCoordinate[i])/visionData.intervalTime;			
		visionData.lastCoordinate[i] = MM_TO_M_CONVERSION(&pos[i].s16_temp);
	}
}
//抛物线函数及其导数计算
void ballisticParabolaFunCalculation(float resRadian,float * originalValue,float * derivativeValue, float P, float H, float v){
	float sinResAngle = arm_sin_f32(resRadian);
	float cosResAngle = arm_cos_f32(resRadian);
	float tanResAngle = sinResAngle / (cosResAngle + 1.0e-10f);
	float v_pow = v * v;
	float p_pow = P * P;
	float cosRes_pow = cosResAngle * cosResAngle;
	
	*originalValue = tanResAngle * P - 0.5f * GRAVITY * p_pow / (v_pow * cosRes_pow+ + 1.0e-10f) - H;
	*derivativeValue = P / (cosRes_pow + 1.0e-10f) + (GRAVITY * p_pow / (v_pow + 1.0e-10f)) * tanResAngle / (cosRes_pow + 1.0e-10f);
}

uint16_t TEST_CNT = 0;
float P = 0.0f;
//牛顿迭代法（pitch轴）
float newtonIterationMethod(){
	float resPitchAngle = visionData.pitchReal - atan(visionData.coordinatePredict[1] / visionData.coordinatePredict[2]) \
						  * 180.0f / PI;
	float originalValue;
	float derivativeValue;
	uint16_t cnt = 0;
	float slope = 1.0f;
	
	//相机坐标
	float y = -visionData.coordinatePredict[1];
	float z = visionData.coordinatePredict[2];
	//射击速度
	float v = visionData.shootSpeed;
	//解算实际世界坐标参数
	//深度，高度			
	float alpha = visionData.pitchReal;  					//当前pitch角
	float beta = atan(y / z) * 180.0f / PI;					//云台中心与装甲板夹角
	float gamma = alpha + beta;								//与水平夹角
	//相机坐标系下与物体的夹角三角函数
	float sinBeta = sin(ANGLE_TO_RADIAN(&beta));
	//水平坐标系下转轴中心与物体的夹角三角函数
	float sinGamma = sin(ANGLE_TO_RADIAN(&gamma));
	float cosGamma = cos(ANGLE_TO_RADIAN(&gamma));
	
	//水平坐标系下物体的真实距离和高度
	P = (y / (sinBeta + 1.0e-10f)) * cosGamma;
	float H = (y / (sinBeta + 1.0e-10f)) * sinGamma;
	
	while((fabsf(slope) > 1e-3f)&&(cnt < 400)){
		ballisticParabolaFunCalculation(ANGLE_TO_RADIAN(&resPitchAngle),&originalValue, &derivativeValue, P, H, v);
		if((derivativeValue != 0)&&(!isnan(derivativeValue))&&(!isnan(originalValue))
			&&(derivativeValue<1e+5f)&&(originalValue<1e+5f)){
			slope = originalValue / (derivativeValue + 1.0e-10f);
			if(slope < 70){
				resPitchAngle -= slope;
			}
		}
		
		cnt++;
	}
	TEST_CNT = cnt;
	return resPitchAngle;
} 

void visionPTZAnglePredict(){
	//static uint32_t turnBackCNT = 0;
	//static uint32_t resetCNT = 0;
	static float targetRunTime = 0;
	static float predictScale = 0.0f;
	static float lastVelX = 0.0f;
	static float lastPredictBias = 0.0f;
	targetRunTime = P / visionData.shootSpeed;
	visionData.realDelayTime = visionData.shootDelayTime + targetRunTime;
	visionData.baseBiasX= atan(UKF_POS_X / UKF_POS_Z) * 180.0f / PI;

	//不同目标预判清零
	if((!visionData.sameTargetFlag)||(fabsf(UKF_VEL_X-lastVelX)>5.0f)){
		visionData.predictBias = 0.0f;
		predictScale = 0.0f;
		visionData.fireFbd = false;	//开枪失能
	}
	else{
		visionData.predictBias = atan(UKF_VEL_X * visionData.realDelayTime/ visionData.coordinatePredict[2]) * 180.0f / PI;
	}
	//预判增量与实际偏差决策
	if(fabs(visionData.baseBiasX)<0.5f*shootLimit){
		visionData.baseBiasX = 0.0f;
	}	
	if(fabs(visionData.predictBias)< 0.8f*shootLimit){
		visionData.predictBias = 0.0f;
		limitScale = 0.0f;
		predictScale = 0.0f;
	}
	//预判限幅
	visionData.predictBias = constrainFloat(visionData.predictBias, -10.0f*shootLimit, 10.0f*shootLimit);//0-1
	visionData.yawCmdBuff = visionData.yawReal - gimbalData.yawAngleSave + visionData.baseBiasX ;
	
	if(fabs(visionData.predictBias)>0.8f*shootLimit){
		if(!(cnt%50)){
			predictScale += 0.1f;		
		}
		if(cnt>200){
			cnt = 0;
			limitScale += 0.05f;
		}
		
		predictScale = constrainFloat(predictScale, 0.0f, 1.0f);//0-1
		limitScale = constrainFloat(limitScale, 0.0f, 0.5f);//0-1
	}
	else{
		predictScale = 0.0f;
		limitScale = 0.0f;
	}
	
	
	visionData.predictShootLimit = limitScale;
	if(lastPredictBias != 0.0f){
		if((fabs(visionData.predictBias - lastPredictBias)>3)){
			visionData.predictBias = 0;
		}
	}

	
	visionData.yawCmdBuff += visionData.predictBias;
	float buff = visionData.yawCmdBuff;
	if(visionData.predictBias>0){
		visionData.yawCmdBuff += predictScale;
	}
	else{
		visionData.yawCmdBuff -= predictScale;
	}
	

	visionData.pitchCmdBuff = newtonIterationMethod();	
	
	visionData.angleBias[0] =  buff - 0.2f * predictScale - visionData.yawReal + gimbalData.yawAngleSave;
  visionData.angleBias[1] =  visionData.pitchCmdBuff - visionData.pitchReal;
	lastVelX = UKF_VEL_X;
	lastPredictBias = visionData.predictBias;
}

//float testAng = 2.0f;
//视觉云台到达角度计算
void visionPTZAngleCalculation(){

//#if USE_PREJUDG
//	visionPTZAnglePredict();
//#else
//	visionData.angleBias[0] = atan(UKF_POS_X / UKF_POS_Z) * 180.0f / PI;
//	visionData.yawCmdBuff = visionData.yawReal - gimbalData.yawAngleSave + atan(visionData.coordinatePredict[0] / visionData.coordinatePredict[2]) * 180.0f / PI;	
//	visionData.pitchCmdBuff = newtonIterationMethod();	
//	visionData.angleBias[1] = visionData.pitchCmdBuff - visionData.pitchReal;
//#endif
	if((ROBOT == SMALLGIMBAL_ID)||(ROBOT == SENTRY_ID))	{
		visionPTZAnglePredict();
	}
	else{
		visionData.angleBias[0] = atan(UKF_POS_X / UKF_POS_Z) * 180.0f / PI;
		visionData.yawCmdBuff = visionData.yawReal - gimbalData.yawAngleSave + atan(visionData.coordinatePredict[0] / visionData.coordinatePredict[2]) * 180.0f / PI;	
		visionData.pitchCmdBuff = newtonIterationMethod();	
		if(visionData.workMode == TX2_DISTINGUISH_ARMOR){
			visionData.pitchCmdBuff += visionData.buffBias;
		}	
		visionData.angleBias[1] = visionData.pitchCmdBuff - visionData.pitchReal;
	}
	if((fabs(visionData.angleBias[0])<80.0f)&&(!isnan(visionData.yawCmdBuff))){
			visionData.yawCmd = visionData.yawCmdBuff;	
			visionData.lastYawCmdBuff = visionData.yawCmdBuff;	
      visionData.lastPredictBias = visionData.predictBias;	
	}
	if((fabs(visionData.angleBias[1])<80.0f)&&(!isnan(visionData.pitchCmdBuff))){
		visionData.pitchCmd = visionData.pitchCmdBuff;
	}
	
	//手动预判
	if(visionData.prejudgFlag&&(visionData.workMode == TX2_DISTINGUISH_ARMOR)){
		visionData.pitchCmd = visionData.pitchCmd + visionData.manualPitchBias;
		visionData.yawCmd = visionData.yawCmd + visionData.manualYawBias;
	}
}

//迫击炮云台到达角度计算
void visionMortar(){
	visionData.mortarYawCmdBuff = 30.0f;	
	visionData.mortarPitCmdBuff = 30.0f;	

	visionData.mortarYawCmd = visionData.manualYawBias + visionData.mortarYawCmdBuff; 
	visionData.mortarPitCmd = visionData.manualPitchBias + visionData.mortarPitCmdBuff;
}

//实际解算坐标计算
void visionCoordinateCalculation(){
			switch(visionData.workMode){
				case TX2_STOP:
				case TX2_DISTINGUISH_BUFF:{
					visionData.coordinatePredict[0] = UKF_POS_X;
					visionData.coordinatePredict[1] = UKF_POS_Y;
					visionData.coordinatePredict[2] = UKF_POS_Z;
				}break;
				case TX2_DISTINGUISH_ARMOR:{
					visionData.coordinatePredict[0] = UKF_POS_X;
					visionData.coordinatePredict[1] = UKF_POS_Y;
					visionData.coordinatePredict[2] = UKF_POS_Z;				
				}break;
			}	
}
void  visionPredictReset(){
	for (int i = 0; i < 3; i++) {
		visionData.targeRate[i] = 0.0f;	
		UKF_VEL_X = 0.0;
		UKF_VEL_Y = 0.0;
		UKF_VEL_Z = 0.0;					
	}
	visionData.pitchCmdBuff = 0.0f;
	visionData.yawCmdBuff = 0.0f;
	visionData.lastYawCmdBuff = 0.0f;
	visionData.lastYawCmd = 0.0f;
	visionData.baseBiasX = 0.0f;
	visionData.baseBiasBuffX = 0.0f;
	visionData.baseBiasBuffY = 0.0f ;
	visionData.predictBias = 0.0f;
	visionData.lastPredictBias = 0.0f;
	visionData.lastPitchCmd  = 0.0f;
	visionData.predictShootLimit = 0.0f;
	cnt = 0;
}
//视觉任务更新
void visionUpdateTask(void *Parameters){
	static bool captureFlag = 0,lastCaptureFlag = 0;	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(true){
		vTaskDelayUntil(&xLastWakeTime,VISION_PERIOD);
		captureFlag = visionData.captureFlag;
		//ukf滤波
		if(!captureFlag){
			//没有识别到目标则重置滤波器
			lastCaptureFlag = captureFlag;
		}
		else{
			//根据CNTR得到当前坐标所属的历史角度，找到则赋值，找不到跳出本次循环
			uint8_t dataMatchIndex = visionMatchAngle();
			if(dataMatchIndex != 255){
				visionData.pitchReal = visionData.pitchDataSet[dataMatchIndex];
				visionData.yawReal = visionData.yawDataSet[dataMatchIndex];
			}
			else{
				goto loopIncreasing;
			}
			
			if((visionData.coordinateBase[2].s16_temp < 50)
				||(visionData.coordinateBase[2].s16_temp <= visionData.coordinateBase[0].s16_temp)
				||(visionData.coordinateBase[2].s16_temp <= visionData.coordinateBase[1].s16_temp)){
				goto loopIncreasing;
			}
			//第一次更新则初始化
			if((!lastCaptureFlag)||(fabs(UKF_VEL_X)>3.0f)){
				memset((void *)&visionData.posHist, 0, sizeof(visionData.posHist));
				memset((void *)&visionData.sumPos, 0, sizeof(visionData.sumPos));
				for(int i = 0; i < 3; i++){
					visionData.lastCoordinate[i] = MM_TO_M_CONVERSION(&visionData.coordinate[i].s16_temp);
				}
				visionUkfStateInit(MM_TO_M_CONVERSION(&visionData.coordinateBase[0].s16_temp),\
								   MM_TO_M_CONVERSION(&visionData.coordinateBase[1].s16_temp),\
								   MM_TO_M_CONVERSION(&visionData.coordinateBase[2].s16_temp));
				RampResetCounter(&UKFXVisionRamp);
				RampResetCounter(&UKFYVisionRamp);
				RampResetCounter(&UKFZVisionRamp);
				visionPredictReset();
			}
			
			if(visionData.CNTR.u16_temp != visionData.lastCNTR){
				getVisionRate(visionData.coordinate);
				visionData.cailSuccess = false;
			}
			else{
				visionData.cailSuccess = true;
			}
			visionData.lastCNTR = visionData.CNTR.u16_temp;
			//先验
			visionUkfProcess();
			
			if(!(visionData.loops % 2)){
				//位置后验更新
				visionUkfDoPosUpdate(MM_TO_M_CONVERSION(&visionData.coordinateBase[0].s16_temp),\
														MM_TO_M_CONVERSION(&visionData.coordinateBase[1].s16_temp),\
														MM_TO_M_CONVERSION(&visionData.coordinateBase[2].s16_temp));
			}
			else{
				//速度后验更新
				visionUkfDoVelUpdate(visionData.targeRate[0],visionData.targeRate[1],visionData.targeRate[2]);				
			}		
			//实际解算坐标计算
			visionCoordinateCalculation();
			//角度计算
			visionPTZAngleCalculation();
			lastCaptureFlag = captureFlag;
		}
		loopIncreasing:
			digitalIncreasing(&visionData.loops);
	}
}

void miniPTZAutomaticAimUpdate(uint8_t * schedule){												//辅助打击/自瞄任务更新
	static TickType_t miniPTZLastWakeTime = 0;	  //时间监测
	switch(*schedule){													
		case 1: {
			//gimbalSwitch(DISABLE);												
			//chassisSwitch(DISABLE);
			//配置TX2数据
			visionSendDataUpdate(TX2_DISTINGUISH_ARMOR, SMALL_BULLET);		
			digitalIncreasing(schedule);	
			break;
		}
		case 2: {
			//gimbalSwitch(ENABLE);	
			shootVisionInit();
			//自瞄模式决策
			visionData.prejudgFlag = false;											
			//自瞄射击决策				
			visionFireFbdUpdata(&shootData.fireFlag_17mm);						
			if(visionData.fireFbd && controlTransData.otherAutoMaticFlag){
				if(shootData.fricMotorFlag){
					digitalHi(&shootData.fireFlag_17mm);   //射击使能
					miniPTZLastWakeTime = xTaskGetTickCount();
				}
			}	
			if(xTaskGetTickCount() - miniPTZLastWakeTime > 200){
					digitalLo(&shootData.fireFlag_17mm);		//射击失能
					digitalLo(&shootData.shootTrigger);	//拨盘失能
			};	//拨盘失能			
			break;
		}
	}
}


//视觉数据初始化
void visionInit(void){
	memset((void *)&visionData, 0, sizeof(visionData));
	visionUkfInit();
	RampInit(&UKFXVisionRamp, 150);
	RampInit(&UKFYVisionRamp, 150);
	RampInit(&UKFZVisionRamp, 150);
	visionData.lastCNTR = visionData.CNTR.u16_temp;
	visionData.initFlag = true;
	shootVisionInit();
	visionData.shootDelayTime = 0.05f;
	visionData.prejudgFlag = false;
	visionData.rBiasMode = 0;
	visionData.buffBias = 1.0f;
	supervisorData.taskEvent[VISION_TASK] = xTaskCreate(visionUpdateTask, "VISION", VISION_STACK_SIZE, NULL, \
														VISION_PRIORITY, &visionData.xHandleTask);
}
