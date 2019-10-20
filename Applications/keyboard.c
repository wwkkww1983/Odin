#include "keyboard.h"
#include "ramp.h"
#include "control.h"
#include "gimbal.h"
#include "chassis.h"
#include "vision.h"
#include "shoot.h"
#include "deforming.h"
#include "rc.h"
#include "math.h"
#include "config.h"

keyBoardCtrlStruct_t keyBoardCtrlData;
keyBoardState_e leftKeyDriver(void){      
	static uint8_t keyState = 0;         // 按键状态变量
	static uint16_t keyTime = 0;         // 按键计时变量
	uint8_t keyPress;
	keyBoardState_e keyReturn; 

	keyReturn = KEY_RELEASE;                         			 // 清除 返回按键值
	keyPress = remoteControlData.dt7Value.mouse.Press_L;   // 当前键值
	switch (keyState){       
		case KEY_STATE_0:{                  // 按键状态0：判断有无按键按下
			if(keyPress){               // 有按键按下
				keyTime = 0;                   // 清零时间间隔计数
				keyState = KEY_STATE_1;        // 然后进入 按键状态1
			}        
		}break;
		case KEY_STATE_1:{                  // 按键状态1：软件消抖（确定按键是否有效，而不是误触）。按键有效的定义：按键持续按下超过设定的消抖时间。
			if(keyPress){                     
				keyTime++;                     // 一次5ms
				if(keyTime>=SINGLE_PRESS_TIME){   // 消抖
					keyState = KEY_STATE_2;      // 如果按键时间超过 消抖时间，即判定为按下的按键有效。按键有效包括两种：单击或者长按，进入 按键状态2， 继续判定到底是那种有效按键
				}
			}         
			else keyState = KEY_STATE_0;      // 如果按键时间没有超过，判定为误触，按键无效，返回 按键状态0，继续等待按键
		}break; 
		case KEY_STATE_2:{                   // 按键状态2：判定按键有效的种类：是单击，还是长按
			if(!keyPress){                // 如果按键在 设定的长按时间 内释放，则判定为单击
				keyReturn = KEY_PRESS_ONCE;      // 返回 有效按键值：单击
				keyState = KEY_STATE_0;         // 返回 按键状态0，继续等待按键
			} 
			else{
				keyTime++;                     
				if(keyTime >= LONG_PRESS_TIME){   // 如果按键时间超过 设定的长按时间
					keyReturn = KEY_PRESS_LONG;            // 返回 有效键值值：长按
					keyState = KEY_STATE_3;       // 去状态3，等待按键释放
				}
			}
		}break;
		case KEY_STATE_3:{                   // 等待按键释放
			if (!keyPress){ 
				keyState = KEY_STATE_0;         // 按键释放后，进入 按键状态0 ，进行下一次按键的判定
			}
			else	keyReturn = KEY_PRESS_LONG;				
		}break; 
	}
	return keyReturn;                     // 返回 按键值
}
keyBoardState_e rightKeyDriver(void){     
	volatile static uint8_t keyState = 0;    				// 按键状态变量
	volatile static uint16_t keyTime = 0;           // 按键计时变量
	uint8_t keyPress;
	keyBoardState_e keyReturn; 

	keyReturn = KEY_RELEASE;                         // 清除 返回按键值
	keyPress = remoteControlData.dt7Value.mouse.Press_R;   // 当前键值
	switch (keyState){       
		case KEY_STATE_0:{                  // 按键状态0：判断有无按键按下
			if(keyPress){               // 有按键按下
				keyTime = 0;                   // 清零时间间隔计数
				keyState = KEY_STATE_1;        // 然后进入 按键状态1
			}        
		}break;
		case KEY_STATE_1:{                  // 按键状态1：软件消抖（确定按键是否有效，而不是误触）。按键有效的定义：按键持续按下超过设定的消抖时间。
			if(keyPress){                     
				keyTime++;                     // 一次5ms
				if(keyTime>=SINGLE_PRESS_TIME){   // 消抖
					keyState = KEY_STATE_2;      // 如果按键时间超过 消抖时间，即判定为按下的按键有效。按键有效包括两种：单击或者长按，进入 按键状态2， 继续判定到底是那种有效按键
				}
			}         
			else keyState = KEY_STATE_0;      // 如果按键时间没有超过，判定为误触，按键无效，返回 按键状态0，继续等待按键
		}break; 
		case KEY_STATE_2:{                   // 按键状态2：判定按键有效的种类：是单击，还是长按
			if(keyPress == 0){                // 如果按键在 设定的长按时间 内释放，则判定为单击
				keyReturn = KEY_PRESS_ONCE;      // 返回 有效按键值：单击
				keyState = KEY_STATE_0;         // 返回 按键状态0，继续等待按键
			} 
			else{
				keyTime++;                     
				if(keyTime >= LONG_PRESS_TIME){   // 如果按键时间超过 设定的长按时间
					keyReturn = KEY_PRESS_LONG;            // 返回 有效键值值：长按
					keyState = KEY_STATE_3;       // 去状态3，等待按键释放
				}
			}
		}break;
		case KEY_STATE_3:{                   // 等待按键释放
			if (keyPress == 0){ 
				keyState = KEY_STATE_0;         // 按键释放后，进入 按键状态0 ，进行下一次按键的判定
			}
			else	keyReturn = KEY_PRESS_LONG;				
		}break; 
	}
	return keyReturn;                     // 返回 按键值
}

void keyFsm(keyBoardState_e *sta, uint8_t key){
  switch (*sta){
    case KEY_RELEASE:{
      if (key)
        *sta = KEY_WAIT_EFFECTIVE;    //按键按下       
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_WAIT_EFFECTIVE:{					//等待按键按下有效（消抖）
      if (key){	
        *sta = KEY_PRESS_ONCE;        //状态转移
			}
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_ONCE:{							//按键按下有效
      if (key){ 
        *sta = KEY_PRESS_DOWN;        //状态转移
        if (sta == &keyBoardCtrlData.lkSta)
          keyBoardCtrlData.lk_cnt = 0;  //左长按计数清零
        else
          keyBoardCtrlData.rk_cnt = 0; //右长按计数清零	
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_DOWN:{							//按键一直按着
      if (key){  
        if (sta == &keyBoardCtrlData.lkSta){
          if (keyBoardCtrlData.lk_cnt++ > LONG_PRESS_TIME)//按下足够时间
            *sta = KEY_PRESS_LONG;                                  //长按状态
        }
        else{
          if (keyBoardCtrlData.rk_cnt++ > LONG_PRESS_TIME)
            *sta = KEY_PRESS_LONG;
        }
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_LONG:{							//长按状态下
      if (!key){											//没有按键按下
        *sta = KEY_RELEASE;     
      }
    }break;
    
    default:
			break;
  }
}

static void moveDirectionCtrl(uint8_t forward, uint8_t back,uint8_t left, uint8_t right,uint8_t fast,uint8_t slow,int16_t rotate) {
	static float forwardAccele = 0, backAccele = 0;      //保存减速的速度
	static float rightAccele = 0, leftAccele = 0;
	if(robotMode == MODE_KM){
	  keyBoardCtrlData.chassisSpeedTarget.z = rotate * REAL_MOTOR_SPEED_SCALE * 65;	
		if(forward){
			//速度逐渐递增
			forwardAccele += parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE * 0.75f;
			backAccele = 0;
			forwardAccele = constrainFloat(forwardAccele, 0.0f, parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE);
			//按下按键速度从500开始逐渐递增，按下shift直接满速
			keyBoardCtrlData.chassisSpeedTarget.y = MIN_CHASSIS_SPEED * chassisData.speedLimit \
													+ forwardAccele * chassisData.speedLimit \
													+ fast * parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, 0.0f, \
																   parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE);                                
			//保留当前前进速度，松开按键时以这个速度开始快速递减
			backAccele = keyBoardCtrlData.chassisSpeedTarget.y; 						
		}
		else if(back){
			//清除用于递增的变量
			forwardAccele = 0;
			backAccele -= parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE * 0.75f;
			//后退速度为前进速度一半
			backAccele = constrainFloat(backAccele, -0.5f * parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE, 0.0f);
			//按下按键速度从500开始逐渐递增，按下shift直接满速
			keyBoardCtrlData.chassisSpeedTarget.y = -MIN_CHASSIS_SPEED * chassisData.speedLimit \
													+ backAccele * chassisData.speedLimit \
													- fast * parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, \
																-0.5f*parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE, 0.0f);     
		}
		else{
			//松开按键时清除左右键递增的变量
			forwardAccele = 0;
			backAccele = 0;
			//之前按的是前键
			if(keyBoardCtrlData.chassisSpeedTarget.y >= 0) { 		 					
				//松开按键时速度快速递减 
				keyBoardCtrlData.chassisSpeedTarget.y = (keyBoardCtrlData.chassisSpeedTarget.y \
														 - 4 * parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE) * (!fast);
				keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, 0.0, \
																	   parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE);  
			}
			//之前按的是后键
			if(keyBoardCtrlData.chassisSpeedTarget.y < 0) {
				//松开按键时速度快速递减 
				keyBoardCtrlData.chassisSpeedTarget.y = (keyBoardCtrlData.chassisSpeedTarget.y \
														 + 4 * parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE) * (!fast);
				keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, \
																-0.5f*parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE, 0.0f);  
			}
		}
		//按下右键时清除左键递增的变量
		if(right){
			leftAccele = 0;                                  
			//速度逐渐递增			
			rightAccele += parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE * 0.5f;          
			rightAccele = constrainFloat(rightAccele, 0.0f, parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE); 
			//按下按键速度从500开始逐渐递增，按下shift直接满速
			keyBoardCtrlData.chassisSpeedTarget.x = MIN_CHASSIS_SPEED * chassisData.speedLimit \
													+ rightAccele * chassisData.speedLimit \
													+ fast * parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, 0.0f, \
																   parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE);
		}
		//按下左键时清除右键递增的变量
		else if(left){
			rightAccele = 0;
			//速度逐渐递增
			leftAccele -= parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE * 0.5f;
			leftAccele = constrainFloat(leftAccele, -parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE, 0.0f); 
			//按下按键速度从500开始逐渐递增，按下shift直接满速
			keyBoardCtrlData.chassisSpeedTarget.x = -MIN_CHASSIS_SPEED * chassisData.speedLimit \
													+ leftAccele * chassisData.speedLimit \
													- fast * parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, \
																		-parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE,0.0f);                                
		}
		else{
			//松开按键时清除左右键递增的变量
			rightAccele = 0;                                							
			leftAccele = 0;
			//之前按的是右键
			if(keyBoardCtrlData.chassisSpeedTarget.x >= 0) { 							
				//松开按键时速度快速递减 
				keyBoardCtrlData.chassisSpeedTarget.x = (keyBoardCtrlData.chassisSpeedTarget.x \
														 - 4 * parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE) * (!fast);  
				keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, 0.0, \
																	   parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE);  
			}
			//之前按的是左键
			if(keyBoardCtrlData.chassisSpeedTarget.x < 0) {  
				//松开按键时速度快速递减 
				keyBoardCtrlData.chassisSpeedTarget.x = (keyBoardCtrlData.chassisSpeedTarget.x \
														 + 4 * parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE) * (!fast); 
				keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, \
																	   -parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE, 0.0f);  
			}
		}
	}
	else{
		keyBoardCtrlData.chassisSpeedTarget.x = keyBoardCtrlData.chassisSpeedTarget.y = 0;
		forwardAccele = backAccele = rightAccele = leftAccele = 0;
	}
}

static void gimbalOperationFunc(int16_t pitRefSpd, int16_t yawRefSpd){
	if( robotMode == MODE_KM ){
	#if YAW_SPEED_SINGLE
		if(robotMode == MODE_KM && !(gimbalData.autoMode && visionData.captureFlagvisionData.captureFlag&&visionData.cailSuccess) \
			&& !infantryAutoData.rotateFlag && ROBOT == INFANTRY_ID){
		
			//鼠标给云台角速度期望，Yaw轴闭角速度单环
			keyBoardCtrlData.yawSpeedTarget = -yawRefSpd * parameter[GIMBAL_KB_SCALE] * 32;
			//角速度期望限幅
			keyBoardCtrlData.yawSpeedTarget = keyBoardCtrlData.yawSpeedTarget > 7.3f?  7.3f : \
			(keyBoardCtrlData.yawSpeedTarget < -7.3f? -7.3f : keyBoardCtrlData.yawSpeedTarget);
		}
		else{
			//闭角度外环
			keyBoardCtrlData.yawGyroTarget = yawRefSpd * parameter[GIMBAL_KB_SCALE] * 2;
		}
 	#else
		keyBoardCtrlData.yawGyroTarget = yawRefSpd * parameter[GIMBAL_KB_SCALE] * 2;
	#endif
		keyBoardCtrlData.pitchGyroTarget = -pitRefSpd * parameter[GIMBAL_KB_SCALE] * 2;
	}
	else{
		keyBoardCtrlData.pitchSpeedTarget = 0;
		keyBoardCtrlData.yawSpeedTarget   = 0;
		keyBoardCtrlData.pitchGyroTarget  = 0;
		keyBoardCtrlData.yawGyroTarget	  = 0;
	}
}

void getKeyboardMouseState(void){
   keyFsm(&keyBoardCtrlData.lkSta,remoteControlData.dt7Value.mouse.Press_L);
   keyFsm(&keyBoardCtrlData.rkSta,remoteControlData.dt7Value.mouse.Press_R);
}

void keyboardChassisHook(void){
	moveDirectionCtrl(FORWARD, BACK, LEFT, RIGHT, FAST_SPD, SLOW_SPD, remoteControlData.dt7Value.mouse.X);
}

void keyboardGimbalHook(void){
	gimbalOperationFunc(remoteControlData.dt7Value.mouse.Y, remoteControlData.dt7Value.mouse.X);
}



