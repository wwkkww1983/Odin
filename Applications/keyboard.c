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
	static uint8_t keyState = 0;         // ����״̬����
	static uint16_t keyTime = 0;         // ������ʱ����
	uint8_t keyPress;
	keyBoardState_e keyReturn; 

	keyReturn = KEY_RELEASE;                         			 // ��� ���ذ���ֵ
	keyPress = remoteControlData.dt7Value.mouse.Press_L;   // ��ǰ��ֵ
	switch (keyState){       
		case KEY_STATE_0:{                  // ����״̬0���ж����ް�������
			if(keyPress){               // �а�������
				keyTime = 0;                   // ����ʱ��������
				keyState = KEY_STATE_1;        // Ȼ����� ����״̬1
			}        
		}break;
		case KEY_STATE_1:{                  // ����״̬1�����������ȷ�������Ƿ���Ч���������󴥣���������Ч�Ķ��壺�����������³����趨������ʱ�䡣
			if(keyPress){                     
				keyTime++;                     // һ��5ms
				if(keyTime>=SINGLE_PRESS_TIME){   // ����
					keyState = KEY_STATE_2;      // �������ʱ�䳬�� ����ʱ�䣬���ж�Ϊ���µİ�����Ч��������Ч�������֣��������߳��������� ����״̬2�� �����ж�������������Ч����
				}
			}         
			else keyState = KEY_STATE_0;      // �������ʱ��û�г������ж�Ϊ�󴥣�������Ч������ ����״̬0�������ȴ�����
		}break; 
		case KEY_STATE_2:{                   // ����״̬2���ж�������Ч�����ࣺ�ǵ��������ǳ���
			if(!keyPress){                // ��������� �趨�ĳ���ʱ�� ���ͷţ����ж�Ϊ����
				keyReturn = KEY_PRESS_ONCE;      // ���� ��Ч����ֵ������
				keyState = KEY_STATE_0;         // ���� ����״̬0�������ȴ�����
			} 
			else{
				keyTime++;                     
				if(keyTime >= LONG_PRESS_TIME){   // �������ʱ�䳬�� �趨�ĳ���ʱ��
					keyReturn = KEY_PRESS_LONG;            // ���� ��Ч��ֵֵ������
					keyState = KEY_STATE_3;       // ȥ״̬3���ȴ������ͷ�
				}
			}
		}break;
		case KEY_STATE_3:{                   // �ȴ������ͷ�
			if (!keyPress){ 
				keyState = KEY_STATE_0;         // �����ͷź󣬽��� ����״̬0 ��������һ�ΰ������ж�
			}
			else	keyReturn = KEY_PRESS_LONG;				
		}break; 
	}
	return keyReturn;                     // ���� ����ֵ
}
keyBoardState_e rightKeyDriver(void){     
	volatile static uint8_t keyState = 0;    				// ����״̬����
	volatile static uint16_t keyTime = 0;           // ������ʱ����
	uint8_t keyPress;
	keyBoardState_e keyReturn; 

	keyReturn = KEY_RELEASE;                         // ��� ���ذ���ֵ
	keyPress = remoteControlData.dt7Value.mouse.Press_R;   // ��ǰ��ֵ
	switch (keyState){       
		case KEY_STATE_0:{                  // ����״̬0���ж����ް�������
			if(keyPress){               // �а�������
				keyTime = 0;                   // ����ʱ��������
				keyState = KEY_STATE_1;        // Ȼ����� ����״̬1
			}        
		}break;
		case KEY_STATE_1:{                  // ����״̬1�����������ȷ�������Ƿ���Ч���������󴥣���������Ч�Ķ��壺�����������³����趨������ʱ�䡣
			if(keyPress){                     
				keyTime++;                     // һ��5ms
				if(keyTime>=SINGLE_PRESS_TIME){   // ����
					keyState = KEY_STATE_2;      // �������ʱ�䳬�� ����ʱ�䣬���ж�Ϊ���µİ�����Ч��������Ч�������֣��������߳��������� ����״̬2�� �����ж�������������Ч����
				}
			}         
			else keyState = KEY_STATE_0;      // �������ʱ��û�г������ж�Ϊ�󴥣�������Ч������ ����״̬0�������ȴ�����
		}break; 
		case KEY_STATE_2:{                   // ����״̬2���ж�������Ч�����ࣺ�ǵ��������ǳ���
			if(keyPress == 0){                // ��������� �趨�ĳ���ʱ�� ���ͷţ����ж�Ϊ����
				keyReturn = KEY_PRESS_ONCE;      // ���� ��Ч����ֵ������
				keyState = KEY_STATE_0;         // ���� ����״̬0�������ȴ�����
			} 
			else{
				keyTime++;                     
				if(keyTime >= LONG_PRESS_TIME){   // �������ʱ�䳬�� �趨�ĳ���ʱ��
					keyReturn = KEY_PRESS_LONG;            // ���� ��Ч��ֵֵ������
					keyState = KEY_STATE_3;       // ȥ״̬3���ȴ������ͷ�
				}
			}
		}break;
		case KEY_STATE_3:{                   // �ȴ������ͷ�
			if (keyPress == 0){ 
				keyState = KEY_STATE_0;         // �����ͷź󣬽��� ����״̬0 ��������һ�ΰ������ж�
			}
			else	keyReturn = KEY_PRESS_LONG;				
		}break; 
	}
	return keyReturn;                     // ���� ����ֵ
}

void keyFsm(keyBoardState_e *sta, uint8_t key){
  switch (*sta){
    case KEY_RELEASE:{
      if (key)
        *sta = KEY_WAIT_EFFECTIVE;    //��������       
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_WAIT_EFFECTIVE:{					//�ȴ�����������Ч��������
      if (key){	
        *sta = KEY_PRESS_ONCE;        //״̬ת��
			}
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_ONCE:{							//����������Ч
      if (key){ 
        *sta = KEY_PRESS_DOWN;        //״̬ת��
        if (sta == &keyBoardCtrlData.lkSta)
          keyBoardCtrlData.lk_cnt = 0;  //�󳤰���������
        else
          keyBoardCtrlData.rk_cnt = 0; //�ҳ�����������	
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_DOWN:{							//����һֱ����
      if (key){  
        if (sta == &keyBoardCtrlData.lkSta){
          if (keyBoardCtrlData.lk_cnt++ > LONG_PRESS_TIME)//�����㹻ʱ��
            *sta = KEY_PRESS_LONG;                                  //����״̬
        }
        else{
          if (keyBoardCtrlData.rk_cnt++ > LONG_PRESS_TIME)
            *sta = KEY_PRESS_LONG;
        }
      }
      else
        *sta = KEY_RELEASE;
    }break;
    
    case KEY_PRESS_LONG:{							//����״̬��
      if (!key){											//û�а�������
        *sta = KEY_RELEASE;     
      }
    }break;
    
    default:
			break;
  }
}

static void moveDirectionCtrl(uint8_t forward, uint8_t back,uint8_t left, uint8_t right,uint8_t fast,uint8_t slow,int16_t rotate) {
	static float forwardAccele = 0, backAccele = 0;      //������ٵ��ٶ�
	static float rightAccele = 0, leftAccele = 0;
	if(robotMode == MODE_KM){
	  keyBoardCtrlData.chassisSpeedTarget.z = rotate * REAL_MOTOR_SPEED_SCALE * 65;	
		if(forward){
			//�ٶ��𽥵���
			forwardAccele += parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE * 0.75f;
			backAccele = 0;
			forwardAccele = constrainFloat(forwardAccele, 0.0f, parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE);
			//���°����ٶȴ�500��ʼ�𽥵���������shiftֱ������
			keyBoardCtrlData.chassisSpeedTarget.y = MIN_CHASSIS_SPEED * chassisData.speedLimit \
													+ forwardAccele * chassisData.speedLimit \
													+ fast * parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, 0.0f, \
																   parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE);                                
			//������ǰǰ���ٶȣ��ɿ�����ʱ������ٶȿ�ʼ���ٵݼ�
			backAccele = keyBoardCtrlData.chassisSpeedTarget.y; 						
		}
		else if(back){
			//������ڵ����ı���
			forwardAccele = 0;
			backAccele -= parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE * 0.75f;
			//�����ٶ�Ϊǰ���ٶ�һ��
			backAccele = constrainFloat(backAccele, -0.5f * parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE, 0.0f);
			//���°����ٶȴ�500��ʼ�𽥵���������shiftֱ������
			keyBoardCtrlData.chassisSpeedTarget.y = -MIN_CHASSIS_SPEED * chassisData.speedLimit \
													+ backAccele * chassisData.speedLimit \
													- fast * parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, \
																-0.5f*parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE, 0.0f);     
		}
		else{
			//�ɿ�����ʱ������Ҽ������ı���
			forwardAccele = 0;
			backAccele = 0;
			//֮ǰ������ǰ��
			if(keyBoardCtrlData.chassisSpeedTarget.y >= 0) { 		 					
				//�ɿ�����ʱ�ٶȿ��ٵݼ� 
				keyBoardCtrlData.chassisSpeedTarget.y = (keyBoardCtrlData.chassisSpeedTarget.y \
														 - 4 * parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE) * (!fast);
				keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, 0.0, \
																	   parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE);  
			}
			//֮ǰ�����Ǻ��
			if(keyBoardCtrlData.chassisSpeedTarget.y < 0) {
				//�ɿ�����ʱ�ٶȿ��ٵݼ� 
				keyBoardCtrlData.chassisSpeedTarget.y = (keyBoardCtrlData.chassisSpeedTarget.y \
														 + 4 * parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE) * (!fast);
				keyBoardCtrlData.chassisSpeedTarget.y = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.y, \
																-0.5f*parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE, 0.0f);  
			}
		}
		//�����Ҽ�ʱ�����������ı���
		if(right){
			leftAccele = 0;                                  
			//�ٶ��𽥵���			
			rightAccele += parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE * 0.5f;          
			rightAccele = constrainFloat(rightAccele, 0.0f, parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE); 
			//���°����ٶȴ�500��ʼ�𽥵���������shiftֱ������
			keyBoardCtrlData.chassisSpeedTarget.x = MIN_CHASSIS_SPEED * chassisData.speedLimit \
													+ rightAccele * chassisData.speedLimit \
													+ fast * parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, 0.0f, \
																   parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE);
		}
		//�������ʱ����Ҽ������ı���
		else if(left){
			rightAccele = 0;
			//�ٶ��𽥵���
			leftAccele -= parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE * 0.5f;
			leftAccele = constrainFloat(leftAccele, -parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE, 0.0f); 
			//���°����ٶȴ�500��ʼ�𽥵���������shiftֱ������
			keyBoardCtrlData.chassisSpeedTarget.x = -MIN_CHASSIS_SPEED * chassisData.speedLimit \
													+ leftAccele * chassisData.speedLimit \
													- fast * parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE;
			keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, \
																		-parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE,0.0f);                                
		}
		else{
			//�ɿ�����ʱ������Ҽ������ı���
			rightAccele = 0;                                							
			leftAccele = 0;
			//֮ǰ�������Ҽ�
			if(keyBoardCtrlData.chassisSpeedTarget.x >= 0) { 							
				//�ɿ�����ʱ�ٶȿ��ٵݼ� 
				keyBoardCtrlData.chassisSpeedTarget.x = (keyBoardCtrlData.chassisSpeedTarget.x \
														 - 4 * parameter[CHASSIS_KB_ACC] * REAL_MOTOR_SPEED_SCALE) * (!fast);  
				keyBoardCtrlData.chassisSpeedTarget.x = constrainFloat(keyBoardCtrlData.chassisSpeedTarget.x, 0.0, \
																	   parameter[CHASSIS_KB_SPEED] * REAL_MOTOR_SPEED_SCALE);  
			}
			//֮ǰ���������
			if(keyBoardCtrlData.chassisSpeedTarget.x < 0) {  
				//�ɿ�����ʱ�ٶȿ��ٵݼ� 
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
		
			//������̨���ٶ�������Yaw��ս��ٶȵ���
			keyBoardCtrlData.yawSpeedTarget = -yawRefSpd * parameter[GIMBAL_KB_SCALE] * 32;
			//���ٶ������޷�
			keyBoardCtrlData.yawSpeedTarget = keyBoardCtrlData.yawSpeedTarget > 7.3f?  7.3f : \
			(keyBoardCtrlData.yawSpeedTarget < -7.3f? -7.3f : keyBoardCtrlData.yawSpeedTarget);
		}
		else{
			//�սǶ��⻷
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



