/*
					This file is part of DaggerUAV.
					
！！！！				//////		//		   //////		 //////		 ///////	 //////		！！！！！！！！！！！！！！
！！！！			 //	  //   ////	   //    		 //    			//				//	 //		！！！！！！！！！！！！
！！！！			//	  //  // //   //  ////  //  ////	 ///////	 //////			！！！！！！！！！！
！！！！		 //		//	 //////	  //   //   //   //   //        //		//		！！！！！！！！
！！！！		//////    //	 //	   //////	   //////  ///////	 //			//		！！！！！！
			
					Date:	Dec 19, 2017
					Author: AGKODY
					Filename: Driver_PitchServo.c
*/

#include "BSP.h"
#include "Util.h"
#include "Driver_PitchServo.h"

void pitchServoAdjust(uint16_t angle)
{
	SERVO_MOTOR_NO1 = constrainInt(angle,SERVO_ANGLE_MIN,SERVO_ANGLE_MAX);
	SERVO_MOTOR_NO2 = constrainInt(angle,SERVO_ANGLE_MIN,SERVO_ANGLE_MAX);
}

void pitchServoConfig()
{
	BSP_TIM_PWM_Init(TIM4,SERVO_LENGTH,SERVO_PRESCALER,NULL,NULL,BSP_GPIOD14,BSP_GPIOD15);	
}
