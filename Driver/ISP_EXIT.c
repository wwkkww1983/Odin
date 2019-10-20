#include "board.h"
#include "Driver_Coder.h"

void EXTI0_IRQHandler(void){   
	if(EXTI_GetITStatus(EXTI_Line0) != RESET){    
		EXTI_ClearFlag(EXTI_Line0);          
		EXTI_ClearITPendingBit(EXTI_Line0);
		
		/*********以下是自定义部分**********/
	}
}

void EXTI1_IRQHandler(void){   
	if(EXTI_GetITStatus(EXTI_Line1) != RESET){    
		EXTI_ClearFlag(EXTI_Line1);          
		EXTI_ClearITPendingBit(EXTI_Line1);
		
		/*********以下是自定义部分**********/
	}
}

void EXTI2_IRQHandler(void){   
	if(EXTI_GetITStatus(EXTI_Line2) != RESET){    
		EXTI_ClearFlag(EXTI_Line2);          
		EXTI_ClearITPendingBit(EXTI_Line2);
		
		/*********以下是自定义部分**********/
	}
}

void EXTI3_IRQHandler(void){   
	if(EXTI_GetITStatus(EXTI_Line3) != RESET){    
		EXTI_ClearFlag(EXTI_Line3);          
		EXTI_ClearITPendingBit(EXTI_Line3);
		
		/*********以下是自定义部分**********/
		Read_MPU6500();
	}
}

void EXTI4_IRQHandler(void){   
	if(EXTI_GetITStatus(EXTI_Line4) != RESET){    
		EXTI_ClearFlag(EXTI_Line4);          
		EXTI_ClearITPendingBit(EXTI_Line4);
		
		/*********以下是自定义部分**********/
	}
}

void EXTI9_5_IRQHandler(void) {
	//根据实际使用修改
	if(EXTI_GetITStatus(EXTI_Line8) != RESET){
		EXTI_ClearFlag(EXTI_Line8); 
    EXTI_ClearITPendingBit(EXTI_Line8);
		
		/*********以下是自定义部分**********/
  }
}

void EXTI15_10_IRQHandler(void) {
	//根据实际使用修改
	if(EXTI_GetITStatus(EXTI_Line14) != RESET){
		EXTI_ClearFlag(EXTI_Line14); 
    EXTI_ClearITPendingBit(EXTI_Line14);
		    
		    coderData.coderBack++;
				if(chassisData.landingSpeedy > 0)
				  coderData.xLocation++;  //编码器（后）脉冲计数
				else if(chassisData.landingSpeedy < 0)
					coderData.xLocation--;
					
  }
	if(EXTI_GetITStatus(EXTI_Line13) != RESET){
		EXTI_ClearFlag(EXTI_Line13); 
    EXTI_ClearITPendingBit(EXTI_Line13);
		
				coderData.coderAhead++;
  }
}

