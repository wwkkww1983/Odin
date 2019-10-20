/**
  ******************************************************************************
  * @file    dual_core_demo.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    09-November-2015
  * @brief   This file is the header file for dual core demo
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USR_DEMO_H__
#define __USR_DEMO_H__


/* Includes ------------------------------------------------------------------*/
#include "usb_conf.h"
#include <stdio.h>

/** @addtogroup USBH_USER
* @{
*/

/** @addtogroup USBH_DUAL_CORE_DEMO
* @{
*/
  
/** @defgroup USBH_DUAL_CORE_DEMO
  * @brief This file is the header file for user action
  * @{
  */ 


/** @defgroup USBH_DUAL_CORE_DEMO_Exported_Variables
  * @{
  */ 
#define LINE_01                           (LCD_PIXEL_HEIGHT - 36)
#define LINE_02                           (LCD_PIXEL_HEIGHT - 24)
#define LINE_03                           (LCD_PIXEL_HEIGHT - 12) 


#define DEMO_LOCK()                       demo.lock = 1;
#define DEMO_UNLOCK()                     demo.lock = 0;
#define DEMO_IS_LOCKED()                  (demo.lock == 1)


typedef enum {
  DEMO_IDLE   = 0,
  DEMO_WAIT,  
  DEMO_HID,
  DEMO_MSC,
}Demo_State;

typedef enum {
  DEMO_MSC_IDLE   = 0,
  DEMO_MSC_WAIT,  
}Demo_MSC_State;

typedef enum {
  DEMO_HID_IDLE   = 0,
  DEMO_HID_WAIT,    
}Demo_HID_State;

typedef struct _DemoStateMachine
{
  
  __IO Demo_State           state;
  __IO Demo_MSC_State       msc_state;
  __IO Demo_HID_State       hid_state;
  __IO uint8_t              select;
  __IO uint8_t              lock;
  
}DEMO_StateMachine;


extern uint8_t USBFS_EnumDone;
/**
  * @}
  */ 


/** @defgroup USBH_DUAL_CORE_DEMO_Exported_FunctionsPrototype
  * @{
  */ 

void Demo_Init (void);
void Demo_Process (void);
void Demo_ProbeKey (JOYState_TypeDef state);
#endif /* __USR_DEMO_H__ */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
