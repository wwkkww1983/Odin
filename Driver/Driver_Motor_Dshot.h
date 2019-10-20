#ifndef __DRIVER_MOTOR_DSHOT_H
#define __DRIVER_MOTOR_DSHOT_H

#include "stdbool.h"
#include "stm32f4xx.h"
#include "Driver_Motor.h"
#include "stm32f4xx_dma.h"

typedef enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1,
    DSHOT_CMD_BEEP2,
    DSHOT_CMD_BEEP3,
    DSHOT_CMD_BEEP4,
    DSHOT_CMD_BEEP5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_MAX = 47
} dshotCommands_e;

typedef enum {
	PWM_TYPE_DSHOT150 = 0,
	PWM_TYPE_DSHOT300,
	PWM_TYPE_DSHOT600,
	PWM_TYPE_DSHOT1200,
	PWM_TYPE_PROSHOT1000,
	PWM_TYPE_MAX
} motorPwmProtocolTypes_e;

#define DSHOT_DISARM_COMMAND      0
#define DSHOT_MIN_THROTTLE       48
#define DSHOT_MAX_THROTTLE     2047
#define DSHOT_3D_DEADBAND_LOW  1047
#define DSHOT_3D_DEADBAND_HIGH 1048

#define MHZ_TO_HZ(x) ((x) * 1000000)

#define MOTOR_PROSHOT1000_HZ         MHZ_TO_HZ(24)
#define MOTOR_DSHOT1200_HZ    MHZ_TO_HZ(24)
#define MOTOR_DSHOT600_HZ     MHZ_TO_HZ(12)
#define MOTOR_DSHOT300_HZ     MHZ_TO_HZ(6)
#define MOTOR_DSHOT150_HZ     MHZ_TO_HZ(3)

#define MOTOR_BIT_0           7
#define MOTOR_BIT_1           14
#define MOTOR_BITLENGTH       19

#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */
#define PROSHOT_DMA_BUFFER_SIZE 6  /* resolution + frame reset (2us) */

#define DSHOT_MOTOR_NUM 4

enum
{
	RIGHT_FRONT_TOP = 0,
	LEFT_FRONT_TOP,
	LEFT_REAR_TOP,
	RIGHT_REAR_TOP,
	RIGHT_FRONT_BOTTOM,
	LEFT_FRONT_BOTTOM,
	LEFT_REAR_BOTTOM,
	RIGHT_REAR_BOTTOM,
};

typedef const struct
{
	TIM_TypeDef * tim;
	uint8_t timChannels;
	BSP_GPIOSource_TypeDef _io;
}BSP_Motor_TypeDef;

typedef struct motorDma_s {
	uint16_t dmaBuffer[DSHOT_DMA_BUFFER_SIZE];
	uint16_t value;
	volatile bool requestTelemetry;
} motorDma_t;

void pwmWriteDshot(uint8_t index, float value);
void dshotWriteMotors(float *motor);
void pwmDshotDmaIrqnConfig(uint8_t irqN,uint8_t PreemptionPriority,uint8_t SubPriority);
void dshotMotorConfig(motorPwmProtocolTypes_e typeMode);

extern motorDma_t motorDmaBuffer[MOTORS_NUM];
#endif
