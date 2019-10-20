#ifndef __PNEUMATIC_H
#define __PNEUMATIC_H

#include "BSP.h"
#include "Util.h"

#define P_HERO_FIRE			pneumaticData.send4[0]							//������ŷ�
#define P_HERO_LOAD			pneumaticData.send4[1]							//��������
#define P_HERO_42_LID		pneumaticData.send4[2]							//�󵯲ָ�
#define P_HERO_17_LID		pneumaticData.send4[3]							//С���ָ�
#define P_HERO_TV				pneumaticData.send4[4]							//����Ӱ��
#define P_HERO_CCD			pneumaticData.send4[5]							//��������ͷ����

#define ID_PNEUMATIC    0x702																//�����巴��

typedef struct 
{
	uint8_t penumaticFlag;
	uint8_t send1[6];
	uint8_t send2[6];
	uint8_t send3[6];
	uint8_t send4[6];
	uint8_t read1[6];
}pneumatic_state_data_t;


void pneumaticInit(void);
void pneumatic_can1_sentData(u32 ID_CAN,uint8_t *pneumatic_state);
void pneumatic_can2_sentData(u32 ID_CAN,uint8_t *pneumatic_state);
void pneumatic_readData(CanRxMsg *can_rx_msg);
extern pneumatic_state_data_t pneumaticData;


#endif


