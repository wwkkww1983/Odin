#include "power.h"
#include "Driver_powerContol.h"
#include "auto_infantry.h"
#include "application.h"
#include "Util.h"
#include "auto_tank.h"

powerstruct_t powerData;
BSP_USART_TypeDef POWER; 

uint16_t adc_value[10][4];

void sendPowerDataInit(void){
	Driver_Power_Init(POWER_USARTX,POWER_USARTX_RX_PIN,POWER_USARTX_TX_PIN,POWER_USART_PreemptionPriority,POWER_USART_SubPriority);
	supervisorData.taskEvent[WIRELESS_TASK] = xTaskCreate(powerDataTransportTask,"DATATRANS",WIRELESS_STACK_SIZE,NULL,WIRELESS_PRIORITY,&wirelessData.xHandleTask);
}

void powerLinkInit(void){
	BSP_GPIO_Init(BSP_GPIOD14,GPIO_Mode_Out_PP);
	BSP_GPIO_Init(BSP_GPIOD15,GPIO_Mode_Out_PP);
	LINK_CHASSIS = CHASSIS_ON;
	LINK_CAP = CAP_OFF;
	powerData.linkCapFlag = false;
}

void adcInit_cap(void){
	GPIO_InitTypeDef  		GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStrcture;
	ADC_InitTypeDef 			ADC_InitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA2, ENABLE);//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;//DMA�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)adc_value;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//�����裩��ַ��DMA������
	DMA_InitStructure.DMA_BufferSize = 4 * 10;//
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//DMA������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//DMA�洢����ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//DMA�洢����ֽ�
	DMA_InitStructure.DMA_MemoryDataSize =DMA_PeripheralDataSize_HalfWord;//���������ݳ���16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//ѭ��ģʽ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//���ȼ� ���ߣ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	
	DMA_Init(DMA2_Stream0,&DMA_InitStructure);//��ʼ��DMA Stream0
	DMA_Cmd(DMA2_Stream0, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0|GPIO_Pin_1 |GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//����������
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	ADC_CommonInitStrcture.ADC_Mode = ADC_Mode_Independent;//����ģʽ
	ADC_CommonInitStrcture.ADC_DMAAccessMode = ADC_DMAAccessMode_1;// DMAģʽ1
	ADC_CommonInitStrcture.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��84/4M��
	ADC_CommonInitStrcture.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;//��������ʱ����ʱ20��ʱ��
	
	ADC_CommonInit(&ADC_CommonInitStrcture);
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//  �ֱ���12λ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ɨ��ģʽ	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
	ADC_InitStructure.ADC_NbrOfConversion = 4;//�ĸ�ͨ��

	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 0, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_144Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_144Cycles);
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);	
	ADC_DMACmd(ADC1,ENABLE);
	
	ADC_Cmd(ADC1, ENABLE);//����ADת����
	ADC_SoftwareStartConv(ADC1);//�������ADC
}

void GetAdcAverage(u8 channel,u8 times,float *volPin){
	static uint16_t sum = 0;
	static float vol;
	u8 i;
		i = channel;
		for(u8 j = 0;j < 10; j++){
			sum += adc_value[j][i];
		}
		vol =(float) sum/10.0f;
		*volPin =(float)((vol* 3.3f) / 4095.0f);	
		powerData.volPIn = *volPin;
		vol = 0.0f;
		sum = 0.0f;
}

void powerDataReceive(u8 *array){
	uint8_t cnt = 0;
	FormatTrans dataTrans;
	for(cnt = 0; cnt < 10; cnt++){
		if(array[cnt] != 0xAD){
		}
		else{
			dataTrans.u8_temp[0] = array[++cnt];
			dataTrans.u8_temp[1] = array[++cnt];
			dataTrans.u8_temp[2] = array[++cnt];
			dataTrans.u8_temp[3] = array[++cnt];
			powerData.capVol = dataTrans.float_temp;
			if(powerData.capVol < 1e-4f || powerData.capVol > 1e+4f){
				LINK_CHASSIS = CHASSIS_ON;    //�л���ֱ������
				LINK_CAP = CAP_OFF;
				powerData.capVol = 0.0f;
				powerData.CapError = true;
				powerData.linkCapFlag = false;
			}
			else{
				powerData.CapError = false;
			}
		}
	}
}

void powerDataUpdata(void){
	u8 index_ptr = 0;
	Array_UART7_TX[index_ptr++] = 0xAD;
	Array_UART7_TX[index_ptr++] = 0x2C;
	Array_UART7_TX[index_ptr++] = powerData.SwitchPolicy;
	POWER.USARTx = UART7;
//	BSP_UART7_DMA_SendData(Array_UART7_TX,index_ptr);
	BSP_USART_SendData(&POWER,Array_UART7_TX,index_ptr);
	USART_ClearFlag(UART7,USART_FLAG_TC);		
}

/*****�ڶ����������ݣ��������߼�,ֱ�����̣������л����ӳ�������******
******�¼ӵ͹����Զ����*****************************************/
float lastCapVol[21];
uint8_t  num = 0;
void linkAimUpdata(){
static bool autoCharge,manualCharge;
	if(ROBOT == INFANTRY_ID || ROBOT == TANK_ID){
	#ifdef autocharge
		//�Զ�����߼�
		static uint32_t startTime;
		if(!infantryAutoData.rotateFlag && !infantryAutoData.aviodFlag){
			switch(powerData.step){
				case 0://��⹦��С��55Wʱ�Զ����
								if(judgeData.extPowerHeatData.chassis_power < 55){
									startTime = powerData.loops;
									powerData.step +=1;
								}
								else
									//���Զ����
									autoCharge = false;
								break;
				case 1: if((powerData.loops - startTime) > 100){
									if(judgeData.extPowerHeatData.chassis_power < 55){
										//�Զ����
										autoCharge = true;
										powerData.step +=1;
									}
									else{
										//���Զ����
										autoCharge = false;
										//����һ�μ��
										powerData.step = 0;
									}
								}break;
				case 2: //��ѹ�½��Ͽ����ݣ�������һ�μ���Զ����
								if((powerData.standardVol - powerData.capVol) > 0.3f){
									//���Զ����
									autoCharge = false;
									//��ʼ��һ�μ���Զ����
									powerData.step = 0;
								}
								break;
			}
			lastCapVol[num++] = powerData.capVol;
			if(num == 20){
				num = 0;
				lastCapVol[20] = (lastCapVol[0]+lastCapVol[1]+lastCapVol[2]+lastCapVol[3]+lastCapVol[4]+lastCapVol[5]+lastCapVol[6] 
													+lastCapVol[7]+lastCapVol[8]+lastCapVol[9]+lastCapVol[10]+lastCapVol[11]+lastCapVol[12] 
													+lastCapVol[13]+lastCapVol[14]+lastCapVol[15]+lastCapVol[16]  
													+lastCapVol[17]+lastCapVol[18]+lastCapVol[19]) / 20.0f;
			}
			powerData.standardVol = lastCapVol[20];
		}
		else{
			//���Զ���磬��ʼ�����һ���Զ����
			autoCharge = false;
			powerData.step = 0;
		}
	#else
		autoCharge = false;
	#endif
		//�ֶ�������߼�
		if(judgeData.extPowerHeatData.chassis_power < 55){
			if((!PRESS_X)&&(RC_ROTATE < 10))
				manualCharge = false;
			else
				//�ֶ����
				manualCharge = true;
		}
		else
			manualCharge = false;
		
		//����X��磬���ֳ�磬�����̹����Զ����
		if(!manualCharge && !autoCharge){
			if(powerData.capVol < 17.0f)							//�͵�ѹ��Ҫǿ���л�
				infantryAutoData.forceSwitch.vol_limit = true;     //ǿ���л���ֱ������
			else if(powerData.capVol > 18.0f)
			  infantryAutoData.forceSwitch.vol_limit = false;
			
			if(!infantryAutoData.forceSwitch.vol_limit){    //û��ǿ���л���ֱ������
				if(openCap || (RC_ROTATE < -100)){		   //shift �����л������ӳ�������
					powerData.linkCapFlag = true;
					powerData.chargeFlag = false;
					powerData.rotateFast = true;
					LINK_CAP = CAP_ON;   
					LINK_CHASSIS = CHASSIS_OFF;					
				}
				else if(fabs(IMU_CHASE_RATEZ) > 2.0f){	
					//������Ť����С������رյ���
					if(infantryAutoData.rotateFlag || infantryAutoData.aviodFlag){  //�л���ֱ������
						powerData.linkCapFlag = false;
						LINK_CHASSIS = CHASSIS_ON;   
						LINK_CAP = CAP_OFF;
					}
					//����ʱ�Զ�������
					else{
						powerData.linkCapFlag = true;
						LINK_CAP = CAP_ON;
						LINK_CHASSIS = CHASSIS_OFF;
					}
					powerData.rotateFast = false;
				}
				else{
					powerData.linkCapFlag = false;
					powerData.rotateFast = false;
					LINK_CHASSIS = CHASSIS_ON;    //�л���ֱ������
					LINK_CAP = CAP_OFF;
				}
			}
			//�͵�ѹ��������ֱ������ģʽ
			else{
				powerData.linkCapFlag = false;
				LINK_CHASSIS = CHASSIS_ON;    //�л���ֱ������
				LINK_CAP = CAP_OFF;
			}
		}
		//���
		else{
			powerData.chargeFlag = true;
			powerData.linkCapFlag = true;
			LINK_CAP = CAP_ON;
			LINK_CHASSIS = CHASSIS_OFF;		
		}			
	}
}

void powerDataTransportTask(void *Parameters){
	while(1){
		vTaskDelay(WIRELESS_PERIOD);
		if(!powerData.initFlag){
			//��Դ·���л��˿ڳ�ʼ��
			powerLinkInit();
			if(ROBOT == TANK_ID){
				//Ӣ��������adc�ɼ���ʼ��
//				adcInit_cap();      //������ADͨ���Ͳ���ϵͳ����ӳ���ͻ
			}				
			digitalHi(&powerData.initFlag);
		}
		if(ROBOT == TANK_ID){
			//�ɵ��ݵ�ѹ
			GetAdcAverage(capChannel,average_times_cap,&powerData.volCap);
			powerData.capVol = (float)powerData.volCap*20400.0f/2400.0f;
			if(powerData.capVol < 1e-4f || powerData.capVol > 1e+4f){
				LINK_CHASSIS = CHASSIS_ON;    //�л���ֱ������
				LINK_CAP = CAP_OFF;
				powerData.capVol = 0.0f;
				powerData.CapError = true;
				powerData.linkCapFlag = false;
			}
			else{
				powerData.CapError = false;
			}
		}
		if(powerData.CapError)
			continue;
		linkAimUpdata();  
		//�ó����������ѹ
		if(powerData.capVol >= powerData.capVolMax){
			powerData.capVolMax = powerData.capVol;
			//��������ʱ��ѹ�����������ܳ��������ѹ
			if(powerData.capVolMax > 26.8f)
				//��Ϊ��Ч����
				powerData.capVolMax = 0.0f;
			//��������ʱ������������������ѹ
			else if(powerData.capVolMax < 24.0f)
				powerData.capVolMax = 24.0f;
		}
		digitalIncreasing(&powerData.loops);     
	}
}
