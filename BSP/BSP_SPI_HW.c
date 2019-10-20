#include "BSP_SPI_HW.h"

/******************************�ⲿ���ú���************************************/
void BSP_SPI_Init(BSP_SPI_TypeDef* BSP_SPIx);
void BSP_SPIx_SetSpeed(BSP_SPI_TypeDef* BSP_SPIx,u8 SPI_BaudRatePrescaler);
u8	 BSP_SPI_ReadWriteByte(BSP_SPI_TypeDef* BSP_SPIx,u8 TxData);
/*****************************************************************************/


/*
***************************************************
��������BSP_SPI_RCC_Init
���ܣ�����SPI����ʱ��
��ڲ�����	BSP_SPIx��SPI��
����ֵ����
Ӧ�÷�Χ���ڲ�����
��ע��
***************************************************
*/
void BSP_SPI_RCC_Init(SPI_TypeDef* BSP_SPIx){
	if(BSP_SPIx == SPI1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	else if(BSP_SPIx == SPI2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	else if(BSP_SPIx == SPI3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
	else if(BSP_SPIx == SPI4)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI4,ENABLE);
	else if(BSP_SPIx == SPI5)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5,ENABLE);
	else if(BSP_SPIx == SPI6)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI6,ENABLE);
}

/*
***************************************************
��������SPIx_TO_GPIO_AF
���ܣ���SPI�����GPIO_AF
��ڲ�����	SPIx��SPI��
����ֵ��GPIO_AF:���õ�SPIģʽ
Ӧ�÷�Χ���ڲ�����
��ע��
***************************************************
*/
uint8_t SPIx_TO_GPIO_AF(SPI_TypeDef *SPIx){
	uint8_t GPIO_AF;
	if(SPIx == SPI1)			
		GPIO_AF = GPIO_AF_SPI1;
	else if(SPIx == SPI2)	
		GPIO_AF = GPIO_AF_SPI2;
	else if(SPIx == SPI3)	
		GPIO_AF = GPIO_AF_SPI3;
	else if(SPIx == SPI4)	
		GPIO_AF = GPIO_AF_SPI4;
	else if(SPIx == SPI5)	
		GPIO_AF = GPIO_AF_SPI5;
	else if(SPIx == SPI6)	
		GPIO_AF = GPIO_AF_SPI6;
	return GPIO_AF;
}

/*
***************************************************
��������BSP_SPI_Init
���ܣ�Ӳ��SPI���ܳ�ʼ��
��ڲ�����	BSP_SPIx��SPI��
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
void BSP_SPI_Init(BSP_SPI_TypeDef* BSP_SPIx){
	SPI_InitTypeDef  SPI_InitStructure;
	
	/*************��ʼ��SPIʱ��***************/
	BSP_SPI_RCC_Init(BSP_SPIx->SPIx);
	
	/*************��ʼ��SPI����***************/
	BSP_GPIO_Init(BSP_SPIx->SPI_NSS,GPIO_Mode_Out_PP);	//��ʼ��SPIx_NSS����Ϊ�������
	BSP_GPIO_Init(BSP_SPIx->SPI_SCK ,GPIO_Mode_AF_PP);	//��ʼ��SPIx_SCK ����Ϊ��������
	BSP_GPIO_Init(BSP_SPIx->SPI_MISO,GPIO_Mode_AF_PP);	//��ʼ��SPIx_MISO����Ϊ��������
	BSP_GPIO_Init(BSP_SPIx->SPI_MOSI,GPIO_Mode_AF_PP);	//��ʼ��SPIx_MOSI����Ϊ��������
	
	/*************�������Ÿ���SPI����***************/
	GPIO_Pin_TO_PinAFConfig(BSP_SPIx->SPI_SCK ,SPIx_TO_GPIO_AF(BSP_SPIx->SPIx));	//SPIx_SCK ���Ÿ���Ϊ SPIx
	GPIO_Pin_TO_PinAFConfig(BSP_SPIx->SPI_MISO,SPIx_TO_GPIO_AF(BSP_SPIx->SPIx));	//SPIx_MISO���Ÿ���Ϊ SPIx
	GPIO_Pin_TO_PinAFConfig(BSP_SPIx->SPI_MOSI,SPIx_TO_GPIO_AF(BSP_SPIx->SPIx));	//SPIx_MOSI���Ÿ���Ϊ SPIx
	
	/**************��ʼ��SPIxģʽ**************/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;				//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;					//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;				//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;						//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;						//CRCֵ����Ķ���ʽ
	SPI_Init(BSP_SPIx->SPIx, &SPI_InitStructure);				//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	
	/**************ʹ��SPIx����*******************/
	SPI_Cmd(BSP_SPIx->SPIx, ENABLE);
}

/*
***************************************************
��������BSP_SPIx_SetSpeed
���ܣ�SPIx�ٶ����ú���
��ڲ�����	SPIx��SPI��
					SPI_BaudRatePrescaler�����岨����Ԥ��Ƶ��ֵ��
						SPI_BaudRatePrescaler_2��SPI_BaudRatePrescaler_4��SPI_BaudRatePrescaler_8��SPI_BaudRatePrescaler_16��
						SPI_BaudRatePrescaler_32��SPI_BaudRatePrescaler_64��SPI_BaudRatePrescaler_128��SPI_BaudRatePrescaler_256
����ֵ����
Ӧ�÷�Χ���ⲿ����
��ע��	SPI2��SPI3��ʱ��Ƶ��Ϊ168MHz
			SPI1��SPI4��SPI5��SPI6��ʱ��Ƶ��Ϊ84MHz
***************************************************
*/
void BSP_SPIx_SetSpeed(BSP_SPI_TypeDef* BSP_SPIx,u8 SPI_BaudRatePrescaler){
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//�ж���Ч��
	BSP_SPIx->SPIx->CR1&=0XFFC7;								//λ3-5���㣬�������ò�����
	BSP_SPIx->SPIx->CR1|=SPI_BaudRatePrescaler;	//����SPIx�ٶ� 
	SPI_Cmd(BSP_SPIx->SPIx,ENABLE); //ʹ��SPIx
}

/*
***************************************************
��������BSP_SPI_ReadWriteByte
���ܣ�SPIx��дһ���ֽ�
��ڲ�����	SPIx��SPI��
					TxData��Ҫд����ֽ�
����ֵ����ȡ�����ֽ�
Ӧ�÷�Χ���ⲿ����
��ע��	
***************************************************
*/
u8 BSP_SPI_ReadWriteByte(BSP_SPI_TypeDef* BSP_SPIx,u8 TxData){
	u8 retry=0;		
	while((BSP_SPIx->SPIx->SR&1<<1)==0){	//�ȴ��������� 
		retry++;
		if(retry>200)	
			return 0;
	}	
	BSP_SPIx->SPIx->DR=TxData;	 	  			//����һ��byte  
	
	retry=0;
	while((BSP_SPIx->SPIx->SR&1<<0)==0){	//�ȴ�������һ��byte
		retry++;
		if(retry>200)	
			return 0;
	}
 	return BSP_SPIx->SPIx->DR;  
}



/***************����SPI����*******************/
BSP_SPI_TypeDef BSP_SPI1 = {
	.SPIx = SPI1,
	.SPI_NSS = BSP_GPIOA4,
	.SPI_SCK = BSP_GPIOA5,
	.SPI_MISO = BSP_GPIOA6,
	.SPI_MOSI = BSP_GPIOA7
};

BSP_SPI_TypeDef BSP_SPI2 = {
	.SPIx = SPI2,
	.SPI_NSS = BSP_GPIOB12,
	.SPI_SCK = BSP_GPIOB13,
	.SPI_MISO = BSP_GPIOB14,
	.SPI_MOSI = BSP_GPIOB15
};

BSP_SPI_TypeDef BSP_SPI3 = {
	.SPIx = SPI3,
	.SPI_NSS = BSP_GPIOA15,
	.SPI_SCK = BSP_GPIOC10,
	.SPI_MISO = BSP_GPIOC11,
	.SPI_MOSI = BSP_GPIOC12
};





