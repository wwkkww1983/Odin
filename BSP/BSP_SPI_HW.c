#include "BSP_SPI_HW.h"

/******************************外部调用函数************************************/
void BSP_SPI_Init(BSP_SPI_TypeDef* BSP_SPIx);
void BSP_SPIx_SetSpeed(BSP_SPI_TypeDef* BSP_SPIx,u8 SPI_BaudRatePrescaler);
u8	 BSP_SPI_ReadWriteByte(BSP_SPI_TypeDef* BSP_SPIx,u8 TxData);
/*****************************************************************************/


/*
***************************************************
函数名：BSP_SPI_RCC_Init
功能：配置SPI外设时钟
入口参数：	BSP_SPIx：SPI号
返回值：无
应用范围：内部调用
备注：
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
函数名：SPIx_TO_GPIO_AF
功能：从SPI号输出GPIO_AF
入口参数：	SPIx：SPI号
返回值：GPIO_AF:复用的SPI模式
应用范围：内部调用
备注：
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
函数名：BSP_SPI_Init
功能：硬件SPI功能初始化
入口参数：	BSP_SPIx：SPI号
返回值：无
应用范围：外部调用
备注：
***************************************************
*/
void BSP_SPI_Init(BSP_SPI_TypeDef* BSP_SPIx){
	SPI_InitTypeDef  SPI_InitStructure;
	
	/*************初始化SPI时钟***************/
	BSP_SPI_RCC_Init(BSP_SPIx->SPIx);
	
	/*************初始化SPI引脚***************/
	BSP_GPIO_Init(BSP_SPIx->SPI_NSS,GPIO_Mode_Out_PP);	//初始化SPIx_NSS引脚为推挽输出
	BSP_GPIO_Init(BSP_SPIx->SPI_SCK ,GPIO_Mode_AF_PP);	//初始化SPIx_SCK 引脚为复用推挽
	BSP_GPIO_Init(BSP_SPIx->SPI_MISO,GPIO_Mode_AF_PP);	//初始化SPIx_MISO引脚为复用推挽
	BSP_GPIO_Init(BSP_SPIx->SPI_MOSI,GPIO_Mode_AF_PP);	//初始化SPIx_MOSI引脚为复用推挽
	
	/*************设置引脚复用SPI外设***************/
	GPIO_Pin_TO_PinAFConfig(BSP_SPIx->SPI_SCK ,SPIx_TO_GPIO_AF(BSP_SPIx->SPIx));	//SPIx_SCK 引脚复用为 SPIx
	GPIO_Pin_TO_PinAFConfig(BSP_SPIx->SPI_MISO,SPIx_TO_GPIO_AF(BSP_SPIx->SPIx));	//SPIx_MISO引脚复用为 SPIx
	GPIO_Pin_TO_PinAFConfig(BSP_SPIx->SPI_MOSI,SPIx_TO_GPIO_AF(BSP_SPIx->SPIx));	//SPIx_MOSI引脚复用为 SPIx
	
	/**************初始化SPIx模式**************/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;				//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;					//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;				//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;						//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;						//CRC值计算的多项式
	SPI_Init(BSP_SPIx->SPIx, &SPI_InitStructure);				//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	
	/**************使能SPIx外设*******************/
	SPI_Cmd(BSP_SPIx->SPIx, ENABLE);
}

/*
***************************************************
函数名：BSP_SPIx_SetSpeed
功能：SPIx速度设置函数
入口参数：	SPIx：SPI号
					SPI_BaudRatePrescaler：定义波特率预分频的值：
						SPI_BaudRatePrescaler_2，SPI_BaudRatePrescaler_4，SPI_BaudRatePrescaler_8，SPI_BaudRatePrescaler_16，
						SPI_BaudRatePrescaler_32，SPI_BaudRatePrescaler_64，SPI_BaudRatePrescaler_128，SPI_BaudRatePrescaler_256
返回值：无
应用范围：外部调用
备注：	SPI2，SPI3的时钟频率为168MHz
			SPI1，SPI4，SPI5，SPI6的时钟频率为84MHz
***************************************************
*/
void BSP_SPIx_SetSpeed(BSP_SPI_TypeDef* BSP_SPIx,u8 SPI_BaudRatePrescaler){
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
	BSP_SPIx->SPIx->CR1&=0XFFC7;								//位3-5清零，用来设置波特率
	BSP_SPIx->SPIx->CR1|=SPI_BaudRatePrescaler;	//设置SPIx速度 
	SPI_Cmd(BSP_SPIx->SPIx,ENABLE); //使能SPIx
}

/*
***************************************************
函数名：BSP_SPI_ReadWriteByte
功能：SPIx读写一个字节
入口参数：	SPIx：SPI号
					TxData：要写入的字节
返回值：读取到的字节
应用范围：外部调用
备注：	
***************************************************
*/
u8 BSP_SPI_ReadWriteByte(BSP_SPI_TypeDef* BSP_SPIx,u8 TxData){
	u8 retry=0;		
	while((BSP_SPIx->SPIx->SR&1<<1)==0){	//等待发送区空 
		retry++;
		if(retry>200)	
			return 0;
	}	
	BSP_SPIx->SPIx->DR=TxData;	 	  			//发送一个byte  
	
	retry=0;
	while((BSP_SPIx->SPIx->SR&1<<0)==0){	//等待接收完一个byte
		retry++;
		if(retry>200)	
			return 0;
	}
 	return BSP_SPIx->SPIx->DR;  
}



/***************常用SPI配置*******************/
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





