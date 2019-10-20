#include "tf_card_parameter.h"
#include "config.h"

uint8_t tfFATFS_Init(void);
uint8_t tfWriteOneParameter(uint8_t id, const uint8_t* strPara, float parameter);
uint8_t tfReadOneParameter(uint8_t id, const uint8_t* strPara, float* parameter);
uint8_t tfOverwrite(uint8_t id, const char ** strPara, float* parameter, uint16_t amount);
uint8_t tfOverread(uint8_t id, const char ** strPara, float* parameter, uint16_t amount);

FATFS FATFS_TF; //文件系统目标

/*
***************************************************
函数名：tfFATFS_Init
功能：TF卡与FATFS初始化
入口参数：无
返回值：初始化结果
				0x00：正常
				0x10：(STA_NOINIT<<4)，TF卡初始化失败
				其余：请看FRESULT枚举定义
应用范围：外部调用
备注：
***************************************************
*/
uint8_t tfFATFS_Init(void){
	uint8_t res;
	res = disk_initialize(0);	//初始化TF卡磁盘，获取状态
	if(res){
		return res<<4;					//为防止与下面挂载函数反馈冲突将其左移4位
	}
	res = f_mount(&FATFS_TF,"0:",1);	//立即挂载SD卡
	return res;
}

uint8_t tFCardConfig(void){
	BSP_GPIO_Init(TFCARD_INT,GPIO_Mode_IPU);
	if(TFCARD_INSERT_IO == TFCARD_INSERT){						//开机如果检测到就直接初始化一次
		parameterRunData.TFError = tfFATFS_Init();	
	}
	return parameterRunData.TFError;
}

/*
***************************************************
函数名：f_readParaLine
功能：读取存放参数的一行
入口参数：fp：文件指针
				strPara：参数字符串
				strLine：输出参数行字符串
返回值：读取结果，0：成功，1：失败
应用范围：内部调用
备注：
***************************************************
*/
uint8_t f_readParaLine(FIL* fp, const uint8_t* strPara, uint8_t* strLine){
	uint8_t arrayTemp[64];	//暂存每一行字符串
	uint8_t res = 1;
	TCHAR* pr_res = NULL;
	
	f_lseek(fp,0);	//移动文件对象指针到最开始
	
	do{
		pr_res = f_gets ((TCHAR *)arrayTemp, 64, fp);	//读取一行字符串(以"\r\n"结尾)
		if(strstr((const char *)arrayTemp,(const char *)strPara) != NULL)
		{
			//判断参数字符串后面是否接着'='，防止出现行字符串包含参数字符串
			if(arrayTemp[strlen((const char *)strPara)] == '=')
			{
				res = 0;
				break;
			}
		}
	}while(pr_res != NULL);	//判断是否读完文件
	
	if(res == 0)
	{
		strcpy((char *)strLine,(const char *)arrayTemp);
	}
	return res;
}

/*
***************************************************
函数名：tfOpenFileID
功能：打开ID对应文件
入口参数：fp：文件指针
					id：ID号
				mode：文件打开模式
返回值：请看枚举FRESULT
应用范围：外部调用
备注：
***************************************************
*/
FRESULT tfOpenFileID(FIL* fp, uint8_t id, BYTE mode){
	FRESULT res;
	char pathFileID[16];	//暂存ID对应文件的路径
	//获取ID对应的文件路径
	sprintf(pathFileID, "0:/RM/ID%d.txt", (int)id);
	res = f_open(fp, pathFileID, mode);
	if(res) 	
		f_close(fp);	//关闭文件
	return res;
}

/*
***************************************************
函数名：tfOpenMotorFileID
功能：打开ID对应的电机配置文件
入口参数：fp：文件指针
					id：ID号
				mode：文件打开模式
返回值：请看枚举FRESULT
应用范围：外部调用
备注：
***************************************************
*/
FRESULT tfOpenMotorFileID(FIL* fp, uint8_t id, BYTE mode){    /*************************/
	FRESULT res;
	char pathFileID[16];	//暂存ID对应文件的路径
	//获取ID对应的文件路径
	sprintf(pathFileID, "0:/RM/MOTOR%d.txt", (int)id);         /**********测试************/
	res = f_open(fp, pathFileID, mode);
	if(res) 	
		f_close(fp);	//关闭文件
	return res;
}                                                            /*************************/

/*
***************************************************
函数名：tfReadOneParameter
功能：读取一个指定参数的值
入口参数：id：ID号
				strPara：参数字符串
				parameter：输出参数
返回值：初始化结果
				0x00：正常
				0x10：(STA_NOINIT<<4)，TF卡初始化失败
				0x20：读取参数行失败
				其余：请看FRESULT枚举定义
应用范围：外部调用
备注：
***************************************************
*/
uint8_t tfReadOneParameter(uint8_t id, const uint8_t* strPara, float* parameter){
	FIL fpID;
	uint8_t strLine[64];	//存放txt文件相关参数的行字符串
	char strFormat[64];	//格式字符串，用于sscanf使用
	uint8_t res = 0;			//结果
	
	//打开已存在的ID文件以及以读取的方式打开
	res = tfOpenFileID(&fpID, id, FA_OPEN_EXISTING | FA_READ);
	//如果打开失败则退出
	if(res)		return res;
	
	//根据 参数字符串 获取 格式字符串 
	strcpy(strFormat,(const char*)strPara);
	strcat(strFormat,"=%f");
	
	//获取存放相关参数的行
	res = f_readParaLine(&fpID, strPara, strLine);
	//读取失败退出
	if(res == 1)	
		res = 0x20;
	else	//格式化读取参数
		sscanf((const char*)strLine, strFormat, parameter);

	f_close(&fpID);	//关闭文件
	return res;
}

/*
***************************************************
函数名：tfWriteOneParameter
功能：写一个指定参数的值
入口参数：id：ID号
				strPara：参数字符串
				parameter：输出参数
返回值：初始化结果
				0x00：正常
				其余：请看FRESULT枚举定义
应用范围：外部调用
备注：如果不存在对应的参数字符串则在文件后面加入新的参数
			如果不存在对应的ID号的文件则新建一个文件
***************************************************
*/
uint8_t tfWriteOneParameter(uint8_t id, const uint8_t* strPara, float parameter){
	FIL fpID, fp_temp;		//ID文件与暂存文件
	uint8_t strLine[64];	//暂存每一行参数字符串
	uint8_t res = 1;			//结果
	TCHAR* pr_res = NULL;
	
	//打开ID文件以及以读写的方式打开，如不存在则新建一个
	res = tfOpenFileID(&fpID, id, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	//如果打开失败则退出
	if(res)		return res;
	
	//创建暂存文件ID_t.txt
	f_open(&fp_temp, "0:/RM/ID_t.txt", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	//文件指针回到最初
	f_lseek(&fpID,0);
	f_lseek(&fp_temp,0);
	res = 1;
	do{
		pr_res = f_gets ((TCHAR *)strLine, 64, &fpID);	//读取一行字符串(以"\r\n"结尾)
		if(res){
			if(strstr((const char *)strLine,(const char *)strPara) != NULL){	
				//判断当前行字符串包含参数字符串的后面是不是接着'='
				if(strLine[strlen((const char *)strPara)] == '='){
					res = 0;
					sprintf((char *)strLine, "%s=%g\r\n", strPara, parameter);
				}
			}
		}
		f_puts((const char *)strLine, &fp_temp);	//向暂存文件写入其余函数
	}while(pr_res != NULL);
	
	if(res){
		sprintf((char *)strLine, "%s=%g\r\n", strPara, parameter);
		f_puts((const char *)strLine, &fp_temp);	//向暂存文件写入其余函数
		res = 0;
	}
	
	sprintf((char *)strLine, "0:/RM/ID%d.txt", (int)id);
	f_close(&fpID);
	f_close(&fp_temp);
	f_unlink((const char *)strLine);
	f_rename("0:/RM/ID_t.txt", (const char *)strLine);
	return 0;
}

/*
***************************************************
函数名：tfOverwrite
功能：覆写参数
入口参数：id：ID号
				strPara：参数字符串指针
				parameter：输出参数
				amount：数量
返回值：初始化结果
				0x00：正常
				0x10：(STA_NOINIT<<4)，TF卡初始化失败
				其余：请看FRESULT枚举定义
应用范围：外部调用
备注：如果不存在对应的ID号的文件则新建一个文件
***************************************************
*/
uint8_t tfOverwrite(uint8_t id, const char ** strPara, float* parameter, uint16_t amount){
	FIL fpID;		//ID文件
	char strLine[64];			//暂存每一行参数字符串
	uint16_t index = 0;
	uint8_t res;
	//创建ID文件以及以读写的方式打开，如存在则覆盖
	res = tfOpenFileID(&fpID, id, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	//如果打开失败则退出
	if(res)		return res;
	f_lseek(&fpID,0);
	while(index < amount){
		sprintf(strLine, "%s=%g\r\n", strPara[index], parameter[index]);
		f_puts(strLine, &fpID);	//向暂存文件写入其余函数
		index ++;
	}
	f_close(&fpID);
	return 0;
}

/*
***************************************************
函数名：tfOverread
功能：通读参数
入口参数：id：ID号
				strPara：参数字符串指针
				parameter：输出参数
				amount：数量
返回值：初始化结果
				0x00：正常
				0x10：(STA_NOINIT<<4)，TF卡初始化失败
				其余：请看FRESULT枚举定义
应用范围：外部调用
备注：
***************************************************
*/
uint8_t tfOverread(uint8_t id, const char ** strCMD, float* parameter, uint16_t amount){
	FIL fpID;		//ID文件
	char strLine[64];			//暂存每一行参数字符串
	char strFormat[64];	//暂存每一行参数字符串
	uint16_t index = 0;
	uint8_t res;
	//打开已存在的ID文件以及以读取的方式打开
	res = tfOpenFileID(&fpID, id, FA_OPEN_EXISTING | FA_READ);
	//如果打开失败则退出
	if(res)		return res;
	
	f_lseek(&fpID,0);
	
	while(index < amount){
		f_gets(strLine, 64, &fpID);
		strcpy(strFormat,(const char*)strCMD[index]);
		strcat(strFormat,"=%f");
		sscanf(strLine,strFormat,&parameter[index]);
		index ++;
	}
	f_close(&fpID);
	return 0;
}
/*
***************************************************
函数名：tfOverread
功能：通读电机配置参数
入口参数：id：ID号
				strPara：参数字符串指针
				parameter：输出参数
				amount：数量
返回值：初始化结果
				0x00：正常
				0x10：(STA_NOINIT<<4)，TF卡初始化失败
				其余：请看FRESULT枚举定义
应用范围：外部调用
备注：
***************************************************
*/
uint8_t tfMotorread(uint8_t id, const char ** strCMD, float* parameter, uint16_t amount){        /**********************************测试*********************************/
	FIL fpID;		//ID文件
	char strLine[64];			//暂存每一行参数字符串
	char strFormat[64];	//暂存每一行参数字符串
	uint16_t index = 0;
	uint8_t res;
	//打开已存在的ID文件以及以读取的方式打开
	res = tfOpenMotorFileID(&fpID, id, FA_OPEN_EXISTING | FA_READ);
	//如果打开失败则退出
	if(res)		return res;
	
	f_lseek(&fpID,0);
	
	while(index < amount){
		f_gets(strLine, 64, &fpID);
		strcpy(strFormat,(const char*)strCMD[index]);
		strcat(strFormat,"=%f");
		sscanf(strLine,strFormat,&parameter[index]);
		index ++;
	}
	f_close(&fpID);
	return 0;
}

/*
***************************************************
函数名：tfOverwrite
功能：覆写电机配置文件参数
入口参数：id：ID号
				strPara：参数字符串指针
				parameter：输出参数
				amount：数量
返回值：初始化结果
				0x00：正常
				0x10：(STA_NOINIT<<4)，TF卡初始化失败
				其余：请看FRESULT枚举定义
应用范围：外部调用
备注：如果不存在对应的ID号的文件则新建一个文件
***************************************************
*/
uint8_t tfMotorwrite(uint8_t id, const char ** strPara, float* parameter, uint16_t amount){    /**********************************测试*********************************/
	FIL fpID;		//ID文件
	char strLine[64];			//暂存每一行参数字符串
	uint16_t index = 0;
	uint8_t res;
	//创建ID文件以及以读写的方式打开，如存在则覆盖
	res = tfOpenMotorFileID(&fpID, id, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	//如果打开失败则退出
	if(res)		return res;
	f_lseek(&fpID,0);
	while(index < amount){
		sprintf(strLine, "%s=%g\r\n", strPara[index], parameter[index]);
		f_puts(strLine, &fpID);	//向暂存文件写入其余函数
		index ++;
	}
	f_close(&fpID);
	return 0;
}



