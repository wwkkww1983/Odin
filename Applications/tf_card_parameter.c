#include "tf_card_parameter.h"
#include "config.h"

uint8_t tfFATFS_Init(void);
uint8_t tfWriteOneParameter(uint8_t id, const uint8_t* strPara, float parameter);
uint8_t tfReadOneParameter(uint8_t id, const uint8_t* strPara, float* parameter);
uint8_t tfOverwrite(uint8_t id, const char ** strPara, float* parameter, uint16_t amount);
uint8_t tfOverread(uint8_t id, const char ** strPara, float* parameter, uint16_t amount);

FATFS FATFS_TF; //�ļ�ϵͳĿ��

/*
***************************************************
��������tfFATFS_Init
���ܣ�TF����FATFS��ʼ��
��ڲ�������
����ֵ����ʼ�����
				0x00������
				0x10��(STA_NOINIT<<4)��TF����ʼ��ʧ��
				���ࣺ�뿴FRESULTö�ٶ���
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
uint8_t tfFATFS_Init(void){
	uint8_t res;
	res = disk_initialize(0);	//��ʼ��TF�����̣���ȡ״̬
	if(res){
		return res<<4;					//Ϊ��ֹ��������غ���������ͻ��������4λ
	}
	res = f_mount(&FATFS_TF,"0:",1);	//��������SD��
	return res;
}

uint8_t tFCardConfig(void){
	BSP_GPIO_Init(TFCARD_INT,GPIO_Mode_IPU);
	if(TFCARD_INSERT_IO == TFCARD_INSERT){						//���������⵽��ֱ�ӳ�ʼ��һ��
		parameterRunData.TFError = tfFATFS_Init();	
	}
	return parameterRunData.TFError;
}

/*
***************************************************
��������f_readParaLine
���ܣ���ȡ��Ų�����һ��
��ڲ�����fp���ļ�ָ��
				strPara�������ַ���
				strLine������������ַ���
����ֵ����ȡ�����0���ɹ���1��ʧ��
Ӧ�÷�Χ���ڲ�����
��ע��
***************************************************
*/
uint8_t f_readParaLine(FIL* fp, const uint8_t* strPara, uint8_t* strLine){
	uint8_t arrayTemp[64];	//�ݴ�ÿһ���ַ���
	uint8_t res = 1;
	TCHAR* pr_res = NULL;
	
	f_lseek(fp,0);	//�ƶ��ļ�����ָ�뵽�ʼ
	
	do{
		pr_res = f_gets ((TCHAR *)arrayTemp, 64, fp);	//��ȡһ���ַ���(��"\r\n"��β)
		if(strstr((const char *)arrayTemp,(const char *)strPara) != NULL)
		{
			//�жϲ����ַ��������Ƿ����'='����ֹ�������ַ������������ַ���
			if(arrayTemp[strlen((const char *)strPara)] == '=')
			{
				res = 0;
				break;
			}
		}
	}while(pr_res != NULL);	//�ж��Ƿ�����ļ�
	
	if(res == 0)
	{
		strcpy((char *)strLine,(const char *)arrayTemp);
	}
	return res;
}

/*
***************************************************
��������tfOpenFileID
���ܣ���ID��Ӧ�ļ�
��ڲ�����fp���ļ�ָ��
					id��ID��
				mode���ļ���ģʽ
����ֵ���뿴ö��FRESULT
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
FRESULT tfOpenFileID(FIL* fp, uint8_t id, BYTE mode){
	FRESULT res;
	char pathFileID[16];	//�ݴ�ID��Ӧ�ļ���·��
	//��ȡID��Ӧ���ļ�·��
	sprintf(pathFileID, "0:/RM/ID%d.txt", (int)id);
	res = f_open(fp, pathFileID, mode);
	if(res) 	
		f_close(fp);	//�ر��ļ�
	return res;
}

/*
***************************************************
��������tfOpenMotorFileID
���ܣ���ID��Ӧ�ĵ�������ļ�
��ڲ�����fp���ļ�ָ��
					id��ID��
				mode���ļ���ģʽ
����ֵ���뿴ö��FRESULT
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
FRESULT tfOpenMotorFileID(FIL* fp, uint8_t id, BYTE mode){    /*************************/
	FRESULT res;
	char pathFileID[16];	//�ݴ�ID��Ӧ�ļ���·��
	//��ȡID��Ӧ���ļ�·��
	sprintf(pathFileID, "0:/RM/MOTOR%d.txt", (int)id);         /**********����************/
	res = f_open(fp, pathFileID, mode);
	if(res) 	
		f_close(fp);	//�ر��ļ�
	return res;
}                                                            /*************************/

/*
***************************************************
��������tfReadOneParameter
���ܣ���ȡһ��ָ��������ֵ
��ڲ�����id��ID��
				strPara�������ַ���
				parameter���������
����ֵ����ʼ�����
				0x00������
				0x10��(STA_NOINIT<<4)��TF����ʼ��ʧ��
				0x20����ȡ������ʧ��
				���ࣺ�뿴FRESULTö�ٶ���
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
uint8_t tfReadOneParameter(uint8_t id, const uint8_t* strPara, float* parameter){
	FIL fpID;
	uint8_t strLine[64];	//���txt�ļ���ز��������ַ���
	char strFormat[64];	//��ʽ�ַ���������sscanfʹ��
	uint8_t res = 0;			//���
	
	//���Ѵ��ڵ�ID�ļ��Լ��Զ�ȡ�ķ�ʽ��
	res = tfOpenFileID(&fpID, id, FA_OPEN_EXISTING | FA_READ);
	//�����ʧ�����˳�
	if(res)		return res;
	
	//���� �����ַ��� ��ȡ ��ʽ�ַ��� 
	strcpy(strFormat,(const char*)strPara);
	strcat(strFormat,"=%f");
	
	//��ȡ�����ز�������
	res = f_readParaLine(&fpID, strPara, strLine);
	//��ȡʧ���˳�
	if(res == 1)	
		res = 0x20;
	else	//��ʽ����ȡ����
		sscanf((const char*)strLine, strFormat, parameter);

	f_close(&fpID);	//�ر��ļ�
	return res;
}

/*
***************************************************
��������tfWriteOneParameter
���ܣ�дһ��ָ��������ֵ
��ڲ�����id��ID��
				strPara�������ַ���
				parameter���������
����ֵ����ʼ�����
				0x00������
				���ࣺ�뿴FRESULTö�ٶ���
Ӧ�÷�Χ���ⲿ����
��ע����������ڶ�Ӧ�Ĳ����ַ��������ļ���������µĲ���
			��������ڶ�Ӧ��ID�ŵ��ļ����½�һ���ļ�
***************************************************
*/
uint8_t tfWriteOneParameter(uint8_t id, const uint8_t* strPara, float parameter){
	FIL fpID, fp_temp;		//ID�ļ����ݴ��ļ�
	uint8_t strLine[64];	//�ݴ�ÿһ�в����ַ���
	uint8_t res = 1;			//���
	TCHAR* pr_res = NULL;
	
	//��ID�ļ��Լ��Զ�д�ķ�ʽ�򿪣��粻�������½�һ��
	res = tfOpenFileID(&fpID, id, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	//�����ʧ�����˳�
	if(res)		return res;
	
	//�����ݴ��ļ�ID_t.txt
	f_open(&fp_temp, "0:/RM/ID_t.txt", FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
	//�ļ�ָ��ص����
	f_lseek(&fpID,0);
	f_lseek(&fp_temp,0);
	res = 1;
	do{
		pr_res = f_gets ((TCHAR *)strLine, 64, &fpID);	//��ȡһ���ַ���(��"\r\n"��β)
		if(res){
			if(strstr((const char *)strLine,(const char *)strPara) != NULL){	
				//�жϵ�ǰ���ַ������������ַ����ĺ����ǲ��ǽ���'='
				if(strLine[strlen((const char *)strPara)] == '='){
					res = 0;
					sprintf((char *)strLine, "%s=%g\r\n", strPara, parameter);
				}
			}
		}
		f_puts((const char *)strLine, &fp_temp);	//���ݴ��ļ�д�����ຯ��
	}while(pr_res != NULL);
	
	if(res){
		sprintf((char *)strLine, "%s=%g\r\n", strPara, parameter);
		f_puts((const char *)strLine, &fp_temp);	//���ݴ��ļ�д�����ຯ��
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
��������tfOverwrite
���ܣ���д����
��ڲ�����id��ID��
				strPara�������ַ���ָ��
				parameter���������
				amount������
����ֵ����ʼ�����
				0x00������
				0x10��(STA_NOINIT<<4)��TF����ʼ��ʧ��
				���ࣺ�뿴FRESULTö�ٶ���
Ӧ�÷�Χ���ⲿ����
��ע����������ڶ�Ӧ��ID�ŵ��ļ����½�һ���ļ�
***************************************************
*/
uint8_t tfOverwrite(uint8_t id, const char ** strPara, float* parameter, uint16_t amount){
	FIL fpID;		//ID�ļ�
	char strLine[64];			//�ݴ�ÿһ�в����ַ���
	uint16_t index = 0;
	uint8_t res;
	//����ID�ļ��Լ��Զ�д�ķ�ʽ�򿪣�������򸲸�
	res = tfOpenFileID(&fpID, id, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	//�����ʧ�����˳�
	if(res)		return res;
	f_lseek(&fpID,0);
	while(index < amount){
		sprintf(strLine, "%s=%g\r\n", strPara[index], parameter[index]);
		f_puts(strLine, &fpID);	//���ݴ��ļ�д�����ຯ��
		index ++;
	}
	f_close(&fpID);
	return 0;
}

/*
***************************************************
��������tfOverread
���ܣ�ͨ������
��ڲ�����id��ID��
				strPara�������ַ���ָ��
				parameter���������
				amount������
����ֵ����ʼ�����
				0x00������
				0x10��(STA_NOINIT<<4)��TF����ʼ��ʧ��
				���ࣺ�뿴FRESULTö�ٶ���
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
uint8_t tfOverread(uint8_t id, const char ** strCMD, float* parameter, uint16_t amount){
	FIL fpID;		//ID�ļ�
	char strLine[64];			//�ݴ�ÿһ�в����ַ���
	char strFormat[64];	//�ݴ�ÿһ�в����ַ���
	uint16_t index = 0;
	uint8_t res;
	//���Ѵ��ڵ�ID�ļ��Լ��Զ�ȡ�ķ�ʽ��
	res = tfOpenFileID(&fpID, id, FA_OPEN_EXISTING | FA_READ);
	//�����ʧ�����˳�
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
��������tfOverread
���ܣ�ͨ��������ò���
��ڲ�����id��ID��
				strPara�������ַ���ָ��
				parameter���������
				amount������
����ֵ����ʼ�����
				0x00������
				0x10��(STA_NOINIT<<4)��TF����ʼ��ʧ��
				���ࣺ�뿴FRESULTö�ٶ���
Ӧ�÷�Χ���ⲿ����
��ע��
***************************************************
*/
uint8_t tfMotorread(uint8_t id, const char ** strCMD, float* parameter, uint16_t amount){        /**********************************����*********************************/
	FIL fpID;		//ID�ļ�
	char strLine[64];			//�ݴ�ÿһ�в����ַ���
	char strFormat[64];	//�ݴ�ÿһ�в����ַ���
	uint16_t index = 0;
	uint8_t res;
	//���Ѵ��ڵ�ID�ļ��Լ��Զ�ȡ�ķ�ʽ��
	res = tfOpenMotorFileID(&fpID, id, FA_OPEN_EXISTING | FA_READ);
	//�����ʧ�����˳�
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
��������tfOverwrite
���ܣ���д��������ļ�����
��ڲ�����id��ID��
				strPara�������ַ���ָ��
				parameter���������
				amount������
����ֵ����ʼ�����
				0x00������
				0x10��(STA_NOINIT<<4)��TF����ʼ��ʧ��
				���ࣺ�뿴FRESULTö�ٶ���
Ӧ�÷�Χ���ⲿ����
��ע����������ڶ�Ӧ��ID�ŵ��ļ����½�һ���ļ�
***************************************************
*/
uint8_t tfMotorwrite(uint8_t id, const char ** strPara, float* parameter, uint16_t amount){    /**********************************����*********************************/
	FIL fpID;		//ID�ļ�
	char strLine[64];			//�ݴ�ÿһ�в����ַ���
	uint16_t index = 0;
	uint8_t res;
	//����ID�ļ��Լ��Զ�д�ķ�ʽ�򿪣�������򸲸�
	res = tfOpenMotorFileID(&fpID, id, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
	//�����ʧ�����˳�
	if(res)		return res;
	f_lseek(&fpID,0);
	while(index < amount){
		sprintf(strLine, "%s=%g\r\n", strPara[index], parameter[index]);
		f_puts(strLine, &fpID);	//���ݴ��ļ�д�����ຯ��
		index ++;
	}
	f_close(&fpID);
	return 0;
}



