//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK ս��V3 STM32������
//ATK-AS608ָ��ʶ��ģ����������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/3/29
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights dataerved									  
////////////////////////////////////////////////////////////////////////////////// 	
#include <string.h>
//#include "delay.h" 	
//#include "usart2.h"
#include "AS608.h"
#include "nrf_delay.h"

#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
#include "SEGGER_RTT.h"
#endif

uint32_t AS608Addr = 0XFFFFFFFF; //Ĭ��
extern uint8_t data_array[100];
extern uint8_t data_array_index;
extern uint8_t ErrMessage;

extern uint32_t app_uart_put(uint8_t byte);
//��ʼ��PA6Ϊ��������		    
//��������Ӧ״̬(������Ӧʱ����ߵ�ƽ�ź�)
void PS_StaGPIO_Init(void)
{  
#if 0	
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  //��ʼ����״̬����GPIOA
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//��������ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO	
#endif
}
//���ڷ���һ���ֽ�
static void MYUSART_SendData(uint8_t data)
{
	//while((USART2->SR&0X40)==0); 
	//USART2->DR = data;
	app_uart_put(data);
}

//���Ͱ�ͷ
static void SendHead(void)
{
	MYUSART_SendData(0xEF);
	MYUSART_SendData(0x01);
}
//���͵�ַ
static void SendAddr(void)
{
	MYUSART_SendData(AS608Addr>>24);
	MYUSART_SendData(AS608Addr>>16);
	MYUSART_SendData(AS608Addr>>8);
	MYUSART_SendData(AS608Addr);
}
//���Ͱ���ʶ,
static void SendFlag(uint8_t flag)
{
	MYUSART_SendData(flag);
}
//���Ͱ�����
static void SendLength(int length)
{
	MYUSART_SendData(length>>8);
	MYUSART_SendData(length);
}
//����ָ����
static void Sendcmd(uint8_t cmd)
{
	MYUSART_SendData(cmd);
}
//����У���
static void SendCheck(uint16_t check)
{
	MYUSART_SendData(check>>8);
	MYUSART_SendData(check);
}
//�ж��жϽ��յ�������û��Ӧ���
//waittimeΪ�ȴ��жϽ������ݵ�ʱ�䣨��λ1ms��
//����ֵ�����ݰ��׵�ַ
static uint8_t *JudgeStr(uint16_t waittime)
{
	char *data;
	uint8_t str[8];
	str[0]=0xef;
	str[1]=0x01;
	str[2]=AS608Addr>>24;
	str[3]=AS608Addr>>16;
	str[4]=AS608Addr>>8;
	str[5]=AS608Addr;
	str[6]=0x07;
	str[7]='\0';
	while(waittime)
	{
		nrf_delay_ms(10);//delay_ms(1);
		//if(USART2_RX_STA&0X8000)//���յ�һ������
		//{
		//	USART2_RX_STA=0;
		if(data_array_index > 0)
		{
			data=strstr((const char*)data_array,(const char*)str);
			//#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
			//SEGGER_RTT_printf(0,"JudgeStr:%s\r\n",data);//ͨѶ�ɹ�
			//#endif	
			if(data)
				return (uint8_t*)data;	
		}
		//}
		waittime-=10;
	}
	return 0;
}
//¼��ͼ�� PS_GetImage
//����:̽����ָ��̽�⵽��¼��ָ��ͼ�����ImageBuffer�� 
//ģ�鷵��ȷ����
uint8_t PS_GetImage(void)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x03);
	Sendcmd(0x01);
	temp =  0x01+0x03+0x01;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_GetImage\r\n");//ͨѶ�ɹ�
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//�������� PS_GenChar
//����:��ImageBuffer�е�ԭʼͼ������ָ�������ļ�����CharBuffer1��CharBuffer2			 
//����:BufferID --> charBuffer1:0x01	charBuffer1:0x02												
//ģ�鷵��ȷ����
uint8_t PS_GenChar(uint8_t BufferID)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x04);
	Sendcmd(0x02);
	MYUSART_SendData(BufferID);
	temp = 0x01+0x04+0x02+BufferID;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_GenChar\r\n");//ͨѶ�ɹ�
	#endif		
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//��ȷ�ȶ���öָ������ PS_Match
//����:��ȷ�ȶ�CharBuffer1 ��CharBuffer2 �е������ļ� 
//ģ�鷵��ȷ����
uint8_t PS_Match(void)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x03);
	Sendcmd(0x03);
	temp = 0x01+0x03+0x03;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_Match\r\n");//ͨѶ�ɹ�
	#endif		
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//����ָ�� PS_Search
//����:��CharBuffer1��CharBuffer2�е������ļ����������򲿷�ָ�ƿ�.�����������򷵻�ҳ�롣			
//����:  BufferID @ref CharBuffer1	CharBuffer2
//˵��:  ģ�鷵��ȷ���֣�ҳ�루����ָ��ģ�壩
uint8_t PS_Search(uint8_t BufferID,uint16_t StartPage,uint16_t PageNum,SearchResult *p)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x08);
	Sendcmd(0x04);
	MYUSART_SendData(BufferID);
	MYUSART_SendData(StartPage>>8);
	MYUSART_SendData(StartPage);
	MYUSART_SendData(PageNum>>8);
	MYUSART_SendData(PageNum);
	temp = 0x01+0x08+0x04+BufferID
	+(StartPage>>8)+(uint8_t)StartPage
	+(PageNum>>8)+(uint8_t)PageNum;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_Search\r\n");//ͨѶ�ɹ�
	#endif		
	data=JudgeStr(2000);
	if(data)
	{
		ensure = data[9];
		p->pageID   =(data[10]<<8)+data[11];
		p->mathscore=(data[12]<<8)+data[13];	
	}
	else
		ensure = 0xff;
	return ensure;	
}
//�ϲ�����������ģ�壩PS_RegModel
//����:��CharBuffer1��CharBuffer2�е������ļ��ϲ����� ģ��,�������CharBuffer1��CharBuffer2	
//˵��:  ģ�鷵��ȷ����
uint8_t PS_RegModel(void)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x03);
	Sendcmd(0x05);
	temp = 0x01+0x03+0x05;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_RegModel\r\n");//ͨѶ�ɹ�
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;		
}
//����ģ�� PS_StoreChar
//����:�� CharBuffer1 �� CharBuffer2 �е�ģ���ļ��浽 PageID ��flash���ݿ�λ�á�			
//����:  BufferID @ref charBuffer1:0x01	charBuffer1:0x02
//       PageID��ָ�ƿ�λ�úţ�
//˵��:  ģ�鷵��ȷ����
uint8_t PS_StoreChar(uint8_t BufferID,uint16_t PageID)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x06);
	Sendcmd(0x06);
	MYUSART_SendData(BufferID);
	MYUSART_SendData(PageID>>8);
	MYUSART_SendData(PageID);
	temp = 0x01+0x06+0x06+BufferID
	+(PageID>>8)+(uint8_t)PageID;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_StoreChar\r\n");//ͨѶ�ɹ�
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;	
}
//ɾ��ģ�� PS_DeletChar
//����:  ɾ��flash���ݿ���ָ��ID�ſ�ʼ��N��ָ��ģ��
//����:  PageID(ָ�ƿ�ģ���)��Nɾ����ģ�������
//˵��:  ģ�鷵��ȷ����
uint8_t PS_DeletChar(uint16_t PageID,uint16_t N)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x07);
	Sendcmd(0x0C);
	MYUSART_SendData(PageID>>8);
	MYUSART_SendData(PageID);
	MYUSART_SendData(N>>8);
	MYUSART_SendData(N);
	temp = 0x01+0x07+0x0C
	+(PageID>>8)+(uint8_t)PageID
	+(N>>8)+(uint8_t)N;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_DeletChar\r\n");//ͨѶ�ɹ�
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//���ָ�ƿ� PS_Empty
//����:  ɾ��flash���ݿ�������ָ��ģ��
//����:  ��
//˵��:  ģ�鷵��ȷ����
uint8_t PS_Empty(void)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x03);
	Sendcmd(0x0D);
	temp = 0x01+0x03+0x0D;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_Empty\r\n");//ͨѶ�ɹ�
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//дϵͳ�Ĵ��� PS_WriteReg
//����:  дģ��Ĵ���
//����:  �Ĵ������RegNum:4\5\6
//˵��:  ģ�鷵��ȷ����
uint8_t PS_WriteReg(uint8_t RegNum,uint8_t DATA)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x05);
	Sendcmd(0x0E);
	MYUSART_SendData(RegNum);
	MYUSART_SendData(DATA);
	temp = RegNum+DATA+0x01+0x05+0x0E;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_WriteReg\r\n");//ͨѶ�ɹ�
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	//if(ensure==0)
	//	printf("\r\n���ò����ɹ���");
	//else
	//	printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//��ϵͳ�������� PS_ReadSysPara
//����:  ��ȡģ��Ļ��������������ʣ�����С��)
//����:  ��
//˵��:  ģ�鷵��ȷ���� + ����������16bytes��
uint8_t PS_ReadSysPara(SysPara *p)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x03);
	Sendcmd(0x0F);
	temp = 0x01+0x03+0x0F;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_ReadSysPara\r\n");//ͨѶ�ɹ�
	#endif
	data=JudgeStr(1000);
	if(data)
	{
		ensure = data[9];
		p->PS_max = (data[14]<<8)+data[15];
		p->PS_level = data[17];
		p->PS_addr=(data[18]<<24)+(data[19]<<16)+(data[20]<<8)+data[21];
		p->PS_size = data[23];
		p->PS_N = data[25];
	}		
	else
		ensure=0xff;
	if(ensure==0x00)
	{
		//printf("\r\nģ�����ָ������=%d",p->PS_max);
		//printf("\r\n�Աȵȼ�=%d",p->PS_level);
		//printf("\r\n��ַ=%x",p->PS_addr);
		//printf("\r\n������=%d",p->PS_N*9600);
	}
	//else 
		//printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//����ģ���ַ PS_SetAddr
//����:  ����ģ���ַ
//����:  PS_addr
//˵��:  ģ�鷵��ȷ����
uint8_t PS_SetAddr(uint32_t PS_addr)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x07);
	Sendcmd(0x15);
	MYUSART_SendData(PS_addr>>24);
	MYUSART_SendData(PS_addr>>16);
	MYUSART_SendData(PS_addr>>8);
	MYUSART_SendData(PS_addr);
	temp = 0x01+0x07+0x15
	+(uint8_t)(PS_addr>>24)+(uint8_t)(PS_addr>>16)
	+(uint8_t)(PS_addr>>8) +(uint8_t)PS_addr;				
	SendCheck(temp);
	AS608Addr=PS_addr;//������ָ�������ַ
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_SetAddr\r\n");//ͨѶ�ɹ�
	#endif
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;	
		AS608Addr = PS_addr;
	//if(ensure==0x00)
	//	printf("\r\n���õ�ַ�ɹ���");
	//else
	//	printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//���ܣ� ģ���ڲ�Ϊ�û�������256bytes��FLASH�ռ����ڴ��û����±�,
//	�ü��±��߼��ϱ��ֳ� 16 ��ҳ��
//����:  NotePageNum(0~15),Byte32(Ҫд�����ݣ�32���ֽ�)
//˵��:  ģ�鷵��ȷ����
uint8_t PS_WriteNotepad(uint8_t NotePageNum,uint8_t *Byte32)
{
	uint16_t temp;
	uint8_t  ensure,i;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(36);
	Sendcmd(0x18);
	MYUSART_SendData(NotePageNum);
	for(i=0;i<32;i++)
	 {
		 MYUSART_SendData(Byte32[i]);
		 temp += Byte32[i];
	 }
	temp =0x01+36+0x18+NotePageNum+temp;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	 
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_WriteNotepad\r\n");//ͨѶ�ɹ�
	#endif
	 data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//������PS_ReadNotepad
//���ܣ�  ��ȡFLASH�û�����128bytes����
//����:  NotePageNum(0~15)
//˵��:  ģ�鷵��ȷ����+�û���Ϣ
uint8_t PS_ReadNotepad(uint8_t NotePageNum,uint8_t *Byte32)
{
	uint16_t temp;
	uint8_t  ensure,i;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x04);
	Sendcmd(0x19);
	MYUSART_SendData(NotePageNum);
	temp = 0x01+0x04+0x19+NotePageNum;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_ReadNotepad\r\n");//ͨѶ�ɹ�
	#endif
	data=JudgeStr(2000);
	if(data)
	{
		ensure=data[9];
		for(i=0;i<32;i++)
		{
			Byte32[i]=data[10+i];
		}
	}
	else
		ensure=0xff;
	return ensure;
}
//��������PS_HighSpeedSearch
//���ܣ��� CharBuffer1��CharBuffer2�е������ļ��������������򲿷�ָ�ƿ⡣
//		  �����������򷵻�ҳ��,��ָ����ڵ�ȷ������ָ�ƿ��� ���ҵ�¼ʱ����
//		  �ܺõ�ָ�ƣ���ܿ�������������
//����:  BufferID�� StartPage(��ʼҳ)��PageNum��ҳ����
//˵��:  ģ�鷵��ȷ����+ҳ�루����ָ��ģ�壩
uint8_t PS_HighSpeedSearch(uint8_t BufferID,uint16_t StartPage,uint16_t PageNum,SearchResult *p)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x08);
	Sendcmd(0x1b);
	MYUSART_SendData(BufferID);
	MYUSART_SendData(StartPage>>8);
	MYUSART_SendData(StartPage);
	MYUSART_SendData(PageNum>>8);
	MYUSART_SendData(PageNum);
	temp = 0x01+0x08+0x1b+BufferID
	+(StartPage>>8)+(uint8_t)StartPage
	+(PageNum>>8)+(uint8_t)PageNum;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_HighSpeedSearch\r\n");//ͨѶ�ɹ�
	#endif
	data=JudgeStr(2000);
 	if(data)
	{
		ensure=data[9];
		p->pageID 	=(data[10]<<8) +data[11];
		p->mathscore=(data[12]<<8) +data[13];
	}
	else
		ensure=0xff;
	return ensure;
}
//����Чģ����� PS_ValidTempleteNum
//���ܣ�����Чģ�����
//����: ��
//˵��: ģ�鷵��ȷ����+��Чģ�����ValidN
uint8_t PS_ValidTempleteNum(uint16_t *ValidN)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//�������ʶ
	SendLength(0x03);
	Sendcmd(0x1d);
	temp = 0x01+0x03+0x1d;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_ValidTempleteNum\r\n");//ͨѶ�ɹ�
	#endif
	data=JudgeStr(2000);
	if(data)
	{
		ensure=data[9];
		*ValidN = (data[10]<<8) +data[11];
	}		
	else
		ensure=0xff;
	
	//if(ensure==0x00)
	//{
	//	printf("\r\n��Чָ�Ƹ���=%d",(data[10]<<8)+data[11]);
	//}
	//else
	//	printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//��AS608���� PS_HandShake
//����: PS_Addr��ַָ��
//˵��: ģ�鷵�µ�ַ����ȷ��ַ��	
uint8_t PS_HandShake(uint32_t *PS_Addr)
{
	SendHead();
	SendAddr();
	MYUSART_SendData(0X01);
	MYUSART_SendData(0X00);
	MYUSART_SendData(0X00);	
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if 0
	delay_ms(200);
	if(USART2_RX_STA&0X8000)//���յ�����
	{		
		if(//�ж��ǲ���ģ�鷵�ص�Ӧ���				
				USART2_RX_BUF[0]==0XEF
				&&USART2_RX_BUF[1]==0X01
				&&USART2_RX_BUF[6]==0X07
			)
			{
				*PS_Addr=(USART2_RX_BUF[2]<<24) + (USART2_RX_BUF[3]<<16)
								+(USART2_RX_BUF[4]<<8) + (USART2_RX_BUF[5]);
				USART2_RX_STA=0;
				return 0;
			}
		USART2_RX_STA=0;					
	}
	#else
	nrf_delay_ms(200);
	if(data_array_index)
	{
		if(data_array[0]==0xEF && data_array[1]==0x01 && data_array[6]==0x07)//�ж��ǲ���ģ�鷵�ص�Ӧ���	
		{
			*PS_Addr=(data_array[2]<<24) + (data_array[3]<<16)
								+(data_array[4]<<8) + (data_array[5]);
			data_array_index=0;
			return 0;
		}		
	}
	#endif
	
	return 1;		
}
//ģ��Ӧ���ȷ������Ϣ����
//���ܣ�����ȷ���������Ϣ������Ϣ
//����: ensure

const char *EnsureMessage(uint8_t ensure) 
{
	const char *p;
	switch(ensure)
	{
		case  ENSURE_OK:
			p="ENSURE_OK";break;		
		case  ENSURE_PACKAGE_ERROR:
			p="ENSURE_PACKAGE_ERROR";break;
		case  ENSURE_NO_FINGER:
			p="ENSURE_NO_FINGER";break;
		case  ENSURE_ENTERING_ERROR:
			p="ENSURE_ENTERING_ERROR";break;
		case  ENSURE_FINGER_WEAK:
			p="ENSURE_FINGER_WEAK";break;//ָ��ͼ��̫�ɡ�̫��������������
		case  ENSURE_FINGER_VAGUE:
			p="ENSURE_FINGER_VAGUE";break;//ָ��ͼ��̫ʪ��̫��������������
		case  ENSURE_FINGER_MESS:
			p="ENSURE_FINGER_MESS";break;//ָ��ͼ��̫�Ҷ�����������
		case  ENSURE_FINGER_SMALL:
			p="ENSURE_FINGER_SMALL";break;//ָ��ͼ����������������̫�٣������̫С��������������
		case  ENSURE_NOT_MATCHING:
			p="ENSURE_NOT_MATCHING";break;//ָ�Ʋ�ƥ��
		case  ENSURE_NOT_FINDING:
			p="ENSURE_NOT_FINDING";break;//û������ָ��
		case  ENSURE_FAIL_MERGE:
			p="ENSURE_FAIL_MERGE";break;//�����ϲ�ʧ��
		case  ENSURE_OVERRANGE:
			p="ENSURE_OVERRANGE";//����ָ�ƿ�ʱ��ַ��ų���ָ�ƿⷶΧ
		case  ENSURE_FAIL_DEL:
			p="ENSURE_FAIL_DEL";break;//ɾ��ģ��ʧ��
		case  ENSURE_FAIL_EMPTY:
			p="ENSURE_FAIL_EMPTY";break;//���ָ�ƿ�ʧ��	
		case  ENSURE_FAIL_IMAGE:
			p="ENSURE_FAIL_IMAGE";break;//��������û����Чԭʼͼ��������ͼ��
		case  ENSURE_FLASH_ERROR:
			p="ENSURE_FLASH_ERROR";break;//��д FLASH ����
		case  ENSURE_UNDEFINED:
			p="ENSURE_UNDEFINED";break;//δ�������
		case  ENSURE_ERROR_REG:
			p="ENSURE_ERROR_REG";break;//��Ч�Ĵ�����
		case  ENSURE_REG_ERROR:
			p="ENSURE_REG_ERROR";break;//�Ĵ����趨���ݴ���
		case  ENSURE_REG_PAGE:
			p="ENSURE_REG_PAGE";break;//���±�ҳ��ָ������
		case  ENSURE_FINGER_FULL:
			p="ENSURE_FINGER_FULL";break;//ָ�ƿ���
		case  ENSURE_ADDRESS_ERR:
			p="ENSURE_ADDRESS_ERR";break;//��ַ����
		default :
			p="ENSURE_ERROR_CODE";break;//ģ�鷵��ȷ��������
	}
 return p;	
}


//��ʾȷ���������Ϣ
void ShowErrMessage(uint8_t ensure)
{
	//LCD_Fill(0,120,lcddev.width,160,WHITE);
	//Show_Str_Mid(0,120,(u8*)EnsureMessage(ensure),16,240);
	//printf("%s\r\n",(uint8_t*)EnsureMessage(ensure));//ͨѶ�ɹ�
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
		SEGGER_RTT_printf(0,"%s\r\n",(uint8_t*)EnsureMessage(ensure));//ͨѶ�ɹ�
	#endif	
	ErrMessage  = ensure;
}
//¼ָ��
uint16_t Add_FR(uint16_t ValidN)
{
	uint8_t i=0, ensure ,processnum=0;
	uint16_t ID;
	//uint16_t ValidN;//ģ������Чģ�����
	while(1)
	{
		switch (processnum)
		{
			case 0:
				i++;
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"Pleas touch finger!",16,240);//�밴��ָ
				//printf("Pleas touch finger!\r\n");//�밴��ָ
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
					SEGGER_RTT_printf(0,"Pleas touch finger!\r\n");//�밴��ָ
				#endif				
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(CharBuffer1);//��������
					if(ensure==0x00)
					{
						//LCD_Fill(0,120,lcddev.width,160,WHITE);
						//Show_Str_Mid(0,120,"Fingerprint correct",16,240);//ָ����ȷ
						//printf("Fingerprint correct!\r\n");//ָ����ȷ
						#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
							SEGGER_RTT_printf(0,"Fingerprint correct!\r\n");//ָ����ȷ
						#endif							
						i=0;
						processnum=1;//�����ڶ���						
					}
					else 
						ShowErrMessage(ensure);				
				}
				else 
					ShowErrMessage(ensure);						
				break;
			
			case 1:
				i++;
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"Pleas touch finger again!",16,240);//�ٰ�һ����ָ
				//printf("Pleas touch finger again!\r\n");//�ٰ�һ����ָ
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
					SEGGER_RTT_printf(0,"Pleas touch finger again!\r\n");//�ٰ�һ����ָ
				#endif					
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(CharBuffer2);//��������			
					if(ensure==0x00)
					{
						//LCD_Fill(0,120,lcddev.width,160,WHITE);
						//Show_Str_Mid(0,120,"Fingerprint correct",16,240);//ָ����ȷ
						//printf("Fingerprint correct\r\n");//ָ����ȷ
						#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
							SEGGER_RTT_printf(0,"Fingerprint correct!\r\n");//ָ����ȷ
						#endif							
						i=0;
						processnum=2;//����������
					}
					else 
						ShowErrMessage(ensure);	
				}
				else 
					ShowErrMessage(ensure);		
				break;

			case 2:
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"Compare twice fingerprint",16,240);//�Ա�����ָ��
				//printf("Compare twice fingerprint!\r\n");//�Ա�����ָ��
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
					SEGGER_RTT_printf(0,"Compare twice fingerprint!\r\n");//�Ա�����ָ��
				#endif					
				ensure=PS_Match();
				if(ensure==0x00) 
				{
					//LCD_Fill(0,120,lcddev.width,160,WHITE);
					//Show_Str_Mid(0,120,"Twice fingerprint are same",16,240);//����ָ����һ����
					//printf("Twice fingerprint are same!\r\n");//����ָ����һ����
					#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
						SEGGER_RTT_printf(0,"Twice fingerprint are same!\r\n");//����ָ����һ����
					#endif					
					processnum=3;//�������Ĳ�
				}
				else 
				{
					//LCD_Fill(0,100,lcddev.width,160,WHITE);
					//Show_Str_Mid(0,100,"Compare fail,pleas touch again!",16,240);//�Ա�ʧ�ܣ������°���ָ
					//printf("Compare fail,pleas touch again!\r\n");//�Ա�ʧ�ܣ������°���ָ
					#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
						SEGGER_RTT_printf(0,"Compare fail,pleas touch again!\r\n");//�Ա�ʧ�ܣ������°���ָ
					#endif						
					ShowErrMessage(ensure);
					i=0;
					processnum=0;//���ص�һ��		
				}
				nrf_delay_ms(1000);//delay_ms(1000);
				break;

			case 3:
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"Generate fingerprint template",16,240);//����һ��ָ��ģ��
				//printf("Generate fingerprint template!\r\n");//����һ��ָ��ģ��
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
					SEGGER_RTT_printf(0,"Generate fingerprint template!\r\n");//����һ��ָ��ģ��
				#endif					
				ensure=PS_RegModel();
				if(ensure==0x00) 
				{
					//LCD_Fill(0,120,lcddev.width,160,WHITE);
					//Show_Str_Mid(0,120,"Generate fingerprint success",16,240);//����ָ��ģ��ɹ�
					//printf("Generate fingerprint success!\r\n");//����ָ��ģ��ɹ�
					#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
						SEGGER_RTT_printf(0,"Generate fingerprint success!\r\n");//����ָ��ģ��ɹ�
					#endif						
					processnum=4;//�������岽
				}
				else 
				{
					processnum=0;
					ShowErrMessage(ensure);
				}
				nrf_delay_ms(1000);//delay_ms(1000);
				break;
				
			case 4:	
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"Intput ID and save with ENTER!",16,240);//����ID������Enter������
				//Show_Str_Mid(0,120,"0=< ID <=299",16,240);	
				//printf("Intput ID and save with ENTER!\r\n");//����ID������Enter������
				//printf("0=< ID <=299\r\n");
				//do
				//	ID=GET_NUM();
				//while(!(ID<300));//����DI����С��300
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
					SEGGER_RTT_printf(0,"Intput ID and save with ENTER!\r\n");//����ID������Enter������
				#endif					
				ensure=PS_StoreChar(CharBuffer2,ID);//����ģ��
				if(ensure==0x00) 
				{			
					//LCD_Fill(0,100,lcddev.width,160,WHITE);					
					//Show_Str_Mid(0,120,"Add fingerprint success!!!",16,240);//���ָ�Ƴɹ�
					//printf("Add fingerprint success!!!\r\n");//���ָ�Ƴɹ�
					PS_ValidTempleteNum(&ValidN);//����ָ�Ƹ���
					#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
						SEGGER_RTT_printf(0,"Add fingerprint success!!!\r\n");//���ָ�Ƴɹ�
						SEGGER_RTT_printf(0,"%d\r\n",ValidN);//��ʾʣ��ָ�Ƹ���
					#endif						
					//printf("%d\r\n",AS608Para.PS_max-ValidN);//��ʾʣ��ָ�Ƹ���
					//LCD_ShowNum(80,80,AS608Para.PS_max-ValidN,3,16);//��ʾʣ��ָ�Ƹ���
					nrf_delay_ms(1500);//delay_ms(1500);//��ʱ�������ʾ	
					//LCD_Fill(0,100,240,160,WHITE);
					return ensure;
				}
				else 
				{
					processnum=0;
					ShowErrMessage(ensure);
				}					
				break;				
		}
		nrf_delay_ms(800);//delay_ms(800);
		if(i==5)//����5��û�а���ָ���˳�
		{
			//LCD_Fill(0,100,lcddev.width,160,WHITE);
			//printf("exit\r\n");//exit
			#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
				SEGGER_RTT_printf(0,"exit\r\n");//exit
			#endif			
			return 1;	
		}				
	}
}

//ˢָ��
uint8_t press_FR(SearchResult *seach)
{
	uint8_t ensure;
	//char *str;
	SearchResult seachtmp;
	ensure=PS_GetImage();
	if(ensure==0x00)//��ȡͼ��ɹ� 
	{	
		ensure=PS_GenChar(CharBuffer1);
		if(ensure==0x00) //���������ɹ�
		{		
			ensure=PS_HighSpeedSearch(CharBuffer1, 0, 300, &seachtmp);
			if(ensure==0x00)//�����ɹ�
			{				
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"Search fingerprint success",16,240);//����ָ�Ƴɹ�				
				//printf("Search fingerprint success\r\n");//����ָ�Ƴɹ�		
				//sprintf(str,"Match ID:%d  Match score:%d",seach.pageID,seach.mathscore);//��ʾƥ��ָ�Ƶ�ID�ͷ���
				//printf("%s\r\n",(uint8_t*)str);//��ʾƥ��ָ�Ƶ�ID�ͷ���
				//Show_Str_Mid(0,140,(u8*)str,16,240);
				//myfree(str);
				seach->pageID = seachtmp.pageID;
				seach->mathscore = seachtmp.mathscore;
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
				SEGGER_RTT_printf(0,"Search fingerprint success\r\n");
				SEGGER_RTT_printf(0,"Match ID:%d \r\n",seach->pageID);
				SEGGER_RTT_printf(0,"Match score:%d \r\n",seach->mathscore);
				//SEGGER_RTT_printf(0,"Match ID:%d \r\n",seachtmp.pageID);
				//SEGGER_RTT_printf(0,"Match score:%d \r\n",seachtmp.mathscore);
				//sprintf(str,"Match ID:%d  Match score:%d",seach->pageID,seach->mathscore);//��ʾƥ��ָ�Ƶ�ID�ͷ���
				//SEGGER_RTT_printf(0,"%s\r\n",(uint8_t*)str);//��ʾƥ��ָ�Ƶ�ID�ͷ���
				#endif
				return ensure;
			}
			else
			{				
				ShowErrMessage(ensure);	
				return ensure;
			}				
		}
		//else
			//ShowErrMessage(ensure);
		nrf_delay_ms(1000); //delay_ms(1000);//��ʱ�������ʾ
		//LCD_Fill(0,100,lcddev.width,160,WHITE);
	}
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"Search fingerprint exit\r\n");//����ָ�Ƴɹ�		
	#endif	
	ShowErrMessage(ensure);		
	//printf("Search fingerprint exit\r\n");//����ָ�Ƴɹ�		
	return ensure;	
}

//ɾ��ָ��
uint8_t Del_FR(uint16_t ValidN)
{
	uint8_t  ensure;
	uint16_t num;
	//LCD_Fill(0,100,lcddev.width,160,WHITE);
	//Show_Str_Mid(0,100,"Delete fingerprint",16,240);//��ʾɾ��ָ��
	//Show_Str_Mid(0,120,"Input ID and touch Enter!",16,240);//��ʾ����ID�����¡�Enter��
	//Show_Str_Mid(0,140,"0=< ID <=299",16,240);
	nrf_delay_ms(50);//delay_ms(50);
	//AS608_load_keyboard(0,170,(u8**)kbd_delFR);
	num = 0xFFFF;//GET_NUM();//��ȡ���ص���ֵ
	if(num==0xFFFF)
	{
		return 0; //������ҳ��
	}
	else if(num==0xFF00)
	{
		ensure=PS_Empty();//���ָ�ƿ�
	}
	else 
	{
		ensure=PS_DeletChar(num,1);//ɾ������ָ��
	}
	if(ensure==0)
	{
		//LCD_Fill(0,120,lcddev.width,160,WHITE);
		//Show_Str_Mid(0,140,"Delete fingerprint success!!!",16,240);//ɾ��ָ�Ƴɹ�		
	}
	else
	{
		ShowErrMessage(ensure);	
	}
	nrf_delay_ms(1500);	//delay_ms(1500);//��ʱ�������ʾ
	PS_ValidTempleteNum(&ValidN);//����ָ�Ƹ���
	//LCD_ShowNum(80,80,AS608Para.PS_max-ValidN,3,16);//��ʾʣ��ָ�Ƹ���
	//MENU:	
	//LCD_Fill(0,100,lcddev.width,160,WHITE);
	nrf_delay_ms(50);//delay_ms(50);
	//AS608_load_keyboard(0,170,(u8**)kbd_menu);
	return ensure;
}



