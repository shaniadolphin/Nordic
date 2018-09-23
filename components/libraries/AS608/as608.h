#ifndef __AS608_H
#define __AS608_H

#include <stdint.h>

#define PS_Sta   PAin(6)//��ָ��ģ��״̬����
#define CharBuffer1 0x01
#define CharBuffer2 0x02

#define ENSURE_OK				0x00
#define ENSURE_PACKAGE_ERROR 	0x01
#define ENSURE_NO_FINGER 		0x02
#define ENSURE_ENTERING_ERROR 	0x03
#define ENSURE_FINGER_WEAK 		0x04
#define ENSURE_FINGER_VAGUE 	0x05
#define ENSURE_FINGER_MESS 		0x06
#define ENSURE_FINGER_SMALL 	0x07
#define ENSURE_NOT_MATCHING		0x08
#define ENSURE_NOT_FINDING		0x09
#define ENSURE_FAIL_MERGE		0x0a
#define ENSURE_OVERRANGE		0x0b
#define ENSURE_FAIL_DEL			0x10
#define ENSURE_FAIL_EMPTY		0x11
#define ENSURE_FAIL_IMAGE		0x15
#define ENSURE_FLASH_ERROR		0x18
#define ENSURE_UNDEFINED		0x19
#define ENSURE_ERROR_REG		0x1a
#define ENSURE_REG_ERROR		0x1b
#define ENSURE_REG_PAGE			0x1c
#define ENSURE_FINGER_FULL		0x1f
#define ENSURE_ADDRESS_ERR		0x20

//extern uint32_t AS608Addr;//ģ���ַ

//extern uint8_t data_array[100];

//extern uint8_t data_array_index;

typedef struct  
{
	uint16_t pageID;//ָ��ID
	uint16_t mathscore;//ƥ��÷�
}SearchResult;

typedef struct
{
	uint16_t PS_max;//ָ���������
	uint8_t  PS_level;//��ȫ�ȼ�
	uint32_t PS_addr;
	uint8_t  PS_size;//ͨѶ���ݰ���С
	uint8_t  PS_N;//�����ʻ���N
}SysPara;

void PS_StaGPIO_Init(void);//��ʼ��PA6��״̬����
	
uint8_t PS_GetImage(void); //¼��ͼ�� 
 
uint8_t PS_GenChar(uint8_t BufferID);//�������� 

uint8_t PS_Match(void);//��ȷ�ȶ���öָ������ 

uint8_t PS_Search(uint8_t BufferID,uint16_t StartPage,uint16_t PageNum,SearchResult *p);//����ָ�� 
 
uint8_t PS_RegModel(void);//�ϲ�����������ģ�壩 
 
uint8_t PS_StoreChar(uint8_t BufferID,uint16_t PageID);//����ģ�� 

uint8_t PS_DeletChar(uint16_t PageID,uint16_t N);//ɾ��ģ�� 

uint8_t PS_Empty(void);//���ָ�ƿ� 

uint8_t PS_WriteReg(uint8_t RegNum,uint8_t DATA);//дϵͳ�Ĵ��� 
 
uint8_t PS_ReadSysPara(SysPara *p); //��ϵͳ�������� 

uint8_t PS_SetAddr(uint32_t addr);  //����ģ���ַ 

uint8_t PS_WriteNotepad(uint8_t NotePageNum,uint8_t *content);//д���±� 

uint8_t PS_ReadNotepad(uint8_t NotePageNum,uint8_t *note);//������ 

uint8_t PS_HighSpeedSearch(uint8_t BufferID,uint16_t StartPage,uint16_t PageNum,SearchResult *p);//�������� 
  
uint8_t PS_ValidTempleteNum(uint16_t *ValidN);//����Чģ����� 

uint8_t PS_HandShake(uint32_t *PS_Addr); //��AS608ģ������

const char *EnsureMessage(uint8_t ensure);//ȷ���������Ϣ����

uint8_t press_FR(SearchResult *seach);//ˢָ��

uint8_t Del_FR(uint16_t ValidN);//ɾ��ָ��

uint16_t Add_FR(uint16_t ValidN);//¼ָ��
#endif

