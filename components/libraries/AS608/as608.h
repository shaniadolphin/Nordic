#ifndef __AS608_H
#define __AS608_H

#include <stdint.h>

#define PS_Sta   PAin(6)//读指纹模块状态引脚
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

//extern uint32_t AS608Addr;//模块地址

//extern uint8_t data_array[100];

//extern uint8_t data_array_index;

typedef struct  
{
	uint16_t pageID;//指纹ID
	uint16_t mathscore;//匹配得分
}SearchResult;

typedef struct
{
	uint16_t PS_max;//指纹最大容量
	uint8_t  PS_level;//安全等级
	uint32_t PS_addr;
	uint8_t  PS_size;//通讯数据包大小
	uint8_t  PS_N;//波特率基数N
}SysPara;

void PS_StaGPIO_Init(void);//初始化PA6读状态引脚
	
uint8_t PS_GetImage(void); //录入图像 
 
uint8_t PS_GenChar(uint8_t BufferID);//生成特征 

uint8_t PS_Match(void);//精确比对两枚指纹特征 

uint8_t PS_Search(uint8_t BufferID,uint16_t StartPage,uint16_t PageNum,SearchResult *p);//搜索指纹 
 
uint8_t PS_RegModel(void);//合并特征（生成模板） 
 
uint8_t PS_StoreChar(uint8_t BufferID,uint16_t PageID);//储存模板 

uint8_t PS_DeletChar(uint16_t PageID,uint16_t N);//删除模板 

uint8_t PS_Empty(void);//清空指纹库 

uint8_t PS_WriteReg(uint8_t RegNum,uint8_t DATA);//写系统寄存器 
 
uint8_t PS_ReadSysPara(SysPara *p); //读系统基本参数 

uint8_t PS_SetAddr(uint32_t addr);  //设置模块地址 

uint8_t PS_WriteNotepad(uint8_t NotePageNum,uint8_t *content);//写记事本 

uint8_t PS_ReadNotepad(uint8_t NotePageNum,uint8_t *note);//读记事 

uint8_t PS_HighSpeedSearch(uint8_t BufferID,uint16_t StartPage,uint16_t PageNum,SearchResult *p);//高速搜索 
  
uint8_t PS_ValidTempleteNum(uint16_t *ValidN);//读有效模板个数 

uint8_t PS_HandShake(uint32_t *PS_Addr); //与AS608模块握手

const char *EnsureMessage(uint8_t ensure);//确认码错误信息解析

uint8_t press_FR(SearchResult *seach);//刷指纹

uint8_t Del_FR(uint16_t ValidN);//删除指纹

uint16_t Add_FR(uint16_t ValidN);//录指纹
#endif

