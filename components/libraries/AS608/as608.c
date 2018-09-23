//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 战舰V3 STM32开发板
//ATK-AS608指纹识别模块驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/3/29
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
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

uint32_t AS608Addr = 0XFFFFFFFF; //默认
extern uint8_t data_array[100];
extern uint8_t data_array_index;
extern uint8_t ErrMessage;

extern uint32_t app_uart_put(uint8_t byte);
//初始化PA6为下拉输入		    
//读摸出感应状态(触摸感应时输出高电平信号)
void PS_StaGPIO_Init(void)
{  
#if 0	
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOA时钟
  //初始化读状态引脚GPIOA
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//输入下拉模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO	
#endif
}
//串口发送一个字节
static void MYUSART_SendData(uint8_t data)
{
	//while((USART2->SR&0X40)==0); 
	//USART2->DR = data;
	app_uart_put(data);
}

//发送包头
static void SendHead(void)
{
	MYUSART_SendData(0xEF);
	MYUSART_SendData(0x01);
}
//发送地址
static void SendAddr(void)
{
	MYUSART_SendData(AS608Addr>>24);
	MYUSART_SendData(AS608Addr>>16);
	MYUSART_SendData(AS608Addr>>8);
	MYUSART_SendData(AS608Addr);
}
//发送包标识,
static void SendFlag(uint8_t flag)
{
	MYUSART_SendData(flag);
}
//发送包长度
static void SendLength(int length)
{
	MYUSART_SendData(length>>8);
	MYUSART_SendData(length);
}
//发送指令码
static void Sendcmd(uint8_t cmd)
{
	MYUSART_SendData(cmd);
}
//发送校验和
static void SendCheck(uint16_t check)
{
	MYUSART_SendData(check>>8);
	MYUSART_SendData(check);
}
//判断中断接收的数组有没有应答包
//waittime为等待中断接收数据的时间（单位1ms）
//返回值：数据包首地址
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
		//if(USART2_RX_STA&0X8000)//接收到一次数据
		//{
		//	USART2_RX_STA=0;
		if(data_array_index > 0)
		{
			data=strstr((const char*)data_array,(const char*)str);
			//#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
			//SEGGER_RTT_printf(0,"JudgeStr:%s\r\n",data);//通讯成功
			//#endif	
			if(data)
				return (uint8_t*)data;	
		}
		//}
		waittime-=10;
	}
	return 0;
}
//录入图像 PS_GetImage
//功能:探测手指，探测到后录入指纹图像存于ImageBuffer。 
//模块返回确认字
uint8_t PS_GetImage(void)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x01);
	temp =  0x01+0x03+0x01;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_GetImage\r\n");//通讯成功
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//生成特征 PS_GenChar
//功能:将ImageBuffer中的原始图像生成指纹特征文件存于CharBuffer1或CharBuffer2			 
//参数:BufferID --> charBuffer1:0x01	charBuffer1:0x02												
//模块返回确认字
uint8_t PS_GenChar(uint8_t BufferID)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x04);
	Sendcmd(0x02);
	MYUSART_SendData(BufferID);
	temp = 0x01+0x04+0x02+BufferID;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_GenChar\r\n");//通讯成功
	#endif		
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//精确比对两枚指纹特征 PS_Match
//功能:精确比对CharBuffer1 与CharBuffer2 中的特征文件 
//模块返回确认字
uint8_t PS_Match(void)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x03);
	temp = 0x01+0x03+0x03;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_Match\r\n");//通讯成功
	#endif		
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//搜索指纹 PS_Search
//功能:以CharBuffer1或CharBuffer2中的特征文件搜索整个或部分指纹库.若搜索到，则返回页码。			
//参数:  BufferID @ref CharBuffer1	CharBuffer2
//说明:  模块返回确认字，页码（相配指纹模板）
uint8_t PS_Search(uint8_t BufferID,uint16_t StartPage,uint16_t PageNum,SearchResult *p)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
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
	SEGGER_RTT_printf(0,"PS_Search\r\n");//通讯成功
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
//合并特征（生成模板）PS_RegModel
//功能:将CharBuffer1与CharBuffer2中的特征文件合并生成 模板,结果存于CharBuffer1与CharBuffer2	
//说明:  模块返回确认字
uint8_t PS_RegModel(void)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x05);
	temp = 0x01+0x03+0x05;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_RegModel\r\n");//通讯成功
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;		
}
//储存模板 PS_StoreChar
//功能:将 CharBuffer1 或 CharBuffer2 中的模板文件存到 PageID 号flash数据库位置。			
//参数:  BufferID @ref charBuffer1:0x01	charBuffer1:0x02
//       PageID（指纹库位置号）
//说明:  模块返回确认字
uint8_t PS_StoreChar(uint8_t BufferID,uint16_t PageID)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
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
	SEGGER_RTT_printf(0,"PS_StoreChar\r\n");//通讯成功
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;	
}
//删除模板 PS_DeletChar
//功能:  删除flash数据库中指定ID号开始的N个指纹模板
//参数:  PageID(指纹库模板号)，N删除的模板个数。
//说明:  模块返回确认字
uint8_t PS_DeletChar(uint16_t PageID,uint16_t N)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
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
	SEGGER_RTT_printf(0,"PS_DeletChar\r\n");//通讯成功
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//清空指纹库 PS_Empty
//功能:  删除flash数据库中所有指纹模板
//参数:  无
//说明:  模块返回确认字
uint8_t PS_Empty(void)
{
	uint16_t temp;
  uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x0D);
	temp = 0x01+0x03+0x0D;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_Empty\r\n");//通讯成功
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//写系统寄存器 PS_WriteReg
//功能:  写模块寄存器
//参数:  寄存器序号RegNum:4\5\6
//说明:  模块返回确认字
uint8_t PS_WriteReg(uint8_t RegNum,uint8_t DATA)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x05);
	Sendcmd(0x0E);
	MYUSART_SendData(RegNum);
	MYUSART_SendData(DATA);
	temp = RegNum+DATA+0x01+0x05+0x0E;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_WriteReg\r\n");//通讯成功
	#endif	
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	//if(ensure==0)
	//	printf("\r\n设置参数成功！");
	//else
	//	printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//读系统基本参数 PS_ReadSysPara
//功能:  读取模块的基本参数（波特率，包大小等)
//参数:  无
//说明:  模块返回确认字 + 基本参数（16bytes）
uint8_t PS_ReadSysPara(SysPara *p)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x0F);
	temp = 0x01+0x03+0x0F;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_ReadSysPara\r\n");//通讯成功
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
		//printf("\r\n模块最大指纹容量=%d",p->PS_max);
		//printf("\r\n对比等级=%d",p->PS_level);
		//printf("\r\n地址=%x",p->PS_addr);
		//printf("\r\n波特率=%d",p->PS_N*9600);
	}
	//else 
		//printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//设置模块地址 PS_SetAddr
//功能:  设置模块地址
//参数:  PS_addr
//说明:  模块返回确认字
uint8_t PS_SetAddr(uint32_t PS_addr)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
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
	AS608Addr=PS_addr;//发送完指令，更换地址
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_SetAddr\r\n");//通讯成功
	#endif
	data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;	
		AS608Addr = PS_addr;
	//if(ensure==0x00)
	//	printf("\r\n设置地址成功！");
	//else
	//	printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//功能： 模块内部为用户开辟了256bytes的FLASH空间用于存用户记事本,
//	该记事本逻辑上被分成 16 个页。
//参数:  NotePageNum(0~15),Byte32(要写入内容，32个字节)
//说明:  模块返回确认字
uint8_t PS_WriteNotepad(uint8_t NotePageNum,uint8_t *Byte32)
{
	uint16_t temp;
	uint8_t  ensure,i;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
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
	SEGGER_RTT_printf(0,"PS_WriteNotepad\r\n");//通讯成功
	#endif
	 data=JudgeStr(2000);
	if(data)
		ensure=data[9];
	else
		ensure=0xff;
	return ensure;
}
//读记事PS_ReadNotepad
//功能：  读取FLASH用户区的128bytes数据
//参数:  NotePageNum(0~15)
//说明:  模块返回确认字+用户信息
uint8_t PS_ReadNotepad(uint8_t NotePageNum,uint8_t *Byte32)
{
	uint16_t temp;
	uint8_t  ensure,i;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x04);
	Sendcmd(0x19);
	MYUSART_SendData(NotePageNum);
	temp = 0x01+0x04+0x19+NotePageNum;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_ReadNotepad\r\n");//通讯成功
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
//高速搜索PS_HighSpeedSearch
//功能：以 CharBuffer1或CharBuffer2中的特征文件高速搜索整个或部分指纹库。
//		  若搜索到，则返回页码,该指令对于的确存在于指纹库中 ，且登录时质量
//		  很好的指纹，会很快给出搜索结果。
//参数:  BufferID， StartPage(起始页)，PageNum（页数）
//说明:  模块返回确认字+页码（相配指纹模板）
uint8_t PS_HighSpeedSearch(uint8_t BufferID,uint16_t StartPage,uint16_t PageNum,SearchResult *p)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
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
	SEGGER_RTT_printf(0,"PS_HighSpeedSearch\r\n");//通讯成功
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
//读有效模板个数 PS_ValidTempleteNum
//功能：读有效模板个数
//参数: 无
//说明: 模块返回确认字+有效模板个数ValidN
uint8_t PS_ValidTempleteNum(uint16_t *ValidN)
{
	uint16_t temp;
	uint8_t  ensure;
	uint8_t  *data;
	SendHead();
	SendAddr();
	SendFlag(0x01);//命令包标识
	SendLength(0x03);
	Sendcmd(0x1d);
	temp = 0x01+0x03+0x1d;
	SendCheck(temp);
	memset(data_array,0,sizeof(data_array));
	data_array_index = 0;	
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"PS_ValidTempleteNum\r\n");//通讯成功
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
	//	printf("\r\n有效指纹个数=%d",(data[10]<<8)+data[11]);
	//}
	//else
	//	printf("\r\n%s",EnsureMessage(ensure));
	return ensure;
}
//与AS608握手 PS_HandShake
//参数: PS_Addr地址指针
//说明: 模块返新地址（正确地址）	
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
	if(USART2_RX_STA&0X8000)//接收到数据
	{		
		if(//判断是不是模块返回的应答包				
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
		if(data_array[0]==0xEF && data_array[1]==0x01 && data_array[6]==0x07)//判断是不是模块返回的应答包	
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
//模块应答包确认码信息解析
//功能：解析确认码错误信息返回信息
//参数: ensure

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
			p="ENSURE_FINGER_WEAK";break;//指纹图像太干、太淡而生不成特征
		case  ENSURE_FINGER_VAGUE:
			p="ENSURE_FINGER_VAGUE";break;//指纹图像太湿、太糊而生不成特征
		case  ENSURE_FINGER_MESS:
			p="ENSURE_FINGER_MESS";break;//指纹图像太乱而生不成特征
		case  ENSURE_FINGER_SMALL:
			p="ENSURE_FINGER_SMALL";break;//指纹图像正常，但特征点太少（或面积太小）而生不成特征
		case  ENSURE_NOT_MATCHING:
			p="ENSURE_NOT_MATCHING";break;//指纹不匹配
		case  ENSURE_NOT_FINDING:
			p="ENSURE_NOT_FINDING";break;//没搜索到指纹
		case  ENSURE_FAIL_MERGE:
			p="ENSURE_FAIL_MERGE";break;//特征合并失败
		case  ENSURE_OVERRANGE:
			p="ENSURE_OVERRANGE";//访问指纹库时地址序号超出指纹库范围
		case  ENSURE_FAIL_DEL:
			p="ENSURE_FAIL_DEL";break;//删除模板失败
		case  ENSURE_FAIL_EMPTY:
			p="ENSURE_FAIL_EMPTY";break;//清空指纹库失败	
		case  ENSURE_FAIL_IMAGE:
			p="ENSURE_FAIL_IMAGE";break;//缓冲区内没有有效原始图而生不成图像
		case  ENSURE_FLASH_ERROR:
			p="ENSURE_FLASH_ERROR";break;//读写 FLASH 出错
		case  ENSURE_UNDEFINED:
			p="ENSURE_UNDEFINED";break;//未定义错误
		case  ENSURE_ERROR_REG:
			p="ENSURE_ERROR_REG";break;//无效寄存器号
		case  ENSURE_REG_ERROR:
			p="ENSURE_REG_ERROR";break;//寄存器设定内容错误
		case  ENSURE_REG_PAGE:
			p="ENSURE_REG_PAGE";break;//记事本页码指定错误
		case  ENSURE_FINGER_FULL:
			p="ENSURE_FINGER_FULL";break;//指纹库满
		case  ENSURE_ADDRESS_ERR:
			p="ENSURE_ADDRESS_ERR";break;//地址错误
		default :
			p="ENSURE_ERROR_CODE";break;//模块返回确认码有误
	}
 return p;	
}


//显示确认码错误信息
void ShowErrMessage(uint8_t ensure)
{
	//LCD_Fill(0,120,lcddev.width,160,WHITE);
	//Show_Str_Mid(0,120,(u8*)EnsureMessage(ensure),16,240);
	//printf("%s\r\n",(uint8_t*)EnsureMessage(ensure));//通讯成功
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
		SEGGER_RTT_printf(0,"%s\r\n",(uint8_t*)EnsureMessage(ensure));//通讯成功
	#endif	
	ErrMessage  = ensure;
}
//录指纹
uint16_t Add_FR(uint16_t ValidN)
{
	uint8_t i=0, ensure ,processnum=0;
	uint16_t ID;
	//uint16_t ValidN;//模块内有效模板个数
	while(1)
	{
		switch (processnum)
		{
			case 0:
				i++;
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"Pleas touch finger!",16,240);//请按手指
				//printf("Pleas touch finger!\r\n");//请按手指
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
					SEGGER_RTT_printf(0,"Pleas touch finger!\r\n");//请按手指
				#endif				
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(CharBuffer1);//生成特征
					if(ensure==0x00)
					{
						//LCD_Fill(0,120,lcddev.width,160,WHITE);
						//Show_Str_Mid(0,120,"Fingerprint correct",16,240);//指纹正确
						//printf("Fingerprint correct!\r\n");//指纹正确
						#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
							SEGGER_RTT_printf(0,"Fingerprint correct!\r\n");//指纹正确
						#endif							
						i=0;
						processnum=1;//跳到第二步						
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
				//Show_Str_Mid(0,100,"Pleas touch finger again!",16,240);//再按一次手指
				//printf("Pleas touch finger again!\r\n");//再按一次手指
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
					SEGGER_RTT_printf(0,"Pleas touch finger again!\r\n");//再按一次手指
				#endif					
				ensure=PS_GetImage();
				if(ensure==0x00) 
				{
					ensure=PS_GenChar(CharBuffer2);//生成特征			
					if(ensure==0x00)
					{
						//LCD_Fill(0,120,lcddev.width,160,WHITE);
						//Show_Str_Mid(0,120,"Fingerprint correct",16,240);//指纹正确
						//printf("Fingerprint correct\r\n");//指纹正确
						#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
							SEGGER_RTT_printf(0,"Fingerprint correct!\r\n");//指纹正确
						#endif							
						i=0;
						processnum=2;//跳到第三步
					}
					else 
						ShowErrMessage(ensure);	
				}
				else 
					ShowErrMessage(ensure);		
				break;

			case 2:
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"Compare twice fingerprint",16,240);//对比两次指纹
				//printf("Compare twice fingerprint!\r\n");//对比两次指纹
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
					SEGGER_RTT_printf(0,"Compare twice fingerprint!\r\n");//对比两次指纹
				#endif					
				ensure=PS_Match();
				if(ensure==0x00) 
				{
					//LCD_Fill(0,120,lcddev.width,160,WHITE);
					//Show_Str_Mid(0,120,"Twice fingerprint are same",16,240);//两次指纹是一样的
					//printf("Twice fingerprint are same!\r\n");//两次指纹是一样的
					#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
						SEGGER_RTT_printf(0,"Twice fingerprint are same!\r\n");//两次指纹是一样的
					#endif					
					processnum=3;//跳到第四步
				}
				else 
				{
					//LCD_Fill(0,100,lcddev.width,160,WHITE);
					//Show_Str_Mid(0,100,"Compare fail,pleas touch again!",16,240);//对比失败，请重新按手指
					//printf("Compare fail,pleas touch again!\r\n");//对比失败，请重新按手指
					#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
						SEGGER_RTT_printf(0,"Compare fail,pleas touch again!\r\n");//对比失败，请重新按手指
					#endif						
					ShowErrMessage(ensure);
					i=0;
					processnum=0;//跳回第一步		
				}
				nrf_delay_ms(1000);//delay_ms(1000);
				break;

			case 3:
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"Generate fingerprint template",16,240);//产生一个指纹模板
				//printf("Generate fingerprint template!\r\n");//产生一个指纹模板
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
					SEGGER_RTT_printf(0,"Generate fingerprint template!\r\n");//产生一个指纹模板
				#endif					
				ensure=PS_RegModel();
				if(ensure==0x00) 
				{
					//LCD_Fill(0,120,lcddev.width,160,WHITE);
					//Show_Str_Mid(0,120,"Generate fingerprint success",16,240);//生成指纹模板成功
					//printf("Generate fingerprint success!\r\n");//生成指纹模板成功
					#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
						SEGGER_RTT_printf(0,"Generate fingerprint success!\r\n");//生成指纹模板成功
					#endif						
					processnum=4;//跳到第五步
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
				//Show_Str_Mid(0,100,"Intput ID and save with ENTER!",16,240);//输入ID并按“Enter”保存
				//Show_Str_Mid(0,120,"0=< ID <=299",16,240);	
				//printf("Intput ID and save with ENTER!\r\n");//输入ID并按“Enter”保存
				//printf("0=< ID <=299\r\n");
				//do
				//	ID=GET_NUM();
				//while(!(ID<300));//输入DI必须小于300
				#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
					SEGGER_RTT_printf(0,"Intput ID and save with ENTER!\r\n");//输入ID并按“Enter”保存
				#endif					
				ensure=PS_StoreChar(CharBuffer2,ID);//储存模板
				if(ensure==0x00) 
				{			
					//LCD_Fill(0,100,lcddev.width,160,WHITE);					
					//Show_Str_Mid(0,120,"Add fingerprint success!!!",16,240);//添加指纹成功
					//printf("Add fingerprint success!!!\r\n");//添加指纹成功
					PS_ValidTempleteNum(&ValidN);//读库指纹个数
					#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
						SEGGER_RTT_printf(0,"Add fingerprint success!!!\r\n");//添加指纹成功
						SEGGER_RTT_printf(0,"%d\r\n",ValidN);//显示剩余指纹个数
					#endif						
					//printf("%d\r\n",AS608Para.PS_max-ValidN);//显示剩余指纹个数
					//LCD_ShowNum(80,80,AS608Para.PS_max-ValidN,3,16);//显示剩余指纹个数
					nrf_delay_ms(1500);//delay_ms(1500);//延时后清除显示	
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
		if(i==5)//超过5次没有按手指则退出
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

//刷指纹
uint8_t press_FR(SearchResult *seach)
{
	uint8_t ensure;
	//char *str;
	SearchResult seachtmp;
	ensure=PS_GetImage();
	if(ensure==0x00)//获取图像成功 
	{	
		ensure=PS_GenChar(CharBuffer1);
		if(ensure==0x00) //生成特征成功
		{		
			ensure=PS_HighSpeedSearch(CharBuffer1, 0, 300, &seachtmp);
			if(ensure==0x00)//搜索成功
			{				
				//LCD_Fill(0,100,lcddev.width,160,WHITE);
				//Show_Str_Mid(0,100,"Search fingerprint success",16,240);//搜索指纹成功				
				//printf("Search fingerprint success\r\n");//搜索指纹成功		
				//sprintf(str,"Match ID:%d  Match score:%d",seach.pageID,seach.mathscore);//显示匹配指纹的ID和分数
				//printf("%s\r\n",(uint8_t*)str);//显示匹配指纹的ID和分数
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
				//sprintf(str,"Match ID:%d  Match score:%d",seach->pageID,seach->mathscore);//显示匹配指纹的ID和分数
				//SEGGER_RTT_printf(0,"%s\r\n",(uint8_t*)str);//显示匹配指纹的ID和分数
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
		nrf_delay_ms(1000); //delay_ms(1000);//延时后清除显示
		//LCD_Fill(0,100,lcddev.width,160,WHITE);
	}
	#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
	SEGGER_RTT_printf(0,"Search fingerprint exit\r\n");//搜索指纹成功		
	#endif	
	ShowErrMessage(ensure);		
	//printf("Search fingerprint exit\r\n");//搜索指纹成功		
	return ensure;	
}

//删除指纹
uint8_t Del_FR(uint16_t ValidN)
{
	uint8_t  ensure;
	uint16_t num;
	//LCD_Fill(0,100,lcddev.width,160,WHITE);
	//Show_Str_Mid(0,100,"Delete fingerprint",16,240);//显示删除指纹
	//Show_Str_Mid(0,120,"Input ID and touch Enter!",16,240);//显示输入ID并按下“Enter”
	//Show_Str_Mid(0,140,"0=< ID <=299",16,240);
	nrf_delay_ms(50);//delay_ms(50);
	//AS608_load_keyboard(0,170,(u8**)kbd_delFR);
	num = 0xFFFF;//GET_NUM();//获取返回的数值
	if(num==0xFFFF)
	{
		return 0; //返回主页面
	}
	else if(num==0xFF00)
	{
		ensure=PS_Empty();//清空指纹库
	}
	else 
	{
		ensure=PS_DeletChar(num,1);//删除单个指纹
	}
	if(ensure==0)
	{
		//LCD_Fill(0,120,lcddev.width,160,WHITE);
		//Show_Str_Mid(0,140,"Delete fingerprint success!!!",16,240);//删除指纹成功		
	}
	else
	{
		ShowErrMessage(ensure);	
	}
	nrf_delay_ms(1500);	//delay_ms(1500);//延时后清除显示
	PS_ValidTempleteNum(&ValidN);//读库指纹个数
	//LCD_ShowNum(80,80,AS608Para.PS_max-ValidN,3,16);//显示剩余指纹个数
	//MENU:	
	//LCD_Fill(0,100,lcddev.width,160,WHITE);
	nrf_delay_ms(50);//delay_ms(50);
	//AS608_load_keyboard(0,170,(u8**)kbd_menu);
	return ensure;
}



