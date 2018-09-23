#include "..\oled\oled.h"
#include "..\oled\OLEDFONT.h"
#include "..\oled\bmp.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_drv_common.h"
#include "nrf_drv_spi.h"
#include "..\prink\printk.h"
//#include "nrf_drv_config.h"
#include "..\pca10040\s132\config\sdk_config.h"

#define SSD1306_LCDHEIGHT 	64
#define SSD1306_LCDWIDTH  	128
#define BUFFER_SIZE 		(SSD1306_LCDWIDTH / 8)
//static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE); 

static volatile bool spi_xfer_done;  //SPI数据传输完成标志
static uint8_t buffer[SSD1306_LCDHEIGHT * SSD1306_LCDWIDTH / 8];
typedef struct ArrayList
{
	uint8_t buffer[128 / 8];
}ArrayList_type;

ArrayList_type MyArrayList[2];

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
}lcd_init_cmd_t;

static const lcd_init_cmd_t oled_init_cmds[]={
	{0xae,{0x00},0},//--turn off oled panel
    {0x00,{0x00},0},//---set low column address
    {0x10,{0x00},0},//---set high column address
    {0x40,{0x00},0},//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    {0x81,{0x00},0},//--set contrast control register
    {0xcf,{0x00},0},// Set SEG Output Current Brightness

    {0xa1,{0x00},0},//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    {0xc8,{0x00},0},//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    {0xa6,{0x00},0},//--set normal display
    {0xa8,{0x00},0},//--set multiplex ratio(1 to 64)
    {0x3f,{0x00},0},//--1/64 duty
    {0xd3,{0x00},0},//-set display offset    Shift Mapping RAM Counter (0x00~0x3F)
    {0x00,{0x00},0},//-not offset

    {0xd5,{0x00},0},//--set display clock divide ratio/oscillator frequency
    {0x80,{0x00},0},//--set divide ratio, Set Clock as 100 Frames/Sec
    {0xd9,{0x00},0},//--set pre-charge period

    {0xf1,{0x00},0},
    {0xda,{0x00},0},
    {0x12,{0x00},0},
    {0xdb,{0x00},0},

    {0x40,{0x00},0},

    {0x20,{0x00},0},
    {0x02,{0x00},0},
    {0x8d,{0x00},0},
    {0x14,{0x00},0},
    {0xa4,{0x00},0},
    {0xa6,{0x00},0},
    {0xaf,{0x00},0},
	{0, {0}, 0xff},
};

uint8_t spi_transfer(uint8_t data)
{ 
	uint8_t ret; 
	//printk("33");
	NRF_SPI0->TXD = data; 
	while( NRF_SPI0->EVENTS_READY == 0 ); //等待传输结束 
	NRF_SPI0->EVENTS_READY = 0; 
	ret = NRF_SPI0->RXD; 
	//printk("44");
	return ret; 
} 

void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    
	//printk(" Received: 0x%02X\r\n",m_rx_buf[0]);
	switch(p_event -> type)
	{
		case NRF_DRV_SPI_EVENT_DONE:
			spi_xfer_done = true;
			//printk(".");
		break;
		default:
			//printk("+");
		break;
	}
}

void init_spi_master(void)
{ 
#if 1
	//按手册要求设置
	nrf_gpio_cfg_input(SPI0_CONFIG_MISO_PIN, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_output(SPI0_CONFIG_MOSI_PIN);
	nrf_gpio_cfg_output(SPI0_CONFIG_SCK_PIN);	
	nrf_gpio_cfg_output(OLED_CS);
	nrf_gpio_cfg_output(OLED_EN);
	nrf_gpio_pin_set(OLED_CS);
	
	//flash芯片要求在MSB先传输，并且在第一个上升沿采样。所以设置下。 CPOL和CPHA都设置成1也是可以的,这里都设置成0了。
	NRF_SPI0->CONFIG = (0 << 0) | (0 << 1) | (0 << 2);//SCK active high, sample on leading edge of clock.
	NRF_SPI0->FREQUENCY = 0x80000000;
	NRF_SPI0->PSELSCK = SPI0_CONFIG_SCK_PIN;
	NRF_SPI0->PSELMOSI = SPI0_CONFIG_MOSI_PIN;
	NRF_SPI0->PSELMISO = SPI0_CONFIG_MISO_PIN;	
	NRF_SPI0->ENABLE = 1;
#else
	uint32_t err_code;
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
	spi_config.mosi_pin = SPI0_CONFIG_MOSI_PIN;
	spi_config.sck_pin = SPI0_CONFIG_SCK_PIN;
	spi_config.miso_pin = SPI0_CONFIG_MISO_PIN;
	spi_config.frequency = NRF_DRV_SPI_FREQ_4M;			// Default 4M
	spi_config.mode = NRF_DRV_SPI_MODE_0;				// Default mode 0 
	spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST; // Matches Default.
	spi_config.irq_priority = SPI0_CONFIG_IRQ_PRIORITY;
	
	err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler);
	printk("nrf_drv_spi_init %d.\r\n",err_code);
	APP_ERROR_CHECK(err_code);	
#endif
#if 0
	NRF_SPIM0->FREQUENCY = 0x80000000;  
	NRF_SPIM0->TXD.MAXCNT = BUFFER_SIZE;
	NRF_SPIM0->TXD.PTR = &MyArrayList;
#endif
}


/******************************************************************************
 * 描  述 : 向OLED写入1字节数据
 * 入  参 : dat:数据；mode：=0：写入命令，=1：写入数据
 * 返回值 : 无
 ******************************************************************************/
void OLED_WrByte(uint8_t dat, uint8_t mode)     
{
	//uint8_t err;
	nrf_gpio_pin_clear(OLED_CS);
	
	if(mode == OLED_WR_CMD)OLED_DC_CLEAR;
	else OLED_DC_SET;
#if 1	
	spi_transfer(dat);
#else	
	//nrf_drv_spi_transfer(&spi, &dat, 1, &err, 1);
	spi_xfer_done = false;
	spi_tx_buf[0] = dat;
	nrf_drv_spi_transfer(&spi, spi_tx_buf, 1, spi_rx_buf, 1);
	while(spi_xfer_done == false)
	{
		retry++;
		if(retry>20000)
		{
			nrf_gpio_pin_set(OLED_CS);
			printk("-");
			return;
		}
	}
#endif	
	nrf_gpio_pin_set(OLED_CS);
}

/******************************************************************************
 * 描  述 : 向OLED写入多字节数据
 * 入  参 : dat:数据；mode：=0：写入命令，=1：写入数据
 * 返回值 : 无
 ******************************************************************************/
void OLED_WrBuff(const uint8_t *dat, uint8_t len)     
{
	nrf_gpio_pin_clear(OLED_CS);
	OLED_DC_SET;
	if(len == 0)return;
	else
	{
		for(int i = 0;i < len; i++)
			spi_transfer(*(dat + i));		
	}
	nrf_gpio_pin_set(OLED_CS);
}
/******************************************************************************
 * 描  述 : 设置坐标
 * 入  参 : x：x坐标；y：y坐标
 * 返回值 : 无
 ******************************************************************************/
void OLED_Set_Pos(uint8_t x, uint8_t y) 
{ 
    OLED_WrByte((0xb0+y),OLED_WR_CMD);
    OLED_WrByte(((x&0xf0)>>4)|0x10,OLED_WR_CMD);
    OLED_WrByte((x&0x0f)|0x01,OLED_WR_CMD); 
} 
/******************************************************************************
 * 描  述 : LCD初始化
 * 入  参 : 无
 * 返回值 : 无
 ******************************************************************************/
void OLED_Fill(uint8_t dat) 
{
    uint8_t y,x;
    for(y=0;y<8;y++)
    {
        OLED_WrByte(0xb0+y,OLED_WR_CMD);//设置页地址（0~7）
        OLED_WrByte(0x02,OLED_WR_CMD); //设置显示位置—列低地址
        OLED_WrByte(0x10,OLED_WR_CMD); //设置显示位置—列高地址
        for(x=0; x<X_WIDTH; x++)
            OLED_WrByte(dat,OLED_WR_DAT);
    }
}

/******************************************************************************
 * 描  述 : 指定位置显示一个字符
 * 入  参 : x:列0~127；y:页地址0~7；
 * 返回值 : 无
 ******************************************************************************/
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr)
{      	
	uint8_t c=0,i=0;	

	c = chr-' ';//得到偏移后的值			
	if(x > Max_Column-1)
	{
		x = 0;
		y = y+2;
	}

	OLED_Set_Pos(x,y);	
	for(i=0; i<8; i++)OLED_WrByte(F8X16[c*16+i],OLED_WR_DAT); 
	OLED_Set_Pos(x,y+1);
	for(i=0;i<8;i++)OLED_WrByte(F8X16[c*16+i+8],OLED_WR_DAT);
}
/*****************************************************************************
 函 数 名  : LED_P6x8Char
 功能描述  : 显示一个6x8标准ASCII字符
 输入参数  : UCHAR8 ucIdxX  显示的横坐标0~122
             UCHAR8 ucIdxY  页范围0～7
             UCHAR8 ucData  显示的字符
 输出参数  : NONE
 返 回 值  : NONE
*****************************************************************************/
void LED_P20x40Char(uint8_t ucIdxX, uint8_t ucIdxY, uint8_t ucData,uint8_t ucFlag)
{
    uint8_t i, ucDataTmp;     
    //ucIdxY <<= 1;   
    ucDataTmp = ucData - 0;

    OLED_Set_Pos(ucIdxX, ucIdxY);   
    
    for(i = 0; i < 20; i++) 
    {
        if(ucFlag)OLED_WrByte(F20X40_NUM[(ucDataTmp * 100) + i],OLED_WR_DAT); //(20 * 40 / 8)
        else OLED_WrByte(~(F20X40_NUM[(ucDataTmp * 100) + i]),OLED_WR_DAT); //(20 * 40 / 8)
    }
    
    OLED_Set_Pos(ucIdxX, ucIdxY + 1);   
    
    for(i = 0; i < 20; i++) 
    {
        if(ucFlag)OLED_WrByte(F20X40_NUM[(ucDataTmp *100) + i + 20],OLED_WR_DAT); 
        else OLED_WrByte(~(F20X40_NUM[(ucDataTmp *100) + i + 20]),OLED_WR_DAT); 
    }
    
    OLED_Set_Pos(ucIdxX, ucIdxY + 2);      
    for(i = 0; i < 20; i++) 
    {
        if(ucFlag)OLED_WrByte(F20X40_NUM[(ucDataTmp *100) + i + 40],OLED_WR_DAT); 
        else OLED_WrByte(~(F20X40_NUM[(ucDataTmp *100) + i + 40]),OLED_WR_DAT); 
    } 
    
    OLED_Set_Pos(ucIdxX, ucIdxY + 3);       
    for(i = 0; i < 20; i++) 
    {
        if(ucFlag)OLED_WrByte(F20X40_NUM[(ucDataTmp *100) + i + 60],OLED_WR_DAT); 
        else OLED_WrByte(~(F20X40_NUM[(ucDataTmp *100) + i + 60]),OLED_WR_DAT); 
    } 
    
    OLED_Set_Pos(ucIdxX, ucIdxY + 4);   
    for(i = 0; i < 20; i++) 
    {
        if(ucFlag)OLED_WrByte(F20X40_NUM[(ucDataTmp *100) + i + 80],OLED_WR_DAT); 
        else OLED_WrByte(~(F20X40_NUM[(ucDataTmp *100) + i + 80]),OLED_WR_DAT); 
    }     
    //ucIdxX += 8;
}
void LED_P20x40String(uint8_t ucIdxX, uint8_t ucIdxY, char ch[], uint8_t ucFlag)
{
	uint8_t j=0;
    
	while (ch[j] != '\0')
	{    
		LED_P20x40Char(ucIdxX + 0 * 20, ucIdxY, ch[j], ucFlag);
		j++;
	}	
}
void LED_P20x40Short(uint8_t ucIdxX, uint8_t ucIdxY, int16_t shortdata, uint8_t ucFlag)
{
	char data1[10];
	sprintf(data1, "%6d", shortdata);    
	LED_P20x40String(ucIdxX, ucIdxY, data1, ucFlag);
}
/******************************************************************************
 * 描  述 : 指定位置显示一个字符
 * 入  参 : x:列0~127；y:页地址0~7；
 * 返回值 : 无
 ******************************************************************************/
void LCD_P8x16Ch(uint8_t x,uint8_t y,uint8_t chr)
{      	
  uint8_t c=0,i=0;	
  
  c = chr-' ';//得到偏移后的值			
  if(x > Max_Column-1)
  {
    x = 0;
    y = y+2;
  }
  OLED_Set_Pos(x,y);	
  for(i=0; i<8; i++)OLED_WrByte(F8X16[c*16+i],OLED_WR_DAT); 
  OLED_Set_Pos(x,y+1);
  for(i=0;i<8;i++)OLED_WrByte(F8X16[c*16+i+8],OLED_WR_DAT);
}
/******************************************************************************
 * 描  述 : 显示8*16一组标准ASCII字符串
 * 入  参 : x:列0~127；y:页地址0~7；
 * 返回值 : 无
 ******************************************************************************/
void LCD_P8x16Str(uint8_t x, uint8_t y,char ch[])
{
  uint8_t c=0,i=0,j=0;
    
  while (ch[j] != '\0')
  {    
    c = ch[j]-32;
    if(x>120){x=0;y++;}
    OLED_Set_Pos(x,y);    
    for(i=0; i<8; i++)OLED_WrByte(F8X16[c*16+i],OLED_WR_DAT);          
    OLED_Set_Pos(x,y+1);    
    for(i=0;i<8;i++) OLED_WrByte(F8X16[c*16+i+8],OLED_WR_DAT);        
    x += 8;
    j++;
  }
}
/******************************************************************************
 * 描  述 : 显示汉字
 * 入  参 : x:列0~127；y:页地址0~7；
 * 返回值 : 无
 ******************************************************************************/
void LCD_P16x16Ch(uint8_t x, uint8_t y, uint8_t N)
{
    uint8_t wm=0;
    unsigned int adder=32*N;        
    OLED_Set_Pos(x , y);
    for(wm = 0;wm < 16;wm++)               
    {
        OLED_WrByte(F16x16[adder],OLED_WR_DAT);    
        adder += 1;
    }      
    OLED_Set_Pos(x,y + 1); 
    for(wm = 0;wm < 16;wm++)         
    {
        OLED_WrByte(F16x16[adder],OLED_WR_DAT);
        adder += 1;
    }           
}

void LED_Printint(uint8_t ucIdxX, uint8_t ucIdxY, int cData)
{
	char data1[10];
	if(cData > 99999)cData = 99999;
	if(cData < -99999)cData = -99999;
	sprintf(data1, "%8d", cData);    
	LCD_P8x16Str(ucIdxX, ucIdxY, data1);
    return;
}
/******************************************************************************
 * 描  述 : 显示BMP图片128×64
 * 入  参 : 起始点坐标(x,y),x的范围0～127，y为页的范围0～7
 * 返回值 : 无
 ******************************************************************************/
void OLED_DrawBMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,const uint8_t BMP[])
{ 	
	uint16_t j=0;
	uint8_t x,y;

	if((y1 % 8) == 0) y = y1/8;      
	else y = y1/8+1;
	for(y=y0;y<y1;y++)
	{
		OLED_Set_Pos(x0,y);
		for(x=x0; x<x0+x1; x++)
		{      
			OLED_WrByte(BMP[j++],OLED_WR_DAT);	    	
		}
	}
} 

void oled_battery_statues_display(int capacity)
{	
	switch(capacity)
	{
		case 0://充电图标是24*16像素的  每个y1都是代表8个像素的;
			OLED_DrawBMP(100, 0, 24, 2, Battery_low);
		break;
		case 1:
			OLED_DrawBMP(100, 0, 24, 2, Battery_1_4);
		break;
		case 2:
			OLED_DrawBMP(100, 0, 24, 2, Battery_2_4);
		break;
		case 3:
			OLED_DrawBMP(100, 0, 24, 2, Battery_3_4);
		break;
		case 4:
			OLED_DrawBMP(100, 0, 24, 2, Battery_full);
		break;
		case 5:
			OLED_DrawBMP(100, 0, 24, 2, Battery_charge);
		break;
		default:
			OLED_DrawBMP(100, 0, 24, 2, Battery_full);
		break;
	}
}

//读取GSM 模块的信号 判断当前信号状态
void oled_signal_display(int signal_state)
{
	switch(signal_state)
	{
		case 0:
			OLED_DrawBMP(0, 0, 25, 2, Signal_no);
		break;
		case 1:
			OLED_DrawBMP(0, 0, 25, 2, Signal_1_4);
		break;
		case 2:
			OLED_DrawBMP(0, 0, 25, 2, Signal_2_4);
		break;
		case 3:
			OLED_DrawBMP(0, 0, 25, 2, Signal_3_4);
		break;
		case 4:
			OLED_DrawBMP(0, 0, 25, 2, Signal_full);
		break;
		default: 
			OLED_DrawBMP(0, 0, 25, 2, Signal_full);
		break;
	}
}

/******************************************************************************
 * 描  述 : LCD初始化
 * 入  参 : 无
 * 返回值 : 无
 ******************************************************************************/
void OLED_Init(void)     
{
	int cmd=0;
    const lcd_init_cmd_t *lcd_init_cmds;
	nrf_gpio_cfg_output(OLED_CS);
	nrf_gpio_cfg_output(OLED_DC);
	nrf_gpio_cfg_output(OLED_RST);
	nrf_gpio_cfg_output(OLED_EN);
	nrf_gpio_pin_set(OLED_EN);
	nrf_gpio_pin_clear(OLED_RST);	
	nrf_gpio_pin_set(OLED_CS);
	nrf_gpio_pin_set(OLED_DC);
	//OLED_RST_CLEAR;
	nrf_delay_ms(100);
	init_spi_master();
	nrf_gpio_pin_set(OLED_RST);
	//OLED_RST_SET;
	//nrf_gpio_pin_clear(OLED_CS);
	//nrf_delay_ms(10);
	//OLED_CS_SET;
	//nrf_delay_ms(10);
	//OLED_CS_CLEAR;
	
    nrf_delay_ms(100);
#if 0
    OLED_WrByte(0xae,OLED_WR_CMD);//--turn off oled panel
	OLED_WrByte(0x00,OLED_WR_CMD);//---set low column address
    OLED_WrByte(0x10,OLED_WR_CMD);//---set high column address
    OLED_WrByte(0x40,OLED_WR_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WrByte(0x81,OLED_WR_CMD);//--set contrast control register
    OLED_WrByte(0xcf,OLED_WR_CMD);// Set SEG Output Current Brightness
	
    OLED_WrByte(0xa1,OLED_WR_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WrByte(0xc8,OLED_WR_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WrByte(0xa6,OLED_WR_CMD);//--set normal display
    OLED_WrByte(0xa8,OLED_WR_CMD);//--set multiplex ratio(1 to 64)
    OLED_WrByte(0x3f,OLED_WR_CMD);//--1/64 duty
	
    OLED_WrByte(0xd3,OLED_WR_CMD);//-set display offset    Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WrByte(0x00,OLED_WR_CMD);//-not offset
		
    OLED_WrByte(0xd5,OLED_WR_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WrByte(0x80,OLED_WR_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WrByte(0xd9,OLED_WR_CMD);//--set pre-charge period
		
    OLED_WrByte(0xf1,OLED_WR_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
		//OLED_WrByte(0x22,OLED_WR_CMD);
    OLED_WrByte(0xda,OLED_WR_CMD);//--set com pins hardware configuration
    OLED_WrByte(0x12,OLED_WR_CMD);
    OLED_WrByte(0xdb,OLED_WR_CMD);//--set vcomh
		
    OLED_WrByte(0x40,OLED_WR_CMD);//Set VCOM Deselect Level
		
    OLED_WrByte(0x20,OLED_WR_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WrByte(0x02,OLED_WR_CMD);//
    OLED_WrByte(0x8d,OLED_WR_CMD);//--set Charge Pump enable/disable
    OLED_WrByte(0x14,OLED_WR_CMD);//--set(0x10) disable
    OLED_WrByte(0xa4,OLED_WR_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WrByte(0xa6,OLED_WR_CMD);// Disable Inverse Display On (0xa6/a7) 
    OLED_WrByte(0xaf,OLED_WR_CMD);//--turn on oled panel
#else
    lcd_init_cmds = oled_init_cmds;	
    //Send all the commands
    while (lcd_init_cmds[cmd].databytes != 0xff) 
	{
        //lcd_cmd(0, lcd_init_cmds[cmd].cmd);
		OLED_WrByte(lcd_init_cmds[cmd].cmd, OLED_WR_CMD);
        OLED_WrBuff(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);
        if (lcd_init_cmds[cmd].databytes & 0x80) 
		{
            //vTaskDelay(100 / portTICK_RATE_MS);
			nrf_delay_ms(100);
        }
        cmd++;
    }
#endif
    OLED_Fill(0x00);  //初始清屏
    //LCD_P8x16Str(0,6,"program start");
	OLED_Set_Pos(0,0);     
}

//Initialize the display
void lcd_init(void)
{
    int cmd=0;
    const lcd_init_cmd_t* lcd_init_cmds;

    //const lcd_init_cmd_t *lcd_init_cmds;
	nrf_gpio_cfg_output(OLED_CS);
	nrf_gpio_cfg_output(OLED_DC);
	nrf_gpio_cfg_output(OLED_RST);
	nrf_gpio_cfg_output(OLED_EN);
	nrf_gpio_pin_set(OLED_EN);
	nrf_gpio_pin_clear(OLED_RST);	
	nrf_gpio_pin_set(OLED_CS);
	nrf_gpio_pin_set(OLED_DC);
	nrf_delay_ms(100);
	init_spi_master();
	nrf_gpio_pin_set(OLED_RST);
    nrf_delay_ms(100);

    lcd_init_cmds = oled_init_cmds;	
    //Send all the commands
    while (lcd_init_cmds[cmd].databytes != 0xff) 
	{
        //lcd_cmd(0, lcd_init_cmds[cmd].cmd);
		OLED_WrByte(lcd_init_cmds[cmd].cmd, OLED_WR_CMD);
        OLED_WrBuff(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);
        if (lcd_init_cmds[cmd].databytes & 0x80) 
		{
            //vTaskDelay(100 / portTICK_RATE_MS);
			nrf_delay_ms(100);
        }
        cmd++;
    }
	
    OLED_Fill(0x00);  //初始清屏
	OLED_Set_Pos(0,0);   
    ///Enable backlight
    //gpio_set_level(PIN_NUM_BCKL, 0);
}


// the most basic function, set a single pixel
void SSD1306_drawPixel(int16_t x, int16_t y, uint16_t color) 
{
  if ((x < 0) || (x >= SSD1306_LCDWIDTH) || (y < 0) || (y >= SSD1306_LCDHEIGHT))
    return;
#if 0
  // check rotation, move pixel around if necessary
  switch (getRotation()) {
  case 1:
    ssd1306_swap(x, y);
    x = WIDTH - x - 1;
    break;
  case 2:
    x = WIDTH - x - 1;
    y = HEIGHT - y - 1;
    break;
  case 3:
    ssd1306_swap(x, y);
    y = HEIGHT - y - 1;
    break;
  }
#endif
  // x is which column
    int index = x+ (y/8) * SSD1306_LCDWIDTH;
    //printf("index = %d\n", index);

    switch (color)
    {
      case WHITE:   buffer[index] |=  (1 << (y&7)); break;
      case BLACK:   buffer[index] &= ~(1 << (y&7)); break;
      case INVERSE: buffer[index] ^=  (1 << (y&7)); break;
    }

}

void SSD1306_command(uint8_t dat)
{

}

void SSD1306_invertDisplay(uint8_t i) 
{
	if (i) 
	{
		SSD1306_command(SSD1306_INVERTDISPLAY);
	} 
	else 
	{
		SSD1306_command(SSD1306_NORMALDISPLAY);
	}
}
// startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306_startscrollright(uint8_t start, uint8_t stop)
{
	SSD1306_command(SSD1306_RIGHT_HORIZONTAL_SCROLL);
	SSD1306_command(0x00);
	SSD1306_command(start);
	SSD1306_command(0x00);
	SSD1306_command(stop);
	SSD1306_command(0x00);
	SSD1306_command(0xFF);
	SSD1306_command(SSD1306_ACTIVATE_SCROLL);
}
// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306_startscrollleft(uint8_t start, uint8_t stop)
{
	SSD1306_command(SSD1306_LEFT_HORIZONTAL_SCROLL);
	SSD1306_command(0x00);
	SSD1306_command(start);
	SSD1306_command(0x00);
	SSD1306_command(stop);
	SSD1306_command(0x00);
	SSD1306_command(0xFF);
	SSD1306_command(SSD1306_ACTIVATE_SCROLL);
}
// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306_startscrolldiagright(uint8_t start, uint8_t stop)
{
	SSD1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
	SSD1306_command(0x00);
	SSD1306_command(SSD1306_LCDHEIGHT);
	SSD1306_command(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
	SSD1306_command(0x00);
	SSD1306_command(start);
	SSD1306_command(0x00);
	SSD1306_command(stop);
	SSD1306_command(0x01);
	SSD1306_command(SSD1306_ACTIVATE_SCROLL);
}
// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
void SSD1306_startscrolldiagleft(uint8_t start, uint8_t stop)
{
	SSD1306_command(SSD1306_SET_VERTICAL_SCROLL_AREA);
	SSD1306_command(0x00);
	SSD1306_command(SSD1306_LCDHEIGHT);
	SSD1306_command(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
	SSD1306_command(0x00);
	SSD1306_command(start);
	SSD1306_command(0x00);
	SSD1306_command(stop);
	SSD1306_command(0x01);
	SSD1306_command(SSD1306_ACTIVATE_SCROLL);
}
void SSD1306_stopscroll(void)
{
	SSD1306_command(SSD1306_DEACTIVATE_SCROLL);
}
// Dim the display
// dim = true: display is dimmed
// dim = false: display is normal
void SSD1306_dim(bool dim) 
{
	uint8_t contrast;
	if (dim) 
	{
		contrast = 0; // Dimmed display
	} 
	else 
	{
		//if (_vccstate == SSD1306_EXTERNALVCC) 
		//{
		//	contrast = 0x9F;
		//} 
		//else 
		//{
			contrast = 0xCF;
		//}
	}
	// the range of contrast to too small to be really useful
	// it is useful to dim the display
	SSD1306_command(SSD1306_SETCONTRAST);
	SSD1306_command(contrast);
}

void SSD1306_display(void) 
{
	SSD1306_command(SSD1306_COLUMNADDR);
	SSD1306_command(0);   // Column start address (0 = reset)
	SSD1306_command(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

	SSD1306_command(SSD1306_PAGEADDR);
	SSD1306_command(0); // Page start address (0 = reset)
	//#if SSD1306_LCDHEIGHT == 64
	SSD1306_command(7); // Page end address
	//#endif
	for (uint16_t i=0; i<(SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8); i++) 
	{

	}

}

// clear everything
void SSD1306_clearDisplay(void) 
{
	memset(buffer, 0, (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8));
}
// draw a character
void Adafruit_GFX_drawChar(int16_t x, int16_t y, uint8_t ch, uint16_t color, uint16_t bgcolor) 
{

}




/*********************************END FILE*************************************/


