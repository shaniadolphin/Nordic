#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include "nrf_gpio.h"


#define  OLED_SCL    26  //OLED时钟
#define  OLED_SDA    25  //OLED数据
#define  OLED_DC     30   //OLED命令/数据选择 H=命令 L=数据
#define  OLED_CS     28   //OLED片选
#define  OLED_RST    29   //OLED复位
#define  OLED_EN	 31

#define OLED_DC_SET    nrf_gpio_pin_set(OLED_DC)
#define OLED_DC_CLEAR  nrf_gpio_pin_clear(OLED_DC)

#define OLED_RST_SET    nrf_gpio_pin_set(OLED_RST)
#define OLED_RST_CLEAR  nrf_gpio_pin_clear(OLED_RST)

#define OLED_CS_SET    nrf_gpio_pin_set(OLED_CS)
#define OLED_CS_CLEAR  nrf_gpio_pin_clear(OLED_CS)

#define OLED_WR_CMD     0
#define OLED_WR_DAT     1


#define SIZE 16



#define XLevelL        0x00
#define XLevelH        0x10
#define XLevel         ((XLevelH&0x0F)*16+XLevelL)
#define Max_Column     128
#define Max_Row        64
#define Brightness     0xCF 
#define X_WIDTH        128
#define Y_WIDTH        64



extern  void DelayMS(uint32_t dly);
extern  void OLED_Init(void);
extern  void OLED_Fill(uint8_t bmp_dat);
extern  void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr);
extern  void OLED_Set_Pos(uint8_t x, uint8_t y);
extern  void LCD_P8x16Str(uint8_t x, uint8_t y,char ch[]);
extern  void LCD_P16x16Ch(uint8_t x, uint8_t y, uint8_t N);
extern  void OLED_DrawBMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,const uint8_t BMP[]);
extern  void hal_spi_init(void);
#endif

