#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include "nrf_gpio.h"



#define OLED_DC_SET    nrf_gpio_pin_set(OLED_DC)
#define OLED_DC_CLEAR  nrf_gpio_pin_clear(OLED_DC)

#define OLED_RST_SET    nrf_gpio_pin_set(OLED_RST)
#define OLED_RST_CLEAR  nrf_gpio_pin_clear(OLED_RST)

#define OLED_CS_SET    nrf_gpio_pin_set(OLED_CS)
#define OLED_CS_CLEAR  nrf_gpio_pin_clear(OLED_CS)

#define OLED_WR_CMD     0
#define OLED_WR_DAT     1


#define SIZE 16

#define BLACK 0
#define WHITE 1
#define INVERSE 2

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_SETSTARTLINE 0x40

#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A

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
extern  void LED_Printint(uint8_t ucIdxX, uint8_t ucIdxY, int cData);
extern  void LCD_P16x16Ch(uint8_t x, uint8_t y, uint8_t N);
extern  void OLED_DrawBMP(uint8_t x0, uint8_t y0,uint8_t x1, uint8_t y1,const uint8_t BMP[]);
extern  void LED_P20x40Char(uint8_t ucIdxX, uint8_t ucIdxY, uint8_t ucData,uint8_t ucFlag);
extern  void LED_P20x40String(uint8_t ucIdxX, uint8_t ucIdxY, char ch[], uint8_t ucFlag);
extern  void LED_P20x40Short(uint8_t ucIdxX, uint8_t ucIdxY, int16_t shortdata, uint8_t ucFlag);
extern  void init_spi_master(void);
extern  void oled_battery_statues_display(int capacity);
extern  void oled_signal_display(int signal_state);
#endif

