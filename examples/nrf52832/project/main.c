/****************************************Copyright (c)****************************************************
**                                        
**                                      
**                                        
**--------------File Info---------------------------------------------------------------------------------
** File name:			     main.c
** Last modified Date: 2016.3.27         
** Last Version:		   1.1
** Descriptions:		   使用的SDK版本-SDK_11.0.0
**						
**--------------------------------------------------------------------------------------------------------
** Created by:			FiYu
** Created date:		2015-7-1
** Version:			    1.0
** Descriptions:		12864OLED显示实验
**--------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "oled.h"
#include "OLEDFONT.h"
#include "nrf_drv_spi.h"
#include "nrf52832_peripherals.h"
#define LED_D1		19

/* 开发板中OLED显示屏占用的nRF51822管脚资源

P0.22：输出：OLED片选
P0.23：输出：OLED命令/数据选择 H=命令 L=数据
P0.24：输出：OLED复位
P0.25：输出：OLED数据
P0.26：输出：OLED时钟

*/
//注意，SPI配置的时候使用了空闲的管脚P0.11作为MISO(实际没起作用)，如果需要使用P0.11，更换MISO的管脚编号就可以了
//在“nrf_drv_config.h∥文件中更换
/**********************************************************************************************
 * 描  述 : main函数
 * 入  参 : 无
 * 返回值 : 无
 ***********************************************************************************************/ 
int main(void)
{	
	uint8_t i;
	nrf_gpio_cfg_output(LED_D1);
	//配置用于控制OLED的管脚
	//nrf_gpio_cfg_output(OLED_DC);
	//nrf_gpio_cfg_output(OLED_RST);
	//hal_spi_init();  //SPI初始化
	nrf_delay_ms(200); 
    OLED_Init();                      //oled 初始化  
	
    while (true)
    {
        //显示ACSII 使用单个字符显示函数
		    OLED_Fill(0x00);//清屏
        for(i=0; i<16 ;i++)
        {
            OLED_ShowChar(i*8,0,' '+i);
            OLED_ShowChar(i*8,2,' '+i+16);  
            OLED_ShowChar(i*8,4,' '+i+32);
            OLED_ShowChar(i*8,6,' '+i+48);    
        }
        nrf_delay_ms(2000);
        nrf_gpio_pin_set(LED_D1);
        //显示数字和字母 使用字符串显示函数
        OLED_Fill(0x00);
        LCD_P8x16Str(0,0,"0123456789");
        LCD_P8x16Str(0,2,"abcdefghijklmnop");
        LCD_P8x16Str(0,4,"ABCDEFGHIJKLMNOP");
        LCD_P8x16Str(0,6,"0123456789");
        nrf_delay_ms(2000);
		nrf_gpio_pin_clear(LED_D1);	
#if 0        
        //显示汉字
        OLED_Fill(0x00); //清屏
        for(i=0; i<6; i++)
        {
            LCD_P16x16Ch(i*16+16,2,i);    
        }
        for(i=0; i<4; i++)
        {
            LCD_P16x16Ch(i*16+32,4,i+6); 
            LCD_P16x16Ch(i*16+32,6,i+6+4);
        }
        nrf_delay_ms(2000);      
		
		    //显示图形
        OLED_Fill(0x00);     //清屏
        OLED_DrawBMP(0,0,128,8,BMP1);//显示图形
        nrf_delay_ms(3000);  //延时，方便观察OLED显示
        
        OLED_Fill(0x00); //清屏
        OLED_DrawBMP(0,0,128,8,BMP2);
        nrf_delay_ms(3000); 
#endif		
    }
}
/********************************************END FILE*******************************************/
