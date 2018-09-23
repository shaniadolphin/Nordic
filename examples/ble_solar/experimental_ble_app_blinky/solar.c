/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */
#include "app_timer.h"
#include "nrf_drv_saadc.h"
#include "nrf_saadc.h"
#include "..\prink\printk.h"
#include "..\oled\oled.h"

int32_t solar_vol;
int32_t solar_volbuf[5] = {0};
uint8_t solar_volbuf_cnt;
int32_t solar_vol_old;
int32_t solar_power = 0;
int32_t batt_vol;
int32_t char_cur;
int32_t char_cur_sum = 0;
uint8_t solar_state = 0;
uint8_t keycnt = 0;

uint8_t data_array[100];
uint32_t data_array_lasttime;

uint8_t finger_result_en = 0;

uint8_t data_array_index = 0;

uint8_t ErrMessage = 0;

int32_t battVol[5] = {0};
int32_t battCur[5] = {0};
uint8_t detCnt = 0;
int32_t tickcnt = 0;

APP_TIMER_DEF(m_volmeas_timer_id);      /**< adc采样定时器 */
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define VOLMEAS_READ_INTERVAL           APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)  /**< ADC采样间隔 100ms读取一次*/
#define SAMPLES_IN_BUFFER 				3             								/**< ADC采样数据缓存大小(字节数)  */

#define PIN_OTG_EN			7
#define PIN_SEN_IN			22
#define PIN_LED2			10
#define PIN_LED1			9
#define PIN_SOLAR_EN		12
#define PIN_SOLAR_VOL1		26
#define PIN_SOLAR_VOL2		25

typedef struct
{
    uint8_t cnt;
	uint8_t buf[20]; 
}adc_value_t;

typedef enum
{
	get_solar_default = 0,	//默认状态，太阳能电池电压小于5V的时候
    get_solar_level = 1, 	//计算电阳能最高电压,斜率为0时
	get_solar_cap = 2,		//电阳能给电容充电
	get_solar_batt = 3,		//电阳能给电池充电
}SolarWorking;

#define SOLAR_NIGHTING   5500
#define SOLAR_DAYING     8000
#define SOLAR_DEADMAX    17500

typedef struct{
	int32_t battbuf[10];	//存储历史数据
	int32_t currbuf[10];	//存储历史数据
	int32_t solarbuf[10];	//太阳能电池历史数据
	uint8_t solarbuffull;	//历史数据满标志位
	int32_t currbufsum;		//平均电流
	uint8_t bufcnt;//
	uint16_t solarcapcnt;
	uint16_t solarpwrcnt;
	int32_t solarcapvola;	//电容充电的启始电压	
	int32_t solarcapvolb;	//电容充电的结束电压
	uint8_t solarstate;
	int32_t solar_solarmax; 
	int32_t solar_capstop; 	
}Solar;

static nrf_saadc_value_t       m_buffer_pool[2][SAMPLES_IN_BUFFER];
adc_value_t adc_value;
Solar 	solardeal;
#ifdef ADC_LOWPWR
static bool m_saadc_initialized = false;
#endif
//adc_value_t adc_valu                                  /**< LED Button Service instance. */
uint8_t restart_flag  = 0;
//uint8_t myaddr[BLE_GAP_ADDR_LEN] = {0xc5,0x15,0x83,0xbc,0xd2,0x6a};
uint8_t rssi = 0;
uint8_t received_blue;

extern uint32_t ble_send_solardata(int32_t value1, int32_t value2);
void saadc_init(void);

#define     my_pow2(x)      ((x) * (x))
float Paradeal(void)
{
	float Para[2][3] = {0.0f};
	int32_t Amount = 5;
	int32_t xi = 0;
	int32_t yi = 0;
	float X[5];
	float Y[5];
	X[0] = 0.0f;
	X[1] = 1.0f;
	X[2] = 2.0f;
	X[3] = 3.0f;
	X[4] = 4.0f;
	Y[0] = (float)solardeal.solarbuf[9];
	Y[1] = (float)solardeal.solarbuf[8];
	Y[2] = (float)solardeal.solarbuf[7];
	Y[3] = (float)solardeal.solarbuf[6];
	Y[4] = (float)solardeal.solarbuf[5];
	Para[1][1] = Amount;
	for ( ; Amount > 0; Amount--, xi++, yi++)
	{
		Para[0][0] += my_pow2(X[xi]);
		Para[0][1] += (X[xi]);
		Para[0][2] += (X[xi]) * (Y[yi]);
		Para[1][2] += (Y[yi]);
	}
	
	Para[1][0] = Para[0][1];
	
	Para[0][0] -= Para[1][0] * (Para[0][1] / Para[1][1]);

	Para[0][2] -= Para[1][2] * (Para[0][1] / Para[1][1]);
	Para[0][1] = 0;
	Para[1][2] -= Para[0][2] * (Para[1][0] / Para[0][0]);
	Para[1][0] = 0;
	Para[0][2] /= Para[0][0];
	//Para[0][0] = 1.0;
	Para[1][2] /= Para[1][1];
	//Para[1][1] = 1.0;
	//printf("拟合数据成功，拟合直线为：\r\ny = (%lf) * x + (%lf);\r\n", Para[0][2], Para[1][2]);
	return Para[0][2];
}
float Paramax(void)
{
	int32_t Amount = 5;
	int32_t maxdata = 0;
	int32_t xi = 0;
	for(; Amount > 0; Amount--, xi++)
	{
	   if(solardeal.solarbuf[5 + xi] > maxdata)	
		   maxdata = solardeal.solarbuf[5 + xi];
	}
	return maxdata;
}

static void volmeas_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	ret_code_t err_code;
#ifdef ADC_LOWPWR
	if (!m_saadc_initialized)
	{
		saadc_init();                                              //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
	}
	m_saadc_initialized = true;                                    //Set SAADC as initialized	
#endif
	err_code = nrf_drv_saadc_sample();//adc采样
	APP_ERROR_CHECK(err_code);
	//printk("volmeas_timeout_handler\r\n");
}

/**********************************************************************************************
 * 描  述 : ADC中断处理函数
 * 参   数: 无
 * 返回值 : 无
 ***********************************************************************************************/ 
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	float data = 0;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
		data = (150.0f / 12.0f + 1.0f) * 4.0f * 600.0f;
		data = data * (p_event->data.done.p_buffer[0]);
		data = data / 4096.0f;
		solar_vol  = (int32_t)data;
		
		
		data = (100.47f / 100.0f + 1.0f) * 4.0f * 600.0f;
		data = data * (p_event->data.done.p_buffer[1]);
		data = data / 4096.0f;
		batt_vol  = (int32_t)data;	
		//battVol[detCnt] = solar_vol;		

		data = 5.0f * 600.0f / (50.0f * 0.025f);  //I = U/50/R  R = 0.025欧
		//data = 5.0f * 600.0f / (50.0f * 0.05f);  //I = U/50/R  R = 0.025欧
		data = data * (p_event->data.done.p_buffer[2]);
		data = data / 4096.0f;
		char_cur  = (int32_t)data;	
#ifdef ADC_LOWPWR
		nrf_drv_saadc_uninit();                                                                   //Unintialize SAADC to disable EasyDMA and save power
		NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);               //Disable the SAADC interrupt
		NVIC_ClearPendingIRQ(SAADC_IRQn);                                                         //Clear the SAADC interrupt if set
		m_saadc_initialized = false;    
#endif
    } 

	tickcnt ++;

	switch(solardeal.solarstate)
	{
		case 0://表示为晚上时间
			nrf_gpio_pin_set(PIN_SOLAR_EN);	
			nrf_gpio_pin_set(PIN_LED2);					
			if(solar_vol > SOLAR_DAYING)			//天亮了
			{
				solardeal.solarstate = 3; 		    //切换流程	
				solardeal.solarcapcnt = 0;			//清零
				solardeal.solarpwrcnt = 0;
				solardeal.bufcnt = 0;				//清零			
				solardeal.currbufsum = 0;			//清零
				char_cur_sum = 0;					//对充电累积电流归零	
				solardeal.solarcapvola = solar_vol;	//
				solar_volbuf_cnt = 0;
				solardeal.solarbuffull = 0;
				solardeal.solar_solarmax = 17000;
				solardeal.solarbuf[0] = 0;	
				solardeal.solarbuf[1] = 0;
				solardeal.solar_capstop = 15000;	
				
			}
		break;
		case 1://超级电容充电流程
			nrf_gpio_pin_set(PIN_LED2);
			solardeal.solarcapcnt ++;				//超级电容充电时间计数
			if(solar_vol < SOLAR_NIGHTING)			//开始晚上流程阈值
			{	
				solardeal.solarstate = 0; 			//切换晚上流程 
			}
			if(solar_vol > solardeal.solar_solarmax)//开始电池充电流程
			{
				solardeal.solarstate = 2;			//切换电池充电流程 
				solardeal.solarcapvolb = solar_vol; //超级电容充电的电压		
			}
		break;
		case 2://电池充电池流程
			nrf_gpio_pin_clear(PIN_SOLAR_EN);		//使能充电
			nrf_gpio_pin_toggle(PIN_LED2);			//闪烁表示电池充电			
			solardeal.bufcnt++;						//充电时间计数
			solardeal.currbufsum += char_cur;		//充电电流累加	
			if(solar_vol < solardeal.solar_capstop)	//停止电池充电阈值
			{
				nrf_gpio_pin_set(PIN_SOLAR_EN);		//停止充电
				solardeal.solarstate = 3; 			//切换超级电容充电流程
				solardeal.solarcapvola = solar_vol; //
				solardeal.solarbuffull = 0;
				solardeal.solarbuf[0] = 0;	
			}
		break;
		case 3://计算太阳能电池最高电压并完成超级电容充电
			nrf_gpio_pin_set(PIN_LED2);				//关闭LED
			//solardeal.solarcapcnt ++;				//超级电容充电时间计数
			if(solar_vol < SOLAR_NIGHTING)			//开始晚上流程阈值
			{	
				solardeal.solarstate = 0; 			//切换晚上流程 
			}
			else
			{
				solardeal.solarbuf[0] += solar_vol; 		//存储solar数据											
				solardeal.solarcapcnt ++;					//超级电容充电时间计数
				if(solardeal.solarcapcnt % 10 == 0)			//每10个值求一次平均值
				{	
					solardeal.solarbuf[4] = solardeal.solarbuf[0] / 10;
					solardeal.solarbuf[0] = 0;
					solardeal.solarbuf[9] = solardeal.solarbuf[8];
					solardeal.solarbuf[8] = solardeal.solarbuf[7];
					solardeal.solarbuf[7] = solardeal.solarbuf[6];
					solardeal.solarbuf[6] = solardeal.solarbuf[5];
					solardeal.solarbuf[5] = solardeal.solarbuf[4];
					if(solardeal.solarcapcnt > 41)//solardeal.solarbuffull = 1;					//设置标志，允许计算斜率
					//if(solardeal.solarbuffull == 1)//至少有5个历史数据开始计算
					{
						data = 3.0f * Paradeal();	//通过斜率计算电流
						if((data < 5.0f) || (solardeal.solarcapcnt > 5000) || (solar_vol  > 17500))//当太阳电流小于0.5mA或者电容充电时间大于500S表示准备放电
						{
							solardeal.solar_solarmax = Paramax();//计算正确峰值电压
							//if(solardeal.solar_solarmax > 17500)solardeal.solar_solarmax = 17500;
							solardeal.solar_capstop = solardeal.solar_solarmax * 80 /100;
							if(solardeal.solar_capstop < SOLAR_NIGHTING)solardeal.solar_capstop = SOLAR_NIGHTING;
							solardeal.solarstate = 2;			//切换电池充电流程 
							solar_power = solardeal.solarbuf[1] / solardeal.solarpwrcnt;
							solardeal.solarcapcnt = 0;
							solardeal.solarpwrcnt = 0;
							solardeal.solarbuf[1] = 0;
							solardeal.solarbuffull = 0;								
						}
						else
						{
							//data = 3000.0f * data;				//Cap * (delta Vol / delta time);
							solardeal.solarbuf[2] = (int32_t) data;	
							solardeal.solarbuf[1] += solardeal.solarbuf[2];	
							solardeal.solarpwrcnt ++;
							//solardeal.solarcapcnt ++;				//超级电容充电时间计数
						}
					}				
				}
			}
		break;
		default:
			solardeal.solarstate = 0; 
		break;
		
		
	}

	
	solar_vol_old = solar_vol;
	if(batt_vol < 3500)
	{
		nrf_gpio_pin_clear(PIN_OTG_EN);
		nrf_gpio_pin_set(PIN_LED1);
	}
	else if(batt_vol > 3800)
	{
		nrf_gpio_pin_set(PIN_OTG_EN);
		nrf_gpio_pin_toggle(PIN_LED1);
	}
	//printk("myaddr:%d\r\n",myaddr[0]);
	//I = C * delta / T
	//NRF_LOG_RAW_INFO("V%d", p_event->data.done.p_buffer[0]);
}

void saadc_init(void)
{
    ret_code_t err_code;

    nrf_saadc_channel_config_t channel_0_config;
	nrf_saadc_channel_config_t channel_1_config;
	nrf_saadc_channel_config_t channel_2_config;
	nrf_drv_saadc_config_t saadc_config;
	channel_0_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_0_config.gain = NRF_SAADC_GAIN1_4;                                   			//Set input gain to 1/2. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_0_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_0_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_0_config.pin_p = NRF_SAADC_INPUT_AIN0;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_0_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_0_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_0_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin	
	err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);

	//nrf_saadc_channel_config_t channel_1_config =
	//NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
	//channel_1_config.gain = SAADC_CH_CONFIG_GAIN_Gain1_2;
	channel_1_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_1_config.gain = NRF_SAADC_GAIN1_4;                                   			//Set input gain to 1/2. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_1_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_1_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_1_config.pin_p = NRF_SAADC_INPUT_AIN1;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_1_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_1_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_1_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin		
	err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);

    //nrf_saadc_channel_config_t channel_2_config =
	//NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
	channel_2_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_2_config.gain = NRF_SAADC_GAIN1_5;                                   			//Set input gain to 1/2. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_2_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_2_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_2_config.pin_p = NRF_SAADC_INPUT_AIN2;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_2_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_2_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_2_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin	
	err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);

	    //Configure SAADC
    saadc_config.low_power_mode = false;                                                   	//Enable low power mode.false true
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;                                 	//Set SAADC resolution to 10-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;                         		//Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = SAADC_CONFIG_IRQ_PRIORITY;  	
	err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
	
	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
	//err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
	
	err_code = err_code;
#ifndef ADC_LOWPWR	
	// Create timers.
	app_timer_create(&m_volmeas_timer_id, APP_TIMER_MODE_REPEATED, volmeas_timeout_handler);
    app_timer_start(m_volmeas_timer_id, VOLMEAS_READ_INTERVAL, NULL);
#endif	
	adc_value.cnt = 0;
}

void solar_init(void)
{
	nrf_gpio_cfg_output(PIN_SOLAR_EN);
	nrf_gpio_pin_clear(PIN_SOLAR_EN);
	nrf_gpio_cfg_output(PIN_SOLAR_VOL1);
	nrf_gpio_pin_set(PIN_SOLAR_VOL1);
	nrf_gpio_cfg_output(PIN_SOLAR_VOL2);
	nrf_gpio_pin_clear(PIN_SOLAR_VOL2);	
	nrf_gpio_cfg_output(PIN_OTG_EN);
	nrf_gpio_pin_set(PIN_OTG_EN);	
#ifndef ADC_LOWPWR	
	saadc_init();
#else
	// Create timers.
	app_timer_create(&m_volmeas_timer_id, APP_TIMER_MODE_REPEATED, volmeas_timeout_handler);
    app_timer_start(m_volmeas_timer_id, VOLMEAS_READ_INTERVAL, NULL);		
#endif
}

extern uint32_t ble_send_solardata(int32_t value1, int32_t value2);

void solar_run(uint8_t discnt)
{
	int32_t solardeal_currbufsum;
	#if 0		
		if(tickcnt % 4 == 0)
		{
			if(tickcnt % 40 < 20)
			{
				LED_Printint(30, 0, solar_vol);
				LED_Printint(30, 2, batt_vol);
				LED_Printint(30, 4, char_cur);
				LCD_P8x16Str(0, 6, "I:");
				LED_Printint(30, 6, solardeal.currbufsum/36000);		
				
			}
			else
			{
				LED_Printint(30, 0, solar_vol);
				LED_Printint(30, 2, batt_vol);
				LED_Printint(30, 4, solardeal.solarstate);		
				LCD_P8x16Str(0, 6, "P:");
				LED_Printint(30, 6, solar_power);//显示实时太阳能功率	
				
			}	
		}
#else
		if(tickcnt % 10 == 0)
		{
			solardeal_currbufsum = solardeal.currbufsum/36000;
			if(discnt == 0)//显示电池
			{
				LCD_P8x16Str(0, 0, "0");				
				LED_Printint(30, 0, solar_vol);
				LED_Printint(30, 2, batt_vol);
				LED_Printint(30, 4, char_cur);
				LED_Printint(30, 6, solardeal_currbufsum);
				ble_send_solardata(solar_vol, batt_vol);		
			}	
			else if(discnt == 1)
			{
				LCD_P8x16Str(0, 0, "1");				
				LED_Printint(30, 0, solardeal.solarbuf[2]);
				LED_Printint(30, 2, solardeal.solarstate);
				LED_Printint(30, 4, solardeal.solarcapcnt);		
				LED_Printint(30, 6, solar_power);//显示实时太阳能功率			
				ble_send_solardata(char_cur, solar_power);		
			}
			else if(discnt == 2)
			{
				LCD_P8x16Str(0, 0, "2");				
				LED_Printint(30, 0, solardeal.solarbuf[6]);
				LED_Printint(30, 2, solardeal.solarbuf[7]);
				LED_Printint(30, 4, solardeal.solarbuf[8]);		
				LED_Printint(30, 6, solardeal.solarbuf[9]);//显示实时太阳能功率				
				
				ble_send_solardata(solardeal_currbufsum, solardeal.solarbuf[7]);	
			}
			else if(discnt == 3)
			{
				LCD_P8x16Str(0, 0, "3");				
				LED_Printint(30, 0, solar_vol);
				LED_Printint(30, 2, batt_vol);
				LED_Printint(30, 4, solardeal.solar_capstop);		
				LED_Printint(30, 6, solardeal.solar_solarmax);//显示实时太阳能功率	
				ble_send_solardata(solardeal.solar_capstop, solardeal.solar_solarmax);	
			}		
			else if(discnt == 4)		
			{
				LCD_P8x16Str(0, 0, "4");				
				LED_Printint(30, 0, solar_vol);
				LED_Printint(30, 2, batt_vol);
				LED_Printint(30, 4, solardeal.solarstate);		
				LED_Printint(30, 6, solar_power);//显示实时太阳能功率			
				//ble_send_solardata(solardeal.solar_capstop, solardeal.solar_solarmax);		
			}
			else	
			{
				LCD_P8x16Str(0, 0, "5");				
				LED_Printint(30, 0, solar_vol);
				LED_Printint(30, 2, batt_vol);
				LED_Printint(30, 4, solardeal.solarstate);		
				LED_Printint(30, 6, solar_power);//显示实时太阳能功率					
			}			
		}
#endif
}
