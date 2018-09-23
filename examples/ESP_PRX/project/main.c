/****************************************Copyright (c)****************************************************
**                                        
**                                 
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			     main.c
** Last modified Date:         
** Last Version:		   
** Descriptions:		   ʹ�õ�SDK�汾-SDK_11.0.0
**				
**--------------------------------------------------------------------------------------------------------
** Created by:			FiYu
** Created date:		2016-5-19
** Version:			    1.0
** Descriptions:		2.4G����ͨѶ��nRF51822֮��ͨѶ
**--------------------------------------------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_pwm.h"
#include "boards.h"
#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "sdk_common.h"
#include "oled.h"
#include "..\fft\fft.h"

/* ��������ָʾ�ƺʹ���ռ�õ�nRF51822�ܽ���Դ
P0.21�����������ָʾ��D1
P0.22�����������ָʾ��D2
P0.23�����������ָʾ��D3
P0.24�����������ָʾ��D4

P0.09��UART_TXD   �����ڷ���
P0.11��UART_RXD   �����ڽ���
P0.08��UART_CTS
P0.10��UART_RTS

��Ҫ�̽Ӷ�Ӧ������ñ
*/

#define RX 0 
#define TX 1 

#define LEDS_N   			24

#define TIMING_ONE  		14  //1000/72 * 50 ns= 694.4ns 
#define TIMING_ZERO 		6  //1000/72 * 25 ns= 347.2ns
#define TIMING_RESET 		0
#define TIMING_HIGH 		20 

unsigned short LED_BYTE_Buffer[LEDS_N * 24 +1];
unsigned char ledbit[LEDS_N][3] ={0};
unsigned int colortmp = 0;

const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
uint8_t RX_MODE_FLAG = 0;
uint8_t TX_MODE_FLAG = 0;

COMPLEX x[SAMPLE_NODES];  

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */
static nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);
nrf_esb_payload_t rx_payload;
nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;

nrf_pwm_values_individual_t m_demo1_seq_values[LEDS_N * 24 + 1];

nrf_pwm_sequence_t const m_demo1_seq =
{
	.values.p_individual = m_demo1_seq_values,
	.length = NRF_PWM_VALUES_LENGTH(m_demo1_seq_values),
	.repeats = 0,
	.end_delay = 0
};

void pwm_event_handler(nrf_drv_pwm_evt_type_t event_type)
{
    //switch(event_type)
    if(event_type == NRF_DRV_PWM_EVT_FINISHED)
	{	
		printf("NRF_DRV_PWM_EVT_FINISHED\r\n");			
	}
	else if(event_type == NRF_DRV_PWM_EVT_END_SEQ0)
	{
		printf("NRF_DRV_PWM_EVT_END_SEQ0\r\n");
	}
	else if(event_type == NRF_DRV_PWM_EVT_END_SEQ1)
	{
		printf("NRF_DRV_PWM_EVT_END_SEQ1\r\n");
	}
	else if(event_type == NRF_DRV_PWM_EVT_STOPPED)
	{	
		printf("NRF_DRV_PWM_EVT_STOPPED\r\n");
	}
}

void pwm_init(void)
{
#if 1
	#define PWM_REF_BP_PIN		12
	#define PWM_REF_BN_PIN		18
	#define PWM_REF_TEMP_PIN	19
	uint32_t err_code = NRF_SUCCESS;
	nrf_drv_pwm_config_t const config0 =
	{
		.output_pins =
		{
			PWM_REF_BP_PIN | NRF_DRV_PWM_PIN_INVERTED,//ch0 | NRF_DRV_PWM_PIN_INVERTED
			NRF_DRV_PWM_PIN_NOT_USED,//PWM_REF_BN_PIN | NRF_DRV_PWM_PIN_INVERTED,//ch1
			NRF_DRV_PWM_PIN_NOT_USED,//PWM_REF_TEMP_PIN | NRF_DRV_PWM_PIN_INVERTED,//ch2
			NRF_DRV_PWM_PIN_NOT_USED//ch3
		},
		.irq_priority = APP_IRQ_PRIORITY_LOW,
		.base_clock = NRF_PWM_CLK_16MHz,
		.count_mode = NRF_PWM_MODE_UP,
		.top_value = 20,   //freq = 16000000/20 = 800KHz
		.load_mode = NRF_PWM_LOAD_INDIVIDUAL,
		.step_mode = NRF_PWM_STEP_AUTO
	};
	err_code = nrf_drv_pwm_init(&m_pwm0, &config0, pwm_event_handler);
	APP_ERROR_CHECK(err_code);
	//for(uint32_t i = 0;i < LEDS_N;i++)
	//{
	//	m_demo1_seq_values[i].channel_0 = 4;
		//m_demo1_seq_values[i].channel_1 = 5;
		//m_demo1_seq_values[i].channel_2 = 10;
	//}
	//nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, LEDS_N, NRF_DRV_PWM_FLAG_NO_EVT_FINISHED);	
#else
	NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
	NRF_PWM0->PRESCALER = NRF_PWM_CLK_16MHz;
	NRF_PWM0->MODE = NRF_PWM_MODE_UP;
	NRF_PWM0->COUNTERTOP = 20;
	NRF_PWM0->PSEL.OUT[0] = 1;
	for(uint32_t i = 1; i < NRF_PWM_CHANNEL_COUNT; i++)
	{
		NRF_PWM0->PSEL.OUT[i] = NRF_PWM_PIN_NOT_CONNECTED;
	}
	NRF_PWM0->DECODER = ((uint32_t)NRF_PWM_LOAD_INDIVIDUAL << PWM_DECODER_LOAD_Pos) |
		((uint32_t)NRF_PWM_STEP_AUTO << PWM_DECODER_MODE_Pos);
	NRF_PWM0->SHORTS = 0;
	NRF_PWM0->INTEN = 0;
	static nrf_pwm_values_individual_t duty_cycles = 
	{
		.channel_0 = 10,
		.channel_1 = 10,
		.channel_2 = 10,
		.channel_3 = 10,
	};

	NRF_PWM0->SEQ[0].PTR = (uint32_t)&duty_cycles;
	NRF_PWM0->SEQ[0].CNT = NRF_PWM_VALUES_LENGTH(duty_cycles);
	NRF_PWM0->SEQ[0].REFRESH = 0;
	NRF_PWM0->SEQ[0].ENDDELAY = 0;
	NRF_PWM0->TASKS_SEQSTART[0] = 1;
#endif
}
/* This function sends data bytes out to a string of WS2812s
 * The first argument is a pointer to the first RGB triplet to be sent
 * The seconds argument is the number of LEDs in the chain
 * 
 * This will result in the RGB triplet passed by argument 1 being sent to 
 * the LED that is the furthest away from the controller (the point where
 * data is injected into the chain)
 */
void WS2812_send(uint8_t (*color)[3], uint16_t len)
{
	uint8_t i;
	uint16_t memaddr;
	uint16_t index;
	uint16_t buffersize;
	buffersize = (len * 24) + 1;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes for RESET CODE
	memaddr = 0;
	index = 0;
	while (len)
	{	
		for(i=0; i<8; i++) // GREEN data
		{
			//LED_BYTE_Buffer[memaddr] = ((color[index + 0][1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			m_demo1_seq_values[memaddr].channel_0 = 0x8000 | (((color[index + 0][1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO);
			memaddr++;
		}
		for(i=0; i<8; i++) // RED
		{
			//LED_BYTE_Buffer[memaddr] = ((color[index + 0][0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			m_demo1_seq_values[memaddr].channel_0 = 0x8000 | (((color[index + 0][0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO);
			memaddr++;
		}
		for(i=0; i<8; i++) // BLUE
		{
			//LED_BYTE_Buffer[memaddr] = ((color[index + 0][2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			m_demo1_seq_values[memaddr].channel_0 = 0x8000 | (((color[index + 0][2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO);
			memaddr++;
		}
		len--;
		index++;
	}
	//===================================================================//	
	//bug�����һ�����ڲ��β�֪��Ϊʲôȫ�Ǹߵ�ƽ��������һ������
	//LED_BYTE_Buffer[memaddr] = TIMING_HIGH;//((color[0][2]<<8) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	//===================================================================//	
	//memaddr++;	
	while(memaddr < buffersize)
	{
		m_demo1_seq_values[memaddr].channel_0 = 0x8000 | 0;
		memaddr++;
	}
	nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, 1, NRF_DRV_PWM_FLAG_NO_EVT_FINISHED);	
}

void ws2812Test(void)
{ 
	unsigned short i;
	unsigned short k;
	unsigned short j;
	unsigned char inc;
	unsigned char redtmp,greentmp,bluetmp;
	unsigned char ledbit[LEDS_N][3] ={0};
	unsigned int colortmp = 0;
	i = 0;
	k = 0;
	inc = 0;
	while (1)
	{			
		colortmp += 10;
		if(colortmp > 0x00ffffff)colortmp = 0;
		for(j = 0;j < LEDS_N;j++)
		{
			if(j == k)
			{
				ledbit[j][0] = redtmp;//RED (colortmp & 0x000000ff);0xff;//
				ledbit[j][1] = greentmp;//GREEN(colortmp & 0x0000ff00) >> 8;//0xff;//
				ledbit[j][2] = bluetmp;	//(colortmp & 0x00ff0000) >> 16;//0xff;//
			}
			else
			{
				ledbit[j][0] = 0;//RED
				ledbit[j][1] = 0;//GREEN
				ledbit[j][2] = 0;					
			}
		}

#if 1		
		//if(inc == 0)
		{
			k ++;
			if(k==LEDS_N)
			{
				inc = 1;
				if(i%3 == 0){redtmp=100;greentmp=0;bluetmp=0;}
				else if(i%3 == 1){redtmp=0;greentmp=100;bluetmp=0;}
				else if(i%3 == 2){redtmp=0;greentmp=0;bluetmp=100;}
				i++;
				if(i==3)i=0;
				k = 0;
			}
		}
		#if 0
		else 
		{
			k --;
			if(k==0)
			{
				inc = 0;
				//if(i%3 == 0){redtmp=255;greentmp=0;bluetmp=0;}
				//else if(i%3 == 1){redtmp=0;greentmp=255;bluetmp=0;}
				//else if(i%3 == 2){redtmp=0;greentmp=0;bluetmp=255;}
				//i++;
				//if(i==3)i=0;
			}
		}
		#endif
#else

		k ++;
		if(k==LEDS_N)
		{
			inc = 1;
			if(i%3 == 0){redtmp=255;greentmp=0;bluetmp=0;}
			else if(i%3 == 1){redtmp=0;greentmp=255;bluetmp=0;}
			else if(i%3 == 2){redtmp=0;greentmp=0;bluetmp=255;}
			i++;
			if(i==3)i=0;
			k = 0;
		}
#endif		
		WS2812_send(&ledbit[0], LEDS_N);
		
		printf("Sending\r\n");
		nrf_delay_ms(100);
		//OSTimeDlyHMSM(0, 0, 0, 100);
	}
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    static uint32_t i;
    //uint32_t led_to_invert = (1 << leds_list[(i++) % LEDS_NUMBER]);
    
    switch(event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            //LEDS_INVERT(led_to_invert);
			//i = m_demo1_seq_values[0].channel_1;
			//if(i<20)i+=1;
			//else i = 0;
			//m_demo1_seq_values[0].channel_1 = i;
			printf("NRF_TIMER_EVENT_COMPARE0\r\n");
			//for(uint32_t j = 0;j < LEDS_N;j++)
			//{
			//	ledbit[j][0] = 0x000000ff;//RED (colortmp & 0x000000ff);
			//	ledbit[j][1] = 0;//GREEN(colortmp & 0x0000ff00) >> 8;//
			//	ledbit[j][2] = 0;	//(colortmp & 0x00ff0000) >> 16;//
			//}
			//WS2812_send(&ledbit[0], LEDS_N);			
            break;
        
        default:
            //Do nothing.
            break;
    }    
}
void timer_init(void)
{
    uint32_t time_ms = 200; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    err_code = nrf_drv_timer_init(&TIMER_LED, NULL, timer_led_event_handler);
    APP_ERROR_CHECK(err_code);
    
    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);
    
    nrf_drv_timer_extended_compare(
         &TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&TIMER_LED);	
}


//���ڳ�ʼ������ֹ���أ�������115200
void uart_init(void)
{
	  uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED, //��ֹ����
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200//������115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
}

void nrf_esb_error_handler(uint32_t err_code, uint32_t line)
{
#if DEBUG //lint -e553
    while(true);
#else
    NVIC_SystemReset();
#endif

}
//ESB�¼��������������¼��п��Զ������յ������ݺ�RSSIֵ
void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    uint8_t i;
	uint8_t tmp;
	switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
			printf("TX SUCCESS EVENT\r\n");
			// Toggle one of the LEDs.
			//nrf_gpio_pin_toggle(LED_2);
			RX_MODE_FLAG = 1;		
		break;
        case NRF_ESB_EVENT_TX_FAILED:
			printf("TX FAILED EVENT\r\n");
			//nrf_gpio_pin_toggle(LED_3);
			nrf_esb_flush_tx();
			RX_MODE_FLAG = 1;				
		break;
        case NRF_ESB_EVENT_RX_RECEIVED:
			printf("RX RECEIVED: ");
            if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
				printf("length = %02X",(uint8_t)rx_payload.length); //���ڴ�ӡ�����յ����ݳ���
				printf("DATA = ");
				for(i=0;i<rx_payload.length;i++)printf(" %02X",(uint8_t)rx_payload.data[i]);  //���ڴ�ӡ�����յ�������
				printf("  RSSI = -%d",(uint8_t)rx_payload.rssi);    //���ڴ�ӡ��RSSIֵ
				printf("\r\n");
				nrf_gpio_pin_toggle(LED_1);                 //D1ָʾ��״̬��ת��ָʾ�ƽ��յ��µ�����              
            	tmp = (uint8_t)rx_payload.rssi;
				OLED_ShowChar( 0,0,tmp/100 + 48);
				tmp = tmp%100;
				OLED_ShowChar( 8,0,tmp/10 + 48);
				OLED_ShowChar(16,0,tmp%10 + 48);
			}
			nrf_esb_flush_rx();
			TX_MODE_FLAG = 1;			
       break;
    }
}

//���߹���4�ֱ���2440MHz��ͨ��0��ַ0x99999999C0��ͨ��1~7��ַ0x66666666XX
//����2MHz�������ֽ�3�ֽ�QxC(Q��ʼ��C������x������char)
void radio_configure(uint8_t MHz,uint8_t Length,unsigned short kbps)//�������ã�׼����nrf24L01ͨѶ
{
  //���߹���04��+4�ֱ���0��0�ֱ���FC��-4�ֱ���F8��-8�ֱ�
	//    F4��-12�ֱ���F0��-16�ֱ���EC��-20�ֱ���D8��-30�ֱ�
	NRF_RADIO->TXPOWER = (0x04<<0);//���߹���4�ֱ�
  
	NRF_RADIO->FREQUENCY = MHz&0x3F;//50UL;//����Ƶ��40MHz+2400MHz=2440MHz

	//�������ʣ�00��1Mbit��01��2Mbit��02��250Kbit��03��1Mbit��������
	//NRF_RADIO->MODE = (02<<0);//����2MHz
	//�������ʣ�00��1Mbit��01��2Mbit��02��250Kbit��03��1Mbit��������
	if(kbps==1000)
		NRF_RADIO->MODE = (00<<0);//����1Mbit
	else if(kbps==2000)
		NRF_RADIO->MODE = (01<<0);//����2Mbit
	else if(kbps==250)
		NRF_RADIO->MODE = (02<<0);//����250Kbit
	else
		NRF_RADIO->MODE = (01<<0);//����1Mbit(����)
	// ���ߵ�ַ����
	NRF_RADIO->PREFIX0 = 0xE5C2C3C4UL;//0xC3C2C1C0UL;  // ͨ��3 �� 0 �ĵ�1�ֽ�
	NRF_RADIO->PREFIX1 = 0xC5C6C7C8UL;//0xC7C6C5C4UL;  // ͨ��7 �� 4 �ĵ�1�ֽ�
	NRF_RADIO->BASE0   = 0xE4E3E2E1UL;//0x99999999UL;  // ͨ��0�ĸ��ֽ�
	NRF_RADIO->BASE1   = 0xC2C2C2C2UL;//0x66666666UL;  // ͨ��1-7�ĸ��ֽ�
	NRF_RADIO->TXADDRESS = 0x00UL;      // ����ʹ�õ�ͨ���ţ�0ͨ��
	NRF_RADIO->RXADDRESSES = 0x01UL;    // ���յ�ͨ���ţ�1ͨ��

	// ���ð�0������
	NRF_RADIO->PCNF0 = (0<<16)| //S1����ĳ���
                       (0<<8) | //S0���ĳ���
                       (0<<0);  //�����ֶ��еı�����
	// ���ð�1������
	NRF_RADIO->PCNF1 = (0<<25)| //Ч��λ��0�أ�1����
                       (1<<24)| //���ݴ�С�ˣ��ߵ��ֽ��ĸ��ȷ� 0���ֽڣ�1���ֽڣ�
                       (4<<16)| //ͨ��1~7���ֽڳ��ȣ� nrf24�ߵ��ֽ�5�ֽڣ�4����+1���ͣ�
                       (Length<<8) | //�����ֽڳ��ȣ�255~1��3�ֽ�QxC
                       (Length<<0);  //Ӳ�������ֽڳ��ȣ�255~1��3�ֽ�QxC
	// CRC У�鳤������
	NRF_RADIO->CRCCNF = 2; // У�鳤�� 2��char
	if ((NRF_RADIO->CRCCNF & 0x03)== 2 )
	{
		NRF_RADIO->CRCINIT = 0xFFFFUL;      // У���ʼֵ
		NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
	}
	else if ((NRF_RADIO->CRCCNF & 0x03) == 1 )
	{
		NRF_RADIO->CRCINIT = 0xFFUL;        // У���ʼֵ
		NRF_RADIO->CRCPOLY = 0x107UL;       // CRC poly: x^8+x^2^x^1+1
	}
	//���ռĴ�����  NRF_RADIO->PACKETPTR	
}
//nrf51822���亯��
uint8_t NRF5x_TX(uint8_t *packet)
{
	NRF_GPIO->OUTSET = (1 << BSP_TXEN_PIN);
	NRF_GPIO->OUTCLR = (1 << BSP_RXEN_PIN);
	NRF_RADIO->EVENTS_READY = 0U;//�շ�ģʽ�л����ָʾ�ơ�
	NRF_RADIO->TASKS_TXEN = 1U;//�������߷���ģʽ
	NRF_RADIO->PACKETPTR = (uint32_t)packet;//ָ��ָ�����ݻ���packet 
	while (NRF_RADIO->EVENTS_READY == 0U)//�ȴ��շ�ģʽ�л����
	{// �������ݴ���
		nrf_gpio_pin_set(30);//blue�������ݳɹ��ˣ���Խ��˵�����ݴ���Խ��
	}
	NRF_RADIO->EVENTS_END = 0U;//�������ָʾ�� ���ص� 
	NRF_RADIO->TASKS_START = 1U;//��ʼ����??
	while(NRF_RADIO->EVENTS_END == 0U) //�ȴ��������
	{}
	NRF_RADIO->EVENTS_DISABLED = 0U;//���ߵ��ѹر�ָʾ�ƣ��ص�
	NRF_RADIO->TASKS_DISABLE = 1U; // �ر����ߵ�
	NRF_GPIO->OUTCLR = (1 << BSP_TXEN_PIN);
	while(NRF_RADIO->EVENTS_DISABLED == 0U)//�ȴ����ߵ��豸�ر�
	{
		nrf_gpio_pin_clear(30);//blue�������ݳɹ��ˣ���Խ��˵�����ݴ���Խ��
	}
	return 1;
}
//nrf51822���պ���
uint8_t NRF5x_RX(uint8_t *packet)
{
	uint8_t CRC=0;
	NRF_GPIO->OUTSET = (1 << BSP_RXEN_PIN);
	NRF_GPIO->OUTCLR = (1 << BSP_TXEN_PIN);
	NRF_RADIO->EVENTS_READY = 0U; //�շ�ģʽת�����??��־λ? ?? ? 
	NRF_RADIO->TASKS_RXEN= 1U; //��������ģʽ
	NRF_RADIO->PACKETPTR = (uint32_t)packet;//ָ��ָ�����ݻ���packet
	while(NRF_RADIO->EVENTS_READY == 0U) //�ȴ��շ�ģʽת�����(ָʾ��)
	{
		nrf_gpio_pin_set(30);//blue�������ݳɹ��ˣ���Խ��˵�����ݴ���Խ��
		// Do nothing.�ȴ�
	}
	NRF_RADIO->EVENTS_END = 0U;//�������(ָʾ��)
	NRF_RADIO->TASKS_START = 1U; // ��ʼ����

	nrf_delay_ms(3);//���տ����е������ź�
	//û�յ�����
	while(NRF_RADIO->EVENTS_END == 0U)//�ȴ��������(ָʾ��)
	{
		//һֱû�յ�����
		NRF_RADIO->EVENTS_DISABLED = 0U;//���߹ر� ��־λ
		NRF_RADIO->TASKS_DISABLE = 1U;// �ر������豸
		while(NRF_RADIO->EVENTS_DISABLED == 0U);//�ȴ��豸�ر�
		nrf_gpio_pin_clear(30);//blue�������ݳɹ��ˣ���Խ��˵�����ݴ���Խ��
		return 0;
	}
	//�ɹ��յ�����
	if (NRF_RADIO->CRCSTATUS == 1U)//���CRCУ����ȷ
	{
		CRC=1;
	}
	else
	{
		CRC=0;
	}
	NRF_RADIO->EVENTS_DISABLED = 0U;//���߹ر�? ?��־λ
	NRF_RADIO->TASKS_DISABLE= 1U;// �ر������豸
	NRF_GPIO->OUTCLR = (1 << BSP_RXEN_PIN);
	while(NRF_RADIO->EVENTS_DISABLED == 0U);//�ȴ��豸�ر�
	if(CRC==1)
	{
		return 1;//���ճɹ�
	}
	return 0;//����CRCУ��������Խ���ʧ�� 
}

uint32_t esb_init( uint8_t state )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE4, 0xE3, 0xE2, 0xE1};   //����pipe0��ַ
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE5, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };//����pipe0��ַ������addr_prefix[0]����pipe0
    
    nrf_esb_config.payload_length           = 32;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_1MBPS; // ������������Ϊ250K
	switch(state)
	{
		case RX:
			nrf_esb_config.mode             = NRF_ESB_MODE_PRX;
			nrf_gpio_pin_clear(24);
			nrf_gpio_pin_set(20);
		break;
		case TX:
			nrf_esb_config.mode             = NRF_ESB_MODE_PTX;
			nrf_gpio_pin_clear(20);
			nrf_gpio_pin_set(24);			
		break;  
	}
    nrf_esb_config.event_handler            = nrf_esb_event_handler;       // �Ǽ��¼�������
    nrf_esb_config.selective_auto_ack       = false;
	//radio_configure(50, 32, 250);
    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    //���õ�ַ
	err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);
     
    return err_code;
}

void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}
/**********************************************************************************************
 * ��  �� : main����
 * ��  �� : ��
 * ����ֵ : ��
 ***********************************************************************************************/ 
int main(void)
{
    unsigned char cnt;

	//LEDS_CONFIGURE((unsigned int)LEDS_MASK); //��������ָʾ�ƵĹܽ�
    nrf_gpio_cfg_output(LED_1);
	nrf_gpio_pin_set(LED_1);
	//LEDS_OFF((unsigned int)LEDS_MASK);       //����ָʾ�Ƶĳ�ʼ״̬ΪϨ��
	nrf_gpio_cfg_output(OLED_DC);
	nrf_gpio_cfg_output(OLED_RST);
	nrf_gpio_cfg_output(OLED_CS);	
	nrf_gpio_cfg_output(OLED_EN);
	nrf_gpio_pin_clear(OLED_EN);	
	
	nrf_gpio_cfg_output(BSP_TXEN_PIN);
	nrf_gpio_cfg_output(BSP_RXEN_PIN);
	//LEDS_ON(BSP_LED_0_MASK);
	//LEDS_ON(BSP_LED_1_MASK);
    uint32_t err_code;
	uart_init();               //��ʼ������
	clocks_start();
	hal_spi_init();  //SPI��ʼ��
	//err_code = esb_init();     //��ʼ��ESB
	//APP_ERROR_CHECK(err_code);
	//esb_init(RX);
	//nrf_esb_start_rx();

	nrf_delay_ms(100);
    OLED_Init();                      //oled ��ʼ��  
	timer_init();
	pwm_init();
	LCD_P8x16Str(0,6,"program start");	
	printf("...program start\r\n");
 
	while (true)
    {
#if 0		
		if(RX_MODE_FLAG==1)
		{               
			RX_MODE_FLAG = 0;
			// Back to RX
			nrf_esb_disable();
			nrf_esb_config.mode = NRF_ESB_MODE_PRX;
			nrf_esb_init(&nrf_esb_config);
			nrf_esb_start_rx();
		}
		else if(TX_MODE_FLAG==1)
		{     
			TX_MODE_FLAG = 0;   

			nrf_esb_stop_rx();                  
			// Back to TX
			nrf_esb_disable();
			nrf_esb_config.mode = NRF_ESB_MODE_PTX;
			nrf_esb_init(&nrf_esb_config);



			tx_payload.length = 32;
			tx_payload.pipe = 0;
			tx_payload.noack = true;
			tx_payload.data[1]++;

			nrf_esb_start_tx();
			err_code = nrf_esb_write_payload(&tx_payload);	
			if (err_code == NRF_SUCCESS)
			{
				tx_payload.data[1]++;
				printf("Sending %dB packet OK %d\r\n",tx_payload.length,err_code);  
            } 
			else
			{
				printf("Sending %dB packet failed %d\r\n",tx_payload.length,err_code);
				nrf_delay_ms(100);
			}		
       }
      //  __WFE();	
#else

	printf("Sending\r\n");
	ws2812Test();   
	nrf_delay_ms(500);
#endif	   	   
#if 0
		tx_payload.noack = false;
		tx_payload.data[2] = cnt;
		cnt ++;
		err_code = nrf_esb_write_payload(&tx_payload);
        if (err_code == 0)
        {
            // Toggle one of the LEDs.
            //nrf_gpio_pin_write(LED_1, !(tx_payload.data[1]%8>0 && tx_payload.data[1]%8<=4));
            //nrf_gpio_pin_write(LED_2, !(tx_payload.data[1]%8>1 && tx_payload.data[1]%8<=5));
            //nrf_gpio_pin_write(LED_3, !(tx_payload.data[1]%8>2 && tx_payload.data[1]%8<=6));
            //nrf_gpio_pin_write(LED_4, !(tx_payload.data[1]%8>3));
            tx_payload.data[1]++;
			printf("Sending %dB packet OK %d\r\n",tx_payload.length,err_code);
        }
        else
        {
            printf("Sending %dB packet failed %d\r\n",tx_payload.length,err_code);
			err_code = esb_init();     //��ʼ��ESB
			nrf_delay_ms(100);
        }
		//LEDS_ON(BSP_LED_0_MASK);
		//LEDS_ON(BSP_LED_1_MASK);
		nrf_gpio_pin_set(LED_1)
		//fA = fB * fC;
		//arm_sqrt_f32(fA , &fB);
		//printf("fB * fC = %f\r\n",fA);
	
        nrf_delay_ms(200);
		nrf_delay_ms(200);
		nrf_gpio_pin_clr(LED_1);
		//LEDS_OFF(BSP_LED_0_MASK);
		//LEDS_OFF(BSP_LED_1_MASK);		
		nrf_delay_ms(100);
#endif	
    }
}
/********************************************END FILE*******************************************/
