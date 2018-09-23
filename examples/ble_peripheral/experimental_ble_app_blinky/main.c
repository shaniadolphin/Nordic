/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "stdarg.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "bsp.h"
#include "ble_gap.h"
#include "..\oled\oled.h"
#include "nrf_drv_saadc.h"
#include "nrf_saadc.h"
#include "nrf_drv_pwm.h"
#include "nrf_delay.h"
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "app_uart.h"
#include "AS608.h"
#include "..\prink\printk.h"




#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define ADVERTISING_LED_PIN             BSP_BOARD_LED_0                             /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN               BSP_BOARD_LED_1                             /**< Is on when device has connected. */

#define LEDBUTTON_LED_PIN               BSP_BOARD_LED_2                             /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON_PIN            15                                /**< Button that will trigger the notification event with the LED Button Service */
#define AS608_PS_STA_PIN				14
#define DEVICE_NAME                     "Nordic_Blinky"                             /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED       /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            6                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

APP_TIMER_DEF(m_volmeas_timer_id);      /**< adc采样定时器 */
#define VOLMEAS_READ_INTERVAL           APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< ADC采样间隔 1000ms读取一次*/
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_lbs_t                        m_lbs;   
#define SAMPLES_IN_BUFFER 				2             								/**< ADC采样数据缓存大小(字节数)  */
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */
unsigned char oled_busy = 0;

typedef struct
{
    uint8_t cnt;
	uint8_t buf[20]; 
}adc_value_t;
short batt_vol;
short usb_vol;

uint8_t data_array[100];
uint32_t data_array_lasttime;

uint8_t finger_result_en = 0;

uint8_t data_array_index = 0;

uint8_t ErrMessage = 0;

static nrf_saadc_value_t       m_buffer_pool[2][SAMPLES_IN_BUFFER];
adc_value_t adc_value;
//adc_value_t adc_valu                                  /**< LED Button Service instance. */
uint8_t restart_flag  = 0;
uint8_t myaddr[BLE_GAP_ADDR_LEN] = {0xc5,0x15,0x83,0xbc,0xd2,0x6a};
uint8_t rssi = 0;
uint8_t received_blue;
SearchResult finger_result;

typedef struct {
	unsigned int  rx_starttime;
	unsigned short rx_timeout_counter;//=
	unsigned char  rx_cnt;
	unsigned char  rx_size;
	unsigned char  rx_data;
	unsigned char  rx_buf[100];

	unsigned int  tx_starttime;
	unsigned short tx_timeout_counter;
	unsigned char  tx_cnt;
	unsigned char  tx_size;
	unsigned char  tx_data;
	unsigned char  tx_buf[100];

	unsigned char  is_exist;
	unsigned short retry_cnt;
	unsigned int   test_timer;

	unsigned int   test_cnt;
	unsigned short fail_cnt;
}COM_STRUCT;

COM_STRUCT ch[2];

#define LEDS_N   			24
#define TIMING_ONE  		14  //1000/72 * 50 ns= 694.4ns 
#define TIMING_ZERO 		6  //1000/72 * 25 ns= 347.2ns
#define TIMING_RESET 		0
#define TIMING_HIGH 		20 
nrf_pwm_values_individual_t m_demo1_seq_values[1];
const nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
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
		//printf("NRF_DRV_PWM_EVT_FINISHED\r\n");	
		#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
			SEGGER_RTT_printf(0,"NRF_DRV_PWM_EVT_FINISHED\r\n");//通讯成功
		#else
			NRF_LOG_RAW_INFO("NRF_DRV_PWM_EVT_FINISHED\r\n");//通讯成功
		#endif			
	}
	else if(event_type == NRF_DRV_PWM_EVT_END_SEQ0)
	{
		//printf("NRF_DRV_PWM_EVT_END_SEQ0\r\n");
		#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
			SEGGER_RTT_printf(0,"NRF_DRV_PWM_EVT_END_SEQ0\r\n");//通讯成功
		#else
			NRF_LOG_RAW_INFO("NRF_DRV_PWM_EVT_END_SEQ0\r\n");//通讯成功
		#endif			
	}
	else if(event_type == NRF_DRV_PWM_EVT_END_SEQ1)
	{
		//printf("NRF_DRV_PWM_EVT_END_SEQ1\r\n");
		#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
			SEGGER_RTT_printf(0,"NRF_DRV_PWM_EVT_END_SEQ1\r\n");//通讯成功
		#else
			NRF_LOG_RAW_INFO("NRF_DRV_PWM_EVT_END_SEQ1\r\n");//通讯成功
		#endif			
	}
	else if(event_type == NRF_DRV_PWM_EVT_STOPPED)
	{	
		//printf("NRF_DRV_PWM_EVT_STOPPED\r\n");
		#if defined(NRF_LOG_USES_RTT) && NRF_LOG_USES_RTT == 1
			SEGGER_RTT_printf(0,"NRF_DRV_PWM_EVT_STOPPED\r\n");//通讯成功
		#else
			NRF_LOG_RAW_INFO("NRF_DRV_PWM_EVT_STOPPED\r\n");//通讯成功
		#endif			
	}
}
void Steering_Eng_Angle_PWM1(float angle)
{
    unsigned int angle_tmp = (unsigned int)(1520 + 10 * angle);
    //TA1CCR0=19999;//set_pwm_freq(100);//t=10ms
    
    //TA1CCR1=angle_tmp*2-1;//10000us/(19999+1)=angle_tmp/(TA1CCR1+1)  
    if(angle_tmp < 10000)m_demo1_seq_values[0].channel_0 = 10000 - angle_tmp;//0x8000 | 0;   
    //USART_OUT("angle_tmp=%d,TA1CCR1=%d\r\n",angle_tmp,angle_tmp*2-1);
}

void pwm_init(void)
{
	#define PWM_REF_BP_PIN		18
	#define PWM_REF_BN_PIN		0
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
		.base_clock = NRF_PWM_CLK_1MHz,//.base_clock = NRF_PWM_CLK_16MHz,
		.count_mode = NRF_PWM_MODE_UP,
		.top_value = 10000,//20,   //freq = 16000000/20 = 800KHz  //1000000 / 10000 = 100Hz
		.load_mode = NRF_PWM_LOAD_INDIVIDUAL,
		.step_mode = NRF_PWM_STEP_AUTO
	};
	m_demo1_seq_values[0].channel_0 = 10000 - 1520;//0x8000 | 0;
	//Steering_Eng_Angle_PWM1(90);
	err_code = nrf_drv_pwm_init(&m_pwm0, &config0, pwm_event_handler);
	APP_ERROR_CHECK(err_code);
	nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, 100, NRF_DRV_PWM_FLAG_LOOP);	
}


#define PIN_OTG_EN			19
#define PIN_KEY_IN			14
#define PIN_CHG_EN1			21
#define PIN_CHG_EN2			20

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{  
    
    uint32_t       err_code = 0;
	//while (app_uart_put(p_data[i]) != NRF_SUCCESS);
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[data_array_index]));
            if(data_array_index<100)data_array_index++;
#if 0
            if (data_array[data_array_index - 1] == '\n' || (data_array_index >= 10))
            {
                //err_code = ble_nus_string_send(&m_nus, data_array, index);
				data_array_lasttime = NRF_RTC1->COUNTER;//app_timer_cnt_get();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
				data_array[index] = '\0';
				//printk("%s", data_array);
				NRF_LOG_RAW_INFO("%s", data_array);
                data_array_index = 0;
            }
#endif
        break;
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
        break;
        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
        break;
        default:
			
        break;
    }
}

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_leds_init();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
	uint32_t err_code;
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	// Create timers.
	//err_code = app_timer_create(&v_volmeas_timer_id,
	//			APP_TIMER_MODE_REPEATED,
	//			volmeas_timeout_handler);
	//APP_ERROR_CHECK(err_code);	
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(ble_lbs_t * p_lbs, uint8_t led_state)
{
    received_blue = led_state;
	if (led_state)
    {
        bsp_board_led_on(LEDBUTTON_LED_PIN);
        printk("Received %d!\r\n",led_state);
    }
    else
    {
        bsp_board_led_off(LEDBUTTON_LED_PIN);
        printk("Received %d!\r\n",led_state);
    }
	rssi = 0;
	if(led_state == 0)//主机发送了清零按键
	{
		restart_flag  = 1;  //从机允许操作
		LED_P20x40Char(54, 3, 11, 1);
	}
	else
	{
		restart_flag  = 0;
		if(led_state != myaddr[0])
		{
			LED_P20x40Char(54, 3, 11, 1);
		}
		else
		{
			LED_P20x40Char(54, 3, led_state%10, 1);
		}
	}
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_lbs_init_t init;

    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    bsp_board_led_on(ADVERTISING_LED_PIN);
}


/**@brief Function for handling the Application's BLE stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            printk("Connected 1 %d\r\n",m_conn_handle);
            bsp_board_led_on(CONNECTED_LED_PIN);
            bsp_board_led_off(ADVERTISING_LED_PIN);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			//NRF_LOG_INFO("Connected 2 %d\r\n",m_conn_handle);
            //err_code = app_button_enable();
            //APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            printk("Disconnected\r\n");
            bsp_board_led_off(CONNECTED_LED_PIN);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            //err_code = app_button_disable();
            //APP_ERROR_CHECK(err_code);

            advertising_start();
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            printk("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            printk("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_lbs_on_ble_evt(&m_lbs, p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN:
			printk("LEDBUTTON_BUTTON_PIN:%d\r\n",button_action);
			
            //NRF_LOG_INFO("Send button state change.\r\n"); 
			if(button_action == 1)
			{
				//err_code = ble_lbs_on_button_change(&m_lbs, button_action);myaddr[0]%10
				if(restart_flag  == 1 && m_conn_handle != BLE_CONN_HANDLE_INVALID)
				{
					err_code = ble_lbs_on_button_change(&m_lbs, myaddr[0]);
					if (err_code != NRF_SUCCESS &&
						err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
						err_code != NRF_ERROR_INVALID_STATE)
					{
						APP_ERROR_CHECK(err_code);
					}
					finger_result_en = 1;
				}
			}
            break;
		case AS608_PS_STA_PIN:
		{
			//NRF_LOG_INFO("Send button state change.\r\n"); 
			//NRF_LOG_INFO("%d:AS608_PS_STA_PIN:%d\r\n",NRF_RTC1->COUNTER, button_action);
			printk("%d:AS608_PS_STA_PIN:%d\r\n",NRF_RTC1->COUNTER, button_action);			
			
			if(button_action == 0)
			{
				//press_FR(finger_result);//刷指纹		
				//finger_result_en = 1;
			}
			
		}
		break;
        default:
            APP_ERROR_HANDLER(pin_no);
        break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    uint32_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON_PIN, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
		{AS608_PS_STA_PIN, false, NRF_GPIO_PIN_PULLUP, button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
	err_code = app_button_enable();
	APP_ERROR_CHECK(err_code);
}

static void volmeas_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	ret_code_t err_code;	
	err_code = nrf_drv_saadc_sample();//adc采样
	APP_ERROR_CHECK(err_code);
	printk("volmeas_timeout_handler\r\n");
}
/**********************************************************************************************
 * 描  述 : ADC中断处理函数
 * 参   数: 无
 * 返回值 : 无
 ***********************************************************************************************/ 
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
     
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
		batt_vol  = 2*6*(p_event->data.done.p_buffer[0])*600/1024;
		usb_vol  = 2*6*(p_event->data.done.p_buffer[1])*600/1024;
		printk("%d:saadc0=%d\r\n",NRF_RTC1->COUNTER,batt_vol);
		printk("%d:saadc1=%d\r\n",NRF_RTC1->COUNTER,usb_vol);
        printk("saadc0=%d\r\n", batt_vol);//buf[0] for battery
		printk("saadc1=%d\r\n", usb_vol);//buf[1] for USB
    }
	//if(received_blue == myaddr[0])LED_P20x40Char(54, 3, received_blue%10, 1); 
	//else LED_P20x40Char(54, 3, 11, 1);
	if(finger_result.mathscore > 20)LED_P20x40Char(54, 3, finger_result.pageID%10+1, 1); 
	else LED_P20x40Char(54, 3, 11, 1);
	oled_signal_display(4);
	if(usb_vol > 3500)oled_battery_statues_display(5);
	else oled_battery_statues_display(5000*(unsigned int)(batt_vol-3000)/1200);

	printk("myaddr:%d\r\n",myaddr[0]);
	//NRF_LOG_RAW_INFO("V%d", p_event->data.done.p_buffer[0]);
}

void saadc_init(void)
{
    ret_code_t err_code;
#if 0
    //nrf_saadc_channel_config_t channel_0_config =
	//NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
	//channel_0_config.gain = SAADC_CH_CONFIG_GAIN_Gain1_2;
    //Configure SAADC channel
    nrf_saadc_channel_config_t channel_0_config;
	nrf_saadc_channel_config_t channel_1_config;
	nrf_saadc_channel_config_t channel_2_config;
	nrf_saadc_channel_config_t channel_3_config;
	nrf_drv_saadc_config_t saadc_config;
	channel_0_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_0_config.gain = NRF_SAADC_GAIN1_2;                                   			//Set input gain to 1/2. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
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
    channel_1_config.gain = NRF_SAADC_GAIN1_2;                                   			//Set input gain to 1/2. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
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
    channel_2_config.gain = NRF_SAADC_GAIN1_6;                                   			//Set input gain to 1/2. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_2_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_2_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_2_config.pin_p = NRF_SAADC_INPUT_AIN2;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_2_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_2_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_2_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin	
	err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);

    //nrf_saadc_channel_config_t channel_3_config =
	//NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
	channel_3_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_3_config.gain = NRF_SAADC_GAIN1_6;                                   			//Set input gain to 1/2. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_3_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_3_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_3_config.pin_p = NRF_SAADC_INPUT_AIN3;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_3_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_3_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_3_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin		
	err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);

	//nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    //Configure SAADC
    saadc_config.low_power_mode = true;                                                   	//Enable low power mode.false true
    saadc_config.resolution = NRF_SAADC_RESOLUTION_10BIT;                                 	//Set SAADC resolution to 10-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;                         		//Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = SAADC_CONFIG_IRQ_PRIORITY;  	
	err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);

	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
	//err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
	//err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[2],SAMPLES_IN_BUFFER);
	//err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[3],SAMPLES_IN_BUFFER);
	NRF_LOG_RAW_INFO("Function: %s error: %d\r\n",(uint32_t)__func__, err_code);
#else
    nrf_saadc_channel_config_t channel_0_config;
	nrf_saadc_channel_config_t channel_1_config;
	nrf_drv_saadc_config_t saadc_config;
	channel_0_config.reference = NRF_SAADC_REFERENCE_INTERNAL;                              //Set internal reference of fixed 0.6 volts
    channel_0_config.gain = NRF_SAADC_GAIN1_6;                                   			//Set input gain to 1/2. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
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
    channel_1_config.gain = NRF_SAADC_GAIN1_6;                                   			//Set input gain to 1/2. The maximum SAADC input voltage is then 0.6V/(1/6)=3.6V. The single ended input range is then 0V-3.6V
    channel_1_config.acq_time = NRF_SAADC_ACQTIME_10US;                                     //Set acquisition time. Set low acquisition time to enable maximum sampling frequency of 200kHz. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS. 
    channel_1_config.mode = NRF_SAADC_MODE_SINGLE_ENDED;                                    //Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
    channel_1_config.pin_p = NRF_SAADC_INPUT_AIN1;                                          //Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
    channel_1_config.pin_n = NRF_SAADC_INPUT_DISABLED;                                      //Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
    channel_1_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pullup resistor on the input pin
    channel_1_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              //Disable pulldown resistor on the input pin		
	err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);

	    //Configure SAADC
    saadc_config.low_power_mode = false;                                                   	//Enable low power mode.false true
    saadc_config.resolution = NRF_SAADC_RESOLUTION_10BIT;                                 	//Set SAADC resolution to 10-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=2048 (when input voltage is 3.6V for channel gain setting of 1/6).
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;                         		//Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.
    saadc_config.interrupt_priority = SAADC_CONFIG_IRQ_PRIORITY;  	
	err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
	
	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
	//err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
	
	err_code = err_code;
#endif
//NRF_SAADC->ENABLE = 0x01;
	//err_code = nrf_drv_saadc_init(NULL, saadc_callback);
	//APP_ERROR_CHECK(err_code);
	// Create timers.
	err_code = app_timer_create(&m_volmeas_timer_id, APP_TIMER_MODE_REPEATED, volmeas_timeout_handler);
	//APP_ERROR_CHECK(err_code);
    // Start application timers.
    err_code = app_timer_start(m_volmeas_timer_id, VOLMEAS_READ_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);	
	adc_value.cnt = 0;
}

/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
	//sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

static void pa_assist(uint32_t gpio_pa_pin,uint32_t gpio_lna_pin)
{
	ret_code_t err_code;
	static const uint32_t gpio_toggle_ch = 0;
	static const uint32_t ppi_set_ch = 0;
	static const uint32_t ppi_clr_ch = 1;

	ble_opt_t opt;
	memset (&opt,0,sizeof (ble_opt_t));

	opt.common_opt .pa_lna .gpiote_ch_id = gpio_toggle_ch ;
	opt.common_opt .pa_lna .ppi_ch_id_clr = ppi_set_ch ;
	opt.common_opt .pa_lna .ppi_ch_id_set = ppi_clr_ch ;

	opt.common_opt .pa_lna .pa_cfg .active_high = 1;
	opt.common_opt .pa_lna .pa_cfg .enable = 1;
	opt.common_opt .pa_lna .pa_cfg .gpio_pin = gpio_pa_pin;
	opt.common_opt .pa_lna .lna_cfg .active_high = 1;
	opt.common_opt .pa_lna .lna_cfg .enable = 1;
	opt.common_opt .pa_lna .lna_cfg .gpio_pin = gpio_lna_pin;

	err_code = sd_ble_opt_set (BLE_COMMON_OPT_PA_LNA ,&opt);
	APP_ERROR_CHECK(err_code);
}

void get_mac_addr(void)
{
	uint32_t error_code;
	uint8_t *p_mac_addr;
	uint8_t i;
	
	ble_gap_addr_t *p_mac_addr_t = (ble_gap_addr_t*)malloc(sizeof(ble_gap_addr_t));
	uint8_t *d = p_mac_addr_t->addr;
	error_code = sd_ble_gap_addr_get(p_mac_addr_t);
	APP_ERROR_CHECK(error_code);
	for(i = 6;i > 0;)
	{
		i--;
		p_mac_addr[5-i] = d[i];
		p_mac_addr_t->addr[i] = myaddr[5-i];
	}
	error_code = sd_ble_gap_addr_set((ble_gap_addr_t const *)p_mac_addr_t);
	APP_ERROR_CHECK(error_code);
	free(p_mac_addr_t);
	p_mac_addr_t = NULL;
}

void gpio_init(void)
{
	//nrf_gpio_cfg_output(PIN_OTG_EN);
	//nrf_gpio_pin_clear(PIN_OTG_EN);
	//nrf_gpio_cfg_input(PIN_KEY_IN, NRF_GPIO_PIN_PULLUP);
	//nrf_gpio_cfg_output(PIN_CHG_EN1);
	//nrf_gpio_pin_clear(PIN_CHG_EN1);
	//nrf_gpio_cfg_output(PIN_CHG_EN2);
	//nrf_gpio_pin_clear(PIN_CHG_EN2);	
}


/**@snippet [UART Initialization] */
/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    
    // Initialize.
    leds_init();
    timers_init();
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    buttons_init();
	OLED_Init();
	//uart_init();
	//pwm_init();
	uart_init(TX_PIN_NUMBER, RX_PIN_NUMBER, UART_BAUDRATE_BAUDRATE_Baud115200);
    ble_stack_init();	//协议栈初试化，设置时钟，demo里面设置为外部时钟。并且注册事件派发函数
	pa_assist(24, 20);
    gap_params_init();	//GAP一些参数的设置，设置设备名，设置PPCP(外围设备首选链接参数)
    get_mac_addr();
	services_init();	//服务初始化。添加uart的串口服务。主要提供两个特征值来供手机和板子以及电脑的通信
	advertising_init();	//设置广播数据以及扫描响应数据
    conn_params_init();	//链接参数设置。主要设置什么时候发起更新链接参数请求以及间隔和最大尝试次数。
	//printk("\r\nUART Start!\r\n");
    // Start execution.
    //NRF_LOG_INFO("Blinky Start!\r\n");
    advertising_start();//设置广播类型，白名单，间隔，超时等特性。并开始广播。	
	LED_P20x40Char(54, 3, myaddr[0]%10, 1);
    saadc_init();
	received_blue = myaddr[0];
	// Enter main loop.
    for (;;)
    {
        //if (NRF_LOG_PROCESS() == false)
        //{
            power_manage();
		//sleep_mode_enter();
        //}
    }
}

/******************************************************
		??????????
		char *itoa(int value, char *string, int radix)
		radix=10 ???10??	????,?????0;

		?:d=-379;
		??	itoa(d, buf, 10); ?

		buf="-379"
**********************************************************/
char *itoa(int value, char *string, int radix)
{
	int i = 0, j = 0;
	int data = value;
	char temp[16];
	char MinusFlag;
	int pos;

	if (radix != 16 && radix != 10)
	{
		string[j] = 0;
		return 0;
	}

	MinusFlag = (radix == 10 && value < 0);
	if (MinusFlag)
	{
		data = -data;
	}
	else
	{
		data = value;
	}

	do
	{
		pos = data % radix;
		data /= radix;
		if (pos < 10)
		{
			temp[i] = pos + '0';
		}
		else
		{
			temp[i] = pos + 'a' - 10;
		}
		i++;
	}while (data > 0);

	if (MinusFlag)
	{
		temp[i++] = '-';
	}
	temp[i] = 0;
	i--;

	while (i >= 0)
	{
		string[j] = temp[i];
		j++;
		i--;
	}
	string[j] = 0;

	return string;

} /* NCL_Itoa */

void uartSendByte(char byte)
{
	NRF_UART0->EVENTS_TXDRDY = 0;
	NRF_UART0->TXD = byte;
	while(NRF_UART0->EVENTS_TXDRDY == 0);
}

void USART_PRINT(char *Data, va_list ap)
{
	const char *s;
	int d;
	char buf[100];
	unsigned char prefix = 0x20;

	while (*Data != 0)
	{                                         //????????????
		if (*Data == 0x5c)                       //'\'
		{
			switch (*++Data)
			{
				case 'r':	
					uartSendByte(0x0d);//USART_SendData( 0x0d);
					Data++;
				break;
				case 'n':							          //???
					app_uart_put(0x0a);//USART_SendData(0x0a);
					Data++;
				break;
				case 0x22://" " "
					app_uart_put(0x22);
					Data++;
				break;
				case 0x27://" ' "
					app_uart_put(0x27);
					Data++;
				break;				
				case 0x5c://" / "	
					app_uart_put(0x5c);
					Data++;
				break;				
				default:
					Data++;
				break;
			}
		}
		else if (*Data == '%')
		{
			switch (*++Data)
			{
				case 's':       //???
					s = va_arg(ap, const char*);
					for (; *s; s++)
					{
						app_uart_put(*s);
					}
					Data++;
					break;
				case 'd':   //???
					d = va_arg(ap, int);
					itoa(d, buf, 10);
					for (s = buf; *s; s++)
					{
						app_uart_put(*s);
					}
					Data++;
					break;
				case 'f':	//???
				case 'F':
					//f = va_arg(ap, double);
					//ftoa(f, buf);
					for (s = buf; *s; s++)
					{
						app_uart_put(*s);
					}
					Data++;
				break;					
				case 'x':       //????
				case 'X':       //????
					d = va_arg(ap, int);
					itoa(d, buf, 16);
					for (s = buf; *s; s++)
					{
						app_uart_put(*s);
					}
					Data++;
					break;
				case '0':
					prefix = '0';
					Data++;
				case '1':
				case '2':
				case '3':
				case '4':
				case '5':
				case '6':
				case '7':
				case '8':
				case '9':
				{
					int outlen = *Data - '0';
					int len;
					int radix;
					int i;

					Data++;
					if (*Data == 'd' || *Data == 'x' || *Data == 'X')
					{
						if (*Data == 'd')
						{
							radix = 10;
						}
						else
						{
							radix = 16;
						}
						d = va_arg(ap, int);
						itoa(d, buf, radix);
						len = strlen(buf);
						s = buf;
						if (outlen >= len)
						{
							for (i = 0; i < outlen - len; i++)
							{
								app_uart_put(*s);
							}
						}
						else
						{
							for (i = 0; i < len - outlen; i++)
							{
								s++;
							}
						}
						for (; *s; s++)
						{
							app_uart_put(*s);
						}
						prefix = 0x20;
					}
					Data++;
				}
					break;
				default:
					Data++;
					break;
			}
		}
		else
		{
			app_uart_put(*Data++);
		}
		prefix = prefix;
	}
}


/**
 * @}
 */
