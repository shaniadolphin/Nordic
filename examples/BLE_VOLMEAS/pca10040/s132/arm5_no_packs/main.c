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
#include "bsp.h"
#include "ble_gap.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_spi.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "ble_volmeas.h"
//#include "..\oled\oled.h"

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define ADVERTISING_LED_PIN             BSP_BOARD_LED_0                             /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN               BSP_BOARD_LED_1                             /**< Is on when device has connected. */

#define LEDBUTTON_LED_PIN               BSP_BOARD_LED_2                             /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_0                                /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "Solar_test_board"                                  /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                3200                                          /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED       /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            6                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define VOLMEAS_READ_INTERVAL           APP_TIMER_TICKS(200, APP_TIMER_PRESCALER)  /**< ADC采样间隔 1000ms读取一次*/


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

#define OLED_DISPLAY_INTERVAL			APP_TIMER_TICKS(200, APP_TIMER_PRESCALER)  /**< OLED显示间隔 1000ms读取一次*/
APP_TIMER_DEF(m_oled_timer_id);      	/**< OLED显示定时器 */

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */


static ble_volmeass_t                   m_volmeass; 

#define SAMPLES_IN_BUFFER 				4             /**< ADC采样数据缓存大小(字节数)  */

APP_TIMER_DEF(m_volmeas_timer_id);      /**< adc采样定时器 */
static nrf_saadc_value_t       m_buffer_pool[4][SAMPLES_IN_BUFFER];

short batt_vol;
short usb_vol;
short otg1_vol;
short otg2_vol;
unsigned char sendflag = 0;
unsigned char sendbits = 0;
unsigned char senddata = 0x55;
#define LEADLENGTH			5
adc_value_t adc_value;
#define PIN_OTG_EN			19
#define PIN_CHG_EN			16
#define PIN_CHG_MODE		15
#define PIN_CHG_IN			14
#define PIN_TX_EN			12
#define PIN_RX_EN			11
#define PIN_KEY_IN1			 9
#define PIN_KEY_IN2			10
#define PIN_CHG_SET0		19
#define PIN_CHG_SET1		18
#define PIN_CHG_SET2		17
#define PIN_HV_DIS			 7
#define PIN_DEV_PWR			 5


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

ret_code_t nrf_drv_oled_display(void)
{
    ret_code_t err_code = NRF_SUCCESS;
	nrf_gpio_pin_toggle(LED_2);
#if 0	
	nrf_gpio_pin_toggle(PIN_OTG_EN);
	//nrf_gpio_pin_set(PIN_OTG_EN);
	//LCD_P8x16Str(0,6,"program start");	
    //NRF_LOG_INFO("Function: %s, error code: %s.\r\n", (uint32_t)__func__, (uint32_t)ERR_TO_STR(err_code));
#else	
	if(sendflag)
	{
		if(sendbits < LEADLENGTH)
		{
			if(sendbits < (LEADLENGTH - 1))
			{
				nrf_gpio_pin_set(PIN_OTG_EN);
				NRF_LOG_RAW_INFO("sendbit%2d:----1\r\n", sendbits);//buf[1] for USB		
			}
			else
			{				
				nrf_gpio_pin_clear(PIN_OTG_EN);
				NRF_LOG_RAW_INFO("sendbit%2d:----0\r\n", sendbits);//buf[1] for USB		
			}
			sendbits ++;
		}
		else
		{
			if(senddata & (0x01 << (sendbits - LEADLENGTH))) 
			{
				nrf_gpio_pin_set(PIN_OTG_EN);
				NRF_LOG_RAW_INFO("sendbit%2d:----1\r\n", sendbits);//buf[1] for USB	
			}
			else
			{
				nrf_gpio_pin_clear(PIN_OTG_EN);
				NRF_LOG_RAW_INFO("sendbit%2d:----0\r\n", sendbits);//buf[1] for USB	
			}
			sendbits ++;
			if(sendbits >  (LEADLENGTH + 8))
			{
				nrf_gpio_pin_set(PIN_OTG_EN);
				NRF_LOG_RAW_INFO("sendbit%2d:----1\r\n", sendbits);//buf[1] for USB	
				sendbits = 0;
				sendflag = 0;
			}
		}
	}
#endif	
    return err_code;
}

static void oled_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
    APP_ERROR_CHECK(nrf_drv_oled_display());//adc采样
}


static void volmeas_timeout_handler(void * p_context)
{
	  UNUSED_PARAMETER(p_context);
    APP_ERROR_CHECK(nrf_drv_saadc_sample());//adc采样
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
	err_code = app_timer_create(&m_volmeas_timer_id,
				APP_TIMER_MODE_REPEATED,
				volmeas_timeout_handler);
	APP_ERROR_CHECK(err_code);
	
	// Create timers.
	err_code = app_timer_create(&m_oled_timer_id,
				APP_TIMER_MODE_REPEATED,
				oled_timeout_handler);
	APP_ERROR_CHECK(err_code);	
}

static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_volmeas_timer_id, VOLMEAS_READ_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    // Start application timers.
    err_code = app_timer_start(m_oled_timer_id, OLED_DISPLAY_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);	
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
	
    ble_uuid_t adv_uuids[] = {{VOLMEASS_UUID_SERVICE, m_volmeass.uuid_type}};
	NRF_LOG_INFO("advertising_init\r\n");
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

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_volmeass_init_t init;

    err_code = ble_volmeass_init(&m_volmeass, &init);
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
            NRF_LOG_INFO("Connected\r\n");
            //bsp_board_led_on(CONNECTED_LED_PIN);
            //bsp_board_led_off(ADVERTISING_LED_PIN);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected\r\n");
            //bsp_board_led_off(CONNECTED_LED_PIN);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

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
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
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
    ble_volmeass_on_ble_evt(&m_volmeass, p_ble_evt);
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

    switch (pin_no)
    {
        case LEDBUTTON_BUTTON_PIN:
			//NRF_LOG_RAW_INFO("button action %d\r\n", button_action);//buf[0] for battery
			if(button_action   == 1)
			{
				//button_action = 0;
				NRF_LOG_RAW_INFO("button action %d\r\n", button_action);//buf[0] for battery
				sendflag = 1;
				senddata = 0x5a;
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
        {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
	
	//err_code = bsp_event_to_button_action_assign(LEDBUTTON_BUTTON_PIN, BSP_BUTTON_ACTION_LONG_PUSH, BSP_EVENT_KEY_4);	
	err_code = app_button_enable();
	APP_ERROR_CHECK(err_code);	
}


/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
	//nrf_pwr_mgmt_run();
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
		otg1_vol  = 2*5*(p_event->data.done.p_buffer[0])*600/1024;//1024/235;
		otg2_vol  = 6*4*(p_event->data.done.p_buffer[1])*600/1024;//1024/235;
		//batt_vol  = 3*6*(p_event->data.done.p_buffer[2])*600/1024;
		//usb_vol   = 2*6*(p_event->data.done.p_buffer[3])*600/1024;		
        NRF_LOG_RAW_INFO("saadc0=%d\r\n", otg1_vol);//buf[0] for battery
		NRF_LOG_RAW_INFO("saadc1=%d\r\n", otg2_vol);//buf[1] for USB
        NRF_LOG_RAW_INFO("saadc2=%d\r\n", batt_vol);//buf[0] for battery
		NRF_LOG_RAW_INFO("saadc3=%d\r\n", usb_vol);//buf[1] for USB		
    }
	//oled_signal_display(4);
	//oled_battery_statues_display(5);
	//NRF_LOG_INFO("V%d", p_event->data.done.p_buffer[0]);
}

void saadc_init(void)
{
    ret_code_t err_code;
#if 0
	NRF_SAADC->RESOLUTION = (3 << 0); 		//adc转换精度为14位 
	NRF_SAADC->SAMPLERATE = 
	  (2000 << 0)				//Capture and compare value. Sample rate is 16 MHz/2000
	| (1<<12);					//Rate is controlled from local timer (use CC to control the rate)
	//NRF_SAADC->RESULT.PTR  = m_buffer_pool[0];	
	//NRF_SAADC->RESULT.MAXCNT  = 1;
	nrf_saadc_buffer_init(m_buffer_pool[0],SAMPLES_IN_BUFFER);

	NRF_SAADC->CH[0].PSELP = 1;	//AIN0
	NRF_SAADC->CH[0].PSELN = 0;	//Not connected
	NRF_SAADC->CH[0].CONFIG =
	  (0<<0)					//RESP:Bypass 0 Bypass resistor ladder	
	| (0<<4)					//RESN:Bypass 0 Bypass resistor ladder	
	| (4<<8)					//GAIN:Gain1_2 4 1/2
	| (0<<12)					//REFSEL:Internal 0 Internal reference (0.6 V)
	| (2<<16)					//TACQ:10us 2 10 us
	| (0<<20);					//MODE:Single ended, PSELN will be ignored, negative input to ADC
	NRF_SAADC->CH[1].PSELP = 2;	//AIN0
	NRF_SAADC->CH[1].PSELN = 0;	//Not connected
	NRF_SAADC->CH[1].CONFIG =
	  (0<<0)					//RESP:Bypass 0 Bypass resistor ladder	
	| (0<<4)					//RESN:Bypass 0 Bypass resistor ladder	
	| (4<<8)					//GAIN:Gain1_2 4 1/2
	| (0<<12)					//REFSEL:Internal 0 Internal reference (0.6 V)
	| (2<<16)					//TACQ:10us 2 10 us
	| (0<<20);					//MODE:Single ended, PSELN will be ignored, negative input to ADC
#else
    nrf_saadc_channel_config_t channel_0_config = 
	NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0); 
	channel_0_config.gain = SAADC_CH_CONFIG_GAIN_Gain1_5;		
	err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    
	nrf_saadc_channel_config_t channel_1_config = 
	NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
	channel_1_config.gain = SAADC_CH_CONFIG_GAIN_Gain1_4;			
	err_code = nrf_drv_saadc_channel_init(1, &channel_1_config);
	
    //nrf_saadc_channel_config_t channel_2_config = 
	//NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
	//err_code = nrf_drv_saadc_channel_init(2, &channel_2_config);	
	
    //nrf_saadc_channel_config_t channel_3_config = 
	//NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
	//err_code = nrf_drv_saadc_channel_init(3, &channel_3_config);	
	
	nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
	err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
	
	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0],SAMPLES_IN_BUFFER);
	err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1],SAMPLES_IN_BUFFER);
	//err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[2],SAMPLES_IN_BUFFER);	
	//err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[3],SAMPLES_IN_BUFFER);
	
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

void gpio_init(void)
{
	nrf_gpio_cfg_output(PIN_OTG_EN);
	nrf_gpio_pin_clear(PIN_OTG_EN);
	
	nrf_gpio_cfg_input(PIN_KEY_IN1, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(PIN_KEY_IN2, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(PIN_CHG_IN,	NRF_GPIO_PIN_PULLUP);

	nrf_gpio_cfg_output(PIN_CHG_EN);
	nrf_gpio_pin_set(PIN_CHG_EN);//nrf_gpio_pin_set(PIN_CHG_EN1);//

	nrf_gpio_cfg_output(PIN_CHG_MODE);
	nrf_gpio_pin_clear(PIN_CHG_MODE);//nrf_gpio_pin_clear(PIN_CHG_EN2);//	

	nrf_gpio_cfg_output(PIN_TX_EN);
	nrf_gpio_pin_set(PIN_TX_EN);
	
	nrf_gpio_cfg_output(PIN_RX_EN);
	nrf_gpio_pin_clear(PIN_RX_EN);	
	
	nrf_gpio_cfg_output(PIN_CHG_SET0);//27K
	nrf_gpio_pin_set(PIN_CHG_SET0);		
	nrf_gpio_cfg_output(PIN_CHG_SET1);//10K
	nrf_gpio_pin_clear(PIN_CHG_SET1);	
	nrf_gpio_cfg_output(PIN_CHG_SET2);//4.7K
	nrf_gpio_pin_clear(PIN_CHG_SET2);	
	
	nrf_gpio_cfg_output(PIN_HV_DIS);
	nrf_gpio_pin_clear(PIN_HV_DIS);	
}

int main(void)
{
    ret_code_t err_code;
    
    // Initialize.
    leds_init();
	gpio_init();
    timers_init();
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    buttons_init();
    ble_stack_init();
	pa_assist(24, 20);
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
	//init_spi_master();
    saadc_init();
	//OLED_Init();
	//rtc_init();
	//NRF_LOG_INFO("saadc_init init ok\r\n");
	  // Start execution.
	application_timers_start();//启动定时器
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
