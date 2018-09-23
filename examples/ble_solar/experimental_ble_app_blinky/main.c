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
#include "nrf_drv_gpiote.h"
#include "app_uart.h"

#include "..\prink\printk.h"
#include "solar.h"

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define ADVERTISING_LED_PIN             BSP_BOARD_LED_0                             /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN               BSP_BOARD_LED_1                             /**< Is on when device has connected. */

#define LEDBUTTON_LED_PIN               BSP_BOARD_LED_2                             /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON_PIN            15                                			/**< Button that will trigger the notification event with the LED Button Service */
#define AS608_PS_STA_PIN				14
#define DEVICE_NAME                     "Solar"                             		/**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                320                                          /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
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

APP_TIMER_DEF(stamp_timer_id);     
#define STAMP_CNT_INTERVAL           	APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_lbs_t                        m_lbs;   
#define SAMPLES_IN_BUFFER 				3             								/**< ADC采样数据缓存大小(字节数)  */
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */
unsigned char oled_busy = 0;

#define PIN_OTG_EN			7
#define PIN_KEY_IN			5
#define PIN_SEN_IN			22

#define PIN_OLED_PWR		OLED_EN
#define PIN_OLED_DC			OLED_DC
#define PIN_OLED_RST		OLED_RST
#define PIN_OLED_CS			OLED_CS
#define PIN_SPI_MISO		SPI0_CONFIG_MISO_PIN
#define PIN_SPI_SCK			SPI0_CONFIG_SCK_PIN
#define PIN_SPI_MOSI		SPI0_CONFIG_MOSI_PIN
#define PIN_I2C_SDA			23
#define PIN_I2C_SCL			22
#define PIN_FLASH_CS		13

#define PIN_SOLAR_EN		12
#define PIN_SOLAR_VOL1		26
#define PIN_SOLAR_VOL2		25

#define PIN_RXD				8
#define PIN_TXD				6
#define PIN_LED2			10
#define PIN_LED1			9

#define PIN_AIN2			4	//Charge Current
#define PIN_AIN1			3	//Batt Voltage
#define PIN_AIN0			2	//Solar Voltage


static uint8_t keycnt = 0;
uint32_t timestamp = 0;
extern int32_t tickcnt;

uint8_t myaddr[BLE_GAP_ADDR_LEN] = {0xc5,0x15,0x83,0xbc,0xd2,0x6a};
extern uint8_t rssi;
extern uint8_t received_blue;

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
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
			
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
    printk("app_error_handler-%d-%ds\r\n",line_num, p_file_name);
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static void stamp_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	timestamp ++;	
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
	uint32_t err_code;
	
    //Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	//Create timers.
	err_code = app_timer_create(&stamp_timer_id, APP_TIMER_MODE_REPEATED, stamp_timeout_handler);
	APP_ERROR_CHECK(err_code);	
    err_code = app_timer_start(stamp_timer_id, STAMP_CNT_INTERVAL, NULL);	
	//err_code = 0;
	NRF_POWER->DCDCEN = 1;
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




static void lbs_handler(ble_lbs_t * p_lbs, uint8_t * p_data, uint16_t length)
{
	int8_t rssi;
	sd_ble_gap_rssi_get(p_lbs->conn_handle, &rssi);
	printk("%d:rssi:%d,",timestamp,rssi);
	if(length > 0)
	{
		for (uint32_t i = 0; i < length; i++)
		{
			printk("%d,",p_data[i]);

		}
		keycnt = p_data[0];
		if(keycnt > 3)keycnt = 0;
		printk("\r\n");
	}
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_lbs_init_t init;

    init.lbs_data_handler = lbs_handler;

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
    //bsp_board_led_on(ADVERTISING_LED_PIN);
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
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
			sd_ble_gap_rssi_start(m_conn_handle, BLE_GAP_RSSI_THRESHOLD_INVALID, 0);
            printk("Connected 1 %d\r\n",m_conn_handle);
			break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            printk("Disconnected\r\n");
            //bsp_board_led_off(CONNECTED_LED_PIN);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
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

#define NRF_CLOCK_LFCLK      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLK;

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
        case PIN_KEY_IN:				
			if(button_action == 0)
			{
                keycnt ++;
				if(keycnt > 3)keycnt = 0;
				//buf[0] = keycnt	;		
				//err_code = ble_lbs_string_send(&m_lbs, buf, 5);
				printk("ble_lbs_string_send:%d\r\n",keycnt);
				//if (err_code != NRF_ERROR_INVALID_STATE)
				//{
				//	APP_ERROR_CHECK(err_code);
				//}
			}
			else
			{	
				//buf[0] = PIN_KEY_IN;
				//nrf_gpio_pin_set(PIN_LED2);	
				//LED_Printint(0,0,0);				
				//printk("LEDBUTTON_BUTTON_PIN:%d\r\n",button_action);	
			}			
        break;
        default:
            //APP_ERROR_HANDLER(pin_no);
        break;
    }
	printk("%d:button_event_handler:%d-%d\r\n",app_timer_cnt_get(),pin_no,keycnt);	
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    uint32_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {PIN_KEY_IN, false, NRF_GPIO_PIN_PULLUP, button_event_handler}//,
		//{PIN_SEN_IN, false, NRF_GPIO_PIN_PULLUP, button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
	err_code = app_button_enable();
	APP_ERROR_CHECK(err_code);
}



/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code;
	//__set_FPSCR(__get_FPSCR() & ~(FPU_EXCEPTION_MASK));
	//(void) __get_FPSCR();
	//NVIC_ClearPendingIRQ(FPU_IRQn);
	err_code = sd_app_evt_wait();
	//sd_power_system_off();
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
	//GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);   //低功耗低精度IO口中断模式
	nrf_gpio_cfg_output(PIN_SOLAR_EN);
	nrf_gpio_pin_clear(PIN_SOLAR_EN);
	nrf_gpio_cfg_output(PIN_SOLAR_VOL1);
	nrf_gpio_pin_set(PIN_SOLAR_VOL1);
	nrf_gpio_cfg_output(PIN_SOLAR_VOL2);
	nrf_gpio_pin_clear(PIN_SOLAR_VOL2);	
	nrf_gpio_cfg_output(PIN_OTG_EN);
	nrf_gpio_pin_set(PIN_OTG_EN);	

	nrf_gpio_cfg_output(PIN_FLASH_CS);//27K
	nrf_gpio_pin_set(PIN_FLASH_CS);		
	
	nrf_gpio_cfg_output(PIN_LED1);//27K
	nrf_gpio_pin_clear(PIN_LED1);		
	nrf_gpio_cfg_output(PIN_LED2);//27K
	nrf_gpio_pin_clear(PIN_LED2);			
}
#if 0
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
#endif
uint32_t ble_send_solardata(int32_t value1, int32_t value2)	
{
	uint32_t err_code;
	uint8_t buf[9];
	buf[0] = (value1 >>  0) & 0xff;
	buf[1] = (value1 >>  8) & 0xff;
	buf[2] = (value1 >> 16) & 0xff;
	buf[3] = (value1 >> 24) & 0xff;
	buf[4] = (value2 >>  0) & 0xff;
	buf[5] = (value2 >>  8) & 0xff;
	buf[6] = (value2 >> 16) & 0xff;
	buf[7] = (value2 >> 24) & 0xff;
	buf[8] = 2;
	err_code = ble_lbs_string_send(&m_lbs, buf, 9);
	return err_code;
}

/**@snippet [UART Initialization] */
/**@brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    uint32_t tickcnt_old;
    // Initialize.
    //leds_init();
    timers_init();
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    buttons_init();
	gpio_init();
	OLED_Init();
	//uart_init();
	//pwm_init();
	uart_init(PIN_TXD, PIN_RXD, UART_BAUDRATE_BAUDRATE_Baud115200);
    ble_stack_init();	//协议栈初试化，设置时钟，demo里面设置为外部时钟。并且注册事件派发函数
	//pa_assist(24, 20);
    gap_params_init();	//GAP一些参数的设置，设置设备名，设置PPCP(外围设备首选链接参数)
    get_mac_addr();
	services_init();	//服务初始化。添加uart的串口服务。主要提供两个特征值来供手机和板子以及电脑的通信
	advertising_init();	//设置广播数据以及扫描响应数据
    conn_params_init();	//链接参数设置。主要设置什么时候发起更新链接参数请求以及间隔和最大尝试次数。
	//err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle , 4);
    APP_ERROR_CHECK(err_code);
	//printk("\r\nUART Start!\r\n");
    // Start execution.
    //NRF_LOG_INFO("Blinky Start!\r\n");
    advertising_start();//设置广播类型，白名单，间隔，超时等特性。并开始广播。	
	//LED_P20x40Char(54, 3, myaddr[0]%10, 1);
    solar_init();
	received_blue = myaddr[0];
	// Enter main loop.
    for (;;)
    {
        //if (NRF_LOG_PROCESS() == false)
        //{
        power_manage();
		//sleep_mode_enter();
        //}
		if(tickcnt != tickcnt_old)
		{
			solar_run(keycnt);
		}
		tickcnt_old = tickcnt;
    }
}
/**
 * @}
 */
