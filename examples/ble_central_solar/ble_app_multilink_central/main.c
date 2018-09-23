/*
 * Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

/**
 * @brief BLE LED Button Service central and client application main file.
 *
 * This example can be a central for up to 8 peripherals.
 * The peripheral is called ble_app_blinky and can be found in the ble_peripheral
 * folder.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "app_uart.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_lbs_c.h"
#include "ble_conn_state.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "..\oled\oled.h"

#include "nrf_drv_pwm.h"

#include "..\prink\printk.h"

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE      GATT_MTU_SIZE_DEFAULT                      /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define CENTRAL_LINK_COUNT        8                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT     0                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define TOTAL_LINK_COUNT          CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT /**< Total number of links used by the application. */

#define CENTRAL_SCANNING_LED      BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1

#define APP_TIMER_PRESCALER       0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS      (2 + BSP_APP_TIMERS_NUMBER)                  /**< Maximum number of timers used by the application. */
#define APP_TIMER_OP_QUEUE_SIZE   2                                          /**< Size of timer operation queues. */

#define SCAN_INTERVAL             0x0140//0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW               0x0050                                     /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT              0x0000                                     /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)           /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL   MSEC_TO_UNITS(30, UNIT_1_25_MS)            /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY             0                                          /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT       MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Determines supervision time-out in units of 10 milliseconds. */

#define UUID16_SIZE               2                                          /**< Size of a UUID, in bytes. */

#define LEDBUTTON_LED             BSP_BOARD_LED_1                            /**< LED to indicate a change of state of the the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON_PIN      BUTTON_STOP                               /**< Button that will write to the LED characteristic of the peer */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

static const char m_target_periph_name[] = "Solar";//"Nordic_Blinky";                  /**< Name of the device we try to connect to. This name is searched for in the scan report data*/

int32_t solar_vol1;
int32_t batt_vol1;
int32_t solar_vol2;
int32_t batt_vol2;
unsigned char dev_address[48];
unsigned char linkedcnt = 0;
uint32_t timestamp = 0;

//#define PIN_KEY_IN			14

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


#define PIN_OTG_EN			7
#define PIN_SEN_IN			22

#ifndef PTR9618PA
#define PIN_LED2			10
#define PIN_LED1			9
#define PIN_KEY_IN			5
#else
#define PIN_LED2			10
#define PIN_LED1			10
#define PIN_KEY_IN			14
#endif
#define PIN_AIN2			4	//Charge Current
#define PIN_AIN1			3	//Batt Voltage
#define PIN_AIN0			2	//Solar Voltage
#define LEDS_N  			4


const nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
nrf_pwm_values_individual_t m_demo1_seq_values[LEDS_N];
nrf_pwm_sequence_t const m_demo1_seq =
{
	.values.p_individual = m_demo1_seq_values,
	.length = NRF_PWM_VALUES_LENGTH(m_demo1_seq_values),
	.repeats = 0,
	.end_delay = 0
};


/** @brief Scan parameters requested for scanning and connection. */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 0,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,

    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif

    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist  = 0,
        .adv_dir_report = 0,
    #endif
};

/**@brief Connection parameters requested for connection. */
static const ble_gap_conn_params_t m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};

static ble_lbs_c_t        m_ble_lbs_c[TOTAL_LINK_COUNT];           /**< Main structures used by the LED Button client module. */
static uint8_t            m_ble_lbs_c_count;                       /**< Keeps track of how many instances of LED Button client module have been initialized. >*/
static ble_db_discovery_t m_ble_db_discovery[TOTAL_LINK_COUNT];    /**< list of DB structures used by the database discovery module. */

/**@brief Function to handle asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}



/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, uint8_array_t * p_advdata, uint8_array_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->size)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data = &p_data[index + 2];
            p_typedata->size   = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t ret;

    (void) sd_ble_gap_scan_stop();

    printk("Start scanning for device name %s.\r\n", (uint32_t)m_target_periph_name);
    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
}


/**@brief Handles events coming from the LED Button central module.
 *
 * @param[in] p_lbs_c     The instance of LBS_C that triggered the event.
 * @param[in] p_lbs_c_evt The LBS_C event.
 */
static void lbs_c_evt_handler(ble_lbs_c_t * p_lbs_c,const ble_lbs_c_evt_t * p_lbs_c_evt)
{
    uint32_t err_code;
	int32_t data[10];
	switch (p_lbs_c_evt->evt_type)
    {
        case BLE_LBS_C_EVT_DISCOVERY_COMPLETE:
        {
            //ret_code_t err_code;

            printk("LED Button service discovered on conn_handle 0x%x\r\n",
                    p_lbs_c_evt->conn_handle);

            //err_code = app_button_enable();
            //APP_ERROR_CHECK(err_code);

            // LED Button service discovered. Enable notification of Button.
            err_code = ble_lbs_c_button_notif_enable(p_lbs_c);
            APP_ERROR_CHECK(err_code);
        } 
		break; // BLE_LBS_C_EVT_DISCOVERY_COMPLETE

        case BLE_LBS_C_EVT_BUTTON_NOTIFICATION:
        {
			if((p_lbs_c_evt->data_len) > 8)
			{
				data[0] = p_lbs_c_evt->p_data[3];
				data[0] = (data[0] << 8) | p_lbs_c_evt->p_data[2];
				data[0] = (data[0] << 8) | p_lbs_c_evt->p_data[1];
				data[0] = (data[0] << 8) | p_lbs_c_evt->p_data[0];
				//data[0] = (int)data[0];
				data[1] = p_lbs_c_evt->p_data[7];
				data[1] = (data[1] << 8) | p_lbs_c_evt->p_data[6];
				data[1] = (data[1] << 8) | p_lbs_c_evt->p_data[5];
				data[1] = (data[1] << 8) | p_lbs_c_evt->p_data[4];
				if(p_lbs_c_evt->conn_handle == 0)
				{
					solar_vol1 = data[0];
					batt_vol1 = data[1];
				}	
				else if(p_lbs_c_evt->conn_handle == 1)
				{
					solar_vol2 = data[0];
					batt_vol2 = data[1];					
				}
				//data[1] = (int)data[1];	
			}

			//printk("Notification on conn_handle 0x%x:%d,%d\r\n",p_lbs_c_evt->conn_handle,data[0],data[1]);	
			//for (uint32_t i = 0; i < p_lbs_c_evt->data_len; i++)
            //{
			//	printk("%d ",p_lbs_c_evt->p_data[i]);
            //}
			//printk("%d,%d,%d,%d",app_timer_cnt_get(),p_lbs_c_evt->conn_handle,data[0],data[1]);timestamp
			printk("%d,%d,%d,%d,%d",timestamp,p_lbs_c_evt->conn_handle,data[0],data[1],p_lbs_c_evt->rssi);
			printk("\r\n");					
        } 
		break; // BLE_LBS_C_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
        break;
    }
}

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_adv_report(const ble_evt_t * const p_ble_evt)
{
    uint32_t      err_code;
    uint8_array_t adv_data;
    uint8_array_t dev_name;
    bool          do_connect = false;

    // For readibility.
    const ble_gap_evt_t * const p_gap_evt    = &p_ble_evt->evt.gap_evt;
    const ble_gap_addr_t  * const peer_addr  = &p_gap_evt->params.adv_report.peer_addr;
	


    // Initialize advertisement report for parsing
    adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data;
    adv_data.size   = p_gap_evt->params.adv_report.dlen;


    //search for advertising names
    bool found_name = false;
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                &adv_data,
                                &dev_name);

	if (err_code != NRF_SUCCESS)
    {
        // Look for the short local name if it was not found as complete
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            // If we can't parse the data, then exit
            return;
        }
        else
        {
            found_name = true;
        }
    }
    else
    {
        found_name = true;		
    }
    if (found_name)
    {
        if (strlen(m_target_periph_name) != 0)
        {
            if (memcmp(m_target_periph_name, dev_name.p_data, dev_name.size) == 0)
            {
#if 1
				ble_gap_evt_adv_report_t adv_report = p_ble_evt->evt.gap_evt.params.adv_report;
				printk("\r\n设备：");
				for(uint8_t i = 0;i<dev_name.size;i++)
				{
					printk("%c",dev_name.p_data[i]);
				}
				printk("\r\n地址:0x",adv_report.peer_addr.addr_type);
				for(uint8_t i = 0;i<6;i++)
				{
					printk("%02x ",adv_report.peer_addr.addr[i]);//dev_address[j*6+i]
				}
				printk("\r\nRSSI:%d\r\n",adv_report.rssi);
				//printf("",adv_report.type);
				//NRF_LOG_RAW_INFO("\r\nadv data:");
				for(uint8_t i = 0;i<adv_report.dlen;i++)
				{
					printk("%02x",adv_report.data[i]);
				}	
//#else
                do_connect = true;
#endif	
					
            }			
        }
	
    }

    if (do_connect)
    {
		// Initiate connection.
		//unsigned char i;		
        err_code = sd_ble_gap_connect(peer_addr, &m_scan_params, &m_connection_param);
        //NRF_LOG_INFO("sd_ble_gap_connect:%2x %2x %2x %2x %2x %2x",peer_addr->addr[0],peer_addr->addr[1],peer_addr->addr[2],
		//peer_addr->addr[3],peer_addr->addr[4],peer_addr->addr[5]);
		printk("%2x %2x %2x %2x %2x %2x\r\n",peer_addr[0].addr[0], peer_addr[0].addr[1],
		peer_addr[0].addr[2],peer_addr[0].addr[3],peer_addr[0].addr[4],
		peer_addr[0].addr[5]); 
		if (err_code != NRF_SUCCESS)
        {
            printk("Connection Request Failed, reason %d\r\n", err_code);
        }
    }
}

/**@brief Function for handling BLE Stack events concerning central applications.
 *
 * @details This function keeps the connection handles of central applications up-to-date. It
 *          parses scanning reports, initiating a connection attempt to peripherals when a
 *          target UUID is found, and manages connection parameter update requests. Additionally,
 *          it updates the status of LEDs used to report central applications activity.
 *
 * @note Since this function updates connection handles, @ref BLE_GAP_EVT_DISCONNECTED events
 *       should be dispatched to the target application before invoking this function.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(const ble_evt_t * const p_ble_evt)
{
    ret_code_t err_code;

    // For readability.
    const ble_gap_evt_t * const p_gap_evt = &p_ble_evt->evt.gap_evt;
    //const ble_gap_addr_t  * const peer_addr  = &p_gap_evt->params.adv_report.peer_addr;
    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral has connected, initiate DB
        // discovery, update LEDs status and resume scanning if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
			//unsigned char i;			
			//for(i=0;i<6;i++){dev_address[(p_gap_evt->conn_handle)*6+i] = peer_addr[p_gap_evt->conn_handle].addr[i];}
			//NRF_LOG_INFO("%2x %2x %2x %2x %2x %2x\r\n",dev_address[(p_gap_evt->conn_handle)*6+0], dev_address[(p_gap_evt->conn_handle)*6+1],
			//dev_address[(p_gap_evt->conn_handle)*6+2],dev_address[(p_gap_evt->conn_handle)*6+3],dev_address[(p_gap_evt->conn_handle)*6+4],
			//dev_address[(p_gap_evt->conn_handle)*6+5]);
			//NRF_LOG_INFO("%2x %2x %2x %2x %2x %2x\r\n",peer_addr[p_gap_evt->conn_handle].addr[0], peer_addr[p_gap_evt->conn_handle].addr[1],
			//peer_addr[p_gap_evt->conn_handle].addr[2],peer_addr[p_gap_evt->conn_handle].addr[3],peer_addr[p_gap_evt->conn_handle].addr[4],
			//peer_addr[p_gap_evt->conn_handle].addr[5]);  			
			linkedcnt ++;
			OLED_ShowChar(0,0,linkedcnt%10 + '0');
			sd_ble_gap_rssi_start(p_gap_evt->conn_handle, BLE_GAP_RSSI_THRESHOLD_INVALID, 0);
			printk("%d:Connection 0x%x established, starting DB discovery.\r\n",__LINE__,
                         p_gap_evt->conn_handle);
            APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < TOTAL_LINK_COUNT);

            err_code = ble_lbs_c_handles_assign(&m_ble_lbs_c[p_gap_evt->conn_handle],
                                                p_gap_evt->conn_handle,
                                                NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_ble_db_discovery[p_gap_evt->conn_handle],
                                              p_gap_evt->conn_handle);
            if (err_code != NRF_ERROR_BUSY)
            {
                APP_ERROR_CHECK(err_code);
            }

            // Update LEDs status, and check if we should be looking for more
            // peripherals to connect to.
            //bsp_board_led_on(CENTRAL_CONNECTED_LED);
            if (ble_conn_state_n_centrals() == CENTRAL_LINK_COUNT)
            {
                //bsp_board_led_off(CENTRAL_SCANNING_LED);
            }
            else
            {
                // Resume scanning.
                //bsp_board_led_on(CENTRAL_SCANNING_LED);
                scan_start();
            }
        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer which disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            uint32_t central_link_cnt; // Number of central links.
			linkedcnt --;
			OLED_ShowChar(0,0,linkedcnt%10 + '0' );
            printk("%d:LBS central link 0x%x disconnected (reason: 0x%x)\r\n",__LINE__,
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            //err_code = app_button_disable();
            //APP_ERROR_CHECK(err_code);

            // Start scanning
            scan_start();

            // Update LEDs status.
            //bsp_board_led_on(CENTRAL_SCANNING_LED);
            central_link_cnt = ble_conn_state_n_centrals();
            if (central_link_cnt == 0)
            {
                //bsp_board_led_off(CENTRAL_CONNECTED_LED);
            }
        } 
		break;

        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(p_ble_evt);
        break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // We have not specified a timeout for scanning, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                printk("Connection request timed out.\r\n");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
        {
            // Disconnect on GATT Client timeout event.
            printk("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT Server timeout event.
            printk("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

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
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 * been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    uint16_t conn_handle;
    conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_conn_state_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);

    // Make sure taht an invalid connection handle are not passed since
    // our array of modules is bound to TOTAL_LINK_COUNT.
    if (conn_handle < TOTAL_LINK_COUNT)
    {
        ble_db_discovery_on_ble_evt(&m_ble_db_discovery[conn_handle], p_ble_evt);
        ble_lbs_c_on_ble_evt(&m_ble_lbs_c[conn_handle], p_ble_evt);
    }
}


/**@brief LED Button collector initialization.
 */
static void lbs_c_init(void)
{
    uint32_t         err_code;
    ble_lbs_c_init_t lbs_c_init_obj;

    lbs_c_init_obj.evt_handler = lbs_c_evt_handler;

    for (m_ble_lbs_c_count = 0; m_ble_lbs_c_count < TOTAL_LINK_COUNT; m_ble_lbs_c_count++)
    {
        err_code = ble_lbs_c_init(&m_ble_lbs_c[m_ble_lbs_c_count], &lbs_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
    m_ble_lbs_c_count = 0;
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Use the max config: 8 central, 0 periph, 10 VS UUID
    ble_enable_params.common_enable_params.vs_uuid_count = 10;

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function to write to the LED characterestic of all connected clients.
 *
 * @details Based on if the button is pressed or released, we write a high or low LED status to
 *          the server.
 *
 * @param[in] button_action The button action (press/release).
 *            Determines if the LEDs of the servers will be ON or OFF.
 *
 * @return NRF_SUCCESS on success, else the error code from ble_lbs_led_status_send.
 */
static uint32_t master_cmd_send_to_all(uint8_t cmd, uint8_t data[4])
{
    uint32_t err_code;
	uint8_t buf[6];
	buf[0] = cmd;
	buf[1] = cmd;
	buf[2] = data[1];
	buf[3] = data[2];
	buf[4] = data[3];
	buf[5] = data[3];
    for (uint32_t i = 0; i< CENTRAL_LINK_COUNT; i++)
    {
        //err_code = ble_lbs_led_status_send(&m_ble_lbs_c[i], button_action);
        err_code = ble_lbs_mydata_send(&m_ble_lbs_c[i], &buf[0], 4);
		if (err_code != NRF_SUCCESS &&
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE)
        {
            return err_code;
        }
    }
    return NRF_SUCCESS;
}

uint8_t keycnt = 0;
/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;
	uint8_t cmddata[4];
	uint32_t clock;
    switch (pin_no)
    {
        //printk("BUTTON_PIN:%d\r\n",button_action);
		case PIN_KEY_IN:
			if(button_action == 0)
			{
				clock = app_timer_cnt_get();
				cmddata[0] = (clock >>  0) & 0xff;
				cmddata[1] = (clock >>  8) & 0xff;
				cmddata[2] = (clock >> 16) & 0xff;
				cmddata[3] = (clock >> 24) & 0xff;				
				keycnt ++;
				if(keycnt > 3)keycnt = 0;
				//button_action = 0;
				err_code = master_cmd_send_to_all(keycnt, cmddata);
				printk("%d:key pin %d = %d\r\n",clock, keycnt, err_code);
			}
				
            break;
		case 13:
			printk("key pin %d = %d\r\n",pin_no, button_action);
		break;
		case 12:
			printk("key pin %d = %d\r\n",pin_no, button_action);
		break;
		case 11:
			printk("key pin %d = %d\r\n",pin_no, button_action);
		break;
        default:
            //APP_ERROR_HANDLER(pin_no);
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
        {PIN_KEY_IN, false, NRF_GPIO_PIN_PULLUP, button_event_handler}//,
		//{PIN_SEN_IN, false, NRF_GPIO_PIN_PULLUP, button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
	err_code = app_button_enable();
	APP_ERROR_CHECK(err_code);
}

void pwm_event_handler(nrf_drv_pwm_evt_type_t event_type)
{
    //switch(event_type)
    if(event_type == NRF_DRV_PWM_EVT_FINISHED)
	{	
		//printk("NRF_DRV_PWM_EVT_FINISHED\r\n");	//printk("\r\nUART Start!\r\n");		
	}
	else if(event_type == NRF_DRV_PWM_EVT_END_SEQ0)
	{
		printk("NRF_DRV_PWM_EVT_END_SEQ0\r\n");
	}
	else if(event_type == NRF_DRV_PWM_EVT_END_SEQ1)
	{
		printk("NRF_DRV_PWM_EVT_END_SEQ1\r\n");
	}
	else if(event_type == NRF_DRV_PWM_EVT_STOPPED)
	{	
		printk("NRF_DRV_PWM_EVT_STOPPED\r\n");
	}
}

void pwm_init(void)
{
	#define PWM_REF_BP_PIN		18
	//#define PWM_REF_BN_PIN		18
	//#define PWM_REF_TEMP_PIN	19
	uint32_t err_code = NRF_SUCCESS;
	nrf_drv_pwm_config_t const config0 =
	{
		.output_pins =
		{
			PWM_REF_BP_PIN | NRF_DRV_PWM_PIN_INVERTED,//ch0
			NRF_DRV_PWM_PIN_NOT_USED,//PWM_REF_BN_PIN | NRF_DRV_PWM_PIN_INVERTED,//ch1
			NRF_DRV_PWM_PIN_NOT_USED,//PWM_REF_TEMP_PIN | NRF_DRV_PWM_PIN_INVERTED,//ch2
			NRF_DRV_PWM_PIN_NOT_USED//ch3
		},
		.irq_priority = APP_IRQ_PRIORITY_LOW,
		.base_clock = NRF_PWM_CLK_125kHz,
		.count_mode = NRF_PWM_MODE_UP,
		.top_value = 2500,   //freq = 500 000/10000 = 50Hz
		.load_mode = NRF_PWM_LOAD_INDIVIDUAL,
		.step_mode = NRF_PWM_STEP_AUTO
	};
	err_code = nrf_drv_pwm_init(&m_pwm0, &config0, pwm_event_handler);
	APP_ERROR_CHECK(err_code);
	for(uint32_t i = 0;i < LEDS_N;i++)
	{
		m_demo1_seq_values[i].channel_0 = 200;
		//m_demo1_seq_values[i].channel_1 = 5;
		//m_demo1_seq_values[i].channel_2 = 10;
	}
	nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, LEDS_N, NRF_DRV_PWM_FLAG_LOOP);	
	//nrf_drv_pwm_simple_playback(&m_pwm0, &m_demo1_seq, LEDS_N, NRF_DRV_PWM_FLAG_STOP);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    printk("call to ble_lbs_on_db_disc_evt for instance %d and link 0x%x!\r\n",
                    p_evt->conn_handle,
                    p_evt->conn_handle);
    ble_lbs_on_db_disc_evt(&m_ble_lbs_c[p_evt->conn_handle], p_evt);
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

APP_TIMER_DEF(m_volmeas_timer_id);      /**< adc采样定时器 */
#define VOLMEAS_READ_INTERVAL           APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< ADC采样间隔 100ms读取一次*/
static void volmeas_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	timestamp++;
	//printk("volmeas_timeout_handler\r\n");
}
static void timers_init(void)
{
	uint32_t err_code;
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	// Create timers.	
	err_code = app_timer_create(&m_volmeas_timer_id, APP_TIMER_MODE_REPEATED, volmeas_timeout_handler);
	APP_ERROR_CHECK(err_code);
    // Start application timers.
    err_code = app_timer_start(m_volmeas_timer_id, VOLMEAS_READ_INTERVAL, NULL);
}
/** @brief Function to sleep until a BLE event is received by the application.
 */
static void power_manage(void)
{
    ret_code_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

static void pa_assist(uint32_t gpio_pa_pin,uint32_t gpio_lna_pin)
{
#if defined PTR9618PA	
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
#endif	
}

void gpio_init(void)
{
	nrf_gpio_cfg_output(PIN_SOLAR_EN);
	nrf_gpio_pin_clear(PIN_SOLAR_EN);
	nrf_gpio_cfg_output(PIN_SOLAR_VOL1);
	nrf_gpio_pin_set(PIN_SOLAR_VOL1);
	nrf_gpio_cfg_output(PIN_SOLAR_VOL2);
	nrf_gpio_pin_clear(PIN_SOLAR_VOL2);	
	nrf_gpio_cfg_output(PIN_OTG_EN);
	nrf_gpio_pin_clear(PIN_OTG_EN);	

	nrf_gpio_cfg_output(PIN_FLASH_CS);//27K
	nrf_gpio_pin_set(PIN_FLASH_CS);		
	
	nrf_gpio_cfg_output(PIN_LED1);//27K
	nrf_gpio_pin_clear(PIN_LED1);		
	nrf_gpio_cfg_output(PIN_LED2);//27K
	nrf_gpio_pin_clear(PIN_LED2);			
}


int main(void)
{
    ret_code_t err_code;
	uint32_t timestamp_old;
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    //NRF_LOG_INFO("Multilink Example\r\n");
    //leds_init();	
	gpio_init();
	uart_init(PIN_TXD, PIN_RXD, UART_BAUDRATE_BAUDRATE_Baud115200);
	printk("Multilink Example\r\n");
    //APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
	timers_init();
    buttons_init();
    //pwm_init();
	printk("1\r\n");		
	OLED_Init();
	printk("2\r\n");		
	ble_stack_init();	//协议栈初试化，设置时钟，demo里面设置为外部时钟。并且注册事件派发函数
	pa_assist(24, 20);
	
    db_discovery_init();//服务发现
    lbs_c_init();		//把lbs_c_evt_handler函数地址传送给m_ble_lbs_c结构体的evt_handler函数指针
				//当通过notification接收到数据时回调lbs_c_evt_handler函数
	//OLED_ShowChar(0,0,'0' );

    // Start scanning for peripherals and initiate connection to devices which
    // advertise.
    scan_start();

    // Turn on the LED to signal scanning.
    //bsp_board_led_on(CENTRAL_SCANNING_LED);

    for (;;)
    {

		power_manage();
		if(timestamp != timestamp_old)
		{
			//LCD_P8x16Str(0, 0, "4");				
			LED_Printint(30, 0, solar_vol1);
			LED_Printint(30, 2, batt_vol1);
			LED_Printint(30, 4, solar_vol2);		
			LED_Printint(30, 6, batt_vol2);//显示实时太阳能功率			
			nrf_gpio_pin_toggle(PIN_LED1);
			nrf_gpio_pin_toggle(PIN_LED2);
			timestamp_old = timestamp;
		}
		
    }
}
