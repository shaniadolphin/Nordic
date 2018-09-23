/*
 * Copyright (c) 2016 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic Semiconductor. The use,
 * copying, transfer or disclosure of such information is prohibited except by express written
 * agreement with Nordic Semiconductor.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "boards.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "ble_advdata.h"
#include "ble_conn_state.h"
#include "ble_nus_c.h"
#include "..\prink\printk.h"

#define CENTRAL_LINK_COUNT      5                               /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT   0                               /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define TOTAL_LINK_COUNT          CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT /**< Total number of links used by the application. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE    GATT_MTU_SIZE_DEFAULT           /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define UART_TX_BUF_SIZE        256                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                             /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN      /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 2                               /**< Size of timer operation queues. */

#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                               /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Determines supervision time-out in units of 10 millisecond. */
#define BUTTON_DETECTION_DELAY  APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */

static ble_nus_c_t              m_ble_nus_c[TOTAL_LINK_COUNT];                    	/**< Instance of NUS service. Must be passed to all NUS_C API calls. */
static ble_db_discovery_t       m_ble_db_discovery[TOTAL_LINK_COUNT];             	/**< Instance of database discovery module. Must be passed to all db_discovert API calls */
static uint8_t            		m_ble_lbs_c_count;                       			/**< Keeps track of how many instances of LED Button client module have been initialized. >*/
static uint8_t            		linkedcnt = 0;
static const char m_target_periph_name[] = "Nordic_UART";//"Nordic_Blinky";         /**< Name of the device we try to connect to. This name is searched for in the scan report data*/
#define PIN_OTG_EN			7
//#define PIN_KEY_IN			5
#define PIN_KEY_IN			14
#define PIN_OLED_PWR		OLED_EN
#define PIN_OLED_DC			OLED_DC
#define PIN_OLED_RST		OLED_RST
#define PIN_OLED_CS			OLED_CS
#define PIN_SPI_MISO		SPI0_CONFIG_MISO_PIN
#define PIN_SPI_SCK			SPI0_CONFIG_SCK_PIN
#define PIN_SPI_MOSI		SPI0_CONFIG_MOSI_PIN
//#define PIN_I2C_SDA		23
#define PIN_I2C_SCL			22
#define PIN_FLASH_CS		13

#define PIN_SOLAR_EN		12
#define PIN_SOLAR_VOL1		26
#define PIN_SOLAR_VOL2		25

#define PIN_RXD				8
#define PIN_TXD				6
#define PIN_LED2			19
#define PIN_LED1			19

#define PIN_AIN2			4	//Charge Current
#define PIN_AIN1			3	//Batt Voltage
#define PIN_AIN0			2	//Solar Voltage
/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
  {
    (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
    (uint16_t)SLAVE_LATENCY,            // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
  };

/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist = 0,
    #endif
};

/**
 * @brief NUS uuid
 */
static const ble_uuid_t m_nus_uuid =
  {
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
  };

/**@brief Function for asserts in the SoftDevice.
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


/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(ret);

    //ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    //APP_ERROR_CHECK(ret);
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
    //ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
	ble_nus_c_on_db_disc_evt(&m_ble_nus_c[p_evt->conn_handle], p_evt);
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            //UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                //while (ble_nus_c_string_send(&m_ble_nus_c, data_array, index) != NRF_SUCCESS)
                //{
                    // repeat until sent.
                //}
                index = 0;
            }
            break;
        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            //APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
{
    uint32_t err_code;
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            //err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            //APP_ERROR_CHECK(err_code);
			err_code = ble_nus_c_handles_assign(&p_ble_nus_c[p_ble_nus_evt->conn_handle], p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);	
            err_code = ble_nus_c_rx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
		    printk("The device has the Nordic UART Service:%d\r\n", p_ble_nus_evt->conn_handle);
            break;

        case BLE_NUS_C_EVT_NUS_RX_EVT:
            for (uint32_t i = 0; i < p_ble_nus_evt->data_len; i++)
            {
                //while (app_uart_put( p_ble_nus_evt->p_data[i]) != NRF_SUCCESS);
				printk("%d ",p_ble_nus_evt->p_data[i]);
            }
			printk("\r\n", p_ble_nus_evt->data_len);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            printk("Disconnected\r\n");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
/**@brief Reads an advertising report and checks if a uuid is present in the service list.
 *
 * @details The function is able to search for 16-bit, 32-bit and 128-bit service uuids.
 *          To see the format of a advertisement packet, see
 *          https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
 *
 * @param[in]   p_target_uuid The uuid to search fir
 * @param[in]   p_adv_report  Pointer to the advertisement report.
 *
 * @retval      true if the UUID is present in the advertisement report. Otherwise false
 */
static bool is_uuid_present(const ble_uuid_t *p_target_uuid,
                            const ble_gap_evt_adv_report_t *p_adv_report)
{
    uint32_t err_code;
    uint32_t index = 0;
    uint8_t *p_data = (uint8_t *)p_adv_report->data;
    ble_uuid_t extracted_uuid;

    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if ( (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
           || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE)
           )
        {
            for (uint32_t u_index = 0; u_index < (field_length / UUID16_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(  UUID16_SIZE,
                                                &p_data[u_index * UUID16_SIZE + index + 2],
                                                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE)
                )
        {
            for (uint32_t u_index = 0; u_index < (field_length / UUID32_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(UUID16_SIZE,
                &p_data[u_index * UUID32_SIZE + index + 2],
                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE)
                )
        {
            err_code = sd_ble_uuid_decode(UUID128_SIZE,
                                          &p_data[index + 2],
                                          &extracted_uuid);
            if (err_code == NRF_SUCCESS)
            {
                if ((extracted_uuid.uuid == p_target_uuid->uuid)
                    && (extracted_uuid.type == p_target_uuid->type))
                {
                    return true;
                }
            }
        }
        index += field_length + 1;
    }
    return false;
}

static uint32_t adv_report_parse(uint8_t type, uint8_array_t * p_advdata, uint8_array_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;
	//printk("adv_report_parse:\r\n",p_advdata->size);
    while (index < p_advdata->size)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];
		//printk("field_type:%d-type:%d\r\n",field_type, type);
		p_typedata->size = 0;
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

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    uint8_array_t 	adv_data;
    uint8_array_t 	dev_name;
	bool 			found_name = false;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;

            if (is_uuid_present(&m_nus_uuid, p_adv_report))
            {
				err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                              &m_scan_params,
                                              &m_connection_param);

                if (err_code == NRF_SUCCESS)
                {
                    // scan is automatically stopped by the connect
                    //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                    //APP_ERROR_CHECK(err_code);
					//printk("target:%s\r\n",dev_name);
                    printk("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
                             p_adv_report->peer_addr.addr[0],
                             p_adv_report->peer_addr.addr[1],
                             p_adv_report->peer_addr.addr[2],
                             p_adv_report->peer_addr.addr[3],
                             p_adv_report->peer_addr.addr[4],
                             p_adv_report->peer_addr.addr[5]
                             );
					printk("\r\nRSSI:%d\r\n",p_adv_report->rssi);
                }
            }
			#if 0
			else
			{
				
				err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);
				if (err_code != NRF_SUCCESS)
				{
					// Look for the short local name if it was not found as complete
					err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
					if(err_code == NRF_SUCCESS)found_name = true;
				}
				else
				{
					found_name = true;
				}
				if (found_name)
				{
					printk("\r\n…Ë±∏£∫");
					for(uint8_t i = 0;i<dev_name.size;i++)
					{
						printk("%c",dev_name.p_data[i]);
					}
				}
				
				printk("found:%02x%02x%02x%02x%02x%02x ",
						 p_adv_report->peer_addr.addr[0],
						 p_adv_report->peer_addr.addr[1],
						 p_adv_report->peer_addr.addr[2],
						 p_adv_report->peer_addr.addr[3],
						 p_adv_report->peer_addr.addr[4],
						 p_adv_report->peer_addr.addr[5]
						 );
				printk("RSSI:%d\r\n",p_adv_report->rssi);				
			}
			#endif
        }break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
			linkedcnt ++;
			printk("%d:Connection 0x%x established, starting DB discovery.\r\n",__LINE__, p_ble_evt->evt.gap_evt.conn_handle);
            // start discovery of services. The NUS Client waits for a discovery result
            //err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
		     
			err_code = ble_db_discovery_start(&m_ble_db_discovery[p_ble_evt->evt.gap_evt.conn_handle], p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
		
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
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                //NRF_LOG_DEBUG("Scan timed out.\r\n");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                printk("Connection Request timed out.\r\n");
            }
            break; // BLE_GAP_EVT_TIMEOUT

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

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

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    uint16_t conn_handle;
    conn_handle = p_ble_evt->evt.gap_evt.conn_handle;	
    on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    //ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    //ble_nus_c_on_ble_evt(&m_ble_nus_c,p_ble_evt);
    // Make sure taht an invalid connection handle are not passed since
    // our array of modules is bound to TOTAL_LINK_COUNT.
    if (conn_handle < TOTAL_LINK_COUNT)
    {
        ble_db_discovery_on_ble_evt(&m_ble_db_discovery[conn_handle], p_ble_evt);
        ble_nus_c_on_ble_evt(&m_ble_nus_c[conn_handle], p_ble_evt);
    }	
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

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            //sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            //err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
            //                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            //if (err_code != NRF_ERROR_INVALID_STATE)
            //{
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the NUS Client.
 */
static void nus_c_init(void)
{
    uint32_t         err_code;
    ble_nus_c_init_t nus_c_init_t;

    nus_c_init_t.evt_handler = ble_nus_c_evt_handler;

    //err_code = ble_nus_c_init(&m_ble_nus_c, &nus_c_init_t);
    //APP_ERROR_CHECK(err_code);
    for (m_ble_lbs_c_count = 0; m_ble_lbs_c_count < TOTAL_LINK_COUNT; m_ble_lbs_c_count++)
    {
        err_code = ble_nus_c_init(&m_ble_nus_c[m_ble_lbs_c_count], &nus_c_init_t);
        APP_ERROR_CHECK(err_code);
    }
    m_ble_lbs_c_count = 0;	
}

/**@brief Function for initializing buttons and leds.
 */
static void buttons_leds_init(void)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED,
                                 APP_TIMER_TICKS(500, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_btn_ble_init(NULL, &startup_event);
    //APP_ERROR_CHECK(err_code);
}
static uint32_t string_send_to_all(uint8_t buf[], uint8_t length)
{
    uint32_t err_code;

    for (uint32_t i = 0; i< CENTRAL_LINK_COUNT; i++)
    {
        //err_code = ble_lbs_led_status_send(&m_ble_lbs_c[i], button_action);
		err_code = ble_nus_c_string_send(&m_ble_nus_c[i], &buf[0], length);
		printk("ble_nus_c_string_send=%d\r\n", err_code);
		if(err_code != NRF_SUCCESS &&
            err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
            err_code != NRF_ERROR_INVALID_STATE)
        {
            return err_code;
        }
    }
    return NRF_SUCCESS;
}
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    uint32_t err_code;
	uint8_t buf[5] = {0};
	buf[0] = 1;
	buf[1] = 2;
	buf[2] = 3;
	buf[3] = 4;
	buf[4] = 5;	
    switch (pin_no)
    {
        case PIN_KEY_IN:				
			if(button_action == 0)
			{
                buf[0] = PIN_KEY_IN;		
				nrf_gpio_pin_clear(PIN_LED2);
			}
			else
			{	
				buf[0] = PIN_KEY_IN;	
				nrf_gpio_pin_set(PIN_LED2);		
				//printk("LEDBUTTON_BUTTON_PIN:%d\r\n",button_action);	
			}
			
        break;
        default:
            APP_ERROR_HANDLER(pin_no);
        break;
    }
	string_send_to_all(&buf[0], 5);
	printk("button_event_handler done:%d-%d\r\n",button_action,err_code);		
}
/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    uint32_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {PIN_KEY_IN, false, NRF_GPIO_PIN_PULLUP, button_event_handler}
    };

    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
	err_code = app_button_enable();
	APP_ERROR_CHECK(err_code);
}
/** @brief Function for initializing the Database Discovery Module.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);
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
void gpio_init(void)
{
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
int main(void)
{

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    //uart_init();
	uart_init(PIN_TXD, PIN_RXD, UART_BAUDRATE_BAUDRATE_Baud115200);
    buttons_init();
	gpio_init();
    db_discovery_init();
    ble_stack_init();
	pa_assist(24, 20);
    nus_c_init();

    // Start scanning for peripherals and initiate connection
    // with devices that advertise NUS UUID.
    printk("Uart Master started\r\n");
    scan_start();

    for (;;)
    {
        power_manage();
    }
}
