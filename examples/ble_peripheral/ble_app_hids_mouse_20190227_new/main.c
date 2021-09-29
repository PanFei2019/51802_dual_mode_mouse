/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/** @file
 *
 * @defgroup ble_sdk_app_hids_mouse_main main.c
 * @{
 * @ingroup ble_sdk_app_hids_mouse
 * @brief HID Mouse Sample Application main file.
 *
 * This file contains is the source code for a sample application using the HID, Battery and Device
 * Information Service for implementing a simple mouse functionality. This application uses the
 * @ref app_scheduler.
 *
 * Also it would accept pairing requests from any peer device. This implementation of the
 * application will not know whether a connected central is a known device or not.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "softdevice_handler_appsh.h"
#include "app_timer_appsh.h"
#include "peer_manager.h"
#include "app_button.h"
#include "ble_advertising.h"
#include "fds.h"
#include "fstorage.h"
#include "ble_conn_state.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "nrf_drv_common.h"
#include "nrf_drv_qdec.h"

#include "nrf_drv_spi.h"

#include "nrf_drv_timer.h"
#include "motion_sensor.h"

#include "configuration.h"

#include "ble_conn_params.h"

#include "nrf_drv_rng.h"

#include "peer_data_storage.h"

#include "fstorage.h"
#include "nrf_delay.h"

#include "fstorage_internal_defs.h"

#include "encrypt.h"

#include "nrf_esb.h"

#include "nrf_drv_gpiote.h"

#include "nrf_drv_adc.h"

#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"

#if 0//NRF_LOG_BACKEND_SERIAL_USES_UART
#include "nrf_drv_uart.h"
#endif


#define Enable_2_point_4G 1
#define ENABLE_AUTH 1

enum
{
	BT_mode    = 0x00,
	RF_2_point_4_mode = 0x01
};

uint8_t work_mode = BT_mode;//RF_2_point_4_mode;
//1356 1016 7624
#if Enable_2_point_4G
//2.4G
typedef enum {
    NRF_ESB_STATE_IDLE,                                     /**< Module idle. */
    NRF_ESB_STATE_PTX_TX,                                   /**< Module transmitting without acknowledgment. */
    NRF_ESB_STATE_PTX_TX_ACK,                               /**< Module transmitting with acknowledgment. */
    NRF_ESB_STATE_PTX_RX_ACK,                               /**< Module transmitting with acknowledgment and reception of payload with the acknowledgment response. */
    NRF_ESB_STATE_PRX,                                      /**< Module receiving packets without acknowledgment. */
    NRF_ESB_STATE_PRX_SEND_ACK,                             /**< Module transmitting acknowledgment in RX mode. */
} nrf_esb_mainstate_t;
extern nrf_esb_mainstate_t          m_nrf_esb_mainstate;

#define SYSTEM_SYNCHRONIZE      	0x02
#define SYSTEM_END	             			0x04
#define SYSTEM_NEXT_CH          		0x08
#define SYSTEM_SLEEP              		0x8a
#define PAGE_WAIT_TIME            		5000      // keyboard 4ms*5000=20s, mouse 8ms*5000=40s

//uint8_t base_addr_0[4] = {0x3e, 0x14, 0x5b, 0x6d};
//uint8_t addr_prefix[8] = {0xa3, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };
const uint8_t RF_CHANNEL_TABLE[16] = {8, 37, 68, 21, 40, 77, 7, 35, 67, 10, 42, 55, 14, 28, 49, 41};//{3, 28, 53, 8, 33, 58, 13, 38, 63, 18, 43, 68, 23, 48, 73, 78};
uint8_t rf_channel = 0;
extern nrf_esb_config_t         m_config_local;
static nrf_esb_payload_t        rx_payload;
static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x20, 0x00, 0x00, 0x00, 0x11);
bool m_TX_SUCCESS = true;
//bool m_TX_FAIL = false;
bool m_RX_SUCCESS = false;
bool flag_rf_paged = false;
bool change_cpi_flag = false;

uint32_t button_state = 0;
uint32_t previous_button_state = 0;
#if 0
uint8_t button_buffer[5] = {0};
uint8_t button_buf_index = 0;
uint8_t read_buf_index = 0;
uint8_t button_cnt = 0;
#endif
uint32_t sleep_time_cnt = 0;


const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);
#if ENABLE_RTC_TIMER
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0); /**< Declaring an instance of nrf_drv_rtc for RTC0. */
#endif

enum
{
	key_event    = 0x01,
	motion_event = 0x02,
	scroll_event = 0x04
};

uint8_t active_event = 0;
bool polling_motion = false;
#endif

//BLE
#define FEATURE_REPORT_ENABLE	
#define OUTPUT_REPORT_ENABLE

uint8_t KeepConnectReport[7] = {0xFF,0x20,0x73,0xAB,0x00,0x95,0x08};

#if ENABLE_AUTH
#if 0 //number 1
const uint8_t encryptPassword[6] = {0x38,0x67,0x84,0x29,0x31,0x72};	
const uint8_t decryptPassword[6] = {0x75,0x34,0x26,0x81,0x95,0x18};	
#else //number 3 for ble
//uint8_t encryptPassword[6] = {0x46,0x23,0x75,0x35,0x78,0x72};
//uint8_t decryptPassword[6] = {0x67,0x38,0x53,0x26,0x19,0x62};
const uint8_t encryptPassword[6] = {0x36,0x87,0x94,0x56,0x23,0x18};
const uint8_t decryptPassword[6] = {0x45,0x78,0x35,0x28,0x63,0x92};
#endif

const uint8_t BDaddrPassword[6] = {0x78,0x65,0x23,0x97,0x17,0x62};
//uint8_t BDaddrEncryptData[6];
uint8_t BDaddrSourceEncryptData[6];
//uint8_t BDaddrData[6];
uint8_t BDaddrSource[6];
#endif

static void qdec_event_handler(nrf_drv_qdec_event_t event);
static void adc_config(void);
void spi_event_handler(nrf_drv_spi_evt_t const * p_event);
static void fs_evt_handler(fs_evt_t const * const evt, fs_ret_t result);
void m_flash_write(uint8_t page_num);
bool m_flash_read(const uint32_t* address,uint8_t* bd_addr);
uint32_t const * address_of_page(uint8_t page_num);
extern bool  m_whitelist_temporarily_disabled;
#if 0//NRF_LOG_BACKEND_SERIAL_USES_UART
extern nrf_drv_uart_t m_uart;
#endif

bool test_dongle_connected = false;

#define DATA_SIZE  6

uint32_t m_datas[DATA_SIZE]={0,1,2,3,88,5};
//uint8_t  erase_flag=0;
//uint8_t  store_flag=0;
uint8_t fs_page_num = 0;

FS_REGISTER_CFG(fs_config_t fs_config) =

{

		.callback  = fs_evt_handler,// Function for event callbacks.

		.num_pages = 1,             // Number of physical flash pages required.

		.priority  = 0xFE           // Priority for flash usage.

};

FS_REGISTER_CFG(fs_config_t fs_config_encrypt) =

{

		.callback  = fs_evt_handler,// Function for event callbacks.

		.num_pages = 1,             // Number of physical flash pages required.

		.priority  = 0xFD           // Priority for flash usage.

};

FS_REGISTER_CFG(fs_config_t fs_config_work_mode) =
{

		.callback  = fs_evt_handler,// Function for event callbacks.

		.num_pages = 1,             // Number of physical flash pages required.

		.priority  = 0xFD           // Priority for flash usage.

};

FS_REGISTER_CFG(fs_config_t fs_config_work_mode_change_indicate) =
{

		.callback  = fs_evt_handler,// Function for event callbacks.

		.num_pages = 1,             // Number of physical flash pages required.

		.priority  = 0xFD           // Priority for flash usage.

};

#if 0
FS_REGISTER_CFG(fs_config_t fs_config_auth_encrypt) =
{

		.callback  = fs_evt_handler,// Function for event callbacks.

		.num_pages = 1,             // Number of physical flash pages required.

		.priority  = 0xFD           // Priority for flash usage.

};
#endif
#if 0
typedef struct
{
    uint16_t              length_words; /**< @brief The length of the data in words. */
    pm_peer_data_id_t     data_id;      /**< @brief ID that specifies the type of data (defines which member of the union is used). */
		uint8_t all_data[80];
} my_peer_data_t;
#endif

extern ble_adv_evt_t m_adv_evt;

const nrf_drv_timer_t TIMER_motion = NRF_DRV_TIMER_INSTANCE(1);

static volatile bool m_report_ready_flag = false;
static volatile bool m_first_report_flag = true;
static volatile uint32_t m_accdblread;
static volatile int32_t m_accread = 0;

uint16_t m_time_low_voltage_alter = 0;
uint16_t m_time_voltage_measurement = 1420;
uint16_t m_time_thress_key_pressed_measurement = 0;
uint16_t m_time_measurement = 0;
uint16_t m_time_keep_connection = 0;
uint16_t m_cold_boot_cnt = 0;
uint8_t m_time_keep_connection_cnt = 0;
uint8_t m_sleep_timeout_cnt = 0;
uint8_t m_voltage_measurement_cnt = 0;
//bool m_enter_sleep_mode = false;
bool m_motion_timer_is_start = false;
bool m_sleep_timer_is_start = false;
bool Authorization = false;
bool Authorization_enable = false;//false;
bool Authorization_store = false;//false;
bool repairing = false;
bool m_three_keys_pressed = false;
bool m_spi_is_enable  = false;
bool m_adc_is_enable = false;
bool m_cold_boot = true;
bool m_qdec_is_enable = false;
bool m_motion_sensor_is_enable = false;
bool m_rtc_timer_is_enable = false;
bool change_work_mode_completed = false;
bool change_work_mode_start = false;
bool change_work_mode_indicate = false;
bool just_erase_page_3 = false;
#if  NRF_LOG_BACKEND_SERIAL_USES_UART
bool m_uart_is_enable = true;
#endif

#if ENABLE_CLEAR_AUTH
bool m_just_erase_flash = false;
#endif
//bool repairingToPreviousBoonded = false;
uint16_t peer_id_stored[2] = {0xFFFF};
uint16_t peer_id_cnt = 0;
//my_peer_data_t my_peer_data;


#define SPI_INSTANCE  0 /**< SPI instance index. */
const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#if 0
#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */

static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */
#endif
//uint8_t       m_rx_buf[5];    /**< RX buffer. */


#if BUTTONS_NUMBER < 4
#error "Not enough resources on board to run example"
#endif

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                      /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT              0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME_TEST                "Only_test"
#define DEVICE_NAME                     "Bluetooth_Mouse4.0"                             /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_TIMER_PRESCALER             0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                          /**< Size of timer operation queues. */

#if CONSUMPTION_TEST_ENABLE
#define TIME_TO_SLEEP_INTERVAL          APP_TIMER_TICKS(6000, APP_TIMER_PRESCALER)
#else
#define TIME_TO_SLEEP_INTERVAL          APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER)
#endif
#define MOTION_INTERVAL                 APP_TIMER_TICKS(8, APP_TIMER_PRESCALER) /**< motion interval (ticks). */
#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL               100                                        /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT         1                                          /**< Increment between each simulated battery level measurement. */

#define PNP_ID_VENDOR_ID_SOURCE         0x02                                       /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                0x1915                                     /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID               0xEEEE                                     /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION          0x0001                                     /**< Product Version. */

/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)            /**< Minimum connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(8/*60*/, UNIT_1_25_MS)             /**< Maximum connection interval (15 ms). */
#define SLAVE_LATENCY                   130//66//20                                          /**< Slave latency. */
#define SLAVE_LATENCY_TEST              0 
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(3000, UNIT_10_MS)             /**< Connection supervisory timeout (3000 ms). */
#define CONN_SUP_TIMEOUT_TEST						MSEC_TO_UNITS(100, UNIT_10_MS)

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAM_UPDATE_COUNT     3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define MOVEMENT_SPEED                  5                                           /**< Number of pixels by which the cursor is moved each time a button is pushed. */
#define INPUT_REPORT_COUNT              4//3                                           /**< Number of input reports in this application. */
#define INPUT_REP_BUTTONS_LEN           3                                           /**< Length of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_LEN          3                                           /**< Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1                                           /**< Length of Mouse Input Report containing media player data. */
#define INPUT_REP_Authorization_LEN     7
#define INPUT_REP_BUTTONS_INDEX         0                                           /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_INDEX        1                                           /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MPLAYER_INDEX         2                                           /**< Index of Mouse Input Report containing media player data. */
#define INPUT_REP_Authorization_INDEX   3 //felix
#define INPUT_REP_REF_BUTTONS_ID        1                                           /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_MOVEMENT_ID       2                                           /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MPLAYER_ID        3                                           /**< Id of reference to Mouse Input Report containing media player data. */
#define INPUT_REP_REF_Authorization_ID        0xF0 

#define BASE_USB_HID_SPEC_VERSION       0x0101                                      /**< Version number of base USB HID Specification implemented by this application. */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, \
                                            BLE_STACK_HANDLER_SCHED_EVT_SIZE)                     /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                 20                                         /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                10                                                        /**< Maximum number of events in the scheduler queue. */
#endif

#define DEAD_BEEF                       0xDEADBEEF                                                /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2                      /**< Reply when unsupported features are requested. */

#define APP_ADV_FAST_INTERVAL           0x0028                                                    /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL           0x0C80                                                    /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */

#if CONSUMPTION_TEST_ENABLE
#define APP_ADV_FAST_TIMEOUT            10                                                       /**< The duration of the fast advertising period (in seconds). */
#else
#define APP_ADV_FAST_TIMEOUT            180 
#endif

#define APP_ADV_SLOW_TIMEOUT            1//180                                                       /**< The duration of the slow advertising period (in seconds). */

static ble_hids_t m_hids;                                                                         /**< Structure used to identify the HID service. */
static ble_bas_t  m_bas;                                                                          /**< Structure used to identify the battery service. */
static bool       m_in_boot_mode = false;                                                         /**< Current protocol mode. */
static uint16_t   m_conn_handle  = BLE_CONN_HANDLE_INVALID;                                       /**< Handle of the current connection. */

//static sensorsim_cfg_t   m_battery_sim_cfg;                                                       /**< Battery Level sensor simulator configuration. */
//static sensorsim_state_t m_battery_sim_state;                                                     /**< Battery Level sensor simulator state. */

//APP_TIMER_DEF(m_battery_timer_id);                                                                /**< Battery timer. */
APP_TIMER_DEF(m_motion_timer_id); 
APP_TIMER_DEF(m_sleep_timer_id);

static pm_peer_id_t m_peer_id;                                                                    /**< Device reference handle to the current bonded central. */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

static pm_peer_id_t   m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];  /**< List of peers currently in the whitelist. */
static uint32_t       m_whitelist_peer_cnt;                                 /**< Number of peers currently in the whitelist. */
static bool           m_is_wl_changed;                                      /**< Indicates if the whitelist has been changed since last time it has been updated in the Peer Manager. */

static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);
static void mouse_movement_send(int16_t x_delta, int16_t y_delta);
static void sleep_mode_enter(void);
void set_generate_random_addr(void);
void flash_page_erase(uint32_t * page_address);
void flash_word_write(uint32_t * address, uint32_t value);
static void mouse_Authorization_send(void);

uint8_t button_status = 0;
uint8_t key_buffer[INPUT_REP_BUTTONS_LEN] = {0,0,0};

ble_gap_addr_t my_device_addr = {BLE_GAP_ADDR_TYPE_RANDOM_STATIC,{1,2,3,4,5,6}};
ble_gap_addr_t device_addr;

uint8_t cpiCnt = 0;
uint8_t cpiCntStore = 0;
extern uint8_t ChipVersion;


//ADC
uint32_t AverageVoltage = 1023;
#define ADC_BUFFER_SIZE 4                                /**< Size of buffer for ADC samples.  */
static nrf_adc_value_t       adc_buffer[ADC_BUFFER_SIZE]; /**< ADC buffer. */
static nrf_drv_adc_channel_t m_channel_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_4); /**< Channel instance. Default configuration used. */





/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t ret;

    memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
    m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

    peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

    ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
    APP_ERROR_CHECK(ret);

    // Setup the device identies list.
    // Some SoftDevices do not support this feature.
    ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
    if (ret != NRF_ERROR_NOT_SUPPORTED)
    {
        APP_ERROR_CHECK(ret);
    }
    m_is_wl_changed = false;
	#if DEBUG_TRACE_ENABLE
		NRF_LOG_INFO("m_whitelist_peer_cnt=%d\r\n",m_whitelist_peer_cnt);
	#endif
		if(m_whitelist_peer_cnt)
		{
				m_whitelist_temporarily_disabled = false;
		}
		else
		{
				m_whitelist_temporarily_disabled = true;
		}
    ret = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(ret);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
		//uint16_t i = 0;
    ret_code_t err_code;
		ret_code_t ret;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
			#if DEBUG_TRACE_ENABLE
            NRF_LOG_INFO("Connected to a previously bonded device.\r\n");
			#endif
			if(!m_spi_is_enable)
			{
				nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
				spi_config.ss_pin   = SPI_SS_PIN;
				spi_config.miso_pin = SPI_MISO_PIN;
				spi_config.mosi_pin = SPI_MOSI_PIN;
				spi_config.sck_pin  = SPI_SCK_PIN;
				APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
				m_spi_is_enable = true;
			}
			if(!m_motion_sensor_is_enable)
			{
				#if SensorJustSleep
				enterOrExitSleepmode(0xB1);
				nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pulldown);
				nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pulldown);
				#else
				enterOrExitSleepmode(0x1B);
				nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pulldown);
				nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pulldown);
				#endif
				m_motion_sensor_is_enable = true;
			}
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
						#if 1
						if(repairing)
						{
								repairing = false;
								if(peer_id_cnt)
								{
									do
									{
											err_code = pm_peer_delete(peer_id_stored[--peer_id_cnt]);
											APP_ERROR_CHECK(err_code);
											#if DEBUG_TRACE_ENABLE
											NRF_LOG_INFO("delete peer id=%d\r\n",peer_id_stored[peer_id_cnt]);
											#endif
									}while(peer_id_cnt);
								}
								#if 0
								if(peer_id_stored!=0xFFFF)
								{
									pm_peer_delete(peer_id_stored);
									#if DEBUG_TRACE_ENABLE
									NRF_LOG_INFO("delete peer id=%d\r\n",peer_id_stored);
									#endif
								}
								#endif
								#if 0
								memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
								ret = pm_whitelist_set(NULL, 0);
								APP_ERROR_CHECK(ret);
								// Setup the device identies list.
								// Some SoftDevices do not support this feature.
								ret = pm_device_identities_list_set(NULL, 0);
								if (ret != NRF_ERROR_NOT_SUPPORTED)
								{
										APP_ERROR_CHECK(ret);
								}
								#endif
								m_whitelist_peer_cnt = 0;
								#if DEBUG_TRACE_ENABLE
								NRF_LOG_INFO("m_whitelist_peer_cnt=%d\r\n",m_whitelist_peer_cnt);
								#endif
								if(Authorization)
								{
									m_flash_write(0);
								}
						}
						else
						{
								if(!m_flash_read(address_of_page(0),my_device_addr.addr))
								{ 
									if(Authorization)
									{
										err_code = sd_ble_gap_address_get(&my_device_addr); 
										APP_ERROR_CHECK(err_code);
										#if DEBUG_TRACE_ENABLE
										NRF_LOG_INFO("write flash page 0!\r\n");
										#endif
										m_flash_write(0);
									}							
								}
						}
						#endif
						#if DEBUG_TRACE_ENABLE
            NRF_LOG_INFO("Connection secured. Role: %d. conn_handle: %d, Procedure: %d\r\n",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
						#endif

            m_peer_id = p_evt->peer_id;
						
						if(Authorization)	
						{
								if(!m_spi_is_enable)
								{
									nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
									spi_config.ss_pin   = SPI_SS_PIN;
									spi_config.miso_pin = SPI_MISO_PIN;
									spi_config.mosi_pin = SPI_MOSI_PIN;
									spi_config.sck_pin  = SPI_SCK_PIN;
									APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
									m_spi_is_enable = true;
								}
								
								if(!m_motion_sensor_is_enable)
								{
									#if SensorJustSleep
									enterOrExitSleepmode(0xB1);
									nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pulldown);
									nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pulldown);
									#else
									enterOrExitSleepmode(0x1B);
									nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pulldown);
									nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pulldown);
									#endif
									m_motion_sensor_is_enable = true;
								}
								// Note: You should check on what kind of white list policy your application should use.
								if (p_evt->params.conn_sec_succeeded.procedure == PM_LINK_SECURED_PROCEDURE_BONDING)
								{
										#if DEBUG_TRACE_ENABLE
										NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible\r\n");
										NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d\r\n",
																	 m_whitelist_peer_cnt + 1,
																	 BLE_GAP_WHITELIST_ADDR_MAX_COUNT);
										#endif

										if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
										{
												// Bonded to a new peer, add it to the whitelist.
												m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;
												//m_is_wl_changed = true;
											
												// The whitelist has been modified, update it in the Peer Manager.
												err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
												APP_ERROR_CHECK(err_code);

												err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
												if (err_code != NRF_ERROR_NOT_SUPPORTED)
												{
														APP_ERROR_CHECK(err_code);
												}
										}
								}
						}
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_BUSY || err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start();
        } break;

        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
        {
            // The local database has likely changed, send service changed indications.
            pm_local_database_has_changed();
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

#if 0
/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}
#endif

#if 0
/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}
#endif

static void motion_timeout_handler(void * p_context)
{
		pm_peer_id_t current_peer_id = 0;
		uint8_t i = 0;
		ble_gap_conn_params_t   gap_conn_params;
		uint32_t err_code;
		int16_t motion_x, motion_y;
    UNUSED_PARAMETER(p_context);
		
		#if 1
		if(Authorization)
		{		
			if(!m_spi_is_enable)
			{
				nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
				spi_config.ss_pin	= SPI_SS_PIN;
				spi_config.miso_pin = SPI_MISO_PIN;
				spi_config.mosi_pin = SPI_MOSI_PIN;
				spi_config.sck_pin	= SPI_SCK_PIN;
				APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
				m_spi_is_enable = true;
				//NRF_LOG_INFO("enable spi\r\n");
			}
			
			if(getMotion(&motion_x,&motion_y,3))
			{	
				m_time_measurement = 0;
				if(m_sleep_timer_is_start)
				{
					m_sleep_timeout_cnt = 0;
					m_sleep_timer_is_start = false;
					err_code = app_timer_stop(m_sleep_timer_id);
					APP_ERROR_CHECK(err_code);
				}
				#if Negate_X
				motion_x = -motion_x;
				#endif
				
				#if Negate_Y
				motion_y = -motion_y;
				#endif

				//#if VOLTAGE_CHECK_ENABLE
				//if(AverageVoltage>=425)//3V
				//#else
				//if(AverageVoltage>=0)
				//#endif
				//{
					if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
					{
						#if Swap_XY
						mouse_movement_send(motion_y, motion_x);
						#else
						mouse_movement_send(motion_x, motion_y);
						#endif
					}
				//}
			}

			if(m_cold_boot)
			{
				if(m_cold_boot_cnt==0)
				{
					bsp_board_led_on(BSP_BOARD_LED_0);
				}
				if(++m_cold_boot_cnt>284)
				{
					m_cold_boot_cnt = 0;
					m_cold_boot = false;
					bsp_board_led_off(BSP_BOARD_LED_0);
				}
			}

			if(m_three_keys_pressed)
			{
				if(++m_time_thress_key_pressed_measurement>=400)
				{
					if(!repairing)
					{
						peer_id_cnt = 0;
						current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
						while (current_peer_id != PM_PEER_ID_INVALID)
						{
								peer_id_stored[peer_id_cnt++] = current_peer_id;
								current_peer_id = pdb_next_peer_id_get(current_peer_id);
						}
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_INFO("peer_id_cnt=%d\r\n",peer_id_cnt);
						#endif
						
						set_generate_random_addr();
						repairing = true;

						if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
						{
							err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);	
						}
						else
						{
							if((m_adv_evt!=BLE_ADV_EVT_FAST)&&(m_adv_evt!=BLE_ADV_EVT_SLOW))
							{
								ble_advertising_restart_without_whitelist();
							}				
						}
					}
				}
			}
			else
			{
				m_time_thress_key_pressed_measurement = 0;
			}

			#if 1
			
			if(++m_time_voltage_measurement>=1420)
			{
				#if 1
				if(!m_adc_is_enable)
				{
					adc_config();
					APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
					m_adc_is_enable = true;
					m_voltage_measurement_cnt = ADC_BUFFER_SIZE;
				}
				#endif
				if(m_voltage_measurement_cnt)
				{
					nrf_drv_adc_sample();
					
					if((--m_voltage_measurement_cnt)==0)
					{
						m_time_voltage_measurement = 0;
					}
				}
			}
			#endif
			
			m_time_measurement++;
			#if VOLTAGE_CHECK_ENABLE
			if(AverageVoltage>=425)//3V
			#else
			if(AverageVoltage>=0)
			#endif
			{
				bsp_board_led_off(BSP_BOARD_LED_1);
				if(!m_three_keys_pressed)
				{
					if(++m_time_measurement==428)//5)//7ms*4285=30s
					{
						if(m_motion_timer_is_start)
						{
							err_code = app_timer_stop(m_motion_timer_id);
							APP_ERROR_CHECK(err_code);
							m_motion_timer_is_start = false;
						}

						if((m_conn_handle != BLE_CONN_HANDLE_INVALID)||(m_adv_evt != BLE_ADV_EVT_IDLE))
						{
							if(m_spi_is_enable)
							{
								nrf_drv_spi_uninit(&spi);
								m_spi_is_enable = false;
							}
							#if ENABLE_CHANGE_CONNECTION_INTERVAL
							memset(&gap_conn_params, 0, sizeof(gap_conn_params));
							gap_conn_params.min_conn_interval = 12;//500ms MAX_CONN_INTERVAL
							gap_conn_params.max_conn_interval = 12;//500ms;
							gap_conn_params.slave_latency     = 20;//SLAVE_LATENCY;
							gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
							ble_conn_params_change_conn_params(&gap_conn_params);
							#if DEBUG_TRACE_ENABLE
							NRF_LOG_INFO("change to low active\r\n");
							#endif
							#endif
							#if DEBUG_TRACE_ENABLE
							//NRF_LOG_INFO("NO MOTION\r\n");
							#endif

							m_sleep_timer_is_start = true;
							m_sleep_timeout_cnt = 0;
							err_code = app_timer_start(m_sleep_timer_id, TIME_TO_SLEEP_INTERVAL, NULL);
							APP_ERROR_CHECK(err_code);
						}
						else
						{
							#if DEBUG_TRACE_ENABLE
							NRF_LOG_INFO("enter_sleep_mode\r\n");
							#endif
							if(!m_spi_is_enable)
							{
								nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
								spi_config.ss_pin   = SPI_SS_PIN;
								spi_config.miso_pin = SPI_MISO_PIN;
								spi_config.mosi_pin = SPI_MOSI_PIN;
								spi_config.sck_pin  = SPI_SCK_PIN;
								APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
								m_spi_is_enable = true;
								NRF_LOG_INFO("enable spi\r\n");
							}

							if(m_motion_sensor_is_enable)
							{
								#if SensorJustSleep
								enterOrExitSleepmode(0xBC);
								#else
								enterOrExitSleepmode(0x13);
								#endif	
								m_motion_sensor_is_enable = false;
								NRF_LOG_INFO("sensor enter sleep\r\n");
							}
							
							if(m_spi_is_enable)
							{
								nrf_drv_spi_uninit(&spi);
								m_spi_is_enable = false;
							}

							if(m_qdec_is_enable)
							{
								nrf_drv_qdec_uninit();
								m_qdec_is_enable = false;
							}
							
							nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pullup);
							nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pullup);

							if(NRF_LOG_BACKEND_SERIAL_USES_UART)
							{
								sleep_mode_enter();
							}
						}
					}
				}
			}
			else
			{
				if(++m_time_low_voltage_alter==142)
				{
					bsp_board_led_off(BSP_BOARD_LED_1);
				}
				
				if(m_time_low_voltage_alter>=426)
				{
					m_time_low_voltage_alter = 0;
					bsp_board_led_on(BSP_BOARD_LED_1);
				}
			}
		}
		else
		{
			if(Authorization_enable&&(!Authorization_store)&&(m_time_keep_connection_cnt>=20))
			{
					Authorization_store = true;
					#if ENABLE_AUTH
					memcpy(BDaddrSource,device_addr.addr,6);
					for(i=0;i<6;i++)
					{
						BDaddrSource[i] += 0x48 + i;
					}
					memcpy(BDaddrSourceEncryptData,encrypt(BDaddrSource,BDaddrPassword),6);
					m_flash_write(1);
					#endif
					if (m_whitelist_peer_cnt>0)
					{
							m_whitelist_peer_cnt = 0;
							// The whitelist has been modified, update it in the Peer Manager.
							err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
							APP_ERROR_CHECK(err_code);

							err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
							if (err_code != NRF_ERROR_NOT_SUPPORTED)
							{
									APP_ERROR_CHECK(err_code);
							}
					}
					#if DEBUG_TRACE_ENABLE
					NRF_LOG_INFO("Authorization_store\r\n");
					#endif
			}
			if(Authorization_store)
			{
				#if 0
				if(AverageVoltage>=0)
				{
					if(getMotion(&motion_x,&motion_y,3))
					{	
						m_time_measurement = 0;
						if(m_sleep_timer_is_start)
						{
							m_sleep_timer_is_start = false;
							err_code = app_timer_stop(m_sleep_timer_id);
							APP_ERROR_CHECK(err_code);
						}
						#if Negate_X
						motion_x = -motion_x;
						#endif
						
						#if Negate_Y
						motion_y = -motion_y;
						#endif

						#if Swap_XY
						mouse_movement_send(motion_y, motion_x);
						#else
						mouse_movement_send(motion_x, motion_y);
						#endif
					}
				}
				#endif			
				//err_code = pm_peers_delete();
				//APP_ERROR_CHECK(err_code);
				
				if(++m_time_measurement>=71)//5)//7ms*71=500ms
				{
						m_time_measurement = 0;
						if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
						{
									//mouse_Authorization_send();
									mouse_movement_send(0,0);
								//NRF_LOG_INFO("*");
						}
				}
			}
			if(++m_time_keep_connection>=20)//5)//7ms*20=140ms
			{
					m_time_keep_connection = 0;
					if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
					{
								mouse_Authorization_send();
								if(m_time_keep_connection_cnt<20)
								{
										m_time_keep_connection_cnt++;
								}
								//mouse_movement_send(0,0);
							//NRF_LOG_INFO("*");
					}
			}
		}
		#endif
}

void sleep_timeout_handler(void * p_context)
{
		uint32_t err_code;
		UNUSED_PARAMETER(p_context);
		//NRF_LOG_INFO("enter_sleep_mode\r\n");
		//m_enter_sleep_mode = true;
		if(++m_time_measurement>=428)
		{
			#if CONSUMPTION_TEST_ENABLE
			if(m_sleep_timeout_cnt<1)
			#else
			if(m_sleep_timeout_cnt<9)
			#endif
			{
				if(m_sleep_timeout_cnt==0)
				{
					if(m_qdec_is_enable)
					{
						nrf_drv_qdec_uninit();
						m_qdec_is_enable = false;
						
						nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pullup);
						nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pullup);
					}

					#if 0
					if(m_conn_handle == BLE_CONN_HANDLE_INVALID)
					{
						if(!m_spi_is_enable)
						{
							nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
							spi_config.ss_pin   = SPI_SS_PIN;
							spi_config.miso_pin = SPI_MISO_PIN;
							spi_config.mosi_pin = SPI_MOSI_PIN;
							spi_config.sck_pin  = SPI_SCK_PIN;
							APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
							m_spi_is_enable = true;
							NRF_LOG_INFO("enable spi\r\n");
						}

						if(m_motion_sensor_is_enable)
						{
							#if SensorJustSleep
							enterOrExitSleepmode(0xBC);
							#else
							enterOrExitSleepmode(0x13);
							#endif	
							m_motion_sensor_is_enable = false;
							NRF_LOG_INFO("sensor enter sleep\r\n");
						}
				
						if(m_spi_is_enable)
						{
							nrf_drv_spi_uninit(&spi);
							m_spi_is_enable = false;							
							NRF_LOG_INFO("disable spi\r\n");
						}						
					}
					#endif
				}
				m_sleep_timeout_cnt++;
				#if DEBUG_TRACE_ENABLE
				NRF_LOG_INFO("m_sleep_timeout_cnt=%d\r\n",m_sleep_timeout_cnt);
				#endif
				err_code = app_timer_start(m_sleep_timer_id, TIME_TO_SLEEP_INTERVAL, NULL);
			}
			else
			{
				#if 0
				#if DEBUG_TRACE_ENABLE
				NRF_LOG_INFO("enter_sleep_mode\r\n");
				#endif
				if(!m_spi_is_enable)
				{
					nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
					spi_config.ss_pin   = SPI_SS_PIN;
					spi_config.miso_pin = SPI_MISO_PIN;
					spi_config.mosi_pin = SPI_MOSI_PIN;
					spi_config.sck_pin  = SPI_SCK_PIN;
					APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
					m_spi_is_enable = true;
					NRF_LOG_INFO("enable spi\r\n");
				}

				if(m_motion_sensor_is_enable)
				{
					#if SensorJustSleep
					enterOrExitSleepmode(0xBC);
					#else
					enterOrExitSleepmode(0x13);
					#endif	
					m_motion_sensor_is_enable = false;
					NRF_LOG_INFO("sensor enter sleep\r\n");
				}
				
				if(m_spi_is_enable)
				{
					nrf_drv_spi_uninit(&spi);
					m_spi_is_enable = false;
					nrf_gpio_cfg_output(SPI_MISO_PIN);
					nrf_gpio_cfg_output(SPI_MOSI_PIN);
					nrf_gpio_cfg_output(SPI_SCK_PIN);
					nrf_gpio_pin_set(SPI_MISO_PIN);
					nrf_gpio_pin_set(SPI_MOSI_PIN);
					nrf_gpio_pin_set(SPI_SCK_PIN);
					
					NRF_LOG_INFO("disable spi\r\n");
				}
				
				if(m_qdec_is_enable)
				{
					nrf_drv_qdec_uninit();
					m_qdec_is_enable = false;
					nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pullup);
					nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pullup);
				}
				#endif

				#if SensorJustSleep
				//sleep_mode_enter();		
				#if 1
				if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
				{
					err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
					APP_ERROR_CHECK(err_code);
				}
				#endif
				#else
				#if 1
				if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
				{
					err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
					APP_ERROR_CHECK(err_code);
				}
				#endif
				#endif
			}
		}
		else
		{
			m_sleep_timer_is_start = false;
			m_sleep_timeout_cnt = 0;
			err_code = app_timer_stop(m_sleep_timer_id);
			APP_ERROR_CHECK(err_code);
		}
}



/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create battery timer.
    #if 0
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	#endif
	

    err_code = app_timer_create(&m_motion_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                motion_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&m_sleep_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                sleep_timeout_handler);
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

		if(Authorization)
		{
		    err_code = sd_ble_gap_device_name_set(&sec_mode,
		                                          (const uint8_t *)DEVICE_NAME,
		                                          strlen(DEVICE_NAME));
    }
    else
    {
    		err_code = sd_ble_gap_device_name_set(&sec_mode,
		                                          (const uint8_t *)DEVICE_NAME_TEST,
		                                          strlen(DEVICE_NAME_TEST));
    }
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_MOUSE);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    
		if(Authorization)
		{
			gap_conn_params.slave_latency     = SLAVE_LATENCY;
			gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
		}
		else
		{
			gap_conn_params.slave_latency     = SLAVE_LATENCY_TEST;
			gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT_TEST;
			
		}

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    uint32_t         err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    uint32_t       err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_report_read_perm);

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    uint32_t                  err_code;
    ble_hids_init_t           hids_init_obj;
    ble_hids_inp_rep_init_t   inp_rep_array[INPUT_REPORT_COUNT];
    ble_hids_inp_rep_init_t * p_input_report;
    uint8_t                   hid_info_flags;
	
		#ifdef OUTPUT_REPORT_ENABLE
		ble_hids_outp_rep_init_t   output_report_array[1];
    ble_hids_outp_rep_init_t * p_output_report;
		#endif
	
	#ifdef FEATURE_REPORT_ENABLE
    ble_hids_feature_rep_init_t feature_report_array[1];
    ble_hids_feature_rep_init_t * p_feature_report;
	#endif 

    static uint8_t rep_map_data[] =
    {
        0x05, 0x01, // Usage Page (Generic Desktop)
        0x09, 0x02, // Usage (Mouse)

        0xA1, 0x01, // Collection (Application)

        // Report ID 1: Mouse buttons + scroll/pan
        0x85, 0x01,       // Report Id 1
        0x09, 0x01,       // Usage (Pointer)
        0xA1, 0x00,       // Collection (Physical)
        0x95, 0x05,       // Report Count (3)
        0x75, 0x01,       // Report Size (1)
        0x05, 0x09,       // Usage Page (Buttons)
        0x19, 0x01,       // Usage Minimum (01)
        0x29, 0x05,       // Usage Maximum (05)
        0x15, 0x00,       // Logical Minimum (0)
        0x25, 0x01,       // Logical Maximum (1)
        0x81, 0x02,       // Input (Data, Variable, Absolute)
        0x95, 0x01,       // Report Count (1)
        0x75, 0x03,       // Report Size (3)
        0x81, 0x01,       // Input (Constant) for padding
        0x75, 0x08,       // Report Size (8)
        0x95, 0x01,       // Report Count (1)
        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x38,       // Usage (Wheel)
        0x15, 0x81,       // Logical Minimum (-127)
        0x25, 0x7F,       // Logical Maximum (127)
        0x81, 0x06,       // Input (Data, Variable, Relative)
        0x05, 0x0C,       // Usage Page (Consumer)
        0x0A, 0x38, 0x02, // Usage (AC Pan)
        0x95, 0x01,       // Report Count (1)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0xC0,             // End Collection (Physical)

        // Report ID 2: Mouse motion
        0x85, 0x02,       // Report Id 2
        0x09, 0x01,       // Usage (Pointer)
        0xA1, 0x00,       // Collection (Physical)
        0x75, 0x0C,       // Report Size (12)
        0x95, 0x02,       // Report Count (2)
        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x30,       // Usage (X)
        0x09, 0x31,       // Usage (Y)
        0x16, 0x01, 0xF8, // Logical maximum (2047)
        0x26, 0xFF, 0x07, // Logical minimum (-2047)
        0x81, 0x06,       // Input (Data, Variable, Relative)
        0xC0,             // End Collection (Physical)
        0xC0,             // End Collection (Application)

        // Report ID 3: Advanced buttons
        0x05, 0x0C,       // Usage Page (Consumer)
        0x09, 0x01,       // Usage (Consumer Control)
        0xA1, 0x01,       // Collection (Application)
        0x85, 0x03,       // Report Id (3)
        0x15, 0x00,       // Logical minimum (0)
        0x25, 0x01,       // Logical maximum (1)
        0x75, 0x01,       // Report Size (1)
        0x95, 0x01,       // Report Count (1)

        0x09, 0xCD,       // Usage (Play/Pause)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x83, 0x01, // Usage (AL Consumer Control Configuration)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xB5,       // Usage (Scan Next Track)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xB6,       // Usage (Scan Previous Track)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)

        0x09, 0xEA,       // Usage (Volume Down)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xE9,       // Usage (Volume Up)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x25, 0x02, // Usage (AC Forward)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x24, 0x02, // Usage (AC Back)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0xC0,              // End Collection
        
        0x05 , 0x01,          // USAGE_PAGE (Generic Desktop)
        0x09 , 0x00,          // USAGE (Keyboard)
        0xA1 , 0x01,          // COLLECTION (Application)
        0x85 , 0xF0,          //    REPORT_ID (F0)
        0x15 , 0x00,					// LOGICAL_MINIMUM (0)
        0x25 , 0xff,					// LOGICAL_MAXIMUM (0xFF)
        0x95 , 0x06,					// REPORT_COUNT (7)
        0x75 , 0x08,					// REPORT_SIZE (8)
        0x81 , 0x03,          // INPUT (Cnst,Var,Abs)
				
				#ifdef OUTPUT_REPORT_ENABLE 
				0x95, 0x05,       // Report Count (5)
        0x75, 0x01,       // Report Size (1)
        0x05, 0x08,       // Usage Page (Page# for LEDs)
        0x19, 0x01,       // Usage Minimum (1)
        0x29, 0x05,       // Usage Maximum (5)
        0x91, 0x02,       // Output (Data, Variable, Absolute), Led report
        0x95, 0x01,       // Report Count (1)
        0x75, 0x03,       // Report Size (3)
        0x91, 0x01,       // Output (Data, Variable, Absolute), Led report padding
				#endif
				
				#ifdef FEATURE_REPORT_ENABLE 
				0x09, 0x05,       // Usage (Vendor Defined)
        0x15, 0x00,       // Logical Minimum (0)
        0x26, 0xFF, 0x00, // Logical Maximum (255)
        0x75, 0x08,       // Report Count (2)
        0x95, 0x02,       // Report Size (8 bit)
        0xB1, 0x02,       // Feature (Data, Variable, Absolute)
				#endif
        0xc0
    };

    memset(inp_rep_array, 0, sizeof(inp_rep_array));
		#ifdef OUTPUT_REPORT_ENABLE
		memset(output_report_array, 0, sizeof(output_report_array));
		#endif
		#ifdef FEATURE_REPORT_ENABLE 
		memset(feature_report_array, 0, sizeof(feature_report_array));
		#endif
    // Initialize HID Service.
    p_input_report                      = &inp_rep_array[INPUT_REP_BUTTONS_INDEX];
    p_input_report->max_len             = INPUT_REP_BUTTONS_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_BUTTONS_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &inp_rep_array[INPUT_REP_MOVEMENT_INDEX];
    p_input_report->max_len             = INPUT_REP_MOVEMENT_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MOVEMENT_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_input_report                      = &inp_rep_array[INPUT_REP_MPLAYER_INDEX];
    p_input_report->max_len             = INPUT_REP_MEDIA_PLAYER_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MPLAYER_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);
    
    p_input_report                      = &inp_rep_array[INPUT_REP_Authorization_INDEX];
    p_input_report->max_len             = INPUT_REP_Authorization_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_Authorization_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);
		
		#ifdef OUTPUT_REPORT_ENABLE
		p_output_report                      = &output_report_array[0];
    p_output_report->max_len             = 1;
    p_output_report->rep_ref.report_id   = 0xF0;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.write_perm);
		#endif
		
		#ifdef FEATURE_REPORT_ENABLE    
    p_feature_report                      = &feature_report_array[0];
    p_feature_report->max_len             = 2; // FEATURE_REPORT_MAX_LEN
    p_feature_report->rep_ref.report_id   = 0; // FEATURE_REP_REF_ID
    p_feature_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_FEATURE;
		//BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_feature_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_feature_report->security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_feature_report->security_mode.read_perm);
	#endif  

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = false;//true;//
    hids_init_obj.is_mouse                       = true;//false;//
    hids_init_obj.inp_rep_count                  = INPUT_REPORT_COUNT;
    hids_init_obj.p_inp_rep_array                = inp_rep_array;
		#ifdef OUTPUT_REPORT_ENABLE
		hids_init_obj.outp_rep_count                 = 1;
    hids_init_obj.p_outp_rep_array               = output_report_array;
		#else
    hids_init_obj.outp_rep_count                 = 0;
    hids_init_obj.p_outp_rep_array               = NULL;
		#endif
		#ifdef FEATURE_REPORT_ENABLE  
	hids_init_obj.feature_rep_count              = 1;
    hids_init_obj.p_feature_rep_array            = feature_report_array;
	#else
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
		#endif
    hids_init_obj.rep_map.data_len               = sizeof(rep_map_data);
    hids_init_obj.rep_map.p_data                 = rep_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_mouse_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_mouse_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_mouse_inp_rep.write_perm);
				
		#ifdef OUTPUT_REPORT_ENABLE		
		BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(
        &hids_init_obj.security_mode_boot_kb_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_boot_kb_inp_rep.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.write_perm);
		#endif
		
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    dis_init();
    bas_init();
    hids_init();
}

#if 0
/**@brief Function for initializing the battery sensor simulator.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}
#endif

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
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
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void timers_start(void)
{
    //uint32_t err_code;

    //err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);
	
		//err_code = app_timer_start(m_motion_timer_id, MOTION_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);
}


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


/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
		#if DEBUG_TRACE_ENABLE
		NRF_LOG_INFO("p_evt->evt_type=%d\r\n",p_evt->evt_type);
		#endif
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
            break;
				
				case BLE_HIDS_EVT_REP_CHAR_WRITE:
					#if DEBUG_TRACE_ENABLE
						NRF_LOG_INFO("BLE_HIDS_EVT_REP_CHAR_WRITE\r\n");
					#endif
            //on_hid_rep_char_write(p_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED:
			#if DEBUG_TRACE_ENABLE
            NRF_LOG_INFO("BLE_ADV_EVT_DIRECTED\r\n");
			#endif
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
			#if DEBUG_TRACE_ENABLE
            NRF_LOG_INFO("BLE_ADV_EVT_FAST\r\n");
			#endif
			if(Authorization)
			{
	            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
	            APP_ERROR_CHECK(err_code);
			}
			
			if(!m_motion_timer_is_start)
			{		
				err_code = app_timer_start(m_motion_timer_id, MOTION_INTERVAL, NULL);
				APP_ERROR_CHECK(err_code);
				m_motion_timer_is_start = true;
			}
						
						//err_code = app_timer_start(m_leds_timer_id, MOTION_INTERVAL, NULL);
						//APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
			#if DEBUG_TRACE_ENABLE
            NRF_LOG_INFO("BLE_ADV_EVT_SLOW\r\n");
			#endif
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
			#if DEBUG_TRACE_ENABLE
            NRF_LOG_INFO("BLE_ADV_EVT_FAST_WHITELIST\r\n");
			#endif
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            //APP_ERROR_CHECK(err_code);
				
						err_code = app_timer_start(m_motion_timer_id, MOTION_INTERVAL, NULL);
						APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
			#if DEBUG_TRACE_ENABLE
            NRF_LOG_INFO("BLE_ADV_EVT_SLOW_WHITELIST\r\n");
			#endif
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            err_code = ble_advertising_restart_without_whitelist();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_INFO("BLE_ADV_EVT_IDLE\r\n");
			#endif
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
			if(++m_time_measurement>=428)//5)//7ms*4285=30s
			{
				#if DEBUG_TRACE_ENABLE
				NRF_LOG_INFO("enter_sleep_mode\r\n");
				#endif
				if(!m_spi_is_enable)
				{
					nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
					spi_config.ss_pin   = SPI_SS_PIN;
					spi_config.miso_pin = SPI_MISO_PIN;
					spi_config.mosi_pin = SPI_MOSI_PIN;
					spi_config.sck_pin  = SPI_SCK_PIN;
					APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
					m_spi_is_enable = true;
					NRF_LOG_INFO("enable spi\r\n");
				}

				if(m_motion_sensor_is_enable)
				{
					#if SensorJustSleep
					enterOrExitSleepmode(0xBC);
					#else
					enterOrExitSleepmode(0x13);
					#endif	
					m_motion_sensor_is_enable = false;
					NRF_LOG_INFO("sensor enter sleep\r\n");
				}
				
				if(m_spi_is_enable)
				{
					nrf_drv_spi_uninit(&spi);
					m_spi_is_enable = false;
				}
				
				
				if(m_qdec_is_enable)
				{
					nrf_drv_qdec_uninit();
					m_qdec_is_enable = false;
					nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pullup);
					nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pullup);
				}

				if(m_adc_is_enable)
				{
					nrf_drv_adc_uninit();
					m_adc_is_enable = false;
				}
				
				err_code = app_timer_stop(m_motion_timer_id);
				APP_ERROR_CHECK(err_code);
				m_motion_timer_is_start = false;
				
				err_code = app_timer_stop(m_sleep_timer_id);
				APP_ERROR_CHECK(err_code);

				//sleep_mode_enter();
			}
			#if 0
			if(change_work_mode)
			{
				change_work_mode = false;
				m_flash_write(2);						
			}
			#endif
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist\r\n",
                           addr_cnt,
                           irk_cnt);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(whitelist_addrs, addr_cnt,
                                                       whitelist_irks,  irk_cnt);
            APP_ERROR_CHECK(err_code);
        }
        break;

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_INFO("BLE_ADV_EVT_PEER_ADDR_REQUEST\r\n");
			#endif
            pm_peer_data_bonding_t peer_bonding_data;

            // Only Give peer address if we have a handle to the bonded peer.
            if (m_peer_id != PM_PEER_ID_INVALID)
            {

                err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                	#if DEBUG_TRACE_ENABLE
					NRF_LOG_INFO("BLE_ADV_EVT_PEER_ADDR_REQUEST_1\r\n");
					#endif
                    APP_ERROR_CHECK(err_code);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }

            }
            break;
        }

        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
			#if DEBUG_TRACE_ENABLE
            NRF_LOG_INFO("Connected\r\n");
			#endif
						if(Authorization||test_dongle_connected)
						{
								test_dongle_connected = false;
								if(!m_cold_boot)
								{
									err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
									APP_ERROR_CHECK(err_code);
								}
								m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						}
						else
						{
								err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
								APP_ERROR_CHECK(err_code);
						}         
				
						#if 0
						err_code = app_timer_start(m_motion_timer_id, MOTION_INTERVAL, NULL);
						APP_ERROR_CHECK(err_code);
						#else
						//motion_timeout_handler(NULL);
						#endif
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
			#if DEBUG_TRACE_ENABLE
            NRF_LOG_INFO("Disconnected\r\n");
			#endif
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						
						#if 0
            if (m_is_wl_changed)
            {
                // The whitelist has been modified, update it in the Peer Manager.
                err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                APP_ERROR_CHECK(err_code);

                err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                if (err_code != NRF_ERROR_NOT_SUPPORTED)
                {
                    APP_ERROR_CHECK(err_code);
                }

                m_is_wl_changed = false;
            }
						#endif
						
						#if 0
						if(repairing)
						{
							advertising_start();
						}
						#endif
						//sleep_mode_enter();
            break; // BLE_GAP_EVT_DISCONNECTED

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
            err_code = sd_ble_user_mem_reply(m_conn_handle, NULL);
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
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_hids_on_ble_evt(&m_hids, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
    ble_advertising_on_sys_evt(sys_evt);
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
    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);
	
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
	
    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
		
		

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    uint8_t                adv_flags;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    adv_flags                       = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = adv_flags;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_whitelist_enabled      = true;//false;//
    options.ble_adv_directed_enabled       = false;//true;
    options.ble_adv_directed_slow_enabled  = false;
    options.ble_adv_directed_slow_interval = 0;
    options.ble_adv_directed_slow_timeout  = 0;
    options.ble_adv_fast_enabled           = true;
    options.ble_adv_fast_interval          = APP_ADV_FAST_INTERVAL;
    options.ble_adv_fast_timeout           = APP_ADV_FAST_TIMEOUT;
    options.ble_adv_slow_enabled           = true;
    options.ble_adv_slow_interval          = APP_ADV_SLOW_INTERVAL;
    options.ble_adv_slow_timeout           = APP_ADV_SLOW_TIMEOUT;

    err_code = ble_advertising_init(&advdata,
                                    NULL,
                                    &options,
                                    on_adv_evt,
                                    ble_advertising_error_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for sending a Mouse Movement.
 *
 * @param[in]   x_delta   Horizontal movement.
 * @param[in]   y_delta   Vertical movement.
 */
static void mouse_movement_send(int16_t x_delta, int16_t y_delta)
{
    uint32_t err_code;

    if (m_in_boot_mode)
    {
        x_delta = MIN(x_delta, 0x00ff);
        y_delta = MIN(y_delta, 0x00ff);

        err_code = ble_hids_boot_mouse_inp_rep_send(&m_hids,
                                                    0x00,
                                                    (int8_t)x_delta,
                                                    (int8_t)y_delta,
                                                    0,
                                                    NULL);
    }
    else
    {
        static uint8_t buffer[INPUT_REP_MOVEMENT_LEN];

        APP_ERROR_CHECK_BOOL(INPUT_REP_MOVEMENT_LEN == 3);

        x_delta = MIN(x_delta, 0x0fff);
        y_delta = MIN(y_delta, 0x0fff);

        buffer[0] = x_delta & 0x00ff;
        buffer[1] = ((y_delta & 0x000f) << 4) | ((x_delta & 0x0f00) >> 8);
        buffer[2] = (y_delta & 0x0ff0) >> 4;
	
        err_code = ble_hids_inp_rep_send(&m_hids,
                                         INPUT_REP_MOVEMENT_INDEX,
                                         INPUT_REP_MOVEMENT_LEN,
                                         buffer);
				//NRF_LOG_INFO("err_code:%02x!\r\n",err_code);
    }

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

static void mouse_Authorization_send(void)
{
    uint32_t err_code;  
		//static uint8_t buffer[INPUT_REP_Authorization_LEN];

		//APP_ERROR_CHECK_BOOL(INPUT_REP_Authorization_LEN == 7);

		err_code = ble_hids_inp_rep_send(&m_hids,
																		 INPUT_REP_Authorization_INDEX,
																		 INPUT_REP_Authorization_LEN,
																		 KeepConnectReport);
		//NRF_LOG_INFO("err_code:%02x!\r\n",err_code);

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

static void mouse_key_send(uint8_t key)
{
    uint32_t err_code;

        APP_ERROR_CHECK_BOOL(INPUT_REP_BUTTONS_LEN == 3);
				
				key_buffer[0] = key;
			
				err_code = ble_hids_inp_rep_send(&m_hids,
                                         INPUT_REP_BUTTONS_INDEX,
                                         INPUT_REP_BUTTONS_LEN,
                                         key_buffer);

				//NRF_LOG_INFO("err_code:%02x!\r\n",err_code);

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

static void mouse_scroll_send(uint8_t scroll)
{
    uint32_t err_code;

        APP_ERROR_CHECK_BOOL(INPUT_REP_BUTTONS_LEN == 3);
				
				key_buffer[1] = scroll;
			
				err_code = ble_hids_inp_rep_send(&m_hids,
                                         INPUT_REP_BUTTONS_INDEX,
                                         INPUT_REP_BUTTONS_LEN,
                                         key_buffer);
				key_buffer[1] = 0;

				//NRF_LOG_INFO("err_code:%02x!\r\n",err_code);

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}



/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
		ble_gap_conn_params_t   gap_conn_params;
		pm_peer_id_t current_peer_id = 0;
		
		//pm_peer_data_t my_peer_data;
		//uint32_t my_peer_data_len = 80;
																					
    uint32_t err_code;
		//NRF_LOG_INFO("event:%d!\r\n",event);
		if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			if(Authorization||Authorization_store)
			{
				if(!m_qdec_is_enable)
				{
					err_code = nrf_drv_qdec_init(NULL, qdec_event_handler);
					APP_ERROR_CHECK(err_code);
					nrf_qdec_reportper_to_value(0);
					nrf_drv_qdec_enable(); 
					m_qdec_is_enable = true;
				}
				
				switch (event)
				{
						case BSP_EVENT_KEY_7:
							if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
							{						
									//NRF_LOG_INFO("+\r\n");
									#if 1
									if(m_time_measurement>=428)//5)//7ms*4285=30s
									{
										#if ENABLE_CHANGE_CONNECTION_INTERVAL
										memset(&gap_conn_params, 0, sizeof(gap_conn_params));
										gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
										gap_conn_params.max_conn_interval = MIN_CONN_INTERVAL;
										gap_conn_params.slave_latency     = 0;//SLAVE_LATENCY;
										gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
										ble_conn_params_change_conn_params(&gap_conn_params);
										#if DEBUG_TRACE_ENABLE
										NRF_LOG_INFO("change to high active\r\n");
										#endif
										#endif
										if(!m_spi_is_enable)
										{
											nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
											spi_config.ss_pin   = SPI_SS_PIN;
											spi_config.miso_pin = SPI_MISO_PIN;
											spi_config.mosi_pin = SPI_MOSI_PIN;
											spi_config.sck_pin  = SPI_SCK_PIN;
											APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
											m_spi_is_enable = true;
										}

										if(!m_motion_sensor_is_enable)
										{
											#if SensorJustSleep
											enterOrExitSleepmode(0xB1);
											#else
											enterOrExitSleepmode(0x1B);
											#endif
											m_motion_sensor_is_enable = true;
										}
										
										if(!m_motion_timer_is_start)
										{		
											err_code = app_timer_start(m_motion_timer_id, MOTION_INTERVAL, NULL);
											APP_ERROR_CHECK(err_code);
											m_motion_timer_is_start = true;
										}
										#if DEBUG_TRACE_ENABLE
										NRF_LOG_INFO("MOTION\r\n");
										#endif
										m_time_measurement = 0;
									}
									#endif
							}
							break;
								
						case BSP_EVENT_KEY_7_release:
							if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
							{
									//NRF_LOG_INFO("-\r\n");
									//button_status &= ~0x10;
									//mouse_key_send(button_status);
							}
							break;
								
						#if 1
						case BSP_EVENT_SLEEP:
							#if !ENABLE_PAIR_TO_CHANGE_WORK_MODE
							//sleep_mode_enter();
							#if DEBUG_TRACE_ENABLE
							NRF_LOG_INFO("BSP_EVENT_SLEEP\r\n");
							#endif
							peer_id_cnt = 0;
							current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
							while (current_peer_id != PM_PEER_ID_INVALID)
							{
									peer_id_stored[peer_id_cnt++] = current_peer_id;
									current_peer_id = pdb_next_peer_id_get(current_peer_id);
							}
							#if DEBUG_TRACE_ENABLE
							NRF_LOG_INFO("peer_id_cnt=%d\r\n",peer_id_cnt);
							#endif
							
							set_generate_random_addr();
							repairing = true;
							err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

							if(!m_spi_is_enable)
							{
								nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
								spi_config.ss_pin   = SPI_SS_PIN;
								spi_config.miso_pin = SPI_MISO_PIN;
								spi_config.mosi_pin = SPI_MOSI_PIN;
								spi_config.sck_pin  = SPI_SCK_PIN;
								APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
								m_spi_is_enable = true;
							}

							if(!m_motion_sensor_is_enable)
							{
								#if SensorJustSleep
								enterOrExitSleepmode(0xB1);
								#else
								enterOrExitSleepmode(0x1B);
								#endif
								m_motion_sensor_is_enable = true;
							}
							
							if(!m_motion_timer_is_start)
							{		
								err_code = app_timer_start(m_motion_timer_id, MOTION_INTERVAL, NULL);
								APP_ERROR_CHECK(err_code);
								m_motion_timer_is_start = true;
							}
							#if DEBUG_TRACE_ENABLE
							NRF_LOG_INFO("MOTION\r\n");
							#endif
							m_time_measurement = 0;
							#else
							m_flash_write(2);	
							#endif
							
								break;

						case BSP_EVENT_DISCONNECT:
								#if 0
								err_code = sd_ble_gap_disconnect(m_conn_handle,
																								 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
								if (err_code != NRF_ERROR_INVALID_STATE)
								{
										APP_ERROR_CHECK(err_code);
								}
								#else
								#if ENABLE_LONG_PRESS_DIP_TO_CHANGE_WORK_MODE
								change_work_mode_completed = true;
								bsp_board_led_on(BSP_BOARD_LED_0);
								bsp_board_led_on(BSP_BOARD_LED_1);
								//m_flash_write(2);	
								#endif
								#endif
								break;

						case BSP_EVENT_WHITELIST_OFF:
							#if 0
								if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
								{
										err_code = ble_advertising_restart_without_whitelist();
										if (err_code != NRF_ERROR_INVALID_STATE)
										{
												APP_ERROR_CHECK(err_code);
										}
								}
							#else
								//NRF_LOG_RAW_INFO("DPI\r\n");
								if(change_work_mode_completed)
								{
									m_flash_write(2);	
								}
								else
								{
									if(ChipVersion==0x02)						
									{							
										if(++cpiCnt>=5)							
										{								
											cpiCnt = 1;							
										}						
									}						
									else						
									{							
										if(++cpiCnt>=4)							
										{								
											cpiCnt = 1;							
										}						
									}	
									cpiCntStore = (cpiCnt<<1)-1;
									if(!m_spi_is_enable)
									{
										nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
										spi_config.ss_pin   = SPI_SS_PIN;
										spi_config.miso_pin = SPI_MISO_PIN;
										spi_config.mosi_pin = SPI_MOSI_PIN;
										spi_config.sck_pin  = SPI_SCK_PIN;
										APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
										m_spi_is_enable = true;
									}
									usrPaw3205setCPI(cpiCnt);
									
									err_code = bsp_indication_set(BSP_INDICATE_DPI_SETTING);
									APP_ERROR_CHECK(err_code);
								}
							#endif
								break;
						#endif

						case BSP_EVENT_KEY_2:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										button_status |= 0x01;
										mouse_key_send(button_status);
										//turnOn();
										//verifyProductId();
										//APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));
										//getMotion(&motion_x,&motion_y,3);
										//mouse_movement_send(motion_x, motion_y);
								}
								break;
								
						case BSP_EVENT_KEY_2_release:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										button_status &= ~0x01;
										mouse_key_send(button_status);
										//NRF_LOG_INFO("release!\r\n");
								}
								break;

						case BSP_EVENT_KEY_3:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										button_status |= 0x02;
										mouse_key_send(button_status);
								}
								break;
								
						case BSP_EVENT_KEY_3_release:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										button_status &= ~0x02;
										mouse_key_send(button_status);
								}
								break;
								
						case BSP_EVENT_KEY_4:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										#if 1
										button_status |= 0x04;
										mouse_key_send(button_status);
										#else
										m_flash_write(2);				
										#endif
								}
						break;
								
						case BSP_EVENT_KEY_4_release:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										button_status &= ~0x04;
										mouse_key_send(button_status);
								}
								break;

						case BSP_EVENT_KEY_5:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										button_status |= 0x08;
										mouse_key_send(button_status);
								}
								break;
								
						case BSP_EVENT_KEY_5_release:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										button_status &= ~0x08;
										mouse_key_send(button_status);
								}
								break;
								
						case BSP_EVENT_KEY_6:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
									 button_status |= 0x10;
									 mouse_key_send(button_status);
								}
								break;
								
						case BSP_EVENT_KEY_6_release:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										button_status &= ~0x10;
										mouse_key_send(button_status);
								}
								break;
								
		#if 0
						case BSP_EVENT_KEY_2:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										mouse_movement_send(MOVEMENT_SPEED, 0);
								}
								break;

						case BSP_EVENT_KEY_3:
								if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
								{
										mouse_movement_send(0, MOVEMENT_SPEED);
								}
								break;
		#endif
						default:
								break;
				}
			}
		}
		else
		{
				#if 1
				if(event==BSP_EVENT_KEY_7)
				{			
						//NRF_LOG_INFO("+\r\n");
						#if 1
						if(m_time_measurement>=428)//5)//7ms*4285=30s
						{
							#if 0
							memset(&gap_conn_params, 0, sizeof(gap_conn_params));
							gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
							gap_conn_params.max_conn_interval = MIN_CONN_INTERVAL;
							gap_conn_params.slave_latency     = 0;//SLAVE_LATENCY;
							gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
							ble_conn_params_change_conn_params(&gap_conn_params);
							#endif
							
							if(!m_spi_is_enable)
							{
								nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
								spi_config.ss_pin   = SPI_SS_PIN;
								spi_config.miso_pin = SPI_MISO_PIN;
								spi_config.mosi_pin = SPI_MOSI_PIN;
								spi_config.sck_pin  = SPI_SCK_PIN;
								APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
								m_spi_is_enable = true;
							}

							#if 0

							if(!m_motion_sensor_is_enable)
							{
								#if SensorJustSleep
								enterOrExitSleepmode(0xB1);
								#else
								enterOrExitSleepmode(0x1B);
								#endif
								m_motion_sensor_is_enable = true;
							}
							#endif

							err_code = app_timer_start(m_motion_timer_id, MOTION_INTERVAL, NULL);
							APP_ERROR_CHECK(err_code);
							#if DEBUG_TRACE_ENABLE
							NRF_LOG_INFO("MOTION\r\n");
							#endif
							m_time_measurement = 0;

						}
						#endif
						//motion_timeout_handler(NULL);
						//button_status &= ~0x10;
						//mouse_key_send(button_status);
				}
				else if(event==BSP_EVENT_SLEEP)
				{
						#if !ENABLE_PAIR_TO_CHANGE_WORK_MODE
						//sleep_mode_enter();
						//NRF_LOG_INFO("BSP_EVENT_SLEEP\r\n");						
						peer_id_cnt = 0;
						current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
						while (current_peer_id != PM_PEER_ID_INVALID)
						{
								peer_id_stored[peer_id_cnt++] = current_peer_id;
								current_peer_id = pdb_next_peer_id_get(current_peer_id);
						}
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_INFO("peer_id_cnt=%d\r\n",peer_id_cnt);
						#endif
						set_generate_random_addr();
						if((m_adv_evt!=BLE_ADV_EVT_FAST)&&(m_adv_evt!=BLE_ADV_EVT_SLOW))
						{
								repairing = true;
								//advertising_start();
								ble_advertising_restart_without_whitelist();
						}	

						if(!m_spi_is_enable)
						{
							nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
							spi_config.ss_pin   = SPI_SS_PIN;
							spi_config.miso_pin = SPI_MISO_PIN;
							spi_config.mosi_pin = SPI_MOSI_PIN;
							spi_config.sck_pin  = SPI_SCK_PIN;
							APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
							m_spi_is_enable = true;
						}

						if(!m_motion_sensor_is_enable)
						{
							#if SensorJustSleep
							enterOrExitSleepmode(0xB1);
							#else
							enterOrExitSleepmode(0x1B);
							#endif
							m_motion_sensor_is_enable = true;
						}
						
						if(!m_motion_timer_is_start)
						{		
							err_code = app_timer_start(m_motion_timer_id, MOTION_INTERVAL, NULL);
							APP_ERROR_CHECK(err_code);
							m_motion_timer_is_start = true;
						}
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_INFO("MOTION\r\n");
						#endif
						m_time_measurement = 0;
						#else
						m_flash_write(2);	
						#endif
				}
				#if ENABLE_LONG_PRESS_DIP_TO_CHANGE_WORK_MODE
				else if(event==BSP_EVENT_DISCONNECT)
				{						
					//m_flash_write(2);	
					change_work_mode_completed = true;
					bsp_board_led_on(BSP_BOARD_LED_0);
					bsp_board_led_on(BSP_BOARD_LED_1);
				}
				else if(event==BSP_EVENT_WHITELIST_OFF)
				{
					if(change_work_mode_completed)
					{
						m_flash_write(2);	
					}
				}
				#endif
				
				#if ENABLE_CLEAR_AUTH
				else if(event==BSP_EVENT_KEY_4_release)
				{
					m_just_erase_flash = true;
					m_flash_write(1);
				}
				#endif
				#endif

				switch (event)
				{
					case BSP_EVENT_KEY_2:
							button_status |= 0x01;
							break;
					case BSP_EVENT_KEY_2_release:
							button_status &= ~0x01;
							break;

					case BSP_EVENT_KEY_3:
							button_status |= 0x02;
							break;
							
					case BSP_EVENT_KEY_3_release:
							button_status &= ~0x02;
							break;
							
					case BSP_EVENT_KEY_4:
							button_status |= 0x04;
							//m_flash_write(2);	
							break;
							
					case BSP_EVENT_KEY_4_release:
							button_status &= ~0x04;
							break;

					default:
						break;
				}
				
				if(m_adv_evt == BLE_ADV_EVT_IDLE)
				{
						if((event!=BSP_EVENT_SLEEP)&&(!repairing))
						{							
									advertising_start();
						}
				}
		}

		if(Authorization&&(button_status==0x07))
		{
			if(m_time_measurement>=428)//5)//7ms*4285=30s
			{
				#if 0
				memset(&gap_conn_params, 0, sizeof(gap_conn_params));
				gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
				gap_conn_params.max_conn_interval = MIN_CONN_INTERVAL;
				gap_conn_params.slave_latency     = 0;//SLAVE_LATENCY;
				gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
				ble_conn_params_change_conn_params(&gap_conn_params);
				#endif
				if(!m_spi_is_enable)
				{
					nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
					spi_config.ss_pin   = SPI_SS_PIN;
					spi_config.miso_pin = SPI_MISO_PIN;
					spi_config.mosi_pin = SPI_MOSI_PIN;
					spi_config.sck_pin  = SPI_SCK_PIN;
					APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
					m_spi_is_enable = true;
				}

				if(!m_motion_sensor_is_enable)
				{
					#if SensorJustSleep
					enterOrExitSleepmode(0xB1);
					#else
					enterOrExitSleepmode(0x1B);
					#endif
					m_motion_sensor_is_enable = true;
				}
				
				err_code = app_timer_start(m_motion_timer_id, MOTION_INTERVAL, NULL);
				APP_ERROR_CHECK(err_code);
				#if DEBUG_TRACE_ENABLE
				NRF_LOG_INFO("MOTION\r\n");
				#endif
				m_time_measurement = 0;
			}
			m_three_keys_pressed = true;	
		}
		else
		{
			m_three_keys_pressed = false;
		}
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
	#if 1
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);

    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
	#else
	uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        bsp_event_handler);
    APP_ERROR_CHECK(err_code);
	#endif
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

static void qdec_event_handler(nrf_drv_qdec_event_t event)
{
    if (event.type == NRF_QDEC_EVENT_REPORTRDY)
    {
        m_accdblread        = event.data.report.accdbl;
        m_accread           += event.data.report.acc;
        //m_report_ready_flag = true;
        //nrf_drv_qdec_disable();
				#if Negate_scroll
				m_accread = -m_accread;
				#endif
				if(work_mode == BT_mode)
				{
					mouse_scroll_send(m_accread);
					m_accread = 0;
				}
				else
				{
					//NRF_LOG_INFO("m_accdblread:%d\r\n",m_accdblread);
					//NRF_LOG_INFO("m_accread:%d\r\n",m_accread);
					#if Enable_2_point_4G
					active_event |= scroll_event;	
					#endif			
				}			
    }
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    //NRF_LOG_INFO("Transfer completed.\r\n");
		//NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
		#if 0
    //if (m_rx_buf[0] != 0)
    //{
        //NRF_LOG_INFO(" Received: \r\n");
        //NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    //}
		#endif
}

static uint8_t random_vector_generate(uint8_t * p_buff, uint8_t size)
{
    uint32_t err_code;
    uint8_t  available;

		do
		{
			nrf_drv_rng_bytes_available(&available);
		}while(available<6);
    uint8_t length = MIN(size, available);
	
		//NRF_LOG_INFO("available=%d\r\n",available);

    err_code = nrf_drv_rng_rand(p_buff, length);
    APP_ERROR_CHECK(err_code);

    return length;
}

void set_generate_random_addr(void)
{
		uint32_t err_code;
		//ble_gap_addr_t my_device_addr = {BLE_GAP_ADDR_TYPE_RANDOM_STATIC,{1,2,3,4,5,6}};
		err_code = nrf_drv_rng_init(NULL);
		APP_ERROR_CHECK(err_code);
		random_vector_generate(my_device_addr.addr,6);
		nrf_drv_rng_uninit();

		#if DEBUG_TRACE_ENABLE
		NRF_LOG_HEXDUMP_INFO(my_device_addr.addr,6);
		NRF_LOG_INFO("\r\n");
		#endif
		my_device_addr.addr[5] |= 0xC0;

		#if DEBUG_TRACE_ENABLE
		NRF_LOG_HEXDUMP_INFO(my_device_addr.addr,6);
		NRF_LOG_INFO("\r\n");
		#endif
		
		err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &my_device_addr);
		APP_ERROR_CHECK(err_code);
}

static void fs_evt_handler(fs_evt_t const * const evt, fs_ret_t result)
{
	//uint32_t m_datas[DATA_SIZE]={0,1,2,3,88,5};
	uint8_t i = 0;
     if( (evt->id == FS_EVT_STORE) && (result == FS_SUCCESS) )
     {     
		  switch(fs_page_num)
		  {
				case 2:
				m_flash_write(3);
				break;
				
				case 3:
				if(!m_spi_is_enable)
				{
					nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
					spi_config.ss_pin   = SPI_SS_PIN;
					spi_config.miso_pin = SPI_MISO_PIN;
					spi_config.mosi_pin = SPI_MOSI_PIN;
					spi_config.sck_pin  = SPI_SCK_PIN;
					APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
					m_spi_is_enable = true;
				}

				if(!m_motion_sensor_is_enable)
				{
					#if SensorJustSleep
					enterOrExitSleepmode(0xB1);
					#else
					enterOrExitSleepmode(0x1B);
					#endif
					m_motion_sensor_is_enable = true;
				}
				nrf_delay_ms(5);
				NVIC_SystemReset();	
				break;

				case 4:
				m_flash_write(2);
				#if DEBUG_TRACE_ENABLE
				NRF_LOG_RAW_INFO("going to write flash page: 2  \r\n");
				#endif
				break;

				default: break;
		  }

     	#if DEBUG_TRACE_ENABLE
		NRF_LOG_RAW_INFO("write flash page: %d  \r\n", fs_page_num);
		#endif
     }
     else if( (evt->id == FS_EVT_ERASE) && (result == FS_SUCCESS) )
     {
          switch(fs_page_num)
					{
						case 0:
						if(Authorization)
						{							
								for(i=0;i<6;i++)
								{
										m_datas[i] = my_device_addr.addr[i];
								}
								fs_store(&fs_config, fs_config.p_start_addr, m_datas,6,NULL);
						}
						break;
						
						case 1:
						#if ENABLE_CLEAR_AUTH
						if(m_just_erase_flash==false)
						{
							#if ENABLE_AUTH
							for(i=0;i<6;i++)
							{
									m_datas[i] = BDaddrSourceEncryptData[i];
							}
							fs_store(&fs_config_encrypt, fs_config_encrypt.p_start_addr, m_datas,6,NULL);
							#endif
						}
						else
						{
							m_just_erase_flash = false;
							#if DEBUG_TRACE_ENABLE
							NRF_LOG_RAW_INFO("just erase flash\r\n");
							#endif
						}
						#else
						#if ENABLE_AUTH
						for(i=0;i<6;i++)
						{
								m_datas[i] = BDaddrSourceEncryptData[i];
						}
						fs_store(&fs_config_encrypt, fs_config_encrypt.p_start_addr, m_datas,6,NULL);
						#endif
						#endif
						break;
						
						case 2:
						m_datas[0] = 0x00;
						fs_store(&fs_config_work_mode, fs_config_work_mode.p_start_addr, m_datas,1,NULL);
						//change_work_mode_completed = true;
						break;
						
						#if 1
						case 4:
						for(int i=0; i<6; i++ )
						{
								m_datas[i] = KeepConnectReport[i+1];
						}	
						fs_store(&fs_config_encrypt, (fs_config_encrypt.p_start_addr+6), m_datas,6,NULL);
						break;

						#endif
						
						default: break;
						
					}
					
          //NRF_LOG_RAW_INFO("erase_flag: %d  \r\n", erase_flag);
     }
     else if (result != NRF_SUCCESS) 
     {
     	#if DEBUG_TRACE_ENABLE
         NRF_LOG_RAW_INFO("fstorage error and code: %d  \r\n", result);
		#endif
     }
}

uint32_t const * address_of_page(uint8_t page_num)
{
		switch(page_num)
			{
				case 0:
					return fs_config.p_start_addr;
					break;
				
				case 1:
					return fs_config_encrypt.p_start_addr;
					break;
				
				case 2:
					return fs_config_work_mode.p_start_addr;
					break;
				
				case 3:
					return fs_config_work_mode_change_indicate.p_start_addr;
					break;
				
				#if 1
				case 4:
					return (fs_config_encrypt.p_start_addr+6);
					break;
				#endif
				
				default: 
					return fs_config.p_start_addr;
					break;
			}	
}

bool m_flash_read(const uint32_t* address,uint8_t* bd_addr)
{
		bool valid_addr = false;
    uint32_t* m_addr = (uint32_t*)address;
    //uint32_t buff[DATA_SIZE];
    #if DEBUG_TRACE_ENABLE
		NRF_LOG_RAW_INFO("%08x ",(uint32_t)address);
	#endif
    for(int i=0; i<6; i++ )
    {
				*(bd_addr+i) = (uint8_t)(*m_addr);
				m_addr++;
				#if DEBUG_TRACE_ENABLE
				NRF_LOG_RAW_INFO("%02x ",*(bd_addr+i));
				#endif
				if((*(bd_addr+i))!=0xFF)
				{
					valid_addr = true;
				}
    }
		#if 0
    for(int i = 0; i<6; i++)
    {
      NRF_LOG_RAW_INFO("%02x ",(uint8_t)buff[i]);
			my_device_addr.addr[i] = (uint8_t)buff[i];
			if(my_device_addr.addr[i]!=0xFF)
			{
				valid_addr = true;
			}
    }
		#endif
		return valid_addr;
}

void m_flash_write(uint8_t page_num)
{
      fs_ret_t ret;
      //erase_flag=1;
      fs_page_num = page_num;
	
			switch(fs_page_num)
			{
				case 0:
					ret = fs_erase(&fs_config, address_of_page(fs_page_num), 1,NULL);
					break;
				case 1:
					ret = fs_erase(&fs_config_encrypt, address_of_page(fs_page_num), 1,NULL);
					break;
				case 2:	
					ret = fs_erase(&fs_config_work_mode, address_of_page(fs_page_num), 1,NULL);
					break;
				case 3:				
					if(just_erase_page_3)
					{
						just_erase_page_3 = false;
						ret = fs_erase(&fs_config_work_mode_change_indicate, address_of_page(fs_page_num), 1,NULL);
					}
					else
					{
						m_datas[0] = 0x01;
						ret = fs_store(&fs_config_work_mode_change_indicate, fs_config_work_mode_change_indicate.p_start_addr, m_datas,1,NULL);
						change_work_mode_indicate = true;
					}
					break;
					
				case 4:
					ret = fs_erase(&fs_config_encrypt, address_of_page(1), 1,NULL);
					break;
				default: break;
			}	
      //ret = fs_erase(&fs_config, address_of_page(fs_page_num), 1,NULL);
	
      if (ret != FS_SUCCESS)
      {
      	#if DEBUG_TRACE_ENABLE
		NRF_LOG_INFO("fs_erase error\r\n");
		#endif
      }
      else
      {
					//NRF_LOG_INFO("fs_erase FS_SUCCESS\r\n");
      }
}

static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        uint32_t i;
		AverageVoltage = 0;
        for (i = 0; i < p_event->data.done.size; i++)
        {
            //NRF_LOG_INFO("Current sample value: %d\r\n", p_event->data.done.p_buffer[i]);
			AverageVoltage += p_event->data.done.p_buffer[i];
        }
		AverageVoltage /= p_event->data.done.size;
		#if DEBUG_TRACE_ENABLE
		NRF_LOG_INFO("ADC done size: %d\r\n", p_event->data.done.size);
		NRF_LOG_INFO("Average value: %d\r\n", AverageVoltage);
		#endif
		m_adc_is_enable = false;
		nrf_drv_adc_uninit();
    }
}


static void adc_config(void)
{
    ret_code_t ret_code;
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;

    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    APP_ERROR_CHECK(ret_code);

    nrf_drv_adc_channel_enable(&m_channel_config);
}




#if Enable_2_point_4G

//2.4G


#if 1
void set_BT_work_mode()
{
	uint32_t * work_mode_addr;
	work_mode_addr = (uint32_t *)0x0003e800;			
	flash_page_erase(work_mode_addr);
	flash_word_write((work_mode_addr), 0x01);
	work_mode_addr = (uint32_t *)0x0003e400;			
	//flash_page_erase(work_mode_addr);
	flash_word_write((work_mode_addr), 0x01);
	if((uint8_t)*( (uint32_t *)0x0003e800)==0x01&&(uint8_t)*( (uint32_t *)0x0003e400)==0x01)
	{
		if(!m_spi_is_enable)
		{
			nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
			spi_config.ss_pin   = SPI_SS_PIN;
			spi_config.miso_pin = SPI_MISO_PIN;
			spi_config.mosi_pin = SPI_MOSI_PIN;
			spi_config.sck_pin  = SPI_SCK_PIN;
			APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
			m_spi_is_enable = true;
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_INFO("enable spi\r\n");
			#endif
		}
			
		switch(peer_id_cnt)
		{
			case 1:
				enterOrExitjustSleepmode(0xB1);
				peer_id_cnt = 0;
				break;
			
			case 2:
				enterOrExitPowerDownMode(0x1B);
				peer_id_cnt = 0;
				break;
			
			default: break;
		}
		nrf_delay_ms(5);					
		NVIC_SystemReset();
	}
}
#endif

void write_encrypt_information()
{
	uint32_t *encrypt_information;
	encrypt_information = (uint32_t *)0x0003ec00;
	flash_page_erase(encrypt_information);
	nrf_delay_ms(5);
	for(uint8_t j=0;j<6;j++)
	{
		flash_word_write(encrypt_information+j, BDaddrSourceEncryptData[j]);
		nrf_delay_ms(5);
		#if DEBUG_TRACE_ENABLE
		NRF_LOG_RAW_INFO("%02x ",(uint8_t)*(encrypt_information+j));
		#endif
		if((uint8_t)*(encrypt_information+j)!= BDaddrSourceEncryptData[j])
		{
			flash_page_erase(encrypt_information);
			nrf_delay_ms(5);
			j = 0;
		}
	}
	set_BT_work_mode();
}

void clear_encrypt_information()
{
	uint32_t *encrypt_information;
	encrypt_information = (uint32_t *)0x0003ec00;
	flash_page_erase(encrypt_information);
	nrf_delay_ms(5);
	NVIC_SystemReset();
}

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
	#if 1
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
						m_TX_SUCCESS = true;
						#if 0
						if(flag_rf_paged)
						{
							//tx_payload.data[1] = 0;
							tx_payload.data[2] = 0;
							tx_payload.data[3] = 0;
							tx_payload.data[4] = 0;
							tx_payload.data[5] = 0;
							tx_payload.data[6] = 0;
						}
						#endif
						//NRF_LOG_RAW_INFO("*");
            //NRF_LOG_DEBUG("TX SUCCESS EVENT\r\n");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            //NRF_LOG_DEBUG("TX FAILED EVENT\r\n");
			//m_TX_FAIL = true;
			//m_TX_attemp = p_event->tx_attempts;
            (void) nrf_esb_flush_tx();
			//nrf_esb_flush_rx();
            //(void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
						m_RX_SUCCESS = true;
            //NRF_LOG_RAW_INFO("RX RECEIVED EVENT\r\n");
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length > 0)
                {
                    //NRF_LOG_DEBUG("RX RECEIVED PAYLOAD\r\n");
                }
            }
            break;
    }
    //NRF_GPIO->OUTCLR = 0xFUL << 12;
    //NRF_GPIO->OUTSET = (p_event->tx_attempts & 0x0F) << 12;
		#endif
}

uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0x3e, 0x14, 0x5b, 0x6d};
    //uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    //uint8_t addr_prefix[8] = {0xa3, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };


    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;//NRF_ESB_PROTOCOL_ESB;//
    nrf_esb_config.retransmit_delay         = 1000;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_1MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = true;//false;//
		nrf_esb_config.payload_length           = 5;
		nrf_esb_config.crc                      = NRF_ESB_CRC_16BIT;//NRF_ESB_CRC_OFF;//
		nrf_esb_config.retransmit_count         = 0;

		
    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);
		
		err_code = nrf_esb_set_address_length(3);
		VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

	#if 0
    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);
	#endif

	err_code = nrf_esb_update_prefix(0,0xa3);
	VERIFY_SUCCESS(err_code);
    //err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    //VERIFY_SUCCESS(err_code);
	
	 err_code = nrf_esb_set_tx_power(NRF_ESB_TX_POWER_4DBM);
	 VERIFY_SUCCESS(err_code);

    return err_code;
}

uint32_t modify_base_address_0(uint8_t addr_0)
{
	uint32_t err_code;
	uint8_t base_addr_0[4] = {0x3E, 0xBC, 0xBC, 0xBC};
	base_addr_0[0] = addr_0;
	err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);
}

uint32_t modify_base_address_0_encrypt(void)
{
	uint32_t err_code;
	uint8_t base_addr_0[4] = {0xe3, 0x41, 0x5b, 0x6d};
	err_code = nrf_esb_set_base_address_0(base_addr_0);
  VERIFY_SUCCESS(err_code);
}

void clocks_start( void )
{
    // Start HFCLK and wait for it to start.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

#if 1
void system_off( void )
{
#ifdef NRF51
    NRF_POWER->RAMON |= (POWER_RAMON_OFFRAM0_RAM0Off << POWER_RAMON_OFFRAM0_Pos) |
                        (POWER_RAMON_OFFRAM1_RAM1Off << POWER_RAMON_OFFRAM1_Pos);
#endif //NRF51
#ifdef NRF52
    NRF_POWER->RAM[0].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[1].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[2].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[3].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[4].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[5].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[6].POWER = RAM_RETENTION_OFF;
    NRF_POWER->RAM[7].POWER = RAM_RETENTION_OFF;
#endif //NRF52

    // Turn off LEDs before sleeping to conserve energy.
    //bsp_board_leds_off();

    // Set nRF5 into System OFF. Reading out value and looping after setting the register
    // to guarantee System OFF in nRF52.
    NRF_POWER->SYSTEMOFF = 0x1;
    (void) NRF_POWER->SYSTEMOFF;
    while (true);
}
#endif

void NRF_power_manage( void )
{
    // WFE - SEV - WFE sequence to wait until a radio event require further actions.
    __WFE();
    __SEV();
    __WFE();
}

void driver_rf_channel_switch(void)
{
		uint32_t err_code;
    rf_channel++;
    rf_channel &= 0x0f; //<=15
    nrf_esb_set_rf_channel(RF_CHANNEL_TABLE[rf_channel]);
		APP_ERROR_CHECK(err_code);
	//NRF_LOG_RAW_INFO("%d",RF_CHANNEL_TABLE[rf_channel]);
}

/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context)
{
	uint32_t err_code;
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:

						polling_motion = true;
						nrf_gpio_ports_read(0, 1, &button_state);
		
						#if DEBUG_TRACE_ENABLE
						//NRF_LOG_RAW_INFO("button_state=%08x\r\n",button_state);
						#endif
		
						#if ENABLE_PAIR_TO_CHANGE_WORK_MODE
						if(((~(button_state>>15))&0x01))
						{
							//button_state = ((~(button_state>>15))&0x01);
							if(!change_work_mode_start)
							{
								change_work_mode_start = true;
								#if DEBUG_TRACE_ENABLE
								NRF_LOG_RAW_INFO("set bt mode\r\n");
								#endif
							}
						}	
						else
						{
							if(change_work_mode_start)
							{
								#if !ENABLE_CLEAR_AUTH
								set_BT_work_mode();
								#else
								clear_encrypt_information();
								#endif
							}
						}
						#endif			
						
						if(((~(button_state>>17))&0x01))
						{
							m_time_thress_key_pressed_measurement++;
							#if ENABLE_LONG_PRESS_DIP_TO_CHANGE_WORK_MODE
							if(m_time_thress_key_pressed_measurement==625)
							{
								bsp_board_led_on(BSP_BOARD_LED_0);
								bsp_board_led_on(BSP_BOARD_LED_1);
							}
							#endif
							button_state = ((~(button_state>>17))&0x01);
							#if 0
							if(button_state!=previous_button_state)
							{
								if(cpiCntStore==0)
								{
									change_cpi_flag = true;
								}
							}
							#endif
						}
						else
						{
							#if ENABLE_LONG_PRESS_DIP_TO_CHANGE_WORK_MODE
							if(m_time_thress_key_pressed_measurement>=625)
							{
								set_BT_work_mode();							
							}
							else
							#endif
							{
								if(m_time_thress_key_pressed_measurement)
								{
									if(cpiCntStore==0)
									{
										change_cpi_flag = true;
									}
								}
							}
							m_time_thress_key_pressed_measurement = 0;
							
							button_state = ((~(button_state>>18))&0x1F);
							if(button_state!=previous_button_state)
							{
								#if 0
								//tx_payload.data[1] = button_state;
								if(button_cnt>=6)
								{
									button_cnt = 0;
									button_buf_index = 0;
									read_buf_index = 0;
								}
								if(button_buf_index>=5)
								{
									button_buf_index = 0;	
								}
								button_buffer[button_buf_index++] = button_state;
								button_cnt++;
								#else
								tx_payload.data[1] = button_state;
								#endif

								active_event |= key_event;
								//NRF_LOG_RAW_INFO("@");
							}
						}
									
						previous_button_state = button_state;
						
						#if 1			
						if(++m_time_voltage_measurement>=1420)
						{
							#if 1
							if(!m_adc_is_enable)
							{
								adc_config();
								APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
								m_adc_is_enable = true;
								m_voltage_measurement_cnt = ADC_BUFFER_SIZE;
							}
							#endif
							if(m_voltage_measurement_cnt)
							{
								nrf_drv_adc_sample();
								
								if((--m_voltage_measurement_cnt)==0)
								{
									m_time_voltage_measurement = 0;
								}
							}
						}
						#endif
						#if VOLTAGE_CHECK_ENABLE
						if(AverageVoltage>=425)//3V
						#else
						if(AverageVoltage>=0)
						#endif
						{
							sleep_time_cnt++;
						}
						else
						{
							#if 1
							if(++m_time_low_voltage_alter==142)
							{
								bsp_board_led_off(BSP_BOARD_LED_1);
							}
							
							if(m_time_low_voltage_alter>=426)
							{
								m_time_low_voltage_alter = 0;
								bsp_board_led_on(BSP_BOARD_LED_1);
							}
							#endif
						}
						
						#if 1
						if(m_cold_boot)
						{
							if(m_cold_boot_cnt==0)
							{
								bsp_board_led_on(BSP_BOARD_LED_1);
							}
							if(++m_cold_boot_cnt>284)
							{
								m_cold_boot_cnt = 0;
								m_cold_boot = false;
								bsp_board_led_off(BSP_BOARD_LED_1);
							}
						}
						
						if(cpiCntStore)
						{
							if(++m_time_measurement>=65)//125
							{
								m_time_measurement = 0;
								bsp_board_led_invert(BSP_BOARD_LED_0);
								cpiCntStore--;
							}
						}
						#endif
						
            break;

        default:
            //Do nothing.
            break;
    }
}


void button_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//NRF_LOG_RAW_INFO("pin:%d\r\n",pin);
	//NRF_LOG_RAW_INFO("action:%d\r\n",action);
	nrf_drv_gpiote_in_event_disable(15);
	nrf_drv_gpiote_in_event_disable(17);
	nrf_drv_gpiote_in_event_disable(18);
	nrf_drv_gpiote_in_event_disable(19);
	nrf_drv_gpiote_in_event_disable(20);
	nrf_drv_gpiote_in_event_disable(21);
	nrf_drv_gpiote_in_event_disable(22);
	nrf_drv_gpiote_in_event_disable(23);
	sleep_time_cnt = 0;
	nrf_drv_timer_enable(&TIMER_LED);
	#if ENABLE_RTC_TIMER
	if(m_rtc_timer_is_enable)
	{
		m_time_keep_connection = 0;
		nrf_drv_rtc_disable(&rtc);
		m_rtc_timer_is_enable = false;
	}
	#endif
	#if DEBUG_TRACE_ENABLE
	NRF_LOG_RAW_INFO("wake\r\n");
	#endif
}

#if ENABLE_RTC_TIMER
/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	uint32_t err_code;
	#if 1
    if (int_type == NRF_DRV_RTC_INT_COMPARE0)
    {
        //nrf_gpio_pin_toggle(COMPARE_EVENT_OUTPUT);
		#if DEBUG_TRACE_ENABLE
		NRF_LOG_RAW_INFO("NRF_DRV_RTC_INT_COMPARE0\r\n");
		#endif
		nrf_drv_rtc_counter_clear(&rtc);
		nrf_drv_rtc_disable(&rtc);
		if(m_qdec_is_enable)
		{
			nrf_drv_qdec_uninit();
			m_qdec_is_enable = false;
			nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pullup);
			nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pullup);
		}
		system_off();
    }
    else 
	#endif
	if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        //nrf_gpio_pin_toggle(TICK_EVENT_OUTPUT);
		if(++m_time_keep_connection==80)
		{
			m_time_keep_connection = 0;
			if(m_qdec_is_enable)
			{
				nrf_drv_qdec_uninit();
				m_qdec_is_enable = false;
				nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pullup);
				nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pullup);
			}
			nrf_drv_rtc_disable(&rtc);
			#if DEBUG_TRACE_ENABLE
			NRF_LOG_RAW_INFO("NRF_DRV_RTC_INT_TICK\r\n");
			#endif
		}
    }
}


/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc,false);

    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
    err_code = nrf_drv_rtc_cc_set(&rtc,0,10 * 8,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}
#endif

/** @brief Function for erasing a page in flash.
 *
 * @param page_address Address of the first word in the page to be erased.
 */
static void flash_page_erase(uint32_t * page_address)
{
    // Turn on flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Erase page:
    NRF_NVMC->ERASEPAGE = (uint32_t)page_address;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash erase enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}


/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
static void flash_word_write(uint32_t * address, uint32_t value)
{
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    *address = value;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}

#if ENABLE_RF_TEST_MODE
static void radio_disable(void)
{
    NRF_RADIO->SHORTS          = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
#ifdef NRF51
    NRF_RADIO->TEST            = 0;
#endif
    NRF_RADIO->TASKS_DISABLE   = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0)
    {
        // Do nothing.
    }
    NRF_RADIO->EVENTS_DISABLED = 0;
}

/**
 * @brief Function for turning on the TX carrier test mode.
*/
void radio_tx_carrier(uint8_t txpower, uint8_t mode, uint8_t channel)
{
    radio_disable();
    NRF_RADIO->SHORTS     = RADIO_SHORTS_READY_START_Msk;
    NRF_RADIO->TXPOWER    = (txpower << RADIO_TXPOWER_TXPOWER_Pos);
    NRF_RADIO->MODE       = (mode << RADIO_MODE_MODE_Pos);
    NRF_RADIO->FREQUENCY  = channel;
#ifdef NRF51
    NRF_RADIO->TEST       = (RADIO_TEST_CONST_CARRIER_Enabled << RADIO_TEST_CONST_CARRIER_Pos) \
                            | (RADIO_TEST_PLL_LOCK_Enabled << RADIO_TEST_PLL_LOCK_Pos);
#endif
    NRF_RADIO->TASKS_TXEN = 1;
}
#endif


#endif

/**@brief Function for application main entry.
 */
int main(void)
{
	uint8_t BDaddrEncryptData[6];
	uint8_t BDaddrData[6];
	int16_t motion_x, motion_y;
	uint16_t i = 0;

	uint32_t ResetReason = 0;
	uint32_t time_ms = 8; //Time(in miliseconds) between consecutive compare events.
	uint32_t time_ticks;
	uint32_t hop_frequency_timeout = 0;
	uint32_t err_code;
	uint32_t * flash_addr;
	
	uint16_t pairing_loop_count = 0;
	uint16_t page_loop_count = 0;                     //0-749 -10dbm, 750-1499, -5dbm, 1500-2250 0dbm; 750*4ms=3s, 3000*3ms=9s
	uint8_t page_common_count = 0;
	uint8_t m_TX_attempt = 0;
	uint8_t my_page_rx_count = 0;
	uint8_t my_temp = 0;
	uint8_t k = 0;
	uint8_t n = 0;
	
	bool     erase_bonds;
	bool hop_frequency_mode = true;
		
	uint32_t pg_size = NRF_FICR->CODEPAGESIZE;
    uint32_t pg_num  = NRF_FICR->CODESIZE - 1;  // Use last page in flash
	
	//ble_gap_addr_t device_addr;
	//ble_gap_addr_t my_device_addr = {BLE_GAP_ADDR_TYPE_RANDOM_STATIC,{1,2,3,4,5,6}};
		
	//usrPaw3205ReSynchronousSPI();

    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
#if !ENABLE_PAIR_TO_CHANGE_WORK_MODE
#if !DEBUG_TRACE_ENABLE
	nrf_gpio_cfg_input(9, GPIO_PIN_CNF_PULL_Pullup);
	for(k=0;k<5;k++)
	{
		nrf_gpio_ports_read(0, 1, &button_state);
		if(((~(button_state>>9))&0x01)==0) break;
		nrf_delay_ms(5);	
	}
	
	if(k>=5)
	{
		work_mode=BT_mode;
		nrf_gpio_cfg_output(9);
		nrf_gpio_pin_write(9, 0);
	}
	else
	{
		work_mode=RF_2_point_4_mode;
	}
#else
	work_mode=BT_mode;//RF_2_point_4_mode;//
#endif
#else	
	flash_addr = (uint32_t *)0x0003e800;			
	
	#if DEBUG_TRACE_ENABLE
	for(n=0;n<1;n++)
	{
		NRF_LOG_RAW_INFO("%02x ",(uint8_t)*(flash_addr));
	}
	NRF_LOG_RAW_INFO("\r\n");
	#endif
	//if(!(uint8_t)*(flash_addr))
	if((uint8_t)*(flash_addr))
	{
		work_mode=BT_mode;
	}
	else
	{
		work_mode=RF_2_point_4_mode;
	}
#endif
	
	#if ENABLE_RF_TEST_MODE
	nrf_gpio_cfg_input(18, GPIO_PIN_CNF_PULL_Pullup);
	nrf_gpio_cfg_input(19, GPIO_PIN_CNF_PULL_Pullup);
	nrf_gpio_cfg_input(20, GPIO_PIN_CNF_PULL_Pullup);
	for(k=0;k<200;k++)
	{
		nrf_gpio_ports_read(0, 1, &button_state);
		if(((~(button_state>>18))&0x07)!=0x04) break;
		nrf_delay_ms(50);	
	}
	
	if(k>=200)
	{
		#if DEBUG_TRACE_ENABLE
		NRF_LOG_RAW_INFO("enter RF test mode\r\n");
		#endif
		bsp_board_leds_init();
		bsp_board_led_on(BSP_BOARD_LED_0);
		bsp_board_led_on(BSP_BOARD_LED_1);
		k = 0;
		n = 0;
		radio_tx_carrier(RADIO_TXPOWER_TXPOWER_Pos4dBm, RADIO_MODE_MODE_Nrf_1Mbit, 40);
		while(1)
		{
			nrf_gpio_ports_read(0, 1, &button_state);
			button_state = ((~(button_state>>20))&0x01);
			if(button_state!=previous_button_state)
			{
				if((++k)%2==0)
				{
					n++;
					switch(n)
					{
						case 1:
							radio_tx_carrier(RADIO_TXPOWER_TXPOWER_Pos4dBm, RADIO_MODE_MODE_Nrf_1Mbit, 2);
							#if DEBUG_TRACE_ENABLE
							NRF_LOG_RAW_INFO("test 02 channel\r\n");
							#endif
							break;
						case 2:
							radio_tx_carrier(RADIO_TXPOWER_TXPOWER_Pos4dBm, RADIO_MODE_MODE_Nrf_1Mbit, 40);
							#if DEBUG_TRACE_ENABLE
							NRF_LOG_RAW_INFO("test 40 channel\r\n");
							#endif
							break;
						case 3:
							radio_tx_carrier(RADIO_TXPOWER_TXPOWER_Pos4dBm, RADIO_MODE_MODE_Nrf_1Mbit, 80);
							n = 0;
							#if DEBUG_TRACE_ENABLE
							NRF_LOG_RAW_INFO("test 80 channel\r\n");
							#endif
							break;
						default: break;
					}
				}
				
				if(k>=200)
				{
					k = 0;
				}
			}
			previous_button_state = button_state;
			
			NRF_LOG_PROCESS() ;
		}
	}
	#endif
	
#if INCLUD_BT_MODE
		if(work_mode==BT_mode)
		{
			timers_init();
			buttons_leds_init(&erase_bonds);
			bsp_board_leds_off();

				ble_stack_init();
				scheduler_init();
				sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
				usrPaw3205ReSynchronousSPI();
				
				#if 0
				while(1)
				{
					//bsp_board_leds_off();
            		bsp_board_led_on(BSP_BOARD_LED_0);
					nrf_delay_ms(500);
					bsp_board_led_off(BSP_BOARD_LED_0);
					nrf_delay_ms(500);
				}
				#endif
				//erase_bonds = true;
				#if 0
				erase_bonds = false;
				peer_manager_init(erase_bonds);
				if (erase_bonds == true)
				{
						NRF_LOG_INFO("Bonds erased!\r\n");
				}
				#endif
				//gap_params_init();
				advertising_init();
				services_init();
				//sensor_simulator_init();
				conn_params_init();

				// Start execution.
				#if DEBUG_TRACE_ENABLE
				NRF_LOG_INFO("HID Mouse Start!\r\n");
				#endif
				//timers_start();
				
				#if 1
				err_code = sd_ble_gap_address_get(&device_addr);	
				APP_ERROR_CHECK(err_code);
				#if DEBUG_TRACE_ENABLE
				NRF_LOG_INFO("addr_type=%d\r\n",device_addr.addr_type);
				NRF_LOG_INFO("addr1=");
				NRF_LOG_HEXDUMP_INFO(device_addr.addr,6);
				NRF_LOG_INFO("\r\n");
				#endif
				#if ENABLE_AUTH
				memcpy(&KeepConnectReport[1],encrypt(device_addr.addr,encryptPassword),6);
				#endif
				//memcpy(&KeepConnectReport[1],device_addr.addr,6);
				//NRF_LOG_HEXDUMP_INFO(KeepConnectReport,7);
				#else
				err_code = sd_ble_gap_address_get((ble_gap_addr_t*)&KeepConnectReport[1]);	
				APP_ERROR_CHECK(err_code);
				
				//NRF_LOG_INFO("addr_type=%d\r\n",device_addr.addr_type);
				KeepConnectReport[1] = 0xFF;
				NRF_LOG_INFO("addr1=");
				NRF_LOG_HEXDUMP_INFO(((ble_gap_addr_t*)&KeepConnectReport[2])->addr,6);
				NRF_LOG_INFO("\r\n");
				#endif

				#if 1
				// Initialize hardware
				if(!m_qdec_is_enable)
				{
					err_code = nrf_drv_qdec_init(NULL, qdec_event_handler);
					APP_ERROR_CHECK(err_code);
					nrf_qdec_reportper_to_value(0);
					nrf_drv_qdec_enable(); 
					m_qdec_is_enable = true;
				}
				#endif
				
				nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
				spi_config.ss_pin   = SPI_SS_PIN;
				spi_config.miso_pin = SPI_MISO_PIN;
				spi_config.mosi_pin = SPI_MOSI_PIN;
				spi_config.sck_pin  = SPI_SCK_PIN;
				APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
				m_spi_is_enable = true;
				turnOn();
				m_motion_sensor_is_enable = true;
				
				fs_init();
				#if ENABLE_AUTO_AUTH
				if(!m_flash_read(address_of_page(1),BDaddrEncryptData))
				{
					memcpy(BDaddrSource,device_addr.addr,6);
					for(k=0;k<6;k++)
					{
						BDaddrSource[k] += 0x48 + k;
					}
					memcpy(BDaddrSourceEncryptData,encrypt(BDaddrSource,BDaddrPassword),6);
					m_flash_write(1);
				}
				#endif
			
				m_flash_read(address_of_page(3),BDaddrEncryptData);
				if(BDaddrEncryptData[0]==0xFF)
				{
					sd_power_reset_reason_get(&ResetReason);
					#if DEBUG_TRACE_ENABLE
					NRF_LOG_INFO("ResetReason=%04x\r\n",ResetReason);
					#endif
					if(ResetReason&0x10000)
					{
						#if SensorJustSleep
						//usrPaw3205ReSynchronousSPI();
						//enterOrExitSleepmode(0xB1);
						#else
						//enterOrExitSleepmode(0x1B);
						#endif
						m_cold_boot = false;
					}
					else
					{
						m_cold_boot = true;
						//usrPaw3205ReSynchronousSPI();
					}
				}
				else
				{
					m_cold_boot = true;
					just_erase_page_3 = true;
					m_flash_write(3);
					#if DEBUG_TRACE_ENABLE
					NRF_LOG_INFO("Work mode changed, cold boot\r\n");
					#endif
				}
				#if DEBUG_TRACE_ENABLE
				m_flash_read(address_of_page(2),BDaddrEncryptData);
				#endif
				
				#if 1
				m_flash_read(address_of_page(1),BDaddrEncryptData);	
				memcpy(BDaddrData,encrypt(BDaddrEncryptData,BDaddrPassword),6);		
				for(k=0;k<6;k++)
				{
					BDaddrData[k] -= 0x48 + k;
				}
				for(k=0;k<6;k++)
				{
					if(device_addr.addr[k]!=BDaddrData[k])
					{
						break;
					}
				}
				if(k<6)
				{
					Authorization = false;
					just_erase_page_3 = true;
					m_flash_write(3);
					
					m_flash_read(address_of_page(4),BDaddrEncryptData);
					for(k=0;k<6;k++)
					{
						if(BDaddrEncryptData[k] != 0xFF) break;
					}
					if(k>=6)
					{
						m_flash_write(4);
					}
					else
					{
						m_flash_write(2);
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_RAW_INFO("going to change work mode \r\n");
						#endif
					}	
					while(1)
					{
						app_sched_execute();
						NRF_LOG_PROCESS() ;
					}
				}
				else
				{ 
					Authorization = true;
				}
				#else
				//Authorization = false;
				Authorization = true;
				
				#endif	
				gap_params_init();			
				
				#if 1
				if(!Authorization)	
				{	
						m_cold_boot = false;
						bsp_board_led_on(BSP_BOARD_LED_0);
						erase_bonds = true;
						peer_manager_init(erase_bonds);
						
						ble_advertising_restart_without_whitelist();
				}
				else
				{
						erase_bonds = false;
						peer_manager_init(erase_bonds);	
						
						peer_id_cnt = 0;
						pm_peer_id_t current_peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
						while (current_peer_id != PM_PEER_ID_INVALID)
						{
								peer_id_stored[peer_id_cnt++] = current_peer_id;
								current_peer_id = pdb_next_peer_id_get(current_peer_id);
						}
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_INFO("peer_id_cnt=%d\r\n",peer_id_cnt);
						#endif
										
						if(m_flash_read(address_of_page(0),my_device_addr.addr))
						{
							err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &my_device_addr);
							APP_ERROR_CHECK(err_code);
							advertising_start();
						}
						else
						{
							m_cold_boot = false;
							if(peer_id_cnt)
							{
								err_code = pm_peers_delete();
	        					APP_ERROR_CHECK(err_code);
							}
							ble_advertising_restart_without_whitelist();
						}
				}
				if (erase_bonds == true)
				{
					#if DEBUG_TRACE_ENABLE
						NRF_LOG_INFO("Bonds erased!\r\n");
					#endif
				}
		#endif		
				// Enter main loop.
				for (;;)
				{
						app_sched_execute();
						if (NRF_LOG_PROCESS() == false)
						{
								//sleep_mode_enter();
								#if 0
								if(change_work_mode_completed)
								{
									m_flash_read(address_of_page(2),BDaddrEncryptData);
									if(BDaddrEncryptData[0]!=0xFF)
									{
										change_work_mode_completed = false;
										m_flash_write(3);
										//NVIC_SystemReset();
									}
								}
								
								
								if(change_work_mode_indicate)
								{
									m_flash_read(address_of_page(3),BDaddrEncryptData);
									if(BDaddrEncryptData[0]!=0xFF)
									{
										change_work_mode_indicate = false;
										NVIC_SystemReset();
									}
								}
								#endif
								
								power_manage();
						}
				}
		}
		else
#endif
		{
			#if INCLUD_2_point_4G//Enable_2_point_4G
				clocks_start();
				//APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
				//buttons_leds_init(&erase_bonds);
				err_code = esb_init();
				APP_ERROR_CHECK(err_code);
			
				#if ENABLE_RTC_TIMER
				lfclk_config();
				rtc_config();
				#endif

				#if 1
				bsp_board_leds_init();
				usrPaw3205ReSynchronousSPI();
			
				if((uint8_t)*( (uint32_t *)0x0003e400)==0xFF)
				{
					#if DEBUG_TRACE_ENABLE
					NRF_LOG_RAW_INFO("RESETREAS=%08x \r\n",NRF_POWER->RESETREAS);
					#endif
					if((NRF_POWER->RESETREAS)&0x10000)
					{
						m_cold_boot = false;
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_RAW_INFO("not cold boot \r\n");
						#endif
					}
					else
					{
						m_cold_boot = true;
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_RAW_INFO("cold boot \r\n");
						#endif
					}
				}
				else
				{
						m_cold_boot = true;
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_RAW_INFO("work mode changed \r\n");
						NRF_LOG_RAW_INFO("cold boot \r\n");
						#endif
						flash_page_erase((uint32_t *)0x0003e400);
				}
				
				NRF_POWER->DCDCEN = 0x01;
			
				#if 0//DEBUG_TRACE_ENABLE
				flash_addr = (uint32_t *)0x0003ec00;
				for(k=0;k<6;k++)
				{
					NRF_LOG_RAW_INFO("%02x ",(uint8_t)*(flash_addr+k));
				}
				NRF_LOG_RAW_INFO("\r\n");
				
				flash_addr = (uint32_t *)0x0003f000;
				for(k=0;k<6;k++)
				{
					NRF_LOG_RAW_INFO("%02x ",(uint8_t)*(flash_addr+k));
				}
				NRF_LOG_RAW_INFO("\r\n");
				
				flash_addr = (uint32_t *)0x0003e000;
				for(k=0;k<6;k++)
				{
					NRF_LOG_RAW_INFO("%02x ",(uint8_t)*(flash_addr+k));
				}
				NRF_LOG_RAW_INFO("\r\n");
				#endif
				flash_addr = (uint32_t *)0x0003ec00;
				for(k=0;k<6;k++)
				{
					if((uint8_t)*(flash_addr+k)!=0xFF) break;
				}
				if(k<6)
				{
					#if 1
					#if 1
					my_device_addr.addr[0] = (uint8_t)*(flash_addr);
					my_device_addr.addr[1] = (uint8_t)*(flash_addr+5);
					#else
					err_code = nrf_drv_rng_init(NULL);
					APP_ERROR_CHECK(err_code);
					random_vector_generate(my_device_addr.addr,6);
					nrf_drv_rng_uninit();
					#endif
					
					#if DEBUG_TRACE_ENABLE
					NRF_LOG_RAW_INFO("RF ID: ");
					NRF_LOG_RAW_INFO("%02x ",my_device_addr.addr[0]);
					NRF_LOG_RAW_INFO("%02x ",my_device_addr.addr[1]);
					NRF_LOG_RAW_INFO("\r\n");
					#endif
					
					#else
					bsp_board_led_on(BSP_BOARD_LED_0);
					#if DEBUG_TRACE_ENABLE
					flash_addr = (uint32_t *)0x0003ec00;
					for(k=0;k<6;k++)
					{
						NRF_LOG_RAW_INFO("%02x ",(uint8_t)*(flash_addr+k));
					}
					NRF_LOG_RAW_INFO("\r\n");
					#endif
					flash_addr = (uint32_t *)0x0003ec18;
					for(k=0;k<6;k++)
					{
						tx_payload.data[k] = (uint8_t)*(flash_addr+k);
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_RAW_INFO("%02x ",tx_payload.data[k]);
						#endif
					}
					#if DEBUG_TRACE_ENABLE
					NRF_LOG_RAW_INFO("\r\n");
					#endif

					tx_payload.length = 6;
					nrf_esb_set_rf_channel(8);
					
					while(1)
					{					
							m_TX_SUCCESS = false;
							tx_payload.noack = true;//false;//
							m_config_local.mode = NRF_ESB_MODE_PTX;
							err_code = nrf_esb_write_payload(&tx_payload);
							if(err_code==NRF_SUCCESS)
							{							
									while(!m_TX_SUCCESS);
						
									m_config_local.mode = NRF_ESB_MODE_PRX;
									m_RX_SUCCESS = false;
									nrf_esb_start_rx();
									for(i=0;i<10000;i++)
									{
										if(m_RX_SUCCESS) break;
										nrf_delay_us(1);  
									}
									if(i<10000)
									{
										nrf_delay_us(200);  
									}
									nrf_esb_stop_rx();
																
									if(m_RX_SUCCESS)
									{
										memcpy(BDaddrSourceEncryptData,encrypt(rx_payload.data,decryptPassword),6);	
										write_encrypt_information();
										#if 0
										//memcpy(BDaddrSource,encrypt(BDaddrSourceEncryptData,BDaddrPassword),6);	
										for(k=0;k<6;k++)
										{
											//BDaddrSource[k] -= 0x48 + k;
											NRF_LOG_RAW_INFO("%02x ",BDaddrSourceEncryptData[k]);
										}
										#endif
										#if 0//DEBUG_TRACE_ENABLE									
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[0]);
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[1]);
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[2]);
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[3]);
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[4]);
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[5]);
										NRF_LOG_RAW_INFO("\r\n");
										#endif
										//NRF_LOG_RAW_INFO("+");
										//nrf_delay_ms(500);  
									}
									else
									{
										//nrf_delay_ms(1000);  
									}
							}
							NRF_LOG_PROCESS();
					}
					#endif
				}
				else
				{
					bsp_board_led_on(BSP_BOARD_LED_0);
					#if DEBUG_TRACE_ENABLE
					flash_addr = (uint32_t *)0x0003ec00;
					for(k=0;k<6;k++)
					{
						NRF_LOG_RAW_INFO("%02x ",(uint8_t)*(flash_addr+k));
					}
					NRF_LOG_RAW_INFO("\r\n");
					#endif
					flash_addr = (uint32_t *)0x0003ec18;
					for(k=0;k<6;k++)
					{
						tx_payload.data[k] = (uint8_t)*(flash_addr+k);
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_RAW_INFO("%02x ",tx_payload.data[k]);
						#endif
					}
					#if DEBUG_TRACE_ENABLE
					NRF_LOG_RAW_INFO("\r\n");
					#endif

					tx_payload.length = 6;
					nrf_esb_set_rf_channel(8);
					
					modify_base_address_0_encrypt();				
					err_code = nrf_esb_update_prefix(0,0x3a);
					VERIFY_SUCCESS(err_code);
					
					while(1)
					{					
							m_TX_SUCCESS = false;
							tx_payload.noack = true;//false;//
							m_config_local.mode = NRF_ESB_MODE_PTX;
							err_code = nrf_esb_write_payload(&tx_payload);
							if(err_code==NRF_SUCCESS)
							{							
									while(!m_TX_SUCCESS);
						
									m_config_local.mode = NRF_ESB_MODE_PRX;
									m_RX_SUCCESS = false;
									nrf_esb_start_rx();
									for(i=0;i<10000;i++)
									{
										if(m_RX_SUCCESS) break;
										nrf_delay_us(1);  
									}
									if(i<10000)
									{
										nrf_delay_us(200);  
									}
									nrf_esb_stop_rx();
																
									if(m_RX_SUCCESS)
									{
										memcpy(BDaddrSourceEncryptData,encrypt(rx_payload.data,decryptPassword),6);	
										write_encrypt_information();
										#if 0
										//memcpy(BDaddrSource,encrypt(BDaddrSourceEncryptData,BDaddrPassword),6);	
										for(k=0;k<6;k++)
										{
											//BDaddrSource[k] -= 0x48 + k;
											NRF_LOG_RAW_INFO("%02x ",BDaddrSourceEncryptData[k]);
										}
										//write_encrypt_information();
										while(1)
										{
											NRF_LOG_PROCESS();
										}
										#endif
										#if 0//DEBUG_TRACE_ENABLE									
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[0]);
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[1]);
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[2]);
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[3]);
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[4]);
										NRF_LOG_RAW_INFO("%02x ",rx_payload.data[5]);
										NRF_LOG_RAW_INFO("\r\n");
										#endif
										//NRF_LOG_RAW_INFO("+");
										nrf_delay_ms(500);  
									}
									else
									{
										//nrf_delay_ms(1000);  
									}
							}
							NRF_LOG_PROCESS();
					}
				}
				
				#if 0
				while(1)
				{
					//bsp_board_leds_off();
            		bsp_board_led_on(BSP_BOARD_LED_0);
					nrf_delay_ms(500);
					bsp_board_led_off(BSP_BOARD_LED_0);
					nrf_delay_ms(500);
				}
				#endif
				#endif
			
				nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
				timer_cfg.bit_width = TIMER_BITMODE_BITMODE_32Bit;
				err_code = nrf_drv_timer_init(&TIMER_LED, &timer_cfg, timer_led_event_handler);
				APP_ERROR_CHECK(err_code);
			
				time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);

				nrf_drv_timer_extended_compare(
         &TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

			#if 1
				#if DEBUG_TRACE_ENABLE
				NRF_LOG_INFO("HID Mouse Start!\r\n");
				NRF_LOG_INFO("tx_payload.length=%d\r\n",tx_payload.length);
				NRF_LOG_INFO("tx_payload.pipe=%d\r\n",tx_payload.pipe);
				#endif
				#if 0
				tx_payload.data[0] = 0xE0;//0xA5; //DATATYPE_PAGE_START
				tx_payload.data[1] = 0x00;//0x00; //DATATYPE_MOUSE;
				tx_payload.data[2] = 0x00;//0x01; //rf_address_mouse1;
				tx_payload.data[3] = 0x00;//0x02; //rf_address_mouse2;
				tx_payload.data[4] = 0x00;//4;    //0dbm
				tx_payload.length = 5;
				nrf_esb_set_rf_channel(8);
				#else
				#if 1
				tx_payload.data[0] = 0xA5; //DATATYPE_PAGE_START
				tx_payload.data[1] = 0x00;//0x00; //DATATYPE_MOUSE;
				tx_payload.data[2] = my_device_addr.addr[0];//0x15; //rf_address_mouse1;
				tx_payload.data[3] = my_device_addr.addr[1];//0x9C;//0x02; //rf_address_mouse2;
				tx_payload.data[4] = 4;//4;    //0dbm
				tx_payload.length = 5;
				#else
				tx_payload.data[0] = 0x34; //DATATYPE_PAGE_START
				tx_payload.data[1] = 0x01;//0x00; //DATATYPE_MOUSE;
				tx_payload.data[2] = 0x58; //rf_address_mouse1;
				tx_payload.data[3] = 0xBD;//0x02; //rf_address_mouse2;
				tx_payload.data[4] = 0;//4;    //0dbm
				tx_payload.data[5] = 0x15;//4;    //0dbm
				tx_payload.data[6] = 0x9C;//4;    //0dbm
				
				tx_payload.length = 7;
				#endif
				nrf_esb_set_rf_channel(77);
				#endif
				#endif
				//m_config_local.mode = NRF_ESB_MODE_PRX;
				//m_RX_SUCCESS = false;
				//nrf_esb_start_rx();
				//err_code = app_timer_start(m_motion_timer_id, MOTION_INTERVAL, NULL);
				//APP_ERROR_CHECK(err_code);
				// Initialize hardware
				#if 1
				nrf_drv_gpiote_init();
				nrf_drv_gpiote_in_config_t gpiote_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
				gpiote_config.pull  = GPIO_PIN_CNF_PULL_Pullup;
				nrf_drv_gpiote_in_init(15,&gpiote_config,button_event_handler);
				nrf_drv_gpiote_in_init(17,&gpiote_config,button_event_handler);
				nrf_drv_gpiote_in_init(18,&gpiote_config,button_event_handler);
				nrf_drv_gpiote_in_init(19,&gpiote_config,button_event_handler);
				nrf_drv_gpiote_in_init(20,&gpiote_config,button_event_handler);
				nrf_drv_gpiote_in_init(21,&gpiote_config,button_event_handler);
				nrf_drv_gpiote_in_init(22,&gpiote_config,button_event_handler);
				nrf_drv_gpiote_in_init(23,&gpiote_config,button_event_handler);
				#if 0
				nrf_drv_gpiote_in_event_enable(18,false);
				nrf_drv_gpiote_in_event_enable(19,false);
				nrf_drv_gpiote_in_event_enable(20,false);
				nrf_drv_gpiote_in_event_enable(21,false);
				nrf_drv_gpiote_in_event_enable(22,false);
				nrf_drv_gpiote_in_event_enable(23,false);
				#endif
				#endif
				#if 0
				nrf_gpio_cfg_input(18, GPIO_PIN_CNF_PULL_Pullup);
				nrf_gpio_cfg_input(19, GPIO_PIN_CNF_PULL_Pullup);
				nrf_gpio_cfg_input(20, GPIO_PIN_CNF_PULL_Pullup);
				nrf_gpio_cfg_input(21, GPIO_PIN_CNF_PULL_Pullup);
				nrf_gpio_cfg_input(22, GPIO_PIN_CNF_PULL_Pullup);
				#endif
				
				#if 1
				err_code = nrf_drv_qdec_init(NULL, qdec_event_handler);
				APP_ERROR_CHECK(err_code);
				nrf_qdec_reportper_to_value(0);
				nrf_drv_qdec_enable(); 
				m_qdec_is_enable = true;
				#endif

				
				#if 1
				nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
				spi_config.ss_pin   = SPI_SS_PIN;
				spi_config.miso_pin = SPI_MISO_PIN;
				spi_config.mosi_pin = SPI_MOSI_PIN;
				spi_config.sck_pin  = SPI_SCK_PIN;
				APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
				m_spi_is_enable = true;
				turnOn();
				m_motion_sensor_is_enable = true;
				#endif


				#if 1
				m_TX_SUCCESS = true;
				nrf_drv_timer_enable(&TIMER_LED);
				tx_payload.noack = true;//false;//
				m_config_local.mode = NRF_ESB_MODE_PTX;
				page_loop_count = 0;
				my_page_rx_count = 0;
				page_common_count = 0;
				memset(BDaddrSource, 0, 6);
				#else
				m_config_local.mode = NRF_ESB_MODE_PRX;
				m_RX_SUCCESS = false;
				nrf_esb_start_rx();
				#endif
				#if CONSUMPTION_TEST_ENABLE
				pairing_loop_count = 20;
				#else
				pairing_loop_count = 20;//250;//1 minute
				#endif
				if(m_cold_boot)
				{
					while (true)
					{	
							
							#if 0
							while(1)
							{
								//bsp_board_leds_off();
								bsp_board_led_on(BSP_BOARD_LED_0);
								nrf_delay_ms(500);
								bsp_board_led_off(BSP_BOARD_LED_0);
								nrf_delay_ms(500);
							}
							#endif	
							#if 0
							//nrf_delay_us(10000);
							//getMotion(&tx_payload.data[0],&tx_payload.data[1],3);
						
							if(polling_motion)
							{
									polling_motion = false;
									if(getMotion(&motion_x,&motion_y,3))
									{
											tx_payload.data[3] = motion_x;
											tx_payload.data[2] = motion_y;
											active_event |= motion_event;
									}
							}
							
							if(active_event&&m_TX_SUCCESS)
							{
									if(active_event&key_event)
									{
										if(read_buf_index>=5)
										{
											read_buf_index = 0;
										}
										tx_payload.data[1] = button_buffer[read_buf_index++];
										button_cnt--;
										if(!button_cnt)
										{
											active_event &= ~key_event;
										}
									}
										
									if(active_event&motion_event)
									{
										active_event &= ~motion_event;
											
									}

									if(active_event&scroll_event)
									{
										active_event &= ~scroll_event;
										tx_payload.data[4] = (uint8_t)m_accread;
										m_accread = 0;
									}
									
									m_TX_SUCCESS = false;
									err_code = nrf_esb_write_payload(&tx_payload);
									//if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
									if(err_code!=NRF_SUCCESS)
									{
											NRF_LOG_INFO("err_code=%d\r\n",err_code);
											//nrf_delay_us(200000);
									}
									
							}
							#else
							#if 0
							if(++i<=500)
							{
								if(m_RX_SUCCESS)
								{
									//i = 0;
									m_RX_SUCCESS = 0;
									#if 0
									m_TX_SUCCESS = false;
									tx_payload.noack = true;//false;//
									m_config_local.mode = NRF_ESB_MODE_PTX;
									tx_payload.data[0] = 0x34; //DATATYPE_PAGE_START
									tx_payload.data[1] = 0x01;//0x00; //DATATYPE_MOUSE;
									tx_payload.data[2] = 0x58; //rf_address_mouse1;
									tx_payload.data[3] = 0xBD;//0x02; //rf_address_mouse2;
									tx_payload.data[4] = 0;//4;    //0dbm
									tx_payload.data[5] = 0x15;//4;    //0dbm
									tx_payload.data[6] = 0x9C;//4;    //0dbm
									tx_payload.length = 7;
									err_code = nrf_esb_write_payload(&tx_payload);
									if(err_code==NRF_SUCCESS)
									{
										while(!m_TX_SUCCESS);
									}

									
									m_config_local.mode = NRF_ESB_MODE_PRX;
									nrf_esb_start_rx();
									#endif
									NRF_LOG_RAW_INFO("%d     ",rf_channel);
									NRF_LOG_HEXDUMP_INFO(rx_payload.data,rx_payload.length);
									NRF_LOG_RAW_INFO("\r\n");
								}
							}
							else
							{
									driver_rf_channel_switch();	
									i = 0;
									bsp_board_led_invert(BSP_BOARD_LED_1);
							}
							#else
							if(page_loop_count >= 100)
							 {
								 #if DEBUG_TRACE_ENABLE
								 NRF_LOG_RAW_INFO("%d\r\n",page_common_count);	
								 //NRF_LOG_RAW_INFO("page_common_count=%d\r\n",page_common_count);	
								 //NRF_LOG_RAW_INFO("my_page_rx_count=%d\r\n",my_page_rx_count);	
								 #endif
								 if(page_common_count >= 10)
								 {
									 flag_rf_paged = 1;
									 #if DEBUG_TRACE_ENABLE
									 NRF_LOG_RAW_INFO("pairing success\r\n");	
									 #endif
									 bsp_board_led_off(BSP_BOARD_LED_0);
									 break;
								 }
								 else
								 {
										if(--pairing_loop_count)
										{
											page_loop_count = 0;
											my_page_rx_count = 0;
											page_common_count = 0;
											memset(BDaddrSource, 0, 6);
											bsp_board_led_invert(BSP_BOARD_LED_0);
										}
										else
										{
											#if DEBUG_TRACE_ENABLE
											NRF_LOG_RAW_INFO("pairing fail\r\n");	
											#endif
											bsp_board_led_off(BSP_BOARD_LED_0);
											break;
										}
								 }
							 }
							 
							 page_loop_count++;
							//if(1)
							if(m_TX_SUCCESS)
							{
									//nrf_delay_ms(500);						
									m_TX_SUCCESS = false;
									tx_payload.noack = true;//false;//
									m_config_local.mode = NRF_ESB_MODE_PTX;
									err_code = nrf_esb_write_payload(&tx_payload);
									
									//if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
									if(err_code==NRF_SUCCESS)
									{
											// Toggle one of the LEDs.
											//tx_payload.data[1]++;
											//NRF_LOG_RAW_INFO("+");
										#if 0
										#if 0
											for(i=0;i<500;i++)
											{
												if(m_TX_SUCCESS) break;
											}
											#else
											nrf_delay_us(2000);  
											#endif
											//if(i<500)
											if(m_TX_SUCCESS)
											{
												NRF_LOG_RAW_INFO("@");
											}
											else
											{
												//NRF_LOG_RAW_INFO("+");
											}
											#else
											while(!m_TX_SUCCESS);
											m_TX_attempt++;
											#endif
											#if 1									
											m_config_local.mode = NRF_ESB_MODE_PRX;
											m_RX_SUCCESS = false;
											nrf_esb_start_rx();
											for(i=0;i<600;i++)
											{
												if(m_RX_SUCCESS) break;
												nrf_delay_us(1);  
											}
											//nrf_delay_us(2000);  
											if(i<600)
											{
												nrf_delay_us(200);  
											}
											nrf_esb_stop_rx();
											
											//NRF_LOG_RAW_INFO("+");									
											if(m_RX_SUCCESS)
											{
													hop_frequency_mode = false;
													#if 0
													if(tx_payload.data[1] &0x08)
													{
														tx_payload.data[1]  &= ~0x08;
														driver_rf_channel_switch();
														NRF_LOG_RAW_INFO("n");
													}
												
													if(m_TX_attempt==3)
													{
														tx_payload.data[1]  |= 0x08;
														NRF_LOG_RAW_INFO("s");
													}
													#endif
													
													m_TX_attempt = 0;
													if((rx_payload.data[0]==0x34)&&(rx_payload.data[5] == my_device_addr.addr[0]) && (rx_payload.data[6] == my_device_addr.addr[1]))
													 {
														 #if 1
														if(rx_payload.data[4])
														 {
															 BDaddrSource[0] = rx_payload.data[1];
															 BDaddrSource[1] = rx_payload.data[2];
															 my_page_rx_count = 1;
															 flag_rf_paged = 1;
															 #if DEBUG_TRACE_ENABLE
															 NRF_LOG_RAW_INFO("reconnect\r\n");
															 #endif
															 break;
														 }
														 else
														 #endif
														 {
															 for(i=0; i<2; i++)
															 {
																 if((BDaddrSource[i*3] == rx_payload.data[1]) && (BDaddrSource[i*3+1] == rx_payload.data[2]))
																 {
																	 if(BDaddrSource[i*3+2] < 250)
																	 {
																		 BDaddrSource[i*3+2]++;
																		 //NRF_LOG_RAW_INFO("+");		
																	 }
																	 break;
																 }
															 }
															 if(i >= 2)
															 {
																 if(my_page_rx_count < 2)
																 {
																	 BDaddrSource[my_page_rx_count*3] = rx_payload.data[1];
																	 BDaddrSource[my_page_rx_count*3+1] = rx_payload.data[2];
																	 BDaddrSource[my_page_rx_count*3+2] = 1;
																	 my_page_rx_count++;
																 }
															 }
														 }
														 page_common_count++;
													 }
											}
											else
											{												
													//bsp_board_led_invert(BSP_BOARD_LED_1);
													if(!hop_frequency_mode)
													{
														if(m_TX_attempt>=2)
														{
															//tx_payload.data[1]  &= ~0x08;
															hop_frequency_mode = true;
															m_TX_attempt = 0;
															driver_rf_channel_switch();	
															#if DEBUG_TRACE_ENABLE
															NRF_LOG_RAW_INFO("@");
															#endif
														}	
													}	
													else
													{
															//tx_payload.data[1]  &= ~0x08;
															m_TX_attempt = 0;
															driver_rf_channel_switch();	
															//NRF_LOG_RAW_INFO("h");
													}
											}
											#endif
									}
									else
									{
											#if DEBUG_TRACE_ENABLE
											NRF_LOG_INFO("err_code=%d\r\n",err_code);
											#endif
									}
							}
							#endif
							#endif
							// Wait for esb completed and all buttons released before going to system off.
							if (NRF_LOG_PROCESS() == false)
							{
										//NRF_power_manage();
							}			
					}
				}
				
				//NRF_LOG_RAW_INFO("m_nrf_esb_mainstate=%d\r\n",m_nrf_esb_mainstate);
				//NRF_LOG_RAW_INFO("my_page_rx_count=%d\r\n",my_page_rx_count);	

				sleep_time_cnt = 0;
				if(flag_rf_paged)
				{
					if(my_page_rx_count == 1)  //just use the only one
					{
						BDaddrSourceEncryptData[0] = BDaddrSource[0];
						BDaddrSourceEncryptData[1] = BDaddrSource[1];
						//addr_prefix[0] = BDaddrSource[0];
						//base_addr_0[0] = BDaddrSource[1];	
					}
					else if(my_page_rx_count) //search for the max count one
					{
						page_common_count = 0;
						my_temp = BDaddrSource[2];
						for(k=1; k<my_page_rx_count; k++)
						{
							if(BDaddrSource[k*3+2] >= my_temp)
							{
								my_temp = BDaddrSource[k*3+2];
								page_common_count = k;
							}
						}
						BDaddrSourceEncryptData[0] = BDaddrSource[page_common_count*3];
						BDaddrSourceEncryptData[1] = BDaddrSource[page_common_count*3+1];
						//addr_prefix[0] = BDaddrSource[page_common_count*3];
						//base_addr_0[0] = BDaddrSource[page_common_count*3+1];
					}
	
					//base_addr_0[1] = 0xBC;
					//base_addr_0[2] = 0xBC;
					//base_addr_0[3] = 0xBC;
					
					modify_base_address_0(BDaddrSourceEncryptData[1]);
					//err_code = nrf_esb_set_base_address_0(base_addr_0);
					//VERIFY_SUCCESS(err_code);
					
					err_code = nrf_esb_update_prefix(0,BDaddrSourceEncryptData[0]);
					VERIFY_SUCCESS(err_code);
					//err_code = nrf_esb_set_prefixes(addr_prefix, 8);
					//VERIFY_SUCCESS(err_code);
					
					#if 1			
					flash_addr = (uint32_t *)0x0003e000;				
					if(((uint8_t)*(flash_addr)!=BDaddrSourceEncryptData[0])||((uint8_t)*(flash_addr+1)!=BDaddrSourceEncryptData[1])||((uint8_t)*(flash_addr+2)!=0xBC))
					{	
						flash_page_erase(flash_addr);
						flash_word_write((flash_addr), BDaddrSourceEncryptData[0]);
						flash_word_write((flash_addr+1), BDaddrSourceEncryptData[1]);
						flash_word_write((flash_addr+2), 0xBC);
						
						#if DEBUG_TRACE_ENABLE
						for(n=0;n<3;n++)
						{
							NRF_LOG_RAW_INFO("%02x ",(uint8_t)*(flash_addr+n));
						}
						NRF_LOG_RAW_INFO("\r\n");
						NRF_LOG_RAW_INFO("store remote ID\r\n");
						#endif
					}
					else
					{
						#if DEBUG_TRACE_ENABLE
						NRF_LOG_RAW_INFO("remote ID already stored\r\n");
						#endif
					}			
					#endif
					
					#if DEBUG_TRACE_ENABLE
					NRF_LOG_HEXDUMP_INFO(BDaddrSourceEncryptData,2);
					//NRF_LOG_HEXDUMP_INFO(&BDaddrSourceEncryptData[1],1);
					#endif
				}
				else
				{
							flash_addr = (uint32_t *)0x0003e000;		
							#if CONSUMPTION_TEST_ENABLE
							if(1)
							#else
							if((uint8_t)*(flash_addr+2)!=0xBC)
							#endif
							{
								nrf_drv_gpiote_in_event_disable(15);
								nrf_drv_gpiote_in_event_disable(17);
								nrf_drv_gpiote_in_event_disable(18);
								nrf_drv_gpiote_in_event_disable(19);
								nrf_drv_gpiote_in_event_disable(20);
								nrf_drv_gpiote_in_event_disable(21);
								nrf_drv_gpiote_in_event_disable(22);
								nrf_drv_gpiote_in_event_disable(23);						
								nrf_drv_timer_disable(&TIMER_LED);
								
								if(!m_spi_is_enable)
								{
									nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
									spi_config.ss_pin   = SPI_SS_PIN;
									spi_config.miso_pin = SPI_MISO_PIN;
									spi_config.mosi_pin = SPI_MOSI_PIN;
									spi_config.sck_pin  = SPI_SCK_PIN;
									APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
									m_spi_is_enable = true;
									#if DEBUG_TRACE_ENABLE
									NRF_LOG_INFO("enable spi\r\n");
									#endif
								}					
								
								if(m_motion_sensor_is_enable)
								{
									#if SensorJustSleep
									enterOrExitSleepmode(0xBC);
									#else
									enterOrExitSleepmode(0x13);
									#endif	
									m_motion_sensor_is_enable = false;
									#if DEBUG_TRACE_ENABLE
									NRF_LOG_INFO("sensor enter sleep\r\n");
									#endif
								}
								
								
								if(m_spi_is_enable)
								{
									nrf_drv_spi_uninit(&spi);
									m_spi_is_enable = false;
									#if 0
									nrf_gpio_cfg_output(SPI_MISO_PIN);
									nrf_gpio_cfg_output(SPI_MOSI_PIN);
									nrf_gpio_cfg_output(SPI_SCK_PIN);
									nrf_gpio_pin_set(SPI_MISO_PIN);
									nrf_gpio_pin_set(SPI_MOSI_PIN);
									nrf_gpio_pin_set(SPI_SCK_PIN);
									#endif
									#if DEBUG_TRACE_ENABLE
									NRF_LOG_INFO("disable spi\r\n");
									#endif
								}
								
								if(m_qdec_is_enable)
								{
									nrf_drv_qdec_uninit();
									m_qdec_is_enable = false;
									nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pullup);
									nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pullup);
								}
								
								if(m_adc_is_enable)
								{
									m_adc_is_enable = false;
									nrf_drv_adc_uninit();
								}
								#if DEBUG_TRACE_ENABLE
								NRF_LOG_INFO("no pairing before\r\n");
								#endif
								while(1)
								{
									if (NRF_LOG_PROCESS() == false)
									{
												NRF_power_manage();
												//system_off();
									}	
								}
							}
							else
							{
								#if DEBUG_TRACE_ENABLE
								NRF_LOG_INFO("pairing before\r\n");
								NRF_LOG_INFO("set before pairing remote id\r\n");
								#endif
								modify_base_address_0((uint8_t)*(flash_addr+1));
								
								err_code = nrf_esb_update_prefix(0,(uint8_t)*(flash_addr));
								VERIFY_SUCCESS(err_code);
							}
				}
				
				tx_payload.data[0] = 0x00;
				//tx_payload.data[0] |= rf_connect;
				tx_payload.data[0] |= SYSTEM_SYNCHRONIZE;
				tx_payload.data[1] = 0;//
				tx_payload.data[2] = 0;//
				tx_payload.data[3] = 0x01;//
				tx_payload.data[4] = 0;//
				tx_payload.data[5] = 0;//
				tx_payload.data[6] = 0;//
				tx_payload.data[7] = my_device_addr.addr[0];//0x15;//rf_address_mouse1;
				tx_payload.data[8] = my_device_addr.addr[1];//0x9C;//rf_address_mouse2;
				tx_payload.length = 9;
				
				m_TX_SUCCESS = true;
				tx_payload.noack = true;//false;//
				m_config_local.mode = NRF_ESB_MODE_PTX;
				
				#if 0
					while(1)
					{
						NRF_LOG_PROCESS();
					}
					#endif	
				
				while(1)
				{
					#if 0
					m_TX_FAIL = false;
					m_TX_SUCCESS = false;
					tx_payload.data[3] = 0x01;//
					err_code = nrf_esb_write_payload(&tx_payload);
					//if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
					if(err_code==NRF_SUCCESS)
					{
							// Toggle one of the LEDs.
							//tx_payload.data[1]++;
							//NRF_LOG_RAW_INFO("+");
							#if 0
							while(!m_TX_SUCCESS)
							{
								if(m_TX_FAIL)
								break;
							}
							nrf_delay_us(8000);  
							NRF_LOG_RAW_INFO("@");
							#else
							nrf_delay_us(2000);  
							//if(i<500)
							if(m_TX_SUCCESS)
							{
								NRF_LOG_RAW_INFO("@");
							}
							#endif						
							//NRF_LOG_RAW_INFO("%d\r\n",m_TX_attemp);
							//NRF_LOG_RAW_INFO("@");
					}
					#else
					if(polling_motion)
					{
						#if ENABLE_RTC_TIMER
						if(sleep_time_cnt>=375)
						#else
						if(sleep_time_cnt>=800)//2500)//1250)75000
						#endif
						{
							nrf_drv_gpiote_in_event_enable(15,false);
							nrf_drv_gpiote_in_event_enable(18,false);
							nrf_drv_gpiote_in_event_enable(19,false);
							nrf_drv_gpiote_in_event_enable(20,false);
							nrf_drv_gpiote_in_event_enable(21,false);
							nrf_drv_gpiote_in_event_enable(22,false);
							nrf_drv_gpiote_in_event_enable(23,false);						
							nrf_drv_timer_disable(&TIMER_LED);
							sleep_time_cnt = 0;
							#if ENABLE_RTC_TIMER
							if(!m_rtc_timer_is_enable)
							{
								nrf_drv_rtc_enable(&rtc);
								nrf_drv_rtc_counter_clear(&rtc);
								m_rtc_timer_is_enable = true;
							}
							#endif
							
							if(!m_spi_is_enable)
							{
								nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
								spi_config.ss_pin   = SPI_SS_PIN;
								spi_config.miso_pin = SPI_MISO_PIN;
								spi_config.mosi_pin = SPI_MOSI_PIN;
								spi_config.sck_pin  = SPI_SCK_PIN;
								APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
								m_spi_is_enable = true;
								#if DEBUG_TRACE_ENABLE
								NRF_LOG_INFO("enable spi\r\n");
								#endif
							}
							
							#if 1
							if(m_motion_sensor_is_enable)
							{
								#if SensorJustSleep
								enterOrExitSleepmode(0xBC);
								#else
								enterOrExitSleepmode(0x13);
								#endif	
								m_motion_sensor_is_enable = false;
								#if DEBUG_TRACE_ENABLE
								NRF_LOG_INFO("sensor enter sleep\r\n");
								#endif
							}
							#else
							switch(peer_id_cnt)
							{
								case 0:
									enterOrExitjustSleepmode(0x13);
									peer_id_cnt = 2;
									break;
								
								case 1:
									enterOrExitjustSleepmode(0xB1);
									enterOrExitPowerDownMode(0x13);
									peer_id_cnt = 2;
									break;
								
								default: break;
							}
							#endif
							
							
							if(m_spi_is_enable)
							{
								nrf_drv_spi_uninit(&spi);
								m_spi_is_enable = false;
								//nrf_gpio_cfg_output(SPI_MISO_PIN);
								//nrf_gpio_cfg_output(SPI_MOSI_PIN);
								//nrf_gpio_cfg_output(SPI_SCK_PIN);
								//nrf_gpio_pin_set(SPI_MISO_PIN);
								//nrf_gpio_pin_set(SPI_MOSI_PIN);
								//nrf_gpio_pin_set(SPI_SCK_PIN);
								#if DEBUG_TRACE_ENABLE
								NRF_LOG_INFO("disable spi\r\n");
								#endif
							}
							
							#if !ENABLE_RTC_TIMER
							if(m_qdec_is_enable)
							{
								nrf_drv_qdec_uninit();
								m_qdec_is_enable = false;
								nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pullup);
								nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pullup);
							}
							#endif
							
							if(m_adc_is_enable)
							{
								m_adc_is_enable = false;
								nrf_drv_adc_uninit();
							}		
							system_off();
							//NRF_LOG_RAW_INFO("sleep\r\n");
						}
						else if(sleep_time_cnt>=750)						
						{
							if(m_qdec_is_enable)
							{
								nrf_drv_qdec_uninit();
								m_qdec_is_enable = false;
								nrf_gpio_cfg_input(QDEC_CONFIG_PIO_A, GPIO_PIN_CNF_PULL_Pullup);
								nrf_gpio_cfg_input(QDEC_CONFIG_PIO_B, GPIO_PIN_CNF_PULL_Pullup);
							}
						}
						#if 0
						else if(sleep_time_cnt>=375)
						{
							if(peer_id_cnt==0)
							{
								if(!m_spi_is_enable)
								{
									nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
									spi_config.ss_pin   = SPI_SS_PIN;
									spi_config.miso_pin = SPI_MISO_PIN;
									spi_config.mosi_pin = SPI_MOSI_PIN;
									spi_config.sck_pin  = SPI_SCK_PIN;
									APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
									m_spi_is_enable = true;
									#if DEBUG_TRACE_ENABLE
									NRF_LOG_INFO("enable spi\r\n");
									#endif
								}
													
								enterOrExitjustSleepmode(0xBC);
								peer_id_cnt = 1;					
							
								if(m_spi_is_enable)
								{
									nrf_drv_spi_uninit(&spi);
									m_spi_is_enable = false;
									//nrf_gpio_cfg_output(SPI_MISO_PIN);
									//nrf_gpio_cfg_output(SPI_MOSI_PIN);
									//nrf_gpio_cfg_output(SPI_SCK_PIN);
									//nrf_gpio_pin_set(SPI_MISO_PIN);
									//nrf_gpio_pin_set(SPI_MOSI_PIN);
									//nrf_gpio_pin_set(SPI_SCK_PIN);
									#if DEBUG_TRACE_ENABLE
									NRF_LOG_INFO("disable spi\r\n");
									#endif
								}
							}
						}
						#endif
						else
						{
							if(!m_spi_is_enable)
							{
								nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
								spi_config.ss_pin   = SPI_SS_PIN;
								spi_config.miso_pin = SPI_MISO_PIN;
								spi_config.mosi_pin = SPI_MOSI_PIN;
								spi_config.sck_pin  = SPI_SCK_PIN;
								APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
								m_spi_is_enable = true;
								#if DEBUG_TRACE_ENABLE
								NRF_LOG_INFO("enable spi\r\n");
								#endif
							}
							
							#if 1
							if(!m_motion_sensor_is_enable)
							{
								#if SensorJustSleep
								enterOrExitSleepmode(0xB1);
								#else
								enterOrExitSleepmode(0x1B);
								#endif
								m_motion_sensor_is_enable = true;
								#if DEBUG_TRACE_ENABLE
								NRF_LOG_INFO("enable sensor\r\n");
								#endif
							}
							#else
							switch(peer_id_cnt)
							{
								case 1:
									enterOrExitjustSleepmode(0xB1);
									peer_id_cnt = 0;
									break;
								
								case 2:
									enterOrExitPowerDownMode(0x1B);
									peer_id_cnt = 0;
									break;
								
								default: break;
							}
							#endif
							
							if(!m_qdec_is_enable)
							{
								err_code = nrf_drv_qdec_init(NULL, qdec_event_handler);
								APP_ERROR_CHECK(err_code);
								nrf_qdec_reportper_to_value(0);
								nrf_drv_qdec_enable(); 
								m_qdec_is_enable = true;
							}
							
							if(change_cpi_flag)
							{
								change_cpi_flag = false;
								if(ChipVersion==0x02)						
								{							
									if(++cpiCnt>=5)							
									{								
										cpiCnt = 1;							
									}						
								}						
								else						
								{							
									if(++cpiCnt>=4)							
									{								
										cpiCnt = 1;							
									}						
								}	
								cpiCntStore = cpiCnt<<1;
								if(!m_spi_is_enable)
								{
									nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
									spi_config.ss_pin   = SPI_SS_PIN;
									spi_config.miso_pin = SPI_MISO_PIN;
									spi_config.mosi_pin = SPI_MOSI_PIN;
									spi_config.sck_pin  = SPI_SCK_PIN;
									APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
									m_spi_is_enable = true;
								}
								usrPaw3205setCPI(cpiCnt);
							}
						}
						
						polling_motion = false;
						if(getMotion(&motion_x,&motion_y,3))
						{
							tx_payload.data[3] = (uint8_t)(motion_y);
							tx_payload.data[4] = (uint8_t)(motion_y >> 8);
							tx_payload.data[5] = (uint8_t)(motion_x);
							tx_payload.data[6] = (uint8_t)(motion_x >> 8);
							active_event |= motion_event;
						}
					}

					if(active_event&&m_TX_SUCCESS)
					{
						sleep_time_cnt = 0;
						if(active_event&key_event)
						{
							#if 0
							if(read_buf_index>=5)
							{
								read_buf_index = 0;
							}
							tx_payload.data[1] = button_buffer[read_buf_index++];
							button_cnt--;
							if(!button_cnt)
							{
								active_event &= ~key_event;
							}
							#else
							//tx_payload.data[1] = button_buffer[read_buf_index++];
							active_event &= ~key_event;
							#endif
						}

						if(active_event&motion_event)
						{
							active_event &= ~motion_event;
						}

						if(active_event&scroll_event)
						{
							active_event &= ~scroll_event;
							tx_payload.data[2] += (uint8_t)m_accread;
							m_accread = 0;
							#if ENABLE_RTC_TIMER
							if(m_rtc_timer_is_enable)
							{
								m_time_keep_connection = 0;
								nrf_drv_rtc_counter_clear(&rtc);
							}
							#endif
						}

						#if 0
						m_TX_SUCCESS = false;
						err_code = nrf_esb_write_payload(&tx_payload);
						//if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
						if(err_code==NRF_SUCCESS)
						{
							while(!m_TX_SUCCESS);
							NRF_LOG_RAW_INFO("@");
						}
						else
						{
							NRF_LOG_INFO("err_code=%d\r\n",err_code);
							//nrf_delay_us(200000);
						}
						#else
						while(1)
						{					
								m_TX_SUCCESS = false;
								tx_payload.noack = true;//false;//
								m_config_local.mode = NRF_ESB_MODE_PTX;
								err_code = nrf_esb_write_payload(&tx_payload);
								//if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
								if(err_code==NRF_SUCCESS)
								{							
										while(!m_TX_SUCCESS);
										m_TX_attempt++;
							
										m_config_local.mode = NRF_ESB_MODE_PRX;
										m_RX_SUCCESS = false;
										nrf_esb_start_rx();
										for(i=0;i<600;i++)
										{
											if(m_RX_SUCCESS) break;
											nrf_delay_us(1);  
										}
										//nrf_delay_us(2000);  
										if(i<600)
										{
											nrf_delay_us(200);  
										}
										nrf_esb_stop_rx();
										
										//NRF_LOG_RAW_INFO("+");									
										if(m_RX_SUCCESS)
										{
												hop_frequency_mode = false;
												hop_frequency_timeout = 0;
												#if 0
												if(tx_payload.data[0] &0x08)
												{
													tx_payload.data[0]  &= ~0x08;
													driver_rf_channel_switch();
													NRF_LOG_RAW_INFO("n");
												}
											
												if(m_TX_attempt==3)
												{
													tx_payload.data[0]  |= 0x08;
													NRF_LOG_RAW_INFO("s");
												}
												#endif
												m_TX_attempt = 0;	
												tx_payload.data[2] = 0;
												tx_payload.data[3] = 0;
												tx_payload.data[4] = 0;
												tx_payload.data[5] = 0;
												tx_payload.data[6] = 0;
												break;
										}
										else
										{
												if(!hop_frequency_mode)
												{
													if(m_TX_attempt>=2)
													{
														//tx_payload.data[0]  &= ~0x08;
														hop_frequency_mode = true;
														m_TX_attempt = 0;
														driver_rf_channel_switch();	
														//NRF_LOG_RAW_INFO("@");
													}	
												}	
												else
												{
														hop_frequency_timeout++;
														if(hop_frequency_timeout>=32)
														{
															m_TX_attempt = 0;	
															tx_payload.data[1] = 0;
															tx_payload.data[2] = 0;
															tx_payload.data[3] = 0;
															tx_payload.data[4] = 0;
															tx_payload.data[5] = 0;
															tx_payload.data[6] = 0;
															hop_frequency_timeout = 0;
															#if DEBUG_TRACE_ENABLE
															NRF_LOG_RAW_INFO("hop timeout\r\n");
															#endif
															break;
														}
														//tx_payload.data[0]  &= ~0x08;
														m_TX_attempt = 0;
														driver_rf_channel_switch();	
														//NRF_LOG_RAW_INFO("h");
												}
										}
								}
						}
						#endif
						if (NRF_LOG_PROCESS() == false)
						{
									NRF_power_manage();
									//system_off();
						}	
					}
					#endif
					
					if (NRF_LOG_PROCESS() == false)
					{
								NRF_power_manage();
					}	
				}					
				#endif
		}
}


