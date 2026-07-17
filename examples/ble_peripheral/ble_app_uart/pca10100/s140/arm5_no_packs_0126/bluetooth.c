#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "bluetooth.h"
#include "spo2.h"
#include "ecg.h"
#include "ads1292.h"
#include "../arm5_no_packs/attention.h"
#include "semphr.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "QMI8658C.h"
#include "info_collect.h"
#include "../../../debug_print.h"


int flag_run = 0;
extern SemaphoreHandle_t enter_low_power_sem;
extern SemaphoreHandle_t wake_up_sem;
extern SemaphoreHandle_t max_wake_up_sem;


extern uint32_t spo2 ;
extern uint32_t ecg_data;

#define BMP_CACHE_SIZE  2500
extern int32_t bpm_cache[BMP_CACHE_SIZE]; 

static volatile int nrf_connect_stable_flag = 0;

/* BLE callbacks run in interrupt context in this build. Keep their error path
 * non-blocking and report it later from ble_send_thread instead of entering
 * APP_ERROR_CHECK's debugger stop/reset path. */
#define BLE_DIAG_STAGE_QWR_RUNTIME       1U
#define BLE_DIAG_STAGE_CONN_LED          2U
#define BLE_DIAG_STAGE_QWR_ASSIGN        3U
#define BLE_DIAG_STAGE_PHY_UPDATE        4U
#define BLE_DIAG_STAGE_SEC_REPLY         5U
#define BLE_DIAG_STAGE_SYS_ATTR          6U
#define BLE_DIAG_STAGE_GATTC_TIMEOUT     7U
#define BLE_DIAG_STAGE_GATTS_TIMEOUT     8U
#define BLE_DIAG_STAGE_CONN_PARAM        9U
#define BLE_DIAG_STAGE_CONN_DISCONNECT  10U

static volatile uint32_t m_ble_diag_error_code = NRF_SUCCESS;
static volatile uint32_t m_ble_diag_error_stage = 0U;
static volatile uint32_t m_ble_diag_error_sequence = 0U;
static volatile bool m_ble_attention_tx_seen = false;
volatile uint8_t g_last_ble_event = 0U;



extern int32_t  save_ecg_data[CYC_ARR_LEN];	
extern uint32_t  write_ecg_pos;  //当前存储数据的位置
extern uint32_t  read_ecg_pos; //当前分析数据的位置





#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "test_nrf_kaifaban"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                    /**< UUID type assigned to the Nordic UART Service base UUID. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
//#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
//#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(6000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static volatile bool m_nus_notifications_enabled = false;
ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};


static void ble_record_runtime_error(uint32_t stage, uint32_t error_code)
{
    if (error_code != NRF_SUCCESS)
    {
        m_ble_diag_error_stage = stage;
        m_ble_diag_error_code = error_code;
        m_ble_diag_error_sequence++;
    }
}


static void ble_report_runtime_error(void)
{
    static uint32_t reported_sequence = 0U;
    uint32_t sequence = m_ble_diag_error_sequence;

    if (sequence != reported_sequence)
    {
        uint32_t stage = m_ble_diag_error_stage;
        uint32_t error_code = m_ble_diag_error_code;

        reported_sequence = sequence;
        printf("BLE ERR stage=%lu code=0x%08lX\r\n",
               (unsigned long)stage,
               (unsigned long)error_code);
    }
}


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
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

/**@brief Function for initializing the timer module.
 */
void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
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


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void nrf_qwr_error_handler(uint32_t nrf_error)
{
    ble_record_runtime_error(BLE_DIAG_STAGE_QWR_RUNTIME, nrf_error);
}


/**@brief Function for handling Nordic UART Service events.
 *
 * @details BLE events are dispatched from interrupt context in this project. Do not
 *          call printf(), wait for UART FIFO space, or echo a notification from this
 *          callback. Those operations can block the SoftDevice event path while the
 *          sensor tasks are also logging. RX control handling can be added later by
 *          copying bytes into a FreeRTOS queue and consuming them from a task.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_evt_t * p_evt)
{
    /* SoftDevice owns the POWER peripheral while enabled. Record in RAM here;
     * the fault handler copies this byte to GPREGRET2 only during reset. */
    g_last_ble_event = (uint8_t)(0xC0U | ((uint8_t)p_evt->type & 0x0FU));

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        /* Reserved for queued, task-context control commands. */
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
        m_nus_notifications_enabled = true;
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
    {
        m_nus_notifications_enabled = false;
    }
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            ble_record_runtime_error(BLE_DIAG_STAGE_CONN_DISCONNECT, err_code);
        }
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
void conn_params_error_handler(uint32_t nrf_error)
{
    ble_record_runtime_error(BLE_DIAG_STAGE_CONN_PARAM, nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void)
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


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
void sleep_mode_enter(void)
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


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
						advertising_start();
            break;
        default:
            break;
    }
}

void ble_send_data(uint8_t *data, uint16_t length)
{
#if BLE_STREAM_ATTENTION_ONLY
    if ((data == NULL) ||
        (length != 12U) ||
        (data[0] != 0xA5U) ||
        (data[1] != 0x5AU) ||
        (data[2] != 0x01U) ||
        (data[3] != 0x01U))
    {
        return;
    }
#endif

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
        printf("No active connection\r\n");
        return;
    }

    if (!m_nus_notifications_enabled) {
        return;
    }

		if(!nrf_sdh_is_enabled() ){
			 printf("NUS service not readyy \r\n");
       vTaskDelay(pdMS_TO_TICKS(100));
       return ;
		}
    if (length == 0 || data == NULL) {
        printf("Invalid data parameters\r\n");
        return;
    }

    ret_code_t err_code;

//    do {   //do
        err_code = ble_nus_data_send(&m_nus, data, &length, m_conn_handle);
			
				switch (err_code) {
            case NRF_SUCCESS:
                if (!m_ble_attention_tx_seen)
                {
                    m_ble_attention_tx_seen = true;
                    printf("BLE attention TX OK\r\n");
                }
                return;
                
            case NRF_ERROR_INVALID_STATE:
                //printf("NUS service not ready \r\n");
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
                
            case NRF_ERROR_INVALID_PARAM:
                printf("Invalid parameters. Check NUS initialization \r\n" );
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
                
            case NRF_ERROR_RESOURCES:
                printf("BLE stack busy \r\n");
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            
						case NRF_ERROR_NO_MEM:
								printf("NO MEM     ");
								printf("FreeRTOS heap free: %lu bytes          \r\n", xPortGetFreeHeapSize());

                break;
						
            default:
								printf("Send failed: 0x%X \r\n", err_code);    //401:BLE_ERROR_GATTS_SYS_ATTR_MISSING
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
				}
			
//    } while (err_code != NRF_SUCCESS);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */





extern int spo2_wake_up;
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    /* BLE events are dispatched from interrupt context in this project.
     * Keep diagnostics in RAM: direct GPREGRET access is forbidden while the
     * SoftDevice is enabled and itself causes an application memory fault. */
    g_last_ble_event = (uint8_t)(p_ble_evt->header.evt_id & 0xFFU);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            ble_record_runtime_error(BLE_DIAG_STAGE_CONN_LED, err_code);
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            ble_record_runtime_error(BLE_DIAG_STAGE_QWR_ASSIGN, err_code);
            nrf_connect_stable_flag = 1;
				

				
//						if (xSemaphoreGive(wake_up_sem) == pdTRUE) {
//								printf("wake_up_sem released\r\n");
//						} else {
//								printf("wake_up_sem give failed\r\n");
//						}
//						flag_run = 1;
//						if (xSemaphoreGive(max_wake_up_sem) == pdTRUE) {
//								printf("max_wake_up_sem released\r\n");
//						} else {
//								printf("max_wake_up_sem give failed\r\n");
//						}
						

				

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            nrf_connect_stable_flag = 0;
            m_nus_notifications_enabled = false;
            m_ble_attention_tx_seen = false;
				
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            //printf("PHY update request.\r\n");
            ble_gap_phys_t const phys =
            {
                /* The focus/sensor protocol is low bandwidth. Keep the link on
                 * 1M PHY while diagnosing connection-stage controller faults. */
                .rx_phys = BLE_GAP_PHY_1MBPS,
                .tx_phys = BLE_GAP_PHY_1MBPS,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            ble_record_runtime_error(BLE_DIAG_STAGE_PHY_UPDATE, err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
				  
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            ble_record_runtime_error(BLE_DIAG_STAGE_SEC_REPLY, err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
				   
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            ble_record_runtime_error(BLE_DIAG_STAGE_SYS_ATTR, err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
					
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                ble_record_runtime_error(BLE_DIAG_STAGE_GATTC_TIMEOUT, err_code);
            }
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
						
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                ble_record_runtime_error(BLE_DIAG_STAGE_GATTS_TIMEOUT, err_code);
            }
            break;
	
				case BLE_GAP_EVT_SCAN_REQ_REPORT:
						break;
				case BLE_GAP_EVT_CONN_PARAM_UPDATE:
						break;
				case BLE_GAP_EVT_PHY_UPDATE:
						break;
        default:
            // No implementation needed.
						// // // printf("default\r\n");
            break;
    }
}




static void conn_evt_len_ext_set(bool enable)
{
    ret_code_t err_code;
    ble_opt_t opt;
 
    memset(&opt, 0x00, sizeof(opt));
    opt.common_opt.conn_evt_ext.enable = enable ? 1 : 0;
 
    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
	
		uint32_t ram_start = 0; 
		

	
    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
	
	
//		ble_cfg_t ble_cfg; 
//		memset(&ble_cfg, 0, sizeof(ble_cfg));
//		ble_cfg.conn_cfg.conn_cfg_tag = APP_BLE_CONN_CFG_TAG;
//		// 增大GATT客户端（写操作）和服务器（通知操作）的发送队列大小
//		ble_cfg.conn_cfg.params.gattc_conn_cfg.write_cmd_tx_queue_size = 1; // 默认可能是1
//		ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 1;       // 默认可能是1	 
//		// 应用配置
//		err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTC, &ble_cfg, ram_start);
//		APP_ERROR_CHECK(err_code);
//		err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
//		APP_ERROR_CHECK(err_code);	
	
	
    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
	
		/* The active focus protocol sends only 12 bytes about once per second.
		 * Extended connection events add radio latency but no useful throughput. */
		conn_evt_len_ext_set(false);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    /* A focus frame is 12 bytes and fits in the default 23-byte ATT MTU.
     * Keeping the default avoids an unnecessary large-MTU negotiation. */
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, BLE_GATT_ATT_MTU_DEFAULT);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
					printf("sleepmode\r\n");
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */  
void uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
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

void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}






void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE; //BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE    BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE   BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;  //APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;
	
		init.config.ble_adv_slow_enabled  = true;       
		init.config.ble_adv_slow_interval = MSEC_TO_UNITS(1000, UNIT_0_625_MS); 
		init.config.ble_adv_slow_timeout  = 0;         
	
	

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
		
//		err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,m_advertising.adv_handle,8);
//		if (err_code == NRF_SUCCESS) {
//				printf("TX Power set to: 8 dBm\r\n");
//		} else {
//				printf("Failed to set TX power: 0x%X\r\n", err_code);
//		}
}




void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();	
    }
}


void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}




//发送ecg数据
#define ECG_DATA_BUFFER_SIZE 200
#if 0 /* Legacy filtered-EEG sender; attention frames are the active protocol. */
static void ble_send_filtered_eeg_thread(void *pvParameter)
{
    uint8_t data_buffer[ECG_DATA_BUFFER_SIZE]; // 发送缓冲区
    uint16_t buffer_index = 0;

    while(1) {
			
				if( read_ecg_pos != write_ecg_pos ){  

        data_buffer[buffer_index++] = (save_ecg_data[read_ecg_pos] >> 24) & 0xFF;
        data_buffer[buffer_index++] = (save_ecg_data[read_ecg_pos]  >> 16) & 0xFF;
        data_buffer[buffer_index++] = (save_ecg_data[read_ecg_pos]  >> 8)  & 0xFF;
        data_buffer[buffer_index++] = (save_ecg_data[read_ecg_pos]) & 0xFF;
        
        if(buffer_index >= ECG_DATA_BUFFER_SIZE) {
            if((nrf_connect_stable_flag == 1) && m_nus_notifications_enabled) {
                ble_send_data(data_buffer, buffer_index);
            }
            buffer_index = 0; 
        }					
					read_ecg_pos =(read_ecg_pos+1+CYC_ARR_LEN)%CYC_ARR_LEN;	
				}
        
       	vTaskDelay(pdMS_TO_TICKS(10));   
    }
}





//发送六轴数据
#endif

#define ATTENTION_PACKET_MAGIC_0       0xA5U
#define ATTENTION_PACKET_MAGIC_1       0x5AU
#define ATTENTION_PACKET_VERSION       0x01U
#define ATTENTION_PACKET_TYPE          0x01U
#define ATTENTION_PACKET_PAYLOAD_LEN   0x06U
#define ATTENTION_PACKET_SIZE          12U

#define ATTENTION_FLAG_VALID           (1U << 0)
#define ATTENTION_FLAG_FOCUSED         (1U << 1)
#define ATTENTION_FLAG_CALIBRATING     (1U << 2)
#define ATTENTION_FLAG_ARTIFACT        (1U << 3)

static uint8_t attention_packet_checksum(
    const uint8_t *data,
    uint16_t length
)
{
    uint8_t checksum = 0U;
    uint16_t i;

    for (i = 0U; i < length; i++)
    {
        checksum ^= data[i];
    }

    return checksum;
}


static void attention_encode_packet(
    const attention_result_t *result,
    uint8_t packet[ATTENTION_PACKET_SIZE]
)
{
    uint8_t flags = 0U;
    uint32_t window = result->processed_windows;

    if (result->valid)
    {
        flags |= ATTENTION_FLAG_VALID;
    }
    if (result->focused)
    {
        flags |= ATTENTION_FLAG_FOCUSED;
    }
    if (result->calibrating)
    {
        flags |= ATTENTION_FLAG_CALIBRATING;
    }
    if (result->artifact)
    {
        flags |= ATTENTION_FLAG_ARTIFACT;
    }

    packet[0] = ATTENTION_PACKET_MAGIC_0;
    packet[1] = ATTENTION_PACKET_MAGIC_1;
    packet[2] = ATTENTION_PACKET_VERSION;
    packet[3] = ATTENTION_PACKET_TYPE;
    packet[4] = ATTENTION_PACKET_PAYLOAD_LEN;
    packet[5] = flags;
    packet[6] = result->score;
    packet[7] = (uint8_t)((window >> 24) & 0xFFU);
    packet[8] = (uint8_t)((window >> 16) & 0xFFU);
    packet[9] = (uint8_t)((window >> 8) & 0xFFU);
    packet[10] = (uint8_t)(window & 0xFFU);
    packet[11] = attention_packet_checksum(packet, 11U);
}


void ble_send_thread(void *pvParameter)
{
    attention_result_t result;
    uint8_t packet[ATTENTION_PACKET_SIZE];
    uint32_t last_sent_window = 0U;
    bool last_connected = false;
    bool last_notifications_enabled = false;

    (void)pvParameter;

    for (;;)
    {
        bool connected = ((nrf_connect_stable_flag == 1) &&
                          (m_conn_handle != BLE_CONN_HANDLE_INVALID));
        bool notifications_enabled = m_nus_notifications_enabled;

        ble_report_runtime_error();

        if (connected != last_connected)
        {
            printf("BLE connected=%u\r\n", connected ? 1U : 0U);
            last_connected = connected;
        }

        if (notifications_enabled != last_notifications_enabled)
        {
            printf("BLE notify=%u\r\n", notifications_enabled ? 1U : 0U);
            last_notifications_enabled = notifications_enabled;
        }

        if ((nrf_connect_stable_flag == 1) &&
            notifications_enabled &&
            attention_get_result(&result) &&
            (result.processed_windows != last_sent_window))
        {
            attention_encode_packet(&result, packet);
            ble_send_data(packet, ATTENTION_PACKET_SIZE);
            last_sent_window = result.processed_windows;
        }

        vTaskDelay(pdMS_TO_TICKS(50U));
    }
}


//extern QueueHandle_t sensor_data_queue;
//void ble_send_thread(void * pvParameter)
//{
//    sensor_data_t data;
//    uint8_t tx_buffer[52];  // 6 float * 4 + 7 int32 * 4 = 52
//    
//    while (1) {
//        // 等待数据
//        if (xQueueReceive(sensor_data_queue, &data, portMAX_DELAY) == pdTRUE) {
//            // 打包数据
//            memcpy(&tx_buffer[0], &data.ax, sizeof(float));
//            memcpy(&tx_buffer[4], &data.ay, sizeof(float));
//            memcpy(&tx_buffer[8], &data.az, sizeof(float));
//            memcpy(&tx_buffer[12], &data.gx, sizeof(float));
//            memcpy(&tx_buffer[16], &data.gy, sizeof(float));
//            memcpy(&tx_buffer[20], &data.gz, sizeof(float));
//					
//            memcpy(&tx_buffer[24], &data.year, sizeof(int));
//            memcpy(&tx_buffer[28], &data.month, sizeof(int));
//            memcpy(&tx_buffer[32], &data.day, sizeof(int));
//            memcpy(&tx_buffer[36], &data.weekday, sizeof(int));
//            memcpy(&tx_buffer[40], &data.hour, sizeof(int));
//            memcpy(&tx_buffer[44], &data.min, sizeof(int));
//            memcpy(&tx_buffer[48], &data.second, sizeof(int));
//					    
//            ble_send_data(tx_buffer, sizeof(tx_buffer));
//            
//            vTaskDelay(pdMS_TO_TICKS(100));
//        }
//    }
//}





////发送字符串25.6
//#define ECG_DATA_BUFFER_SIZE 200
//void ble_send_thread(void * pvParameter)
//{
//    while(1) {
//			
//        if(nrf_connect_stable_flag == 1) {
//// 方法2：发送格式化字符串
//float temperature = 25.6f;
//char buffer[20];
//int len = snprintf(buffer, sizeof(buffer), "%.1f", temperature);
//uint16_t length = len;
//ble_nus_data_send(&m_nus, (uint8_t*)buffer, &length, m_conn_handle);
//// 发送："25.6" 字符串
//        }
//					
//       	vTaskDelay(pdMS_TO_TICKS(10));   
//    }
//}




void ble_thread(void * pvParameter)
{
	(void)pvParameter;

	  printf("ble_thread\r\n");

	
    timers_init();
	


    ble_stack_init();

    gap_params_init();

    gatt_init();

    services_init();

    advertising_init();

    conn_params_init();


    advertising_start();
			

		printf("advertising...\r\n");

    /* nrf_sdh_freertos_init() owns a dedicated SoftDevice event task. This
     * task is only needed for initialization. Do not call the bare-metal
     * nrf_pwr_mgmt_run() loop from a SysTick-based FreeRTOS task: sleeping the
     * core also stops SysTick and can make every application task appear hung. */
    vTaskDelete(NULL);
}


