#ifndef __bluetooth_H
#define __bluetooth_H

/* Keep the focus firmware's NUS TX stream limited to attention frames. */
#define BLE_STREAM_ATTENTION_ONLY 1

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "nrf_ble_gatt.h"
//#include "ble_nus_c.h"









//-----------------peripheral-----------------
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
void timers_init(void);
void gap_params_init(void);
void nrf_qwr_error_handler(uint32_t nrf_error);
void nus_data_handler(ble_nus_evt_t * p_evt);
void services_init(void);
void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
void conn_params_error_handler(uint32_t nrf_error);
void conn_params_init(void);
void sleep_mode_enter(void);
void on_adv_evt(ble_adv_evt_t ble_adv_evt);
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
void ble_stack_init(void);
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
void gatt_init(void);
void bsp_event_handler(bsp_event_t event);
void uart_event_handle(app_uart_evt_t * p_event);
void uart_init(void);
void advertising_init(void);
void buttons_leds_init(bool * p_erase_bonds);
void power_management_init(void);
void idle_state_handle(void);
void advertising_start(void);
void ble_send_data(uint8_t *data, uint16_t length);



//-----------------central-----------------
//void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
//static void nus_error_handler(uint32_t nrf_error);
//void scan_start(void);
//static void scan_evt_handler(scan_evt_t const * p_scan_evt);
//void scan_init(void);
//static void db_disc_handler(ble_db_discovery_evt_t * p_evt);
//static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len);
//void uart_event_handle(app_uart_evt_t * p_event);
//static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt);
//static bool shutdown_handler(nrf_pwr_mgmt_evt_t event);
//static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
//void ble_stack_init(void);
//void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
//void gatt_init(void);
//void bsp_event_handler(bsp_event_t event);
//void uart_init(void);
//void nus_c_init(void);
//void buttons_leds_init(void);
//void timer_init(void);
//void power_management_init(void);
//void db_discovery_init(void);
//void idle_state_handle(void);

void ble_send_thread(void * pvParameter);
void ble_thread(void * pvParameter);	

#endif
