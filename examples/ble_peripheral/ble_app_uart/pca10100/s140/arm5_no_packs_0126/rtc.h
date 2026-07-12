#ifndef __RTC_H
#define __RTC_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "nrf.h"
#include "bsp.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrfx_twim.h"
#include "nrfx_gpiote.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "nrf_delay.h"

#define TWI0_INST     1
#define RTC_SDA 6
#define RTC_SCL  5
#define RTC_INT  8 
#define RTC_EVI  4
//#define RTC_CLKOUT


void  rtc_twim_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
static ret_code_t rtc_twi_master_init(void);
ret_code_t rtc_i2c_write(uint8_t reg, uint8_t value);
ret_code_t rtc_i2c_read(uint8_t reg, uint8_t * p_value,uint8_t length);
uint16_t BCDToInt(unsigned char bcd);
void rtc_init(void);
void check_rtc_i2c_busy(void);
static void rtc_int_isr_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void rtc_int_config(void);
void rtc_thread(void * pvParameter);


//void  rtc_twim_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
//ret_code_t rtc_i2c_write(uint8_t reg, uint8_t value);
//ret_code_t rtc_i2c_read(uint8_t reg, uint8_t * p_value,uint8_t length);
//uint16_t BCDToInt(unsigned char bcd);
//void rtc_reg_init(void);
//void check_rtc_i2c_busy(void);
//static void rtc_int_isr_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
//void rtc_int_config(void);
//void rtc_init(void);
//void rtc_low_power(void);
//void rtc_wake_up(void);
//int32_t rtc_read_time(uint8_t startaddr,uint8_t data[],uint32_t datasize);

//ret_code_t rtc_twi_master_init(void);

#endif