#ifndef __ads1292_H
#define __ads1292_H

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "arm_math.h"
#include "bluetooth.h"



void spi_event_handler(nrf_drv_spi_evt_t const * p_event,void * p_context);
void ads1292_standby(void);	
void ads1292_wakeup(void);	
void ads1292_init(void);
void ADS1292_val_init(float32_t *data,float32_t *a,float32_t *b);
void bre_val_init(float32_t *data,float32_t *a,float32_t *b);
static void drdy_isr_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void spi_read (void * pvParameter);
void device_init(void);
//void open_ADS1292(void);
//void close_ADS1292(void);
int ads1292_main(void);


#endif