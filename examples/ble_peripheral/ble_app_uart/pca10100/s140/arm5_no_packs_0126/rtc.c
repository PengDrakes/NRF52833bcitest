
#if 1

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
#include "rtc.h"


#define RTC_ADDR 0x52
static const nrf_drv_twi_t twi0_master = NRF_DRV_TWI_INSTANCE(TWI0_INST);


static volatile bool m_xfer_done = false;
void  rtc_twim_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //printf("rx completed!\r\n");
            }
						else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
            {
                //printf("TX transfer completed!\r\n");
            }
            m_xfer_done = true;
            break;
				case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            printf("address nack received!\r\n");
            break;
				case NRF_DRV_TWI_EVT_DATA_NACK:
            printf("data nack received!\r\n");
            break;
        default:
						printf("unknown TWI event :%d\r\n",p_event->type);
            break;
    }
}

static ret_code_t twi0_master_init(void)
{
    ret_code_t ret;

    const nrf_drv_twi_config_t config =
    {
       .scl                = RTC_SCL,
       .sda                = RTC_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,  
       .clear_bus_init     = true ,
			 .hold_bus_uninit    = false
    };

    ret = nrf_drv_twi_init(&twi0_master, &config, rtc_twim_handler , NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&twi0_master);
    }
		printf("i2c master init success\r\n");

    return ret;
}




ret_code_t rtc_i2c_write(uint8_t reg, uint8_t value)
{
    ret_code_t result;
		TickType_t xTimeout;
    uint8_t buffer[2] = {reg, value};
    
    m_xfer_done = false;
    result = nrf_drv_twi_tx(&twi0_master, RTC_ADDR, buffer, sizeof(buffer), false);

		if(result == NRF_SUCCESS){
				xTimeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
				while (!m_xfer_done)
				{
					 vTaskDelay(pdMS_TO_TICKS(1));
					 if (xTaskGetTickCount() > xTimeout)
					 {
							printf("TWI  timeout!=============\r\n");
							return NRF_ERROR_TIMEOUT;
					 }
				}
    }
		else{
				return result;
		}

    return NRF_SUCCESS;
}



		
ret_code_t rtc_i2c_read(uint8_t reg, uint8_t * p_value,uint8_t length)
{
    ret_code_t err_code;
		TickType_t xTimeout;   
	
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi0_master, RTC_ADDR, &reg, 1, true);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    } 
		xTimeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    while (!m_xfer_done)
    {
				vTaskDelay(pdMS_TO_TICKS(1));
				if (xTaskGetTickCount() > xTimeout)
				{
					 printf("TWI  timeout in tx!=============\r\n");
					 return NRF_ERROR_TIMEOUT;
				}
    }
		
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&twi0_master, RTC_ADDR, p_value, length);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		xTimeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    while (!m_xfer_done)
    {
				vTaskDelay(pdMS_TO_TICKS(1));
				if (xTaskGetTickCount() > xTimeout)
				{
					 printf("TWI  timeout in rx!=============\r\n");
					 return NRF_ERROR_TIMEOUT;
				}
    }
		 
    return NRF_SUCCESS;
}


uint16_t BCDToInt(unsigned char bcd) {
    unsigned char ten_weight = (bcd & 0xF0) >> 4; 
    unsigned char one_weight = bcd & 0x0F;       
    return ten_weight * 10 + one_weight;    
}
void rtc_init(void){
		ret_code_t result;
		uint8_t id[1];
		uint8_t seconds[1];		

	
		result = rtc_i2c_write(0x10,0x01);      // reset
		vTaskDelay(pdMS_TO_TICKS(5000)); 
		if (result != NRF_SUCCESS)
    {
        printf("RTC Failed to reset: 0x%X\r\n", result);   
    }else{
        printf("RTC success to reset\r\n"); 		
		}
		
		result = rtc_i2c_read(0x28,id,1);      // id
		if (result != NRF_SUCCESS)
    {
        printf("RTC Failed to reset: 0x%X\r\n", result);   
    }else{
			printf("id:0x%x\r\n",id[0]); 		
		}
		
		rtc_i2c_write(0x00,0x00);   //秒
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x01,0x00);   //分
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x02,0x12);   //时
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x03,0x04);   //星期
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x04,0x13);   //日
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x05,0x02);   // 月
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x06,0x26);   //年
		vTaskDelay(pdMS_TO_TICKS(5)); 		
		
		
		
		
}

void check_rtc_i2c_busy(void)
{
    uint8_t part_id = 0;
    ret_code_t ret;
	
	  if (nrf_drv_twi_is_busy(&twi0_master))
    {
			 printf("error: qmi twi busy\r\n");
       return;
    }
		else{
			 printf("qmi twi is not busy\r\n");
		}
}

volatile int rtc_int_flag ;
static void rtc_int_isr_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin == RTC_INT)
    {
        if (action == NRF_GPIOTE_POLARITY_HITOLO)  
        {
            rtc_int_flag = 1;
        }
    }
}
void rtc_int_config(void){ 

	
		APP_ERROR_CHECK(nrfx_gpiote_init());   
		nrf_gpio_cfg_input(RTC_INT, NRF_GPIO_PIN_PULLUP);    //NRF_GPIO_PIN_NOPULL
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true); //下降沿触发
		in_config.pull = NRF_GPIO_PIN_PULLUP;
    APP_ERROR_CHECK(nrfx_gpiote_in_init(RTC_INT, &in_config, rtc_int_isr_handler));
    nrfx_gpiote_in_event_enable(RTC_INT, true);
	
	  printf("rtc int pin set success\r\n");
	
}



void rtc_thread(void * pvParameter){
		uint8_t time[7];	
	
		twi0_master_init();
		vTaskDelay(pdMS_TO_TICKS(1000)); 
		printf("0\r\n");
		rtc_int_config();
		vTaskDelay(pdMS_TO_TICKS(1000)); 	
		printf("1\r\n");
		check_rtc_i2c_busy();
		vTaskDelay(pdMS_TO_TICKS(1000)); 
		printf("2\r\n");	
		rtc_init();	
		vTaskDelay(pdMS_TO_TICKS(1000)); 

		while(1){
		
			rtc_i2c_read(0x00,time,7); 
			printf("time : 20%d年 %d月 %d日 周%d %d时 %d分 %d秒\r\n", BCDToInt(time[6]),BCDToInt(time[5]),BCDToInt(time[4]),BCDToInt(time[3]),BCDToInt(time[2]),BCDToInt(time[1]),BCDToInt(time[0]&0x7f));			
		  vTaskDelay(pdMS_TO_TICKS(1000)); 				
		
		}


}



#endif











#if 0
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
#include "rtc.h"



	
#define RTC_ADDR 0x52
const nrf_drv_twi_t twi0_master = NRF_DRV_TWI_INSTANCE(TWI0_INST);


static volatile bool m_xfer_done = false;
void  rtc_twim_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                //printf("rx completed!\r\n");
            }
						else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
            {
                //printf("TX transfer completed!\r\n");
            }
            m_xfer_done = true;
            break;
				case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            printf("address nack received!\r\n");
            break;
				case NRF_DRV_TWI_EVT_DATA_NACK:
            printf("data nack received!\r\n");
            break;
        default:
						printf("unknown TWI event :%d\r\n",p_event->type);
            break;
    }
}





ret_code_t twi0_master_init(void)
{
    ret_code_t ret;

    const nrf_drv_twi_config_t config =
    {
       .scl                = RTC_SCL,
       .sda                = RTC_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,  
       .clear_bus_init     = true ,
			 .hold_bus_uninit    = false
    };

    ret = nrf_drv_twi_init(&twi0_master, &config, rtc_twim_handler , NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&twi0_master);
    }
		printf("i2c master init success\r\n");

    return ret;
}




ret_code_t rtc_i2c_write(uint8_t reg, uint8_t value)
{
    ret_code_t result;
		TickType_t xTimeout;
    uint8_t buffer[2] = {reg, value};
    
    m_xfer_done = false;
    result = nrf_drv_twi_tx(&twi0_master, RTC_ADDR, buffer, sizeof(buffer), false);

		if(result == NRF_SUCCESS){
				xTimeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
				while (!m_xfer_done)
				{
					 vTaskDelay(pdMS_TO_TICKS(1));
					 if (xTaskGetTickCount() > xTimeout)
					 {
							printf("TWI  timeout!=============\r\n");
							return NRF_ERROR_TIMEOUT;
					 }
				}
    }
		else{
				return result;
		}

    return NRF_SUCCESS;
}



		
ret_code_t rtc_i2c_read(uint8_t reg, uint8_t * p_value,uint8_t length)
{
    ret_code_t err_code;
		TickType_t xTimeout;   
	
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&twi0_master, RTC_ADDR, &reg, 1, true);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    } 
		xTimeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    while (!m_xfer_done)
    {
				vTaskDelay(pdMS_TO_TICKS(1));
				if (xTaskGetTickCount() > xTimeout)
				{
					 printf("TWI  timeout in tx!=============\r\n");
					 return NRF_ERROR_TIMEOUT;
				}
    }
		
    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&twi0_master, RTC_ADDR, p_value, length);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		xTimeout = xTaskGetTickCount() + pdMS_TO_TICKS(5000);
    while (!m_xfer_done)
    {
				vTaskDelay(pdMS_TO_TICKS(1));
				if (xTaskGetTickCount() > xTimeout)
				{
					 printf("TWI  timeout in rx!=============\r\n");
					 return NRF_ERROR_TIMEOUT;
				}
    }
		 
    return NRF_SUCCESS;
}


uint16_t BCDToInt(unsigned char bcd) {
    unsigned char ten_weight = (bcd & 0xF0) >> 4; 
    unsigned char one_weight = bcd & 0x0F;       
    return ten_weight * 10 + one_weight;    
}
void rtc_reg_init(void){
		ret_code_t result;
		uint8_t id[1];
		uint8_t seconds[1];		

	
		result = rtc_i2c_write(0x10,0x01);      // reset
		vTaskDelay(pdMS_TO_TICKS(5000)); 
		if (result != NRF_SUCCESS)
    {
        printf("RTC Failed to reset: 0x%X\r\n", result);   
    }else{
        printf("RTC success to reset\r\n"); 		
		}
		
		result = rtc_i2c_read(0x28,id,1);      // id
		if (result != NRF_SUCCESS)
    {
        printf("RTC Failed to reset: 0x%X\r\n", result);   
    }else{
			printf("rtc id:0x%x\r\n",id[0]); 		
		}
		
		rtc_i2c_write(0x00,0x00);   //秒
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x01,0x00);   //分
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x02,0x12);   //时
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x03,0x04);   //星期
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x04,0x13);   //日
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x05,0x02);   // 月
		vTaskDelay(pdMS_TO_TICKS(5)); 
		rtc_i2c_write(0x06,0x26);   //年
		vTaskDelay(pdMS_TO_TICKS(5)); 		
		
		
		
		
}

void check_rtc_i2c_busy(void)
{
    uint8_t part_id = 0;
    ret_code_t ret;
	
	  if (nrf_drv_twi_is_busy(&twi0_master))
    {
			 printf("error: qmi twi busy\r\n");
       return;
    }
		else{
			 printf("qmi twi is not busy\r\n");
		}
}

volatile int rtc_int_flag ;
static void rtc_int_isr_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin == RTC_INT)
    {
        if (action == NRF_GPIOTE_POLARITY_HITOLO)  
        {
            rtc_int_flag = 1;
        }
    }
}
void rtc_int_config(void){ 

	
		APP_ERROR_CHECK(nrfx_gpiote_init());   
		nrf_gpio_cfg_input(RTC_INT, NRF_GPIO_PIN_PULLUP);    //NRF_GPIO_PIN_NOPULL
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true); //下降沿触发
		in_config.pull = NRF_GPIO_PIN_PULLUP;
    APP_ERROR_CHECK(nrfx_gpiote_in_init(RTC_INT, &in_config, rtc_int_isr_handler));
    nrfx_gpiote_in_event_enable(RTC_INT, true);
	
	  printf("rtc int pin set success\r\n");
	
}


void rtc_init(void){

		twi0_master_init();
		vTaskDelay(pdMS_TO_TICKS(1000)); 

//		rtc_int_config();
//		vTaskDelay(pdMS_TO_TICKS(1000)); 	
	
		rtc_reg_init();	
		vTaskDelay(pdMS_TO_TICKS(1000)); 
}

void rtc_low_power(void){




}

void rtc_wake_up(void){



}



int32_t rtc_read_time(uint8_t startaddr,uint8_t data[],uint32_t datasize){
		rtc_i2c_read(startaddr,data,datasize); 
		
		vTaskDelay(pdMS_TO_TICKS(1000)); 	

		return 1;	
}   //			uint8_t time[7];		rtc_i2c_read(0x00,time,7);   printf("time : 20%d年 %d月 %d日 周%d %d时 %d分 %d秒\r\n", BCDToInt(time[6]),BCDToInt(time[5]),BCDToInt(time[4]),BCDToInt(time[3]),BCDToInt(time[2]),BCDToInt(time[1]),BCDToInt(time[0]&0x7f));			



#endif







