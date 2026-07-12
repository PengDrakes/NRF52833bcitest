
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
#include "spo2.h"
#include "nrfx_twim.h"
#include "nrfx_gpiote.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "nrf_delay.h"
#include "max30102_fir.h"
#include "bluetooth.h"

uint32_t spo2 ;
int spo2_data_ready ;

uint8_t m_reg_addr;
uint8_t fifo_data[6]; 
float max30102_data[2],fir_output[2];

volatile int max30102_int_flag ;

#define PPG_DATA_THRESHOLD 100000 	//检测阈值
#define CACHE_NUMS 300//缓存数
float ppg_data_cache_RED[CACHE_NUMS]={0};  //缓存区
float ppg_data_cache_IR[CACHE_NUMS]={0};  //缓存区

#define MAX30102_ADDR 0x57
static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);


// TWI is used in blocking mode so NACK/BUSY errors are returned directly by nrf_drv_twi_tx/rx.


static ret_code_t twi_master_init(void)
{
    ret_code_t ret;

    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,  
       .clear_bus_init     = true ,
			 .hold_bus_uninit    = false
    };

    ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi_master);
    }
		printf("i2c master init success\r\n");

    return ret;
}



static void int_isr_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin == INT_PIN)
    {
        if (action == NRF_GPIOTE_POLARITY_HITOLO)  
        {
            max30102_int_flag = 1;
        }
    }
}
void int_pin_config(void){ 

	
		ret_code_t err = nrfx_gpiote_init();

		if ((err != NRF_SUCCESS) &&
				(err != NRFX_ERROR_INVALID_STATE))
		{
				APP_ERROR_CHECK(err);
		}
		nrf_gpio_cfg_input(INT_PIN, NRF_GPIO_PIN_PULLUP);    //NRF_GPIO_PIN_NOPULL
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true); //下降沿触发
		in_config.pull = NRF_GPIO_PIN_PULLUP;
    APP_ERROR_CHECK(nrfx_gpiote_in_init(INT_PIN, &in_config, int_isr_handler));
    nrfx_gpiote_in_event_enable(INT_PIN, true);
	
	  printf("int pin set success\r\n");
	
}



ret_code_t max30102_i2c_write(uint8_t reg, uint8_t value)
{
    ret_code_t result;
    uint8_t buffer[2] = {reg, value};

    result = nrf_drv_twi_tx(&m_twi_master, MAX30102_ADDR, buffer, sizeof(buffer), false);
    if (result != NRF_SUCCESS)
    {
        printf("TWI write failed reg=0x%02X err=0x%X\r\n", reg, result);
    }

    return result;
}

ret_code_t max30102_i2c_read(uint8_t reg, uint8_t * p_value,uint8_t length)
{
    ret_code_t err_code;

    err_code = nrf_drv_twi_tx(&m_twi_master, MAX30102_ADDR, &reg, 1, true);
    if (err_code != NRF_SUCCESS)
    {
        printf("TWI read tx failed reg=0x%02X err=0x%X\r\n", reg, err_code);
        return err_code;
    }

    err_code = nrf_drv_twi_rx(&m_twi_master, MAX30102_ADDR, p_value, length);
    if (err_code != NRF_SUCCESS)
    {
        printf("TWI read rx failed reg=0x%02X err=0x%X\r\n", reg, err_code);
    }

    return err_code;
}

static ret_code_t i2c_probe_address(uint8_t address)
{
    ret_code_t err_code;
    uint8_t probe = 0;

    err_code = nrf_drv_twi_tx(&m_twi_master, address, &probe, 1, false);
    if (err_code == NRF_ERROR_DRV_TWI_ERR_DNACK)
    {
        return NRF_SUCCESS;
    }

    return err_code;
}

static void scan_i2c_bus(void)
{
    bool found = false;
    uint8_t address;

    printf("I2C scan start, scl=%lu(P0.09), sda=%lu(P1.06), int=%lu(P1.04), target=0x%02X\r\n",
           (unsigned long)TWI_SCL_M,
           (unsigned long)TWI_SDA_M,
           (unsigned long)INT_PIN,
           MAX30102_ADDR);
    printf("I2C pin level before scan: scl=%lu sda=%lu int=%lu\r\n",
           (unsigned long)nrf_gpio_pin_read(TWI_SCL_M),
           (unsigned long)nrf_gpio_pin_read(TWI_SDA_M),
           (unsigned long)nrf_gpio_pin_read(INT_PIN));

    for (address = 0x03; address < 0x78; address++)
    {
        ret_code_t err_code = i2c_probe_address(address);
        if (err_code == NRF_SUCCESS)
        {
            printf("I2C device found at 0x%02X%s\r\n", address, (address == MAX30102_ADDR) ? " (MAX30102 target)" : "");
            found = true;
        }
        else if (err_code == NRF_ERROR_BUSY)
        {
            printf("I2C probe busy at 0x%02X\r\n", address);
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    printf("I2C pin level after scan: scl=%lu sda=%lu int=%lu busy=%d\r\n",
           (unsigned long)nrf_gpio_pin_read(TWI_SCL_M),
           (unsigned long)nrf_gpio_pin_read(TWI_SDA_M),
           (unsigned long)nrf_gpio_pin_read(INT_PIN),
           nrf_drv_twi_is_busy(&m_twi_master));

    if (!found)
    {
        printf("I2C scan found no device. Check VCC_1P8V, VCC_3P3V, TXS0102 OE, R68/R69, MAX30102 soldering.\r\n");
    }
}
void max30102_fifo_read(float *output_data)
{
    ret_code_t err_code;
    uint32_t data[2];
	
		err_code = max30102_i2c_read(FIFO_DATA_REG, fifo_data,6);
		
	//printf("fifo:%d %d %d %d %d %d \r\n",fifo_data[0],fifo_data[1],fifo_data[2],fifo_data[3],fifo_data[4],fifo_data[5]);
	
		data[0] = ((fifo_data[0]<<16 | fifo_data[1]<<8 | fifo_data[2]) & 0x03ffff);
		data[1] = ((fifo_data[3]<<16 | fifo_data[4]<<8 | fifo_data[5]) & 0x03ffff);
		*output_data = data[0];
		*(output_data+1) = data[1];
		
		if (err_code != NRF_SUCCESS) {
        printf("Read fifo failed: 0x%X\r\n", err_code);
    }
}



void max30102_init(void){
		ret_code_t result;
		uint8_t status_data;

		result = max30102_i2c_write(MODE_CONFIGURATION,0x40);      // reset
		if (result != NRF_SUCCESS)
    {
        printf("Failed to reset: 0x%X\r\n", result);   
    }
		
		max30102_i2c_write(INTERRUPT_ENABLE1,0xE0);
		max30102_i2c_write(INTERRUPT_ENABLE2,0x00);
		
		max30102_i2c_write(FIFO_WR_POINTER,0x00);
		max30102_i2c_write(FIFO_OV_COUNTER,0x00);
		max30102_i2c_write(FIFO_RD_POINTER,0x00);
		
		max30102_i2c_write(FIFO_CONFIGURATION,0x4F);
		
		
    // spo2_mode
		result = max30102_i2c_write(MODE_CONFIGURATION,0x03);
		if (result != NRF_SUCCESS)
    {
        printf("Failed to set spo2_mode: 0x%X\r\n", result);   
    }
		
		max30102_i2c_write(SPO2_CONFIGURATION,0x2a); //0x32
		
		max30102_i2c_write(LED1_PULSE_AMPLITUDE,0x3f);	//  RED LED current
		max30102_i2c_write(LED2_PULSE_AMPLITUDE,0x3f); //   IR LED
		
		max30102_i2c_write(TEMPERATURE_CONFIG,0x01); 
		
		max30102_i2c_read(INTERRUPT_STATUS1,&status_data,1);
		max30102_i2c_read(INTERRUPT_STATUS2,&status_data,1);  //clear status		
}

void test_i2c_communication(void)
{
    uint8_t part_id = 0;
    ret_code_t ret;
	
	  if (nrf_drv_twi_is_busy(&m_twi_master))
    {
			 printf("error: twi busy\r\n");
       return;
    }
		else{
			 printf("twi is not busy\r\n");
		}
		
		ret = max30102_i2c_read(PART_ID, &part_id,1);
    if (ret == NRF_SUCCESS) {
        printf("part ID: 0x%02X\r\n", part_id);     //0x15
    } else {
        printf("Read part ID failed: 0x%X\r\n", ret);
    }
}

uint16_t max30102_getHeartRate(float *input_data,uint16_t cache_nums)
{
		float input_data_sum_aver = 0;
		uint16_t i,temp;
		
		
		for(i=0;i<cache_nums;i++)
		{
		input_data_sum_aver += *(input_data+i);
		}
		input_data_sum_aver = input_data_sum_aver/cache_nums;
		for(i=0;i<cache_nums;i++)
		{
				if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
				{
					temp = i;
					break;
				}
		}
		i++;
		for(;i<cache_nums;i++)
		{
				if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
				{
					temp = i - temp;
					break;
				}
		}
		if((temp>15)&&(temp<150))   //    20-200
		{
				printf("temp=%d	",temp);
				return 3000/temp;	//SpO2采样频率200Hz，FIFO采样平均过滤值为4，所以最终采样频率算为50Hz。采样一次的周期是0.02s，temp是一次心跳过程中的采样点个数，一次心跳就是temp/50秒,一分钟心跳数就是60/(temp/50)。
		}
		else
		{
				printf("temp=%d	",temp);
				return 0;
		}
}

float max30102_getSpO2(float *ir_input_data,float *red_input_data,uint16_t cache_nums)
{
			float ir_max=*ir_input_data,ir_min=*ir_input_data;
			float red_max=*red_input_data,red_min=*red_input_data;
			float R;
			uint16_t i;
			for(i=1;i<cache_nums;i++)
			{
				if(ir_max<*(ir_input_data+i))
				{
					ir_max=*(ir_input_data+i);
				}
				if(ir_min>*(ir_input_data+i))
				{
					ir_min=*(ir_input_data+i);
				}
				if(red_max<*(red_input_data+i))
				{
					red_max=*(red_input_data+i);
				}
				if(red_min>*(red_input_data+i))
				{
					red_min=*(red_input_data+i);
				}
			}
			R=((ir_max-ir_min)*red_min)/((red_max-red_min)*ir_min);
//			 R=((ir_max+ir_min)*(red_max-red_min))/((red_max+red_min)*(ir_max-ir_min));
			printf("R=%f	",R);
			return ((-45.060)*R*R + 30.354*R + 94.845);
}



static void send_spo2_to_ble(uint16_t sample_count)
{
        int32_t spo2_value = (int32_t)(max30102_getSpO2(ppg_data_cache_IR, ppg_data_cache_RED, sample_count));
        char ble_buffer[20];
        int ble_len;

        if(spo2_value < 50)
        {
                spo2_value = 50;
        }
        else if(spo2_value > 100)
        {
                spo2_value = 100;
        }

        ble_len = snprintf(ble_buffer, sizeof(ble_buffer), "SPO2:%ld\n", (long)spo2_value);
        if(ble_len > 0)
        {
                ble_send_data((uint8_t *)ble_buffer, (uint16_t)ble_len);
                printf("send spo2 samples=%u value=%ld\r\n", sample_count, (long)spo2_value);
        }
}
void spo2_read(void * pvParameter){ 
		uint16_t cache_counter = 0;  //缓存计数器
		uint8_t status_data = 0;
        uint16_t no_touch_counter = 0;
	
		twi_master_init();
		vTaskDelay(pdMS_TO_TICKS(1000)); 
	
		int_pin_config();
	  vTaskDelay(pdMS_TO_TICKS(1000)); 

		        scan_i2c_bus();
        vTaskDelay(pdMS_TO_TICKS(1000));

		test_i2c_communication();
		vTaskDelay(pdMS_TO_TICKS(1000)); 
	
		max30102_init();	
		vTaskDelay(pdMS_TO_TICKS(1000)); 
	
		max30102_fir_init();
				
    while(1){
				//printf("int_flag:%d\r\n",max30102_int_flag);
				if(max30102_int_flag){
						//spo2_data_ready = 0;
					
					
					//printf("max_int_flag==1\r\n");
					
						max30102_i2c_read(INTERRUPT_STATUS1,&status_data,1);
						max30102_i2c_read(INTERRUPT_STATUS2,&status_data,1);  //clear status		
					
						max30102_int_flag = 0;

						max30102_fifo_read(max30102_data);
					
						ir_max30102_fir(&max30102_data[0],&fir_output[0]);//实测ir数据采集在前面，red数据在后面
            red_max30102_fir(&max30102_data[1],&fir_output[1]);  //滤波
					
												if((max30102_data[0]>PPG_DATA_THRESHOLD)&&(max30102_data[1]>PPG_DATA_THRESHOLD))  //大于阈值，说明传感器有接触
						{
                ppg_data_cache_IR[cache_counter]=fir_output[0];
                ppg_data_cache_RED[cache_counter]=fir_output[1];
								cache_counter++;
                                no_touch_counter=0;
                                if((cache_counter % 50) == 0)
                                {
                                        printf("spo2 cache: %u/%u raw_ir=%lu raw_red=%lu\r\n", cache_counter, CACHE_NUMS, (unsigned long)max30102_data[0], (unsigned long)max30102_data[1]);
                                        if(cache_counter >= 100 && cache_counter < CACHE_NUMS)
                                        {
                                                send_spo2_to_ble(cache_counter);
                                        }
                                }
						}
                        else
                        {
                                no_touch_counter++;
                                if((no_touch_counter % 10) == 1)
                                {
                                        printf("please touch the sensor! raw_ir=%lu raw_red=%lu threshold=%lu\r\n", (unsigned long)max30102_data[0], (unsigned long)max30102_data[1], (unsigned long)PPG_DATA_THRESHOLD);
                                }
                                if(no_touch_counter >= 5)
                                {
                                        cache_counter=0;
                                }
						}
						
						
						if(cache_counter>=CACHE_NUMS)  //收集满了数据
            {
								//spo2_data_ready = 1;
								//spo2_data_ready = 1;
                                send_spo2_to_ble(CACHE_NUMS);

                printf("心率：%d 次/min   ",max30102_getHeartRate(ppg_data_cache_IR,CACHE_NUMS));
                printf("血氧：%.2f  \r\n",max30102_getSpO2(ppg_data_cache_IR,ppg_data_cache_RED,CACHE_NUMS));
							
							
							
//							for(int i=0;i<CACHE_NUMS;i++){
//								printf("%f\r\n",ppg_data_cache_IR[i]);
//							
//									nrf_delay_ms(1);
//							
//							}
							
                cache_counter=0;
            }
				
				}
			
				vTaskDelay(pdMS_TO_TICKS(1));
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
#include "spo2.h"
#include "nrfx_twim.h"
#include "nrfx_gpiote.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "nrf_delay.h"
#include "max30102_fir.h"
#include "semphr.h"

extern int flag_run;
extern SemaphoreHandle_t wake_up_sem;

uint8_t m_reg_addr;
uint8_t fifo_data[6]; 
float max30102_data[2],fir_output[2];

volatile int max30102_int_flag ;

#define PPG_DATA_THRESHOLD 100000 	//检测阈值
#define CACHE_NUMS 300//缓存数
float ppg_data_cache_RED[CACHE_NUMS]={0};  //缓存区
float ppg_data_cache_IR[CACHE_NUMS]={0};  //缓存区


#define MAX30102_ADDR 0x57

extern nrf_drv_twi_t m_twi_master;
nrf_drv_twi_t max30102_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);

int spo2_wake_up=0;

static volatile bool m_xfer_done = false;
void  my_twim_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
//                printf("rx completed!\r\n");
            }
						else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
            {
//                printf("TX transfer completed!\r\n");
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

	
	
//	  nrf_drv_twi_disable(&twi0_master);
//		nrf_delay_ms(10);
//    nrf_drv_twi_uninit(&twi0_master);
//		nrf_delay_ms(10);
	
//		nrf_gpio_cfg_default(5);
//		nrf_gpio_cfg_default(6);
//		nrf_delay_ms(10);


//	  nrf_gpio_cfg_output(6);
//    nrf_gpio_pin_set(6);  // 先拉高SDA
//    nrf_delay_us(5);
//	
//    nrf_gpio_cfg_output(5);
//    for (int i = 0; i < 9; i++) {  // 发送9个时钟脉冲
//        nrf_gpio_pin_clear(5);
//        nrf_delay_us(5);
//        nrf_gpio_pin_set(5);
//        nrf_delay_us(5);
//    }

//    nrf_gpio_cfg_default(5);
//    nrf_gpio_cfg_default(6);		
//		nrf_delay_ms(10);
static ret_code_t twi_master_init(void)
{
    ret_code_t ret;
	
//		 nrf_drv_twi_disable(&m_twi_master);
//		nrf_delay_ms(10);
//    nrf_drv_twi_uninit(&m_twi_master);
//		nrf_delay_ms(10);

    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,   //9
       .sda                = TWI_SDA_M,   //38
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,  
       .clear_bus_init     = true ,
			 .hold_bus_uninit    = false
    };

    ret = nrf_drv_twi_init(&max30102_twi_master, &config, my_twim_handler , NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&max30102_twi_master);
    }
		printf("max i2c master init success\r\n");

    return ret;
}



static void int_isr_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin == INT_PIN)
    {
        if (action == NRF_GPIOTE_POLARITY_HITOLO)  
        {
            max30102_int_flag = 1;
        }
    }
}
void int_pin_config(void){ 

		APP_ERROR_CHECK(nrfx_gpiote_init());  
		nrf_gpio_cfg_input(INT_PIN, NRF_GPIO_PIN_PULLUP);    //NRF_GPIO_PIN_NOPULL
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true); //下降沿触发
		in_config.pull = NRF_GPIO_PIN_PULLUP;
    APP_ERROR_CHECK(nrfx_gpiote_in_init(INT_PIN, &in_config, int_isr_handler));
    nrfx_gpiote_in_event_enable(INT_PIN, true);
	
	  printf("int pin set success\r\n");
}



ret_code_t max30102_i2c_write(uint8_t reg, uint8_t value)
{
    ret_code_t result;
		TickType_t xTimeout;
    uint8_t buffer[2] = {reg, value};
    
    m_xfer_done = false;
    result = nrf_drv_twi_tx(&max30102_twi_master, MAX30102_ADDR, buffer, sizeof(buffer), false);

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



		
ret_code_t max30102_i2c_read(uint8_t reg, uint8_t * p_value,uint8_t length)
{
    ret_code_t err_code;
		TickType_t xTimeout;   
	
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&max30102_twi_master, MAX30102_ADDR, &reg, 1, true);
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
    err_code = nrf_drv_twi_rx(&max30102_twi_master, MAX30102_ADDR, p_value, length);
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


void max30102_fifo_read(float *output_data)
{
    ret_code_t err_code;
    uint32_t data[2];
	
		err_code = max30102_i2c_read(FIFO_DATA_REG, fifo_data,6);
		
		data[0] = ((fifo_data[0]<<16 | fifo_data[1]<<8 | fifo_data[2]) & 0x03ffff);
		data[1] = ((fifo_data[3]<<16 | fifo_data[4]<<8 | fifo_data[5]) & 0x03ffff);
		*output_data = data[0];
		*(output_data+1) = data[1];
		
		if (err_code != NRF_SUCCESS) {
        printf("Read fifo failed: 0x%X\r\n", err_code);
    }
}


		uint8_t status_data;
void max30102_init(void){
		ret_code_t result;

		//uint8_t status_data;
		result = max30102_i2c_write(MODE_CONFIGURATION,0x40);      // reset
		if (result != NRF_SUCCESS)
    {
        printf("Failed to reset: 0x%X\r\n", result);   
    }
		
		max30102_i2c_write(INTERRUPT_ENABLE1,0xe0);  //e0----==========
		max30102_i2c_write(INTERRUPT_ENABLE2,0x00);
		
		max30102_i2c_write(FIFO_WR_POINTER,0x00);
		max30102_i2c_write(FIFO_OV_COUNTER,0x00);
		max30102_i2c_write(FIFO_RD_POINTER,0x00);
		
		max30102_i2c_write(FIFO_CONFIGURATION,0x4f);  //4f------------------
		
		
    // spo2_mode
		result = max30102_i2c_write(MODE_CONFIGURATION,0x03);
		if (result != NRF_SUCCESS)
    {
        printf("Failed to set spo2_mode: 0x%X\r\n", result);   
    }
		
		max30102_i2c_write(SPO2_CONFIGURATION,0x2a); //0x32
		
		max30102_i2c_write(LED1_PULSE_AMPLITUDE,0x3f);	//  RED LED current
		max30102_i2c_write(LED2_PULSE_AMPLITUDE,0x3f); //   IR LED
		
		max30102_i2c_write(TEMPERATURE_CONFIG,0x01); 
		
		max30102_i2c_read(INTERRUPT_STATUS1,&status_data,1);
		max30102_i2c_read(INTERRUPT_STATUS2,&status_data,1);  //clear status		
		
		printf("ststus cleared\r\n");
}

void test_i2c_communication(void)
{
    uint8_t part_id = 0;
    ret_code_t ret;
	
	  if (nrf_drv_twi_is_busy(&max30102_twi_master))
    {
			 printf("error: twi busy\r\n");
       return;
    }
		else{
			 printf("twi is not busy\r\n");
		}
		
		ret = max30102_i2c_read(PART_ID, &part_id,1);
    if (ret == NRF_SUCCESS) {
        printf("part ID: 0x%02X\r\n", part_id);     //0x15
    } else {
        printf("Read part ID failed: 0x%X\r\n", ret);
    }
}

uint16_t max30102_getHeartRate(float *input_data,uint16_t cache_nums)
{
		float input_data_sum_aver = 0;
		uint16_t i,temp;
		
		
		for(i=0;i<cache_nums;i++)
		{
		input_data_sum_aver += *(input_data+i);
		}
		input_data_sum_aver = input_data_sum_aver/cache_nums;
		for(i=0;i<cache_nums;i++)
		{
				if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
				{
					temp = i;
					break;
				}
		}
		i++;
		for(;i<cache_nums;i++)
		{
				if((*(input_data+i)>input_data_sum_aver)&&(*(input_data+i+1)<input_data_sum_aver))
				{
					temp = i - temp;
					break;
				}
		}
		if((temp>15)&&(temp<150))   //    20-200
		{
				printf("temp=%d	",temp);
				return 3000/temp;	//SpO2采样频率200Hz，FIFO采样平均过滤值为4，所以最终采样频率算为50Hz。采样一次的周期是0.02s，temp是一次心跳过程中的采样点个数，一次心跳就是temp/50秒,一分钟心跳数就是60/(temp/50)。
		}
		else
		{
				printf("temp=%d	",temp);
				return 0;
		}
}

float max30102_getSpO2(float *ir_input_data,float *red_input_data,uint16_t cache_nums)
{
			float ir_max=*ir_input_data,ir_min=*ir_input_data;
			float red_max=*red_input_data,red_min=*red_input_data;
			float R;
			uint16_t i;
			for(i=1;i<cache_nums;i++)
			{
				if(ir_max<*(ir_input_data+i))
				{
					ir_max=*(ir_input_data+i);
				}
				if(ir_min>*(ir_input_data+i))
				{
					ir_min=*(ir_input_data+i);
				}
				if(red_max<*(red_input_data+i))
				{
					red_max=*(red_input_data+i);
				}
				if(red_min>*(red_input_data+i))
				{
					red_min=*(red_input_data+i);
				}
			}
			R=((ir_max-ir_min)*red_min)/((red_max-red_min)*ir_min);
//			 R=((ir_max+ir_min)*(red_max-red_min))/((red_max+red_min)*(ir_max-ir_min));
			printf("R=%f	",R);
			return ((-45.060)*R*R + 30.354*R + 94.845);
}



int max_init(void){

		twi_master_init();
		printf("30102 init finished\r\n");
		vTaskDelay(pdMS_TO_TICKS(3000)); 

		int_pin_config();
	  vTaskDelay(pdMS_TO_TICKS(1000)); 

		max30102_init();	
		vTaskDelay(pdMS_TO_TICKS(1000)); 
	
		max30102_fir_init();
	
		printf("max_init finished\r\n");
		vTaskDelay(pdMS_TO_TICKS(10)); 
	
	  return 1;
}

void max_low_power(void){
		max30102_i2c_write(LED1_PULSE_AMPLITUDE,0x00);	//  RED LED current
		max30102_i2c_write(LED2_PULSE_AMPLITUDE,0x00); //   IR LED
		max30102_i2c_write(MODE_CONFIGURATION,0x80);
		printf("max_low_power\r\n");
		vTaskDelay(pdMS_TO_TICKS(1000)); 
}

void max_wake_up(void){
		max30102_i2c_write(LED1_PULSE_AMPLITUDE,0x3f);	//  RED LED current
		max30102_i2c_write(LED2_PULSE_AMPLITUDE,0x3f); //   IR LED
		max30102_i2c_write(MODE_CONFIGURATION,0x03);
		vTaskDelay(pdMS_TO_TICKS(1000)); 
}


extern SemaphoreHandle_t max_wake_up_sem;
void max30102_wake_up_thread(void * pvParameter){
	
		while(flag_run == 0){  				vTaskDelay(pdMS_TO_TICKS(10));   }
		
		printf("flag_run == 1 ,max30102 begin wake up\r\n");
		vTaskDelay(pdMS_TO_TICKS(2000));	
		
		while(1){
			  xSemaphoreTake(max_wake_up_sem,portMAX_DELAY);
				printf("max30102 preparing....\r\n");
				vTaskDelay(pdMS_TO_TICKS(1000));
//				max_init();	
//				vTaskDelay(pdMS_TO_TICKS(5000));
			
				max30102_i2c_write(LED1_PULSE_AMPLITUDE,0x3f);	//  RED LED current
				max30102_i2c_write(LED2_PULSE_AMPLITUDE,0x3f); //   IR LED
				max30102_i2c_write(MODE_CONFIGURATION,0x03);
//max30102_i2c_read(INTERRUPT_STATUS1,&status_data,1);
			
				printf("max30102 wake up\r\n");
				vTaskDelay(pdMS_TO_TICKS(1000));
			
			spo2_wake_up=1;
				
						
		}
}




int32_t max_read_data(float ppg_data_cache_IR[],float ppg_data_cache_RED[],uint32_t datasize){   //max_read_data(ppg_data_cache_IR, float ppg_data_cache_RED,600);
	
		uint16_t cache_counter = 0;  //缓存计数器
		uint8_t status_data = 0;
		uint32_t timeout = 0;
		uint32_t MAX_TIMEOUT = 1000;
	
		while(cache_counter<datasize){
			
				if(timeout++ > MAX_TIMEOUT) {
				
						printf("max30102 data timeout\r\n");
						vTaskDelay(pdMS_TO_TICKS(5));	
						return 0;
				}
			
				if(max30102_int_flag){
						
					
						max30102_i2c_read(INTERRUPT_STATUS1,&status_data,1);
						max30102_i2c_read(INTERRUPT_STATUS2,&status_data,1);  //clear status		
					
					max30102_int_flag = 0;
					
						max30102_fifo_read(max30102_data);
					
						ir_max30102_fir(&max30102_data[0],&fir_output[0]);//实测ir数据采集在前面，red数据在后面
            red_max30102_fir(&max30102_data[1],&fir_output[1]);  //滤波
					
						if((max30102_data[0]>PPG_DATA_THRESHOLD)&&(max30102_data[1]>PPG_DATA_THRESHOLD))  //大于阈值，说明传感器有接触
						{
                ppg_data_cache_IR[cache_counter]=fir_output[0];
                ppg_data_cache_RED[cache_counter]=fir_output[1];					
						
								cache_counter++;
								timeout=0;				

							printf("saving:%f %f !\r\n",max30102_data[0],max30102_data[1]);
							vTaskDelay(pdMS_TO_TICKS(1));	
						}else{
							printf("no touch:%f %f !\r\n",max30102_data[0],max30102_data[1]);
							vTaskDelay(pdMS_TO_TICKS(1));	
							cache_counter=0;
						}
				}
				else if(!max30102_int_flag){
					printf("max30102_int_flag==0\r\n");
						vTaskDelay(pdMS_TO_TICKS(5));	
				}
				
				vTaskDelay(pdMS_TO_TICKS(10));		
		}
		return 1;
}





#endif





