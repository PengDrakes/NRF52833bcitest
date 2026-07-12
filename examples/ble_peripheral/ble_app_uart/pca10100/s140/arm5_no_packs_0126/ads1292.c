#if 1
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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ads1292.h"
#include "arm_math.h"
#include "bluetooth.h"
#include "nrf_drv_spi.h"
#include "semphr.h"




//(kaifaban)13 22 24 32 34 36 38           //25 31 3 28 2 37 29
#define RESET_PIN  25
#define DRDY_PIN 31
#define START_PIN 3
#define CS_PIN    28 

#define SCK_PIN    2          
#define MOSI_PIN    37       
#define MISO_PIN    29    

uint8_t ads1292_data[9] = {0};		
uint8_t Rx_ID[3]={0};
uint8_t Rx_config1[3]={0};
uint8_t Rx_config2[3]={0};
uint8_t Rx_CH1SET[3]={0};
uint8_t Rx_CH2SET[3]={0};
uint8_t Rx_LOFF[3]={0};
uint8_t Rx_RLDSENS[3]={0};
uint8_t Rx_LOFFSENS[3]={0};
uint8_t Rx_LOFFSTAT[3]={0};
uint8_t Rx_RESP1[3]={0};
uint8_t Rx_RESP2[3]={0};
uint8_t Rx_GPIO[3]={0};
uint8_t cmd[9] = {0x00};


#define CYC_ARRAY_LEN 2000
int32_t  save_data[CYC_ARRAY_LEN]={0};	
uint32_t  write_pos=0;  //当前存储数据的位置
uint32_t  read_pos=0; //当前分析数据的位置

int32_t  save_data_0[CYC_ARRAY_LEN]={0};	
uint32_t  write_pos_0=0;  //ch0当前存储数据的位置
uint32_t  read_pos_0=0; //当前分析数据的位置

int32_t  save_data_1[CYC_ARRAY_LEN]={0};	
uint32_t  write_pos_1=0;  //ch1当前存储数据的位置
uint32_t  read_pos_1=0; //当前分析数据的位置

//#define INIT_SAMPLE_COUNT 2000
//#define INIT_SAMPLE_START 2501
//#define INIT_SAMPLE_END 4500
//#define INIT_SAMPLE_COUNT 4000
//#define INIT_SAMPLE_START 3751 
//#define INIT_SAMPLE_END 7750
#define INIT_SAMPLE_COUNT 1000
#define INIT_SAMPLE_START 1
#define INIT_SAMPLE_END 1000
uint16_t init_index = 0;

volatile uint8_t init_data_collected = 0;
static uint32_t sample_count = 0;


int32_t chn_adc_value[2];
float ch_v[2];
float vol_v;
int32_t vol_value = 0;
int32_t vol_uv = 0;

float32_t data1[INIT_SAMPLE_COUNT],data2[INIT_SAMPLE_COUNT];

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */


/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
   
}



void ads1292_init(void){

		enum cmd_pos{
					ADS1292_WAKEUP_CMD,
					ADS1292_STANDBY_CMD,
					ADS1292_RESET_CMD,
					ADS1292_START_CMD,
					ADS1292_STOP_CMD,
					ADS1292_OFFSETCAL_CMD,
					ADS1292_RDATAC_CMD,
					ADS1292_SDATAC_CMD,
					ADS1292_RDATA_CMD,
					ADS1292_RREG_CMD,
					ADS1292_WREG_CMD
				};
		uint8_t cmd_list[]={0x02,0x04,0x06,0x08,0x0A,0X1A,0x10,0x11,0x12,0x20,0x40};
		
    uint8_t read_ID_cmd[]={0x20,0x00,0x00};
		uint8_t write_config1_cmd[]={0x41,0x00,0x01};
    uint8_t write_config2_cmd[]={0x42,0x00,0xa0}; 
		uint8_t write_loff_cmd[]={0x43,0x00,0x10};	
		uint8_t write_ch1set_cmd[]={0x44,0x00,0x10};
		uint8_t write_ch2set_cmd[]={0x45,0x00,0x10};
		uint8_t write_rldsens_cmd[]={0x46,0x00,0xef};
		uint8_t write_loffsens_cmd[]={0x47,0x00,0x00};
		uint8_t write_loffstat_cmd[]={0x48,0x00,0x00};
    uint8_t write_resp1_cmd[]={0x49,0x00,0xe2};
    uint8_t write_resp2_cmd[]={0x4A,0x00,0x03};
		uint8_t write_gpio_cmd[]={0x4B,0x00,0x0C};

		uint8_t read_config1_cmd[]={0x21,0x00,0x00};
		uint8_t read_config2_cmd[]={0x22,0x00,0x00};
		uint8_t read_loff_cmd[]={0x23,0x00,0x00};
		uint8_t read_ch1set_cmd[]={0x24,0x00,0x00};
    uint8_t read_ch2set_cmd[]={0x25,0x00,0x00};
		uint8_t read_rldsens_cmd[]={0x26,0x00,0x00};
		uint8_t read_loffsens_cmd[]={0x27,0x00,0x00};
		uint8_t read_loffstat_cmd[]={0x28,0x00,0x00};
		uint8_t read_resp1_cmd[]={0x29,0x00,0x00};
		uint8_t read_resp2_cmd[]={0x2A,0x00,0x00};
		uint8_t read_gpio_cmd[]={0x2B,0x00,0x00};
		
		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_RESET_CMD, 1, NULL, 0);
    nrf_delay_ms(500); 

		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_SDATAC_CMD, 1, NULL, 0);
    nrf_delay_ms(2000);   

    //ID
		nrf_drv_spi_transfer(&spi, read_ID_cmd, 3, Rx_ID, 3);
		nrf_delay_ms(500);
    printf("ID value : 0x%x\r\n",Rx_ID[2]);	
    nrf_delay_ms(500);
    //CONFIG1
		nrf_drv_spi_transfer(&spi, write_config1_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, read_config1_cmd, 3, Rx_config1, 3);
		nrf_delay_ms(500);
    printf("CONFIG1 value : 0x%x %x\r\n",write_config1_cmd[2],Rx_config1[2]);	
    nrf_delay_ms(500); 
    //CONFIG2
		nrf_drv_spi_transfer(&spi, write_config2_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, read_config2_cmd, 3, Rx_config2, 3);
		nrf_delay_ms(500);
    printf("CONFIG2 value : 0x%x %x\r\n",write_config2_cmd[2],Rx_config2[2]);	
    nrf_delay_ms(500);  
    //CH1SET
		nrf_drv_spi_transfer(&spi, write_ch1set_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, read_ch1set_cmd, 3, Rx_CH1SET, 3);
		nrf_delay_ms(500);
    printf("CH1SET value : 0x%x %x\r\n",write_ch1set_cmd[2],Rx_CH1SET[2]);	
    nrf_delay_ms(500);   
    //CH2SET
		nrf_drv_spi_transfer(&spi, write_ch2set_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, read_ch2set_cmd, 3, Rx_CH2SET, 3);
		nrf_delay_ms(500);
    printf("CH2SET value : 0x%x %x\r\n",write_ch2set_cmd[2],Rx_CH2SET[2]);	
    nrf_delay_ms(500);
    //ELSE
		nrf_drv_spi_transfer(&spi, write_loff_cmd, 3, NULL, 0);  
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_rldsens_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_loffsens_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_loffstat_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_resp1_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_resp2_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_gpio_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		
		

		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_START_CMD, 1, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_RDATAC_CMD, 1, NULL, 0);
		nrf_delay_ms(5000);


}

float32_t max_init_val;
uint32_t maxIndex;
float32_t min_init_val;
uint32_t minIndex;
void ADS1292_val_init(float32_t *data,float32_t *a,float32_t *b)
{
	float32_t *data_cache,*a1,*b1;

	data_cache = data;

	a1 = a;

	b1 = b;

	arm_max_f32(data_cache,INIT_SAMPLE_COUNT,&max_init_val,&maxIndex);

	arm_min_f32(data_cache,INIT_SAMPLE_COUNT,&min_init_val,&minIndex);

	*a1 = 180.0/(max_init_val-min_init_val);

	*b1 = 220-(*a1)*max_init_val;
}

void bre_val_init(float32_t *data,float32_t *a,float32_t *b)
{
	float32_t *data_cache,*a1,*b1;

	data_cache = data;

	a1 = a;

	b1 = b;

	arm_max_f32(data_cache,INIT_SAMPLE_COUNT,&max_init_val,&maxIndex);

	arm_min_f32(data_cache,INIT_SAMPLE_COUNT,&min_init_val,&minIndex);

	*a1 = 2.0f/(max_init_val-min_init_val);

	*b1 = 1.0f-(*a1)*max_init_val;
}




void device_init(void){
	
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
	  nrf_gpio_cfg_output(CS_PIN);  //cs
		nrf_gpio_pin_set(CS_PIN); 
		nrf_delay_ms(100);


	  //开发板  34  36  38  32   //2  37  29  28  
    nrf_drv_spi_config_t spi_config = {                                                         
						.sck_pin      = SCK_PIN,                             
						.mosi_pin     = MOSI_PIN,                           
						.miso_pin     = MISO_PIN,                              
						.ss_pin       = CS_PIN,                               
						.irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,         
						.orc          = 0xFF,                                   
						.frequency    = NRF_SPI_FREQ_250K,                     
						.mode         = NRF_DRV_SPI_MODE_1,                      
						.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         
				};
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
				

		nrf_gpio_cfg_output(RESET_PIN);  //reset
		nrf_gpio_pin_clear(RESET_PIN); 
		nrf_delay_ms(1000); 
		nrf_gpio_pin_set(RESET_PIN);  
		nrf_delay_ms(1000); 
	
		nrf_gpio_cfg_output(START_PIN);  //start
		nrf_gpio_pin_clear(START_PIN); 
		nrf_delay_ms(1000);
				
}


extern SemaphoreHandle_t print_mutex;


volatile int intr_flag ;
static void drdy_isr_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin == DRDY_PIN)
    {
        if (action == NRF_GPIOTE_POLARITY_HITOLO)  
        {
            intr_flag = 1;
        }
    }
}
void spi_read (void * pvParameter){   
    uint32_t adc_raw_value = 0;
    int timeout = 0;

    int ret = 0;
		
	  device_init();
		nrf_delay_ms(1000);
	
		ads1292_init();

		APP_ERROR_CHECK(nrfx_gpiote_init());   
		nrf_gpio_cfg_input(DRDY_PIN, NRF_GPIO_PIN_NOPULL);  //drdy   //NRF_GPIO_PIN_PULLUP
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true); //下降沿触发
		in_config.pull = NRF_GPIO_PIN_NOPULL;
    APP_ERROR_CHECK(nrfx_gpiote_in_init(DRDY_PIN, &in_config, drdy_isr_handler));
    nrfx_gpiote_in_event_enable(DRDY_PIN, true);

		while(1){
			
			
				timeout=0;
				while(intr_flag == 0){
            nrf_delay_ms(1);
            if (timeout++ > 3000) {
                printf("\r\nDRDY timeout!\r\n"); 
                timeout = 0;
                ads1292_init();
            }    
        }

				nrf_drv_spi_transfer(&spi, cmd, 9, ads1292_data, 9);	
				intr_flag=0;

        if(ads1292_data[0]!=0xC0){
           // printf("\r\n-----INVALID STATUS,%x %x %x %x %x %x %x %x %x-----\r\n",ads1292_data[0],ads1292_data[1],ads1292_data[2],ads1292_data[3],ads1292_data[4],ads1292_data[5],ads1292_data[6],ads1292_data[7],ads1292_data[8]);
						nrf_delay_ms(1);	
				}else{
						sample_count++;
            for(int i=0;i<2;i++){
                adc_raw_value = ((uint32_t)ads1292_data[3+3*i] << 16) | ((uint32_t)ads1292_data[3+3*i+1] << 8) | ads1292_data[3+3*i+2];
                if(adc_raw_value < 0x800000){
                    chn_adc_value[i] = adc_raw_value; 
                    ch_v[i] = 2.42 * chn_adc_value[i]/8388607.0;
                    vol_value = (int32_t)(ch_v[i]*1000000);
//									  if(i==1){printf("%d\r\n",vol_value);}
									
//										if (xSemaphoreTake(print_mutex, portMAX_DELAY) == pdTRUE) {
//												 if(i==1){printf("%d\r\n",vol_value);}
//												xSemaphoreGive(print_mutex);
//										}		
                }else{
                    chn_adc_value[i] =  adc_raw_value - 16777216; 
                    ch_v[i] = 2.42 * chn_adc_value[i]/8388607.0;
                    vol_value = (int32_t)(ch_v[i]*1000000);
//									  if(i==1){printf("%d\r\n",vol_value);}
										
//										if (xSemaphoreTake(print_mutex, portMAX_DELAY) == pdTRUE) {
//												 if(i==1){printf("%d\r\n",vol_value);}
//												xSemaphoreGive(print_mutex);
//										}		
                }		
								
								if(i==0){
										if(((write_pos_0+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN) !=  read_pos_0 ){
											save_data_0[write_pos_0] = vol_value;
										  write_pos_0 = (write_pos_0+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN;		
										}
								}else{
										if(((write_pos_1+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN) !=  read_pos_1 ){
											save_data_1[write_pos_1] = vol_value;
											
											//printf("%d\r\n",save_data_1[write_pos_1]);
										  write_pos_1 = (write_pos_1+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN;		
										}
										//else{   printf("data full------------\r\n");    }
								}
					}
						
				if (!init_data_collected) {
              if (sample_count >= INIT_SAMPLE_START && sample_count <= INIT_SAMPLE_END) { 
                  if (init_index < INIT_SAMPLE_COUNT) {
											data1[init_index] = ch_v[0]*1000000*1.0;  
											data2[init_index] = ch_v[1]*1000000*1.0;     

                      init_index++;
                  }
              }else if(sample_count > INIT_SAMPLE_END){
										init_data_collected = 1;
										printf("data collceted over\r\n");
								
							}					
        }		
				
				vTaskDelay(pdMS_TO_TICKS(1));
			}
		}
}

#endif





















#if 0

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
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ads1292.h"
#include "arm_math.h"
#include "bluetooth.h"
#include "nrf_drv_spi.h"
#include "semphr.h"




//(kaifaban)13 22 24 32 34 36 38           //25 31 3 28 2 37 29
#define RESET_PIN  25
#define DRDY_PIN 31
#define START_PIN 3
#define CS_PIN    28 

#define SCK_PIN    2          
#define MOSI_PIN    37       
#define MISO_PIN    29    

uint8_t ads1292_data[9] = {0};		
uint8_t Rx_ID[3]={0};
uint8_t Rx_config1[3]={0};
uint8_t Rx_config2[3]={0};
uint8_t Rx_CH1SET[3]={0};
uint8_t Rx_CH2SET[3]={0};
uint8_t Rx_LOFF[3]={0};
uint8_t Rx_RLDSENS[3]={0};
uint8_t Rx_LOFFSENS[3]={0};
uint8_t Rx_LOFFSTAT[3]={0};
uint8_t Rx_RESP1[3]={0};
uint8_t Rx_RESP2[3]={0};
uint8_t Rx_GPIO[3]={0};
uint8_t cmd[9] = {0x00};


#define CYC_ARRAY_LEN 2000
int32_t  save_data[CYC_ARRAY_LEN]={0};	
uint32_t  write_pos=0;  //当前存储数据的位置
uint32_t  read_pos=0; //当前分析数据的位置

int32_t  save_data_0[CYC_ARRAY_LEN]={0};	
uint32_t  write_pos_0=0;  //ch0当前存储数据的位置
uint32_t  read_pos_0=0; //当前分析数据的位置

int32_t  save_data_1[CYC_ARRAY_LEN]={0};	
uint32_t  write_pos_1=0;  //ch1当前存储数据的位置
uint32_t  read_pos_1=0; //当前分析数据的位置

#define INIT_SAMPLE_COUNT 1000
#define INIT_SAMPLE_START 1
#define INIT_SAMPLE_END 1000
uint16_t init_index = 0;

volatile uint8_t init_data_collected = 0;
static uint32_t sample_count = 0;


int32_t chn_adc_value[2];
float ch_v[2];
float vol_v;
int32_t vol_value = 0;
int32_t vol_uv = 0;

float32_t data1[INIT_SAMPLE_COUNT],data2[INIT_SAMPLE_COUNT];

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */


/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
   
}


enum cmd_pos{
			ADS1292_WAKEUP_CMD,
			ADS1292_STANDBY_CMD,
			ADS1292_RESET_CMD,
			ADS1292_START_CMD,
			ADS1292_STOP_CMD,
			ADS1292_OFFSETCAL_CMD,
			ADS1292_RDATAC_CMD,
			ADS1292_SDATAC_CMD,
			ADS1292_RDATA_CMD,
			ADS1292_RREG_CMD,
			ADS1292_WREG_CMD
		};
uint8_t cmd_list[]={0x02,0x04,0x06,0x08,0x0A,0X1A,0x10,0x11,0x12,0x20,0x40};


void ads1292_init(void){
		
    uint8_t read_ID_cmd[]={0x20,0x00,0x00};
		uint8_t write_config1_cmd[]={0x41,0x00,0x01};
    uint8_t write_config2_cmd[]={0x42,0x00,0xa0}; 
		uint8_t write_loff_cmd[]={0x43,0x00,0x10};	
		uint8_t write_ch1set_cmd[]={0x44,0x00,0x10};
		uint8_t write_ch2set_cmd[]={0x45,0x00,0x10};
		uint8_t write_rldsens_cmd[]={0x46,0x00,0xef};
		uint8_t write_loffsens_cmd[]={0x47,0x00,0x00};
		uint8_t write_loffstat_cmd[]={0x48,0x00,0x00};
    uint8_t write_resp1_cmd[]={0x49,0x00,0xe2};
    uint8_t write_resp2_cmd[]={0x4A,0x00,0x03};
		uint8_t write_gpio_cmd[]={0x4B,0x00,0x0C};

		uint8_t read_config1_cmd[]={0x21,0x00,0x00};
		uint8_t read_config2_cmd[]={0x22,0x00,0x00};
		uint8_t read_loff_cmd[]={0x23,0x00,0x00};
		uint8_t read_ch1set_cmd[]={0x24,0x00,0x00};
    uint8_t read_ch2set_cmd[]={0x25,0x00,0x00};
		uint8_t read_rldsens_cmd[]={0x26,0x00,0x00};
		uint8_t read_loffsens_cmd[]={0x27,0x00,0x00};
		uint8_t read_loffstat_cmd[]={0x28,0x00,0x00};
		uint8_t read_resp1_cmd[]={0x29,0x00,0x00};
		uint8_t read_resp2_cmd[]={0x2A,0x00,0x00};
		uint8_t read_gpio_cmd[]={0x2B,0x00,0x00};
		
		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_RESET_CMD, 1, NULL, 0);
    nrf_delay_ms(500); 

		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_SDATAC_CMD, 1, NULL, 0);
    nrf_delay_ms(2000);   

    //ID
		nrf_drv_spi_transfer(&spi, read_ID_cmd, 3, Rx_ID, 3);
		nrf_delay_ms(500);
    printf("ID value : 0x%x\r\n",Rx_ID[2]);	
    nrf_delay_ms(500);
    //CONFIG1
		nrf_drv_spi_transfer(&spi, write_config1_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, read_config1_cmd, 3, Rx_config1, 3);
		nrf_delay_ms(500);
    printf("CONFIG1 value : 0x%x %x\r\n",write_config1_cmd[2],Rx_config1[2]);	
    nrf_delay_ms(500); 
    //CONFIG2
		nrf_drv_spi_transfer(&spi, write_config2_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, read_config2_cmd, 3, Rx_config2, 3);
		nrf_delay_ms(500);
    printf("CONFIG2 value : 0x%x %x\r\n",write_config2_cmd[2],Rx_config2[2]);	
    nrf_delay_ms(500);  
    //CH1SET
		nrf_drv_spi_transfer(&spi, write_ch1set_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, read_ch1set_cmd, 3, Rx_CH1SET, 3);
		nrf_delay_ms(500);
    printf("CH1SET value : 0x%x %x\r\n",write_ch1set_cmd[2],Rx_CH1SET[2]);	
    nrf_delay_ms(500);   
    //CH2SET
		nrf_drv_spi_transfer(&spi, write_ch2set_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, read_ch2set_cmd, 3, Rx_CH2SET, 3);
		nrf_delay_ms(500);
    printf("CH2SET value : 0x%x %x\r\n",write_ch2set_cmd[2],Rx_CH2SET[2]);	
    nrf_delay_ms(500);
    //ELSE
		nrf_drv_spi_transfer(&spi, write_loff_cmd, 3, NULL, 0);  
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_rldsens_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_loffsens_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_loffstat_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_resp1_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_resp2_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, write_gpio_cmd, 3, NULL, 0);
		nrf_delay_ms(1000);
		
		

		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_START_CMD, 1, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_RDATAC_CMD, 1, NULL, 0);
		nrf_delay_ms(5000);


}

float32_t max_init_val;
uint32_t maxIndex;
float32_t min_init_val;
uint32_t minIndex;
void ADS1292_val_init(float32_t *data,float32_t *a,float32_t *b)
{
	float32_t *data_cache,*a1,*b1;

	data_cache = data;

	a1 = a;

	b1 = b;

	arm_max_f32(data_cache,INIT_SAMPLE_COUNT,&max_init_val,&maxIndex);

	arm_min_f32(data_cache,INIT_SAMPLE_COUNT,&min_init_val,&minIndex);

	*a1 = 180.0/(max_init_val-min_init_val);

	*b1 = 220-(*a1)*max_init_val;
}

void bre_val_init(float32_t *data,float32_t *a,float32_t *b)
{
	float32_t *data_cache,*a1,*b1;

	data_cache = data;

	a1 = a;

	b1 = b;

	arm_max_f32(data_cache,INIT_SAMPLE_COUNT,&max_init_val,&maxIndex);

	arm_min_f32(data_cache,INIT_SAMPLE_COUNT,&min_init_val,&minIndex);

	*a1 = 2.0f/(max_init_val-min_init_val);

	*b1 = 1.0f-(*a1)*max_init_val;
}




void device_init(void){
	
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
	  nrf_gpio_cfg_output(CS_PIN);  //cs
		nrf_gpio_pin_set(CS_PIN); 
		nrf_delay_ms(100);


	  //开发板  34  36  38  32   //2  37  29  28  
    nrf_drv_spi_config_t spi_config = {                                                         
						.sck_pin      = SCK_PIN,                             
						.mosi_pin     = MOSI_PIN,                           
						.miso_pin     = MISO_PIN,                              
						.ss_pin       = CS_PIN,                               
						.irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,         
						.orc          = 0xFF,                                   
						.frequency    = NRF_SPI_FREQ_250K,                     
						.mode         = NRF_DRV_SPI_MODE_1,                      
						.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,         
				};
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
				

		nrf_gpio_cfg_output(RESET_PIN);  //reset
		nrf_gpio_pin_clear(RESET_PIN); 
		nrf_delay_ms(1000); 
		nrf_gpio_pin_set(RESET_PIN);  
		nrf_delay_ms(1000); 
	
		nrf_gpio_cfg_output(START_PIN);  //start
		nrf_gpio_pin_clear(START_PIN); 
		nrf_delay_ms(1000);
				
}





volatile int intr_flag ;
static void drdy_isr_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if (pin == DRDY_PIN)
    {
        if (action == NRF_GPIOTE_POLARITY_HITOLO)  
        {
            intr_flag = 1;
        }
    }
}
void spi_read (void * pvParameter){   
    uint32_t adc_raw_value = 0;
    int timeout = 0;

    int ret = 0;
		
	  device_init();
		nrf_delay_ms(1000);
	
		ads1292_init();

		APP_ERROR_CHECK(nrfx_gpiote_init());   
		nrf_gpio_cfg_input(DRDY_PIN, NRF_GPIO_PIN_NOPULL);  //drdy   //NRF_GPIO_PIN_PULLUP
    nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true); //下降沿触发
		in_config.pull = NRF_GPIO_PIN_NOPULL;
    APP_ERROR_CHECK(nrfx_gpiote_in_init(DRDY_PIN, &in_config, drdy_isr_handler));
    nrfx_gpiote_in_event_enable(DRDY_PIN, true);

		while(1){
			
			
				timeout=0;
				while(intr_flag == 0){
            nrf_delay_ms(1);
            if (timeout++ > 3000) {
                printf("\r\nDRDY timeout!\r\n"); 
                timeout = 0;
                ads1292_init();
            }    
        }

				nrf_drv_spi_transfer(&spi, cmd, 9, ads1292_data, 9);	
				intr_flag=0;

        if(ads1292_data[0]!=0xC0){
           // printf("\r\n-----INVALID STATUS,%x %x %x %x %x %x %x %x %x-----\r\n",ads1292_data[0],ads1292_data[1],ads1292_data[2],ads1292_data[3],ads1292_data[4],ads1292_data[5],ads1292_data[6],ads1292_data[7],ads1292_data[8]);
						nrf_delay_ms(1);	
				}else{
						sample_count++;
            for(int i=0;i<2;i++){
                adc_raw_value = ((uint32_t)ads1292_data[3+3*i] << 16) | ((uint32_t)ads1292_data[3+3*i+1] << 8) | ads1292_data[3+3*i+2];
                if(adc_raw_value < 0x800000){
                    chn_adc_value[i] = adc_raw_value; 
                    ch_v[i] = 2.42 * chn_adc_value[i]/8388607.0;
                    vol_value = (int32_t)(ch_v[i]*1000000);
//									  if(i==1){printf("%d\r\n",vol_value);}
                }else{
                    chn_adc_value[i] =  adc_raw_value - 16777216; 
                    ch_v[i] = 2.42 * chn_adc_value[i]/8388607.0;
                    vol_value = (int32_t)(ch_v[i]*1000000);
//									  if(i==1){printf("%d\r\n",vol_value);}
                }		
								
								if(i==0){
										if(((write_pos_0+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN) !=  read_pos_0 ){
											save_data_0[write_pos_0] = vol_value;
										  write_pos_0 = (write_pos_0+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN;		
										}
								}else{
										if(((write_pos_1+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN) !=  read_pos_1 ){
											save_data_1[write_pos_1] = vol_value;
											
											//printf("%d\r\n",save_data_1[write_pos_1]);
										  write_pos_1 = (write_pos_1+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN;		
										}
										//else{   printf("data full------------\r\n");    }
								}
					}
						
				if (!init_data_collected) {
              if (sample_count >= INIT_SAMPLE_START && sample_count <= INIT_SAMPLE_END) { 
                  if (init_index < INIT_SAMPLE_COUNT) {
											data1[init_index] = ch_v[0]*1000000*1.0;  
											data2[init_index] = ch_v[1]*1000000*1.0;     

                      init_index++;
                  }
              }else if(sample_count > INIT_SAMPLE_END){
										init_data_collected = 1;
										printf("data collceted over\r\n");
								
							}					
        }		
				
				vTaskDelay(pdMS_TO_TICKS(1));
			}
		}
}

void ads_standby(void){
		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_SDATAC_CMD, 1, NULL, 0);
    nrf_delay_ms(2000);   
		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_STOP_CMD, 1, NULL, 0);
    nrf_delay_ms(500);   
		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_STANDBY_CMD, 1, NULL, 0);
    nrf_delay_ms(500);  
		printf("standby_mode\r\n");	
}
void ads_wakeup(void){
		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_WAKEUP_CMD, 1, NULL, 0);
   	nrf_delay_ms(500);   
		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_START_CMD, 1, NULL, 0);
		nrf_delay_ms(1000);
		nrf_drv_spi_transfer(&spi, cmd_list+ADS1292_RDATAC_CMD, 1, NULL, 0);
		nrf_delay_ms(5000);
		printf("wakeup_mode\r\n");	
}



#endif










