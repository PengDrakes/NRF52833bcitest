#ifndef __SPO2_H
#define __SPO2_H

#include "sdk_errors.h"

#define MASTER_TWI_INST     0  
#define TWI_SCL_M 9 //13 
#define TWI_SDA_M 38 //24 
#define INT_PIN 36 //22


#define INTERRUPT_STATUS1 0X00
#define INTERRUPT_STATUS2 0X01
#define INTERRUPT_ENABLE1 0X02
#define INTERRUPT_ENABLE2 0X03

#define FIFO_WR_POINTER 0X04
#define FIFO_OV_COUNTER 0X05
#define FIFO_RD_POINTER 0X06
#define FIFO_DATA_REG 0X07

#define FIFO_CONFIGURATION 0X08
#define MODE_CONFIGURATION 0X09
#define SPO2_CONFIGURATION 0X0A
#define LED1_PULSE_AMPLITUDE 0X0C
#define LED2_PULSE_AMPLITUDE 0X0D

#define MULTILED1_MODE 0X11
#define MULTILED2_MODE 0X12

#define TEMPERATURE_INTEGER 0X1F
#define TEMPERATURE_FRACTION 0X20
#define TEMPERATURE_CONFIG 0X21

#define VERSION_ID 0XFE
#define PART_ID 0XFF



static ret_code_t twi_master_init(void);
void spo2_read(void * pvParameter);




//static ret_code_t twi_master_init(void);
//void int_pin_config(void);
//ret_code_t max30102_i2c_write(uint8_t reg, uint8_t value);
//ret_code_t max30102_i2c_read(uint8_t reg, uint8_t * p_value,uint8_t length);
//void max30102_fifo_read(float *output_data);
//void max30102_init(void);
//uint16_t max30102_getHeartRate(float *input_data,uint16_t cache_nums);
//float max30102_getSpO2(float *ir_input_data,float *red_input_data,uint16_t cache_nums);
//int max_init(void);
//void max_low_power(void);
//void max_wake_up(void);
//void max30102_wake_up_thread(void * pvParameter);
//int32_t max_read_data(float ppg_data_cache_IR[],float ppg_data_cache_RED[],uint32_t datasize);





#endif