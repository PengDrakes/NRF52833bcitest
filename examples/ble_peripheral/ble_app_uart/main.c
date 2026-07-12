#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include <stdint.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ads1292.h"
#include "fir_48.h"
#include "midfilt.h"
#include "bpm.h"
#include "ecg.h"
#include "spo2.h"
#include "bluetooth.h"
#include "QMI8658C.h"
#include "rtc.h"
#include "info_collect.h"
#include "low_power_mode.h"
#include "semphr.h"

#if 1

#define BLE_TASK_STACK_WORDS       384U
#define BLE_SEND_STACK_WORDS       256U
#define SPO2_TASK_STACK_WORDS      384U
#define QMI_TASK_STACK_WORDS       256U
#define ADS_TASK_STACK_WORDS       384U


SemaphoreHandle_t print_mutex = NULL;

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    printf("FATAL stack overflow: %s\r\n", task_name != NULL ? task_name : "unknown");
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

void vApplicationMallocFailedHook(void)
{
    printf("FATAL FreeRTOS malloc failed, free=%lu\r\n",
           (unsigned long)xPortGetFreeHeapSize());
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}


int main (void){
		uart_init();

		for (uint8_t i = 0; i < 5; i++)
		{
				printf("hello world\r\n");
				nrf_delay_ms(500);
		}

		printf("enter int main \r\n");
	
	print_mutex = xSemaphoreCreateMutex();
		
		if (print_mutex == NULL)
    {
        printf("print_mutex create failed\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    /*
     * 1. ???????
     * ????? BLE?GATT?NUS????????
     */
    BaseType_t err_ble = xTaskCreate(
            ble_thread,
            "ble_thread",
            BLE_TASK_STACK_WORDS,
            NULL,
            9,
            NULL
    );

    if (err_ble != pdPASS)
    {
        printf("\r\nBLE task not created.\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    else
    {
        printf("\r\nBLE task created.\r\n");
    }

    /*
     * 2. ????????
     * ??????? NUS ??????
     */
    BaseType_t err_ble_send = xTaskCreate(
            ble_send_thread,
            "ble_send_thread",
            BLE_SEND_STACK_WORDS,
            NULL,
            6,
            NULL
    );

    if (err_ble_send != pdPASS)
    {
        printf("\r\nBLE send task not created.\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    else
    {
        printf("\r\nBLE send task created.\r\n");
    }

//    BaseType_t err_spo2 = xTaskCreate(
//            spo2_read,
//            "spo2_thread",
//            SPO2_TASK_STACK_WORDS,
//            NULL,
//            4,
//            NULL
//    );

//    if (err_spo2 != pdPASS)
//    {
//        printf("\r\nSpo2 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }
//    else
//    {
//        printf("\r\nSpo2 task created.\r\n");
//    }



//		BaseType_t err5 = xTaskCreate(
//					ble_thread,           
//					"ble_thread",          
//				  256,               
//					NULL,             
//					9,                 
//					NULL               
//    );
//    if (err5 != pdPASS){
//        printf("\r\nSpo2 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	
		


	
//		BaseType_t err1 = xTaskCreate(
//					spi_read,          
//					"MyTask",         
//					256,               
//					NULL,            
//					8,                
//					NULL             
//    );
//    if (err1 != pdPASS){
//        printf("\r\nSpi task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }
//		
//	  BaseType_t err2 = xTaskCreate(
//					ecg_filiter,           
//					"ecg_filiter",          
//				  256,               // 栈深度（字）
//					NULL,              // 参数
//					7,                 // 优先级（大优先级高）
//					NULL               // 句柄
//    );
//    if (err2 != pdPASS){
//        printf("\r\nFiliter task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }
		
	BaseType_t err_qmi = xTaskCreate(
				qmi8658c_read,
				"qmi8658c_thread",
				QMI_TASK_STACK_WORDS,
				NULL,
				5,
				NULL
	);
	if (err_qmi != pdPASS)
	{
			printf("\r\nQMI task not created.\r\n");
			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
	
	BaseType_t err_ads = xTaskCreate(
        spi_read,
        "ads1292_thread",
        ADS_TASK_STACK_WORDS,
        NULL,
        8,
        NULL
	);

	if (err_ads != pdPASS)
	{
    printf("\r\nADS1292 task not created.\r\n");
    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
	}
	else
	{
    printf("\r\nADS1292 task created.\r\n");
	}

//		BaseType_t err6 = xTaskCreate(
//					ble_send_thread,           
//					"ble_send_thread",          
//				  256,               
//					NULL,             
//					6,                 
//					NULL               
//    );
//    if (err6 != pdPASS){
//        printf("\r\nerr6 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	



//		BaseType_t err10 = xTaskCreate(
//					ADS_Cap_count_process,           
//					"ADS_Cap_count_process",          
//				  256,               
//					NULL,             
//					4,                 
//					NULL               
//    );
//    if (err10 != pdPASS){
//        printf("\r\nerr10 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	

//		
//		BaseType_t err3 = xTaskCreate(
//					breath_thread,           
//					"breath_thread",          
//				  512,               
//					NULL,             
//					5,                 
//					NULL               
//    );
//    if (err3 != pdPASS){
//        printf("\r\nBreath task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	
//		else{
//		    printf("\r\nBreath task created.\r\n");
//		}
		
		
//		BaseType_t err4 = xTaskCreate(
//					spo2_read,           
//					"spo2_thread",          
//				  256,               
//					NULL,             
//					4,                 
//					NULL               
//    );
//    if (err4 != pdPASS){
//        printf("\r\nSpo2 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	
//		

		


		

		
		
//		BaseType_t err8 = xTaskCreate(
//					rtc_thread,           
//					"rtc_thread",          
//				  256,               
//					NULL,             
//					4,                 
//					NULL               
//    );
//    if (err8 != pdPASS){
//        printf("\r\nerr8 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	



//		BaseType_t err9 = xTaskCreate(
//					low_power_thread,           
//					"low_power_thread",          
//				  256,               
//					NULL,             
//					6,                 
//					NULL               
//    );
//    if (err9 != pdPASS){
//        printf("\r\nerr9 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }



		
		printf("Free heap before scheduler: %lu bytes\r\n",
		       (unsigned long)xPortGetFreeHeapSize());
		vTaskStartScheduler();

		printf("FreeRTOS scheduler failed to start\r\n");
		for (;;)
		{
			/* vTaskStartScheduler should never return. */
		}
}


#endif


#if 0

QueueHandle_t sensor_data_queue = NULL;
static TaskHandle_t ble_send_task_handle = NULL;
int main (void){
		uart_init();
	
	
		printf("enter int main \r\n");
	
	  sensor_data_queue = xQueueCreate(10, sizeof(sensor_data_t));
    configASSERT(sensor_data_queue != NULL);
	
		BaseType_t err1 = xTaskCreate(
					ble_thread,           
					"ble_thread",          
				  256,               
					NULL,             
					20,                 
					NULL               
    );
    if (err1 != pdPASS){
        printf("\r\nerr1 task not created.\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }	
		
//low power		
		BaseType_t err2 = xTaskCreate(
					enter_low_power_thread,          
					"enter_low_power_thread",         
					256,               
					NULL,            
					11,                
					NULL             
    );
    if (err2 != pdPASS){
        printf("\r\nerr2 task not created.\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }	
	
//wakeup
		BaseType_t err3 = xTaskCreate(
					QMI8658C_wake_up_thread,          
					"QMI8658C_wake_up_thread",         
					256,               
					NULL,            
					10,                
					NULL             
    );
    if (err3 != pdPASS){
        printf("\r\nerr3 task not created.\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		
		BaseType_t err7 = xTaskCreate(
					max30102_wake_up_thread,          
					"max30102_wake_up_thread",         
					256,               
					NULL,            
					9,                
					NULL             
    );
    if (err7 != pdPASS){
        printf("\r\nerr7 task not created.\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
	
//read data 	
		BaseType_t err4 = xTaskCreate(
					collect_info_thread,          
					"collect_info_thread",         
					256,               
					NULL,            
					6,                
					NULL             
    );
    if (err4 != pdPASS){
        printf("\r\nerr4 task not created.\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
		
		
//		BaseType_t err5 = xTaskCreate(
//					ble_send_thread,           
//					"ble_send_thread",          
//				  256,               
//					NULL,             
//					8,                 
//					NULL               
//    );
//    if (err5 != pdPASS){
//        printf("\r\nerr5 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	
//		
		
		
		
		
		
		
//	BaseType_t err6 = xTaskCreate(
//				qmi8658c_read,           
//				"qmi8658c_thread",          
//				256,               
//				NULL,             
//				5,                 
//				NULL               
//	);
//	if (err6 != pdPASS){
//			printf("\r\nerr6 task not created.\r\n");
//			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//	}		
	
		vTaskStartScheduler();
}



#endif









		
//		BaseType_t err6 = xTaskCreate(
//					ble_send_thread,           
//					"ble_send_thread",          
//				  256,               
//					NULL,             
//					6,                 
//					NULL               
//    );
//    if (err6 != pdPASS){
//        printf("\r\nerr6 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	

		

//		BaseType_t err1 = xTaskCreate(
//					spi_read,          
//					"MyTask",         
//					256,               
//					NULL,            
//					8,                
//					NULL             
//    );
//    if (err1 != pdPASS){
//        printf("\r\nSpi task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }
//		
//	  BaseType_t err2 = xTaskCreate(
//					ecg_filiter,           
//					"ecg_filiter",          
//				  256,               // 栈深度（字）
//					NULL,              // 参数
//					7,                 // 优先级（大优先级高）
//					NULL               // 句柄
//    );
//    if (err2 != pdPASS){
//        printf("\r\nFiliter task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }
	
	

//		BaseType_t err10 = xTaskCreate(
//					ADS_Cap_count_process,           
//					"ADS_Cap_count_process",          
//				  256,               
//					NULL,             
//					4,                 
//					NULL               
//    );
//    if (err10 != pdPASS){
//        printf("\r\nerr10 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	

		
//		BaseType_t err3 = xTaskCreate(
//					breath_thread,           
//					"breath_thread",          
//				  512,               
//					NULL,             
//					5,                 
//					NULL               
//    );
//    if (err3 != pdPASS){
//        printf("\r\nBreath task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	
//		else{
//		    printf("\r\nBreath task created.\r\n");
//		}
		
		
//		BaseType_t err4 = xTaskCreate(
//					spo2_read,           
//					"spo2_thread",          
//				  256,               
//					NULL,             
//					4,                 
//					NULL               
//    );
//    if (err4 != pdPASS){
//        printf("\r\nSpo2 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	
//		



		
//	BaseType_t err7 = xTaskCreate(
//				qmi8658c_read,           
//				"qmi8658c_thread",          
//				256,               
//				NULL,             
//				5,                 
//				NULL               
//	);
//	if (err7 != pdPASS){
//			printf("\r\nerr7 task not created.\r\n");
//			APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//	}			
		
		
//		BaseType_t err8 = xTaskCreate(
//					rtc_thread,           
//					"rtc_thread",          
//				  256,               
//					NULL,             
//					4,                 
//					NULL               
//    );
//    if (err8 != pdPASS){
//        printf("\r\nerr8 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }	



//		BaseType_t err9 = xTaskCreate(
//					low_power_thread,           
//					"low_power_thread",          
//				  256,               
//					NULL,             
//					6,                 
//					NULL               
//    );
//    if (err9 != pdPASS){
//        printf("\r\nerr9 task not created.\r\n");
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }
