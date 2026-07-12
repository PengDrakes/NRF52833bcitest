//#include <string.h>
//#include <stdint.h>
//#include <stdbool.h>
//#include "FreeRTOS.h"
//#include "semphr.h"
//#include "timers.h"
//#include "bsp.h"
//#include "nordic_common.h"
//#include "nrf_drv_clock.h"
//#include "sdk_errors.h"
//#include "app_error.h"

//#include "nrf_drv_rtc.h"
//#include "nrfx_rtc.h"

//#include "low_power.h"
//#include "ads1292.h"


//#define RTC_CC 0
//#define RTC_TICKS   (RTC_US_TO_TICKS(500000ULL*60, RTC_DEFAULT_CONFIG_FREQUENCY))

//static SemaphoreHandle_t enter_stop_mode_sem;

//int  enter_stop_mode_flag=0;
//int  chip_wake_up_flag=0;
//static volatile int  rtc_num=0;
//const nrf_drv_rtc_t  rtc = NRF_DRV_RTC_INSTANCE(RTC_CC);



//static void rtc_handler(nrf_drv_rtc_int_type_t int_type){
//		chip_wake_up_flag=1;
//		enter_stop_mode_flag=0;
//	  ret_code_t err_code;
//	  BaseType_t yield_req = pdFALSE;
//	
//		ads_wakeup();	
//		printf("wake up\r\n");
//	
//	  err_code = nrf_drv_rtc_cc_set(
//        &rtc,
//        RTC_CC,
//        (nrf_rtc_cc_get(rtc.p_reg, RTC_CC) + RTC_TICKS) & RTC_COUNTER_COUNTER_Msk,
//        true);
//    APP_ERROR_CHECK(err_code);
//		
//   UNUSED_VARIABLE(xSemaphoreGiveFromISR(enter_stop_mode_sem, &yield_req));
//   portYIELD_FROM_ISR(yield_req);		

//}
//	

//void low_power_thread(void * pvParameter){
//    ret_code_t err_code;
//	
//		printf("low_power_thread\r\n");
//		nrf_delay_ms(5);
//	
//    err_code = nrf_drv_clock_init();
//    APP_ERROR_CHECK(err_code);
//	
//	  nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
//		err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
//    APP_ERROR_CHECK(err_code);
//	
//	
//    //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
//    err_code = nrf_drv_rtc_cc_set(&rtc,0,RTC_TICKS,true);
//    APP_ERROR_CHECK(err_code);

//    nrf_drv_rtc_enable(&rtc);	
//	
//    enter_stop_mode_sem = xSemaphoreCreateBinary();
//    ASSERT(NULL != enter_stop_mode_sem);
//		
//		while(1){			
//			
//			ads_standby();	
//			printf("stop mode\r\n");
//			nrf_delay_ms(5);
//			
//			enter_stop_mode_flag=1;
//			chip_wake_up_flag=0;		
//			
//			UNUSED_RETURN_VALUE(xSemaphoreTake(enter_stop_mode_sem, portMAX_DELAY));
//			
//		}
//}

