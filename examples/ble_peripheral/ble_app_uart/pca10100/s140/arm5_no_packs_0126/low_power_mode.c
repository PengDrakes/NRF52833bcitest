//#include "low_power_mode.h"
//#include "QMI8658C.h"
//#include "semphr.h"
//#include "rtc.h"
//#include "spo2.h"
//#include "info_collect.h"

//extern int flag_run;
//extern 		sensor_data_t data;

//SemaphoreHandle_t enter_low_power_sem;
//SemaphoreHandle_t wake_up_sem;
//SemaphoreHandle_t max_wake_up_sem;

//int	flag=0;

//void enter_low_power_thread(void * pvParameter){
//	int ret =0;
//	uint8_t time[7];
//	flag=0;
//	
////初始化 1292 加速度 rtc 血氧  初始化蓝牙
//	


//	
//	rtc_init();
//	vTaskDelay(pdMS_TO_TICKS(1000));
//	
//				 ret = rtc_read_time(0x00,time,7);  			 
//				 if(ret == 1){
//						 data.year = BCDToInt(time[6]);
//						 data.month = BCDToInt(time[5]);
//						 data.day = BCDToInt(time[4]);
//						 data.weekday = BCDToInt(time[3]);
//						 data.hour = BCDToInt(time[2]);
//						 data.min = BCDToInt(time[1]);
//						 data.second = BCDToInt(time[0]&0x7f);
//						 printf("time : 20%d年 %d月 %d日 周%d %d时 %d分 %d秒\r\n", data.year,data.month,data.day,data.weekday,data.hour,data.min,data.second);				 
//				 }
//				 vTaskDelay(pdMS_TO_TICKS(1000));

//	QMI8658C_init();
//	vTaskDelay(pdMS_TO_TICKS(2000)); 

//	max_init();
//	vTaskDelay(pdMS_TO_TICKS(2000)); 
//				 
//	wake_up_sem = xSemaphoreCreateBinary();
//	ASSERT(NULL != wake_up_sem);
//				 
//	max_wake_up_sem = xSemaphoreCreateBinary();
//	ASSERT(NULL != max_wake_up_sem);
//	
//	enter_low_power_sem = xSemaphoreCreateBinary();
//	ASSERT(NULL != enter_low_power_sem);
//	
//	flag=1;
//	
//	while(1){	
//		//设置1292进入低功耗模式 -> 加速度计进入低功耗 -> 血氧进入低功耗 
//		  QMI8658C_low_power();
//	    vTaskDelay(pdMS_TO_TICKS(100));
//			max_low_power();
//			vTaskDelay(pdMS_TO_TICKS(100));
//			
//			flag_run=0;
//			
//			printf("qmi start sleep----\r\n");
//		
//		//等待进入低功耗的信号量		
//		xSemaphoreTake(enter_low_power_sem, portMAX_DELAY);
//		
//		printf("sleeping-----------------------------\r\n");
//	  vTaskDelay(pdMS_TO_TICKS(100));
//	}
//}