//#include "info_collect.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "QMI8658C.h"
//#include "semphr.h"
//#include "rtc.h"

//extern int spo2_wake_up;
//extern int	flag_run;
//extern int 	flag;
//extern QueueHandle_t sensor_data_queue;
//sensor_data_t data;

//void collect_info_thread(void * pvParameter){
//		int ret =0;
//    uint8_t data6d[12];
//		uint8_t time[7];
//	
//		float ppg_data_cache_IR[600],ppg_data_cache_RED[600];
//	
//		while(flag == 0){
//		//等待初始化完成
//				vTaskDelay(pdMS_TO_TICKS(10));
//		}
//					
//		while(1){
//			
//			 while(flag_run == 1){
//				 
//				  vTaskDelay(pdMS_TO_TICKS(3000));
//				 
//				 //调用采集函数，直接采集加速度->直接采集血氧->直接采集1292->采集外部RTC的时间年月日时分秒
//				 //蓝牙发送整个数据包
//				 
////				 ret = rtc_read_time(0x00,time,7);  		 
////				 if(ret == 1){
////						 data.year = BCDToInt(time[6]);
////						 data.month = BCDToInt(time[5]);
////						 data.day = BCDToInt(time[4]);
////						 data.weekday = BCDToInt(time[3]);
////						 data.hour = BCDToInt(time[2]);
////						 data.min = BCDToInt(time[1]);
////						 data.second = BCDToInt(time[0]&0x7f);
////						 printf("time : 20%d年 %d月 %d日 周%d %d时 %d分 %d秒\r\n", data.year,data.month,data.day,data.weekday,data.hour,data.min,data.second);				 
////				 }
////				 vTaskDelay(pdMS_TO_TICKS(1000));
//				 
//				 

//				 //采集加速度 角速度
////						 ret = QMI8658C_read_6D(0x35,data6d,12);
////						 while(ret == 0){
////								ret = QMI8658C_read_6D(0x35,data6d,12); 		
////								vTaskDelay(pdMS_TO_TICKS(2000)); 
////						 };
////						 if(ret == 1 ){
////								 data.ax = ((int16_t)((((int16_t)data6d[1])<<8)  | data6d[0])) /2048.0;
////								 data.ay = ((int16_t)((((int16_t)data6d[3])<<8)  | data6d[2])) /2048.0;
////								 data.az = ((int16_t)((((int16_t)data6d[5])<<8)  | data6d[4])) /2048.0;
////								 data.gx = ((int16_t)((((int16_t)data6d[7])<<8)  | data6d[6])) /64.0;
////								 data.gy = ((int16_t)((((int16_t)data6d[9])<<8)  | data6d[8])) /64.0;
////								 data.gz = ((int16_t)((((int16_t)data6d[11])<<8)  | data6d[10])) /64.0;
////								 printf("%f %f %f %f %f %f \r\n", data.ax,data.ay,data.az,data.gx,data.gy,data.gz); 
////						 }		  

////				 vTaskDelay(pdMS_TO_TICKS(3000));

//			if(spo2_wake_up==1){
//				 ret = max_read_data(ppg_data_cache_IR,ppg_data_cache_RED,600);
//				 if(ret == 1){
//						data.heartrate = max30102_getHeartRate(ppg_data_cache_IR,600);
//						data.spo2 = max30102_getSpO2(ppg_data_cache_IR,ppg_data_cache_RED,600);
//						printf("%d %f \r\n", data.heartrate,data.spo2); 
//					 
//					  vTaskDelay(pdMS_TO_TICKS(5));
//				 }
//			
//			}



//	 	 
////				 if (uxQueueMessagesWaiting(sensor_data_queue) >= 10) {
////							sensor_data_t discard;
////							xQueueReceive(sensor_data_queue, &discard, 0);
////         }
////         xQueueSend(sensor_data_queue, &data, 0);
////				 
//			 vTaskDelay(pdMS_TO_TICKS(500));
//				 		 
//			 }
//			 vTaskDelay(pdMS_TO_TICKS(500));		
//		}
//}

