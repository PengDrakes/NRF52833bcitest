#ifndef __info_collect_H
#define __info_collect_H

#include <stdint.h>
#include <string.h>


typedef struct {
    float ax, ay, az, gx, gy, gz ;
		uint32_t year, month, day, weekday, hour, min, second;
		uint16_t heartrate;
		float spo2;
		
} sensor_data_t;



void collect_info_thread(void * pvParameter);




#endif