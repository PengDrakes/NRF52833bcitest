#ifndef __ECG_H
#define __ECG_H

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#define CYC_ARR_LEN 500


void ADS_Cap_count_process(void *p);
int32_t heart_rate_cal(int32_t *cache, int32_t size);
int32_t breath_frequence_cal(int32_t *cache, int32_t size);
void ecg_filiter(void * pvParameter);
void breath_thread(void * pvParameter);


#endif