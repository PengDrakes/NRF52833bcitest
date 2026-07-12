

#if 1

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrfx_gpiote.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
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
#include "spo2.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "ecg.h"
#include "semphr.h"

extern SemaphoreHandle_t print_mutex;

uint32_t ecg_data;

int32_t  save_ecg_data[CYC_ARR_LEN]={0};	
uint32_t  write_ecg_pos=0;  //当前存储数据的位置
uint32_t  read_ecg_pos=0; //当前分析数据的位置

static float32_t uv_cache[36];
static float32_t uv_cache1[18];
static float32_t fir_put[36];  
static int mid_filt_start_flag;
static float32_t mid_filt_cache[midfilt_num];
static float32_t mid_filt_cache1[midfilt_num];
static float32_t mid_val;
#define BMP_CACHE_SIZE  2500
int32_t bpm_cache[BMP_CACHE_SIZE]; 
static int32_t bpm_cache1[BMP_CACHE_SIZE]; 


#define BREATH_CACHE_SIZE  250*60
//static int32_t breath_cache[BREATH_CACHE_SIZE]; 
static int32_t filtered_cache[BREATH_CACHE_SIZE]; 

float32_t a1,b1;//心率系数
float32_t a0,b0;//呼吸频率系数



#define INIT_SAMPLE_COUNT 1000
extern float32_t data1[INIT_SAMPLE_COUNT],data2[INIT_SAMPLE_COUNT];
extern volatile uint8_t init_data_collected;

#define MAX_BEAT 40
static int32_t pn_npks;                           //心率峰值检测函数峰值数量
static int32_t pn_locs[MAX_BEAT];                       //心率峰值检测函数输出峰值点位置
static int32_t bpm;                          //心率数值
int32_t filtered_locs[MAX_BEAT];
int32_t filtered_count ;


#define MAX_BREATH 30
static int32_t b_pn_npks;                         //呼吸峰值检测函数峰值数量
static int32_t b_pn_locs[MAX_BREATH];                     //呼吸峰值检测函数输出峰值点
static int32_t breath_freauence;                          //呼吸频率
int32_t b_filtered_locs[MAX_BREATH];
int32_t b_filtered_count ;

float32_t min;
uint32_t min_index;

#define CYC_ARRAY_LEN 2000
extern int32_t  save_data[CYC_ARRAY_LEN];	
extern uint32_t  write_pos;  
extern uint32_t  read_pos; 

extern int32_t  save_data_0[CYC_ARRAY_LEN];	
extern uint32_t  write_pos_0;  
extern uint32_t  read_pos_0; 

extern int32_t  save_data_1[CYC_ARRAY_LEN];	
extern uint32_t  write_pos_1;  
extern uint32_t  read_pos_1; 

uint32_t  adc_cap_count=0;


void ADS_Cap_count_process(void *p){
		uint32_t   front_value=adc_cap_count;
	  uint32_t   current_value=adc_cap_count;
	
	  while(1){
			vTaskDelay(pdMS_TO_TICKS(1000));
			current_value=adc_cap_count;
			printf("======>1s cap count:%d\r\n",(current_value +10000 - front_value)%10000);
			front_value = current_value;
		
		}
}





int32_t heart_rate_cal(int32_t *cache, int32_t size)
{
		int32_t heart_rate;
//	  int32_t filtered_count;
		
		filtered_count = 0;
    maxim_peaks_above_min_height(pn_locs, &pn_npks, cache, size, 120);
    
    if (pn_npks < 2) {
        //printf("not enough pn_npks\r\n");
        return 0;
    }
		
    int32_t temp_locs[MAX_BEAT];   
    int32_t peak_heights[MAX_BEAT];
    for (int i = 0; i < pn_npks; i++) {
			  temp_locs[i] = pn_locs[i];
        peak_heights[i] = cache[pn_locs[i]];
    }
    
    int32_t temp_count = pn_npks;
    maxim_remove_close_peaks(temp_locs, &temp_count, peak_heights, 100);  
    
    for (int i = 0; i < temp_count ; i++) {
        filtered_locs[filtered_count++] = temp_locs[i];
    }
    
    float32_t float_locs[MAX_BEAT];
    for (int i = 0; i < filtered_count; i++) {
        float_locs[i] = (float32_t)filtered_locs[i];
    }
    
    maxim_sort_ascend(float_locs, filtered_count);
    
    for (int i = 0; i < filtered_count; i++) {
        filtered_locs[i] = (int32_t)float_locs[i];
    }
    

    for (int i = 0; i < filtered_count; i++) {
			  //printf("第[%d]个峰值: 位置%d, 值%d\r\n", i, filtered_locs[i], cache[filtered_locs[i]]);
    }
    
		if (filtered_count >= 2){
				heart_rate = bpm_calculate(filtered_locs, filtered_count);
				//printf("bpm = %d\r\n",heart_rate); 
		}
		
    return heart_rate;
}

int32_t breath_frequence_cal(int32_t *cache, int32_t size)
{
		int32_t breath_rate;
	
		b_filtered_count = 0;
		
    maxim_peaks_above_min_height(b_pn_locs, &b_pn_npks, cache, size, 0);
	
		printf("b_pn_npks：%d\r\n",b_pn_npks);
    
	  printf("前10个峰值位置: ");
    for(int i = 0; i < (b_pn_npks > 10 ? 10 : b_pn_npks); i++) {
        printf("%d ", b_pn_locs[i]);
    }
    printf("\r\n");
		
    if (b_pn_npks < 2) {
        return 0;
    }
		

		
    int32_t temp_locs[MAX_BREATH];   
    int32_t peak_heights[MAX_BREATH];
    for (int i = 0; i < b_pn_npks; i++) {
			  temp_locs[i] = b_pn_locs[i];
        peak_heights[i] = cache[b_pn_locs[i]];
    }
    int32_t temp_count = b_pn_npks;
    maxim_remove_close_peaks(temp_locs, &temp_count, peak_heights, 375);  
		printf("breath_rate = %d\r\n",temp_count); 
		
		
    return temp_count;
}





//void ecg_filiter(void * pvParameter){
//		static uint16_t j,mid_filt_num,n;
//		read_pos_1 = 0;
//		arm_fir48_init(); 
//	
//		while(init_data_collected == 0 ){  vTaskDelay(pdMS_TO_TICKS(1)); }
//		
//		printf("ecg_filiter start\r\n");
//		
//		
//		ADS1292_val_init(data2,&a1,&b1);
//		//printf("a1:%f  b1:%f\r\n",a1,b1);
//		
//		arm_min_f32(data1,INIT_SAMPLE_COUNT,&min,&min_index);
//		
//		
//		while(1){
//				if(read_pos_1 == write_pos_1){      //读完了
//					//printf("no data \r\n");
//					vTaskDelay(pdMS_TO_TICKS(1));
//        }else{
//								//adc_cap_count = (adc_cap_count+1+10000)%10000;
//					
//								uv_cache[j] = save_data_1[read_pos_1] *a1+b1;
//							  //uv_cache[j] = save_data_1[read_pos_1] * 1.0;
//								//printf("%d\r\n",save_data_1[read_pos_1]);
//					
//								j++;
//								if(j == 19){
//										j=18;
//									  arm_fir_f32_lp_48(uv_cache,fir_put);
//										if(mid_filt_start_flag == 0)
//										{
//												mid_filt_cache[mid_filt_num] = fir_put[0];                                                  
//												mid_filt_num++;
//												if(mid_filt_num == midfilt_num)
//												{
//														mid_filt_start_flag = 1;
//												}
//										}
//										else if(mid_filt_start_flag == 1){
//												arm_copy_f32(mid_filt_cache+1,mid_filt_cache1,midfilt_num-1); // 左移一位，去掉最旧的数据
//												mid_filt_cache1[midfilt_num-1] = fir_put[0]; // 在末尾加入最新的ECG值     
//                        mid_val = midfilt1(mid_filt_cache1,midfilt_num,midfilt_num); // 计算中值
//											  
//											  bpm_cache[n] = (fir_put[0]-mid_val+100)*1000000; 
//												
//												ecg_data = (fir_put[0]-mid_val+100) * 1000000;
//													
////												n++;
////												if(n>BMP_CACHE_SIZE)
////												{
////														n=0;
////														bpm =  heart_rate_cal(bpm_cache, BMP_CACHE_SIZE);	
////												}
//											
//												//printf("%d\t%d\t%f\r\n",save_data_1[read_pos_1],read_pos_1,((fir_put[0]-mid_val+100)));
//												if (xSemaphoreTake(print_mutex, portMAX_DELAY) == pdTRUE) {
//														printf("%f\r\n",((fir_put[0]-mid_val+100)));
//														xSemaphoreGive(print_mutex);
//												}
//												
//											
//												if(((write_ecg_pos+1+CYC_ARR_LEN)%CYC_ARR_LEN) !=  read_ecg_pos ){
//													save_ecg_data[write_ecg_pos] = ecg_data;
//													write_ecg_pos = (write_ecg_pos+1+CYC_ARR_LEN)%CYC_ARR_LEN;		
//												}
//																						
//												arm_copy_f32(mid_filt_cache1,mid_filt_cache,midfilt_num);
//										}
//										arm_copy_f32(uv_cache+1,uv_cache1,18);     //将前一数组的后18位拷贝到缓存数组中，作为FIR滤波器的群延时
//										
//										arm_copy_f32(uv_cache1,uv_cache,18); 
//										
//								}
//		
//					 read_pos_1 =(read_pos_1+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN;	
//				}		
//				

//				vTaskDelay(pdMS_TO_TICKS(3));				
//			
//		}
//}



////去掉异常值
void ecg_filiter(void * pvParameter){
		static uint16_t j,mid_filt_num,n;
		read_pos_1 = 0;
		arm_fir48_init(); 
	
		while(init_data_collected == 0 ){  vTaskDelay(pdMS_TO_TICKS(1)); }
		
		printf("ecg_filiter start\r\n");
		
		
		ADS1292_val_init(data2,&a1,&b1);
		//printf("a1:%f  b1:%f\r\n",a1,b1);
		
		arm_min_f32(data1,INIT_SAMPLE_COUNT,&min,&min_index);
		
		while(1){
				if(read_pos_1 == write_pos_1){      //读完了
					//printf("no data \r\n");
					vTaskDelay(pdMS_TO_TICKS(1));
        }else{
//								adc_cap_count = (adc_cap_count+1+10000)%10000;
					
								uv_cache[j] = save_data_1[read_pos_1] *a1+b1;
					
								j++;
								if(j == 19){
										j=18;
									  arm_fir_f32_lp_48(uv_cache,fir_put);
										if(mid_filt_start_flag == 0)
										{
												mid_filt_cache[mid_filt_num] = fir_put[0];                                                  
												mid_filt_num++;
												if(mid_filt_num == midfilt_num)
												{
														mid_filt_start_flag = 1;
												}
										}
										else if(mid_filt_start_flag == 1){
												arm_copy_f32(mid_filt_cache+1,mid_filt_cache1,midfilt_num-1); // 左移一位，去掉最旧的数据
												mid_filt_cache1[midfilt_num-1] = fir_put[0]; // 在末尾加入最新的ECG值     
                        mid_val = midfilt1(mid_filt_cache1,midfilt_num,midfilt_num); // 计算中值
												
												ecg_data = (fir_put[0]-mid_val+100) * 1000000;
													
											
												//if( (fir_put[0]-mid_val+100) >-120 && (fir_put[0]-mid_val+100) <660 ){
												
													if (xSemaphoreTake(print_mutex, portMAX_DELAY) == pdTRUE) {
															printf("%f \r\n",((fir_put[0]-mid_val+100)));
															xSemaphoreGive(print_mutex);
													}
													
													if(((write_ecg_pos+1+CYC_ARR_LEN)%CYC_ARR_LEN) !=  read_ecg_pos ){
														save_ecg_data[write_ecg_pos] = ecg_data;
														write_ecg_pos = (write_ecg_pos+1+CYC_ARR_LEN)%CYC_ARR_LEN;		
													}
												//}
											
																					
												arm_copy_f32(mid_filt_cache1,mid_filt_cache,midfilt_num);
										}
										arm_copy_f32(uv_cache+1,uv_cache1,18);     //将前一数组的后18位拷贝到缓存数组中，作为FIR滤波器的群延时
										
										arm_copy_f32(uv_cache1,uv_cache,18); 
										
								}
		
					 read_pos_1 =(read_pos_1+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN;	
				}		
				

				vTaskDelay(pdMS_TO_TICKS(3));				
			
		}
}


// 滤波器结构
typedef struct {
    double b[5];  // 分子系数
    double a[5];  // 分母系数
    double x[4];  // 输入历史
    double y[4];  // 输出历史
} BreathFilter;


void breath_filter_init(BreathFilter *filt) {
    // 4 阶巴特沃斯带通滤波器：0.1 Hz – 0.5 Hz，fs = 250 Hz
    filt->b[0] = 2.50876372702265e-05;   
    filt->b[1] = 0.0;
    filt->b[2] = -5.01752745404529e-05;   
    filt->b[3] = 0.0;
    filt->b[4] = 2.50876372702265e-05;   

    filt->a[0] = 1.0;
    filt->a[1] = -3.98572007021102;     
    filt->a[2] = 5.95732394935168;     
    filt->a[3] = -3.95748724024486;    
    filt->a[4] = 0.985883362094618;    

    for(int i = 0; i < 4; i++) {
        filt->x[i] = 0.0;
        filt->y[i] = 0.0;
    }
}
double breath_filter_process(BreathFilter *filt, double input) {
    // 1. 计算当前输出 y[n]
    double y_n = filt->b[0] * input
              + filt->b[1] * filt->x[0]
              + filt->b[2] * filt->x[1]
              + filt->b[3] * filt->x[2]
              + filt->b[4] * filt->x[3]
              - filt->a[1] * filt->y[0]
              - filt->a[2] * filt->y[1]
              - filt->a[3] * filt->y[2]
              - filt->a[4] * filt->y[3];

    // 2. 更新输入历史：右移，新输入进入 x[0]
    filt->x[3] = filt->x[2];
    filt->x[2] = filt->x[1];
    filt->x[1] = filt->x[0];
    filt->x[0] = input;

    // 3. 更新输出历史：右移，新输出进入 y[0]
    filt->y[3] = filt->y[2];
    filt->y[2] = filt->y[1];
    filt->y[1] = filt->y[0];
    filt->y[0] = y_n;

    // 4. 返回当前输出
    return y_n;
}




void breath_thread(void * pvParameter){
			static uint16_t n;
	    BreathFilter filter;
			breath_filter_init(&filter); 
	
	
			while(init_data_collected == 0 ){  vTaskDelay(pdMS_TO_TICKS(1)); }
			

			bre_val_init(data1,&a0,&b0);
			arm_min_f32(data1,INIT_SAMPLE_COUNT,&min,&min_index);
			
			//printf("%f\t%f\r\n",a0,b0);
					
			while(1){
			
					if(read_pos_0 != write_pos_0){

						//breath_cache[n] = save_data_0[read_pos_0]*a0 + b0 ;
						filtered_cache[n] = breath_filter_process(&filter, save_data_0[read_pos_0]*a0+b0 ) * 1000000;
						n++;
						if(n>=BREATH_CACHE_SIZE){
								n=0;
								breath_freauence = breath_frequence_cal(filtered_cache, BREATH_CACHE_SIZE);
						}else{
								//printf("%f\t%f\r\n",save_data_0[read_pos_0]*a0+b0,breath_filter_process(&filter, save_data_0[read_pos_0]*a0+b0 ) );
							
						}

						read_pos_0 =(read_pos_0+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN;				
					}
						vTaskDelay(pdMS_TO_TICKS(1));
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
#include "spo2.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "ecg.h"

uint32_t ecg_data;

int32_t  save_ecg_data[CYC_ARR_LEN]={0};	
uint32_t  write_ecg_pos=0;  //当前存储数据的位置
uint32_t  read_ecg_pos=0; //当前分析数据的位置

static float32_t uv_cache[36];
static float32_t uv_cache1[18];
static float32_t fir_put[36];  
static int mid_filt_start_flag;
static float32_t mid_filt_cache[midfilt_num];
static float32_t mid_filt_cache1[midfilt_num];
static float32_t mid_val;
#define BMP_CACHE_SIZE  2500
int32_t bpm_cache[BMP_CACHE_SIZE]; 
static int32_t bpm_cache1[BMP_CACHE_SIZE]; 


#define BREATH_CACHE_SIZE  250*60
//static int32_t breath_cache[BREATH_CACHE_SIZE]; 
static int32_t filtered_cache[BREATH_CACHE_SIZE]; 

float32_t a1,b1;//心率系数
float32_t a0,b0;//呼吸频率系数



#define INIT_SAMPLE_COUNT 1000
extern float32_t data1[INIT_SAMPLE_COUNT],data2[INIT_SAMPLE_COUNT];
extern volatile uint8_t init_data_collected;

#define MAX_BEAT 40
static int32_t pn_npks;                           //心率峰值检测函数峰值数量
static int32_t pn_locs[MAX_BEAT];                       //心率峰值检测函数输出峰值点位置
static int32_t bpm;                          //心率数值
int32_t filtered_locs[MAX_BEAT];
int32_t filtered_count ;


#define MAX_BREATH 30
static int32_t b_pn_npks;                         //呼吸峰值检测函数峰值数量
static int32_t b_pn_locs[MAX_BREATH];                     //呼吸峰值检测函数输出峰值点
static int32_t breath_freauence;                          //呼吸频率
int32_t b_filtered_locs[MAX_BREATH];
int32_t b_filtered_count ;

float32_t min;
uint32_t min_index;

#define CYC_ARRAY_LEN 2000

extern int32_t  save_data_0[CYC_ARRAY_LEN];	
extern uint32_t  write_pos_0;  
extern uint32_t  read_pos_0; 

extern int32_t  save_data_1[CYC_ARRAY_LEN];	
extern uint32_t  write_pos_1;  
extern uint32_t  read_pos_1; 

uint32_t  adc_cap_count=0;


void ADS_Cap_count_process(void *p){
		uint32_t   front_value=adc_cap_count;
	  uint32_t   current_value=adc_cap_count;
	
	  while(1){
			vTaskDelay(pdMS_TO_TICKS(1000));
			current_value=adc_cap_count;
			printf("======>1s cap count:%d\r\n",(current_value +10000 - front_value)%10000);
			front_value = current_value;
		
		}
}


int32_t heart_rate_cal(int32_t *cache, int32_t size)
{
		int32_t heart_rate;
		
		filtered_count = 0;
    maxim_peaks_above_min_height(pn_locs, &pn_npks, cache, size, 120);
    
    if (pn_npks < 2) {
        //printf("not enough pn_npks\r\n");
        return 0;
    }
		
    int32_t temp_locs[MAX_BEAT];   
    int32_t peak_heights[MAX_BEAT];
    for (int i = 0; i < pn_npks; i++) {
			  temp_locs[i] = pn_locs[i];
        peak_heights[i] = cache[pn_locs[i]];
    }
    
    int32_t temp_count = pn_npks;
    maxim_remove_close_peaks(temp_locs, &temp_count, peak_heights, 100);  
    
    for (int i = 0; i < temp_count ; i++) {
        filtered_locs[filtered_count++] = temp_locs[i];
    }
    
    float32_t float_locs[MAX_BEAT];
    for (int i = 0; i < filtered_count; i++) {
        float_locs[i] = (float32_t)filtered_locs[i];
    }
    
    maxim_sort_ascend(float_locs, filtered_count);
    
    for (int i = 0; i < filtered_count; i++) {
        filtered_locs[i] = (int32_t)float_locs[i];
    }
    

    for (int i = 0; i < filtered_count; i++) {
			  //printf("第[%d]个峰值: 位置%d, 值%d\r\n", i, filtered_locs[i], cache[filtered_locs[i]]);
    }
    
		if (filtered_count >= 2){
				heart_rate = bpm_calculate(filtered_locs, filtered_count);
				//printf("bpm = %d\r\n",heart_rate); 
		}
		
    return heart_rate;
}

int32_t breath_frequence_cal(int32_t *cache, int32_t size)
{
		int32_t breath_rate;
	
		b_filtered_count = 0;
		
    maxim_peaks_above_min_height(b_pn_locs, &b_pn_npks, cache, size, 0);
	
		printf("b_pn_npks：%d\r\n",b_pn_npks);
    
	  printf("前10个峰值位置: ");
    for(int i = 0; i < (b_pn_npks > 10 ? 10 : b_pn_npks); i++) {
        printf("%d ", b_pn_locs[i]);
    }
    printf("\r\n");
		
    if (b_pn_npks < 2) {
        return 0;
    }
		

		
    int32_t temp_locs[MAX_BREATH];   
    int32_t peak_heights[MAX_BREATH];
    for (int i = 0; i < b_pn_npks; i++) {
			  temp_locs[i] = b_pn_locs[i];
        peak_heights[i] = cache[b_pn_locs[i]];
    }
    int32_t temp_count = b_pn_npks;
    maxim_remove_close_peaks(temp_locs, &temp_count, peak_heights, 375);  
		printf("breath_rate = %d\r\n",temp_count); 
		
		
    return temp_count;
}





void ecg_filiter(void * pvParameter){
		static uint16_t j,mid_filt_num,n;
		read_pos_1 = 0;
		arm_fir48_init(); 
	
		while(init_data_collected == 0 ){  vTaskDelay(pdMS_TO_TICKS(1)); }
		
		printf("ecg_filiter start\r\n");
		
		
		ADS1292_val_init(data2,&a1,&b1);
		//printf("a1:%f  b1:%f\r\n",a1,b1);
		
		arm_min_f32(data2,INIT_SAMPLE_COUNT,&min,&min_index);
		
		
		while(1){
				if(read_pos_1 == write_pos_1){      //读完了
					//printf("no data \r\n");
					vTaskDelay(pdMS_TO_TICKS(1));
        }else{
								adc_cap_count = (adc_cap_count+1+10000)%10000;
					
								uv_cache[j] = save_data_1[read_pos_1] *a1+b1;
								//uv_cache[j] = save_data_1[read_pos_1] ;
					
								j++;
								if(j == 19){
										j=18;
									  arm_fir_f32_lp_48(uv_cache,fir_put);
										if(mid_filt_start_flag == 0)
										{
												mid_filt_cache[mid_filt_num] = fir_put[0];                                                  
												mid_filt_num++;
												if(mid_filt_num == midfilt_num)
												{
														mid_filt_start_flag = 1;
												}
										}
										else if(mid_filt_start_flag == 1){
												arm_copy_f32(mid_filt_cache+1,mid_filt_cache1,midfilt_num-1); // 左移一位，去掉最旧的数据
												mid_filt_cache1[midfilt_num-1] = fir_put[0]; // 在末尾加入最新的ECG值     
                        mid_val = midfilt1(mid_filt_cache1,midfilt_num,midfilt_num); // 计算中值
											  
											  bpm_cache[n] = (fir_put[0]-mid_val+100)*1000000; 
												
												ecg_data = (fir_put[0]-mid_val+100) * 1000000;
													
//												n++;
//												if(n>BMP_CACHE_SIZE)
//												{
//														n=0;
//														bpm =  heart_rate_cal(bpm_cache, BMP_CACHE_SIZE);	
//												}
											
												//printf("%d\t%d\t%f\r\n",save_data_1[read_pos_1],read_pos_1,((fir_put[0]-mid_val+100)));
												printf("%f\r\n",((fir_put[0]-mid_val+100)));
											
												if(((write_ecg_pos+1+CYC_ARR_LEN)%CYC_ARR_LEN) !=  read_ecg_pos ){
													save_ecg_data[write_ecg_pos] = ecg_data;
													write_ecg_pos = (write_ecg_pos+1+CYC_ARR_LEN)%CYC_ARR_LEN;		
												}
																						
												arm_copy_f32(mid_filt_cache1,mid_filt_cache,midfilt_num);
										}
										arm_copy_f32(uv_cache+1,uv_cache1,18);     //将前一数组的后18位拷贝到缓存数组中，作为FIR滤波器的群延时
										
										arm_copy_f32(uv_cache1,uv_cache,18); 
										
								}
		
					 read_pos_1 =(read_pos_1+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN;	
				}		
				

				vTaskDelay(pdMS_TO_TICKS(4));				
			
		}
}


// 滤波器结构
typedef struct {
    double b[5];  // 分子系数
    double a[5];  // 分母系数
    double x[4];  // 输入历史
    double y[4];  // 输出历史
} BreathFilter;


void breath_filter_init(BreathFilter *filt) {
    // 4 阶巴特沃斯带通滤波器：0.1 Hz – 0.5 Hz，fs = 250 Hz
    filt->b[0] = 2.50876372702265e-05;   
    filt->b[1] = 0.0;
    filt->b[2] = -5.01752745404529e-05;   
    filt->b[3] = 0.0;
    filt->b[4] = 2.50876372702265e-05;   

    filt->a[0] = 1.0;
    filt->a[1] = -3.98572007021102;     
    filt->a[2] = 5.95732394935168;     
    filt->a[3] = -3.95748724024486;    
    filt->a[4] = 0.985883362094618;    

    for(int i = 0; i < 4; i++) {
        filt->x[i] = 0.0;
        filt->y[i] = 0.0;
    }
}
double breath_filter_process(BreathFilter *filt, double input) {
    //  计算当前输出 y[n]
    double y_n = filt->b[0] * input
              + filt->b[1] * filt->x[0]
              + filt->b[2] * filt->x[1]
              + filt->b[3] * filt->x[2]
              + filt->b[4] * filt->x[3]
              - filt->a[1] * filt->y[0]
              - filt->a[2] * filt->y[1]
              - filt->a[3] * filt->y[2]
              - filt->a[4] * filt->y[3];

    //  更新输入历史：右移，新输入进入 x[0]
    filt->x[3] = filt->x[2];
    filt->x[2] = filt->x[1];
    filt->x[1] = filt->x[0];
    filt->x[0] = input;

    //  更新输出历史：右移，新输出进入 y[0]
    filt->y[3] = filt->y[2];
    filt->y[2] = filt->y[1];
    filt->y[1] = filt->y[0];
    filt->y[0] = y_n;

    // 返回当前输出
    return y_n;
}




void breath_thread(void * pvParameter){
			static uint16_t n;
	    BreathFilter filter;
			breath_filter_init(&filter); 
	
	
			while(init_data_collected == 0 ){  vTaskDelay(pdMS_TO_TICKS(1)); }
			

			bre_val_init(data1,&a0,&b0);
			arm_min_f32(data1,INIT_SAMPLE_COUNT,&min,&min_index);
			
			//printf("%f\t%f\r\n",a0,b0);
					
			while(1){
			
					if(read_pos_0 != write_pos_0){

						//breath_cache[n] = save_data_0[read_pos_0]*a0 + b0 ;
						filtered_cache[n] = breath_filter_process(&filter, save_data_0[read_pos_0]*a0+b0 ) * 1000000;
						n++;
						if(n>=BREATH_CACHE_SIZE){
								n=0;
								breath_freauence = breath_frequence_cal(filtered_cache, BREATH_CACHE_SIZE);
						}else{
								//printf("%f\t%f\r\n",save_data_0[read_pos_0]*a0+b0,breath_filter_process(&filter, save_data_0[read_pos_0]*a0+b0 ) );
							
						}

						read_pos_0 =(read_pos_0+1+CYC_ARRAY_LEN)%CYC_ARRAY_LEN;				
					}
						vTaskDelay(pdMS_TO_TICKS(1));
			}


}


#endif




