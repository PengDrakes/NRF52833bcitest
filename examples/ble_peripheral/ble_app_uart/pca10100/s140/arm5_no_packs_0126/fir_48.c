#include <stdint.h>
#include <string.h>
#include "fir_48.h"

uint32_t block_size = BLOCK_SIZE;

static float32_t firStateF32_48[BLOCK_SIZE+NUM_TAPS-1];

arm_fir_instance_f32 S;

const int BL = 49;
const float32_t B[49] = {
  -0.0009602747741,-0.0007406222285,-0.000344334956,0.0003267489665,  0.00132522604,
   0.002570599085, 0.003790664719, 0.004526928533,  0.00422275858, 0.002387276152,
  -0.001199811231,-0.006298475899, -0.01209692098, -0.01723066717, -0.01995631494,
   -0.01846508868, -0.01128097717, 0.002341947285,  0.02211954445,   0.0466186665,
    0.07336764783,  0.09919441491,   0.1207369491,   0.1350278109,   0.1400325894,
     0.1350278109,   0.1207369491,  0.09919441491,  0.07336764783,   0.0466186665,
    0.02211954445, 0.002341947285, -0.01128097717, -0.01846508868, -0.01995631494,
   -0.01723066717, -0.01209692098,-0.006298475899,-0.001199811231, 0.002387276152,
    0.00422275858, 0.004526928533, 0.003790664719, 0.002570599085,  0.00132522604,
  0.0003267489665,-0.000344334956,-0.0007406222285,-0.0009602747741
};
void arm_fir_f32_lp_48(float32_t *Input_buffer,float32_t *Output_buffer)
{
    uint32_t i;
    //arm_fir_instance_f32 S;
    float32_t *inputF32,*outputF32;

    inputF32 = Input_buffer;
    outputF32 = Output_buffer;

    //arm_fir_init_f32(&S,NUM_TAPS,(float32_t *)&B[0],&firStateF32_48[0],block_size);

    
    arm_fir_f32(&S,inputF32,outputF32,block_size);
    
}
void arm_fir48_init(void)
{
	arm_fir_init_f32(&S,NUM_TAPS,(float32_t *)&B[0],&firStateF32_48[0],block_size);
}









//#include <stdio.h>
//#include <math.h>
//#include <stdlib.h>
//#include <time.h>

//// 滤波器结构
//typedef struct {
//    float b[5];  // 分子系数
//    float a[5];  // 分母系数
//    float x[4];  // 输入历史
//    float y[4];  // 输出历史
//} BreathFilter;

//// 初始化呼吸滤波器 (0.1-0.5Hz, fs=250Hz)
////void breath_filter_init(BreathFilter *filt) {
////    // 2阶巴特沃斯带通滤波器系数
////    filt->b[0] = 1.304e-05f;
////    filt->b[1] = 0.0f;
////    filt->b[2] = -2.608e-05f;
////    filt->b[3] = 0.0f;
////    filt->b[4] = 1.304e-05f;
////    
////    filt->a[0] = 1.0f;
////    filt->a[1] = -3.991f;
////    filt->a[2] = 5.974f;
////    filt->a[3] = -3.983f;
////    filt->a[4] = 0.9996f;
////    
////    // 清零历史数据
////    for(int i = 0; i < 4; i++) {
////        filt->x[i] = 0.0f;
////        filt->y[i] = 0.0f;
////    }
////}
//void breath_filter_init(BreathFilter *filt) {
//    // 4 阶巴特沃斯带通滤波器：0.1 Hz – 0.5 Hz，fs = 250 Hz
//    filt->b[0] = 2.49882807e-10f;  // ≈ 2.50e-10
//    filt->b[1] = 0.0f;
//    filt->b[2] = -4.99765613e-10f; // ≈ -5.00e-10
//    filt->b[3] = 0.0f;
//    filt->b[4] = 2.49882807e-10f;  // ≈ 2.50e-10

//    filt->a[0] = 1.0f;
//    filt->a[1] = -3.99989851f;     // ≈ -4.00
//    filt->a[2] = 5.99957001f;      // ≈ 6.00
//    filt->a[3] = -3.99945875f;     // ≈ -4.00
//    filt->a[4] = 0.99988860f;      // ≈ 0.9999

//    // 清零历史状态
//    for(int i = 0; i < 4; i++) {
//        filt->x[i] = 0.0f;
//        filt->y[i] = 0.0f;
//    }
//}

//// 单次滤波处理
//float breath_filter_process(BreathFilter *filt, float input) {
//    // 直接II型转置结构
//    float w = input;
//    
//    // 反馈部分
//    w -= filt->a[1] * filt->y[0];
//    w -= filt->a[2] * filt->y[1];
//    w -= filt->a[3] * filt->y[2];
//    w -= filt->a[4] * filt->y[3];
//    
//    // 前馈部分
//    float output = filt->b[0] * w;
//    output += filt->b[1] * filt->y[0];
//    output += filt->b[2] * filt->y[1];
//    output += filt->b[3] * filt->y[2];
//    output += filt->b[4] * filt->y[3];
//    
//    // 更新历史状态
//    filt->y[3] = filt->y[2];
//    filt->y[2] = filt->y[1];
//    filt->y[1] = filt->y[0];
//    filt->y[0] = w;
//    
//    return output;
//}

//// 简单的呼吸频率检测
//typedef struct {
//    float buffer[250];  // 1秒缓存 (250Hz采样率)
//    int buf_idx;
//    int peak_count;
//    float last_value;
//    float threshold;
//} BreathDetector;

//void breath_detector_init(BreathDetector *det, float thresh) {
//    det->buf_idx = 0;
//    det->peak_count = 0;
//    det->last_value = 0.0f;
//    det->threshold = thresh;
//    for(int i = 0; i < 250; i++) {
//        det->buffer[i] = 0.0f;
//    }
//}

//// 检测波峰
//int detect_breath_peak(BreathDetector *det, float current_value) {
//    int breath_detected = 0;
//    
//    // 简单峰值检测
//    if(det->last_value > det->threshold && 
//       current_value <= det->threshold &&
//       det->last_value > det->buffer[(det->buf_idx + 248) % 250]) {  // 局部最大值
//        det->peak_count++;
//        breath_detected = 1;
//    }
//    
//    // 更新缓存
//    det->buffer[det->buf_idx] = current_value;
//    det->buf_idx = (det->buf_idx + 1) % 250;
//    det->last_value = current_value;
//    
//    return breath_detected;
//}

//// 计算呼吸频率（次/分钟）
//float calculate_breath_rate(BreathDetector *det) {
//    // 假设每秒调用一次此函数
//    float breaths_per_minute = det->peak_count * 60.0f / 2.0f;  // 2秒窗口
//    det->peak_count = 0;  // 重置计数
//    return breaths_per_minute;
//}


//static BreathDetector detector;
//// 主处理函数
//float process_breath_signal(float raw_sample) {
//    static BreathFilter filt;

//    static int initialized = 0;
//    
//    if(!initialized) {
//        breath_filter_init(&filt);
//        breath_detector_init(&detector, 0.5f);  // 阈值设为0.5
//        initialized = 1;
//    }
//    
//    // 1. 滤波
//    float filtered = breath_filter_process(&filt, raw_sample);
//    
//    // 2. 检测呼吸
//    detect_breath_peak(&detector, filtered);
//    
//    return filtered;
//}

//// 使用示例
//int main() {
//    // 模拟数据采集
//    const int SAMPLE_RATE = 250;  // 250Hz
//    const int TEST_SECONDS = 30;  // 测试30秒
//    const int TOTAL_SAMPLES = SAMPLE_RATE * TEST_SECONDS;
//    
//    // 模拟信号：0.2Hz呼吸 + 噪声
//    float simulated_signal[TOTAL_SAMPLES];
//    for(int i = 0; i < TOTAL_SAMPLES; i++) {
//        float t = i / (float)SAMPLE_RATE;
//        // 0.2Hz = 12次/分钟，模拟呼吸
//        simulated_signal[i] = 2.0f * sin(2.0f * 3.14159f * 0.2f * t) +
//                             0.5f * sin(2.0f * 3.14159f * 1.0f * t) +  // 心跳干扰
//                             0.3f * ((float)rand() / RAND_MAX - 0.5f); // 噪声
//    }
//    
//    // 处理信号
//    float breath_rates[TEST_SECONDS];
//    int sample_count = 0;
//    
//    for(int i = 0; i < TOTAL_SAMPLES; i++) {
//        float filtered = process_breath_signal(simulated_signal[i]);
//        sample_count++;
//        
//        // 每秒计算一次呼吸频率
//        if(sample_count >= SAMPLE_RATE) {
//            breath_rates[i / SAMPLE_RATE] = calculate_breath_rate(&detector);
//            sample_count = 0;
//        }
//    }
//    
//    // 输出结果
//    printf("呼吸频率检测结果（次/分钟）:\n");
//    for(int i = 0; i < TEST_SECONDS; i++) {
//        printf("第%d秒: %.1f\n", i, breath_rates[i]);
//    }
//    
//    return 0;
//}
