#ifndef __medfilt_H
#define __medfilt_H


#include "arm_math.h"
#include "arm_const_structs.h"
#include "arm_common_tables.h"
#include "findpeaks.h"


#define midfilt_num 125    //125

float32_t midfilt1(float32_t *p_input,int size,int blocksize);

float32_t find_mid_val(float32_t *p_input,int size);


#endif //__medfilt_H