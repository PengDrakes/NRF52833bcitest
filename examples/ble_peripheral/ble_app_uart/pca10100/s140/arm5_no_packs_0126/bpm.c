#include "bpm.h"

int32_t bpm_calculate(int32_t *loc,int32_t size)
{
    int32_t num,bpm;
	  num = size;
		
    switch (num)
    {
    case 2:
        bpm = 60.0/(loc[num-1]-loc[num-2])*FPS;        //fps=500                      //计算心率 算法:两峰之间点数*采样率
        break;
    
    case 3:
        bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3]))/2)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;

    case 4:
        bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4]))/3)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;

    case 5:
        bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5]))/4)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;

    case 6:
        bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5])+(loc[num-5]-loc[num-6]))/5)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
			 break;
    case 7:
        bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5])+(loc[num-5]-loc[num-6])+(loc[num-6]-loc[num-7]))/6)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;
		case 8:
			  bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5])+(loc[num-5]-loc[num-6])+(loc[num-6]-loc[num-7])+(loc[num-7]-loc[num-8]))/7)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;
		case 9:
			  bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5])+(loc[num-5]-loc[num-6])+(loc[num-6]-loc[num-7])+(loc[num-7]-loc[num-8])+(loc[num-8]-loc[num-9]))/8)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;		
		case 10:
			  bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5])+(loc[num-5]-loc[num-6])+(loc[num-6]-loc[num-7])+(loc[num-7]-loc[num-8])+(loc[num-8]-loc[num-9])+(loc[num-9]-loc[num-10]))/9)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;
		case 11:
			  bpm = 60.0/(((loc[num-1]-loc[num-2])+(loc[num-2]-loc[num-3])+(loc[num-3]-loc[num-4])+(loc[num-4]-loc[num-5])+(loc[num-5]-loc[num-6])+(loc[num-6]-loc[num-7])+(loc[num-7]-loc[num-8])+(loc[num-8]-loc[num-9])+(loc[num-9]-loc[num-10])+(loc[num-10]-loc[num-11]))/10)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;		
		case 12:
			  bpm = 60.0/((loc[num-1]-loc[num-12])/11)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;
		case 13:
			  bpm = 60.0/((loc[num-1]-loc[num-13])/12)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 14:
			  bpm = 60.0/((loc[num-1]-loc[num-14])/13)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 15:
			  bpm = 60.0/((loc[num-1]-loc[num-15])/14)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 16:
			  bpm = 60.0/((loc[num-1]-loc[num-16])/15)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 17:
			  bpm = 60.0/((loc[num-1]-loc[num-17])/16)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 18:
			  bpm = 60.0/((loc[num-1]-loc[num-18])/17)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 19:
			  bpm = 60.0/((loc[num-1]-loc[num-19])/18)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 20:
			  bpm = 60.0/((loc[num-1]-loc[num-20])/19)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 21:
			  bpm = 60.0/((loc[num-1]-loc[num-21])/20)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 22:
			  bpm = 60.0/((loc[num-1]-loc[num-22])/21)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 23:
			  bpm = 60.0/((loc[num-1]-loc[num-23])/22)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;	
		case 24:
			  bpm = 60.0/((loc[num-1]-loc[num-24])/23)*FPS;
				//bpm = 60.0/(location[num-1]-location[num-2])*204;
        break;			
		
    default: bpm = 60.0/(loc[num-1]-loc[num-2])*FPS; //340
        break;
    }
		
		//bpm = 60.0/(location[num-1]-location[num-2])*204;
    return bpm;
}

