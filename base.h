#ifndef __BASE_H
#define __BASE_H

#include "outputdata.h"
#include "stm32f4xx_hal.h"
float avarge(int *data,int count);

typedef struct _kal_struct{
	float A;   //一般为1
	float B;   //一般为0
	float Q;//系统过程噪声的协方差
	float R;//测量噪声的协方差
	
	float kal_out; //上一次卡尔曼的输出
	
	float cov; //上一次卡尔曼的输出的协方差
	
}Kal_Struct;

typedef struct{
  float * Window_Buffer;
  int max;  //数组长度
  int i;
}Window_Filter_Struct;

typedef struct{
    float UTC_TIME;
    unsigned char State;
    double Lat;
    double Long;
    float v;
}GPRMC;


float Window_Filter(Window_Filter_Struct * wfs,float data);

float KalMan(Kal_Struct *kal,float x);

extern char Send_Wave_Flag;
void set_debug_wave(int arg_num,char ** string_prams,float * float_prams);
void debug_send_wave();


#endif
