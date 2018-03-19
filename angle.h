#ifndef __ANGLE_H
#define __ANGLE_H

#include "math.h"
#include "mpu9250.h"
#include "USART.h"
#include "base.h"


#define BLE_CONTROL	1






typedef enum{
	left,
	right,
	ahead,
	back
}DIRECTION;

float KalMan(Kal_Struct *kal,float x);
void Get_Accel_Angle(void);
void Get_Angle_Speed(void);

void Get_Mag_Angle(void);

void Get_Angle(void);
void Adjust_Acc(void);
void Get_Accel_Angle_Fast(void);
void Get_Angle_Speed_Fast(void);
extern float a_z_offset,a_x_offset,a_y_offset;
extern float w_x_offset,w_y_offset,w_z_offset;


float Get_Attitude_Data(int16_t raw_data,float range,int16_t offset);
void IMU_Update(float * ac,float * gy,float *attitude,float *ace);
float Get_Scale_Data(float raw_data,float offset,float scale);
void Scale_Mag(float *mag);
void AHR_Update(float * ac,float * gy,float * mag,float * attitude,float * ace);
void Free_Falling_Detect(float * accel);

void Get_Velocity(float * ace,float *v,int call_time,float ace_sub);
void Get_Height(float *vz,float refer_height,int call_time,float *height);

#endif

