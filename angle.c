#include "angle.h"
#include "usart.h"
#include "arm_math.h"
#include "base.h"



#define INTEGRAL_CONSTANT 0.002f		//积分时间 2ms
#define HALF_T 0.001f
#define GRAVITY_CONSTANT 9.8f			//重力加速度
#define Semig 1e-6f
#define GY521 1
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim7;

extern MPU_Dev MPU9250;

float Angle_Speed[3];
float Accel[3];
float Accel_WFS[3];
float Angle[3];
float Mag[3];
float Accel_E[3]; //天地坐标系下的加速度
float Velocity[3];
float Height;
int16_t Gyro_Offset[3];

float gravity_X,gravity_Y,gravity_Z; //重力加速度分量

float q0=1.0f;
float q1=0.0f;
float q2=0.0f;
float q3=0.0f;

float exInt,eyInt,ezInt;

float IMU_P=1.0f;				//2.0f   //20
float IMU_I=0.005f;		//0.005          0.03




/*
  对数据进行零偏校准
*/
float Get_Attitude_Data(int16_t raw_data,float range,int16_t offset){
  return (raw_data-offset)*range/32768.0f;
}

/*
  对数据进行缩放校准
*/
float Get_Scale_Data(float raw_data,float offset,float scale){
  return (raw_data-offset)*scale;
}

void Scale_Mag(float *mag){
  
  float mag_offset[3]={
    -1.10733816275476,
    1.68293111911182,
   -1.47880622640983
  };
  float B[6]={
    1.06467231442859,
    -0.000170726411333116,
    0.0286170458742405,
    1.0056622056883,
    0.0129797749003217,
    0.934904674137626
  };

  float temp[3];
  int i;
  for(i=0;i<3;++i){
    temp[i]=mag[i]-mag_offset[i];
  }
  mag[0]=B[0]*temp[0]+B[1]*temp[1]+B[2]*temp[2];
  mag[1]=B[1]*temp[0]+B[3]*temp[1]+B[4]*temp[2];
  mag[2]=B[2]*temp[0]+B[4]*temp[1]+B[5]*temp[2];  
}

uint8_t Adjust_Accel_Mag_Flag;
extern uint8_t compare_string(const char *s1,char * s2);
void adjust_gyro(int arg_num,char **s,float * arg);
void adjust_accel_mag(int arg_num,char **s,float * arg){
  int16_t ac[3],gy[3],mag[3],i;
  float accel[3],mag_f[3];
  if(arg_num!=0x0100&&arg_num!=0x0101){
    uprintf("error arg_num!\r\n");
    return ;
  }
  if(compare_string("gyro",s[0])){
    adjust_gyro(0,0,0);
    return ;
  }
  if(arg_num==0x0100){//无参数
    Adjust_Accel_Mag_Flag=!Adjust_Accel_Mag_Flag;
  }else{
    Adjust_Accel_Mag_Flag=(uint8_t)arg[0];
  }
  
  if(Adjust_Accel_Mag_Flag){
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  }else{
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  }
  
  while(Adjust_Accel_Mag_Flag){
    if(compare_string("ac",s[0])){
      MPU_Read6500(&MPU9250,ac,gy);
      for(i=0;i<3;++i){
        accel[i]=Get_Attitude_Data(ac[i],MPU9250.setting->accel_range,0);
      }
      uprintf("%f,%f,%f\r\n",accel[0],accel[1],accel[2]);
      HAL_Delay(50);  //20HZ
    }else if(compare_string("mag",s[0])){
      MPU_ReadM_Mag(&MPU9250,mag);
      for(i=0;i<3;++i){
        mag_f[i]=mag[i]*0.15f/100.0f;
      }
      uprintf("%f,%f,%f\r\n",mag_f[0],mag_f[1],mag_f[2]);
      HAL_Delay(50);   //20HZ
    }
  }
}
void adjust_gyro(int arg_num,char **s,float * arg){
  int i;
  int16_t ac[3],gy[3];
  int32_t gy_sum[3]={0};
  
  HAL_NVIC_DisableIRQ(TIM7_IRQn);
  uprintf("start to adjust gyro!\r\n");
  for(i=0;i<200;++i){
    MPU_Read6500(&MPU9250,ac,gy);
    gy_sum[0]+=gy[0];
    gy_sum[1]+=gy[1];
    gy_sum[2]+=gy[2];
    HAL_Delay(1);
  }
  for(i=0;i<3;++i){
    Gyro_Offset[i]=gy_sum[i]/200; 
    uprintf("Gyro offset[%d]=%d\r\n",i,Gyro_Offset[i]);
  }
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  uprintf("adjust gyro succesful!\r\n");
  
}

extern void Fly_Init();

struct{
  int Falling_Time;
  uint8_t Fall_Detect_Mode;
  uint16_t Start_Time;
  uint8_t  Falling;
}Falling_Detect={0,0,5000,0};

/*
  Call_time 该函数的调用周期，单位：ms
*/
extern uint32_t uwTick;
void Free_Falling_Detect(float * accel,int call_time){
  
  if(accel[2]<0.1f&&uwTick>Falling_Detect.Start_Time){
    Falling_Detect.Falling_Time+=call_time;
    if(Falling_Detect.Falling_Time>300){
      Fly_Init();
      Falling_Detect.Falling_Time=0;
    }
  }else{
    Falling_Detect.Falling_Time=0;
  }
}


float ex,ey,ez;

float Accel_Z_Window[8];
float Accel_X_Window[8];
float Accel_Y_Window[8];

Window_Filter_Struct Accel_Z_WFS={Accel_Z_Window,8,0};
Window_Filter_Struct Accel_X_WFS={Accel_X_Window,8,0};
Window_Filter_Struct Accel_Y_WFS={Accel_Y_Window,8,0};

void IMU_Update(float * ac,float * gy,float *attitude,float *ace){

	float ax,ay,az,wx,wy,wz;
    float q0q0 = q0 * q0;                                                        
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
	float q1q2=q1*q2;
	float q0q3=q0*q3;
	float norm;
    float gbx,gby,gbz;
    
    Accel_WFS[2]=Window_Filter(&Accel_Z_WFS,ac[2]);
    Accel_WFS[0]=Window_Filter(&Accel_X_WFS,ac[0]);
    Accel_WFS[1]=Window_Filter(&Accel_Y_WFS,ac[1]);
    
    az=Accel_WFS[2];
    ax=Accel_WFS[0];
    ay=Accel_WFS[1];

    //ax=ac[0];ay=ac[1];az=ac[2];
    wx=gy[0];wy=gy[1];wz=gy[2];
      
        //计算加速度旋转到天地坐标系的值
    ace[0] = 2*ax*(0.5f - q2q2 - q3q3) + 2*ay*(q1q2 - q0q3) + 2*az*(q1q3 + q0q2); 
	ace[1] = 2*ax*(q1q2 + q0q3) + 2*ay*(0.5f - q1q1 - q3q3) + 2*az*(q2q3 - q0q1); 
	ace[2] = 2*ax*(q1q3 - q0q2) + 2*ay*(q2q3 + q0q1) + 2*az*(0.5f - q1q1 - q2q2);
    
	arm_sqrt_f32(ax*ax+ay*ay+az*az,&norm);
    
	if(norm<Semig)
		return ;
	ax/=norm;
	ay/=norm;
	az/=norm; 
	
	//计算重力加速度旋转到机体坐标系后的值,即以当前估计的姿态作为旋转矩阵
	gbx= 2*(q1q3 - q0q2);
	gby= 2*(q0q1 + q2q3);
	gbz= q0q0 - q1q1 - q2q2 + q3q3;
	
	//与实际加速度计测得的ax,ay,az做叉积，取误差
	
	
    ex = (ay*gbz - az*gby);                                                                
    ey = (az*gbx - ax*gbz);
    ez = (ax*gby - ay*gbx);	
	
	
    exInt += ex*IMU_I*INTEGRAL_CONSTANT;
    eyInt += ey*IMU_I*INTEGRAL_CONSTANT;
    ezInt += ez*IMU_I*INTEGRAL_CONSTANT;
    
    //补偿误差
    wx+=ex*IMU_P+exInt;
    wy+=ey*IMU_P+eyInt;
    wz+=ez*IMU_P+ezInt;
    
    
    //更新四元数
    q0=  q0 + (-q1*wx - q2*wy - q3*wz)*HALF_T;
    q1 = q1 + (q0*wx + q2*wz - q3*wy)*HALF_T;
    q2 = q2 + (q0*wy - q1*wz + q3*wx)*HALF_T;
    q3 = q3 + (q0*wz + q1*wy - q2*wx)*HALF_T; 
	
    //计算欧拉角
    arm_sqrt_f32(q0*q0+q1*q1+q2*q2+q3*q3,&norm);
    if(norm<Semig)
      return ;
    q0/=norm;
    q1/=norm;
    q2/=norm;
    q3/=norm;
    attitude[0]= asin(-2*q1*q3 + 2*q0*q2)* 57.3f;           //pitch
    attitude[1]= atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2+1)* 57.3f;   //roll
    attitude[2]= atan2(2*q1q2 + 2*q0q3, -2*q2q2 - 2*q3q3+1)* 57.3f;         //yaw
}
 
void AHR_Update(float * ac,float * gy,float * mag,float * attitude,float *ace,uint8_t use_mag){
    float ax,ay,az,wx,wy,wz,mx,my,mz;
	float norm;
	float gbx,gby,gbz;
    float q0q0 = q0 * q0;                                                        
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q1q1 = q1 * q1;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
	float q1q2=q1*q2;
	float q0q3=q0*q3;
	float ex,ey,ez;
//	float exInt,eyInt,ezInt;
	float hx,hy,hz,bx,bz;
	float mbx,mby,mbz;
    
    Accel_WFS[2]=Window_Filter(&Accel_Z_WFS,ac[2]);
    Accel_WFS[0]=Window_Filter(&Accel_X_WFS,ac[0]);
    Accel_WFS[1]=Window_Filter(&Accel_Y_WFS,ac[1]);
    
    az=Accel_WFS[2];
    ax=Accel_WFS[0];
    ay=Accel_WFS[1];

    //ax=ac[0];ay=ac[1];az=ac[2];
    wx=gy[0];wy=gy[1];wz=gy[2];
    mx=mag[0];my=mag[1];mz=mag[2];


        //计算加速度旋转到天地坐标系的值
    ace[0] = 2*ax*(0.5f - q2q2 - q3q3) + 2*ay*(q1q2 - q0q3) + 2*az*(q1q3 + q0q2); 
	ace[1] = 2*ax*(q1q2 + q0q3) + 2*ay*(0.5f - q1q1 - q3q3) + 2*az*(q2q3 - q0q1); 
	ace[2] = 2*ax*(q1q3 - q0q2) + 2*ay*(q2q3 + q0q1) + 2*az*(0.5f - q1q1 - q2q2);
    
    
    arm_sqrt_f32(ax*ax+ay*ay+az*az,&norm);
	if(norm<Semig)
		return ;
	ax/=norm;
	ay/=norm;
	az/=norm; 
	
    arm_sqrt_f32(mx*mx+my*my+mz*mz,&norm);
	if(norm<Semig)
		return ;
	mx/=norm;
	my/=norm;
	mz/=norm;
	
	//计算磁场旋转到天地坐标系的值
	hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2); 
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1); 
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2); 
    
    
    
    
    arm_sqrt_f32((hx*hx) + (hy*hy),&bx);
	bz = hz;
	
	//计算重力加速度旋转到机体坐标系后的值,即以当前估计的姿态作为旋转矩阵
	gbx= 2*(q1q3 - q0q2);
	gby= 2*(q0q1 + q2q3);
	gbz= q0q0 - q1q1 - q2q2 + q3q3;
	
	//计算磁场旋转到机体坐标系的值
	mbx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2); 
	mby = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3); 
	mbz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2);   
	
	//与实际加速度计测得的ax,ay,az做叉积，取误差
	//为什么（单位向量）叉积可以作为误差，因为A x B =|A|*|B|*sinSeita=sinSeita, sinSeita约等于Seita
	if(use_mag){
      ex = (ay*gbz - az*gby)+(my*mbz - mz*mby);                                                            
      ey = (az*gbx - ax*gbz)+(mz*mbx - mx*mbz);
      ez = (ax*gby - ay*gbx)+(mx*mby - my*mbx);
	}else{
      //此时，AHR算法退化为IMU算法
      ex = (ay*gbz - az*gby);                                                        
      ey = (az*gbx - ax*gbz);
      ez = (ax*gby - ay*gbx);      
    }
	
    exInt += ex*IMU_I*INTEGRAL_CONSTANT;
    eyInt += ey*IMU_I*INTEGRAL_CONSTANT;
    ezInt += ez*IMU_I*INTEGRAL_CONSTANT;
    
    //补偿误差
    wx+=ex*IMU_P+exInt;
    wy+=ey*IMU_P+eyInt;
    wz+=ez*IMU_P+ezInt;
	
    //更新四元数
    q0=  q0 + (-q1*wx - q2*wy - q3*wz)*HALF_T;
    q1 = q1 + (q0*wx + q2*wz - q3*wy)*HALF_T;
    q2 = q2 + (q0*wy - q1*wz + q3*wx)*HALF_T;
    q3 = q3 + (q0*wz + q1*wy - q2*wx)*HALF_T; 
	
    //计算欧拉角
    arm_sqrt_f32(q0*q0+q1*q1+q2*q2+q3*q3,&norm);
    if(norm<Semig)
      return ;
    q0/=norm;
    q1/=norm;
    q2/=norm;
    q3/=norm;

    attitude[0]= asin(-2*q1*q3 + 2*q0*q2)* 57.3f;           //pitch
    attitude[1]= atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2+1)* 57.3f;   //roll
    attitude[2]= atan2(2*q1q2 + 2*q0q3, -2*q2q2 - 2*q3q3+1)* 57.3f;         //yaw
    
}

/*
call_time：积分时间，单位ms
v ：全局变量数组，储存三轴速度
*/

float ace_sub=0;  //二阶互补滤波的补偿项，补偿在加速度上。

void Get_Velocity(float * ace,float *v,int call_time,float ace_sub){
  int i=0;
  for(i=0;i<2;++i){
    v[i]+=ace[i]*980*call_time/1000;   //*9.8m/s^2
  }
  v[2]+=(ace[2]-1)*980*call_time/1000+ace_sub*0.0025;  //对Z轴特殊处理，因为有重力加速度的影响
}

/*
height 必须指向一个全局变量
call_time 积分时间，单位ms
refer_height 参考高度，一般为气压计得到的高度
vz Z轴速度
*/
void  Get_Height(float *vz,float refer_height,int call_time,float *height){
  float sub;
  sub=refer_height-*height; //一阶互补滤波的补偿项，补偿在速度上
  *height+=(*vz)*call_time/1000+sub*0.02;
  ace_sub=sub;

}


/*
  惯性导航拟合函数
  call_time:调用周期 单位：ms   即惯性导航融合周期
*/
/*
#define K_TIME_CONSTANT  1.0f
float Numer_K_A_H=2.0f;
float Numer_K_V_H=7.0f;
float Numer_K_S_H=3.0f;
#define K_A_H   (Numer_K_A_H/(K_TIME_CONSTANT*K_TIME_CONSTANT*K_TIME_CONSTANT))
#define K_V_H   (Numer_K_V_H/(K_TIME_CONSTANT*K_TIME_CONSTANT))
#define K_S_H   (Numer_K_S_H/K_TIME_CONSTANT)
*/

#define K_TIME_CONSTANT  1.0f
float Numer_K_A_H=1.0f;
float Numer_K_V_H=5.0f;
float Numer_K_S_H=5.0f;
#define K_A_H   (Numer_K_A_H/(K_TIME_CONSTANT*K_TIME_CONSTANT*K_TIME_CONSTANT))
#define K_V_H   (Numer_K_V_H/(K_TIME_CONSTANT*K_TIME_CONSTANT))
#define K_S_H   (Numer_K_S_H/K_TIME_CONSTANT)

float ACE_Z_Offset=0.968;               //重力常量
typedef struct{
  float x;
  float y;
  float z;
}Position;

void set_k_h(int arg_num,char **s,float *args){
  if(arg_num!=0x0101){
    uprintf("error arg_num!\r\n");
    return ;
  }
  if(args[0]<0){
    uprintf("error K!\r\n");
    return ;
  }
  if(compare_string("a",s[0])){
    Numer_K_A_H=args[0];
  }else if(compare_string("v",s[0])){
    Numer_K_V_H=args[0];
  }else if(compare_string("s",s[0])){
    Numer_K_S_H=args[0];
  }
  uprintf("OK,set %s = %f\r\n",s[0],args[0]);
}
Position Quad_Position;
float Height_History_Buffer[20];
History_Buffer Height_HB={Height_History_Buffer,0,20,0};
float a_correct[3];
float v_correct[3],s_correct[3];
float inter_height[3];
float inter_v[3];
void Get_Position(float * ace,float *v,int call_time,float refer_height,float * Height){
  float height_sub=0;
  float a[3];

  float v_dealt[3];
  
  float dt=call_time/1000.0f;
  
  //height_sub=refer_height-*Height;
  
  if(!Height_HB.is_full){
    height_sub=refer_height-*Height;
  }else{
    height_sub=refer_height-HB_Get(&Height_HB,Height_HB.i-11);
  }
  
  HB_Push(&Height_HB,*Height);
 
  a_correct[2]+=height_sub*K_A_H*dt;           //a的微分
  v_correct[2]+=height_sub*K_V_H*dt;            //v的微分
  s_correct[2]+=height_sub*K_S_H*dt;            //s的微分
  
  a[2]=(ace[2]-ACE_Z_Offset)*980.0f+a_correct[2];   //zheli youwenti 
  
  v_dealt[2]=a[2]*dt;                           // dv=a*dt
  

  inter_height[2]+=(v[2]+0.5*v_dealt[2])*dt;        //intergral(v+dv)*d
  *Height=inter_height[2]+s_correct[2];
  
  inter_v[2]+=v_dealt[2];
  
  v[2]=inter_v[2]+v_correct[2];
  
  //v[2]+=a[2]*call_time/1000.0f+v_correct[2];
  
  //*Height+=v[2]*call_time/1000.0f+s_correct[2];
}

uint8_t ACE_Offset_Flag=0;

/*
  由于该函数要获取ACE的值，而ACE在2ms定时中断中解算，因此将此函书放在while中。
*/
void Get_Ace_Offset(){
  int cnt=0;
  float sum;
  if(!ACE_Offset_Flag){
    return ;
  }
  uprintf("OK,start to adjust ace_offset!\r\n");
  for(cnt=0;cnt<500;++cnt){
    sum+=Accel_E[2];
    HAL_Delay(3);
  }
  sum/=500;
  ACE_Z_Offset=sum;
  uprintf("adjust ace_offset ok!\r\n");
  uprintf("ace_offset:%f\r\n",sum);
  ACE_Offset_Flag=0;
}