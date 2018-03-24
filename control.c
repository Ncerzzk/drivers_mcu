#include "control.h"
#include "angle.h"
#include "tim.h"
#include "command.h"

#define HEIGHT_T  120
#define UP_DUTY			60

enum attitude{
  PITCH=0x00,
  ROLL=0x01,
  YAW=0x02,
  X=0x00,
  Y=0x01,
  Z=0x02
};


//期望的俯仰、横滚、偏航角
float pitch_target=0;     //3.25
float roll_target=6;  //48.5
float yaw_target=0;
float height_target=50;   //单位cm
float height_offset=0;  
float Angle_Speed_X_Out=0;
float Angle_Speed_Y_Out=0;
float Angle_Speed_Z_Out=0;

float Accel_Speed_Z_Out=0; //加速度环 高度环的内环

extern float Angle[3];
extern float Angle_Speed[3];
extern float MS5611_Height;
extern float Velocity[3];
extern float Height;

float base_duty=0; //50
float CHn_Out[4];

//PID 结构体参数
//带Single是单级PID

PID_S ROLL_PID={-0.015,-0.045,0,0,0,2};    //{0.1,0,0,0,0,2}
PID_S PITCH_PID={0.025,0.075,0,0,0,2}; //{0.1,0,0,0,0,2}
PID_S YAW_PID={-0.1,0,0,0,0,2};
	
PID_S ANGLE_SPEED_Y_PID={-30,-100,-1,0,0,20};  //{-10,-20,-1,0,0,5}
PID_S ANGLE_SPEED_X_PID={-30,-100,-1,0,0,20};
PID_S ANGLE_SPEED_Z_PID={-10,-20,0,0,0,20}; //{-20,-20,0,0,0,5};

PID_S Height_PID={0.8,0,0,0,0,9000};  //0.3
PID_S ACCEL_SPEED_Z_PID={7,0,0,0,0,2};
PID_S Velocity_PID={0,0,0,0,0,1000};

PID_S X_PID={0.1,0,0,0,0,9000};
PID_S Y_PID={0.1,0,0,0,0,9000};


Fly_State  State=STOP;

int position_x,position_y;

void set_flag_command(char * flag,int arg_num,float * args);
void Fly_Stop(void);

static float * get_pid_ptr(char **s);


void Fly_Init(void){      //解锁飞行，初始化高度、yaw
	State=FLY_WAIT;
	base_duty=0;           //FLY_WAIT状态，占空比可以手动更改
	ROLL_PID.i=0;
	PITCH_PID.i=0;
	ANGLE_SPEED_Y_PID.i=0;
	ANGLE_SPEED_X_PID.i=0;
	ANGLE_SPEED_Z_PID.i=0;
    
    Height_PID.i=0;
    yaw_target=Angle[YAW];
    height_offset=Height;
}
 

void set_fly(int arg_num,char **s,float * args){
	if(arg_num>0){
		uprintf("error arg_num!\r\n");
	}
	if(State==STOP)
		Fly_Init();
	else
		Fly_Stop();
	uprintf("OK,change Fly!\r\n");
}

void set_stop(int arg_num,char **s,float * args){
	if(arg_num>0){
		uprintf("error arg_num!\r\n");
	}
	Fly_Stop();
	uprintf("OK,Stop!\r\n");
}



//set_pid wx p 100
void set_pid(int arg_num,char **s,float * args){
  float *pid_ptr;
  if(arg_num!=0x0201){
    uprintf("error arg_num!\r\n");
    return ;
  }
  pid_ptr=get_pid_ptr(s);
  
  if(pid_ptr==0){
    uprintf("error control type or error pid type!\r\n");
    return ;
  }
  *pid_ptr=args[0];
  
  uprintf("OK ,set %s %s = %f\r\n",s[0],s[1],*pid_ptr);
}

void add_pid(int arg_num,char **s,float *args){
 float *pid_ptr;
  if(arg_num!=0x0201){
    uprintf("error arg_num!\r\n");
    return ;
  }
  pid_ptr=get_pid_ptr(s);
  
  if(pid_ptr==0){
    uprintf("error control type or error pid type!\r\n");
    return ;
  }
  *pid_ptr=*pid_ptr+args[0];
  
  uprintf("OK ,set %s %s = %f\r\n",s[0],s[1],*pid_ptr);  
}

static float * get_pid_ptr(char **s){
  char * control_type=s[0];
  char * pid_type=s[1];
  PID_S *ptr=0;
  float *float_ptr=0;
  if(compare_string("wx",control_type)){
    ptr=&ANGLE_SPEED_X_PID;
  }else if(compare_string("wy",control_type)){
    ptr=&ANGLE_SPEED_Y_PID;
  }else if(compare_string("wz",control_type)){
    ptr=&ANGLE_SPEED_Z_PID;
  }else if(compare_string("roll",control_type)){
    ptr=&ROLL_PID;
  }else if(compare_string("pitch",control_type)){
    ptr=&PITCH_PID;
  }else if(compare_string("yaw",control_type)){
    ptr=&YAW_PID;
  }else if(compare_string("az",control_type)){
    ptr=&ACCEL_SPEED_Z_PID;
  }else if(compare_string("v",control_type)){
    ptr=&Velocity_PID;
  }else if(compare_string("h",control_type)){
    ptr=&Height_PID;
  }else if(compare_string("x",control_type)){
    ptr=&X_PID;
  }else if(compare_string("y",control_type)){
    ptr=&Y_PID;
  }else{
    return 0;
  }
  
  if(compare_string("p",pid_type)){
    float_ptr=&(ptr->KP);
  }else if(compare_string("i",pid_type)){
    float_ptr=&(ptr->KI);
  }else if(compare_string("d",pid_type)){
    float_ptr=&(ptr->KD);
  }else if(compare_string("i_max",pid_type)){
    float_ptr=&(ptr->i_max);
  }else{
    return 0;
  }
  return float_ptr;
  
}

inline void Fly_Stop(void){
	State=STOP;
}


float PID_Control(PID_S *PID,float target,float now){
	float err;
	float err_dt;
	float result;

	err=target-now;
	
	err_dt=err-PID->last_err;
	
	
	err_dt*=0.384f;
	err_dt+=PID->last_d*0.615f;   //低通滤波
	
	
	PID->last_err=err;
	
	PID->i+=err*I_TIME;
	
	Limit(PID->i,PID->i_max);
	PID->last_d=err_dt;
	
	result = err * PID->KP  +   err_dt * PID->KD   +   PID->i * PID->KI;
	return result;
}


//set_target roll 10
void set_target(int arg_num,char **s,float * args){
  float * ptr=0;
  if(arg_num!=0x0101&&arg_num!=0x0100){
    uprintf("error arg_num!\r\n");
    return ;
  }
  if(compare_string("roll",s[0])){
    ptr=&roll_target;
  }else if(compare_string("pitch",s[0])){
    ptr=&pitch_target;
  }else if(compare_string("yaw",s[0])){
    ptr=&yaw_target;
  }else if(compare_string("height",s[0])){
    ptr=&height_target;
  }else{
    uprintf("error target!\r\n");
    return ;
  }
  
  if(arg_num==0x0101){
    *ptr=args[0];
  }
  uprintf("OK,%s_target = %f\r\n",s[0],*ptr); 
}


char Angle_Speed_Z_Flag=1;
char Roll_Pitch_Flag=1;
char Motor_Open_Flag=1;
char Height_Open_Flag=0;

inline void set_flag_command(char * flag,int arg_num,float * args){
	if(arg_num>1){
		uprintf("error arg num!\r\n");
		return ;
	}
	if(arg_num){
		*flag=(int)args[0];
	}else{
		*flag=!(*flag);
	}
	uprintf("ok,set flag=%d\r\n",*flag);
}

void add_duty(int arg_num,char **s ,float *args){
  if(arg_num!=0x0001){
    uprintf("error arg_num!\r\n");
    return ;
  }
  base_duty+=args[0];
  Limit(base_duty,100);
  uprintf("OK,base_duty=%f\r\n",base_duty);
}
void set_flag(int arg_num,char **s,float * args){
  if(arg_num!=0x0101&&arg_num!=0x0100){
    uprintf("error arg_num!\r\n");
    return ;
  }
  if(compare_string("az",s[0])){
    set_flag_command(&Angle_Speed_Z_Flag,arg_num&0x0f,args);
  }else if(compare_string("roll_pitch",s[0])){
    set_flag_command(&Roll_Pitch_Flag,arg_num&0x0f,args);
  }else if(compare_string("motor",s[0])){
    set_flag_command(&Motor_Open_Flag,arg_num&0x0f,args);
  }else if(compare_string("height",s[0])){
    set_flag_command(&Height_Open_Flag,arg_num&0x0f,args);
  }
}

float get_yaw_err(float now,float target){
  float sub,a_sub,result;
  //正方向：逆时针
  sub=target-now;
  a_sub=fabs(sub);
  if(a_sub>=180){
    a_sub=360-a_sub;
    result=-a_sub;
  }else{
    result=sub;
  }
  
  return result;
  
}

/*
thrust 油门
direction 方向舵
ele 升降舵
aile 副翼
*/
void RC_Set_Target(float thrust,float direction,float ele,float aile){
  if(thrust<1 && direction<1 && ele<1 && aile >95){
    Fly_Init();
    uprintf("set_fly!\r\n");
  }else if(thrust <1 && ele <1 && direction>47 &&direction<51){
    Fly_Stop();
    uprintf("set_stop!\r\n");
  }
  base_duty=thrust;
  
  if(State!=STOP){
    if(base_duty<20){
      base_duty=20;
    } 
  }
  roll_target=(10*ele/100-5);
  pitch_target=-(10*aile/100-5);
}

Steer ELE_Steer={TIM3,3,1,2};
Steer AILE_Steer={TIM3,2,1,2.6};
void Fly_Control(){	
  float Roll_Out,Pitch_Out,Yaw_Out;
  Yaw_Out=PID_Control(&YAW_PID,0,get_yaw_err(Angle[Z],yaw_target));

  Roll_Out=PID_Control(&ROLL_PID,roll_target,Angle[ROLL]); 
  Pitch_Out=PID_Control(&PITCH_PID,pitch_target,Angle[PITCH]);
  
  if(Roll_Pitch_Flag){
    Set_Steer(&ELE_Steer,Roll_Out);
    Set_Steer(&AILE_Steer,Pitch_Out);
  }
}





/*
width:脉宽 单位:ms
*/

