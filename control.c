#include "control.h"
#include "angle.h"
#include "tim.h"
#include "command.h"

#define HEIGHT_T  120
#define UP_DUTY			60
//期望的俯仰、横滚、偏航角
float pitch_target=0;     //3.25
float roll_target=0;  //48.5
float yaw_target=0;
float yaw_offset=0;
float yaw_dealed=0;
float height_target=0;   //单位cm
float Angle_Speed_X_Out=0;
float Angle_Speed_Y_Out=0;
float Angle_Speed_Z_Out=0;
float Accel_Speed_Z_Out=0; //加速度环 高度环的内环
float Height_Out=0;
float Pitch_Out=0;
float Roll_Out=0;
float Yaw_Out=0;
float Velocity_Out=0;
char Fly=0;

extern float Angle[3];
extern float Angle_Speed[3];
float height_offset; 		//高度初始值

float base_duty=0; //50
float CHn_Out[4];

//PID 结构体参数
//带Single是单级PID

PID_S ROLL_PID={0,0,0,0,0,2};    //{0.1,0,0,0,0,2}
PID_S PITCH_PID={0,0,0,0,0,2}; //{0.1,0,0,0,0,2}
PID_S YAW_PID={-0.25,0,0,0,0,2};
	
PID_S ANGLE_SPEED_Y_PID={0,0,0,0,0,5};  //{-10,-20,-1,0,0,5}
PID_S ANGLE_SPEED_X_PID={0,0,0,0,0,5};
PID_S ANGLE_SPEED_Z_PID={0,0,0,0,0,5}; //{-20,-20,0,0,0,5};

PID_S Height_PID={0.8,0,0,0,0,9000};  //0.3
PID_S ACCEL_SPEED_Z_PID={7,0,0,0,0,2};
PID_S Velocity_PID={0.32,10,0.77,0,0,1000};

PID_S X_PID={0.1,0,0,0,0,9000};
PID_S Y_PID={0.1,0,0,0,0,9000};


Fly_State  State=STOP;

Kal_Struct kal_acc_z={1,0,0.00001,0.0012,0,1};

int position_x,position_y;

void set_flag_command(char * flag,int arg_num,float * args);
void Fly_Stop(void);

int yaw_offset_init_ok=0;

float * get_pid_ptr(char **s);


void Fly_Init(void){      //解锁飞行，初始化高度、yaw
	State=FLY_WAIT;
	if(!yaw_offset_init_ok)
		yaw_offset=Angle[2];
	base_duty=0;           //FLY_WAIT状态，占空比可以手动更改
	ROLL_PID.i=0;
	PITCH_PID.i=0;
	ANGLE_SPEED_Y_PID.i=0;
	ANGLE_SPEED_X_PID.i=0;
	ANGLE_SPEED_Z_PID.i=0;
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
float * get_pid_ptr(char **s){
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
  }
  
  if(compare_string("p",pid_type)){
    float_ptr=&(ptr->KP);
  }else if(compare_string("i",pid_type)){
    float_ptr=&(ptr->KI);
  }else if(compare_string("d",pid_type)){
    float_ptr=&(ptr->KD);
  }else if(compare_string("i_max",pid_type)){
    float_ptr=&(ptr->i_max);
  }
  return float_ptr;
  
}
void Fly_Up(void){		//起飞
	
	if(HEIGHT_T>70){
		height_target=70;
	}else{
		height_target=HEIGHT_T;
	}  
	State=FLY;
	base_duty=UP_DUTY;
	
}
inline void Fly_Stop(void){
	State=STOP;
	yaw_offset_init_ok=0;
}

/*
void Fly_Land(void){
	State=LAND;
	roll_target=0;
	pitch_target=0;
	//roll_target=ROLL_CONSTANT;
	//pitch_target=PITCH_CONSTANT;
}

*/
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


/*
	这个函数用来处理yaw.
	yaw_offset为刚起飞时候的yaw角度
	返回当前yaw与起飞时的yaw的偏差
*/

float yaw_deal(){
	float temp;
	float result;    //返回处理过的yaw
	temp=Angle[2]-yaw_offset;
	if(temp>180||temp<-180){
		if(temp>0){
			result=temp-360;
		}else{
			result=temp+360;
		}
	}else{
		result=temp;
	} 
	
	if(result>180||result<-180){     //仍然大于或者小于180度，显然有错。
			result=0;
	}
	return result;
}
enum attitude{
  PITCH=0x00,
  ROLL=0x01,
  YAW=0x02,
  X=0x00,
  Y=0x01,
  Z=0x02
};


inline void Motor_Stop(){
  for(int i=1;i<=4;++i){
	Set_Speed(i,0);
  }
}

char Angle_Speed_Z_Flag=0;
char Roll_Pitch_Flag=0;
char Motor_Open_Flag=0;

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
  }
}

void Fly_Control(){		
  
  if(State==STOP){
    Motor_Stop();
    return ;
  }
  
  if(State!=STOP){
    yaw_dealed=yaw_deal();
    
    Yaw_Out=PID_Control(&YAW_PID,yaw_target,yaw_dealed);
    if(Angle_Speed_Z_Flag)
      Angle_Speed_Z_Out=PID_Control(&ANGLE_SPEED_Z_PID,Yaw_Out,Angle_Speed[Z]);
    else
      Angle_Speed_Z_Out=0;
  }
  
  if(Roll_Pitch_Flag){ //外环开启
    Roll_Out=PID_Control(&ROLL_PID,roll_target,Angle[ROLL]);     //如果是调角速度内环，注释掉这句
    Limit(Roll_Out,10);
    Pitch_Out=PID_Control(&PITCH_PID,pitch_target,Angle[PITCH]);  //如果是调角速度内环，注释掉这句
    Limit(Pitch_Out,10);
  }else{//外环关闭
    Roll_Out=0;
    Pitch_Out=0;
  }
  
  Angle_Speed_X_Out=PID_Control(&ANGLE_SPEED_X_PID,Roll_Out,Angle_Speed[X]);
  Angle_Speed_Y_Out=PID_Control(&ANGLE_SPEED_Y_PID,Pitch_Out,Angle_Speed[Y]);
  
  //串级PID
  CHn_Out[0]=Angle_Speed_X_Out-Angle_Speed_Y_Out+Angle_Speed_Z_Out+base_duty+Velocity_Out;  //以角度向前倾，向左倾为标准
  CHn_Out[1]=-Angle_Speed_X_Out-Angle_Speed_Y_Out-Angle_Speed_Z_Out+base_duty+Velocity_Out;
  CHn_Out[2]=-Angle_Speed_X_Out+Angle_Speed_Y_Out+Angle_Speed_Z_Out+base_duty+Velocity_Out;
  CHn_Out[3]=Angle_Speed_X_Out+Angle_Speed_Y_Out-Angle_Speed_Z_Out+base_duty+Velocity_Out;
  
  if(Motor_Open_Flag){
    for(int i=1;i<5;++i){ 
      Set_Speed(i,CHn_Out[i-1]);
    }
  }
}

