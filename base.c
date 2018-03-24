#include "base.h"
#include "angle.h"
#include "command.h"
#include "string.h"

char Send_Wave_Flag=0;
extern UART_HandleTypeDef huart2;
extern void uprintf(char *fmt, ...);

float * debug_wave[4];  //指针数组，指向要发送的四个波形
float wave_gain=1; //波形增益，因为要把float转为int，可能需要适当放大。

extern float CHn_Out[4];
extern float Angle_Speed[3],Accel[3],Angle[3],Accel_E[3],Velocity[3];

extern float pitch_target;
extern float MS5611_Height;

extern float Height;
extern float Accel_WFS[3];

char Wave_Set[4]; //波形设置,写入flash中

typedef struct{
  char * pram_string;
  void * pram_data;
  size_t size;
}pram_node;

#define PRAM_NUM    5
#define FLASH_Start 0x080A0000

pram_node Pram_Array[PRAM_NUM]={
  {"wave1",Wave_Set,sizeof(Wave_Set[0])},
  {"wave2",Wave_Set+1,sizeof(Wave_Set[0])},
  {"wave3",Wave_Set+2,sizeof(Wave_Set[0])},
  {"wave4",Wave_Set+3,sizeof(Wave_Set[0])},
  {"wave_g",&wave_gain,sizeof(wave_gain)}
};


#define WAVE_TYPE_NUM 24

typedef struct{
  char * wave_string;
  float * wave_ptr;
}wave_node;

wave_node Wave_Array[WAVE_TYPE_NUM]={
  {"wx",Angle_Speed+0},
  {"wy",Angle_Speed+1},
  {"wz",Angle_Speed+2},
  
  {"pitch",Angle+0},
  {"roll",Angle+1},
  {"yaw",Angle+2},
  
  {"ax",Accel+0},
  {"ay",Accel+1},
  {"az",Accel+2},
  
  {"c1",CHn_Out+0},
  {"c2",CHn_Out+1},
  {"c3",CHn_Out+2},
  {"c4",CHn_Out+3},
  
  {"ms_height",&MS5611_Height},
  
  {"axe",Accel_E+0},
  {"aye",Accel_E+1},
  {"aze",Accel_E+2},
  
  {"height",&Height},
  
  {"vx",Velocity+0},
  {"vy",Velocity+1},
  {"vz",Velocity+2},
  
  {"axw",Accel_WFS+0},
  {"ayw",Accel_WFS+1},
  {"azw",Accel_WFS+2}
  
  
  
};

/*
	最基本的波形发送函数。
*/
void send_wave(int data1,int data2,int data3,int data4){
	OutData[0]=data1;
	OutData[1]=data2;
	OutData[2]=data3;
	OutData[3]=data4;
	OutPut_Data();
}

/*
	波形发送函数，该函数可以通过设置debug_wave这个指针数组，来控制要发送的变量。
*/


void debug_send_wave(){
	OutData[0]=(int)(*debug_wave[0]* wave_gain);
	OutData[1]=(int)(*debug_wave[1]* wave_gain);
	OutData[2]=(int)(*debug_wave[2]* wave_gain);
	OutData[3]=(int)(*debug_wave[3]* wave_gain);
	OutPut_Data();
}


void set_debug_wave(int arg_num,char ** string_prams,float * float_prams){
	int index;
    char * string;
    int i;
    if(arg_num!=0x0101&&arg_num!=0x0100){
      uprintf("error arg_num!\r\n");
      return ;
    }
	index=(int)float_prams[0];
    string=string_prams[0];
	if(index<0||index>3){
		uprintf("error index!It must be 0-3!\r\n");
		return ;
	}
    
    for(i=0;i<WAVE_TYPE_NUM;++i){
      if(compare_string(Wave_Array[i].wave_string,string)){
        if(arg_num!=0x0100){    //仅提供通道，不提供波形设置，则不修改，只显示当前通道的波形
          debug_wave[index]=Wave_Array[i].wave_ptr;   //修改当前wave输出的指针
          Wave_Set[index]=i;                          //修改保存到prams的值
        }
        uprintf("OK,OutputData[%d] = %s!\r\n",index,Wave_Array[i].wave_string);
        return ;
      }
    }
}

void set_wave_gain(int arg_num,char **s,float * args){
	if(arg_num>0x0001){
		uprintf("error arg_nums!There must be 1 args!\r\n");
        return ;
	}
	if(args[0]<0){
		uprintf("error gain!It must bigger than 0!\r\n");
        return ;
	}
    if(arg_num){
      wave_gain=args[0];
    }
	uprintf("OK,wave_gain=%f\r\n",wave_gain);
}



float avarge(int *data,int count){
	int i=0;
	int sum=0;
	for(i=0;i<count;++i){
		sum+=data[i];
	}
	return (float)sum/(float)count;
}

/*
	卡尔曼滤波函数
*/

float KalMan(Kal_Struct *kal,float x){
	
	float kalman_pre;  //卡尔曼的预测值
	float cov_pre;  //卡尔曼预测值的协方差

	
	float kg;//增益
	kalman_pre=kal->kal_out*kal->A;  //计算本次卡尔曼的预测值
	
	cov_pre=kal->cov*kal->A*kal->A+kal->Q;
	
	kg=cov_pre/(cov_pre+kal->R);   //计算本次的卡尔曼增益
	
	kal->kal_out=kalman_pre+kg*(x-kalman_pre);   //通过预测值来计算本次卡尔曼滤波后的输出
	
	kal->cov=(1-kg)*cov_pre;
	
	return kal->kal_out;
}



float Window_Filter(Window_Filter_Struct * wfs,float data){
  int j;
  float sum=0;
  int count;
  
  if(!wfs->Window_Buffer)
    return 0;
  wfs->Window_Buffer[wfs->i]=data;
  wfs->i++;
  if(wfs->i==wfs->max){
    wfs->i=0; 
  }
  for(j=0;j<wfs->max;++j){
    sum+=wfs->Window_Buffer[j];
  }
  return sum/wfs->max;
}
/*
	控制是否发送波形的命令
*/
void set_send_wave_flag(int arg_num,char **s,float * arg){
	if(arg_num!=1&&arg_num!=0){
		uprintf("error arg num!\r\n");
		return ;
	}
	if(arg_num){
		Send_Wave_Flag=(int)arg[0];
	}else{
		Send_Wave_Flag=!Send_Wave_Flag;
	}
	uprintf("ok.Send_Wave_Flag=%d\r\n",Send_Wave_Flag);
}



void write_prams(int arg_num,char ** s,float * args){
	uint32_t SectorError;
	uint32_t temp;
	int i;
	FLASH_EraseInitTypeDef EraseInitStruct;
	
	if(arg_num!=0x00){
		uprintf("error arg_num!\r\n");
		return ;
	}
	
	HAL_FLASH_Unlock();
    
		
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = 9;   //FLASH_Start 当前位于第九页
  EraseInitStruct.NbSectors = 1;
	
	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  { 
		uprintf("erase flash fail!\r\n");
		HAL_FLASH_Lock();
		return ;
  }
	
	for(i=0;i<PRAM_NUM;++i){
		temp=*((uint32_t *)(Pram_Array[i].pram_data));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_Start+i*4,temp);
		uprintf("write Pram[%d]  %s Ok!\r\n",i,Pram_Array[i].pram_string);
	}
	HAL_FLASH_Lock();
	uprintf("Write OK!\r\n");
}

void load_prams(int arg_num,char ** s,float * args){
	int i;
	if(arg_num!=0x0000){
		uprintf("error arg_num!\r\n");
		return ;
	}
	
	for(i=0;i<PRAM_NUM;++i){
        memcpy(Pram_Array[i].pram_data,(void *)(FLASH_Start+i*4),Pram_Array[i].size);
		uprintf("Load Pram %dth %s:f:%f  d:%d\r\n",i,Pram_Array[i].pram_string,\
                                              *(float *)(Pram_Array[i].pram_data),\
                                              *(int *)(Pram_Array[i].pram_data));
	}	
	for(i=0;i<4;++i){
        debug_wave[i]=Wave_Array[*(char *)(Pram_Array[i].pram_data)].wave_ptr;           
	}
	
}

unsigned char Analize_GPS(char * raw_data,GPRMC * r){
    int i=0;
    char *ptr=0;
    for(ptr=strtok(raw_data,","),i=1;ptr!=0;ptr=strtok(0,","),++i){
        switch(i){
        case 2:
            r->UTC_TIME=atof(ptr);
            break;
        case 4:
            r->Lat=atof(ptr);
            break;
        case 5:
            if(*ptr=='N'){
                r->Lat*=1;
            }else{
                r->Lat*=-1;
            }
            break;
        case 6:
            r->Long=atof(ptr);
            break;
        case 7:
            if(*ptr=='E'){
                r->Long*=1;
            }else{
                r->Long*=-1;
            }
            break;
        default:
            break;
        }
    }
    if(i<7){
        return 0;
    }else{
        return 1;
    }
}
