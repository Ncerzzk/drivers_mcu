#include "mpu9250.h"
#include "usart.h"

#include "string.h"
#include "stdlib.h"



MPU_Dev MPU9250;
MPU_Data MPU9250_Data;
MPU_Setting MPU9250_Setting;

/*
下面三个函数需用户自己实现。
*/
extern I2C_HandleTypeDef hi2c3;
static uint8_t MPU_Write_Buffer(uint8_t slaveAddr, uint8_t writeAddr, uint8_t *pBuffer,uint16_t len);
static uint8_t MPU_Read_Buffer(uint8_t slaveAddr,uint8_t readAddr,uint8_t *pBuffer,uint16_t len);
static void MPU_I2C_Reset(void);

void mpu_error_deal(MPU_Dev *dev);



static void mpu_write_byte(MPU_Dev *dev,uint8_t write_addr,uint8_t data){
	if(dev->i2c_write_buffer(dev->dev_addr,write_addr,&data,1)!=dev->I2C_OK){
		mpu_error_deal(dev);
	}
}



inline void mpu_error_deal(MPU_Dev *dev){
		dev->i2c_error_count++;
		if(dev->i2c_error_count>20){
			dev->i2c_reset();
			dev->i2c_error_count=0;
		}
}

static uint8_t mpu_read_byte(MPU_Dev *dev,uint8_t read_addr){
	uint8_t temp=0;
	if(dev->i2c_read_buffer(dev->dev_addr,read_addr,&temp,1)!=dev->I2C_OK){
		mpu_error_deal(dev);
	}
	return temp;
}

void MPU_Read6500(MPU_Dev *dev){
	uint8_t temp[6];
	dev->i2c_read_buffer(dev->dev_addr,ACCEL_XOUT_H,temp,6);
	dev->data->ac_x=(temp[0]<<8)|temp[1];
	dev->data->ac_y=(temp[2]<<8)|temp[3];
	dev->data->ac_z=(temp[4]<<8)|temp[5];
	dev->i2c_read_buffer(dev->dev_addr,GYRO_XOUT_H,temp,6);
	dev->data->gy_x=(temp[0]<<8)|temp[1];
	dev->data->gy_y=(temp[2]<<8)|temp[3];
	dev->data->gy_z=(temp[4]<<8)|temp[5];	
}

void MPU_ReadM_Mag(MPU_Dev *dev){
	uint8_t temp[6];
	dev->i2c_read_buffer(dev->dev_addr,MAG_XOUT_L,temp,6);
	dev->data->m_x=(temp[1]<<8)|temp[0];
	dev->data->m_y=(temp[3]<<8)|temp[2];
	dev->data->m_z=(temp[5]<<8)|temp[4];
}


void MPU9250_Init(MPU_Dev * dev){
	dev->dev_addr=MPU_Address;
	dev->i2c_read_buffer=MPU_Read_Buffer;
	dev->i2c_write_buffer=MPU_Write_Buffer;
	dev->delay_ms=HAL_Delay;
	
	dev->I2C_OK=0x00;
	dev->i2c_reset=MPU_I2C_Reset;
	dev->data=&MPU9250_Data;
	dev->setting=&MPU9250_Setting;
	
	dev->dev_mag_addr=MAG_Address;
	
	dev->setting->accel_range_setting=RANGE2G;
	dev->setting->accel_high_pass_filter_setting=_5HZ;
	dev->setting->gyro_range_setting=RANGE500;
	
//	I2C_ByteWrite(0xEE,0x1E,1);
	MPU_INIT:
	
	HAL_Delay(50);
	
	mpu_write_byte(dev,PWR_MGMT_1,0x00);
	mpu_write_byte(dev,SMPLRT_DIV, 0x00);
	mpu_write_byte(dev,CONFIG, 0x02);  //之前延时为20ms(0x06，现在为3ms左右 0x02)

	mpu_write_byte(dev,GYRO_CONFIG, GYRO_Range_Configure);   //
	mpu_write_byte(dev,ACCEL_CONFIG, dev->setting->accel_range_setting | dev->setting->accel_high_pass_filter_setting); 
	
	switch(dev->setting->gyro_range_setting){
		case RANGE250:
			dev->setting->gyro_range=250;
			break;
		case RANGE500:
			dev->setting->gyro_range=500;
			break;
		case RANGE1000:
			dev->setting->gyro_range=1000;
			break;
		case RANGE2000:
			dev->setting->gyro_range=2000;
			break;
	}
	
	switch(dev->setting->accel_range_setting){
		case RANGE2G:
			dev->setting->accel_range=2.0f;
			break;
		case RANGE4G:
			dev->setting->accel_range=4.0f;
			break;	
		case RANGE8G:
			dev->setting->accel_range=8.0f;
			break;	
		case RANGE16G:
			dev->setting->accel_range=16.0f;
			break;		
	}
	
	
	
	mpu_write_byte(dev,INT_PIN_CFG,0x02);    //MPU6500 开启路过模式
	
	dev->dev_ID=mpu_read_byte(dev,WHO_AM_I);
	
	if(dev->dev_ID!=0x71){
		dev->i2c_reset();
		goto MPU_INIT;
	}
	

	
}


static uint8_t MPU_Write_Buffer(uint8_t slaveAddr, uint8_t writeAddr, uint8_t *pBuffer,uint16_t len){
	uint8_t * data;
	uint8_t result;
	data=(uint8_t *)malloc((len+1)*sizeof(uint8_t));
	memcpy(data+1,pBuffer,len);
	data[0]=writeAddr;
	result=HAL_I2C_Master_Transmit(&hi2c3,slaveAddr,data,len+1,200);
	free(data);
	return result;
	 
}

static uint8_t MPU_Read_Buffer(uint8_t slaveAddr,uint8_t readAddr,uint8_t *pBuffer,uint16_t len){
	HAL_I2C_Master_Transmit(&hi2c3,slaveAddr,&readAddr,1,200);
	return HAL_I2C_Master_Receive(&hi2c3,slaveAddr,pBuffer,len,200);		 
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c);
extern I2C_HandleTypeDef hi2c;

static void MPU_I2C_Reset(){
	HAL_I2C_MspDeInit(&hi2c);
	HAL_I2C_MspInit(&hi2c);
}

//将陀螺仪直接读出的16位符号数归一为 °/s
float Gyro_Normalize(int16_t gyro){
	return ((float)gyro)*GYRO_Range/32768.0f;
}





