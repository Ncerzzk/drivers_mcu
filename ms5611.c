#include "ms5611.h"

static uint32_t ms5611_ut;  // static result of temperature measurement
static uint32_t ms5611_up;  // static result of pressure measurement
static uint16_t ms5611_c[PROM_NB];  // on-chip ROM
static uint8_t ms5611_osr = CMD_ADC_4096;




typedef struct{
  	i2c_write_buffer_fptr i2c_write_buffer;
	i2c_read_buffer_fptr i2c_read_buffer;
	i2c_err_reset_fptr i2c_reset;
	delay_ms_fptr delay_ms;
    uint8_t address;
}MS5611_Dev;


extern I2C_HandleTypeDef hi2c3;
uint8_t I2C_Write_Buffer(uint8_t slaveAddr, uint8_t writeAddr, uint8_t *pBuffer,uint16_t len);
uint8_t I2C_Read_Buffer(uint8_t slaveAddr,uint8_t readAddr,uint8_t *pBuffer,uint16_t len);
void I2C_Reset(void);
void HAL_Delay(__IO uint32_t Delay);
/*
void MS5611_Init(MS5611_Dev * dev){
  dev->i2c_read_buffer=I2C_Read_Buffer;
  dev->i2c_write_buffer=I2C_Write_Buffer;
  dev->i2c_reset=I2C_Reset();
  dev->delay_ms=HAL_Delay();
  
  dev->address=MS5611_ADDR;
  
  

  for (i = 0; i < PROM_NB; i++)
    ms5611_c[i] = ms5611_prom(i);
}

static uint16_t ms5611_prom(MS5611_Dev * dev,int8_t coef_num)
{
  uint8_t rxbuf[2] = { 0, 0 };
  dev->i2c_read_buffer(dev->address,CMD_PROM_RD+coef_num * 2,rxbuf,2);
  return rxbuf[0] << 8 | rxbuf[1];
}


static void ms5611_calculate(int32_t *pressure, int32_t *temperature)
{
    uint32_t press;
    int64_t temp;
    int64_t delt;
    int32_t dT = (int64_t)ms5611_ut - ((uint64_t)ms5611_c[5] * 256);
    int64_t off = ((int64_t)ms5611_c[2] << 16) + (((int64_t)ms5611_c[4] * dT) >> 7);
    int64_t sens = ((int64_t)ms5611_c[1] << 15) + (((int64_t)ms5611_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (int64_t)ms5611_c[6]) >> 23);

    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    }
    press = ((((int64_t)ms5611_up * sens) >> 21) - off) >> 15;

    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;
}
*/