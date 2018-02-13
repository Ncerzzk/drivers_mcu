#ifndef __MS5611_H
#define __MS5611_H

#include "stm32f4xx_hal.h"

// MS5611, Standard address 0x77
#define MS5611_ADDR                 0xEC
// Autodetect: turn off BMP085 while initializing ms5611 and check PROM crc to confirm device
#define BMP085_OFF                  digitalLo(BARO_GPIO, BARO_PIN);
#define BMP085_ON                   digitalHi(BARO_GPIO, BARO_PIN);

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8


typedef uint8_t (*i2c_write_buffer_fptr)(uint8_t slaveAddr, uint8_t writeAddr, uint8_t *pBuffer,uint16_t len);
typedef uint8_t (*i2c_read_buffer_fptr) (uint8_t slaveAddr, uint8_t writeAddr, uint8_t *pBuffer,uint16_t len);
typedef void (*i2c_err_reset_fptr) (void);
typedef void (*delay_ms_fptr)(uint32_t t);



#endif
