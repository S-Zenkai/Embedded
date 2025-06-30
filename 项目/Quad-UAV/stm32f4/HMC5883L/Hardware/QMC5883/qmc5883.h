/**
  ******************************************************************************
  * @file    qmc5883.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/05/20
  * @brief   QMC5883L磁力计驱动
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __QMC5883_H
#define __QMC5883_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "i2c_sw.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "i2c_sw.h"

/* Exported define ------------------------------------------------------------*/

/*HMC5883L磁力计I2C地址*/
#define HMC5883L_I2C_ADDRESS 0x1E    

/*重新定HMC5883L_DEBUG函数*/
#define DEBUG_HMC5883L_ENABLE

#ifdef DEBUG_HMC5883L_ENABLE
#define HMC5883L_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define HMC5883L_DEBUG(format, ...)
#endif
/* Exported types ------------------------------------------------------------*/

/*HMC5883L寄存器地址*/
typedef enum
{
    /*Configuration Register A*/
    HMC5883L_REG_CONFIG_A = 0x00,
    /*Configuration Register B*/
    HMC5883L_REG_CONFIG_B = 0x01,
    /*Mode Register*/
    HMC5883L_REG_MODE = 0x02,
    /*Data Output X MSB Register*/
    HMC5883L_REG_DOUT_X_MSB = 0x03,
    /*Data Output X LSB Register*/
    HMC5883L_REG_DOUT_X_LSB = 0x04,
    /*Data Output Y MSB Register*/
    HMC5883L_REG_DOUT_Z_MSB = 0x05,
    /*Data Output Y LSB Register*/
    HMC5883L_REG_DOUT_Z_LSB = 0x06,
    /*Data Output Z MSB Register*/
    HMC5883L_REG_DOUT_Y_MSB = 0x07,
    /*Data Output Z LSB Register*/
    HMC5883L_REG_DOUT_Y_LSB = 0x08,
    /*Status Register*/
    HMC5883L_REG_STATUS = 0x09,
    /*Chip ID Register*/
    HMC5883L_REG_CHIP_IDA = 0x0A,
    /*Chip ID Register*/
    HMC5883L_REG_CHIP_IDB = 0x0B,
    /*Chip ID Register*/
    HMC5883L_REG_CHIP_IDC = 0x0C,
}HMC5883L_Reg_t;

/*HMC5883L增益(LSB/Gauss)*/
typedef enum
{
    HMC5883L_GAIN_1370 = 0x00,
    HMC5883L_GAIN_1090 = 0x20,
    HMC5883L_GAIN_820 = 0x40,
    HMC5883L_GAIN_660 = 0x60,
    HMC5883L_GAIN_440 = 0x80,
    HMC5883L_GAIN_390 = 0xA0,
    HMC5883L_GAIN_330 = 0xC0,
    HMC5883L_GAIN_230 = 0xE0,
}HMC5883L_Gain_t;

/*HMC5883L数据结构体*/
typedef struct
{
    float x;
    float y;
    float z;
}HMC5883L_Data_t;


/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void hmc5883l_init(void);
void hmc5883l_single_measurement(void);
void hmc5883l_read_data(HMC5883L_Data_t* data);
bool i2c_master_read_data(uint8_t reg_addr, uint8_t *data, uint8_t length);
#endif /* __QMC5883_H */
