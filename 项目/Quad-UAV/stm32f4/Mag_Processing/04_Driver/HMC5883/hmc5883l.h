/**
  ******************************************************************************
  * @file    hmc5883l.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/05/20
  * @brief   HMC5883L磁力计驱动
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HMC5883L_H
#define __HMC5883L_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "i2c_sw.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "i2c_sw.h"
#include "com_data.h"

/* Exported define ------------------------------------------------------------*/

/*HMC5883L磁力计I2C地址*/
#define HMC5883L_I2C_ADDRESS 0x1E    

/*重新定HMC5883L_DEBUG函数*/
//#define DEBUG_HMC5883L_ENABLE

#ifdef DEBUG_HMC5883L_ENABLE
#define HMC5883L_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define HMC5883L_DEBUG(format, ...)
#endif
/* Exported types ------------------------------------------------------------*/

// HMC5883L 寄存器地址
#define HMC5883L_REG_CONFIG_A       0x00
#define HMC5883L_REG_CONFIG_B       0x01
#define HMC5883L_REG_MODE           0x02
#define HMC5883L_REG_DATA_OUT_X_MSB 0x03
#define HMC5883L_REG_DATA_OUT_X_LSB 0x04
#define HMC5883L_REG_DATA_OUT_Z_MSB 0x05 // 注意：顺序是 X, Z, Y
#define HMC5883L_REG_DATA_OUT_Z_LSB 0x06
#define HMC5883L_REG_DATA_OUT_Y_MSB 0x07
#define HMC5883L_REG_DATA_OUT_Y_LSB 0x08
#define HMC5883L_REG_STATUS         0x09
#define HMC5883L_REG_ID_A           0x0A
#define HMC5883L_REG_ID_B           0x0B
#define HMC5883L_REG_ID_C           0x0C


// 寄存器 CONFIG_A (0x00)
typedef enum {
    HMC5883L_SAMPLES_1 = 0x00, // 平均采样次数
    HMC5883L_SAMPLES_2 = 0x20,
    HMC5883L_SAMPLES_4 = 0x40,
    HMC5883L_SAMPLES_8 = 0x60
} HMC5883L_Samples_t;

typedef enum {
    HMC5883L_RATE_0_75 = 0x00, // 数据输出速率 (Hz)
    HMC5883L_RATE_1_5  = 0x04,
    HMC5883L_RATE_3    = 0x08,
    HMC5883L_RATE_7_5  = 0x0C,
    HMC5883L_RATE_15   = 0x10, // 默认
    HMC5883L_RATE_30   = 0x14,
    HMC5883L_RATE_75   = 0x18
} HMC5883L_Rate_t;


/*寄存器 CONFIG_B,HMC5883L增益(LSB/Gauss)*/
typedef enum {
    HMC5883L_GAIN_1370 = 0x00, // Range: ±0.88 Ga, LSB/Gauss: 1370
    HMC5883L_GAIN_1090 = 0x20, // Range: ±1.3  Ga, LSB/Gauss: 1090 (默认)
    HMC5883L_GAIN_820  = 0x40, // Range: ±1.9  Ga, LSB/Gauss: 820
    HMC5883L_GAIN_660  = 0x60, // Range: ±2.5  Ga, LSB/Gauss: 660
    HMC5883L_GAIN_440  = 0x80, // Range: ±4.0  Ga, LSB/Gauss: 440
    HMC5883L_GAIN_390  = 0xA0, // Range: ±4.7  Ga, LSB/Gauss: 390
    HMC5883L_GAIN_330  = 0xC0, // Range: ±5.6  Ga, LSB/Gauss: 330
    HMC5883L_GAIN_230  = 0xE0  // Range: ±8.1  Ga, LSB/Gauss: 230
} HMC5883L_Gain_t;

// 寄存器 MODE (0x02)
#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

/*HMC5883L原始数据结构体*/
typedef Axis3_i16_u HMC5883L_RawData_t;
/*HMC5883L转换后的数据结构体*/
typedef Axis3_f_u HMC5883L_GaussData_t;
/*HMC5883L校准数据结构体*/
typedef Axis3_f_u HMC5883L_CalData_t;



/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
bool hmc5883l_init(HMC5883L_Gain_t gain, HMC5883L_Rate_t rate, HMC5883L_Samples_t samples);
bool hmc5883l_read_raw_data(HMC5883L_RawData_t *data);
void hmc5883l_convert_to_gauss(HMC5883L_GaussData_t *gauss_data, const HMC5883L_RawData_t *raw_data);
bool hmc5883l_check_id(void);
#endif /* __HMC5883L_H */
