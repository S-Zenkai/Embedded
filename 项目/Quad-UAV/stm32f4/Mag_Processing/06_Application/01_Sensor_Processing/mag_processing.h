/**
  ******************************************************************************
  * @file    mag_processing.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/09/03
  * @brief   磁力计数据处理
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAG_PROCESSING_H
#define __MAG_PROCESSING_H
/* Includes ------------------------------------------------------------------*/
#include "pro_config.h"



#define DEBUG_MAG_PROCESSING_ENABLE
#ifdef DEBUG_MAG_PROCESSING_ENABLE
#define MAG_PROCESSING_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define MAG_PROCESSING_DEBUG(format, ...)
#endif

#define M_PI 3.14159265f

/*地磁校准函数调用频率*/
#define MAG_CALIBRATION_FREQ 50.0f
/*地磁校准函数调用时间*/
#define MAG_CALIBRATION_TIME (1.0f / MAG_CALIBRATION_FREQ)

void MAG_Processing_Init(void);
void MAG_Processing(MAG_CalData_t *mag_cal_data, MAG_GaussData_t *mag_gauss_data);

#endif /* __MAG_PROCESSING_H */
