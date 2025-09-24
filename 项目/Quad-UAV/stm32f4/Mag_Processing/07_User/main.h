/**
  ******************************************************************************
  * @file    main.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/02/28
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
/* Includes ------------------------------------------------------------------*/
#include "pro_config.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "bsp_spi.h"
#include "bsp_mpu6000.h"
#include "bsp_tim.h"
#include "bsp_exti.h"
#include "filter.h"
#include "bsp_ms5611.h"
#include "bsp_dma.h"
#include "ubx.h"
#include "hmc5883l.h"
#include "i2c_sw.h"
#include "mavlink_handle.h"
#include "mavlink.h"
#include "ic_comm.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/*使能各模块测试程序*/
#define         MPU6000_Test                0
/* Exported functions ------------------------------------------------------- */
#endif /* __MAIN_H */
