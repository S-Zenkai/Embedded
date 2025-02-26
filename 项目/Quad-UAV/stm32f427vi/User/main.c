/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_systick.h"
#include "bsp_systick.h"
#include "bsp_mpu6000.h"
#include "bsp_usart.h"

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  uint8_t a=11;
  MPU6000_DataTypedef MPU6000_DataStructure;
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  SystickInit(180);
  mpu6000_init();
  USARTInit();
  printf("%d", a);
  a = MPU6000_ReadReg(MPU6000_WHO_AM_I);
  printf("%d", a);
  while (1)
  {
    //			printf("%d",a);
    // MPU6000GetData(&MPU6000_DataStructure);

    // printf("X??:%d\n",MPU6000_DataStructure.ACCEL_XOUT);
    // USARTx_SendByte(UART7, MPU6000_DataStructure.ACCEL_XOUT & 0xFF);
    // USARTx_SendByte(UART7, (MPU6000_DataStructure.ACCEL_XOUT >> 8) & 0xFF);

    // USARTx_SendByte(UART7, MPU6000_DataStructure.ACCEL_YOUT & 0xFF);
    // USARTx_SendByte(UART7, (MPU6000_DataStructure.ACCEL_YOUT >> 8) & 0xFF);

    // USARTx_SendByte(UART7, MPU6000_DataStructure.ACCEL_ZOUT & 0xFF);
    // USARTx_SendByte(UART7, (MPU6000_DataStructure.ACCEL_ZOUT >> 8) & 0xFF);

    // USARTx_SendByte(UART7, MPU6000_DataStructure.GYRO_XOUT & 0xFF);
    // USARTx_SendByte(UART7, (MPU6000_DataStructure.GYRO_XOUT >> 8) & 0xFF);

    // USARTx_SendByte(UART7, MPU6000_DataStructure.GYRO_YOUT & 0xFF);
    // USARTx_SendByte(UART7, (MPU6000_DataStructure.GYRO_YOUT >> 8) & 0xFF);

    // USARTx_SendByte(UART7, MPU6000_DataStructure.GYRO_ZOUT & 0xFF);
    // USARTx_SendByte(UART7, (MPU6000_DataStructure.GYRO_ZOUT >> 8) & 0xFF);
    delay_ms(100);
    }
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */




