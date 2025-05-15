/**
 ******************************************************************************
 * @file    main.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/02/28
 * @brief   Ö÷º¯Êý
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
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
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void)
{
    systick_init();
    usart_init();
    dma_init();
    gps_init();
    while (1)
    {
        read_rb_process();
    }
}
