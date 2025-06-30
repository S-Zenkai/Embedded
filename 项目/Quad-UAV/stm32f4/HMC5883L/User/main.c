/**
 ******************************************************************************
 * @file    main.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/02/28
 * @brief   主函数
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
#include "qmc5883.h"
#include "i2c_sw.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void)
{
    HMC5883L_Data_t data;
    uint8_t count = 0;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    systick_init();
    usart_init();
    tim_init();
    hmc5883l_init();
    while (1)
    {
        if (TIM6_Update_500Hz_Flag == SET)
        {
            TIM6_Update_500Hz_Flag = RESET;
            if (count >= 9)
            {
								/*50Hz读一次数据*/
                hmc5883l_read_data(&data);
								printf("%f,%f,%f\n", data.x, data.y, data.z);
								hmc5883l_single_measurement();
                count = 0;
            }
            count++;
        }
        // hmc5883l_read_data(&data);
        // printf("x: %f, y: %f, z: %f\r\n", data.x, data.y, data.z);
        // hmc5883l_single_measurement();
        // delay_ms(10);
    }
}
