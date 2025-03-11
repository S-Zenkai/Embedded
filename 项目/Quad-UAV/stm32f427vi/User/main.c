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
#include "bsp_mpu6000.h"
#include "bsp_usart.h"
#include "bsp_tim.h"
#include "bsp_exti.h"
#include "filter.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*MPU6000数据准备完成标志*/
uint8_t mpu6000ready_flag;
/*mpu6000存储数据变量*/
MPU6000Data_t MPU6000Data;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void)
{
    SystickInit(180);
    USARTInit();
    mpu6000_init();
    timer_tbinit();
    exti_init();
    while (1)
    {
        if(mpu6000ready_flag==1)
        {
            MPU6000_GetData(&MPU6000Data);
            MPU6000_ProcessingData(&MPU6000Data);
        }
    }
/* MPU6000测试*****************************************************************/
#if MPU6000_Test
    uint8_t a;
    SystickInit(180);
    USARTInit();
    mpu6000_init();
    // timer_t?binit();
    // timer_ti_init();
    // KEY_EXTI_Config();
    a = MPU6000_ReadReg(MPU6000_WHO_AM_I);
    printf("%d\n", a);
    while (1)
    {
        // if (MPU6000_ReadReg(MPU6000_INT_STATUS) == 1)
        // {
        //     current_value = TIM_GetCounter(TIM6);
        //     dt = (current_value) / 1000000;
        //     TIM_SetCounter(TIM6, 0);
        //     // last_value = 0;
        //     printf("dt=%f\n", dt);
        // }
        MPU6000_GetData(&MPU6000_DataStructure);
        printf("**************************************************************\n");
        /*加速度数据*/
        printf("ACCEL_XOUT=%d\n", MPU6000_DataStructure.ACCEL_XOUT);
        printf("ACCEL_YOUT=%d\n", MPU6000_DataStructure.ACCEL_YOUT);
        printf("ACCEL_ZOUT=%d\n", MPU6000_DataStructure.ACCEL_ZOUT);
        /*角速度数据*/
        printf("GYRO_XOUT=%d\n", MPU6000_DataStructure.GYRO_XOUT);
        printf("GYRO_YOUT=%d\n", MPU6000_DataStructure.GYRO_YOUT);
        printf("GYRO_ZOUT=%d\n", MPU6000_DataStructure.GYRO_ZOUT);
        printf("**************************************************************\n");
        delay_ms(5);
    }
#endif
/* MPU6000测试*****************************************************************/
}
