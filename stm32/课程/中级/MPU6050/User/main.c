/**
 * *****************************************************************************
 * @file        main.c
 * @brief       独立模式多通道ADC实验，使用DMA转移规则组数据寄存器数据
 * @author
 * @date        2024-12-05
 * @version
 * @copyright
 * *****************************************************************************
 * @attention
 *
 * 实验平台:
 *
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
#include "OLED.h"
#include "bsp_usart.h"
#include "bsp_systick.h"
#include "bsp_mpu6050.h"
/*----------------------------------------------------------------------------*/
int main(void)
{
    MPU6050_DataTypedef MPU6050_DataStructure;
    OLED_Init();
    uint8_t ret=0;
    SysTick_Init(72);
    MPU6050Init();
    ret = MPU6050_ReadReg(MPU6050_WHO_AM_I);
    OLED_ShowString(0, 0, "ID:00", OLED_6X8);
    OLED_ShowHexNum(24, 0, ret, 2, OLED_6X8);
    OLED_Update();
    while (1)
    {
        MPU6050GetData(&MPU6050_DataStructure);
        /*注意，MPU6050数据寄存器取出的数据是有符号数 */
        OLED_ShowString(0, 8, "ACCEL:X 0000", OLED_6X8);
        OLED_ShowFloatNum(48, 8, (float)MPU6050_DataStructure.ACCEL_XOUT / 32768 * 16, 2, 2, OLED_6X8);

        OLED_ShowString(0, 16, "ACCEL:Y 0000", OLED_6X8);
        OLED_ShowFloatNum(48, 16, (float)MPU6050_DataStructure.ACCEL_YOUT / 32768 * 16, 2, 2, OLED_6X8);

        OLED_ShowString(0, 24, "ACCEL:Z 0000", OLED_6X8);
        OLED_ShowFloatNum(48, 24, (float)MPU6050_DataStructure.ACCEL_ZOUT / 32768 * 16, 2, 2, OLED_6X8);

        OLED_ShowString(0, 32, "GYRO:X 0000", OLED_6X8);
        OLED_ShowFloatNum(42, 32, (float)MPU6050_DataStructure.GYRO_XOUT / 32768 * 2000, 2, 2, OLED_6X8);

        OLED_ShowString(0, 40, "GYRO:Y 0000", OLED_6X8);
        OLED_ShowFloatNum(42, 40, (float)MPU6050_DataStructure.GYRO_YOUT / 32768 * 2000, 2, 2, OLED_6X8);

        OLED_ShowString(0, 48, "GYRO:Z 0000", OLED_6X8);
        OLED_ShowFloatNum(42, 48, (float)MPU6050_DataStructure.GYRO_ZOUT / 32768 * 2000, 2, 2, OLED_6X8);
        OLED_Update();
        delay_ms(100);
    }
}
