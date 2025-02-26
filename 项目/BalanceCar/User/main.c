/**
 * *****************************************************************************
 * @file        main.c
 * @brief       平衡小车主函数
 * @author
 * @date        2024-11-22
 * @version     0.1
 * @copyright
 * *****************************************************************************
 * @attention
 *
 * 实验平台:
 *
 * *****************************************************************************
 */
#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_tb6612.h"
#include "oled.h"
#include "bsp_encoder.h"
#include "bsp_systick.h"
#include "bsp_mpu6050.h"
#include "kalman_filter.h"
#include <math.h>
#include "car_pid.h"

extern float angle;
#define M_PI 3.14159265358979323846
int32_t motor_speed;
int16_t pwmv;
int main(void)
{
/*********************************USART测试**************************************/

    uint8_t data = 60;
    USARTInit();
    printf("test\n");
    USARTx_SendByte(USART1, data);
	printf("\n");
    while(1)
        ;

/*****************************************************************************/

/*********************************PID测试**************************************/
#if 0
    MPU6050_DataTypedef MPU6050_DataStructure;
    // PID_DataTypeDef OuterPID_DataStructure;
    // PID_DataTypeDef InnerPID_DataStructure;
    CascadePID_DataTypeDef CascadePID_DataStructure;
    float Angle;
    EncoderInit();
    OLED_Init();
    SystickInit(72);
    MPU6050Init();
    TB6612Init();
    PID_Init(&CascadePID_DataStructure.OuterPID_DataStruct, 10, 0, 0);
    PID_Init(&CascadePID_DataStructure.InnerPID_DataStruct, 100, 0, 0);
    // motor_speed = GetSpeed();
    // OLED_ShowString(0, 0, "motor_speed=", OLED_6X8);
    // OLED_ShowSignedNum(72, 0, motor_speed, 4, OLED_6X8);
    // OLED_Update();
    while (1)
    {
        MPU6050GetData(&MPU6050_DataStructure);
        motor_speed = GetSpeed();
        OLED_ShowString(0, 0, "motor_speed=", OLED_6X8);
        OLED_ShowSignedNum(72, 0, motor_speed, 4, OLED_6X8);
        Angle = atan2(MPU6050_DataStructure.ACCEL_XOUT, MPU6050_DataStructure.ACCEL_ZOUT);
        Angle = Angle * 180 / M_PI;
        Com_Filter_Kalman(Angle, (float)MPU6050_DataStructure.GYRO_YOUT * 2000 / 32768);
        OLED_ShowString(0, 8, "Angle=", OLED_6X8);
        OLED_ShowFloatNum(36, 8, Angle, 2, 2, OLED_6X8);
        OLED_ShowString(0, 16, "angle=", OLED_6X8);
        OLED_ShowFloatNum(36, 16, angle, 2, 2, OLED_6X8);
        OLED_Update();
        Cascade_PID(&CascadePID_DataStructure,angle, motor_speed, 0);
        SetPWMADuty(CascadePID_DataStructure.y);
        delay_ms(100);
    }
#endif
/*****************************************************************************/
#if 0
    //    MPU6050Test();
    float Angle = 0;
    float roll;
    MPU6050_DataTypedef MPU6050_DataStructure;
    OLED_Init();
    SystickInit(72);
    MPU6050Init();
    UsartInit();
    // ret = MPU6050_ReadReg(MPU6050_WHO_AM_I);
    // OLED_ShowString(0, 0, "ID:00", OLED_6X8);
    // OLED_ShowHexNum(24, 0, ret, 2, OLED_6X8);
    // OLED_Update();
    while (1)
    {
        MPU6050GetData(&MPU6050_DataStructure);
        Angle = atan2(MPU6050_DataStructure.ACCEL_XOUT, MPU6050_DataStructure.ACCEL_ZOUT);
        Angle = Angle * 180 / M_PI;
        // GitAngle(Angle, (float)MPU6050_DataStructure.GYRO_YOUT * 2000 / 32768);
        Com_Filter_Kalman(Angle, (float)MPU6050_DataStructure.GYRO_YOUT * 2000 / 32768);
        // OLED_ShowString(0, 8, "ACCEL:X 0000", OLED_6X8);
        OLED_ShowFloatNum(0, 0, Angle, 2, 2, OLED_6X8);
        OLED_ShowFloatNum(0, 16, angle, 2, 2, OLED_6X8);
        OLED_Update();
        // printf("Angle=%f\n", Angle);
        // printf("angle=%f\n", angle);
        delay_ms(5);

        // /*注意，MPU6050数据寄存器取出的数据是有符号数 */
        // OLED_ShowString(0, 8, "ACCEL:X 0000", OLED_6X8);
        // OLED_ShowFloatNum(48, 8, (float)MPU6050_DataStructure.ACCEL_XOUT / 32768 * 2, 2, 2, OLED_6X8);

        // OLED_ShowString(0, 16, "ACCEL:Y 0000", OLED_6X8);
        // OLED_ShowFloatNum(48, 16, (float)MPU6050_DataStructure.ACCEL_YOUT / 32768 * 2, 2, 2, OLED_6X8);

        // OLED_ShowString(0, 24, "ACCEL:Z 0000", OLED_6X8);
        // OLED_ShowFloatNum(48, 24, (float)MPU6050_DataStructure.ACCEL_ZOUT / 32768 * 2, 2, 2, OLED_6X8);

        // OLED_ShowString(0, 32, "GYRO:X 0000", OLED_6X8);
        // OLED_ShowFloatNum(42, 32, (float)MPU6050_DataStructure.GYRO_XOUT / 32768 * 2000, 2, 2, OLED_6X8);

        // OLED_ShowString(0, 40, "GYRO:Y 0000", OLED_6X8);
        // OLED_ShowFloatNum(42, 40, (float)MPU6050_DataStructure.GYRO_YOUT / 32768 * 2000, 2, 2, OLED_6X8);

        // OLED_ShowString(0, 48, "GYRO:Z 0000", OLED_6X8);
        // OLED_ShowFloatNum(42, 48, (float)MPU6050_DataStructure.GYRO_ZOUT / 32768 * 2000, 2, 2, OLED_6X8);
        // OLED_Update();
        // delay_ms(100);
    }
#endif
}
