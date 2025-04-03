/**
 * *****************************************************************************
 * @file        bsp_tb6612.c
 * @brief       tim3
 * @author      TB6612驱动程序
 *              电路连接：
 *              输出PWM波：TIM3_CH1(PA6)-PWMA、TIM3_CH2(PA7)-PWMB
 *              AIN1-PC9、AIN2-PC10
 *              BIN1-PC11、AIN2-PC12
 * @date        2024-11-22
 * @version     0.1
 * @copyright
 * *****************************************************************************
 * @attention
 *
 * 实验平台:    STM32F103VET6、TB6612FNG
 *
 * *****************************************************************************
 */

#include "bsp_tb6612.h"
#include "bsp_tim.h"

/************************配置TB6612AIN、BIN相关GPIO**************************/

/**
 * @brief       TB6612AIN、BIN相关GPIO配置
 *
 */
static void TB6612IN_GPIOConfigure(void)
{
    GPIO_InitTypeDef GPIOInitStructure;
    TB6612IN_GPIO_CLK_FUN(TB6612IN_GPIO_CLK, ENABLE);
    /*AIN1-PC9*/
    GPIOInitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIOInitStructure.GPIO_Pin = TB6612AIN1_GPIO_Pin;
    GPIOInitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TB6612AIN1_GPIO_Port, &GPIOInitStructure);
    /*AIN2-PC10*/
    GPIOInitStructure.GPIO_Pin = TB6612AIN2_GPIO_Pin;
    GPIO_Init(TB6612AIN2_GPIO_Port, &GPIOInitStructure);
    /*BIN1-PC11*/
    GPIOInitStructure.GPIO_Pin = TB6612BIN1_GPIO_Pin;
    GPIO_Init(TB6612BIN1_GPIO_Port, &GPIOInitStructure);
    /*BIN2-PC11*/
    GPIOInitStructure.GPIO_Pin = TB6612BIN2_GPIO_Pin;
    GPIO_Init(TB6612BIN2_GPIO_Port, &GPIOInitStructure);
}

/**
 * @brief       TB6612初始化，在该函数中进行输出比较初始化，不必在主函数中调用Tim_OCInit
 *
 */
void TB6612Init(void)
{
    Tim_OCInit();
    TB6612IN_GPIOConfigure();
}

/**
 * @brief       设置MotorA状态
 *
 * @param       status：可以是下面参数：
 *                  Corotation：正转
 *                  Inversion： 反转
 *                  Brake：     刹车
 */
void SetMotorAStutus(uint8_t status)
{
    switch (status)
    {
    case Corotation:
        TB6612AIN1_L;
        TB6612AIN2_H;
        break;
    case Inversion:
        TB6612AIN1_H;
        TB6612AIN2_L;
        break;
    case Brake:
        TB6612AIN1_H;
        TB6612AIN2_H;
        break;
    default:
        break;
    }
}

/**
 * @brief       设置MotorB状态
 *
 * @param       status：可以是下面参数：
 *                  Corotation:正转
 *                  Inversion: 反转
 *                  Brake:     刹车
 */
void SetMotorBStutus(uint8_t status)
{
    switch (status)
    {
    case Corotation:
        TB6612AIN1_L;
        TB6612AIN2_H;
        break;
    case Inversion:
        TB6612AIN1_H;
        TB6612AIN2_L;
        break;
    case Brake:
        TB6612AIN1_H;
        TB6612AIN2_H;
        break;
    default:
        break;
    }
}

/**
 * @brief       
 * 
 * @param       duty 
 */
void SetPWMADuty(int16_t duty)
{
    if(duty>0)
    {
        SetMotorAStutus(Corotation);
    }
    else if(duty<0)
    {
        SetMotorAStutus(Inversion);
        duty = -duty;
    }
    TIM_SetCompare1(Timx, duty);
}

/**
 * @brief
 *
 * @param       duty
 */
void SetPWMBDuty(int16_t duty)
{
    if (duty > 0)
    {
        SetMotorBStutus(Corotation);
    }
    else if (duty < 0)
    {
        SetMotorBStutus(Inversion);
        duty = -duty;
    }
    TIM_SetCompare2(Timx, duty);
}
