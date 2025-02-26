
#ifndef __BSP_TB6612_H
#define __BSP_TB6612_H
/*----------------------------------include-----------------------------------*/
#include "stm32f10x.h"
/*-----------------------------------macro------------------------------------*/
/*B6612AIN、BINGPIO相关宏定义*/
#define                 TB6612IN_GPIO_CLK_FUN           RCC_APB2PeriphClockCmd
#define                 TB6612IN_GPIO_CLK               RCC_APB2Periph_GPIOC
#define                 TB6612AIN1_GPIO_Pin             GPIO_Pin_9
#define                 TB6612AIN1_GPIO_Port            GPIOC
#define                 TB6612AIN2_GPIO_Pin             GPIO_Pin_10
#define                 TB6612AIN2_GPIO_Port            GPIOC
#define                 TB6612BIN1_GPIO_Pin             GPIO_Pin_11
#define                 TB6612BIN1_GPIO_Port            GPIOC
#define                 TB6612BIN2_GPIO_Pin             GPIO_Pin_12
#define                 TB6612BIN2_GPIO_Port            GPIOC

/*TB6612输入口高低电平设置*/
#define    TB6612AIN1_H    GPIO_SetBits(TB6612AIN1_GPIO_Port, TB6612AIN1_GPIO_Pin);
#define    TB6612AIN1_L    GPIO_ResetBits(TB6612AIN1_GPIO_Port, TB6612AIN1_GPIO_Pin);
#define    TB6612AIN2_H    GPIO_SetBits(TB6612AIN2_GPIO_Port, TB6612AIN2_GPIO_Pin);
#define    TB6612AIN2_L    GPIO_ResetBits(TB6612AIN2_GPIO_Port, TB6612AIN2_GPIO_Pin);
#define    TB6612BIN1_H    GPIO_SetBits(TB6612BIN1_GPIO_Port, TB6612BIN1_GPIO_Pin);
#define    TB6612BIN1_L    GPIO_ResetBits(TB6612BIN1_GPIO_Port, TB6612BIN1_GPIO_Pin);
#define    TB6612BIN2_H    GPIO_SetBits(TB6612BIN2_GPIO_Port, TB6612BIN2_GPIO_Pin);
#define    TB6612BIN2_L    GPIO_ResetBits(TB6612BIN2_GPIO_Port, TB6612BIN2_GPIO_Pin);

/*停止*/
#define                 Stop                  0
/*正转*/
#define                 Corotation            1
/*反转*/
#define                 Inversion             2
/*刹车*/
#define                 Brake                 3
/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void TB6612Init(void);
void SetMotorAStutus(uint8_t status);
void SetMotorBStutus(uint8_t status);
void SetPWMADuty(int16_t duty);
void SetPWMBDuty(int16_t duty);
/*------------------------------------test------------------------------------*/

#endif /* __BSP_TB6612_H */
