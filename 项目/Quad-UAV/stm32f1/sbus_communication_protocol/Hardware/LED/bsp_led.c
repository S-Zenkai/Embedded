/**
 * *****************************************************************************
 * @file        bsp_led.c
 * @brief       
 * @author      S-Zenkai (1747098083@qq.com)
 * @date        2025-01-23
 * @version     
 * *****************************************************************************
 * @attention  
 * 
 * 实验平台:
 * 
 * *****************************************************************************
 */

/*----------------------------------include-----------------------------------*/
#include "bsp_led.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
/**
 * @brief       
 * 
 */
static void GPIO_Configure(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);/*开时钟*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
 * @brief       琥珀色led灯初始化
 * 
 */
void amber_init(void)
{
    GPIO_Configure();
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
}
/*------------------------------------test------------------------------------*/



