/**
 ******************************************************************************
 * @file    main.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/03/27
 * @brief
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "globle.h"
#include "bsp_sys.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "bsp_dma.h"
#include "bsp_tim.h"
#include "sbus.h"
#include <stdbool.h>
/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

// uint16_t pwm_buff[RC_PWM_CHANNELS];      /*9通道*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{
    bool decode_status;

    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); /*在整个项目使用一次即可*/
    SystickInit(72);
    sbus_init();
    timer_init(); 
    while (1)
    {
        if (sbus_DF_TC == SET)
        {
            sbus_DF_TC = RESET;
            decode_status = sbus_decode(sbus_buff[active_buff ^ 1], pwm_buff, RC_PWM_CHANNELS);
            //            set_pwm(pwm_buff);
        }
        if(decode_status==true)
        {
            set_pwm(pwm_buff);
        } 
    }
}

/**
 * @}
 */

/* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/

/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @}
 */
