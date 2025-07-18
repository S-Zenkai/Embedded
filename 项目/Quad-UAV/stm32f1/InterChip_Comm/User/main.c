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
#include "pro_common.h"
#include "bsp_sys.h"
#include "bsp_systick.h"
#include "bsp_usart.h"
#include "bsp_dma.h"
#include "bsp_tim.h"
#include "sbus.h"
#include <stdbool.h>
#include "sbus.h"
#include "ic_comm.h"
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
    systick_init();
    IC_Comm_Init();
    dma_init();
    usart_init();
    sbus_init();
    timer_init();

    while (1)
    {
        /*获取rc数据*/
        get_rc_data(rc_data, RC_PWM_CHANNELS);
        IC_Comm_Test();
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
