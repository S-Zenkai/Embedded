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
#include "task_datalog.h"
#include "task_sensor.h"


extern mavlink_message_t MAVLINK_RX_Message;



/**
 * @brief  TIM6定时中断
 * @note
 * @param  无
 * @retval 无
 */
extern ringbuff_t SD_W_RingBuffMgr;
void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        Main_Loop_Update_Flag = SET;
        if (LOOP_FREQ_SET(LOOP_500_Hz, Tick, LOOP_MAIN_RATE))
        {
            /*500hz读取IMU数据*/
            /*50hz读取磁力计数据*/
            Task_Sensor();
        }
        Tick++;
    }
    // ICC_Comm_Test();
}


extern data_buf_t cal_data_buf;
extern data_buf_t filter_data_buf;
extern data_buf_i_t raw_data_buf;

int main(void)
{
    //    HMC5883L_Data_t data;
    uint32_t Tick = 0;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    systick_init();
    usart_init();
    dma_init();
    /*对环形缓冲区的初始化要放在程序开头，以防止开启中断后访问未初始化的缓冲区*/
    MAVLINK_RB_Init();
    task_datalog_init();
    IC_Comm_Init();
    spi_init();
    Task_Sensor_Init();
    tim_init();
//		printf("test");
    while (1)
    {
        if (Main_Loop_Update_Flag == SET)
        {
            Main_Loop_Update_Flag = RESET;
            // if (LOOP_FREQ_SET(LOOP_5_Hz, Tick))
            // {
            //     mavlink_send_heartbeat();
            // }
            // if (LOOP_FREQ_SET(LOOP_25_Hz, Tick))
            // {
            //     mavlink_send_raw_imu();
            // }
            /*数据记录任务*/

            if (LOOP_FREQ_SET(LOOP_25_Hz, Tick, LOOP_MAIN_RATE))
            {
                task_datalog();
                data_buf_t local_copy;
                data_buf_i_t local_copy_2;
                // 进入临界区，防止在复制数据时被中断打断
                __disable_irq();
                local_copy = cal_data_buf; // 快速将共享数据复制到局部变量
                local_copy_2 = raw_data_buf; // 快速将共享数据复制到局部变量
                __enable_irq();
                // 退出临界区
                printf("%d,", local_copy.tick);
                printf("%f,", local_copy.mag_x);
                printf("%f,", local_copy.mag_y);
                printf("%f,", local_copy.mag_z);
                printf("%d,", local_copy_2.tick);
                printf("%d,", local_copy_2.mag_x);
                printf("%d,", local_copy_2.mag_y);
                printf("%d,", local_copy_2.mag_z);
                
                printf("\n");
            }
            Tick++; /*这里是局部变量*/
        }
        MAVLINK_Parse();
        IC_Comm_Task();
    }
}
