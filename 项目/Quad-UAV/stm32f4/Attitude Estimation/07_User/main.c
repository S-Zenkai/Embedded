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
            /*组合导航任务，500hz*/
            Task_IntegratedNavigation();
        }
        Tick++;
    }
    // ICC_Comm_Test();
}


/*角速度积分解算姿态*/
extern Euler_Test_t gyro_euler;
/*加速度、磁力计解算姿态*/
extern Euler_Test_t acc_mag_euler;

int32_t el_roll=0;
int32_t el_pitch=0;
int32_t el_yaw=0;

int32_t num_test=0;

int main(void)
{
    //    HMC5883L_Data_t data;
    uint32_t Main_Tick = 0;
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
    Euler_Test_t acc_mag_euler_buf;/*加速度、磁力计解算姿态*/
    Euler_Test_t gyro_euler_buf;/*角速度积分解算姿态*/
    Euler_u euler_buf;/*mahony解算姿态*/
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

            if (LOOP_FREQ_SET(LOOP_25_Hz, Main_Tick, LOOP_MAIN_RATE))
            {
                #if LOG_RECORD_ENABLE
                task_datalog();
                #endif
                // 进入临界区，防止在复制数据时被中断打断
                __disable_irq();
                acc_mag_euler_buf = acc_mag_euler;
                gyro_euler_buf = gyro_euler;
                euler_buf = flgt_ctl.euler;
                __enable_irq();
                // 退出临界区
                printf("%d,", acc_mag_euler_buf.tick);
                printf("%f,", acc_mag_euler_buf.roll);
                printf("%f,", acc_mag_euler_buf.pitch);
                printf("%f,", acc_mag_euler_buf.yaw);
                printf("%d,", gyro_euler_buf.tick);
                printf("%f,", gyro_euler_buf.roll);
                printf("%f,", gyro_euler_buf.pitch);
                printf("%f,", gyro_euler_buf.yaw);
                printf("%f,", euler_buf.roll);
                printf("%f,", euler_buf.pitch);
                printf("%f", euler_buf.yaw);	
                printf("\n");
                num_test++;
            }
            Main_Tick++; /*这里是局部变量*/
        }
        // MAVLINK_Parse();
        // IC_Comm_Task();
    }
}
