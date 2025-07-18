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

/*主循环频率更新标志位*/
extern __IO FlagStatus Main_Loop_Update_Flag;
extern mavlink_message_t MAVLINK_RX_Message;

int main(void)
{
    //    HMC5883L_Data_t data;
    uint32_t Tick = 0;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    systick_init();
    MAVLINK_RB_Init();
    IC_Comm_Init();
    dma_init();
    usart_init();
    spi_init();
    tim_init();
    hmc5883l_init();
    mpu6000_init();
    while (1)
    {
        if (Main_Loop_Update_Flag == SET)
        {
            Main_Loop_Update_Flag = RESET;
            if (LOOP_FREQ_SET(LOOP_500_Hz, Tick))
            {
                MPU6000_GetData(&mpu6000_raw_acc, &mpu6000_raw_gyro);
                MPU6000_ProcessData(&mpu6000_raw_acc, &mpu6000_raw_gyro, &mpu6000_cal_acc, &mpu6000_cal_gyro);
                //                printf("%f\n",mpu6000_cal_acc.x);
                // printf("%f,%f,%f,%f,%f,%f\n",mpu6000_cal_acc.x,mpu6000_cal_acc.y,mpu6000_cal_acc.z,mpu6000_cal_gyro.x,mpu6000_cal_gyro.y,mpu6000_cal_gyro.z);
            }
            if (LOOP_FREQ_SET(LOOP_5_Hz, Tick))
            {
                mavlink_send_heartbeat();
            }
            if (LOOP_FREQ_SET(LOOP_25_Hz, Tick))
            {
                mavlink_send_raw_imu();
            }
            Tick++;
            MAVLINK_Parse();
        }
        ICC_Comm_Test();
    }
}
