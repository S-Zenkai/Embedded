/**
 ******************************************************************************
 * @file    main.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/02/28
 * @brief   ������
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "task_datalog1.h"

/*��ѭ��Ƶ�ʸ��±�־λ*/
extern __IO FlagStatus Main_Loop_Update_Flag;
extern mavlink_message_t MAVLINK_RX_Message;

uint32_t Tick = 0;

/**
 * @brief  TIM6��ʱ�ж�
 * @note
 * @param  ��
 * @retval ��
 */
extern ringbuff_t SD_W_RingBuffMgr;
void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        Main_Loop_Update_Flag = SET;
        if (LOOP_FREQ_SET(LOOP_500_Hz, Tick))
        {
            MPU6000_GetData(&mpu6000_raw_acc, &mpu6000_raw_gyro);
            MPU6000_ProcessData(&mpu6000_raw_acc, &mpu6000_raw_gyro, &mpu6000_cal_acc, &mpu6000_cal_gyro);
            uint32_t tick = GetTick();
            rb_push_multi((uint8_t *)&tick, sizeof(tick), &SD_W_RingBuffMgr);
            rb_push_multi((uint8_t *)&mpu6000_raw_acc.x, sizeof(mpu6000_raw_acc.x), &SD_W_RingBuffMgr);
            rb_push_multi((uint8_t *)&mpu6000_raw_acc.y, sizeof(mpu6000_raw_acc.y), &SD_W_RingBuffMgr);
            rb_push_multi((uint8_t *)&mpu6000_raw_acc.z, sizeof(mpu6000_raw_acc.z), &SD_W_RingBuffMgr);
            rb_push_multi((uint8_t *)&mpu6000_raw_gyro.x, sizeof(mpu6000_raw_gyro.x), &SD_W_RingBuffMgr);
            rb_push_multi((uint8_t *)&mpu6000_raw_gyro.y, sizeof(mpu6000_raw_gyro.y), &SD_W_RingBuffMgr);
            rb_push_multi((uint8_t *)&mpu6000_raw_gyro.z, sizeof(mpu6000_raw_gyro.z), &SD_W_RingBuffMgr);
        }
        Tick++;
    }
    // ICC_Comm_Test();
}

int main(void)
{
    //    HMC5883L_Data_t data;
    uint32_t Tick = 0;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    systick_init();
    usart_init();
    dma_init();
    /*�Ի��λ������ĳ�ʼ��Ҫ���ڳ���ͷ���Է�ֹ�����жϺ����δ��ʼ���Ļ�����*/
    MAVLINK_RB_Init();
    task_datalog_init();
    IC_Comm_Init();
    spi_init();
    hmc5883l_init();
    mpu6000_init();
    tim_init();

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
            /*���ݼ�¼����*/

            if (LOOP_FREQ_SET(LOOP_25_Hz, Tick))
            {
                task_datalog();
            }
            Tick++; /*�����Ǿֲ�����*/
        }
        MAVLINK_Parse();
        IC_Comm_Task();
    }
}
