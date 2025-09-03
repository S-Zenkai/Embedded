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
#include "task_datalog.h"
#include "task_sensor.h"

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
    /*�Ի��λ������ĳ�ʼ��Ҫ���ڳ���ͷ���Է�ֹ�����жϺ����δ��ʼ���Ļ�����*/
    MAVLINK_RB_Init();
    task_datalog_init();
    IC_Comm_Init();
    spi_init();
    hmc5883l_init();
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
            /*���ݼ�¼����*/

            if (LOOP_FREQ_SET(LOOP_25_Hz, Tick))
            {
                task_datalog();
                data_buf_t local_copy;
                // �����ٽ�������ֹ�ڸ�������ʱ���жϴ��
                __disable_irq();
                local_copy = filter_data_buf; // ���ٽ��������ݸ��Ƶ��ֲ�����
                __enable_irq();
                // �˳��ٽ���
                printf("%d,", local_copy.tick);
                printf("%f,", local_copy.acc_x);
                printf("%f,", local_copy.acc_y);
                printf("%f,", local_copy.acc_z);
                printf("%f,", local_copy.gyro_x);
                printf("%f,", local_copy.gyro_y);
                printf("%f", local_copy.gyro_z);
                printf("\n");
            }
            Tick++; /*�����Ǿֲ�����*/
        }
        MAVLINK_Parse();
        IC_Comm_Task();
    }
}
