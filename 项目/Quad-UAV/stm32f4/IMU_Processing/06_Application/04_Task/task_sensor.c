/**
 ******************************************************************************
 * @file    task_sensor.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/08/29
 * @brief   ����������
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "task_sensor.h"
#include "imu_processing.h"
#include "ringbuff.h"
#include "bsp_systick.h"
#include "com_data.h"
#include "filter.h"
#include "ProConfig.h"
#include "task_datalog.h"
#include <string.h>

LPF2_State_t gyro_filter[3];
LPF2_State_t acc_filter[3];

/**
 * @brief  �����������ʼ��
 * @note
 * @param  ��
 * @retval ��
 */
void Task_Sensor_Init(void)
{
    mpu6000_init();
    IMU_Processing_Init(&imu_raw_data);
    for (int i = 0; i < 3; i++)
    {
        LPF2_Init(&gyro_filter[i], GYRO_FILTER_CUTOFF_FREQUENCY, GYRO_FILTER_SAMPLE_FREQUENCY);
        LPF2_Init(&acc_filter[i], ACC_FILTER_CUTOFF_FREQUENCY, ACC_FILTER_SAMPLE_FREQUENCY);
    }
}


data_buf_t cal_data_buf;
data_buf_t filter_data_buf;
data_buf_i_t raw_data_buf;




static void Sensor_Write_RingBuff(void)
{
    // log_imu_raw_t raw_data;
    // raw_data.acc_x = imu_raw_data.acc.x;
    // raw_data.acc_y = imu_raw_data.acc.y;
    // raw_data.acc_z = imu_raw_data.acc.z;
    // raw_data.gyro_x = imu_raw_data.gyro.x;
    // raw_data.gyro_y = imu_raw_data.gyro.y;
    // raw_data.gyro_z = imu_raw_data.gyro.z;
    
    /* ׼��У׼���� */
    log_imu_cal_t cal_data;
    cal_data.acc_x = imu_cal_data.acc.x;
    cal_data.acc_y = imu_cal_data.acc.y;
    cal_data.acc_z = imu_cal_data.acc.z;
    cal_data.gyro_x = imu_cal_data.gyro.x;
    cal_data.gyro_y = imu_cal_data.gyro.y;
    cal_data.gyro_z = imu_cal_data.gyro.z;

    /* ׼���˲����� */
    log_imu_filter_t filter_data;
    filter_data.acc_x = imu_filter_data.acc.x;
    filter_data.acc_y = imu_filter_data.acc.y;
    filter_data.acc_z = imu_filter_data.acc.z;
    filter_data.gyro_x = imu_filter_data.gyro.x;
    filter_data.gyro_y = imu_filter_data.gyro.y;
    filter_data.gyro_z = imu_filter_data.gyro.z;

    #if 1
    uint32_t tick = GetTick();
    // raw_data_buf.tick = tick;
    // raw_data_buf.acc_x = imu_raw_data.acc.x;
    // raw_data_buf.acc_y = imu_raw_data.acc.y;
    // raw_data_buf.acc_z = imu_raw_data.acc.z;
    // raw_data_buf.gyro_x = imu_raw_data.gyro.x;
    // raw_data_buf.gyro_y = imu_raw_data.gyro.y;
    // raw_data_buf.gyro_z = imu_raw_data.gyro.z;

    cal_data_buf.tick = tick;
    cal_data_buf.acc_x = imu_cal_data.acc.x;
    cal_data_buf.acc_y = imu_cal_data.acc.y;
    cal_data_buf.acc_z = imu_cal_data.acc.z;
    cal_data_buf.gyro_x = imu_cal_data.gyro.x;
    cal_data_buf.gyro_y = imu_cal_data.gyro.y;
    cal_data_buf.gyro_z = imu_cal_data.gyro.z;

    filter_data_buf.tick = tick;
    filter_data_buf.acc_x = imu_filter_data.acc.x;
    filter_data_buf.acc_y = imu_filter_data.acc.y;
    filter_data_buf.acc_z = imu_filter_data.acc.z;
    filter_data_buf.gyro_x = imu_filter_data.gyro.x;
    filter_data_buf.gyro_y = imu_filter_data.gyro.y;
    filter_data_buf.gyro_z = imu_filter_data.gyro.z;
    #endif
    /* ʹ�÷�װ��������ԭʼ������־ */
    // Push_Log_Packet_To_RingBuff(LOG_ID_IMU_RAW, &raw_data, sizeof(raw_data), &SD_W_RingBuffMgr);
    
    /* ʹ�÷�װ��������У׼������־ */
    Push_Log_Packet_To_RingBuff(LOG_ID_IMU_CAL, &cal_data, sizeof(cal_data), &SD_W_RingBuffMgr);

    /* ʹ�÷�װ���������˲�������־ */
    Push_Log_Packet_To_RingBuff(LOG_ID_IMU_FILTER, &filter_data, sizeof(filter_data), &SD_W_RingBuffMgr);
}

// /**
//  * @brief  ������д�뻷�λ��������������ݼ�¼
//  * @note
//  * @param  ��
//  * @retval ��
//  */
// static void Sensor_Write_RingBuff(void)
// {
//     // rb_push_multi((uint8_t *)&imu_raw_data.acc, sizeof(imu_raw_data.acc), &SD_W_RingBuffMgr);
//     // rb_push_multi((uint8_t *)&imu_raw_data.gyro, sizeof(imu_raw_data.gyro), &SD_W_RingBuffMgr);
//     /*��־��*/
//     log_packet_t log_packet;
//     uint8_t checksum = 0;
//     log_imu_cal_t cal_data;
//     log_imu_filter_t filter_data;
//     /*У׼����*/
//     cal_data.acc_x = imu_cal_data.acc.x;
//     cal_data.acc_y = imu_cal_data.acc.y;
//     cal_data.acc_z = imu_cal_data.acc.z;
//     cal_data.gyro_x = imu_cal_data.gyro.x;
//     cal_data.gyro_y = imu_cal_data.gyro.y;
//     cal_data.gyro_z = imu_cal_data.gyro.z;
//     /*�˲�����*/
//     filter_data.acc_x = imu_filter_data.acc.x;
//     filter_data.acc_y = imu_filter_data.acc.y;
//     filter_data.acc_z = imu_filter_data.acc.z;
//     filter_data.gyro_x = imu_filter_data.gyro.x;
//     filter_data.gyro_y = imu_filter_data.gyro.y;
//     filter_data.gyro_z = imu_filter_data.gyro.z;

//     #if 1
//     uint32_t tick = GetTick();
//     cal_data_buf.tick = tick;
//     cal_data_buf.acc_x = imu_cal_data.acc.x;
//     cal_data_buf.acc_y = imu_cal_data.acc.y;
//     cal_data_buf.acc_z = imu_cal_data.acc.z;
//     cal_data_buf.gyro_x = imu_cal_data.gyro.x;
//     cal_data_buf.gyro_y = imu_cal_data.gyro.y;
//     cal_data_buf.gyro_z = imu_cal_data.gyro.z;
//     filter_data_buf.tick = tick;
//     filter_data_buf.acc_x = imu_filter_data.acc.x;
//     filter_data_buf.acc_y = imu_filter_data.acc.y;
//     filter_data_buf.acc_z = imu_filter_data.acc.z;
//     filter_data_buf.gyro_x = imu_filter_data.gyro.x;
//     filter_data_buf.gyro_y = imu_filter_data.gyro.y;
//     filter_data_buf.gyro_z = imu_filter_data.gyro.z;
//     #endif

    
//     /*У׼����*/
//     memset(&log_packet, 0, sizeof(log_packet));
//     log_packet.header1 = LOG_PACKET_HEADER_1;
//     log_packet.header2 = LOG_PACKET_HEADER_2;
//     log_packet.msg_id = LOG_ID_IMU_CAL;
//     log_packet.msg_len = sizeof(cal_data);
//     log_packet.timestamp = GetTick();
//     log_packet.payload.imu_cal = cal_data;
//     // ����У��� (�����У��)
//     uint8_t* p = (uint8_t*)&log_packet;
//     for (int i = 0; i < sizeof(log_packet) - 1; i++) {
//         checksum ^= p[i];
//     }
//     log_packet.checksum = checksum;
//     rb_push_multi((uint8_t *)&log_packet, sizeof(log_packet), &SD_W_RingBuffMgr);

//     /*�˲�����*/
//     memset(&log_packet, 0, sizeof(log_packet));
//     log_packet.header1 = LOG_PACKET_HEADER_1;
//     log_packet.header2 = LOG_PACKET_HEADER_2;
//     log_packet.msg_id = LOG_ID_IMU_FILTER;
//     log_packet.msg_len = sizeof(filter_data);
//     log_packet.timestamp = GetTick();
//     log_packet.payload.imu_filter = filter_data;
//     // ����У��� (�����У��)
//     checksum = 0;
//     p = (uint8_t*)&log_packet;
//     for (int i = 0; i < sizeof(log_packet) - 1; i++) {
//         checksum ^= p[i];
//     }
//     log_packet.checksum = checksum;
//     rb_push_multi((uint8_t *)&log_packet, sizeof(log_packet), &SD_W_RingBuffMgr);
// }

/**
 * @brief  ����������
 * @note
 * @param  ��
 * @retval ��
 */
void Task_Sensor(void)
{
    /*��ȡimuԭʼ����*/
    MPU6000_GetData(&imu_raw_data.acc, &imu_raw_data.gyro);
    /*imu����У׼��ת��*/
    IMU_Processing(&imu_raw_data, &imu_cal_data);
    // printf("%f,", imu_cal_data.acc.x);
    // printf("%f,", imu_cal_data.acc.y);
    // printf("%f,", imu_cal_data.acc.z);
    // printf("%f,", imu_cal_data.gyro.x);
    // printf("%f,", imu_cal_data.gyro.y);
    // printf("%f,", imu_cal_data.gyro.z);
    /*�����˲�*/
    // imu_filter_data.gyro.x = LPF2_Update(&gyro_filter[0], imu_cal_data.gyro.x);
    // imu_filter_data.gyro.y = LPF2_Update(&gyro_filter[1], imu_cal_data.gyro.y);
    // imu_filter_data.gyro.z = LPF2_Update(&gyro_filter[2], imu_cal_data.gyro.z);
    // imu_filter_data.acc.x = LPF2_Update(&acc_filter[0], imu_cal_data.acc.x);
    // imu_filter_data.acc.y = LPF2_Update(&acc_filter[1], imu_cal_data.acc.y);
    // imu_filter_data.acc.z = LPF2_Update(&acc_filter[2], imu_cal_data.acc.z);
    imu_filter_data.gyro.x = LPF2_Update(&gyro_filter[0], imu_cal_data.gyro.x);
    /*��Դ�ɿ�������ת���󣬽����ٶ�y���z������ȡ��������֪��Ϊʲô������������ʱҲȡ��*/
    imu_filter_data.gyro.y = -LPF2_Update(&gyro_filter[1], imu_cal_data.gyro.y);
    imu_filter_data.gyro.z = -LPF2_Update(&gyro_filter[2], imu_cal_data.gyro.z);
    imu_filter_data.acc.x = LPF2_Update(&acc_filter[0], imu_cal_data.acc.x);
    imu_filter_data.acc.y = LPF2_Update(&acc_filter[1], imu_cal_data.acc.y);
    imu_filter_data.acc.z = LPF2_Update(&acc_filter[2], imu_cal_data.acc.z);
    // printf("%f,", imu_cal_data.acc.x);
    // printf("%f,", imu_cal_data.acc.y);
    // printf("%f,", imu_cal_data.acc.z);
    // printf("%f,", imu_cal_data.gyro.x);
    // printf("%f,", imu_cal_data.gyro.y);
    // printf("%f\n", imu_cal_data.gyro.z);
    /*������д�뻷�λ�����*/
    Sensor_Write_RingBuff();
}
