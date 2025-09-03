/**
  ******************************************************************************
  * @file    task_datalog.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/08/05
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_DATALOG_H
#define __TASK_DATALOG_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdio.h>
#include "com_type.h"
#include "ringbuff.h"

#define DEBUG_TASK_DATALOG1_ENABLE
#ifdef DEBUG_TASK_DATALOG1_ENABLE
#define TASK_DATALOG1_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define TASK_DATALOG1_DEBUG(format, ...)
#endif

#ifndef NULL
#define NULL 0
#endif



// 1. ����ÿ�����ݰ���ΨһID
#define LOG_ID_IMU_RAW      0x00
#define LOG_ID_IMU_CAL      0x01
#define LOG_ID_IMU_FILTER   0x02
#define LOG_ID_ATTITUDE     0x03
#define LOG_ID_GPS          0x04
#define LOG_ID_STATUS       0x05 // ϵͳ״̬��Ϣ


// 2. Ϊÿ���������Ͷ�������Ľṹ��
// ʹ��__attribute__((packed))ȷ��û���ڴ���������ֽڣ�����ڿ�ƽ̨��������Ҫ

/*ԭʼimu����*/
typedef struct __attribute__((packed)) {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
} log_imu_raw_t; // 12 bytes

/*У׼��ת�����imu����*/
typedef struct __attribute__((packed)) {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} log_imu_cal_t; // 24 bytes

/*�˲����imu����*/
typedef struct __attribute__((packed)) {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} log_imu_filter_t; // 24 bytes

typedef struct __attribute__((packed)) {
    float roll, pitch, yaw;
} log_attitude_t; // 12 bytes

typedef struct __attribute__((packed)) {
    double latitude;
    double longitude;
    float altitude;
    uint8_t sats_in_view;
} log_gps_t; // 21 bytes

typedef struct __attribute__((packed)) {
    uint8_t flight_mode;
    uint8_t is_armed;
} log_status_t; // 2 bytes


// 3. ����ͳһ����־���ݰ��ṹ (Log Packet)
#define LOG_PACKET_HEADER_1 0xA5 // ��ͷͬ���ֽ�1
#define LOG_PACKET_HEADER_2 0x5A // ��ͷͬ���ֽ�2

typedef struct __attribute__((packed)) {
    uint8_t header1;      // �̶�Ϊ LOG_PACKET_HEADER_1
    uint8_t header2;      // �̶�Ϊ LOG_PACKET_HEADER_2
    uint8_t msg_id;       // ��ϢID, e.g., LOG_ID_IMU
    uint8_t msg_len;      // payload�ĳ���
    uint32_t timestamp;   // ʱ��� (ms or us)
    
    // ʹ��union���洢��ͬ���͵����ݣ���ʡ�ڴ�
    union {
        log_imu_raw_t imu_raw;
        log_imu_cal_t imu_cal;
        log_imu_filter_t imu_filter;
        log_attitude_t attitude;
        log_gps_t gps;
        log_status_t status;
        // uint8_t raw_data[128]; // ����һ���㹻���ԭʼ��������ȷ��union��С
    } payload;

    uint8_t checksum;     // У���
} log_packet_t;

void task_datalog(void);
void task_datalog_init(void);
void Push_Log_Packet_To_RingBuff(uint8_t msg_id, const void* payload_data, uint8_t payload_len, ringbuff_t* rb_mgr);
#endif /* __TASK_DATALOG_H */
