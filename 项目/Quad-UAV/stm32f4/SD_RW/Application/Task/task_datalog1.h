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
#ifndef __TASK_DATALOG1_H
#define __TASK_DATALOG1_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdio.h>
// #include "ff.h"       // FatFsͷ�ļ�
// #include "ringbuff.h" // ��Ļ��λ�����ͷ�ļ�

#define DEBUG_TASK_DATALOG1_ENABLE
#ifdef DEBUG_TASK_DATALOG1_ENABLE
#define TASK_DATALOG1_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define TASK_DATALOG1_DEBUG(format, ...)
#endif

#ifndef NULL
#define NULL 0
#endif

/* 1. ������־���ݰ����� */
typedef enum 
{
    LOG_PACKET_IMU   = 0x01,  // IMU����
    LOG_PACKET_ATT   = 0x02,  // ��̬����
    LOG_PACKET_GPS   = 0x03,  // GPS����
    // ... �ڴ����������������
} LogPacketType_e;

/* 2. ����ͳһ�����ݰ�ͷ�� */
// ʹ��#pragma pack(1)ȷ���ṹ���ǽ��յģ�û�б��������������ֽ�
// ������ļ��洢�Ϳ�ƽ̨����������Ҫ
// #pragma pack(1)
typedef struct
{
    uint16_t sync;       // ͬ��ͷ, e.g., 0xABCD
    uint8_t  type;       // ������
    uint8_t  length;     // ���ݸ��صĳ���
    uint32_t timestamp;  // ʱ��� (ms)
    uint8_t  checksum;   // У���
} LogPacketHeader_t;
// #pragma pack()


/* 3. �����������ݸ��ؽṹ�� */
// #pragma pack(1)
// IMU���ݽṹ��
// typedef struct
// {
//     float acc_x, acc_y, acc_z;
//     float gyro_x, gyro_y, gyro_z;
// } ImuData_t;

// // ��̬���ݽṹ�� (ŷ����)
// typedef struct
// {
//     float roll, pitch, yaw;
// } AttitudeData_t;

// // GPS���ݽṹ�� (�򻯰�)
// typedef struct
// {
//     int32_t latitude;   // γ��, ����1e7
//     int32_t longitude;  // ����, ����1e7
//     uint8_t fix_type;   // ��λ״̬
//     uint8_t satellites; // ��������
// } GpsData_t;
// #pragma pack()

void task_datalog(void);
void task_datalog_init(void);

#endif /* __TASK_DATALOG_H1 */
