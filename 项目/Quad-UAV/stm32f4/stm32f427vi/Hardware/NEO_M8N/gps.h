/**
  ******************************************************************************
  * @file    gps.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/04/09
  * @brief   
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPS_H
#define __GPS_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stdbool.h"
#include "bsp_usart.h"
#include "ringbuff.h"
#include "globle.h"


/* Exported define ------------------------------------------------------------*/

/*GNSS�������ݻ�������С*/
#define GNSS_RX_BUFF_SIZE 128

/* Exported types ------------------------------------------------------------*/

/*�洢GSPģ�������Ķ�λ���ٶȡ�ʱ���״̬��Ϣ�ṹ��*/
struct vehicle_gps_info_s
{
     // required for logger
    uint64_t timestamp;/*����ϵͳʱ�������ʾ�������ɵ�ʱ��*/
    uint64_t time_utc_usec;/*UTC ʱ�䣬���� GPS ���յ�ʱ��*/
    int32_t lat;/*γ�ȣ���λΪ�ȣ��㣩*/
    int32_t lon;/*���ȣ���λΪ�ȣ��㣩*/
    int32_t alt;/*���θ߶ȣ���λΪ��*/
    int32_t alt_ellipsoid;/*����߶ȣ���λΪ��*/
    float s_variance_m_s;/*�ٶȹ��Ƶķ����ȷ���ԣ�*/
    float c_variance_rad;/*������Ƶķ���*/
    float eph;/*ˮƽλ�þ���*/
    float epv;/*��ֱλ�þ���*/
    float hdop;/*ˮƽ���ξ�������*/
    float vdop;/*��ֱ���ξ�������*/
    int32_t noise_per_ms;/*���ջ�����ˮƽ*/
    int32_t jamming_indicator;/*����ָʾ*/
    float vel_m_s;/*�����ٶȣ�ˮƽ�ٶȣ�*/
    float vel_n_m_s;/*�����ٶȣ�NED ����ϵ��*/
    float vel_e_m_s;/*�����ٶȣ�NED ����ϵ��*/
    float vel_d_m_s;/*�����ٶȣ�NED ����ϵ��*/
    float cog_rad;/*�˶�����Course Over Ground��*/
    int32_t timestamp_time_relative;/*���ʱ�����ֵ*/
    uint8_t fix_type;/*��λ����*/
    uint8_t vel_ned_valid;/*NED �ٶ������Ƿ���Ч*/
    uint8_t satellites_used;/*���ڶ�λ����������*/
    uint8_t _padding0[5];/*����ֽڣ�ȷ���ṹ������̶���С*/
     // required for logger
};

/* ʱ��ṹ�� */
struct tm
{
    int tm_sec;                                                     /* second (0-61, allows for leap seconds) */
    int tm_min;                                                     /* minute (0-59) */
    int tm_hour;                                                    /* hour (0-23) */
    int tm_mday;                                                    /* day of the month (1-31) */
    int tm_mon;                                                     /* month (0-11) */
    int tm_year;                                                    /* years since 1900 */
    int tm_wday; /* day of the week (0-6) */                        /*not supported by NuttX*/
    int tm_yday; /* day of the year (0-365) */                      /*not supported by NuttX*/
    int tm_isdst; /* non-0 if daylight savings time is in effect */ /*not supported by NuttX*/
};

/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

#define DEBUG_GPS_ENABLE

/*���¶�GPS_DEBUG����*/
#ifdef DEBUG_GPS_ENABLE
#define GPS_DEBUG(format, ...) printf(format, ##__VA_ARGS__)
#else
#define GPS_DEBUG(format, ...)
#endif

extern ringbuff_t gnss_rb_mng;

/* Exported functions ------------------------------------------------------- */
void GNSS_ResetBaudRate(uint32_t baudrate);
void GNSS_RB_Clear(void);
uint8_t GNSS_RB_CheckOverflow(void);
void GNSS_RB_PushData(uint8_t data);
uint8_t GNSS_RB_PopData(void);
uint32_t GNSS_RB_GetDataCounter(void);
bool GNSS_RB_IsEmpty(void);
void gps_init(void);

#endif /*__GPS_H*/
