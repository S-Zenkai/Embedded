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

/*GNSS接收数据缓冲区大小*/
#define GNSS_RX_BUFF_SIZE 128

/* Exported types ------------------------------------------------------------*/

/*存储GSP模块解析后的定位、速度、时间和状态信息结构体*/
struct vehicle_gps_info_s
{
     // required for logger
    uint64_t timestamp;/*本地系统时间戳，表示数据生成的时间*/
    uint64_t time_utc_usec;/*UTC 时间，基于 GPS 接收的时间*/
    int32_t lat;/*纬度，单位为度（°）*/
    int32_t lon;/*经度，单位为度（°）*/
    int32_t alt;/*海拔高度，单位为米*/
    int32_t alt_ellipsoid;/*椭球高度，单位为米*/
    float s_variance_m_s;/*速度估计的方差（不确定性）*/
    float c_variance_rad;/*航向估计的方差*/
    float eph;/*水平位置精度*/
    float epv;/*垂直位置精度*/
    float hdop;/*水平几何精度因子*/
    float vdop;/*垂直几何精度因子*/
    int32_t noise_per_ms;/*接收机噪声水平*/
    int32_t jamming_indicator;/*干扰指示*/
    float vel_m_s;/*地面速度（水平速度）*/
    float vel_n_m_s;/*北向速度（NED 坐标系）*/
    float vel_e_m_s;/*东向速度（NED 坐标系）*/
    float vel_d_m_s;/*向下速度（NED 坐标系）*/
    float cog_rad;/*运动航向（Course Over Ground）*/
    int32_t timestamp_time_relative;/*相对时间戳差值*/
    uint8_t fix_type;/*定位类型*/
    uint8_t vel_ned_valid;/*NED 速度数据是否有效*/
    uint8_t satellites_used;/*用于定位的卫星数量*/
    uint8_t _padding0[5];/*填充字节，确保结构体对齐或固定大小*/
     // required for logger
};

/* 时间结构体 */
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

/*重新定GPS_DEBUG函数*/
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
