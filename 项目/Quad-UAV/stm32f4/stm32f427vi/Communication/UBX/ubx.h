/**
 ******************************************************************************
 * @file    sbus.h
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
#ifndef __UBX_H
#define __UBX_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "globle.h"
#include "bsp_systick.h"
#include "gps.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <string.h>
/* Exported define ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/*UBXЭ�����״̬*/
typedef enum
{
    UBX_DECODE_SYNC1 = 0,   /**<ͬ����1*/
    UBX_DECODE_SYNC2 = 1,   /**<ͬ����2*/
    UBX_DECODE_CLASS = 2,   /**<��*/
    UBX_DECODE_ID = 3,      /**<ID*/
    UBX_DECODE_LENGTH1 = 4, /**<����1*/
    UBX_DECODE_LENGTH2 = 5, /**<����2*/
    UBX_DECODE_PAYLOAD = 6, /**<����*/
    UBX_DECODE_CKA = 7,     /**<У���A*/
    UBX_DECODE_CKB = 8,     /**<У���B*/

    RTCM_DECODE_SYNC1 = 8, /**<RTCMͬ����1*/
} ubx_decode_state_t;

/*ubx������Ϣ״̬*/
typedef enum
{
    UBX_RX_MSG_IGNORE = 0,       /**<������Ϣ�������ܵ�����Ϣ����δ����ʱ��������Ϣ*/
    UBX_RX_MSG_HANDLE = 1,       /**<������Ϣ�������ܵ�����Ϣ���ͺͳ��Ⱦ��Ϸ���������Ϣ*/
    UBX_RX_MSG_DISABLE = 2,      /**<������Ϣ������⵽�Ǳ�Ҫ��ϢƵ�����ͣ�������Ϣ���Ż�����*/
    UBX_RX_MSG_ERROR_LENGTH = 3, /**<��Ϣ���ȴ���*/
} ubx_rx_msg_state_t;

/*ubxȷ��״̬*/
typedef enum
{
    UBX_ACK_IDLE = 0,
    UBX_ACK_WAITING,
    UBX_ACK_GOT_ACK,
    UBX_ACK_GOT_NAK
} ubx_ack_state_t;

/*GNSS�������������ģʽ*/
typedef enum
{
    OUTPUT_GPS = 0, /**<��־���ģʽ����UBX���*/
    OUTPUT_RTCM,    /**<UBX+RTCM���*/
} output_mode_t;

/*GNSS��������������ģʽ*/
typedef enum
{
    INPUT_UBX = 0, /**<��UBX����*/
    INPUT_GPS,     /**<��־���ģʽ��UBX+RTCM����*/
} input_mode_t;

/*У���*/
typedef struct
{
    uint8_t cka;
    uint8_t ckb;
} ubx_ck_t;

/*UBX-CFG-PRT��Ϣ��Ч�غ�*/
/*��������Ϣ���������û��ѯu-boxģ���ͨ�Ŷ˿ڲ��������������ʡ�Э������״̬�����ݸ�ʽ��*/
typedef struct
{
    uint8_t portID;        /**<�˿ں�*/
    uint8_t reserved0;     /**<�����ֶ�*/
    uint16_t txReady;      /**<TX Ready�������ã�0=���ã���0�����ò�ָ��������Ϊ��*/
    uint32_t mode;         /**<�˿�ģʽ���ã����ݶ˿����Ͳ�ͬ����ʽ��ͬ������UART�Ĳ����ʡ�����λ�ȣ�*/
    uint32_t baudRate;     /**<�����ʣ�����UART�˿���Ч��*/
    uint16_t inProtoMask;  /**<����Э������*/
    uint16_t outProtoMask; /**<���Э������*/
    uint16_t flags;        /**<���ӱ�־��������չ��ʱ�ȣ�*/
    uint16_t reserved1;    /**<�����ֶ�*/
} ubx_cfg_prt_payload_t;

/* Rx NAV-PVT (ubx8) */
typedef struct
{
    uint32_t iTOW;   /**< GPS Time of Week [ms] */
    uint16_t year;   /**< Year (UTC)*/
    uint8_t month;   /**< Month, range 1..12 (UTC) */
    uint8_t day;     /**< Day of month, range 1..31 (UTC) */
    uint8_t hour;    /**< Hour of day, range 0..23 (UTC) */
    uint8_t min;     /**< Minute of hour, range 0..59 (UTC) */
    uint8_t sec;     /**< Seconds of minute, range 0..60 (UTC) */
    uint8_t valid;   /**< Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
    uint32_t tAcc;   /**< Time accuracy estimate (UTC) [ns] */
    int32_t nano;    /**< Fraction of second (UTC) [-1e9...1e9 ns] */
    uint8_t fixType; /**< GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
    uint8_t flags;   /**< Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
    uint8_t reserved1;
    uint8_t numSV;    /**< Number of SVs used in Nav Solution */
    int32_t lon;      /**< Longitude [1e-7 deg] */
    int32_t lat;      /**< Latitude [1e-7 deg] */
    int32_t height;   /**< Height above ellipsoid [mm] */
    int32_t hMSL;     /**< Height above mean sea level [mm] */
    uint32_t hAcc;    /**< Horizontal accuracy estimate [mm] */
    uint32_t vAcc;    /**< Vertical accuracy estimate [mm] */
    int32_t velN;     /**< NED north velocity [mm/s]*/
    int32_t velE;     /**< NED east velocity [mm/s]*/
    int32_t velD;     /**< NED down velocity [mm/s]*/
    int32_t gSpeed;   /**< Ground Speed (2-D) [mm/s] */
    int32_t headMot;  /**< Heading of motion (2-D) [1e-5 deg] */
    uint32_t sAcc;    /**< Speed accuracy estimate [mm/s] */
    uint32_t headAcc; /**< Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
    uint16_t pDOP;    /**< Position DOP [0.01] */
    uint16_t reserved2;
    uint32_t reserved3;
    int32_t headVeh;    /**< (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
    uint32_t reserved4; /**< (ubx8+ only) */
} ubx_nav_pvt_payload_t;

/* Rx NAV-POSLLH */
typedef struct
{
    uint32_t iTOW;  /**< GPS Time of Week [ms] */
    int32_t lon;    /**< Longitude [1e-7 deg] */
    int32_t lat;    /**< Latitude [1e-7 deg] */
    int32_t height; /**< Height above ellipsoid [mm] */
    int32_t hMSL;   /**< Height above mean sea level [mm] */
    uint32_t hAcc;  /**< Horizontal accuracy estimate [mm] */
    uint32_t vAcc;  /**< Vertical accuracy estimate [mm] */
} ubx_nav_posllh_payload_t;

/* Rx NAV-SOL�������⣨λ�á��ٶȡ�״̬��*/
typedef struct
{
    uint32_t iTOW;  /**< GPS Time of Week [ms] */
    int32_t fTOW;   /**< Fractional part of iTOW (range: +/-500000) [ns] */
    int16_t week;   /**< GPS week */
    uint8_t gpsFix; /**< GPSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GPS + dead reckoning, 5 = time only fix */
    uint8_t flags;
    int32_t ecefX;
    int32_t ecefY;
    int32_t ecefZ;
    uint32_t pAcc;
    int32_t ecefVX;
    int32_t ecefVY;
    int32_t ecefVZ;
    uint32_t sAcc;
    uint16_t pDOP; /**< Position DOP [0.01] */
    uint8_t reserved1;
    uint8_t numSV; /**< Number of SVs used in Nav Solution */
    uint32_t reserved2;
} ubx_nav_sol_payload_t;

/* Rx NAV-TIMEUTC��UTCʱ��� */
typedef struct
{
    uint32_t iTOW; /**< GPS Time of Week [ms] */
    uint32_t tAcc; /**< Time accuracy estimate (UTC) [ns] */
    int32_t nano;  /**< Fraction of second, range -1e9 .. 1e9 (UTC) [ns] */
    uint16_t year; /**< Year, range 1999..2099 (UTC) */
    uint8_t month; /**< Month, range 1..12 (UTC) */
    uint8_t day;   /**< Day of month, range 1..31 (UTC) */
    uint8_t hour;  /**< Hour of day, range 0..23 (UTC) */
    uint8_t min;   /**< Minute of hour, range 0..59 (UTC) */
    uint8_t sec;   /**< Seconds of minute, range 0..60 (UTC) */
    uint8_t valid; /**< Validity Flags (see UBX_RX_NAV_TIMEUTC_VALID_...) */
} ubx_nav_timeutc_payload_t;

/* Rx NAV-VELNED��NED�������أ�����ϵ�е��ٶ�*/
typedef struct
{
    uint32_t iTOW;   /**< GPS Time of Week [ms] */
    int32_t velN;    /**< North velocity component [cm/s]*/
    int32_t velE;    /**< East velocity component [cm/s]*/
    int32_t velD;    /**< Down velocity component [cm/s]*/
    uint32_t speed;  /**< Speed (3-D) [cm/s] */
    uint32_t gSpeed; /**< Ground speed (2-D) [cm/s] */
    int32_t heading; /**< Heading of motion 2-D [1e-5 deg] */
    uint32_t sAcc;   /**< Speed accuracy estimate [cm/s] */
    uint32_t cAcc;   /**< Course / Heading accuracy estimate [1e-5 deg] */
} ubx_nav_velned_payload_t;

/* Rx NAV-DOP ���������ӣ�������������������*/
typedef struct
{
    uint32_t iTOW; /**< GPS Time of Week [ms] */
    uint16_t gDOP; /**< Geometric DOP [0.01] */
    uint16_t pDOP; /**< Position DOP [0.01] */
    uint16_t tDOP; /**< Time DOP [0.01] */
    uint16_t vDOP; /**< Vertical DOP [0.01] */
    uint16_t hDOP; /**< Horizontal DOP [0.01] */
    uint16_t nDOP; /**< Northing DOP [0.01] */
    uint16_t eDOP; /**< Easting DOP [0.01] */
} ubx_nav_dop_payload_t;

/* Rx NAV-SVINFO Part 1 */
typedef struct
{
    uint32_t iTOW; /**< GPS Time of Week [ms] */
    uint8_t numCh; /**< Number of channels */
    uint8_t globalFlags;
    uint16_t reserved2;
} ubx_nav_svinfo_payload_part1_t;

/* Rx NAV-SVINFO Part 2 (repeated) */
typedef struct
{
    uint8_t chn;  /**< Channel number, 255 for SVs not assigned to a channel */
    uint8_t svid; /**< Satellite ID */
    uint8_t flags;
    uint8_t quality;
    uint8_t cno;   /**< Carrier to Noise Ratio (Signal Strength) [dbHz] */
    int8_t elev;   /**< Elevation [deg] */
    int16_t azim;  /**< Azimuth [deg] */
    int32_t prRes; /**< Pseudo range residual [cm] */
} ubx_nav_svinfo_payload_part2_t;

/* Rx NAV-SVIN (survey-in info),����״̬������RTK��վ���ã�*/
typedef struct
{
    uint8_t version;
    uint8_t reserved1[3];
    uint32_t iTOW;
    uint32_t dur;
    int32_t meanX;
    int32_t meanY;
    int32_t meanZ;
    int8_t meanXHP;
    int8_t meanYHP;
    int8_t meanZHP;
    int8_t reserved2;
    uint32_t meanAcc;
    uint32_t obs;
    uint8_t valid;
    uint8_t active;
    uint8_t reserved3[2];
} ubx_nav_svin_payload_t;

/* Rx MON-HW (ubx6)��Ӳ��״̬����������״̬������ˮƽ�����ڼ��Ӳ��״̬���������*/
typedef struct
{
    uint32_t pinSel;
    uint32_t pinBank;
    uint32_t pinDir;
    uint32_t pinVal;
    uint16_t noisePerMS;
    uint16_t agcCnt;
    uint8_t aStatus;
    uint8_t aPower;
    uint8_t flags;
    uint8_t reserved1;
    uint32_t usedMask;
    uint8_t VP[25];
    uint8_t jamInd;
    uint16_t reserved3;
    uint32_t pinIrq;
    uint32_t pullH;
    uint32_t pullL;
} ubx_mon_hw_payload_ubx6_t;

/* Rx MON-HW (ubx7+) ��Ӳ��״̬����������״̬������ˮƽ�����ڼ��Ӳ��״̬��������ϣ�*/
typedef struct
{
    uint32_t pinSel;
    uint32_t pinBank;
    uint32_t pinDir;
    uint32_t pinVal;
    uint16_t noisePerMS;
    uint16_t agcCnt;
    uint8_t aStatus;
    uint8_t aPower;
    uint8_t flags;
    uint8_t reserved1;
    uint32_t usedMask;
    uint8_t VP[17];
    uint8_t jamInd;
    uint16_t reserved3;
    uint32_t pinIrq;
    uint32_t pullH;
    uint32_t pullL;
} ubx_mon_hw_payload_ubx7_t;

/* Rx ACK-ACK ��ȷ����������ɹ�������ʹ��union��������Ϊ�����class��id������Ƚϣ�*/
typedef union
{
    uint16_t msg;
    struct
    {
        uint8_t clsID;
        uint8_t msgID;
    };
} ubx_ack_ack_payload_t;

/* Rx ACK-NAK ������ʧ��*/
typedef union
{
    uint16_t msg;
    struct
    {
        uint8_t clsID;
        uint8_t msgID;
    };
} ubx_ack_nak_payload_t;

/* Tx CFG-MSG ��������Ϣ�����CFG-MSG���루��ѯ��ģʽ�£�rate��Ϊ���ѯ��Ϣ��CLASS��ID���ɲ�ѯ����Ϣ���Ƶ�ʣ�*/
typedef struct
{
    union
    {
        uint16_t msg;
        struct
        {
            uint8_t msgClass;
            uint8_t msgID;
        };
    };
    uint8_t rate;
} ubx_tx_cfg_msg_payload_t;

/* Rx MON-VER Part 1 */
/*�ṩģ��İ汾��Ϣ�����۹̼��汾��Ӳ���汾��֧�ֵ�GNSSϵͳ�Լ���չ�������Ϣ*/
typedef struct
{
    uint8_t swVersion[30]; /*�̻��汾��ASCII�ַ������Կ��ַ���0�����30�ֽڣ�*/
    uint8_t hwVersion[10]; /*Ӳ���汾��ASCII�ַ������Կ��ַ���0�����10�ֽڣ�*/
} ubx_payload_rx_mon_ver_part1_t;

/* Rx MON-VER Part 2 (repeated) */
typedef struct
{
    uint8_t extension[30]; /*��չ��Ϣ��ASCII�ַ������Կ��ַ���0�����30�ֽڣ�*/
} ubx_payload_rx_mon_ver_part2_t;

/* Tx CFG-RATE ��������Ϣ���Ƶ��*/
/* Tx CFG-RATE */
typedef struct
{
    uint16_t measRate; /**< Measurement Rate, GPS measurements are taken every measRate milliseconds */
    uint16_t navRate;  /**< Navigation Rate, in number of measurement cycles. This parameter cannot be changed, and must be set to 1 */
    uint16_t timeRef;  /**< Alignment to reference time: 0 = UTC time, 1 = GPS time */
} ubx_payload_tx_cfg_rate_t;

/* Tx CFG-NAV5 */
typedef struct
{
    uint16_t mask;
    uint8_t dynModel; /**< Dynamic Platform model: 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne <4g */
    uint8_t fixMode;  /**< Position Fixing Mode: 1 2D only, 2 3D only, 3 Auto 2D/3D */
    int32_t fixedAlt;
    uint32_t fixedAltVar;
    int8_t minElev;
    uint8_t drLimit;
    uint16_t pDop;
    uint16_t tDop;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t staticHoldThresh;
    uint8_t dgpsTimeOut;
    uint8_t cnoThreshNumSVs; /**< (ubx7+ only, else 0) */
    uint8_t cnoThresh;       /**< (ubx7+ only, else 0) */
    uint16_t reserved;
    uint16_t staticHoldMaxDist; /**< (ubx8+ only, else 0) */
    uint8_t utcStandard;        /**< (ubx8+ only, else 0) */
    uint8_t reserved3;
    uint32_t reserved4;
} ubx_payload_tx_cfg_nav5_t;

/*ubx��Ϣ��Ч�غɻ�����������*/
typedef union
{
    ubx_cfg_prt_payload_t cfg_prt;                   /**<��������Ϣ*/
    ubx_nav_pvt_payload_t nav_pvt;                   /**<�����⣨λ�á��ٶȡ�״̬��*/
    ubx_nav_posllh_payload_t nav_posllh;             /**<�������λ�ã���γ�ȡ��߶ȣ�*/
    ubx_nav_sol_payload_t nav_sol;                   /**<�����⣨λ�á��ٶȡ�״̬��*/
    ubx_nav_timeutc_payload_t nav_timeutc;           /**<UTCʱ���*/
    ubx_nav_velned_payload_t nav_velned;             /**<NED�������أ�����ϵ�е��ٶ�*/
    ubx_nav_dop_payload_t nav_dop;                   /**<�������ӣ�������������������*/
    ubx_nav_svinfo_payload_part1_t nav_svinfo_part1; /**<������Ϣ������1��*/
    ubx_nav_svinfo_payload_part2_t nav_svinfo_part2; /**<������Ϣ������2��*/
    ubx_nav_svin_payload_t nav_svin;                 /**<����״̬������RTK��վ���ã�*/
    ubx_mon_hw_payload_ubx6_t mon_hw_ubx6;           /**<Ӳ��״̬��UBX6��*/
    ubx_mon_hw_payload_ubx7_t mon_hw_ubx7;           /**<Ӳ��״̬��UBX7+��*/
    ubx_ack_ack_payload_t ack_ack;                   /**<ȷ����������ɹ�*/
    ubx_ack_nak_payload_t ack_nak;                   /**<����ʧ��*/
    ubx_tx_cfg_msg_payload_t tx_cfg_msg;             /**<����������Ϣ*/
    ubx_payload_rx_mon_ver_part1_t mon_ver_part1;    /**<ģ��汾��Ϣ������1��*/
    ubx_payload_rx_mon_ver_part2_t mon_ver_part2;    /**<ģ��汾��Ϣ������2��*/
    ubx_payload_tx_cfg_rate_t cfg_rate;              /**<������Ϣ���Ƶ��*/
    ubx_payload_tx_cfg_nav5_t cfg_nav5;              /**<���õ����˲�*/
} ubx_buff_t;

/*RTCM��Ϣ*/
typedef struct
{
    uint8_t *buffer;         /**<���ݻ�����*/
    uint16_t buffer_length;  /**<���ݻ���������*/
    uint16_t buffer_index;   /**<��������һ��Ҫд���λ��*/
    uint16_t payload_length; /**<��Ч�غɳ��ȣ����ʣ�����ָ����ʲô*/
} rtcm_t;

/*���ʣ�������ʲô��*/
/*������Ϣ*/
typedef struct
{
    uint64_t timestamp; // required for logger
    uint8_t count;
    uint8_t svid[20];
    uint8_t used[20];
    uint8_t elevation[20];
    uint8_t azimuth[20];
    uint8_t snr[20];
    uint8_t _padding0[3]; // required for logger
    // const uint8_t SAT_INFO_MAX_SATELLITES = 20;

} svinfo_t;

/* Exported contants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/*UBXͬ����*/
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/*RTCMͬ����*/
#define RTCM_SYNC1 0xD3

#define UBX_CONFIG_TIMEOUT	200	// ms, timeout for waiting ACK
#define UBX_PACKET_TIMEOUT	2		// ms, if now data during this delay assume that full update received
#define DISABLE_MSG_INTERVAL	1000000		// us, try to disable message with this interval


/* Message Classes */
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_INF 0x04
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_MON 0x0A
#define UBX_CLASS_RTCM3 0xF5 /**< This is undocumented (?) */
/* Message IDs */
#define UBX_ID_NAV_POSLLH 0x02
#define UBX_ID_NAV_DOP 0x04
#define UBX_ID_NAV_SOL 0x06
#define UBX_ID_NAV_PVT 0x07
#define UBX_ID_NAV_VELNED 0x12
#define UBX_ID_NAV_TIMEUTC 0x21
#define UBX_ID_NAV_SVINFO 0x30
#define UBX_ID_NAV_SAT 0x35
#define UBX_ID_NAV_SVIN 0x3B
#define UBX_ID_NAV_RELPOSNED 0x3C
#define UBX_ID_INF_DEBUG 0x04
#define UBX_ID_INF_ERROR 0x00
#define UBX_ID_INF_NOTICE 0x02
#define UBX_ID_INF_WARNING 0x01
#define UBX_ID_ACK_NAK 0x00
#define UBX_ID_ACK_ACK 0x01
#define UBX_ID_CFG_PRT 0x00
#define UBX_ID_CFG_MSG 0x01
#define UBX_ID_CFG_RATE 0x08
#define UBX_ID_CFG_NAV5 0x24
#define UBX_ID_CFG_SBAS 0x16
#define UBX_ID_CFG_TMODE3 0x71
#define UBX_ID_MON_VER 0x04
#define UBX_ID_MON_HW 0x09
#define UBX_ID_RTCM3_1005 0x05
#define UBX_ID_RTCM3_1077 0x4D
#define UBX_ID_RTCM3_1087 0x57
/* Message Classes & IDs */
/*stm32����С�������Խ�CLASS���ڵ�8λ����������*/
#define UBX_MSG_NAV_POSLLH ((UBX_CLASS_NAV) | UBX_ID_NAV_POSLLH << 8)
#define UBX_MSG_NAV_SOL ((UBX_CLASS_NAV) | UBX_ID_NAV_SOL << 8)
#define UBX_MSG_NAV_DOP ((UBX_CLASS_NAV) | UBX_ID_NAV_DOP << 8)
#define UBX_MSG_NAV_PVT ((UBX_CLASS_NAV) | UBX_ID_NAV_PVT << 8)
#define UBX_MSG_NAV_VELNED ((UBX_CLASS_NAV) | UBX_ID_NAV_VELNED << 8)
#define UBX_MSG_NAV_TIMEUTC ((UBX_CLASS_NAV) | UBX_ID_NAV_TIMEUTC << 8)
#define UBX_MSG_NAV_SVINFO ((UBX_CLASS_NAV) | UBX_ID_NAV_SVINFO << 8)
#define UBX_MSG_NAV_SAT ((UBX_CLASS_NAV) | UBX_ID_NAV_SAT << 8)
#define UBX_MSG_NAV_SVIN ((UBX_CLASS_NAV) | UBX_ID_NAV_SVIN << 8)
#define UBX_MSG_NAV_RELPOSNED ((UBX_CLASS_NAV) | UBX_ID_NAV_RELPOSNED << 8)
#define UBX_MSG_INF_DEBUG ((UBX_CLASS_INF) | UBX_ID_INF_DEBUG << 8)
#define UBX_MSG_INF_ERROR ((UBX_CLASS_INF) | UBX_ID_INF_ERROR << 8)
#define UBX_MSG_INF_NOTICE ((UBX_CLASS_INF) | UBX_ID_INF_NOTICE << 8)
#define UBX_MSG_INF_WARNING ((UBX_CLASS_INF) | UBX_ID_INF_WARNING << 8)
#define UBX_MSG_ACK_NAK ((UBX_CLASS_ACK) | UBX_ID_ACK_NAK << 8)
#define UBX_MSG_ACK_ACK ((UBX_CLASS_ACK) | UBX_ID_ACK_ACK << 8)
#define UBX_MSG_CFG_PRT ((UBX_CLASS_CFG) | UBX_ID_CFG_PRT << 8)
#define UBX_MSG_CFG_MSG ((UBX_CLASS_CFG) | UBX_ID_CFG_MSG << 8)
#define UBX_MSG_CFG_RATE ((UBX_CLASS_CFG) | UBX_ID_CFG_RATE << 8)
#define UBX_MSG_CFG_NAV5 ((UBX_CLASS_CFG) | UBX_ID_CFG_NAV5 << 8)
#define UBX_MSG_CFG_SBAS ((UBX_CLASS_CFG) | UBX_ID_CFG_SBAS << 8)
#define UBX_MSG_CFG_TMODE3 ((UBX_CLASS_CFG) | UBX_ID_CFG_TMODE3 << 8)
#define UBX_MSG_MON_HW ((UBX_CLASS_MON) | UBX_ID_MON_HW << 8)
#define UBX_MSG_MON_VER ((UBX_CLASS_MON) | UBX_ID_MON_VER << 8)
#define UBX_MSG_RTCM3_1005 ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1005 << 8)
#define UBX_MSG_RTCM3_1077 ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1077 << 8)
#define UBX_MSG_RTCM3_1087 ((UBX_CLASS_RTCM3) | UBX_ID_RTCM3_1087 << 8)

/*UBX��Ϣ��Ч�غɳ���*/
#define NAV_PVT_PAYLOAD_LENGTH_UBX8 sizeof(ubx_nav_pvt_payload_t)
#define NAV_PVT_PAYLOAD_LENGTH_UBX7 sizeof(ubx_nav_pvt_payload_t) - 8

/*������Ϣ���������ʱ����*/
#define MSG_DISABLE_TIME_INTERVAL 1000 /*ms*/

/*���ջ����֧������ͨ����,���ʣ�����Ϊʲô�趨Ϊ20��*/
#define MAX_SUPPORTED_CHANNELS 20

/*FNV-1(Fowler-Noll-Vo)��ϣ�㷨(32λ�汾)*/
#define FNV1_32_INIT ((uint32_t)0x811c9dc5)  // init value for FNV1 hash algorithm
#define FNV1_32_PRIME ((uint32_t)0x01000193) // magic prime for FNV1 hash algorithm

/*UBX-CFG-PRT��Ч�غ�����*/
/*�˿ں�*/
#define UBX_CFG_PRT_portID_UART1 0x01
#define UBX_CFG_PRT_portID_USB 0x03
#define UBX_CFG_PRT_portID_SPI 0x04
/*�˿�ģʽ*/
#define UBX_CFG_PRT_mode_UART 0x000008D0
#define UBX_CFG_PRT_mode_SPI 0x00000100
/*������*/
#define UBX_CFG_PRT_baudRate 38400
/*����Э������*/
/*ʹ��UBX��RTCM3���룬RTCM3����������λ����*/
#define UBX_CFG_PRT_inProtoMask_GPS (0x01 << 5 | 0x01) /*��׼ģʽ*/
#define UBX_CFG_PRT_inProtoMask_UBX (0x01)
/*���Э������*/
#define UBX_CFG_PRT_outProtoMask_RTCM (0x01 << 5 | 0x01) /*�������RTCMЭ����ô�������ת��RTCM���ݡ���ΪRTCM��վ��*/
#define UBX_CFG_PRT_outProtoMask_GPS (0x01)              /*��׼ģʽ*/

/* RX NAV-PVT message content details */
/*   Bitfield "flags" masks */
#define UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK 0x01    /**< gnssFixOK (��λ����Ч(i.e within DOP & accuracy masks)) */
#define UBX_RX_NAV_PVT_FLAGS_DIFFSOLN 0x02     /**< diffSoln (1 if differential corrections were applied) */
#define UBX_RX_NAV_PVT_FLAGS_PSMSTATE 0x1C     /**< psmState (Power Save Mode state (see Power Management)) */
#define UBX_RX_NAV_PVT_FLAGS_HEADVEHVALID 0x20 /**< headVehValid (Heading of vehicle is valid) */
#define UBX_RX_NAV_PVT_FLAGS_CARRSOLN 0xC0     /**< Carrier phase range solution (RTK mode) */
/* Bitfield "valid" masks */
#define UBX_RX_NAV_PVT_VALID_VALIDDATE 0x01     /**< validDate (Valid UTC Date) */
#define UBX_RX_NAV_PVT_VALID_VALIDTIME 0x02     /**< validTime (Valid UTC Time) */
#define UBX_RX_NAV_PVT_VALID_FULLYRESOLVED 0x04 /**< fullyResolved (1 = UTC Time of Day has been fully resolved (no seconds uncertainty)) */

/* RX NAV-TIMEUTC message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDTOW 0x01    /**< validTOW (1 = Valid Time of Week) */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDKWN 0x02    /**< validWKN (1 = Valid Week Number) */
#define UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC 0x04    /**< validUTC (1 = Valid UTC Time) */
#define UBX_RX_NAV_TIMEUTC_VALID_UTCSTANDARD 0xF0 /**< utcStandard (0..15 = UTC standard identifier) */

/* TX CFG-RATE message contents */
#define UBX_TX_CFG_RATE_MEASINTERVAL 200 /*< 200ms for 5Hz */
#define UBX_TX_CFG_RATE_NAVRATE 1        /*< cannot be changed */
#define UBX_TX_CFG_RATE_TIMEREF 0        /*< 0: UTC, 1: GPS time */

/* TX CFG-NAV5 message contents */
#define UBX_TX_CFG_NAV5_MASK		0x0005		/**< Only update dynamic model and fix mode */
#define UBX_TX_CFG_NAV5_DYNMODEL	7		/**< 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne <4g */
#define UBX_TX_CFG_NAV5_DYNMODEL_RTCM	2
#define UBX_TX_CFG_NAV5_FIXMODE		2		/**< 1 2D only, 2 3D only, 3 Auto 2D/3D */

#define M_DEG_TO_RAD_F 0.01745329251994f

/*�ֽ���ת��*/
#define SWAP16(X) ((((X) >> 8) & 0x00ff) | (((X) << 8) & 0xff00))

/* Exported functions ------------------------------------------------------- */
int8_t gnss_configure(uint32_t *baudrate, output_mode_t output_mode, input_mode_t input_mode, uint32_t timeout);
uint8_t read_rb_process(void);
#endif /* __UBX_H */
