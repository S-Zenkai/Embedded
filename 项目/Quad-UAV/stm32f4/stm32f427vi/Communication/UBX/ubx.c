/**
 ******************************************************************************
 * @file    ubx.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/04/09
 * @brief   sbus通信协议解码
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "ubx.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*GNSS接收器配置标志*/
uint8_t gnss_config_flag = 0;
/*GNSS是否支持使用NAV-PVT消息*/
uint8_t use_nav_pvt = 0;

/*UBX协议解码状态*/
ubx_decode_state_t ubx_decode_state = UBX_DECODE_SYNC1;
/*接收消息状态*/
ubx_rx_msg_state_t ubx_rx_msg_state = UBX_RX_MSG_HANDLE;
/*UBX确认状态*/
ubx_ack_state_t ubx_ack_state = UBX_ACK_IDLE;

/*UBX协议校验和A*/
uint8_t ubx_cka = 0;
/*UBX协议校验和B*/
uint8_t ubx_ckb = 0;
/*UBX消息类型*/
uint16_t ubx_msg_type = 0;

/*UBX协议有效载荷长度*/
uint16_t ubx_payload_length = 0;
/*UBX协议有效载荷结构体索引*/
uint8_t ubx_payload_index = 0;

/*RTCM消息结构体*/
rtcm_t *rtcm_msg = NULL;

/*卫星信息*/
svinfo_t *svinfo = NULL;

/*GSP模块解析后的定位、速度、时间和状态信息结构体*/
struct vehicle_gps_info_s *vehicle_gps_info = NULL;

/*UBX消息缓冲区*/
ubx_buff_t ubx_buff;

/*上一次发送禁用消息命令的时间*/
uint32_t last_disable_msg_time = 0;

/*UBX版本号*/
uint32_t ubx_version;

/*疑问：这个变量有什么用*/
uint64_t _last_timestamp_time;

/*速度和位置数据接收计数器*/
uint8_t _rate_count_lat_lon;
uint8_t _rate_count_vel;
/*位置数据接收标志*/
uint8_t _got_posllh;
/*速度数据接收标志*/
uint8_t _got_velned;

/*等待应答的消息*/
uint16_t wait_ack_msg;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint8_t ParseData(uint8_t data);
void ubx_calc_ck(const uint8_t *data, uint16_t len, ubx_ck_t *ck);

/**
 * @brief  向GNSS接收器发送消息
 * @note
 * @param  buff:数据指针
 * @param  len:数据长度
 * @retval 发送成功返回true，否则返回false
 */
bool send_msg(const void *buff, uint16_t len)
{
    return USART_SendBytes(UART4, (uint8_t *)buff, len, 1000);
}

/**
 * @brief  向GNSS接收器发送完整UBX消息
 * @note
 * @param  无
 * @retval 无
 */
bool send_ubx_msg(uint16_t msg_type, const uint8_t *payload, uint16_t payload_len)
{
    ubx_ck_t ck =  {0, 0};
    // uint16_t sync = UBX_SYNC1 | ((UBX_SYNC2 << 8) & 0xFF00);
    uint8_t sync[] = {UBX_SYNC1, UBX_SYNC2}; // 0xB5, 0x62
    /*计算校验和*/
    /*类型*/
    ubx_calc_ck((uint8_t *)&msg_type, sizeof(msg_type), &ck); /*stm32采用小端序*/
    /*长度*/
    ubx_calc_ck((uint8_t *)&payload_len, sizeof(payload_len), &ck);
    if (payload != NULL)
    {
        ubx_calc_ck(payload, payload_len, &ck);
    }
    /*发送消息*/
    if (!send_msg((uint8_t *)&sync, sizeof(sync)))
    {
        return false;
    }
    if (!send_msg((uint8_t *)&msg_type, sizeof(msg_type)))
    {
        return false;
    }
    if (!send_msg((uint8_t *)&payload_len, sizeof(payload_len)))
    {
        return false;
    }
    if (!send_msg(payload, payload_len))
    {
        return false;
    }
    if (!send_msg((uint8_t *)&ck, sizeof(ck)))
    {
        return false;
    }
    return true;
}

/**
 * @brief  从GNSS模块的环形缓冲区读取数据，并存储在ubx_buff中
 * @note
 * @param  无
 * @retval 无
 */
uint8_t read_rb(uint8_t *buff, uint16_t len, uint32_t timeout)
{
    uint64_t t_start = GetTick();
    uint16_t i = 0;
    while (1)
    {
        /*检查缓冲区是否溢出*/
        if (GNSS_RB_CheckOverflow() == 1)
        {
            GPS_DEBUG("GNSS RB overflow\r\n");
            GNSS_RB_Clear();
            return 0;
        }
        /*检查缓冲区中是否存在新数据*/
        while (!GNSS_RB_IsEmpty())
        {
            buff[i++] = GNSS_RB_PopData();
            if (i >= len)
            {
                return i;
            }
            if (GetTick() >= t_start + timeout)
            {
                return i;
            }
        }
    }
}

/**
 * @brief  从环形缓冲区读取数据，并预处理数据
 * @note
 * @param  无
 * @retval 无
 */
uint8_t read_rb_process(void)
{
    uint8_t ret;
    /*检查缓冲区是否溢出*/
    if (GNSS_RB_CheckOverflow() == 1)
    {
        GPS_DEBUG("GNSS RB overflow\r\n");
        GNSS_RB_Clear();
        return 0;
    }
    /*检查缓冲区中是否存在新数据*/
    while (!GNSS_RB_IsEmpty())
    {
        uint8_t u = GNSS_RB_PopData();
        /*疑问：这里为什么使用或运算符*/
        ret |= ParseData(u);
    }
    return ret;
}

/**
 * @brief  UBX协议解码初始化
 * @note
 * @param  无
 * @retval 无
 */
void ubx_decode_init(void)
{
    ubx_decode_state = UBX_DECODE_SYNC1;
    ubx_cka = 0;
    ubx_ckb = 0;
    ubx_payload_length = 0;
    ubx_payload_index = 0;
}

/**
 * @brief  计算校验和（单字节）
 * @note
 * @param  data:数据
 * @retval 无
 */
void ubx_calc_ck_byte(uint8_t data)
{
    ubx_cka += data;
    ubx_ckb += ubx_cka;
}

/**
 * @brief  计算校验和（多字节）
 * @note
 * @param  无
 * @retval 无
 */
void ubx_calc_ck(const uint8_t *data, uint16_t len, ubx_ck_t *ck)
{
    for (uint16_t i = 0; i < len; i++)
    {
        ck->cka += data[i];
        ck->ckb += ck->cka;
    }
}

/**
 * @brief  配置u-blox模块中指定消息的输出速率
 * @note
 * @param  无
 * @retval 无
 */
bool set_ubx_msg_rate(uint16_t msg_type, uint16_t rate)
{
    ubx_tx_cfg_msg_payload_t cfg_msg_payload; /*代码：原代码中解释这里不使用ubx_buff是为了支持并发或中断操作*/
    cfg_msg_payload.msg = msg_type;
    cfg_msg_payload.rate = rate;
    return send_ubx_msg(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg_payload, sizeof(cfg_msg_payload));
}

/**
 * @brief  有效载荷接收初始化，验证消息类型和有效载荷长度、设置适当的接收状态，并准备有效载荷处理来初始化UBX消息的有效载荷接收
 *         这里会设置UBX消息状态机，在UBX消息接收完成后会根据状态机判断是否处理接收到消息的有效载荷(payloadRxDone)
 * @note
 * @param  无
 * @retval 返回true表示消息处理成功，返回false表示消息处理失败
 */
bool payload_rx_init(void)
{
//    uint8_t ret;
    /*默认状态*/
    ubx_rx_msg_state = UBX_RX_MSG_HANDLE;
    switch (ubx_msg_type)
    {
    case UBX_MSG_NAV_PVT:
        /*如果有效载荷长度不符合协议标准，则设置为错误状态*/
        if (ubx_payload_length != NAV_PVT_PAYLOAD_LENGTH_UBX8 && ubx_payload_length != NAV_PVT_PAYLOAD_LENGTH_UBX7)
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        /*如果初始化时未配置消息，则设置为忽略状态*/
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*如果GNSS不支持使用NAV-PVT消息，则设置为禁用状态*/
        else if (!use_nav_pvt)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        break;
    case UBX_MSG_INF_DEBUG:
    case UBX_MSG_INF_ERROR:
    case UBX_MSG_INF_NOTICE:
    case UBX_MSG_INF_WARNING:
        /*无论配置状态如何都处理INF消息*/
        /*INF消息都是变字长的，不检测有效载荷长度，只是防止UBX——buff溢出(后续操作会在ubx_buff_t[ubx_payload_length]添加0，下面操作正是避免加0时溢出)*/
        if (ubx_payload_length >= sizeof(ubx_buff_t))
        {
            /*避免大于UBX消息有效载荷缓冲区*/
            ubx_payload_length = sizeof(ubx_buff_t) - 1;
        }
        break;
    /*下面配置NAV-POSLLH、NAV-SOL、NAV-TIMEUTC、NAV-VELNED消息，配置过程类似*/
    /*NAV-POSLLH消息,大地坐标位置（经纬度、高度）*/
    case UBX_MSG_NAV_POSLLH:
        if (ubx_payload_length != sizeof(ubx_nav_posllh_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*如果GNSS支持使用NAV-PVT消息，则禁用NAV-POSLLH消息，优先使用NAV-PVT消息*/
        else if (use_nav_pvt)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        break;
    /*导航解（位置、速度、状态）*/
    case UBX_MSG_NAV_SOL:
        if (ubx_payload_length != sizeof(ubx_nav_sol_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*如果GNSS支持使用NAV-PVT消息，则禁用NAV-SOL消息，优先使用NAV-PVT消息*/
        else if (use_nav_pvt)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        break;
    /*UTC时间解*/
    case UBX_MSG_NAV_TIMEUTC:
        if (ubx_payload_length != sizeof(ubx_nav_timeutc_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*如果GNSS支持使用NAV-PVT消息，则禁用NAV-TIMEUTC消息，优先使用NAV-PVT消息*/
        else if (use_nav_pvt)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        break;
    /*NED（北东地）坐标系中的速度*/
    case UBX_MSG_NAV_VELNED:
        if (ubx_payload_length != sizeof(ubx_nav_velned_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*如果GNSS支持使用NAV-PVT消息，则禁用NAV-VELNED消息，优先使用NAV-PVT消息*/
        else if (use_nav_pvt)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        break;
    /*DOP（精度因子）,验证长度与配置状态*/
    case UBX_MSG_NAV_DOP:
        if (ubx_payload_length != sizeof(ubx_nav_dop_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        break;
    /*NAV-SVINFO，卫星信息（卫星数量、信号强度）*/
    case UBX_MSG_NAV_SVINFO:
        /*疑问：这里未校验有效载荷长度，可能是因为SVINFO消息是变字长的（有效载荷中包含一个动态重复的卫星信息块，数量由字段numCh决定），也可能是推迟到了其他函数中校验*/
        if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*如果未分配卫星信息缓存*/
        else if (svinfo == NULL)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        else
        {
            /*memset比手动循环要快，因为它通常使用底层硬件指令优化*/
            memset(svinfo, 0, sizeof(svinfo_t));
        }
        break;
    /*测量状态（用于RTK基站设置），支持RTK应用，在配置完成后处理测量数据*/
    case UBX_MSG_NAV_SVIN:
        if (ubx_payload_length != sizeof(ubx_nav_svin_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        break;
    /*模块版本信息，始终处理版本信息，因为其对识别模块功能至关重要*/
    case UBX_MSG_MON_VER:
        break;
    /*HON-HW：硬件状态（例如天线状态、噪声水平，用于监测硬件状态，用于诊断*/
    case UBX_MSG_MON_HW:
        if (ubx_payload_length != sizeof(ubx_mon_hw_payload_ubx6_t) || ubx_payload_length != sizeof(ubx_mon_hw_payload_ubx7_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        break;
    /*ACK-ACK：确认消息，用于确认消息发送成功*/
    case UBX_MSG_ACK_ACK:
        if (ubx_payload_length != sizeof(ubx_ack_ack_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        break;
    /*ACK-NAK：配置失败*/
    case UBX_MSG_ACK_NAK:
        if (ubx_payload_length != sizeof(ubx_ack_nak_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        break;
    /*默认：对于其他UBX消息类型，设置为禁用状态*/
    default:
        ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        break;
    }
    /*状态处理与返回值*/
    switch (ubx_rx_msg_state)
    {
    /*消息未配置时也允许后续操作*/
    case UBX_RX_MSG_HANDLE:
    case UBX_RX_MSG_IGNORE:
        return true;
    case UBX_RX_MSG_DISABLE:
        /*一定时间间隔发送一次禁用消息配置，避免频繁发送禁用命令*/
        if (GetTick() >= last_disable_msg_time + MSG_DISABLE_TIME_INTERVAL)
        {
            set_ubx_msg_rate(ubx_msg_type, 0);
            last_disable_msg_time = GetTick();
        }
        return false;
    case UBX_RX_MSG_ERROR_LENGTH:
        return false;
    default:
        return false;
    }
}

/**
 * @brief  将接收到的有效载荷放入缓冲区(除NAV-SVINFO)
 * @note
 * @param  data:接收到的数据(字节)
 * @retval 返回1表示所有有效载荷接收完成，返回0表示有效载荷接收未完成，返回-1表示错误
 */
int8_t payload_rx_process(uint8_t data)
{
    int8_t ret = -1;
    uint8_t *p_buff = (uint8_t *)&ubx_buff;
    p_buff[ubx_payload_index++] = data;
    if (ubx_payload_index >= ubx_payload_length)
    {
        ret = 1;
    }
    else
    {
        ret = 0;
    }
    return ret;
}

/**
 * @brief  NAV-SVINFO消息处理
 * @note
 * @param  无
 * @retval 无
 */
int8_t nav_svinfo_payload_process(uint8_t data)
{
    int8_t ret = -1;
    uint8_t *p_buff = (uint8_t *)&ubx_buff;
    /*处理前半部分，直接放入buff缓冲区*/
    if (ubx_payload_index < sizeof(ubx_nav_svinfo_payload_part1_t))
    {
        p_buff[ubx_payload_index] = data;
    }
    /*在接受完part1，开始解析part1并接收part2，解析后接收part2时即可覆盖part1*/
    else
    {
        /*解析part1*/
        if (ubx_payload_index == sizeof(ubx_nav_svinfo_payload_part1_t))
        {
            /*numCh中记录了接收机硬件支持或当前配置通道总数，后续重复快数量等于numch*/
            svinfo->count = (ubx_buff.nav_svinfo_part1.numCh < MAX_SUPPORTED_CHANNELS ? ubx_buff.nav_svinfo_part1.numCh : MAX_SUPPORTED_CHANNELS);
        }
        /*接收part2*/
        /*检查ubx_payload_index在nav_svinfo有效载荷中的位置*/
        if (ubx_payload_index < sizeof(ubx_nav_svinfo_payload_part1_t) + svinfo->count * sizeof(ubx_nav_svinfo_payload_part2_t))
        {
            uint8_t index = (ubx_payload_index - sizeof(ubx_nav_svinfo_payload_part1_t)) % sizeof(ubx_nav_svinfo_payload_part2_t);
            p_buff[index] = data;
            /*接收到一组卫星数据，开始解析*/
            if (index == sizeof(ubx_nav_svinfo_payload_part2_t) - 1)
            {
                /*检查第几个卫星*/
                uint8_t sv_index = (ubx_payload_index - sizeof(ubx_nav_svinfo_payload_part1_t)) / sizeof(ubx_nav_svinfo_payload_part2_t);
                /*提取flags的最低位(svUsed:1表示卫星信号被用于导航解算(计算位置、速度、时间)),*/
                svinfo->used[sv_index] = (uint8_t)(ubx_buff.nav_svinfo_part2.flags & 0x01);
                svinfo->snr[sv_index] = ubx_buff.nav_svinfo_part2.cno;                                            /*存储信噪比*/
                svinfo->elevation[sv_index] = ubx_buff.nav_svinfo_part2.elev;                                     /*存储仰角*/
                svinfo->azimuth[sv_index] = (uint8_t)((float)(ubx_buff.nav_svinfo_part2.azim) * 255.0f / 360.0f); /*存储方位角,转化为0-255，可能是节省内存*/
                svinfo->svid[sv_index] = ubx_buff.nav_svinfo_part2.svid;                                          /*存储卫星ID*/
            }
        }
    }
    if (++ubx_payload_index >= ubx_payload_length)
    {
        ret = 1;
    }
    else
    {
        ret = 0;
    }
    return ret;
}

/**
 * @brief  FNV-1(Fowler-Noll-Vo)哈希算法(32位版本)
 * @note
 * @param  str:字符串
 * @param  hval:初始哈希值，可以传入上一次的哈希计算结果，实现链式哈希，从而得到多个数据片段连接起来的整体哈希值
 * @retval 返回哈希值
 */
static uint32_t fnv1_hash(uint8_t *str, uint32_t hval)
{
    uint8_t *s = str;
    while (*s)
    {
#if defined(NO_FNV_GCC_OPTIMIZATION)
        hval *= FNV1_32_PRIME;
#else
        hval += (hval << 1) + (hval << 4) + (hval << 7) + (hval << 8) + (hval << 24);
#endif
        hval ^= (uint32_t)(*s++);
    }
    return hval;
}

/**
 * @brief  MON-VER消息处理
 * @note
 * @param  无
 * @retval 无
 */
int8_t mon_ver_payload_process(uint8_t data)
{
    int8_t ret = -1;
    uint8_t *p_buff = (uint8_t *)&ubx_buff;
    /*处理前半部分，直接放入buff缓冲区*/
    if (ubx_payload_index < sizeof(ubx_payload_rx_mon_ver_part1_t))
    {
        p_buff[ubx_payload_index] = data;
    }
    /*处理后半部分，直接放入buff缓冲区*/
    else
    {
        /*处理part1*/
        if (ubx_payload_index == sizeof(ubx_payload_rx_mon_ver_part1_t))
        {
            /*将固化版本和硬件版本压缩为32位哈希值*/
            ubx_version = fnv1_hash(ubx_buff.mon_ver_part1.swVersion, FNV1_32_INIT);
            ubx_version = fnv1_hash(ubx_buff.mon_ver_part1.hwVersion, ubx_version);
        }
        uint8_t index = (ubx_payload_index - sizeof(ubx_payload_rx_mon_ver_part1_t)) % sizeof(ubx_payload_rx_mon_ver_part2_t);
        p_buff[index] = data;
        /*解析part2*/
        if (index == sizeof(ubx_payload_rx_mon_ver_part2_t) - 1)
        {
            /*打印part2内容*/
        }
    }
    if (++ubx_payload_index >= ubx_payload_length)
    {
        ret = 1;
    }
    else
    {
        ret = 0;
    }
    return ret;
}

/**
 * @brief  处理已接收并校验通过的UBX消息有效载荷(在接收完一帧数据后调用)
 * @note
 * @param  无
 * @retval 无
 */
int payloadRxDone(void)
{
    uint8_t ret = 0;
    /*验证消息状态，这里使用了UBX状态机*/
    if (ubx_rx_msg_state != UBX_RX_MSG_HANDLE)
    {
        return ret;
    }
    switch (ubx_msg_type)
    {
    case UBX_MSG_NAV_PVT:
        /*验证定位解是否有效*/
        if (ubx_buff.nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK == 1)
        {
            /*设置fix_type为fixType(0:无效,1:2D,2:3D)*/
            vehicle_gps_info->fix_type = ubx_buff.nav_pvt.fixType;
            if (ubx_buff.nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_DIFFSOLN)
            {
                vehicle_gps_info->fix_type = 4; /*差分GPS(DGPS)*/
            }
            /*载波相位解状态*/
            uint8_t carr_soln = ubx_buff.nav_pvt.flags >> 6;
            if (carr_soln == 1)
            {
                vehicle_gps_info->fix_type = 5; /*Float RTK*/
            }
            else if (carr_soln == 2)
            {
                vehicle_gps_info->fix_type = 6; /*Fix RTK*/
            }
            /*速度信息有效*/
            vehicle_gps_info->vel_ned_valid = 1;
        }
        else
        {
            vehicle_gps_info->fix_type = 0;
            vehicle_gps_info->vel_ned_valid = 0;
        }
        /*将NAV_PVT中的各个字段提取出来，进行必要的转换*/
        vehicle_gps_info->satellites_used = ubx_buff.nav_pvt.numSV;
        vehicle_gps_info->lat = ubx_buff.nav_pvt.lat;
        vehicle_gps_info->lon = ubx_buff.nav_pvt.lon;
        vehicle_gps_info->alt = ubx_buff.nav_pvt.hMSL;
        /*从毫米转为米，1e-3f=0.001f*/
        vehicle_gps_info->eph = (float)ubx_buff.nav_pvt.hAcc * 1e-3f;
        vehicle_gps_info->epv = (float)ubx_buff.nav_pvt.vAcc * 1e-3f;
        vehicle_gps_info->s_variance_m_s = (float)ubx_buff.nav_pvt.sAcc * 1e-3f;
        vehicle_gps_info->vel_m_s = (float)ubx_buff.nav_pvt.gSpeed * 1e-3f;
        vehicle_gps_info->vel_n_m_s = (float)ubx_buff.nav_pvt.velN * 1e-3f;
        vehicle_gps_info->vel_e_m_s = (float)ubx_buff.nav_pvt.velE * 1e-3f;
        vehicle_gps_info->vel_d_m_s = (float)ubx_buff.nav_pvt.velD * 1e-3f;
        vehicle_gps_info->cog_rad = (float)ubx_buff.nav_pvt.headMot * M_DEG_TO_RAD_F * 1e-5f;
        vehicle_gps_info->c_variance_rad = (float)ubx_buff.nav_pvt.headAcc * M_DEG_TO_RAD_F * 1e-5f;

        GPS_DEBUG("eph: %f ,epv:%f\r\n", vehicle_gps_info->eph, vehicle_gps_info->epv);
        GPS_DEBUG("vehicle_gps_info lat:%lf ,lon:%lf ,alt:%lf \r\n", (double)vehicle_gps_info->lat / 10000000.0, (double)vehicle_gps_info->lon / 10000000.0, (double)vehicle_gps_info->alt / 1000000.0);
        GPS_DEBUG("gps vel N:%f ,E:%f ,D:%f \r\n", vehicle_gps_info->vel_n_m_s, vehicle_gps_info->vel_e_m_s, vehicle_gps_info->vel_d_m_s);
        GPS_DEBUG("gps cog_rad:%f \r\n", vehicle_gps_info->cog_rad);
        /*时间戳与日期有效性检查(时间日期都有效且完全解析)*/
        /*疑问：这里好像没什么用*/
        if ((ubx_buff.nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDDATE) &&
            (ubx_buff.nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDTIME) &&
            (ubx_buff.nav_pvt.valid & UBX_RX_NAV_PVT_VALID_FULLYRESOLVED))
        {
            struct tm time_info;
            /* convert to unix timestamp */
            time_info.tm_year = ubx_buff.nav_pvt.year - 1900;
            time_info.tm_mon = ubx_buff.nav_pvt.month - 1;
            time_info.tm_mday = ubx_buff.nav_pvt.day;
            time_info.tm_hour = ubx_buff.nav_pvt.hour;
            time_info.tm_min = ubx_buff.nav_pvt.min;
            time_info.tm_sec = ubx_buff.nav_pvt.sec;
            vehicle_gps_info->time_utc_usec = 0; /*这里未实现转换Unix时间戳*/
        }
        /*系统时间戳*/
        vehicle_gps_info->timestamp = GetTick();
        _last_timestamp_time = vehicle_gps_info->timestamp;
        /*增加速度和位置数据的接收计数*/
        _rate_count_lat_lon++;
        _rate_count_vel++;
        /*标志已收到有效的位置和速度信息*/
        _got_posllh = 1;
        _got_velned = 1;
        ret = 1;
        break;
    case UBX_MSG_INF_DEBUG:
    case UBX_MSG_INF_NOTICE:
    case UBX_MSG_INF_ERROR:
    case UBX_MSG_INF_WARNING:
		{
        uint8_t *p_buff = (uint8_t *)&ubx_buff;
        /*在ubx_buff末尾添加0，表示字符串结束*/
        p_buff[ubx_payload_length] = 0;
        /*这里未设置返回值，有没有问题*/
        break;
		}
    case UBX_MSG_NAV_POSLLH:
        vehicle_gps_info->lat = ubx_buff.nav_posllh.lat;
        vehicle_gps_info->lon = ubx_buff.nav_posllh.lon;
        vehicle_gps_info->alt = ubx_buff.nav_posllh.hMSL;
        vehicle_gps_info->eph = (float)ubx_buff.nav_posllh.hAcc * 1e-3f; // from mm to m
        vehicle_gps_info->epv = (float)ubx_buff.nav_posllh.vAcc * 1e-3f; // from mm to m
        vehicle_gps_info->alt_ellipsoid = ubx_buff.nav_posllh.height;
        vehicle_gps_info->timestamp = GetTick();
        /*疑问：这里定义了一个与全局变量相同的_rate_count_lat_lon，可能有些bug*/
        uint8_t _rate_count_lat_lon;
        _rate_count_lat_lon++;
        _got_posllh = 1;
        // GPS_DEBUG("_gps_position lat:%d ,lon:%d ,alt:%d \r\n" , _gps_position->lat,_gps_position->lon,_gps_position->alt);
        ret = 1;
        break;
    case UBX_MSG_NAV_SOL:
        vehicle_gps_info->fix_type = ubx_buff.nav_sol.gpsFix;
        vehicle_gps_info->s_variance_m_s = (float)ubx_buff.nav_sol.sAcc * 1e-2f; // from cm to m
        vehicle_gps_info->satellites_used = ubx_buff.nav_sol.numSV;
        ret = 1;
        break;
    case UBX_MSG_NAV_DOP:
        /*提供定位质量指标，评估几何分布*/
        vehicle_gps_info->hdop = ubx_buff.nav_dop.hDOP * 1e-2f;
        vehicle_gps_info->vdop = ubx_buff.nav_dop.vDOP * 1e-2f;
        GPS_DEBUG("hdop:%f ,vdop:%f\r\n", vehicle_gps_info->hdop, vehicle_gps_info->vdop);
        ret = 1;
        break;
    case UBX_MSG_NAV_TIMEUTC:
        /*检查时间有效性*/
        if (ubx_buff.nav_timeutc.valid & UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC)
        {
            struct tm time_info;
            /* convert to unix timestamp */
            time_info.tm_year = ubx_buff.nav_timeutc.year - 1900;
            time_info.tm_mon = ubx_buff.nav_timeutc.month - 1;
            time_info.tm_mday = ubx_buff.nav_timeutc.day;
            time_info.tm_hour = ubx_buff.nav_timeutc.hour;
            time_info.tm_min = ubx_buff.nav_timeutc.min;
            time_info.tm_sec = ubx_buff.nav_timeutc.sec;
            vehicle_gps_info->time_utc_usec = 0; /*这里未实现转换Unix时间戳*/
        }
        _last_timestamp_time = GetTick();
        ret = 1;
        break;
    case UBX_MSG_NAV_SVINFO:
        /*NAV_SVINFO消息之前已处理，这里添加一个时间戳*/
        svinfo->timestamp = GetTick();
        ret = 2;
        break;
    case UBX_MSG_NAV_SVIN:
        ret = 1;
        break;
    case UBX_MSG_NAV_VELNED:
        vehicle_gps_info->vel_m_s = (float)ubx_buff.nav_velned.speed * 1e-2f;
        vehicle_gps_info->vel_n_m_s = (float)ubx_buff.nav_velned.velN * 1e-2f; /* NED NORTH velocity */
        vehicle_gps_info->vel_e_m_s = (float)ubx_buff.nav_velned.velE * 1e-2f; /* NED EAST velocity */
        vehicle_gps_info->vel_d_m_s = (float)ubx_buff.nav_velned.velD * 1e-2f; /* NED DOWN velocity */
        vehicle_gps_info->cog_rad = (float)ubx_buff.nav_velned.heading * M_DEG_TO_RAD_F * 1e-5f;
        vehicle_gps_info->c_variance_rad = (float)ubx_buff.nav_velned.cAcc * M_DEG_TO_RAD_F * 1e-5f;
        vehicle_gps_info->vel_ned_valid = 1;
        _rate_count_vel++;
        _got_velned = 1;
        ret = 1;
        // GPS_DEBUG("gps vel N:%f ,E:%f ,D:%f \r\n" , _gps_position->vel_n_m_s,_gps_position->vel_e_m_s,_gps_position->vel_d_m_s);
    case UBX_MSG_MON_VER:
        ret = 1;
        break;
    case UBX_MSG_MON_HW:
        switch (ubx_payload_length)
        {
        case sizeof(ubx_mon_hw_payload_ubx6_t):
            vehicle_gps_info->noise_per_ms = ubx_buff.mon_hw_ubx6.noisePerMS;
            vehicle_gps_info->jamming_indicator = ubx_buff.mon_hw_ubx6.jamInd;
            ret = 1;
            break;
        case sizeof(ubx_mon_hw_payload_ubx7_t):
            vehicle_gps_info->noise_per_ms = ubx_buff.mon_hw_ubx7.noisePerMS;
            vehicle_gps_info->jamming_indicator = ubx_buff.mon_hw_ubx7.jamInd;
            ret = 1;
            break;
        default:
            ret = 0;
            break;
        }
        break;
    case UBX_MSG_ACK_ACK:
        if (ubx_ack_state == UBX_ACK_WAITING && ubx_buff.ack_ack.msg == wait_ack_msg)
        {
            ubx_ack_state = UBX_ACK_GOT_ACK;
        }
        ret = 1;
        break;
    case UBX_MSG_ACK_NAK:
        if (ubx_ack_state == UBX_ACK_WAITING && ubx_buff.ack_nak.msg == wait_ack_msg)
        {
            ubx_ack_state = UBX_ACK_GOT_NAK;
        }
        ret = 1;
        break;
    default:
        break;
    }
    /*疑问:这里好像没什么用*/
    if (ret > 0)
    {
        vehicle_gps_info->timestamp_time_relative = (int32_t)(vehicle_gps_info->timestamp - _last_timestamp_time);
    }
    return ret;
}

/**
 * @brief  解析UBX数据、RTCM数据
 * @note
 * @param  data:接收到的数据
 * @retval 返回1和2表示一整个数据帧解析成功，返回0表示解析失败或未完成
 */
uint8_t ParseData(uint8_t data)
{
    int8_t ret;
    switch (ubx_decode_state)
    {
    /*data为UBX同步字或RTCM同步字（若支持RTCM）*/
    case UBX_DECODE_SYNC1:
        if (data == UBX_SYNC1)
        {
            ubx_decode_state = UBX_DECODE_SYNC2;
        }
        else if (data == RTCM_SYNC1 && rtcm_msg != NULL)
        {
            ubx_decode_state = RTCM_DECODE_SYNC1;
            rtcm_msg->buffer[rtcm_msg->buffer_index++] = data;
            /*疑问：这里要吗？*/
            if (rtcm_msg->buffer_index >= rtcm_msg->buffer_length)
            {
                rtcm_msg->buffer_index = 0;
            }
        }
        break;
    case UBX_DECODE_SYNC2:
        if (data == UBX_SYNC2)
        {
            ubx_decode_state = UBX_DECODE_CLASS;
        }
        else
        {
            ubx_decode_init(); /*疑问：这里好像可以换成ubx_decode_state = UBX_DECODE_SYNC1;*/
        }
        break;
    case UBX_DECODE_CLASS:
        /*获取消息类型，CLASS低8位，ID高8位*/
        ubx_msg_type = data;
        /*计算校验和*/
        ubx_calc_ck_byte(data);
        ubx_decode_state = UBX_DECODE_ID;
        break;
    case UBX_DECODE_ID:
        /*获取消息ID*/
        ubx_msg_type = ubx_msg_type | (data << 8);
        /*计算校验和*/
        ubx_calc_ck_byte(data);
        ubx_decode_state = UBX_DECODE_LENGTH1;
        break;
    case UBX_DECODE_LENGTH1:
        ubx_payload_length = data;
        ubx_decode_state = UBX_DECODE_LENGTH2;
        ubx_calc_ck_byte(data);
        break;
    case UBX_DECODE_LENGTH2:
        ubx_payload_length = ubx_payload_length | (data << 8);
        ubx_calc_ck_byte(data);
        if (payload_rx_init() == true)
        {
            ubx_decode_state = (ubx_payload_length == 0 ? UBX_DECODE_CKA : UBX_DECODE_PAYLOAD);
        }
        else
        {
            ubx_decode_init();
        }
        break;
    case UBX_DECODE_PAYLOAD:
        ubx_calc_ck_byte(data);
        switch (ubx_msg_type)
        {
        /*NAV_SVINFO和MON_VER消息需要额外处理，其他消息直接存储在ubx_buff*/
        case UBX_MSG_NAV_SVINFO:
            ret = nav_svinfo_payload_process(data);
            break;
        case UBX_MSG_MON_VER:
            ret = mon_ver_payload_process(data);
            break;
        default:
            ret = payload_rx_process(data);
            break;
        }
        if (ret == 1)
        {
            ubx_decode_state = UBX_DECODE_CKA;
        }
        else if (ret == -1)
        {
            ubx_decode_init();
        }
        ret = 0;
        break;
    case UBX_DECODE_CKA:
        if (ubx_cka != data)
        {
            ubx_decode_init();
        }
        else
        {
            ubx_decode_state = UBX_DECODE_CKB;
        }
        break;
    case UBX_DECODE_CKB:
        if (ubx_ckb != data)
        {
            //	UBX_WARN("ubx checksum err");
        }
        else
        {
            ret = payloadRxDone();
        }
        ubx_decode_init();
        break;
    default:
        break;
    }
    return ret;
}

/**
 * @brief  等待GNSS模块的确认响应
 * @note
 * @param  无
 * @retval 无
 */
uint8_t wait_gnss_ack(uint32_t msg, uint32_t timeout, uint8_t report)
{
    uint8_t ret = 0;
    uint32_t start_time = GetTick();
    uint32_t current_time = GetTick();
    ubx_ack_state = UBX_ACK_WAITING;
    wait_ack_msg = msg;
    while ((ubx_ack_state == UBX_ACK_WAITING) && (current_time - start_time < timeout))
    {
        /*接收并解析UBX信息*/
        read_rb_process();
        current_time = GetTick();
    }
    if (ubx_ack_state == UBX_ACK_GOT_ACK)
    {
        GPS_DEBUG(" ACK IS OK-- -- -- -- -- -- --\r\n ");
        ret = 1;
    }
    /*是否报告错误*/
    else if (report)
    {
        if (ubx_ack_state == UBX_ACK_GOT_NAK)
        {
            GPS_DEBUG("ubx msg 0x%04x NAK", SWAP16((unsigned)msg));
        }
        /*超时*/
        else
        {
            GPS_DEBUG("ubx msg 0x%04x timeout", SWAP16((unsigned)msg));
        }
    }
    return ret;
}

/**
 * @brief  GNSS接收器配置函数
 * @note
 * @param  baudrate:配置的波特率
 * @param  output_mode:输出模式
 * @param  input_mode:输入模式
 * @param  timeout:超时时间
 * @retval 无
 */
int8_t gnss_configure(uint32_t *baudrate, output_mode_t output_mode, input_mode_t input_mode, uint32_t timeout)
{
    gnss_config_flag = 0;
    ubx_cfg_prt_payload_t cfg_prt[2];
    /*尝试不同的波特率*/
    const unsigned baudrates[] = {9600, 38400, 19200, 57600, 115200}; /*疑问:为什么选择这几个*/
    uint32_t start_time = GetTick();
    uint32_t current_time = GetTick();
    uint8_t baud_i = 0;
    uint8_t _output_mode = output_mode == OUTPUT_GPS ? UBX_CFG_PRT_outProtoMask_GPS : UBX_CFG_PRT_outProtoMask_RTCM;
    uint8_t _input_mode = input_mode == INPUT_GPS ? UBX_CFG_PRT_inProtoMask_GPS : UBX_CFG_PRT_inProtoMask_UBX;
    while (!gnss_config_flag)
    {
        current_time = GetTick();
        if (current_time - start_time > timeout)
        {
            break;
        }
        /*尝试不同的波特率*/
        for (baud_i = 0; baud_i < sizeof(baudrates) / sizeof(baudrates[0]); baud_i++)
        {
            *baudrate = baudrates[baud_i];
            /*重置MCU串口波特率*/
            GNSS_ResetBaudRate(*baudrate);
            ubx_decode_init();
            delay_ms(20);
            read_rb_process();
            ubx_decode_init();
            read_rb_process();
            ubx_decode_init();
            /*配置端口(发送CFG_PRT消息)*/
            /*这里并未改变GNSS模块的波特率，只是尝试不同波特率与GNSS模块通信*/
            memset(&cfg_prt, 0, 2 * sizeof(ubx_cfg_prt_payload_t));
            cfg_prt[0].portID = UBX_CFG_PRT_portID_UART1;
            cfg_prt[0].outProtoMask = _output_mode;
            cfg_prt[0].inProtoMask = _input_mode;
            cfg_prt[0].baudRate = *baudrate;
            cfg_prt[0].mode = UBX_CFG_PRT_mode_UART;
            GPS_DEBUG("GPS UBX_MSG_CFG_PRT configure:,%d----\r\n", *baudrate);
            // cfg_prt[1].portID = UBX_TX_CFG_PRT_PORTID_USB;
            // cfg_prt[1].mode = UBX_TX_CFG_PRT_MODE;
            // cfg_prt[1].baudRate = *baudrate;
            // cfg_prt[1].inProtoMask = in_proto_mask;
            // cfg_prt[1].outProtoMask = out_proto_mask;
            /*发送CFG_PRT消息*/
            if (!send_ubx_msg(UBX_MSG_CFG_PRT, (uint8_t*)&cfg_prt, sizeof(ubx_cfg_prt_payload_t)))
            {
                /*尝试下一波特率*/
                continue;
            }
            /*如果未收到应答信号，则尝试下一波特率*/
            if (wait_gnss_ack(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, 0) == 0)
            {
							  GPS_DEBUG("try next baudrate----\r\n");
                continue;
                
            }
            /*接收到ACK消息*/
            else
            {
                //	  GPS_DEBUG("GPS UBX_MSG_CFG_PRT ACK----\r\n");
            }
            /*发送CFG-RATE消息以配置更新率*/
            memset(&ubx_buff.cfg_rate, 0, sizeof(ubx_buff.cfg_rate));
            ubx_buff.cfg_rate.measRate = UBX_TX_CFG_RATE_MEASINTERVAL; /*测量率，单位ms*/
            ubx_buff.cfg_rate.navRate = UBX_TX_CFG_RATE_NAVRATE;       /*导航率，单位ms*/
            ubx_buff.cfg_rate.timeRef = UBX_TX_CFG_RATE_TIMEREF;       /*时间参考*/
            if (!send_ubx_msg(UBX_MSG_CFG_RATE, (uint8_t*)&ubx_buff.cfg_rate, sizeof(ubx_buff.cfg_rate)))
            {
                return -1; /*表示配置失败*/
            }
            /*等待ACK*/
            if (wait_gnss_ack(UBX_MSG_CFG_RATE, UBX_CONFIG_TIMEOUT, 0) == 0)
            {
                return -1;
            }
            else
            {
                //	  GPS_DEBUG("GPS UBX_MSG_CFG_PRT ACK----\r\n");
            }
            /*配置导航滤波(CFG-NVA5)*/
            memset(&ubx_buff.cfg_nav5, 0, sizeof(ubx_payload_tx_cfg_nav5_t));
            ubx_buff.cfg_nav5.mask = UBX_TX_CFG_NAV5_MASK;
            ubx_buff.cfg_nav5.dynModel = output_mode == OUTPUT_GPS ? UBX_TX_CFG_NAV5_DYNMODEL : UBX_TX_CFG_NAV5_DYNMODEL_RTCM;
            ubx_buff.cfg_nav5.fixMode = UBX_TX_CFG_NAV5_FIXMODE;
            if (!send_ubx_msg(UBX_MSG_CFG_NAV5, (uint8_t*)&ubx_buff.cfg_nav5, sizeof(ubx_buff.cfg_nav5)))
            {
                return -1;
            }
            /*等待ACK*/
            if (wait_gnss_ack(UBX_MSG_CFG_NAV5, UBX_CONFIG_TIMEOUT, 0) == 0)
            {
                return -1;
            }
            else
            {
                //	  GPS_DEBUG("GPS UBX_MSG_CFG_NAV5 ACK----\r\n");
            }
            /*配置消息输出速率*/
            if (!set_ubx_msg_rate(UBX_MSG_NAV_PVT, 1))
            {
                return -1;
            }
            if (wait_gnss_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, 1) == 0)
            {
                use_nav_pvt = 0;
            }
            else
            {
                use_nav_pvt = 1;
            }
            if (!use_nav_pvt)
            {
                if (!set_ubx_msg_rate(UBX_MSG_NAV_TIMEUTC, 5))
                {
                    return -1;
                }
                if (wait_gnss_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, 1) == 0)
                {
                    return -1;
                }
                if (!set_ubx_msg_rate(UBX_MSG_NAV_POSLLH, 1))
                {
                    return -1;
                }
                if (wait_gnss_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, 1) == 0)
                {
                    return -1;
                }
                if (!set_ubx_msg_rate(UBX_MSG_NAV_SOL, 1))
                {
                    return -1;
                }
                if (wait_gnss_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, 1) == 0)
                {
                    return -1;
                }
                if (!set_ubx_msg_rate(UBX_MSG_NAV_VELNED, 1))
                {
                    return -1;
                }
                if (wait_gnss_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, 1) == 0)
                {
                    return -1;
                }
            }
            if (!set_ubx_msg_rate(UBX_MSG_NAV_DOP, 1))
            {
                return -1;
            }
            if (wait_gnss_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, 1) == 0)
            {
                return -1;
            }
            gnss_config_flag = 1;
            GPS_DEBUG("GPS CONFIGURE COMPLETED----\r\n");
		    break;
        }
    }
    return 0;
}
