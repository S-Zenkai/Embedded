/**
 ******************************************************************************
 * @file    ubx.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/04/09
 * @brief   sbusͨ��Э�����
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

/*GNSS���������ñ�־*/
uint8_t gnss_config_flag = 0;
/*GNSS�Ƿ�֧��ʹ��NAV-PVT��Ϣ*/
uint8_t use_nav_pvt = 0;

/*UBXЭ�����״̬*/
ubx_decode_state_t ubx_decode_state = UBX_DECODE_SYNC1;
/*������Ϣ״̬*/
ubx_rx_msg_state_t ubx_rx_msg_state = UBX_RX_MSG_HANDLE;
/*UBXȷ��״̬*/
ubx_ack_state_t ubx_ack_state = UBX_ACK_IDLE;

/*UBXЭ��У���A*/
uint8_t ubx_cka = 0;
/*UBXЭ��У���B*/
uint8_t ubx_ckb = 0;
/*UBX��Ϣ����*/
uint16_t ubx_msg_type = 0;

/*UBXЭ����Ч�غɳ���*/
uint16_t ubx_payload_length = 0;
/*UBXЭ����Ч�غɽṹ������*/
uint8_t ubx_payload_index = 0;

/*RTCM��Ϣ�ṹ��*/
rtcm_t *rtcm_msg = NULL;

/*������Ϣ*/
svinfo_t *svinfo = NULL;

/*GSPģ�������Ķ�λ���ٶȡ�ʱ���״̬��Ϣ�ṹ��*/
struct vehicle_gps_info_s *vehicle_gps_info = NULL;

/*UBX��Ϣ������*/
ubx_buff_t ubx_buff;

/*��һ�η��ͽ�����Ϣ�����ʱ��*/
uint32_t last_disable_msg_time = 0;

/*UBX�汾��*/
uint32_t ubx_version;

/*���ʣ����������ʲô��*/
uint64_t _last_timestamp_time;

/*�ٶȺ�λ�����ݽ��ռ�����*/
uint8_t _rate_count_lat_lon;
uint8_t _rate_count_vel;
/*λ�����ݽ��ձ�־*/
uint8_t _got_posllh;
/*�ٶ����ݽ��ձ�־*/
uint8_t _got_velned;

/*�ȴ�Ӧ�����Ϣ*/
uint16_t wait_ack_msg;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
uint8_t ParseData(uint8_t data);
void ubx_calc_ck(const uint8_t *data, uint16_t len, ubx_ck_t *ck);

/**
 * @brief  ��GNSS������������Ϣ
 * @note
 * @param  buff:����ָ��
 * @param  len:���ݳ���
 * @retval ���ͳɹ�����true�����򷵻�false
 */
bool send_msg(const void *buff, uint16_t len)
{
    return USART_SendBytes(UART4, (uint8_t *)buff, len, 1000);
}

/**
 * @brief  ��GNSS��������������UBX��Ϣ
 * @note
 * @param  ��
 * @retval ��
 */
bool send_ubx_msg(uint16_t msg_type, const uint8_t *payload, uint16_t payload_len)
{
    ubx_ck_t ck =  {0, 0};
    // uint16_t sync = UBX_SYNC1 | ((UBX_SYNC2 << 8) & 0xFF00);
    uint8_t sync[] = {UBX_SYNC1, UBX_SYNC2}; // 0xB5, 0x62
    /*����У���*/
    /*����*/
    ubx_calc_ck((uint8_t *)&msg_type, sizeof(msg_type), &ck); /*stm32����С����*/
    /*����*/
    ubx_calc_ck((uint8_t *)&payload_len, sizeof(payload_len), &ck);
    if (payload != NULL)
    {
        ubx_calc_ck(payload, payload_len, &ck);
    }
    /*������Ϣ*/
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
 * @brief  ��GNSSģ��Ļ��λ�������ȡ���ݣ����洢��ubx_buff��
 * @note
 * @param  ��
 * @retval ��
 */
uint8_t read_rb(uint8_t *buff, uint16_t len, uint32_t timeout)
{
    uint64_t t_start = GetTick();
    uint16_t i = 0;
    while (1)
    {
        /*��黺�����Ƿ����*/
        if (GNSS_RB_CheckOverflow() == 1)
        {
            GPS_DEBUG("GNSS RB overflow\r\n");
            GNSS_RB_Clear();
            return 0;
        }
        /*��黺�������Ƿ����������*/
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
 * @brief  �ӻ��λ�������ȡ���ݣ���Ԥ��������
 * @note
 * @param  ��
 * @retval ��
 */
uint8_t read_rb_process(void)
{
    uint8_t ret;
    /*��黺�����Ƿ����*/
    if (GNSS_RB_CheckOverflow() == 1)
    {
        GPS_DEBUG("GNSS RB overflow\r\n");
        GNSS_RB_Clear();
        return 0;
    }
    /*��黺�������Ƿ����������*/
    while (!GNSS_RB_IsEmpty())
    {
        uint8_t u = GNSS_RB_PopData();
        /*���ʣ�����Ϊʲôʹ�û������*/
        ret |= ParseData(u);
    }
    return ret;
}

/**
 * @brief  UBXЭ������ʼ��
 * @note
 * @param  ��
 * @retval ��
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
 * @brief  ����У��ͣ����ֽڣ�
 * @note
 * @param  data:����
 * @retval ��
 */
void ubx_calc_ck_byte(uint8_t data)
{
    ubx_cka += data;
    ubx_ckb += ubx_cka;
}

/**
 * @brief  ����У��ͣ����ֽڣ�
 * @note
 * @param  ��
 * @retval ��
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
 * @brief  ����u-bloxģ����ָ����Ϣ���������
 * @note
 * @param  ��
 * @retval ��
 */
bool set_ubx_msg_rate(uint16_t msg_type, uint16_t rate)
{
    ubx_tx_cfg_msg_payload_t cfg_msg_payload; /*���룺ԭ�����н������ﲻʹ��ubx_buff��Ϊ��֧�ֲ������жϲ���*/
    cfg_msg_payload.msg = msg_type;
    cfg_msg_payload.rate = rate;
    return send_ubx_msg(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg_payload, sizeof(cfg_msg_payload));
}

/**
 * @brief  ��Ч�غɽ��ճ�ʼ������֤��Ϣ���ͺ���Ч�غɳ��ȡ������ʵ��Ľ���״̬����׼����Ч�غɴ�������ʼ��UBX��Ϣ����Ч�غɽ���
 *         ���������UBX��Ϣ״̬������UBX��Ϣ������ɺ�����״̬���ж��Ƿ�����յ���Ϣ����Ч�غ�(payloadRxDone)
 * @note
 * @param  ��
 * @retval ����true��ʾ��Ϣ����ɹ�������false��ʾ��Ϣ����ʧ��
 */
bool payload_rx_init(void)
{
//    uint8_t ret;
    /*Ĭ��״̬*/
    ubx_rx_msg_state = UBX_RX_MSG_HANDLE;
    switch (ubx_msg_type)
    {
    case UBX_MSG_NAV_PVT:
        /*�����Ч�غɳ��Ȳ�����Э���׼��������Ϊ����״̬*/
        if (ubx_payload_length != NAV_PVT_PAYLOAD_LENGTH_UBX8 && ubx_payload_length != NAV_PVT_PAYLOAD_LENGTH_UBX7)
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        /*�����ʼ��ʱδ������Ϣ��������Ϊ����״̬*/
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*���GNSS��֧��ʹ��NAV-PVT��Ϣ��������Ϊ����״̬*/
        else if (!use_nav_pvt)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        break;
    case UBX_MSG_INF_DEBUG:
    case UBX_MSG_INF_ERROR:
    case UBX_MSG_INF_NOTICE:
    case UBX_MSG_INF_WARNING:
        /*��������״̬��ζ�����INF��Ϣ*/
        /*INF��Ϣ���Ǳ��ֳ��ģ��������Ч�غɳ��ȣ�ֻ�Ƿ�ֹUBX����buff���(������������ubx_buff_t[ubx_payload_length]���0������������Ǳ����0ʱ���)*/
        if (ubx_payload_length >= sizeof(ubx_buff_t))
        {
            /*�������UBX��Ϣ��Ч�غɻ�����*/
            ubx_payload_length = sizeof(ubx_buff_t) - 1;
        }
        break;
    /*��������NAV-POSLLH��NAV-SOL��NAV-TIMEUTC��NAV-VELNED��Ϣ�����ù�������*/
    /*NAV-POSLLH��Ϣ,�������λ�ã���γ�ȡ��߶ȣ�*/
    case UBX_MSG_NAV_POSLLH:
        if (ubx_payload_length != sizeof(ubx_nav_posllh_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*���GNSS֧��ʹ��NAV-PVT��Ϣ�������NAV-POSLLH��Ϣ������ʹ��NAV-PVT��Ϣ*/
        else if (use_nav_pvt)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        break;
    /*�����⣨λ�á��ٶȡ�״̬��*/
    case UBX_MSG_NAV_SOL:
        if (ubx_payload_length != sizeof(ubx_nav_sol_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*���GNSS֧��ʹ��NAV-PVT��Ϣ�������NAV-SOL��Ϣ������ʹ��NAV-PVT��Ϣ*/
        else if (use_nav_pvt)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        break;
    /*UTCʱ���*/
    case UBX_MSG_NAV_TIMEUTC:
        if (ubx_payload_length != sizeof(ubx_nav_timeutc_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*���GNSS֧��ʹ��NAV-PVT��Ϣ�������NAV-TIMEUTC��Ϣ������ʹ��NAV-PVT��Ϣ*/
        else if (use_nav_pvt)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        break;
    /*NED�������أ�����ϵ�е��ٶ�*/
    case UBX_MSG_NAV_VELNED:
        if (ubx_payload_length != sizeof(ubx_nav_velned_payload_t))
        {
            ubx_rx_msg_state = UBX_RX_MSG_ERROR_LENGTH;
        }
        else if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*���GNSS֧��ʹ��NAV-PVT��Ϣ�������NAV-VELNED��Ϣ������ʹ��NAV-PVT��Ϣ*/
        else if (use_nav_pvt)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        break;
    /*DOP���������ӣ�,��֤����������״̬*/
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
    /*NAV-SVINFO��������Ϣ�������������ź�ǿ�ȣ�*/
    case UBX_MSG_NAV_SVINFO:
        /*���ʣ�����δУ����Ч�غɳ��ȣ���������ΪSVINFO��Ϣ�Ǳ��ֳ��ģ���Ч�غ��а���һ����̬�ظ���������Ϣ�飬�������ֶ�numCh��������Ҳ�������Ƴٵ�������������У��*/
        if (!gnss_config_flag)
        {
            ubx_rx_msg_state = UBX_RX_MSG_IGNORE;
        }
        /*���δ����������Ϣ����*/
        else if (svinfo == NULL)
        {
            ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        }
        else
        {
            /*memset���ֶ�ѭ��Ҫ�죬��Ϊ��ͨ��ʹ�õײ�Ӳ��ָ���Ż�*/
            memset(svinfo, 0, sizeof(svinfo_t));
        }
        break;
    /*����״̬������RTK��վ���ã���֧��RTKӦ�ã���������ɺ����������*/
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
    /*ģ��汾��Ϣ��ʼ�մ���汾��Ϣ����Ϊ���ʶ��ģ�鹦��������Ҫ*/
    case UBX_MSG_MON_VER:
        break;
    /*HON-HW��Ӳ��״̬����������״̬������ˮƽ�����ڼ��Ӳ��״̬���������*/
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
    /*ACK-ACK��ȷ����Ϣ������ȷ����Ϣ���ͳɹ�*/
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
    /*ACK-NAK������ʧ��*/
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
    /*Ĭ�ϣ���������UBX��Ϣ���ͣ�����Ϊ����״̬*/
    default:
        ubx_rx_msg_state = UBX_RX_MSG_DISABLE;
        break;
    }
    /*״̬�����뷵��ֵ*/
    switch (ubx_rx_msg_state)
    {
    /*��Ϣδ����ʱҲ�����������*/
    case UBX_RX_MSG_HANDLE:
    case UBX_RX_MSG_IGNORE:
        return true;
    case UBX_RX_MSG_DISABLE:
        /*һ��ʱ��������һ�ν�����Ϣ���ã�����Ƶ�����ͽ�������*/
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
 * @brief  �����յ�����Ч�غɷ��뻺����(��NAV-SVINFO)
 * @note
 * @param  data:���յ�������(�ֽ�)
 * @retval ����1��ʾ������Ч�غɽ�����ɣ�����0��ʾ��Ч�غɽ���δ��ɣ�����-1��ʾ����
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
 * @brief  NAV-SVINFO��Ϣ����
 * @note
 * @param  ��
 * @retval ��
 */
int8_t nav_svinfo_payload_process(uint8_t data)
{
    int8_t ret = -1;
    uint8_t *p_buff = (uint8_t *)&ubx_buff;
    /*����ǰ�벿�֣�ֱ�ӷ���buff������*/
    if (ubx_payload_index < sizeof(ubx_nav_svinfo_payload_part1_t))
    {
        p_buff[ubx_payload_index] = data;
    }
    /*�ڽ�����part1����ʼ����part1������part2�����������part2ʱ���ɸ���part1*/
    else
    {
        /*����part1*/
        if (ubx_payload_index == sizeof(ubx_nav_svinfo_payload_part1_t))
        {
            /*numCh�м�¼�˽��ջ�Ӳ��֧�ֻ�ǰ����ͨ�������������ظ�����������numch*/
            svinfo->count = (ubx_buff.nav_svinfo_part1.numCh < MAX_SUPPORTED_CHANNELS ? ubx_buff.nav_svinfo_part1.numCh : MAX_SUPPORTED_CHANNELS);
        }
        /*����part2*/
        /*���ubx_payload_index��nav_svinfo��Ч�غ��е�λ��*/
        if (ubx_payload_index < sizeof(ubx_nav_svinfo_payload_part1_t) + svinfo->count * sizeof(ubx_nav_svinfo_payload_part2_t))
        {
            uint8_t index = (ubx_payload_index - sizeof(ubx_nav_svinfo_payload_part1_t)) % sizeof(ubx_nav_svinfo_payload_part2_t);
            p_buff[index] = data;
            /*���յ�һ���������ݣ���ʼ����*/
            if (index == sizeof(ubx_nav_svinfo_payload_part2_t) - 1)
            {
                /*���ڼ�������*/
                uint8_t sv_index = (ubx_payload_index - sizeof(ubx_nav_svinfo_payload_part1_t)) / sizeof(ubx_nav_svinfo_payload_part2_t);
                /*��ȡflags�����λ(svUsed:1��ʾ�����źű����ڵ�������(����λ�á��ٶȡ�ʱ��)),*/
                svinfo->used[sv_index] = (uint8_t)(ubx_buff.nav_svinfo_part2.flags & 0x01);
                svinfo->snr[sv_index] = ubx_buff.nav_svinfo_part2.cno;                                            /*�洢�����*/
                svinfo->elevation[sv_index] = ubx_buff.nav_svinfo_part2.elev;                                     /*�洢����*/
                svinfo->azimuth[sv_index] = (uint8_t)((float)(ubx_buff.nav_svinfo_part2.azim) * 255.0f / 360.0f); /*�洢��λ��,ת��Ϊ0-255�������ǽ�ʡ�ڴ�*/
                svinfo->svid[sv_index] = ubx_buff.nav_svinfo_part2.svid;                                          /*�洢����ID*/
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
 * @brief  FNV-1(Fowler-Noll-Vo)��ϣ�㷨(32λ�汾)
 * @note
 * @param  str:�ַ���
 * @param  hval:��ʼ��ϣֵ�����Դ�����һ�εĹ�ϣ��������ʵ����ʽ��ϣ���Ӷ��õ��������Ƭ�����������������ϣֵ
 * @retval ���ع�ϣֵ
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
 * @brief  MON-VER��Ϣ����
 * @note
 * @param  ��
 * @retval ��
 */
int8_t mon_ver_payload_process(uint8_t data)
{
    int8_t ret = -1;
    uint8_t *p_buff = (uint8_t *)&ubx_buff;
    /*����ǰ�벿�֣�ֱ�ӷ���buff������*/
    if (ubx_payload_index < sizeof(ubx_payload_rx_mon_ver_part1_t))
    {
        p_buff[ubx_payload_index] = data;
    }
    /*�����벿�֣�ֱ�ӷ���buff������*/
    else
    {
        /*����part1*/
        if (ubx_payload_index == sizeof(ubx_payload_rx_mon_ver_part1_t))
        {
            /*���̻��汾��Ӳ���汾ѹ��Ϊ32λ��ϣֵ*/
            ubx_version = fnv1_hash(ubx_buff.mon_ver_part1.swVersion, FNV1_32_INIT);
            ubx_version = fnv1_hash(ubx_buff.mon_ver_part1.hwVersion, ubx_version);
        }
        uint8_t index = (ubx_payload_index - sizeof(ubx_payload_rx_mon_ver_part1_t)) % sizeof(ubx_payload_rx_mon_ver_part2_t);
        p_buff[index] = data;
        /*����part2*/
        if (index == sizeof(ubx_payload_rx_mon_ver_part2_t) - 1)
        {
            /*��ӡpart2����*/
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
 * @brief  �����ѽ��ղ�У��ͨ����UBX��Ϣ��Ч�غ�(�ڽ�����һ֡���ݺ����)
 * @note
 * @param  ��
 * @retval ��
 */
int payloadRxDone(void)
{
    uint8_t ret = 0;
    /*��֤��Ϣ״̬������ʹ����UBX״̬��*/
    if (ubx_rx_msg_state != UBX_RX_MSG_HANDLE)
    {
        return ret;
    }
    switch (ubx_msg_type)
    {
    case UBX_MSG_NAV_PVT:
        /*��֤��λ���Ƿ���Ч*/
        if (ubx_buff.nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK == 1)
        {
            /*����fix_typeΪfixType(0:��Ч,1:2D,2:3D)*/
            vehicle_gps_info->fix_type = ubx_buff.nav_pvt.fixType;
            if (ubx_buff.nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_DIFFSOLN)
            {
                vehicle_gps_info->fix_type = 4; /*���GPS(DGPS)*/
            }
            /*�ز���λ��״̬*/
            uint8_t carr_soln = ubx_buff.nav_pvt.flags >> 6;
            if (carr_soln == 1)
            {
                vehicle_gps_info->fix_type = 5; /*Float RTK*/
            }
            else if (carr_soln == 2)
            {
                vehicle_gps_info->fix_type = 6; /*Fix RTK*/
            }
            /*�ٶ���Ϣ��Ч*/
            vehicle_gps_info->vel_ned_valid = 1;
        }
        else
        {
            vehicle_gps_info->fix_type = 0;
            vehicle_gps_info->vel_ned_valid = 0;
        }
        /*��NAV_PVT�еĸ����ֶ���ȡ���������б�Ҫ��ת��*/
        vehicle_gps_info->satellites_used = ubx_buff.nav_pvt.numSV;
        vehicle_gps_info->lat = ubx_buff.nav_pvt.lat;
        vehicle_gps_info->lon = ubx_buff.nav_pvt.lon;
        vehicle_gps_info->alt = ubx_buff.nav_pvt.hMSL;
        /*�Ӻ���תΪ�ף�1e-3f=0.001f*/
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
        /*ʱ�����������Ч�Լ��(ʱ�����ڶ���Ч����ȫ����)*/
        /*���ʣ��������ûʲô��*/
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
            vehicle_gps_info->time_utc_usec = 0; /*����δʵ��ת��Unixʱ���*/
        }
        /*ϵͳʱ���*/
        vehicle_gps_info->timestamp = GetTick();
        _last_timestamp_time = vehicle_gps_info->timestamp;
        /*�����ٶȺ�λ�����ݵĽ��ռ���*/
        _rate_count_lat_lon++;
        _rate_count_vel++;
        /*��־���յ���Ч��λ�ú��ٶ���Ϣ*/
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
        /*��ubx_buffĩβ���0����ʾ�ַ�������*/
        p_buff[ubx_payload_length] = 0;
        /*����δ���÷���ֵ����û������*/
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
        /*���ʣ����ﶨ����һ����ȫ�ֱ�����ͬ��_rate_count_lat_lon��������Щbug*/
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
        /*�ṩ��λ����ָ�꣬�������ηֲ�*/
        vehicle_gps_info->hdop = ubx_buff.nav_dop.hDOP * 1e-2f;
        vehicle_gps_info->vdop = ubx_buff.nav_dop.vDOP * 1e-2f;
        GPS_DEBUG("hdop:%f ,vdop:%f\r\n", vehicle_gps_info->hdop, vehicle_gps_info->vdop);
        ret = 1;
        break;
    case UBX_MSG_NAV_TIMEUTC:
        /*���ʱ����Ч��*/
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
            vehicle_gps_info->time_utc_usec = 0; /*����δʵ��ת��Unixʱ���*/
        }
        _last_timestamp_time = GetTick();
        ret = 1;
        break;
    case UBX_MSG_NAV_SVINFO:
        /*NAV_SVINFO��Ϣ֮ǰ�Ѵ����������һ��ʱ���*/
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
    /*����:�������ûʲô��*/
    if (ret > 0)
    {
        vehicle_gps_info->timestamp_time_relative = (int32_t)(vehicle_gps_info->timestamp - _last_timestamp_time);
    }
    return ret;
}

/**
 * @brief  ����UBX���ݡ�RTCM����
 * @note
 * @param  data:���յ�������
 * @retval ����1��2��ʾһ��������֡�����ɹ�������0��ʾ����ʧ�ܻ�δ���
 */
uint8_t ParseData(uint8_t data)
{
    int8_t ret;
    switch (ubx_decode_state)
    {
    /*dataΪUBXͬ���ֻ�RTCMͬ���֣���֧��RTCM��*/
    case UBX_DECODE_SYNC1:
        if (data == UBX_SYNC1)
        {
            ubx_decode_state = UBX_DECODE_SYNC2;
        }
        else if (data == RTCM_SYNC1 && rtcm_msg != NULL)
        {
            ubx_decode_state = RTCM_DECODE_SYNC1;
            rtcm_msg->buffer[rtcm_msg->buffer_index++] = data;
            /*���ʣ�����Ҫ��*/
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
            ubx_decode_init(); /*���ʣ����������Ի���ubx_decode_state = UBX_DECODE_SYNC1;*/
        }
        break;
    case UBX_DECODE_CLASS:
        /*��ȡ��Ϣ���ͣ�CLASS��8λ��ID��8λ*/
        ubx_msg_type = data;
        /*����У���*/
        ubx_calc_ck_byte(data);
        ubx_decode_state = UBX_DECODE_ID;
        break;
    case UBX_DECODE_ID:
        /*��ȡ��ϢID*/
        ubx_msg_type = ubx_msg_type | (data << 8);
        /*����У���*/
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
        /*NAV_SVINFO��MON_VER��Ϣ��Ҫ���⴦��������Ϣֱ�Ӵ洢��ubx_buff*/
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
 * @brief  �ȴ�GNSSģ���ȷ����Ӧ
 * @note
 * @param  ��
 * @retval ��
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
        /*���ղ�����UBX��Ϣ*/
        read_rb_process();
        current_time = GetTick();
    }
    if (ubx_ack_state == UBX_ACK_GOT_ACK)
    {
        GPS_DEBUG(" ACK IS OK-- -- -- -- -- -- --\r\n ");
        ret = 1;
    }
    /*�Ƿ񱨸����*/
    else if (report)
    {
        if (ubx_ack_state == UBX_ACK_GOT_NAK)
        {
            GPS_DEBUG("ubx msg 0x%04x NAK", SWAP16((unsigned)msg));
        }
        /*��ʱ*/
        else
        {
            GPS_DEBUG("ubx msg 0x%04x timeout", SWAP16((unsigned)msg));
        }
    }
    return ret;
}

/**
 * @brief  GNSS���������ú���
 * @note
 * @param  baudrate:���õĲ�����
 * @param  output_mode:���ģʽ
 * @param  input_mode:����ģʽ
 * @param  timeout:��ʱʱ��
 * @retval ��
 */
int8_t gnss_configure(uint32_t *baudrate, output_mode_t output_mode, input_mode_t input_mode, uint32_t timeout)
{
    gnss_config_flag = 0;
    ubx_cfg_prt_payload_t cfg_prt[2];
    /*���Բ�ͬ�Ĳ�����*/
    const unsigned baudrates[] = {9600, 38400, 19200, 57600, 115200}; /*����:Ϊʲôѡ���⼸��*/
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
        /*���Բ�ͬ�Ĳ�����*/
        for (baud_i = 0; baud_i < sizeof(baudrates) / sizeof(baudrates[0]); baud_i++)
        {
            *baudrate = baudrates[baud_i];
            /*����MCU���ڲ�����*/
            GNSS_ResetBaudRate(*baudrate);
            ubx_decode_init();
            delay_ms(20);
            read_rb_process();
            ubx_decode_init();
            read_rb_process();
            ubx_decode_init();
            /*���ö˿�(����CFG_PRT��Ϣ)*/
            /*���ﲢδ�ı�GNSSģ��Ĳ����ʣ�ֻ�ǳ��Բ�ͬ��������GNSSģ��ͨ��*/
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
            /*����CFG_PRT��Ϣ*/
            if (!send_ubx_msg(UBX_MSG_CFG_PRT, (uint8_t*)&cfg_prt, sizeof(ubx_cfg_prt_payload_t)))
            {
                /*������һ������*/
                continue;
            }
            /*���δ�յ�Ӧ���źţ�������һ������*/
            if (wait_gnss_ack(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, 0) == 0)
            {
							  GPS_DEBUG("try next baudrate----\r\n");
                continue;
                
            }
            /*���յ�ACK��Ϣ*/
            else
            {
                //	  GPS_DEBUG("GPS UBX_MSG_CFG_PRT ACK----\r\n");
            }
            /*����CFG-RATE��Ϣ�����ø�����*/
            memset(&ubx_buff.cfg_rate, 0, sizeof(ubx_buff.cfg_rate));
            ubx_buff.cfg_rate.measRate = UBX_TX_CFG_RATE_MEASINTERVAL; /*�����ʣ���λms*/
            ubx_buff.cfg_rate.navRate = UBX_TX_CFG_RATE_NAVRATE;       /*�����ʣ���λms*/
            ubx_buff.cfg_rate.timeRef = UBX_TX_CFG_RATE_TIMEREF;       /*ʱ��ο�*/
            if (!send_ubx_msg(UBX_MSG_CFG_RATE, (uint8_t*)&ubx_buff.cfg_rate, sizeof(ubx_buff.cfg_rate)))
            {
                return -1; /*��ʾ����ʧ��*/
            }
            /*�ȴ�ACK*/
            if (wait_gnss_ack(UBX_MSG_CFG_RATE, UBX_CONFIG_TIMEOUT, 0) == 0)
            {
                return -1;
            }
            else
            {
                //	  GPS_DEBUG("GPS UBX_MSG_CFG_PRT ACK----\r\n");
            }
            /*���õ����˲�(CFG-NVA5)*/
            memset(&ubx_buff.cfg_nav5, 0, sizeof(ubx_payload_tx_cfg_nav5_t));
            ubx_buff.cfg_nav5.mask = UBX_TX_CFG_NAV5_MASK;
            ubx_buff.cfg_nav5.dynModel = output_mode == OUTPUT_GPS ? UBX_TX_CFG_NAV5_DYNMODEL : UBX_TX_CFG_NAV5_DYNMODEL_RTCM;
            ubx_buff.cfg_nav5.fixMode = UBX_TX_CFG_NAV5_FIXMODE;
            if (!send_ubx_msg(UBX_MSG_CFG_NAV5, (uint8_t*)&ubx_buff.cfg_nav5, sizeof(ubx_buff.cfg_nav5)))
            {
                return -1;
            }
            /*�ȴ�ACK*/
            if (wait_gnss_ack(UBX_MSG_CFG_NAV5, UBX_CONFIG_TIMEOUT, 0) == 0)
            {
                return -1;
            }
            else
            {
                //	  GPS_DEBUG("GPS UBX_MSG_CFG_NAV5 ACK----\r\n");
            }
            /*������Ϣ�������*/
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
