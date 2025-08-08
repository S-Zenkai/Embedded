/**
 ******************************************************************************
 * @file    task_datalog.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/08/02
 * @brief   ���ݼ�¼����,���ļ���ʵ���˵����ݵļ�¼�������ݵ��ļ���¼��Ҫ����һ�����ݰ��ṹ��
 *          ����ʵ�����ݰ���д��Ͷ�ȡ��������Ҫ��task_datalog.c��ʵ�ֶ����ݵ��ļ���¼
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "task_datalog1.h"
#include "ff.h"
#include "ringbuff.h"
#include "bsp_systick.h"

FATFS fs;       /* FatFs�ļ�ϵͳ���� */
FIL fnew;       /* �ļ����� */
FRESULT res_sd; /* �ļ�������� */
UINT fnum;      /* �ļ��ɹ���д���� */
/*���λ������������*/
ringbuff_t SD_W_RingBuffMgr;
/*���λ�������С*/
#define SD_W_BUFFER_SIZE 1024 * 10
/*���λ���������*/
uint8_t SD_W_Buff[SD_W_BUFFER_SIZE];
/*�ļ��򿪱�־*/
FlagStatus file_open_flag = RESET;
/*���廷�λ������洢һ���������ݺ���д�뵽�ļ��У��Ƽ�ΪSD��������С�������������д��Ч��*/
#define SD_W_DATA_MIN_SIZE 512

/* ���λ�������ʼ�� */
static void SD_W_RB_Init(void)
{
    rb_init(SD_W_Buff, SD_W_BUFFER_SIZE, &SD_W_RingBuffMgr);
}

/**
 * @brief  ���λ���������ж�
 * @note
 * @param  ��
 * @retval ���ΪSET����֮ΪRESET
 */
static FlagStatus SD_W_RB_IsOverFlow(void)
{
    return SD_W_RingBuffMgr.overflow;
}

/**
 * @brief  ��ȡ��ǰ������ʣ����������
 * @note
 * @param  ��
 * @retval ��
 */
static uint32_t SD_W_RB_GetCounter(void)
{
    return rb_GetDataCounter(&SD_W_RingBuffMgr);
}

/**
 * @brief  ���λ�����ѹ��һ���ֽ�
 * @note
 * @param  ��
 * @retval ��
 */
static bool SD_W_RB_Pop(uint8_t *value)
{
    return rb_pop(&SD_W_RingBuffMgr, value);
}

/**
 * @brief  ���ݼ�¼�����ʼ��
 * @note
 * @param  ��
 * @retval ��
 */
void task_datalog_init(void)
{
    SD_W_RB_Init();
    // ���ⲿSPI Flash�����ļ�ϵͳ���ļ�ϵͳ����ʱ����豸��ʼ��
    res_sd = f_mount(&fs, "0:", 1);
    /* ���û���ļ�ϵͳ�͸�ʽ�����������ļ�ϵͳ */
    if (res_sd == FR_NO_FILESYSTEM)
    {
        TASK_DATALOG1_DEBUG("��SD����û���ļ�ϵͳ���������и�ʽ��...\r\n");
        /* ��ʽ�� */
        /*����SD_W_Buff�������ڸ�f_mkfs�ṩ��������Ҫ���С��С��FF_MAX_SS(ϵͳ֧�ֵ����������С)*/
        res_sd = f_mkfs("0:", 0, SD_W_Buff, sizeof(SD_W_Buff));
        if (res_sd == FR_OK)
        {
            TASK_DATALOG1_DEBUG("��SD���ѳɹ���ʽ���ļ�ϵͳ��\r\n");
            /* ��ʽ������ȡ������ */
            res_sd = f_mount(NULL, "0:", 1);
            /* ���¹���	*/
            res_sd = f_mount(&fs, "0:", 1);
        }
        else
        {
            TASK_DATALOG1_DEBUG("������ʽ��ʧ�ܡ�����\r\n");
        }
    }
    else if (res_sd != FR_OK)
    {
        TASK_DATALOG1_DEBUG("��������ԭ��SD����ʼ�����ɹ���\r\n");
        //        TASK_DATALOG1_DEBUG("!!SD�������ļ�ϵͳʧ��.(%d)\r\n", res_sd);
        TASK_DATALOG1_DEBUG("SD�������ļ�ϵͳʧ��(.%d)\r\n", res_sd);
        TASK_DATALOG1_DEBUG("SD�������ļ�ϵͳʧ�ܣ�.%d��\r\n", res_sd);
    }
    else
    {
        // TASK_DATALOG1_DEBUG("���ļ�ϵͳ���سɹ������Խ��ж�д����\r\n");
    }
    /* ���ļ�������ļ��������򴴽�����������ڣ�����ļ�ĩβ��ʼд�� */
    res_sd = f_open(&fnew, "0:datalog.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res_sd == FR_OK)
    {
        file_open_flag = SET;
    }
    else
    {
        file_open_flag = RESET;
        TASK_DATALOG1_DEBUG("�������ļ�ʧ�ܡ�\r\n");
    }
}

/**
 * @brief  ���ݼ�¼����
 * @note
 * @param  ��
 * @retval ��
 */
void task_datalog(void)
{
    static uint8_t temp_buff[SD_W_DATA_MIN_SIZE];/*�������ϴ󣬿�����ɶ�ջ�������Ӳ������*/
    if (GetTick()>=50000&&file_open_flag == SET)
    {
        f_close(&fnew);
        file_open_flag = RESET;
        TASK_DATALOG1_DEBUG("���ļ��ر�\r\n");
        return;
    }
    if (file_open_flag == RESET)
    {
        return;
    }
    if (SD_W_RB_IsOverFlow() == SET)
    {
        rb_clear(&SD_W_RingBuffMgr);
        TASK_DATALOG1_DEBUG("�����λ������������ջ��λ�����\r\n");
        return;
    }
    uint32_t data_counter = SD_W_RB_GetCounter();
    if (data_counter < SD_W_DATA_MIN_SIZE)
    {
        return;
    }
    /*�����ٽ���*/
    // __disable_irq();
    for (uint32_t i = 0; i < SD_W_DATA_MIN_SIZE; i++)
    {
        SD_W_RB_Pop(&temp_buff[i]);
    }
    /*�˳��ٽ���*/
    // __enable_irq();
    // res_sd = f_open(&fnew, "0:datalog.txt", FA_OPEN_APPEND | FA_WRITE);
    res_sd = f_write(&fnew, temp_buff, SD_W_DATA_MIN_SIZE, &fnum);
    if (res_sd != FR_OK)
    {
        //        TASK_DATALOG1_DEBUG("�����ļ�д��ʧ�ܣ�(%d)\r\n", res_sd);
        TASK_DATALOG1_DEBUG("�����ļ�д��ʧ�ܣ���%d��\r\n", res_sd);
    }
    // f_close(&fnew);
}
