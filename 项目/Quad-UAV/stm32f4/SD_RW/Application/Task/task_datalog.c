/**
 ******************************************************************************
 * @file    task_datalog.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/08/02
 * @brief   ���ݼ�¼����
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "task_datalog.h"
#include "ff.h"
#include "ringbuff.h"

FATFS fs;                     /* FatFs�ļ�ϵͳ���� */
FIL fnew;                     /* �ļ����� */
FRESULT res_sd;               /* �ļ�������� */
UINT fnum;                    /* �ļ��ɹ���д���� */
BYTE ReadBuffer[1024] = {0};  /* �������� */
BYTE WriteBuffer[1024] = {0}; /* д������*/
/*���λ������������*/
ringbuff_t SD_W_RingBuffMgr;
/*���λ�������С*/
#define SD_W_BUFFER_SIZE 1024
/*���λ���������*/
uint8_t SD_W_Buff[SD_W_BUFFER_SIZE];
/*�ļ��򿪱�־*/
FlagStatus file_open_flag = RESET;

/* ���λ�������ʼ�� */
static void SD_W_RingBuff_Init(void)
{
    rb_init(SD_W_Buff, SD_W_BUFFER_SIZE, &SD_W_RingBuffMgr);
}

/**
 * @brief  ���λ���������ж�
 * @note
 * @param  ��
 * @retval ���ΪSET����֮ΪRESET
 */
static FlagStatus SD_W_RingBuff_IsOverFlow(void)
{
    return SD_W_RingBuffMgr.overflow;
}

/**
 * @brief  ���ݼ�¼�����ʼ��
 * @note
 * @param  ��
 * @retval ��
 */
void task_datalog_init(void)
{
    SD_W_RingBuff_Init();
    // ���ⲿSPI Flash�����ļ�ϵͳ���ļ�ϵͳ����ʱ����豸��ʼ��
    res_sd = f_mount(&fs, "0:", 1);
    /* ���û���ļ�ϵͳ�͸�ʽ�����������ļ�ϵͳ */
    if (res_sd == FR_NO_FILESYSTEM)
    {
        //printf("��SD����û���ļ�ϵͳ���������и�ʽ��...\r\n");
        /* ��ʽ�� */
        /*����SD_W_Buff�������ڸ�f_mkfs�ṩ��������Ҫ���С��С��FF_MAX_SS(ϵͳ֧�ֵ����������С)*/
        res_sd = f_mkfs("0:", 0, SD_W_Buff, sizeof(ReadBuffer));
        if (res_sd == FR_OK)
        {
            // printf("��SD���ѳɹ���ʽ���ļ�ϵͳ��\r\n");
            /* ��ʽ������ȡ������ */
            res_sd = f_mount(NULL, "0:", 1);
            /* ���¹���	*/
            res_sd = f_mount(&fs, "0:", 1);
        }
        else
        {
            // printf("������ʽ��ʧ�ܡ�����\r\n");
        }
    }
    else if (res_sd != FR_OK)
    {
        // printf("����SD�������ļ�ϵͳʧ�ܡ�(%d)\r\n", res_sd);
        // printf("��������ԭ��SD����ʼ�����ɹ���\r\n");
    }
    else
    {
        // printf("���ļ�ϵͳ���سɹ������Խ��ж�д����\r\n");
    }
    /* ���ļ�������ļ��������򴴽�����������ڣ�����ļ�ĩβ��ʼд�� */
    res_sd = f_open(&fnew, "0:datalog.txt", FA_OPEN_APPEND | FA_WRITE);
    if(res_sd == FR_OK)
    {
        file_open_flag = SET;
    }
    else
    {
        file_open_flag = RESET;
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
    /*----------------------- �ļ�ϵͳ���ԣ�д���� -----------------------------*/
    // printf("\r\n****** ���������ļ�д�����... ******\r\n");
    
    if (res_sd == FR_OK)
    {
        // printf("����/����FatFs��д�����ļ�.txt�ļ��ɹ������ļ�д�����ݡ�\r\n");
        /* ��ָ���洢������д�뵽�ļ��� */
        /*����˼·*/
        /*SD_W_Buff��ʱ���ʵ�����ݣ�����������ݣ�ѹ�뵽SD_W_RingBuffMgr���λ�����*/
        /*����Ӧ�����жϻ��λ������Ƿ������������������ջ��λ�����*/
        if(SD_W_RingBuff_IsOverFlow()==SET)
        {
            rb_clear(&SD_W_RingBuffMgr);
            return;
        }
        /*��SD_W_Buff�е�����ѹ�뵽SD_W_RingBuffMgr���λ�����*/
        res_sd = f_write(&fnew, SD_W_RingBuffMgr.pbuff, SD_W_RingBuffMgr.length, &fnum);
        if (res_sd == FR_OK)
        {
            // printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n", fnum);
            // printf("�����ļ�д�������Ϊ��\r\n%s\r\n", WriteBuffer);
        }
        else
        {
            // printf("�����ļ�д��ʧ�ܣ�(%d)\n", res_sd);
        }
        /* ���ٶ�д���ر��ļ� */
        f_close(&fnew);
    }
    else
    {
        // printf("������/�����ļ�ʧ�ܡ�\r\n");
    }

    /*------------------- �ļ�ϵͳ���ԣ������� ------------------------------------*/
    // printf("****** ���������ļ���ȡ����... ******\r\n");
    res_sd = f_open(&fnew, "0:datalog.txt", FA_OPEN_EXISTING | FA_READ);
    if (res_sd == FR_OK)
    {
        // LED_GREEN;
        // printf("�����ļ��ɹ���\r\n");
        res_sd = f_read(&fnew, ReadBuffer, sizeof(ReadBuffer), &fnum);
        if (res_sd == FR_OK)
        {
            // printf("���ļ���ȡ�ɹ�,�����ֽ����ݣ�%d\r\n", fnum);
            // printf("����ȡ�õ��ļ�����Ϊ��\r\n%s \r\n", ReadBuffer);
        }
        else
        {
            // printf("�����ļ���ȡʧ�ܣ�(%d)\n", res_sd);
        }
    }
    else
    {
        // LED_RED;
        // printf("�������ļ�ʧ�ܡ�\r\n");
    }
    /* ���ٶ�д���ر��ļ� */
    f_close(&fnew);

    /* ����ʹ���ļ�ϵͳ��ȡ�������ļ�ϵͳ */
    f_mount(NULL, "0:", 1);
}




void task_datalog(void)
{
    /*----------------------- �ļ�ϵͳ���ԣ�д���� -----------------------------*/
    /* ���ļ�������ļ��������򴴽��� */
    // printf("\r\n****** ���������ļ�д�����... ******\r\n");
    res_sd = f_open(&fnew, "0:datalog.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res_sd == FR_OK)
    {
        // printf("����/����FatFs��д�����ļ�.txt�ļ��ɹ������ļ�д�����ݡ�\r\n");
        /* ��ָ���洢������д�뵽�ļ��� */
        res_sd = f_write(&fnew, WriteBuffer, 1024, &fnum);
        if (res_sd == FR_OK)
        {
            // printf("���ļ�д��ɹ���д���ֽ����ݣ�%d\n", fnum);
            // printf("�����ļ�д�������Ϊ��\r\n%s\r\n", WriteBuffer);
        }
        else
        {
            // printf("�����ļ�д��ʧ�ܣ�(%d)\n", res_sd);
        }
        /* ���ٶ�д���ر��ļ� */
        f_close(&fnew);
    }
    else
    {
        // printf("������/�����ļ�ʧ�ܡ�\r\n");
    }
    /* ����ʹ���ļ�ϵͳ��ȡ�������ļ�ϵͳ */
    f_mount(NULL, "0:", 1);
}

