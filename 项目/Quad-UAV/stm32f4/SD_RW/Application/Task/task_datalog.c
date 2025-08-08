/**
 ******************************************************************************
 * @file    task_datalog.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/08/02
 * @brief   数据记录任务
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

FATFS fs;                     /* FatFs文件系统对象 */
FIL fnew;                     /* 文件对象 */
FRESULT res_sd;               /* 文件操作结果 */
UINT fnum;                    /* 文件成功读写数量 */
BYTE ReadBuffer[1024] = {0};  /* 读缓冲区 */
BYTE WriteBuffer[1024] = {0}; /* 写缓冲区*/
/*环形缓冲区管理变量*/
ringbuff_t SD_W_RingBuffMgr;
/*环形缓冲区大小*/
#define SD_W_BUFFER_SIZE 1024
/*环形缓冲区数组*/
uint8_t SD_W_Buff[SD_W_BUFFER_SIZE];
/*文件打开标志*/
FlagStatus file_open_flag = RESET;

/* 环形缓冲区初始化 */
static void SD_W_RingBuff_Init(void)
{
    rb_init(SD_W_Buff, SD_W_BUFFER_SIZE, &SD_W_RingBuffMgr);
}

/**
 * @brief  环形缓冲区溢出判断
 * @note
 * @param  无
 * @retval 溢出为SET，反之为RESET
 */
static FlagStatus SD_W_RingBuff_IsOverFlow(void)
{
    return SD_W_RingBuffMgr.overflow;
}

/**
 * @brief  数据记录任务初始化
 * @note
 * @param  无
 * @retval 无
 */
void task_datalog_init(void)
{
    SD_W_RingBuff_Init();
    // 在外部SPI Flash挂载文件系统，文件系统挂载时会对设备初始化
    res_sd = f_mount(&fs, "0:", 1);
    /* 如果没有文件系统就格式化创建创建文件系统 */
    if (res_sd == FR_NO_FILESYSTEM)
    {
        //printf("》SD卡还没有文件系统，即将进行格式化...\r\n");
        /* 格式化 */
        /*这里SD_W_Buff复用用于给f_mkfs提供工作区，要求大小不小于FF_MAX_SS(系统支持的最大扇区大小)*/
        res_sd = f_mkfs("0:", 0, SD_W_Buff, sizeof(ReadBuffer));
        if (res_sd == FR_OK)
        {
            // printf("》SD卡已成功格式化文件系统。\r\n");
            /* 格式化后，先取消挂载 */
            res_sd = f_mount(NULL, "0:", 1);
            /* 重新挂载	*/
            res_sd = f_mount(&fs, "0:", 1);
        }
        else
        {
            // printf("《《格式化失败。》》\r\n");
        }
    }
    else if (res_sd != FR_OK)
    {
        // printf("！！SD卡挂载文件系统失败。(%d)\r\n", res_sd);
        // printf("！！可能原因：SD卡初始化不成功。\r\n");
    }
    else
    {
        // printf("》文件系统挂载成功，可以进行读写测试\r\n");
    }
    /* 打开文件，如果文件不存在则创建它，如果存在，则从文件末尾开始写入 */
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
 * @brief  数据记录任务
 * @note
 * @param  无
 * @retval 无
 */
void task_datalog(void)
{
    /*----------------------- 文件系统测试：写测试 -----------------------------*/
    // printf("\r\n****** 即将进行文件写入测试... ******\r\n");
    
    if (res_sd == FR_OK)
    {
        // printf("》打开/创建FatFs读写测试文件.txt文件成功，向文件写入数据。\r\n");
        /* 将指定存储区内容写入到文件内 */
        /*程序思路*/
        /*SD_W_Buff临时存放实验数据，测量后的数据，压入到SD_W_RingBuffMgr环形缓冲区*/
        /*这里应该先判断环形缓冲区是否溢出，如果溢出，则清空环形缓冲区*/
        if(SD_W_RingBuff_IsOverFlow()==SET)
        {
            rb_clear(&SD_W_RingBuffMgr);
            return;
        }
        /*将SD_W_Buff中的数据压入到SD_W_RingBuffMgr环形缓冲区*/
        res_sd = f_write(&fnew, SD_W_RingBuffMgr.pbuff, SD_W_RingBuffMgr.length, &fnum);
        if (res_sd == FR_OK)
        {
            // printf("》文件写入成功，写入字节数据：%d\n", fnum);
            // printf("》向文件写入的数据为：\r\n%s\r\n", WriteBuffer);
        }
        else
        {
            // printf("！！文件写入失败：(%d)\n", res_sd);
        }
        /* 不再读写，关闭文件 */
        f_close(&fnew);
    }
    else
    {
        // printf("！！打开/创建文件失败。\r\n");
    }

    /*------------------- 文件系统测试：读测试 ------------------------------------*/
    // printf("****** 即将进行文件读取测试... ******\r\n");
    res_sd = f_open(&fnew, "0:datalog.txt", FA_OPEN_EXISTING | FA_READ);
    if (res_sd == FR_OK)
    {
        // LED_GREEN;
        // printf("》打开文件成功。\r\n");
        res_sd = f_read(&fnew, ReadBuffer, sizeof(ReadBuffer), &fnum);
        if (res_sd == FR_OK)
        {
            // printf("》文件读取成功,读到字节数据：%d\r\n", fnum);
            // printf("》读取得的文件数据为：\r\n%s \r\n", ReadBuffer);
        }
        else
        {
            // printf("！！文件读取失败：(%d)\n", res_sd);
        }
    }
    else
    {
        // LED_RED;
        // printf("！！打开文件失败。\r\n");
    }
    /* 不再读写，关闭文件 */
    f_close(&fnew);

    /* 不再使用文件系统，取消挂载文件系统 */
    f_mount(NULL, "0:", 1);
}




void task_datalog(void)
{
    /*----------------------- 文件系统测试：写测试 -----------------------------*/
    /* 打开文件，如果文件不存在则创建它 */
    // printf("\r\n****** 即将进行文件写入测试... ******\r\n");
    res_sd = f_open(&fnew, "0:datalog.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res_sd == FR_OK)
    {
        // printf("》打开/创建FatFs读写测试文件.txt文件成功，向文件写入数据。\r\n");
        /* 将指定存储区内容写入到文件内 */
        res_sd = f_write(&fnew, WriteBuffer, 1024, &fnum);
        if (res_sd == FR_OK)
        {
            // printf("》文件写入成功，写入字节数据：%d\n", fnum);
            // printf("》向文件写入的数据为：\r\n%s\r\n", WriteBuffer);
        }
        else
        {
            // printf("！！文件写入失败：(%d)\n", res_sd);
        }
        /* 不再读写，关闭文件 */
        f_close(&fnew);
    }
    else
    {
        // printf("！！打开/创建文件失败。\r\n");
    }
    /* 不再使用文件系统，取消挂载文件系统 */
    f_mount(NULL, "0:", 1);
}

