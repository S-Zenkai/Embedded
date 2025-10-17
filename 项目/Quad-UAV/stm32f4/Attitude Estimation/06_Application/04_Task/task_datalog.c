/**
 ******************************************************************************
 * @file    task_datalog.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/08/02
 * @brief   数据记录任务,本文件仅实现了单数据的记录，多数据单文件记录需要定义一个数据包结构体
 *          ，并实现数据包的写入和读取，后续需要在task_datalog.c中实现多数据单文件记录
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
#include "bsp_systick.h"
#include <string.h>

FATFS fs;       /* FatFs文件系统对象 */
FIL fnew;       /* 文件对象 */
FRESULT res_sd; /* 文件操作结果 */
UINT fnum;      /* 文件成功读写数量 */
/*环形缓冲区管理变量*/
ringbuff_t SD_W_RingBuffMgr;
/*环形缓冲区大小*/
#define SD_W_BUFFER_SIZE 1024 * 10
/*环形缓冲区数组*/
uint8_t SD_W_Buff[SD_W_BUFFER_SIZE];
/*文件打开标志*/
FlagStatus file_open_flag = RESET;
/*定义环形缓冲区存储一定数量数据后，再写入到文件中，推荐为SD卡扇区大小的整数倍，提高写入效率*/
/*向SD卡写入数据时，单次写入的数据量越大，平均写入效率越高，若缓冲区常溢出可尝试增加该值*/
#define SD_W_DATA_MIN_SIZE 4096

/* 环形缓冲区初始化 */
static void SD_W_RB_Init(void)
{
    rb_init(SD_W_Buff, SD_W_BUFFER_SIZE, &SD_W_RingBuffMgr);
}

/**
 * @brief  环形缓冲区溢出判断
 * @note
 * @param  无
 * @retval 溢出为SET，反之为RESET
 */
static FlagStatus SD_W_RB_IsOverFlow(void)
{
    return SD_W_RingBuffMgr.overflow;
}

/**
 * @brief  获取当前缓冲区剩余数据数量
 * @note
 * @param  无
 * @retval 无
 */
static uint32_t SD_W_RB_GetCounter(void)
{
    return rb_GetDataCounter(&SD_W_RingBuffMgr);
}

/**
 * @brief  环形缓冲区压出一个字节
 * @note
 * @param  无
 * @retval 无
 */
static bool SD_W_RB_Pop(uint8_t *value)
{
    return rb_pop(&SD_W_RingBuffMgr, value);
}

/**
 * @brief  数据记录任务初始化
 * @note
 * @param  无
 * @retval 无
 */
void task_datalog_init(void)
{
    SD_W_RB_Init();
    // 在外部SPI Flash挂载文件系统，文件系统挂载时会对设备初始化
    res_sd = f_mount(&fs, "0:", 1);
    /* 如果没有文件系统就格式化创建创建文件系统 */
    if (res_sd == FR_NO_FILESYSTEM)
    {
        TASK_DATALOG1_DEBUG("》SD卡还没有文件系统，即将进行格式化...\r\n");
        /* 格式化 */
        /*这里SD_W_Buff复用用于给f_mkfs提供工作区，要求大小不小于FF_MAX_SS(系统支持的最大扇区大小)*/
        res_sd = f_mkfs("0:", 0, SD_W_Buff, sizeof(SD_W_Buff));
        if (res_sd == FR_OK)
        {
            TASK_DATALOG1_DEBUG("》SD卡已成功格式化文件系统。\r\n");
            /* 格式化后，先取消挂载 */
            res_sd = f_mount(NULL, "0:", 1);
            /* 重新挂载	*/
            res_sd = f_mount(&fs, "0:", 1);
        }
        else
        {
            TASK_DATALOG1_DEBUG("《《格式化失败。》》\r\n");
        }
    }
    else if (res_sd != FR_OK)
    {
        TASK_DATALOG1_DEBUG("！！可能原因：SD卡初始化不成功。\r\n");
        //        TASK_DATALOG1_DEBUG("!!SD卡挂载文件系统失败.(%d)\r\n", res_sd);
        TASK_DATALOG1_DEBUG("SD卡挂载文件系统失败(.%d)\r\n", res_sd);
        TASK_DATALOG1_DEBUG("SD卡挂载文件系统失败（.%d）\r\n", res_sd);
    }
    else
    {
        // TASK_DATALOG1_DEBUG("》文件系统挂载成功，可以进行读写测试\r\n");
    }
    /* 打开文件，如果文件不存在则创建它，如果存在，则从文件末尾开始写入 */
    res_sd = f_open(&fnew, "0:datalog.txt", FA_CREATE_ALWAYS | FA_WRITE);
    if (res_sd == FR_OK)
    {
        file_open_flag = SET;
    }
    else
    {
        file_open_flag = RESET;
        TASK_DATALOG1_DEBUG("！！打开文件失败。\r\n");
    }
}

/**
 * @brief  构建一个日志包并将其推入环形缓冲区
 * @param  msg_id          消息ID
 * @param  payload_data    指向要发送的负载数据的指针
 * @param  payload_len     负载数据的长度
 * @param  rb_mgr          指向目标环形缓冲区管理器的指针
 */
void Push_Log_Packet_To_RingBuff(uint8_t msg_id, const void *payload_data, uint8_t payload_len, ringbuff_t *rb_mgr)
{
    log_packet_t log_packet;

    // 确保负载数据不会溢出log_packet中的payload联合体
    if (payload_len > sizeof(log_packet.payload))
    {
        // 在这里可以添加错误处理逻辑, 例如打印日志或直接返回
        return;
    }

    // 1. 初始化并填充日志包头部和元数据
    memset(&log_packet, 0, sizeof(log_packet));
    log_packet.header1 = LOG_PACKET_HEADER_1;
    log_packet.header2 = LOG_PACKET_HEADER_2;
    log_packet.msg_id = msg_id;
    log_packet.msg_len = payload_len;
    log_packet.timestamp = GetTick();

    // 2. 使用memcpy安全地拷贝负载数据
    memcpy(&log_packet.payload, payload_data, payload_len);

    // 3. 计算校验和 (覆盖除最后一个字节外的所有数据)
    uint8_t checksum = 0;
    uint8_t *p = (uint8_t *)&log_packet;
    for (size_t i = 0; i < sizeof(log_packet) - 1; i++)
    {
        checksum ^= p[i];
    }
    log_packet.checksum = checksum;

    // 4. 将完整的日志包推入环形缓冲区
    rb_push_multi((uint8_t *)&log_packet, sizeof(log_packet), rb_mgr);
}

/**
 * @brief  数据记录任务，记录50s数据
 * @note
 * @param  无
 * @retval 无
 */
/**
 * @brief  数据记录任务，记录50s数据 (修改注释为：数据记录任务)
 * @note
 * @param  无
 * @retval 无
 */
void task_datalog(void)
{
    static uint8_t temp_buff[SD_W_DATA_MIN_SIZE]; /*这个数组较大，定义为static避免堆栈溢出*/
    
    // 6分钟后关闭文件的逻辑保持不变
    if (GetTick() >= 600000 && file_open_flag == SET)
    {
        f_sync(&fnew); // 在关闭前最好也同步一次，确保万无一失
        f_close(&fnew);
        file_open_flag = RESET;
        TASK_DATALOG1_DEBUG("》文件关闭\r\n");
        return;
    }

    if (file_open_flag == RESET)
    {
        return;
    }

    if (SD_W_RB_IsOverFlow() == SET)
    {
        rb_clear(&SD_W_RingBuffMgr);
        TASK_DATALOG1_DEBUG("》环形缓冲区溢出，清空环形缓冲区\r\n");
        return;
    }

    uint32_t data_counter = SD_W_RB_GetCounter();
    if (data_counter < SD_W_DATA_MIN_SIZE)
    {
        return;
    }

    /* 临界区保护对于多线程/中断访问共享资源是必要的 */
//    __disable_irq();
    for (uint32_t i = 0; i < SD_W_DATA_MIN_SIZE; i++)
    {
        SD_W_RB_Pop(&temp_buff[i]);
    }
//    __enable_irq();
    
    // 写入数据
    res_sd = f_write(&fnew, temp_buff, SD_W_DATA_MIN_SIZE, &fnum);
    if (res_sd == FR_OK)
    {
        // 检查是否所有数据都已写入
        if (fnum == SD_W_DATA_MIN_SIZE)
        {
            // ***** 这是解决问题的关键步骤 *****
            // 将文件缓冲区的数据同步到SD卡物理介质
            res_sd = f_sync(&fnew);
            if (res_sd != FR_OK)
            {
                TASK_DATALOG1_DEBUG("！！文件同步(f_sync)失败：（%d）\r\n", res_sd);
                // 这里可以加入更复杂的错误处理，比如尝试重新打开文件
            }
        }
    }
    else
    {
        TASK_DATALOG1_DEBUG("！！文件写入(f_write)失败：（%d）\r\n", res_sd);
        // 发生写入错误时，可能需要考虑重新初始化SD卡或停止记录
    }
}
