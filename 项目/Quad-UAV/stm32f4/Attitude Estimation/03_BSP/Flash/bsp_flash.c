/**
 ******************************************************************************
 * @file    bsp_flash.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/09/05
 * @brief   内部读写flash操作
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */

#include "bsp_flash.h"

// 读取指定地址的字(32位数据)
// faddr:读地址
// 返回值:对应数据.
// (此函数不依赖库，无需修改)
u32 STMFLASH_ReadWord(u32 faddr)
{
    return *(vu32 *)faddr;
}

// 获取某个地址所在的flash扇区
// addr:flash地址
// 返回值:扇区号, 如: FLASH_Sector_0
// (此函数逻辑不变，但返回值使用了标准库的宏定义，更具可读性)
u16 STMFLASH_GetFlashSector(u32 addr)
{
    if (addr < ADDR_FLASH_SECTOR_1)
        return FLASH_Sector_0;
    else if (addr < ADDR_FLASH_SECTOR_2)
        return FLASH_Sector_1;
    else if (addr < ADDR_FLASH_SECTOR_3)
        return FLASH_Sector_2;
    else if (addr < ADDR_FLASH_SECTOR_4)
        return FLASH_Sector_3;
    else if (addr < ADDR_FLASH_SECTOR_5)
        return FLASH_Sector_4;
    else if (addr < ADDR_FLASH_SECTOR_6)
        return FLASH_Sector_5;
    else if (addr < ADDR_FLASH_SECTOR_7)
        return FLASH_Sector_6;
    else if (addr < ADDR_FLASH_SECTOR_8)
        return FLASH_Sector_7;
    else if (addr < ADDR_FLASH_SECTOR_9)
        return FLASH_Sector_8;
    else if (addr < ADDR_FLASH_SECTOR_10)
        return FLASH_Sector_9;
    else if (addr < ADDR_FLASH_SECTOR_11)
        return FLASH_Sector_10;
    else if (addr < ADDR_FLASH_SECTOR_12)
        return FLASH_Sector_11;
    else if (addr < ADDR_FLASH_SECTOR_13)
        return FLASH_Sector_12;
    else if (addr < ADDR_FLASH_SECTOR_14)
        return FLASH_Sector_13;
    else if (addr < ADDR_FLASH_SECTOR_15)
        return FLASH_Sector_14;
    else if (addr < ADDR_FLASH_SECTOR_16)
        return FLASH_Sector_15;
    else if (addr < ADDR_FLASH_SECTOR_17)
        return FLASH_Sector_16;
    else if (addr < ADDR_FLASH_SECTOR_18)
        return FLASH_Sector_17;
    else if (addr < ADDR_FLASH_SECTOR_19)
        return FLASH_Sector_18;
    else if (addr < ADDR_FLASH_SECTOR_20)
        return FLASH_Sector_19;
    else if (addr < ADDR_FLASH_SECTOR_21)
        return FLASH_Sector_20;
    else if (addr < ADDR_FLASH_SECTOR_22)
        return FLASH_Sector_21;
    else if (addr < ADDR_FLASH_SECTOR_23)
        return FLASH_Sector_22;

    return FLASH_Sector_23;
}

// 从指定地址开始写入指定长度的数据
// 特别注意: 和原版函数行为一致，如果写入区域有数据，会擦除整个扇区！
// WriteAddr:起始地址(此地址必须为4的倍数!!)
// pBuffer:数据指针
// NumToWrite:字(32位)数(就是要写入的32位数据的个数)
void STMFLASH_Write(u32 WriteAddr, u32 *pBuffer, u32 NumToWrite)
{
    FLASH_Status flash_status = FLASH_COMPLETE;
    u32 addrx = 0;
    u32 endaddr = 0;

    if (WriteAddr < STM32_FLASH_BASE || WriteAddr % 4)
        return; // 非法地址

    // 解锁Flash
    FLASH_Unlock();

    // 清除所有待处理的标志位
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    addrx = WriteAddr;                    // 写入的起始地址
    endaddr = WriteAddr + NumToWrite * 4; // 写入的结束地址

    while (addrx < endaddr)
    {
        // 检查地址处是否是0xFFFFFFFF，如果不是，则需要擦除
        if (STMFLASH_ReadWord(addrx) != 0xFFFFFFFF)
        {
            // 擦除这个地址所在的扇区
            // VoltageRange_3 对应 2.7V 到 3.6V 的工作电压，这是最常用的
            flash_status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx), VoltageRange_3);

            // 如果擦除出错，则终止操作
            if (flash_status != FLASH_COMPLETE)
            {
                break;
            }
        }
        // 移动到下一个字的地址
        addrx += 4;
    }

    // 只有在之前的擦除操作都成功的情况下才进行写入
    if (flash_status == FLASH_COMPLETE)
    {
        // 循环写入数据
        while (WriteAddr < endaddr)
        {
            // 调用标准库的字写入函数
            flash_status = FLASH_ProgramWord(WriteAddr, *pBuffer);

            // 如果写入异常，则终止
            if (flash_status != FLASH_COMPLETE)
            {
                break;
            }

            WriteAddr += 4;
            pBuffer++;
        }
    }

    // 上锁Flash
    FLASH_Lock();
}

// 从指定地址开始读出指定长度的数据
// ReadAddr:起始地址
// pBuffer:数据指针
// NumToRead:字(32位)数
// (此函数不依赖库，无需修改)
void STMFLASH_Read(u32 ReadAddr, u32 *pBuffer, u32 NumToRead)
{
    u32 i;
    for (i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = STMFLASH_ReadWord(ReadAddr); // 读取4个字节
        ReadAddr += 4;                            // 偏移4个字节
    }
}
