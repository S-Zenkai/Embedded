#ifndef __BSP_MM_H
#define __BSP_MM_H
#include "stm32f10x.h"

//内存池大小Memory Pool Size
#define      My_MemPoolSz       20*1024
//单个内存块大小
#define      My_MemBlkSz        32
//内存块数量（对应内存管理表数量）
#define      My_MemBlkNum       My_MemPoolSz/My_MemBlkSz
//内存管理表类型Memory management table sz
#define      My_MemMgmtBlkSz     uint16_t


//内存管理就绪标志
#define MemRdy_ok 1
#define MemRdy_no 0

//内存管理控制器
struct My_MemMgmtStruct
{
	void (*Init)(void);//初始化
	uint16_t (*UsageRate)(void);//内存使用率
	uint8_t* MemPool;//内存池
	My_MemMgmtBlkSz* MemMgmtTbl;//内存管理表
	uint8_t MemRdy;//内存管理是否就绪
};

//#define NULL (uint32_t*)0
#define NULL 0

void My_Mallco_Init(void);
uint16_t My_Mallco_UsageRate(void);
void* My_Mallco(uint32_t Num);
uint8_t My_Free(void* addr);

void *mymalloc(uint32_t size);

#endif /*__BSP_MM_H*/

