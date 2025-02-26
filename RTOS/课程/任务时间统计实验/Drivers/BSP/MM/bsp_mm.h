#ifndef __BSP_MM_H
#define __BSP_MM_H
#include "stm32f10x.h"

//�ڴ�ش�СMemory Pool Size
#define      My_MemPoolSz       20*1024
//�����ڴ���С
#define      My_MemBlkSz        32
//�ڴ����������Ӧ�ڴ�����������
#define      My_MemBlkNum       My_MemPoolSz/My_MemBlkSz
//�ڴ���������Memory management table sz
#define      My_MemMgmtBlkSz     uint16_t


//�ڴ���������־
#define MemRdy_ok 1
#define MemRdy_no 0

//�ڴ���������
struct My_MemMgmtStruct
{
	void (*Init)(void);//��ʼ��
	uint16_t (*UsageRate)(void);//�ڴ�ʹ����
	uint8_t* MemPool;//�ڴ��
	My_MemMgmtBlkSz* MemMgmtTbl;//�ڴ�����
	uint8_t MemRdy;//�ڴ�����Ƿ����
};

//#define NULL (uint32_t*)0
#define NULL 0

void My_Mallco_Init(void);
uint16_t My_Mallco_UsageRate(void);
void* My_Mallco(uint32_t Num);
uint8_t My_Free(void* addr);

void *mymalloc(uint32_t size);

#endif /*__BSP_MM_H*/

