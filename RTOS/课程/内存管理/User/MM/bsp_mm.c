#include "bsp_mm.h"

//内存池Memory Pool
uint8_t My_MemPool[My_MemPoolSz];
//内存管理表
My_MemMgmtBlkSz My_MemMgmtTbl[My_MemBlkNum];

//内存分配控制器
struct My_MemMgmtStruct MemMgmtStruct=
{
	My_Mallco_Init,
	My_Mallco_UsageRate,
	My_MemPool,
	My_MemMgmtTbl,
	0
};


static void My_Mallco_WriMemMgmtTbl(uint16_t* MemBlkBaseAddr,uint8_t Val,uint8_t MemBlkNum)
{
	while(MemBlkNum--)
	{
		*MemBlkBaseAddr=Val;
	}
}

/**
  *@brief  
  *@param
  *@retal
  */
void My_Mallco_Init(void)
{
	uint16_t i;
	for(i=0;i<My_MemBlkNum;i++)
	{
		My_MemMgmtTbl[i]=0;
	}
	MemMgmtStruct.MemRdy=1;
}	

uint16_t My_Mallco_UsageRate(void)
{
	uint16_t count=0;
	for(uint16_t i=0;i<My_MemBlkNum;i++)
	{
		if(My_MemMgmtTbl[i])
		{
			count++;
		}
	}
	return count*1000/My_MemBlkNum;
}



#if 1
//内存分配
void* My_Mallco(uint32_t Num)
{
	uint32_t count=0;
	uint32_t blknum=0;//要分配的内存块数
	int i;
	if(!MemMgmtStruct.MemRdy)
	{
		MemMgmtStruct.Init();
	}
	if(Num==0)
	{
		return NULL;
	}
	blknum=Num/My_MemBlkSz;
	if(Num%My_MemBlkSz)
		blknum++;
	for(i=My_MemBlkNum-1;i>=0;i--)
	{
		if(MemMgmtStruct.MemMgmtTbl[i]==0)
		{
			count++;
		}
		else
			count=0;
		if(count==blknum)
			break;
	}
	if(count<blknum)
		return NULL;
	for(count=0;count<blknum;count++)
	{
		MemMgmtStruct.MemMgmtTbl[i+count]=blknum;
	}
	return (void*)((uint32_t)My_MemPool+i*My_MemBlkSz);
}
#elif 0
//内存分配
uint32_t Mallco(uint32_t Num)
{
	uint32_t count=0;
	uint32_t blknum=0;//要分配的内存块数
	int i=0;
	if(!MemMgmtStruct.MemRdy)
	{
		MemMgmtStruct.Init();
	}
	if(Num==0)
	{
		return 0xFFFFFFFF;
	}
	blknum=Num/My_MemBlkSz;
	if(Num%My_MemBlkSz!=0)
		blknum++;
	for(i=My_MemBlkNum-1;i>=0;i--)
	{
		if(MemMgmtStruct.MemMgmtTbl[i]==0)
		{
			count++;
		}
		else
			count=0;
		if(count==blknum)
			break;
	}
	if(count<blknum)
		return 0xFFFFFFFF;
	for(count=0;count<blknum;count++)
	{
		MemMgmtStruct.MemMgmtTbl[i+count]=blknum;
	}
	return (uint32_t)i*My_MemBlkSz;
}
void* My_Mallco(uint32_t Num)
{
    uint32_t offset;
    offset = Mallco(Num);

    if (offset == 0XFFFFFFFF)   /* 申请出错 */
    {
        return NULL;            /* 返回空(0) */
    }
    else    /* 申请没问题, 返回首地址 */
    {
        return (void *)((uint32_t)My_MemPool + offset);
    }
}
#elif 0
/**
 * @brief       内存分配(内部调用)
 * @param       memx : 所属内存块
 * @param       size : 要分配的内存大小(字节)
 * @retval      内存偏移地址
 *   @arg       0 ~ 0XFFFFFFFE : 有效的内存偏移地址
 *   @arg       0XFFFFFFFF     : 无效的内存偏移地址
 */
static uint32_t my_mem_malloc(uint32_t size)
{
    signed long offset = 0;
    uint32_t nmemb;     /* 需要的内存块数 */
    uint32_t cmemb = 0; /* 连续空内存块数 */
    uint32_t i;

    if (!MemMgmtStruct.MemRdy)
    {
			MemMgmtStruct.Init();
    }
    
    if (size == 0) return 0XFFFFFFFF;   /* 不需要分配 */

    nmemb = size / My_MemBlkSz;    /* 获取需要分配的连续内存块数 */

    if (size % My_MemBlkSz) nmemb++;

    for (offset = My_MemBlkNum - 1; offset >= 0; offset--)  /* 搜索整个内存控制区 */
    {
        if (!MemMgmtStruct.MemMgmtTbl[offset])
        {
            cmemb++;            /* 连续空内存块数增加 */
        }
        else 
        {
            cmemb = 0;          /* 连续内存块清零 */
        }
        
        if (cmemb == nmemb)     /* 找到了连续nmemb个空内存块 */
        {
            for (i = 0; i < nmemb; i++) /* 标注内存块非空 */
            {
                MemMgmtStruct.MemMgmtTbl[offset + i] = nmemb;
            }

            return (offset * My_MemBlkSz); /* 返回偏移地址 */
        }
    }

    return 0XFFFFFFFF;  /* 未找到符合分配条件的内存块 */
}

/**
 * @brief       分配内存(外部调用)
 * @param       memx : 所属内存块
 * @param       size : 要分配的内存大小(字节)
 * @retval      分配到的内存首地址.
 */
void *mymalloc(uint32_t size)
{
    uint32_t offset;
    offset = my_mem_malloc(size);

    if (offset == 0XFFFFFFFF)   /* 申请出错 */
    {
        return NULL;            /* 返回空(0) */
    }
    else    /* 申请没问题, 返回首地址 */
    {
        return (void *)((uint32_t)My_MemPool + offset);
    }
}
#endif


//内存释放
uint8_t My_Free(void* addr)
{
	uint32_t blknum=0;//要释放的内存块数
	uint32_t offset=0;
	if(addr==NULL)
	{
		return 0;
	}
	if(!MemMgmtStruct.MemRdy)
	{
		MemMgmtStruct.Init();
		return 0;
	}
	offset=((uint32_t)addr-(uint32_t)My_MemPool)/My_MemBlkSz;
//	if(((uint32_t)addr-(uint32_t)My_MemPool)%My_MemBlkSz)
//		offset++;
	if(offset<My_MemBlkNum)
	{
	  blknum=MemMgmtStruct.MemMgmtTbl[offset];
	  My_Mallco_WriMemMgmtTbl(MemMgmtStruct.MemMgmtTbl+offset,0,blknum);
		return 1;
	}
	else
		return 0;
}	






