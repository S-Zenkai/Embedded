#include "bsp_mm.h"

//�ڴ��Memory Pool
uint8_t My_MemPool[My_MemPoolSz];
//�ڴ�����
My_MemMgmtBlkSz My_MemMgmtTbl[My_MemBlkNum];

//�ڴ���������
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
//�ڴ����
void* My_Mallco(uint32_t Num)
{
	uint32_t count=0;
	uint32_t blknum=0;//Ҫ������ڴ����
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
//�ڴ����
uint32_t Mallco(uint32_t Num)
{
	uint32_t count=0;
	uint32_t blknum=0;//Ҫ������ڴ����
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

    if (offset == 0XFFFFFFFF)   /* ������� */
    {
        return NULL;            /* ���ؿ�(0) */
    }
    else    /* ����û����, �����׵�ַ */
    {
        return (void *)((uint32_t)My_MemPool + offset);
    }
}
#elif 0
/**
 * @brief       �ڴ����(�ڲ�����)
 * @param       memx : �����ڴ��
 * @param       size : Ҫ������ڴ��С(�ֽ�)
 * @retval      �ڴ�ƫ�Ƶ�ַ
 *   @arg       0 ~ 0XFFFFFFFE : ��Ч���ڴ�ƫ�Ƶ�ַ
 *   @arg       0XFFFFFFFF     : ��Ч���ڴ�ƫ�Ƶ�ַ
 */
static uint32_t my_mem_malloc(uint32_t size)
{
    signed long offset = 0;
    uint32_t nmemb;     /* ��Ҫ���ڴ���� */
    uint32_t cmemb = 0; /* �������ڴ���� */
    uint32_t i;

    if (!MemMgmtStruct.MemRdy)
    {
			MemMgmtStruct.Init();
    }
    
    if (size == 0) return 0XFFFFFFFF;   /* ����Ҫ���� */

    nmemb = size / My_MemBlkSz;    /* ��ȡ��Ҫ����������ڴ���� */

    if (size % My_MemBlkSz) nmemb++;

    for (offset = My_MemBlkNum - 1; offset >= 0; offset--)  /* ���������ڴ������ */
    {
        if (!MemMgmtStruct.MemMgmtTbl[offset])
        {
            cmemb++;            /* �������ڴ�������� */
        }
        else 
        {
            cmemb = 0;          /* �����ڴ������ */
        }
        
        if (cmemb == nmemb)     /* �ҵ�������nmemb�����ڴ�� */
        {
            for (i = 0; i < nmemb; i++) /* ��ע�ڴ��ǿ� */
            {
                MemMgmtStruct.MemMgmtTbl[offset + i] = nmemb;
            }

            return (offset * My_MemBlkSz); /* ����ƫ�Ƶ�ַ */
        }
    }

    return 0XFFFFFFFF;  /* δ�ҵ����Ϸ����������ڴ�� */
}

/**
 * @brief       �����ڴ�(�ⲿ����)
 * @param       memx : �����ڴ��
 * @param       size : Ҫ������ڴ��С(�ֽ�)
 * @retval      ���䵽���ڴ��׵�ַ.
 */
void *mymalloc(uint32_t size)
{
    uint32_t offset;
    offset = my_mem_malloc(size);

    if (offset == 0XFFFFFFFF)   /* ������� */
    {
        return NULL;            /* ���ؿ�(0) */
    }
    else    /* ����û����, �����׵�ַ */
    {
        return (void *)((uint32_t)My_MemPool + offset);
    }
}
#endif


//�ڴ��ͷ�
uint8_t My_Free(void* addr)
{
	uint32_t blknum=0;//Ҫ�ͷŵ��ڴ����
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






