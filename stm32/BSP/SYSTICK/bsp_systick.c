
#include "bsp_systick.h"
#include "bsp_sys.h"

static uint16_t  g_fac_us = 0;      /* us��ʱ������ */

/* ���SYS_SUPPORT_OS������,˵��Ҫ֧��OS��(������UCOS) */
#if SYS_SUPPORT_OS

/* ���ӹ���ͷ�ļ� (FreeRTOS ��Ҫ�õ�) */
#include "FreeRTOS.h"
#include "task.h"


extern void xPortSysTickHandler(void);

/**
 * @brief systick �жϷ�����,ʹ�� OS ʱ�õ�
 * @param ticks: ��ʱ�Ľ�����
 * @retval ��
 */
void SysTick_Handler(void)
{
	/* OS ��ʼ����,��ִ�������ĵ��ȴ��� */
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xPortSysTickHandler();
	}
}


#endif

/**
 * @brief       ��ʼ���ӳٺ���
 * @param       sysclk: ϵͳʱ��Ƶ��, ��CPUƵ��(HCLK)
 * @retval      ��
 */
void SysTick_Init(uint16_t sysclk)
{
#if SYS_SUPPORT_OS
	uint32_t reload;
#endif
	SysTick->CTRL = 0;
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	g_fac_us = sysclk / 8;
#if SYS_SUPPORT_OS
	reload = sysclk / 8;
	/* ʹ�� configTICK_RATE_HZ ������װ��ֵ
	* configTICK_RATE_HZ �� FreeRTOSConfig.h �ж���
	*/
	reload *= 1000000 / configTICK_RATE_HZ;
	/* ɾ�����õ� g_fac_ms ��ش��� */
	SysTick->CTRL |= 1 << 1;
	SysTick->LOAD = reload;
	SysTick->CTRL |= 1 << 0;
#endif
}

#if SYS_SUPPORT_OS  /* �����Ҫ֧��OS, �����´��� */

/**
 * @brief       ��ʱnus
 * @param       nus: Ҫ��ʱ��us��.
 * @note        nusȡֵ��Χ: 0 ~ 477218588(���ֵ��2^32 / g_fac_us @g_fac_us = 9)
 * @retval      ��
 */
void delay_us(uint32_t nus)
{
	uint32_t ticks;
	uint32_t told, tnow, tcnt = 0;
	uint32_t reload = SysTick->LOAD;
	/* ɾ�������� ��C/OS ��������������������Զ��庯�� */
	ticks = nus * g_fac_us;
	told = SysTick->VAL;
	while (1)
	{
		tnow = SysTick->VAL;
		if (tnow != told)
		{
			if (tnow < told)
			{
				tcnt += told - tnow;
			}
			else
			{
				tcnt += reload - tnow + told;
			}
			told = tnow;
			if (tcnt >= ticks)
			{
				break;
			}
		}
	}
	/* ɾ�������� ��C/OS ���ڽ���������������Զ��庯�� */
}

/**
 * @brief       ��ʱnms
 * @param       nms: Ҫ��ʱ��ms�� (0< nms <= 65535)
 * @retval      ��
 */
void delay_ms(uint16_t nms)
{
	uint32_t i;
	for (i=0; i<nms; i++)
	{
		delay_us(1000);
	}
}

#else  /* ��ʹ��OSʱ, �����´��� */

/**
 * @brief       ��ʱnus
 * @param       nus: Ҫ��ʱ��us��.
 * @note        ע��: nus��ֵ,��Ҫ����1864135us(���ֵ��2^24 / g_fac_us  @g_fac_us = 9)
 * @retval      ��
 */
void delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD = nus * g_fac_us; /* ʱ����� */
    SysTick->VAL = 0x00;            /* ��ռ����� */
    SysTick->CTRL |= 1 << 0 ;       /* ��ʼ���� */

    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); /* CTRL.ENABLEλ����Ϊ1, ���ȴ�ʱ�䵽�� */

    SysTick->CTRL &= ~(1 << 0) ;    /* �ر�SYSTICK */
    SysTick->VAL = 0X00;            /* ��ռ����� */
}

/**
 * @brief       ��ʱnms
 * @param       nms: Ҫ��ʱ��ms�� (0< nms <= 65535)
 * @retval      ��
 */
void delay_ms(uint16_t nms)
{
    uint32_t repeat = nms / 1000;   /*  ������1000,�ǿ��ǵ������г�ƵӦ��,
                                     *  ����128Mhz��ʱ��, delay_us���ֻ����ʱ1048576us������
                                     */
    uint32_t remain = nms % 1000;

    while (repeat)
    {
        delay_us(1000 * 1000);      /* ����delay_us ʵ�� 1000ms ��ʱ */
        repeat--;
    }

    if (remain)
    {
        delay_us(remain * 1000);    /* ����delay_us, ��β����ʱ(remain ms)������ */
    }
}

#endif
