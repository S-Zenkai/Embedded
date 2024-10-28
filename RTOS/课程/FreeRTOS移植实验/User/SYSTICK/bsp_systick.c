
#include "bsp_systick.h"
#include "bsp_sys.h"



#if 1
static uint16_t  g_fac_us = 0;      /* us延时倍乘数 */

/* 如果SYS_SUPPORT_OS定义了,说明要支持OS了(不限于UCOS) */
#if SYS_SUPPORT_OS

/* 添加公共头文件 (FreeRTOS 需要用到) */
#include "FreeRTOS.h"
#include "task.h"


extern void xPortSysTickHandler(void);


/**
 *  下面一段代码用于展示代替hal库中的函数，之后要搞懂，再想办法使用相同功能的语句替代下面的函数
 */

/*HAL_IncTick()的替代
 *HAL_IncTick()用于增加系统滴答定时器（uwTick）的值，用于时间管理和计时功能，这里默认为1KHz（HAL_TICK_FREQ_DEFAULT）
*/
__IO uint32_t uwTick;
/** @defgroup HAL_TICK_FREQ Tick Frequency
  * @{
  */
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;
HAL_TickFreqTypeDef uwTickFreq = HAL_TICK_FREQ_DEFAULT;  /* 1KHz */

/**
 * @brief systick 中断服务函数,使用 OS 时用到
 * @param ticks: 延时的节拍数
 * @retval 无
 */
void SysTick_Handler(void)
{
	//	HAL_IncTick();
	uwTick += uwTickFreq;
	/* OS 开始跑了,才执行正常的调度处理 */
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xPortSysTickHandler();
	}
}


#endif

/**
 * @brief       初始化延迟函数
 * @param       sysclk: 系统时钟频率, 即CPU频率(HCLK)
 * @retval      无
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
	/* 使用 configTICK_RATE_HZ 计算重装载值
	* configTICK_RATE_HZ 在 FreeRTOSConfig.h 中定义
	*/
	reload *= 1000000 / configTICK_RATE_HZ;
	/* 删除不用的 g_fac_ms 相关代码 */
	SysTick->CTRL |= 1 << 1;
	SysTick->LOAD = reload;
	SysTick->CTRL |= 1 << 0;
#endif
}

#if SYS_SUPPORT_OS  /* 如果需要支持OS, 用以下代码 */

/**
 * @brief       延时nus
 * @param       nus: 要延时的us数.
 * @note        nus取值范围: 0 ~ 477218588(最大值即2^32 / g_fac_us @g_fac_us = 9)
 * @retval      无
 */
void delay_us(uint32_t nus)
{
	uint32_t ticks;
	uint32_t told, tnow, tcnt = 0;
	uint32_t reload = SysTick->LOAD;
	/* 删除适用于 μC/OS 用于锁定任务调度器的自定义函数 */
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
	/* 删除适用于 μC/OS 用于解锁任务调度器的自定义函数 */
}

/**
 * @brief       延时nms
 * @param       nms: 要延时的ms数 (0< nms <= 65535)
 * @retval      无
 */
void delay_ms(uint16_t nms)
{
	uint32_t i;
	for (i=0; i<nms; i++)
	{
		delay_us(1000);
	}
}

#else  /* 不使用OS时, 用以下代码 */

/**
 * @brief       延时nus
 * @param       nus: 要延时的us数.
 * @note        注意: nus的值,不要大于1864135us(最大值即2^24 / g_fac_us  @g_fac_us = 9)
 * @retval      无
 */
void delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD = nus * g_fac_us; /* 时间加载 */
    SysTick->VAL = 0x00;            /* 清空计数器 */
    SysTick->CTRL |= 1 << 0 ;       /* 开始倒数 */

    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); /* CTRL.ENABLE位必须为1, 并等待时间到达 */

    SysTick->CTRL &= ~(1 << 0) ;    /* 关闭SYSTICK */
    SysTick->VAL = 0X00;            /* 清空计数器 */
}

/**
 * @brief       延时nms
 * @param       nms: 要延时的ms数 (0< nms <= 65535)
 * @retval      无
 */
void delay_ms(uint16_t nms)
{
    uint32_t repeat = nms / 1000;   /*  这里用1000,是考虑到可能有超频应用,
                                     *  比如128Mhz的时候, delay_us最大只能延时1048576us左右了
                                     */
    uint32_t remain = nms % 1000;

    while (repeat)
    {
        delay_us(1000 * 1000);      /* 利用delay_us 实现 1000ms 延时 */
        repeat--;
    }

    if (remain)
    {
        delay_us(remain * 1000);    /* 利用delay_us, 把尾数延时(remain ms)给做了 */
    }
}

#endif
#endif


#if 0
static uint16_t  g_fac_us = 0;      /* us延时倍乘数 */

/**
 * @brief       初始化延迟函数
 * @param       sysclk: 系统时钟频率, 即CPU频率(HCLK)
 * @retval      无
 * @note        这里systick时钟源要选择HCLK8分频而不是HCLK是因为：
 *              重装载寄存器的最大值是2^24（16777215），时钟源设计为72M时，delay_us要实现秒级延迟，
 *              需要装入LOAD值是72*M，大于重装载寄存器的最大值，即delay_us无法实现s级延迟，导致delay_ms
 *              也无法实现s级延迟
 */
void SysTick_Init(uint16_t sysclk)
{
		SysTick->CTRL = 0;
	  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
		g_fac_us = sysclk/8;
}




/**
 * @brief       延时nus
 * @param       nus: 要延时的us数.
 * @note        注意: nus的值,不要大于1864135us(最大值即2^24 / g_fac_us  @g_fac_us = 9)
 * @retval      无
 */
void delay_us(uint32_t nus)
{
    uint32_t temp;
    SysTick->LOAD = nus * g_fac_us; /* 时间加载 */
    SysTick->VAL = 0x00;            /* 清空计数器 */
    SysTick->CTRL |= 1 << 0 ;       /* 开始倒数 */

    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); /* CTRL.ENABLE位必须为1, 并等待时间到达 */

    SysTick->CTRL &= ~(1 << 0) ;    /* 关闭SYSTICK */
    SysTick->VAL = 0X00;            /* 清空计数器 */
}

/**
 * @brief       延时nms
 * @param       nms: 要延时的ms数 (0< nms <= 65535)
 * @retval      无
 */
void delay_ms(uint16_t nms)
{
    uint32_t repeat = nms / 1000;   /*  这里用1000,是考虑到可能有超频应用,
                                     *  比如128Mhz的时候, delay_us最大只能延时1048576us左右了
                                     */
    uint32_t remain = nms % 1000;

    while (repeat)
    {
        delay_us(1000 * 1000);      /* 利用delay_us 实现 1000ms 延时 */
        repeat--;
    }

    if (remain)
    {
        delay_us(remain * 1000);    /* 利用delay_us, 把尾数延时(remain ms)给做了 */
    }
}




#if 0
//定时时间计算：T=us*(SystemCoreClock/100000)/72M
//
void delay_us(__IO uint32_t us)
{
	SysTick_Config(SystemCoreClock/100000);
	//在每个定时周期结束后，CTRL的位16会置1
	while(us != 0)
	{
		while((SysTick->CTRL & (1<<16)) == 0);
		us--;
	}
	//关闭定时器
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}


void delay_ms(__IO uint32_t ms)
{
	SysTick_Config(SystemCoreClock/1000);
	//在每个定时周期结束后，CTRL的位16会置1
	while(ms != 0)
	{
		while((SysTick->CTRL & (1<<16)) == 0);
		ms--;
	}
	//关闭定时器
	SysTick->CTRL &=~SysTick_CTRL_ENABLE_Msk;
}


//为什么设置SisTick_Init？SysTick_Config只需配置一次，否则可能出错，故把Init单列出来
void SisTick_Init(void)
{
	//捕获错误
	if(SysTick_Config(SystemCoreClock/1000))
	{
		while(1);
	}
}
#endif
#endif

