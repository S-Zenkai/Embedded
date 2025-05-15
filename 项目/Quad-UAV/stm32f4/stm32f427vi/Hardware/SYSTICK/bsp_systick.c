#include "bsp_systick.h"




/* 如果SYS_SUPPORT_OS定义了,说明要支持OS了(不限于UCOS) */
#if SYS_SUPPORT_OS

/* 添加公共头文件 (FreeRTOS 需要用到) */
#include "FreeRTOS.h"
#include "task.h"

extern void xPortSysTickHandler(void);
static uint16_t g_fac_us = 0; /* us延时倍乘数 */
/**
 * @brief systick 中断服务函数,使用 OS 时用到
 * @param ticks: 延时的节拍数
 * @retval 无
 */
void SysTick_Handler(void)
{
    /* OS 开始跑了,才执行正常的调度处理 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        xPortSysTickHandler();
    }
}

/**
 * @brief       初始化延迟函数
 * @param       sysclk: 系统时钟频率, 即CPU频率(HCLK)
 * @retval      无
 */
void SystickInit(uint16_t sysclk)
{
    uint32_t reload;
    SysTick->CTRL = 0;
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    g_fac_us = sysclk;
    reload = sysclk / 8;
    /* 使用 configTICK_RATE_HZ 计算重装载值
     * configTICK_RATE_HZ 在 FreeRTOSConfig.h 中定义
     */
    reload *= 1000000 / configTICK_RATE_HZ;
    /* 删除不用的 g_fac_ms 相关代码 */
    SysTick->CTRL |= 1 << 1;
    SysTick->LOAD = reload;
    SysTick->CTRL |= 1 << 0;
}

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
    for (i = 0; i < nms; i++)
    {
        delay_us(1000);
    }
}

#else /* 不使用OS时, 用以下代码 */

volatile uint32_t sysTickCounter = 0;

/**
 * @brief  配置SysTick定时器每1ms中断一次
 * @param  None
 * @retval None
 */
void systick_init(void) 
{
    /* 关闭SysTick,清空之前的状态 */
    SysTick->CTRL = 0;
    // 配置SysTick定时器
    // SysTick_Config默认使用HCLK。
    // (RCC_Clocks.HCLK_Frequency / 1000) 计算出1ms需要的计数值
    // SysTick_Config 是 CMSIS 提供的函数，它会：
    // 1. 设置 Reload 值
    // 2. 清零当前计数值
    // 3. 设置时钟源为 HCLK
    // 4. 使能 SysTick 中断
    // 5. 启动 SysTick 定时器
    if (SysTick_Config(SystemCoreClock  / 1000)) {
        // 如果配置失败 (例如 HCLK 频率过高导致 Reload 值超出范围)，进入死循环
        while (1);
    }
    // // 设置SysTick中断优先级 (如果需要调整的话)
    // NVIC_SetPriority(SysTick_IRQn, 0); // 例如设置为最高优先级
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) {
    sysTickCounter++; // 每当中断发生时，计数器加1
}

/**
 * @brief  获取自系统启动以来的毫秒数
 * @param  None
 * @retval uint32_t 当前的毫秒计数值
 */
uint32_t GetTick(void) 
{

    // 直接返回全局计数器的值
    // 在cortex-M4上，读取32位变量是原子操作，所以直接返回sysTickCounter是安全的
    return sysTickCounter;
    // 不过，对于仅读取一个由单个指令递增的变量，直接返回通常可行：
    // return sysTickCounter;
}


void delay_ms(uint32_t nms)
{
    uint32_t startTick = GetTick();
    // 使用差值比较来处理 getTick() 可能发生的溢出
    while ((GetTick() - startTick) < nms) 
    {
        /*在空闲时，CPU会进入低功耗模式，等待中断唤醒，适合电池供电的设备*/
        __WFI(); // 节能等待
    }
}


/**
 * @brief  微秒级延时函数
 * @param  nus: 要延时的微秒数
 * @retval None
 * @note   依赖 SysTick 计数器，精度受限于 HCLK 频率和代码执行时间，微秒级延时使用TIMER精度更高。
 * SystemCoreClock 必须准确反映 HCLK 频率(Hz)。
 */
void delay_us(uint32_t nus) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD; // 获取SysTick的重装载值

    // 计算需要的 SysTick 节拍数 (基于 HCLK)
    //SystemCoreClock是系统时钟频率，也即每秒的时钟周期数
    // SystemCoreClock / 1000000  得到每微秒对应的 HCLK 周期数
    ticks = nus * (SystemCoreClock / 1000000);

    told = SysTick->VAL; // 读取当前的 SysTick 倒计数器值
    while (1) 
    {
        tnow = SysTick->VAL; // 再次读取当前值
        if (tnow != told) 
        {
            if (tnow < told) 
            { 
                // 正常递减 (没有发生回绕)
                tcnt += told - tnow;
            } 
            else 
            { 
                // 发生回绕 (从 0 减到 reload)
                tcnt += reload - tnow + told;
            }
            told = tnow; // 更新上次的值
            if (tcnt >= ticks) 
            {
                break; // 已达到或超过所需的节拍数，延时结束
            }
        }
    }
}


#endif /* SYS_SUPPORT_OS */
