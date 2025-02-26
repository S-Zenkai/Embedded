#include "bsp_sys.h"


/**
  *@brief  时钟配置函数，利用HSE配置各个时钟，HSE为时钟源，通过PLL倍频为系统时钟
	*@param  pllmul，倍频因子,this parameter can be RCC_PLLMul_x where x:[2,16]  
	*@retval 无
	*/
void HSE_SetSysClock(uint32_t pllmul)
{
	__IO uint32_t StartUpCounter = 0, HSEStartUpStatus = 0;
	//把RCC外设初始化成默认复位状态
	RCC_DeInit();
	//使能HSE
	RCC_HSEConfig(RCC_HSE_ON);
	//等待HSE启动
	HSEStartUpStatus=RCC_WaitForHSEStartUp();
	//HSE正常启动
	if(HSEStartUpStatus)
	{
		 // 使能FLASH 预存取缓冲区
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    // SYSCLK周期与闪存访问时间的比例设置，这里统一设置成2
		// 设置成2的时候，SYSCLK低于48M也可以工作，如果设置成0或者1的时候，
		// 如果配置的SYSCLK超出了范围的话，则会进入硬件错误，程序就死了
		// 0：0 < SYSCLK <= 24M
		// 1：24< SYSCLK <= 48M
		// 2：48< SYSCLK <= 72M
    FLASH_SetLatency(FLASH_Latency_2);
		//HCLK = SYSCLK
		//PCLK1=HCLK/2
		//PCLK2=HCLC
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//配置PLL时钟源、倍频因子
		//SYSCLK=PLLCLK*pllmul
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, pllmul);
		//使能PLL
		RCC_PLLCmd(ENABLE);
		/* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		// 读取时钟切换状态位，确保PLLCLK被选为系统时钟
    while (RCC_GetSYSCLKSource() != 0x08)
    {
    }
	}
	else
	{
		/* If HSE fails to start-up, the application will have wrong clock 
         configuration. User can add here some code to deal with this error */
	}
}


/**
 *  下面是正点原则sys.c中提供函数，以后用到要学，主要包括IO口配置，中断管理等
 */

/**
 * @brief       设置中断向量表偏移地址
 * @param       baseaddr: 基址
 * @param       offset: 偏移量(必须是0, 或者0X100的倍数)
 * @retval      无
 */
void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset)
{
    /* 设置NVIC的向量表偏移寄存器,VTOR低9位保留,即[8:0]保留 */
    SCB->VTOR = baseaddr | (offset & (uint32_t)0xFFFFFE00);
}

/**
 * @brief       执行: WFI指令(执行完该指令进入低功耗状态, 等待中断唤醒)
 * @param       无
 * @retval      无
 */
void sys_wfi_set(void)
{
    __ASM volatile("wfi");
}

/**
 * @brief       关闭所有中断(但是不包括fault和NMI中断)
 * @param       无
 * @retval      无
 */
void sys_intx_disable(void)
{
    __ASM volatile("cpsid i");
}

/**
 * @brief       开启所有中断
 * @param       无
 * @retval      无
 */
void sys_intx_enable(void)
{
    __ASM volatile("cpsie i");
}

/**
 * @brief       设置栈顶地址
 * @note        左侧的红X, 属于MDK误报, 实际是没问题的
 * @param       addr: 栈顶地址
 * @retval      无
 */
//void sys_msr_msp(uint32_t addr)
//{
//    __set_MSP(addr);    /* 设置栈顶地址 */
//}

/**
 *  下面的函数使用了hal库中的一个函数，在标准库中暂时未找到定义
 */

///**
// * @brief       进入待机模式
// * @param       无
// * @retval      无
// */
//void sys_standby(void)
//{
//    __HAL_RCC_PWR_CLK_ENABLE();    /* 使能电源时钟 */
//    SET_BIT(PWR->CR, PWR_CR_PDDS); /* 进入待机模式 */
//}


/**
 * @brief       系统软复位
 * @param       无
 * @retval      无
 */
void sys_soft_reset(void)
{
    NVIC_SystemReset();
}


