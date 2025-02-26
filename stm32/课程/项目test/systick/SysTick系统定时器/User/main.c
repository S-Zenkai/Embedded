#include "stm32f10x.h"
#include "bsp.h"
#include "bsp_SisTick.h"

void Delay_us(uint16_t nus);
void Delay_ms(uint16_t nms);


/**
 * @brief       ϵͳʱ�ӳ�ʼ������
 * @param       plln: PLL��Ƶϵ��(PLL��Ƶ), RCC_PLLMul_x ���� x:[2,16]
                �ж�������λ��������ʱ�Ѿ���SystemInit()�г�ʼ��
 * @retval      ��
 */
void sys_stm32_clock_init(uint32_t pllmul)
{
	__IO uint32_t StartUpCounter = 0, HSEStartUpStatus = 0;
	//��RCC�����ʼ����Ĭ�ϸ�λ״̬
	RCC_DeInit();
	//ʹ��HSE
	RCC_HSEConfig(RCC_HSE_ON);
	//�ȴ�HSE����
	HSEStartUpStatus=RCC_WaitForHSEStartUp();
	//HSE��������
	if(HSEStartUpStatus)
	{
		 // ʹ��FLASH Ԥ��ȡ������
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

    // SYSCLK�������������ʱ��ı������ã�����ͳһ���ó�2
		// ���ó�2��ʱ��SYSCLK����48MҲ���Թ�����������ó�0����1��ʱ��
		// ������õ�SYSCLK�����˷�Χ�Ļ���������Ӳ�����󣬳��������
		// 0��0 < SYSCLK <= 24M
		// 1��24< SYSCLK <= 48M
		// 2��48< SYSCLK <= 72M
    FLASH_SetLatency(FLASH_Latency_2);
		//HCLK = SYSCLK
		//PCLK1=HCLK/2
		//PCLK2=HCLC
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//����PLLʱ��Դ����Ƶ����
		//PLLCLK=HSE*pllmul
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, pllmul);
		//ʹ��PLL
		RCC_PLLCmd(ENABLE);
		/* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		// ��ȡʱ���л�״̬λ��ȷ��PLLCLK��ѡΪϵͳʱ��
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


int main(void)
{
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);
	sys_stm32_clock_init(RCC_PLLMul_9);
	while(1)
	{
		LED_B_Config();
		LED_B(ON);
	  Delay_SisTick_ms(1000);
	  LED_B(OFF);
	  Delay_SisTick_ms(1000);
	}
}



