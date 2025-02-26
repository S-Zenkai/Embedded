#include "bsp_sys.h"


/**
  *@brief  ʱ�����ú���������HSE���ø���ʱ�ӣ�HSEΪʱ��Դ��ͨ��PLL��ƵΪϵͳʱ��
	*@param  pllmul����Ƶ����,this parameter can be RCC_PLLMul_x where x:[2,16]  
	*@retval ��
	*/
void HSE_SetSysClock(uint32_t pllmul)
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
		RCC_PCLK1Config(RCC_HCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		//����PLLʱ��Դ����Ƶ����
		//SYSCLK=PLLCLK*pllmul
		RCC_PLLConfig(RCC_PLLSource_HSE_Div2, pllmul);
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

