#ifndef __BSP_SYS_H
#define __BSP_SYS_H
#include "stm32f4xx.h"


/**
 * SYS_SUPPORT_OS���ڶ���ϵͳ�ļ����Ƿ�֧��OS
 * 0,��֧��OS
 * 1,֧��OS
 */
#define SYS_SUPPORT_OS          0


/*��������*******************************************************************************************/

void HSE_SetSysClock(uint32_t pllmul);                                          /* ϵͳʱ�ӳ�ʼ������ */

#endif /*__BSP_SYS_H*/

