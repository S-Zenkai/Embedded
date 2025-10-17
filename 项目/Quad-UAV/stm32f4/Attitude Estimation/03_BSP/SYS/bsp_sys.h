#ifndef __BSP_SYS_H
#define __BSP_SYS_H
#include "stm32f4xx.h"


/**
 * SYS_SUPPORT_OS用于定义系统文件夹是否支持OS
 * 0,不支持OS
 * 1,支持OS
 */
#define SYS_SUPPORT_OS          0


/*函数申明*******************************************************************************************/

void HSE_SetSysClock(uint32_t pllmul);                                          /* 系统时钟初始化函数 */

#endif /*__BSP_SYS_H*/

