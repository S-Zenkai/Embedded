#ifndef __BSP_SYS_H
#define __BSP_SYS_H
#include "stm32f10x.h"


/**
 * SYS_SUPPORT_OS���ڶ���ϵͳ�ļ����Ƿ�֧��OS
 * 0,��֧��OS
 * 1,֧��OS
 */
#define SYS_SUPPORT_OS          1


/*��������*******************************************************************************************/

void HSE_SetSysClock(uint32_t pllmul);                                          /* ϵͳʱ�ӳ�ʼ������ */
void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset);             /* �����ж�ƫ���� */
void sys_standby(void);                                                         /* �������ģʽ */
void sys_soft_reset(void);                                                      /* ϵͳ��λ */
uint8_t sys_clock_set(uint32_t plln);                                           /* ʱ�����ú��� */


/* ����Ϊ��ຯ�� */
void sys_wfi_set(void);                                                         /* ִ��WFIָ�� */
void sys_intx_disable(void);                                                    /* �ر������ж� */
void sys_intx_enable(void);                                                     /* ���������ж� */
void sys_msr_msp(uint32_t addr);                                                /* ����ջ����ַ */




#endif /*__BSP_SYS_H*/

