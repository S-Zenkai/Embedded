#ifndef _STM32F10X_GPIO_H
#define _STM32F10X_GPIO_H
#include "stm32f10x.h"

//置位
void GPIO_SetBits(GPIO_Typedef* GPIOx,uint16_t GPIO_Pin);


//复位
void GPIO_ResetBits(GPIO_Typedef* GPIOx,uint16_t GPIO_Pin);


//GPIO引脚
#define GPIO_Pin0 (uint16_t)0x001
#define GPIO_Pin1 (uint16_t)0x002
#define GPIO_Pin2 (uint16_t)(1<<2)
#define GPIO_Pin3 (uint16_t)(1<<3)
#define GPIO_Pin4 (uint16_t)(1<<4)
#define GPIO_Pin5 (uint16_t)(1<<5)
#define GPIO_Pin6 (uint16_t)(1<<6)
#define GPIO_Pin7 (uint16_t)(1<<7)
#define GPIO_Pin8 (uint16_t)(1<<8)
#define GPIO_Pin9 (uint16_t)(1<<9)
#define GPIO_Pin10 (uint16_t)(1<<10)
#define GPIO_Pin11 (uint16_t)(1<<11)
#define GPIO_Pin12 (uint16_t)(1<<12)
#define GPIO_Pin13 (uint16_t)(1<<13)
#define GPIO_Pin14 (uint16_t)(1<<14)
#define GPIO_Pin15 (uint16_t)(1<<15)
#define GPIO_Pin16 (uint16_t)(1<<16)
#endif //_STM32F10X_GPIO_H


//初始化参数
typedef struct
{
  uint16_t GPIO_Pin; /*!< 选择要配置的 GPIO 引脚 */
  uint16_t GPIO_Speed; /*!< 选择 GPIO 引脚的速率 */
  uint16_t GPIO_Mode; /*!< 选择 GPIO 引脚的工作模式 */
}GPIO_InitTypeDef;

//定义输出速率
typedef enum
{
 GPIO_Speed_10MHz = 1, // 10MHZ (01)b
 GPIO_Speed_2MHz, // 2MHZ (10)b
 GPIO_Speed_50MHz // 50MHZ (11)b
}GPIOSpeed_TypeDef;

//定义工作模式
typedef enum
{
 GPIO_Mode_AIN = 0x0, // 模拟输入 (0000 0000)b
 GPIO_Mode_IN_FLOATING = 0x04, // 浮空输入 (0000 0100)b
 GPIO_Mode_IPD = 0x28, // 下拉输入 (0010 1000)b
 GPIO_Mode_IPU = 0x48, // 上拉输入 (0100 1000)b
 GPIO_Mode_Out_OD = 0x14, // 开漏输出 (0001 0100)b
 GPIO_Mode_Out_PP = 0x10, // 推挽输出 (0001 0000)b
 GPIO_Mode_AF_OD = 0x1C, // 复用开漏输出 (0001 1100)b
 GPIO_Mode_AF_PP = 0x18 // 复用推挽输出 (0001 1000)b
}GPIOMode_TypeDef;

void GPIO_Init(GPIO_Typedef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);

