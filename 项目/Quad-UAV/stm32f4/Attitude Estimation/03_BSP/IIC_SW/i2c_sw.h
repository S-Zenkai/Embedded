#ifndef __I2C_SW_H
#define __I2C_SW_H

#include "stm32f4xx.h"
#include <stdbool.h>
#include "bsp_systick.h"

//================== I2C 配置 ==================//
// 通过修改 I2C_DELAY_US 可以调整I2C速度
// 对于400kHz快速模式, 周期为 2.5us, 高/低电平时间约为 1.25us
// 设置为 1 几乎可以达到 400kHz (考虑到函数调用开销)
// 设置为 5 大约是 100kHz
#define I2C_DELAY_US    1

// 定义I2C引脚和端口
#define I2C_PORT        GPIOB
#define I2C_SCL_PIN     GPIO_Pin_8
#define I2C_SDA_PIN     GPIO_Pin_9
#define I2C_CLK         RCC_AHB1Periph_GPIOB

//================== 内部宏定义 (无需修改) ==================//
// 使用 BSRR 寄存器可以原子地、更快地设置或复位GPIO状态
#define I2C_SCL_H()     (I2C_PORT->BSRR = I2C_SCL_PIN)
#define I2C_SCL_L()     (I2C_PORT->BSRR = (uint32_t)I2C_SCL_PIN << 16) // 正确
#define I2C_SDA_H()     (I2C_PORT->BSRR = I2C_SDA_PIN)
#define I2C_SDA_L()     (I2C_PORT->BSRR = (uint32_t)I2C_SDA_PIN << 16) // 正确
#define I2C_SDA_READ()  ((I2C_PORT->IDR & I2C_SDA_PIN) != 0)

// I2C ACK/NACK 状态
typedef enum {
    I2C_ACK  = 0,
    I2C_NACK = 1
} I2C_AckStatus_t;

// --- 驱动函数声明 ---
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);

/**
 * @brief 发送一个字节并等待ACK
 * @param byte 要发送的字节
 * @return I2C_AckStatus_t I2C_ACK 或 I2C_NACK
 */
I2C_AckStatus_t i2c_write_byte(uint8_t byte);

/**
 * @brief 接收一个字节
 * @param ack_status 在接收后要发送的ACK/NACK状态
 * @return uint8_t 接收到的字节
 */
uint8_t i2c_read_byte(I2C_AckStatus_t ack_status);

#endif /* __I2C_SW_H */

