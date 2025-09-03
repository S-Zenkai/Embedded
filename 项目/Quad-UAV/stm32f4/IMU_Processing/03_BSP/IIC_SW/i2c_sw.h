#ifndef __I2C_SW_H
#define __I2C_SW_H

#include "stm32f4xx.h"
#include <stdbool.h> // Include for bool type
#include "bsp_systick.h"

// 定义I2C引脚和端口
#define I2C_PORT     GPIOB
#define I2C_SCL_PIN  GPIO_Pin_8
#define I2C_SDA_PIN  GPIO_Pin_9
#define I2C_CLK      RCC_AHB1Periph_GPIOB

// 定义I2C ACK/NACK状态
typedef enum {
    I2C_ACK = 0, // 收到ACK (SDA被拉低)
    I2C_NACK = 1 // 收到NACK (SDA保持高电平)
} I2C_AckStatus_t;

// --- 驱动函数声明 ---

/**
  * @brief  初始化模拟I2C GPIO引脚
  */
void i2c_init(void);

/**
  * @brief  发送I2C总线起始信号
  */
void i2c_start(void);

/**
  * @brief  发送I2C总线停止信号
  */
void i2c_stop(void);
void i2c_send_byte(uint8_t byte);
void i2c_send_ack(I2C_AckStatus_t ack_status);

/**
  * @brief  从I2C总线接收一个字节数据
  * @param  send_ack: 是否在接收后发送ACK,I2C_ACK表示发送ACK,I2C_NACK表示发送NACK
  * @retval uint8_t: 接收到的字节
  */
uint8_t i2c_receive_byte(void);

/**
  * @brief  等待并读取从设备的ACK/NACK信号
  * @retval I2C_AckStatus_t: 返回ACK或NACK状态
  */
I2C_AckStatus_t i2c_wait_ack(void);




#endif /* __I2C_SW_H */
