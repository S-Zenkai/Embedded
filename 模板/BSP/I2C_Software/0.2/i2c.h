#ifndef __I2C_SW_H
#define __I2C_SW_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <stdbool.h> // Include for bool type

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

/**
  * @brief  发送一个字节数据到I2C总线
  * @param  byte: 要发送的字节
  * @retval bool: 返回true表示收到ACK，false表示收到NACK
  */
bool i2c_send_byte(uint8_t byte);

/**
  * @brief  从I2C总线接收一个字节数据
  * @param  send_ack: 是否在接收后发送ACK (true发送ACK, false发送NACK)
  * @retval uint8_t: 接收到的字节
  */
uint8_t i2c_receive_byte(bool send_ack);

/**
  * @brief  等待并读取从设备的ACK/NACK信号
  * @retval I2C_AckStatus_t: 返回ACK或NACK状态
  */
I2C_AckStatus_t i2c_wait_ack(void);

// --- 示例用法函数 (可选) ---
/**
  * @brief  I2C写一个字节到指定从设备和地址
  * @param  slave_addr: 从设备地址 (7位地址，不含R/W位)
  * @param  reg_addr: 寄存器地址
  * @param  data: 要写入的数据
  * @retval bool: 返回true表示写入成功，false表示失败 (例如：无ACK)
  */
bool i2c_master_write_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);

/**
  * @brief  I2C从指定从设备和地址读取一个字节
  * @param  slave_addr: 从设备地址 (7位地址，不含R/W位)
  * @param  reg_addr: 寄存器地址
  * @param  data: 指向存储读取数据的变量的指针
  * @retval bool: 返回true表示读取成功，false表示失败 (例如：无ACK)
  */
bool i2c_master_read_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data);


#endif /* __I2C_SW_H */