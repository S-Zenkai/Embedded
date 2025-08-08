#ifndef __I2C_SW_H
#define __I2C_SW_H

#include "stm32f4xx.h"
#include <stdbool.h> // Include for bool type
#include "bsp_systick.h"

// ����I2C���źͶ˿�
#define I2C_PORT     GPIOB
#define I2C_SCL_PIN  GPIO_Pin_8
#define I2C_SDA_PIN  GPIO_Pin_9
#define I2C_CLK      RCC_AHB1Periph_GPIOB

// ����I2C ACK/NACK״̬
typedef enum {
    I2C_ACK = 0, // �յ�ACK (SDA������)
    I2C_NACK = 1 // �յ�NACK (SDA���ָߵ�ƽ)
} I2C_AckStatus_t;

// --- ������������ ---

/**
  * @brief  ��ʼ��ģ��I2C GPIO����
  */
void i2c_init(void);

/**
  * @brief  ����I2C������ʼ�ź�
  */
void i2c_start(void);

/**
  * @brief  ����I2C����ֹͣ�ź�
  */
void i2c_stop(void);
void i2c_send_byte(uint8_t byte);
void i2c_send_ack(I2C_AckStatus_t ack_status);

/**
  * @brief  ��I2C���߽���һ���ֽ�����
  * @param  send_ack: �Ƿ��ڽ��պ���ACK,I2C_ACK��ʾ����ACK,I2C_NACK��ʾ����NACK
  * @retval uint8_t: ���յ����ֽ�
  */
uint8_t i2c_receive_byte(void);

/**
  * @brief  �ȴ�����ȡ���豸��ACK/NACK�ź�
  * @retval I2C_AckStatus_t: ����ACK��NACK״̬
  */
I2C_AckStatus_t i2c_wait_ack(void);




#endif /* __I2C_SW_H */
