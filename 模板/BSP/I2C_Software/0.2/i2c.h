#ifndef __I2C_SW_H
#define __I2C_SW_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include <stdbool.h> // Include for bool type

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

/**
  * @brief  ����һ���ֽ����ݵ�I2C����
  * @param  byte: Ҫ���͵��ֽ�
  * @retval bool: ����true��ʾ�յ�ACK��false��ʾ�յ�NACK
  */
bool i2c_send_byte(uint8_t byte);

/**
  * @brief  ��I2C���߽���һ���ֽ�����
  * @param  send_ack: �Ƿ��ڽ��պ���ACK (true����ACK, false����NACK)
  * @retval uint8_t: ���յ����ֽ�
  */
uint8_t i2c_receive_byte(bool send_ack);

/**
  * @brief  �ȴ�����ȡ���豸��ACK/NACK�ź�
  * @retval I2C_AckStatus_t: ����ACK��NACK״̬
  */
I2C_AckStatus_t i2c_wait_ack(void);

// --- ʾ���÷����� (��ѡ) ---
/**
  * @brief  I2Cдһ���ֽڵ�ָ�����豸�͵�ַ
  * @param  slave_addr: ���豸��ַ (7λ��ַ������R/Wλ)
  * @param  reg_addr: �Ĵ�����ַ
  * @param  data: Ҫд�������
  * @retval bool: ����true��ʾд��ɹ���false��ʾʧ�� (���磺��ACK)
  */
bool i2c_master_write_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);

/**
  * @brief  I2C��ָ�����豸�͵�ַ��ȡһ���ֽ�
  * @param  slave_addr: ���豸��ַ (7λ��ַ������R/Wλ)
  * @param  reg_addr: �Ĵ�����ַ
  * @param  data: ָ��洢��ȡ���ݵı�����ָ��
  * @retval bool: ����true��ʾ��ȡ�ɹ���false��ʾʧ�� (���磺��ACK)
  */
bool i2c_master_read_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data);


#endif /* __I2C_SW_H */