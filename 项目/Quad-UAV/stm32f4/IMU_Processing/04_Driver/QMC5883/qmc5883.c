/**
 ******************************************************************************
 * @file    qmc5883.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/05/20
 * @brief   QMC5883L����������
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "qmc5883.h"

/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*����������*/
uint16_t HMC5883L_GAIN = 0;
/*HMC2883L����*/
HMC5883L_Data_t HMC5883L_Data;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  I2Cдһ���ֽڵ�QMC5883L�Ĵ���
 * @param  reg_addr: �Ĵ�����ַ
 * @param  data: Ҫд�������
 * @retval bool: ����true��ʾд��ɹ���false��ʾʧ�� (���磺��ACK)
 */
bool i2c_master_write_byte(uint8_t reg_addr, uint8_t data)
{
    bool success = false;
    i2c_start();
    // �����豸��ַ+д���� (slave_addr << 1) | 0x00
    i2c_send_byte((HMC5883L_I2C_ADDRESS << 1) | 0x00);
    if(i2c_wait_ack() != I2C_ACK)
    {
        goto flag1;
    }
    i2c_send_byte(reg_addr);
    if(i2c_wait_ack() != I2C_ACK)
    {
        goto flag1;
    }
    i2c_send_byte(data);
    if(i2c_wait_ack() != I2C_ACK)
    {
        goto flag1;
    }
    success = true;
flag1:
    i2c_stop();
    if (success != true)
    {
        HMC5883L_DEBUG("I2CдQMC5883L�Ĵ���ʧ��-reg_addr:%d,data:%d\r\n", reg_addr, data);
    }
    return success;
}

/**
 * @brief  I2C��QMC5883L�Ĵ�����ȡ����(˳���ȡ�������)
 * @note
 * @param  reg_addr: �Ĵ�����ַ
 * @param  data: ָ��洢��ȡ���ݵı�����ָ��
 * @param  length: Ҫ��ȡ�����ݳ���
 * @retval bool: ����true��ʾ��ȡ�ɹ���false��ʾʧ�� (���磺��ACK)
 */
bool i2c_master_read_data(uint8_t reg_addr, uint8_t *data, uint8_t length)
{
    bool success = false;
    i2c_start();
    // 1. �����豸��ַ+д���� (���������ڲ��Ĵ�����ַ)
    i2c_send_byte((HMC5883L_I2C_ADDRESS << 1) | 0x00);
    if(i2c_wait_ack() != I2C_ACK)
    {
        goto flag1;
    }
    // 2. ����Ҫ��ȡ�ļĴ�����ַ
    i2c_send_byte(reg_addr);
    if(i2c_wait_ack() != I2C_ACK)
    {
        goto flag1;
    }
    // 3. �����ظ���ʼ�ź�
    i2c_start();
    // 4. �����豸��ַ+������
    i2c_send_byte((HMC5883L_I2C_ADDRESS << 1) | 0x01);
    if(i2c_wait_ack() != I2C_ACK)
    {
        goto flag1;
    }
    /*5.�������ݲ�����ACK*/
    for (uint8_t i = 0; i < length - 1; i++)
    {
        data[i] = i2c_receive_byte(); // true = ����ACK
        i2c_send_ack(I2C_ACK);
    }
    // 6. �������ݲ�����NACK (��ʾ�������һ��Ҫ��ȡ���ֽ�)
    data[length - 1] = i2c_receive_byte(); // false = ����NACK
    i2c_send_ack(I2C_NACK);
    success = true;
flag1:
    i2c_stop();
    if (success != true)
    {
        HMC5883L_DEBUG("I2C��QMC5883L�Ĵ���ʧ��-reg_addr:%d\r\n", reg_addr);
    }
    return success;
}

/**
 * @brief  ���ô���������
 * @note
 * @param  ��
 * @retval ��
 */
void set_sensor_gain(HMC5883L_Gain_t gain)
{
    i2c_master_write_byte(HMC5883L_REG_CONFIG_B, gain);
    switch (gain)
    {
    case HMC5883L_GAIN_1370:
        HMC5883L_GAIN = 1370;
        break;
    case HMC5883L_GAIN_1090:
        HMC5883L_GAIN = 1090;
        break;
    case HMC5883L_GAIN_820:
        HMC5883L_GAIN = 820;
        break;
    case HMC5883L_GAIN_660:
        HMC5883L_GAIN = 660;
        break;
    case HMC5883L_GAIN_440:
        HMC5883L_GAIN = 440;
        break;
    case HMC5883L_GAIN_390:
        HMC5883L_GAIN = 390;
        break;
    case HMC5883L_GAIN_330:
        HMC5883L_GAIN = 330;
        break;
    case HMC5883L_GAIN_230:
        HMC5883L_GAIN = 230;
        break;
    default:
        break;
    }
}

/**
 * @brief  QMC5883L��ʼ��
 * @note
 * @param  ��
 * @retval ��
 */
void hmc5883l_init(void)
{
    /*�ȴ�qmc5883l�ϵ��ȶ�*/
    delay_ms(10);
    i2c_init();
    /*д���üĴ���A*/
    /*����ƽ������8(11)�������������15Hz(Ĭ��)(100),��������ģʽ(00)*/
    i2c_master_write_byte(HMC5883L_REG_CONFIG_A, 0x70);
    /*���ô���������*/
    set_sensor_gain(HMC5883L_GAIN_1090);
    /*дģʽ�Ĵ���*/
    i2c_master_write_byte(HMC5883L_REG_MODE, 0x01);
    // ����װ��ʱ�������160hz
    delay_ms(10);
}

/**
 * @brief  HMC5883L��������һ������(���β���ģʽ�£���������������ɴ�160Hz����������ģʽ��Ϊ75hz)
 * @note
 * @param  ��
 * @retval ��
 */
void hmc5883l_single_measurement(void)
{
    /*дģʽ�Ĵ���*/
    i2c_master_write_byte(HMC5883L_REG_MODE, 0x01);
}

/**
 * @brief  ��ȡ��������
 * @note
 * @param  ��
 * @retval ��
 */
void hmc5883l_read_data(HMC5883L_Data_t *data)
{
    uint8_t temp[6];
    i2c_master_read_data(HMC5883L_REG_DOUT_X_MSB, temp, 6);
    data->x = (int16_t)((uint16_t)temp[0] << 8 | temp[1]);
    data->z = (int16_t)((uint16_t)temp[2] << 8 | temp[3]);
    data->y = (int16_t)((uint16_t)temp[4] << 8 | temp[5]);
    /*ת��Ϊ��˹��λ*/
    data->x = data->x / HMC5883L_GAIN;
    data->y = data->y / HMC5883L_GAIN;
    data->z = data->z / HMC5883L_GAIN;
}

/**
 * @}
 */
