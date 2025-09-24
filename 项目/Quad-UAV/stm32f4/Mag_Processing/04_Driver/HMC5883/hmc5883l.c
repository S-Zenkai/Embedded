/**
 ******************************************************************************
 * @file    hmc5883l.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/05/20
 * @brief   HMC5883L磁力计驱动
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hmc5883l.h"

// 当前设置的灵敏度（LSB/Gauss），用于后续数据转换
static float sensitivity;

// 内部使用的I2C读写函数，增加了错误处理
static bool hmc_write_reg(uint8_t reg, uint8_t val);
static bool hmc_read_regs(uint8_t reg, uint8_t *data, uint8_t len);

/**
 * @brief I2C写一个字节到HMC5883L寄存器 (内部函数)
 */
static bool hmc_write_reg(uint8_t reg, uint8_t val)
{
    i2c_start();
    if (i2c_write_byte((HMC5883L_I2C_ADDRESS << 1) | 0x00) == I2C_NACK)
    {
        i2c_stop();
        return false;
    }
    if (i2c_write_byte(reg) == I2C_NACK)
    {
        i2c_stop();
        return false;
    }
    if (i2c_write_byte(val) == I2C_NACK)
    {
        i2c_stop();
        return false;
    }
    i2c_stop();
    return true;
}

/**
 * @brief I2C从HMC5883L连续读取多个字节 (内部函数)
 */
static bool hmc_read_regs(uint8_t reg, uint8_t *data, uint8_t len)
{
    i2c_start();
    // 1. 发送写地址，指定要读取的寄存器
    if (i2c_write_byte((HMC5883L_I2C_ADDRESS << 1) | 0x00) == I2C_NACK)
    {
        i2c_stop();
        return false;
    }
    if (i2c_write_byte(reg) == I2C_NACK)
    {
        i2c_stop();
        return false;
    }
    // 2. 重复起始信号，切换到读模式
    i2c_start();
    if (i2c_write_byte((HMC5883L_I2C_ADDRESS << 1) | 0x01) == I2C_NACK)
    {
        i2c_stop();
        return false;
    }
    // 3. 连续读取数据
    for (uint8_t i = 0; i < len - 1; i++)
    {
        data[i] = i2c_read_byte(I2C_ACK);
    }
    data[len - 1] = i2c_read_byte(I2C_NACK);
    i2c_stop();
    return true;
}

/**
 * @brief 设置增益并更新灵敏度变量
 */
static bool hmc5883l_set_gain(HMC5883L_Gain_t gain)
{
    switch (gain)
    {
    case HMC5883L_GAIN_1370:
        sensitivity = 1370.0f;
        break;
    case HMC5883L_GAIN_1090:
        sensitivity = 1090.0f;
        break;
    case HMC5883L_GAIN_820:
        sensitivity = 820.0f;
        break;
    case HMC5883L_GAIN_660:
        sensitivity = 660.0f;
        break;
    case HMC5883L_GAIN_440:
        sensitivity = 440.0f;
        break;
    case HMC5883L_GAIN_390:
        sensitivity = 390.0f;
        break;
    case HMC5883L_GAIN_330:
        sensitivity = 330.0f;
        break;
    case HMC5883L_GAIN_230:
        sensitivity = 230.0f;
        break;
    default:
        return false; // 无效的增益设置
    }
    return hmc_write_reg(HMC5883L_REG_CONFIG_B, gain);
}

/**
 * @brief  HMC5883L 初始化
 * @param  gain: 增益/量程设置
 * @param  rate: 数据输出速率
 * @param  samples: 采样平均次数
 * @retval bool: true表示成功, false表示失败
 */
bool hmc5883l_init(HMC5883L_Gain_t gain, HMC5883L_Rate_t rate, HMC5883L_Samples_t samples)
{
    i2c_init();
    delay_ms(10);

    if (!hmc5883l_check_id())
    {
        HMC5883L_DEBUG("HMC5883L not found!\r\n");
        return false;
    }

    // 1. 设置配置寄存器A (采样平均 + 数据速率)
    // HMC5883L datasheet建议在设置其他寄存器之前，先设置Config A
    uint8_t config_a = samples | rate;
    if (!hmc_write_reg(HMC5883L_REG_CONFIG_A, config_a))
        return false;

    // 2. 设置配置寄存器B (增益)
    if (!hmc5883l_set_gain(gain))
        return false;

    // 3. 设置模式寄存器 (连续测量模式)
    if (!hmc_write_reg(HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS))
        return false;

    delay_ms(10); // 等待模式切换完成
    HMC5883L_DEBUG("HMC5883L initialized successfully.\r\n");
    return true;
}

/**
 * @brief  检查HMC5883L的芯片ID
 * @retval bool: true表示ID正确, false表示错误
 */
bool hmc5883l_check_id(void)
{
    uint8_t id_buffer[3];
    if (hmc_read_regs(HMC5883L_REG_ID_A, id_buffer, 3))
    {
        // ID 寄存器应返回 'H', '4', '3'
        if (id_buffer[0] == 'H' && id_buffer[1] == '4' && id_buffer[2] == '3')
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief  读取测量的原始数据 (int16_t)
 * @param  data: 指向存储数据的结构体指针
 * @retval bool: true表示读取成功, false表示失败
 */
bool hmc5883l_read_raw_data(HMC5883L_RawData_t *data)
{
    uint8_t buffer[6];

    // 从第一个数据寄存器开始连续读取6个字节
    if (!hmc_read_regs(HMC5883L_REG_DATA_OUT_X_MSB, buffer, 6))
    {
        return false;
    }

    // 组合数据，注意HMC5883L的顺序是 X, Z, Y
    // 数据格式为 MSB, LSB (大端)
    data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->z = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->y = (int16_t)((buffer[4] << 8) | buffer[5]);

    return true;
}

/**
 * @brief  将原始数据转换为高斯单位 (float)
 * @note   这是一个独立的计算函数，避免在每次读取时都进行浮点运算
 * @param  gauss_data: 指向存储转换后数据的结构体指针
 * @param  raw_data: 包含原始数据的结构体指针
 */
void hmc5883l_convert_to_gauss(HMC5883L_GaussData_t *gauss_data, const HMC5883L_RawData_t *raw_data)
{
    gauss_data->x = (float)raw_data->x / sensitivity;
    gauss_data->y = (float)raw_data->y / sensitivity;
    gauss_data->z = (float)raw_data->z / sensitivity;
}
