/**
 ******************************************************************************
 * @file    qmc5883.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/05/20
 * @brief   QMC5883L磁力计驱动
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

/*传感器增益*/
uint16_t HMC5883L_GAIN = 0;
/*HMC2883L数据*/
HMC5883L_Data_t HMC5883L_Data;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  I2C写一个字节到QMC5883L寄存器
 * @param  reg_addr: 寄存器地址
 * @param  data: 要写入的数据
 * @retval bool: 返回true表示写入成功，false表示失败 (例如：无ACK)
 */
bool i2c_master_write_byte(uint8_t reg_addr, uint8_t data)
{
    bool success = false;
    i2c_start();
    // 发送设备地址+写操作 (slave_addr << 1) | 0x00
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
        HMC5883L_DEBUG("I2C写QMC5883L寄存器失败-reg_addr:%d,data:%d\r\n", reg_addr, data);
    }
    return success;
}

/**
 * @brief  I2C从QMC5883L寄存器读取数据(顺序读取多个数据)
 * @note
 * @param  reg_addr: 寄存器地址
 * @param  data: 指向存储读取数据的变量的指针
 * @param  length: 要读取的数据长度
 * @retval bool: 返回true表示读取成功，false表示失败 (例如：无ACK)
 */
bool i2c_master_read_data(uint8_t reg_addr, uint8_t *data, uint8_t length)
{
    bool success = false;
    i2c_start();
    // 1. 发送设备地址+写操作 (用于设置内部寄存器地址)
    i2c_send_byte((HMC5883L_I2C_ADDRESS << 1) | 0x00);
    if(i2c_wait_ack() != I2C_ACK)
    {
        goto flag1;
    }
    // 2. 发送要读取的寄存器地址
    i2c_send_byte(reg_addr);
    if(i2c_wait_ack() != I2C_ACK)
    {
        goto flag1;
    }
    // 3. 发送重复起始信号
    i2c_start();
    // 4. 发送设备地址+读操作
    i2c_send_byte((HMC5883L_I2C_ADDRESS << 1) | 0x01);
    if(i2c_wait_ack() != I2C_ACK)
    {
        goto flag1;
    }
    /*5.接收数据并发送ACK*/
    for (uint8_t i = 0; i < length - 1; i++)
    {
        data[i] = i2c_receive_byte(); // true = 发送ACK
        i2c_send_ack(I2C_ACK);
    }
    // 6. 接收数据并发送NACK (表示这是最后一个要读取的字节)
    data[length - 1] = i2c_receive_byte(); // false = 发送NACK
    i2c_send_ack(I2C_NACK);
    success = true;
flag1:
    i2c_stop();
    if (success != true)
    {
        HMC5883L_DEBUG("I2C读QMC5883L寄存器失败-reg_addr:%d\r\n", reg_addr);
    }
    return success;
}

/**
 * @brief  设置传感器增益
 * @note
 * @param  无
 * @retval 无
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
 * @brief  QMC5883L初始化
 * @note
 * @param  无
 * @retval 无
 */
void hmc5883l_init(void)
{
    /*等待qmc5883l上电稳定*/
    delay_ms(10);
    i2c_init();
    /*写配置寄存器A*/
    /*采样平均次数8(11)，数据输出数据15Hz(默认)(100),正常测量模式(00)*/
    i2c_master_write_byte(HMC5883L_REG_CONFIG_A, 0x70);
    /*设置传感器增益*/
    set_sensor_gain(HMC5883L_GAIN_1090);
    /*写模式寄存器*/
    i2c_master_write_byte(HMC5883L_REG_MODE, 0x01);
    // 单次装换时间最快在160hz
    delay_ms(10);
}

/**
 * @brief  HMC5883L启动测量一次数据(单次测量模式下，数据输出速率最大可达160Hz，连续测量模式则为75hz)
 * @note
 * @param  无
 * @retval 无
 */
void hmc5883l_single_measurement(void)
{
    /*写模式寄存器*/
    i2c_master_write_byte(HMC5883L_REG_MODE, 0x01);
}

/**
 * @brief  读取测量数据
 * @note
 * @param  无
 * @retval 无
 */
void hmc5883l_read_data(HMC5883L_Data_t *data)
{
    uint8_t temp[6];
    i2c_master_read_data(HMC5883L_REG_DOUT_X_MSB, temp, 6);
    data->x = (int16_t)((uint16_t)temp[0] << 8 | temp[1]);
    data->z = (int16_t)((uint16_t)temp[2] << 8 | temp[3]);
    data->y = (int16_t)((uint16_t)temp[4] << 8 | temp[5]);
    /*转换为高斯单位*/
    data->x = data->x / HMC5883L_GAIN;
    data->y = data->y / HMC5883L_GAIN;
    data->z = data->z / HMC5883L_GAIN;
}

/**
 * @}
 */
