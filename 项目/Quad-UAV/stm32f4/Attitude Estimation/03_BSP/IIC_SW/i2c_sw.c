/**
 ******************************************************************************
 * @file    i2c_sw.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/09/05
 * @brief   I2C软件模拟驱动
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */

#include "i2c_sw.h"

// 短延时函数，使用 NOP 替代函数调用以提高效率
#if (I2C_DELAY_US > 0)
#define i2c_delay() delay_us(I2C_DELAY_US)
#else
// 对于极高速CPU，可能需要几个NOP来保证时序
/*__NOP()是CMSIS提供的汇编内联函数，相当于什么都不做，只占用1个CPU指令周期*/
#define i2c_delay() \
    do              \
    {               \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
        __NOP();    \
    } while (0)
#endif


// --- 驱动函数实现 ---

/**
 * @brief  初始化模拟I2C GPIO引脚
 */
void i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    // 使能GPIOB时钟
    RCC_AHB1PeriphClockCmd(I2C_CLK, ENABLE);
    // 配置SCL引脚 (PB8)
    GPIO_InitStruct.GPIO_Pin = I2C_SCL_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;      // 输出模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;     // 开漏
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 无上下拉
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // 高速
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);
    // 配置SDA引脚 (PB9) - 初始设置为开漏输出，由外部上拉拉高
    GPIO_InitStruct.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // 高速
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);

    // 确保I2C总线空闲状态 (SCL=高, SDA=高)
    I2C_SCL_H();
    I2C_SDA_H();
    /*确保总线空闲*/
    i2c_stop();
}

/**
 * @brief  发送I2C总线起始信号
 *         条件：SCL为高时，SDA由高变低
 */
void i2c_start(void)
{
    I2C_SDA_H(); // 确保SDA在高电平 (输入模式)
    I2C_SCL_H(); // 确保SCL在高电平
    /*下面的延时可能是没有必要的*/
    i2c_delay(); // 起始信号建立时间 (Ts_sta >= 4.7us for Standard-mode)

    I2C_SDA_L(); // SDA由高变低
    i2c_delay(); // 起始信号保持时间 (Thd_sta >= 4.0us for Standard-mode)

    I2C_SCL_L(); // 拉低SCL，准备传输数据
    /*下面的延时可能是没有必要的*/
    i2c_delay(); // 延时确保SCL已低
}

/**
 * @brief  发送I2C总线停止信号
 *         条件：SCL为高时，SDA由低变高
 */
void i2c_stop(void)
{
    I2C_SCL_L(); // 确保SCL在低电平
    I2C_SDA_L(); // 确保SDA在低电平 (输出模式)
    i2c_delay(); // 延时确保SCL已低

    I2C_SCL_H(); // 拉高SCL
    i2c_delay(); // 停止信号建立时间 (Ts_sto >= 4.0us for Standard-mode)

    I2C_SDA_H(); // SDA由低变高 (切换为输入模式)
    i2c_delay(); // 总线空闲时间 (Tbuf >= 4.7us for Standard-mode)
}

/**
 * @brief  发送一个直接并检查ACK/NACK
 * @param  byte: 要发送的字节 (MSB first)
 * @retval 返回ACK/NACK状态
 */
I2C_AckStatus_t i2c_write_byte(uint8_t byte)
{
    uint8_t i;
    I2C_AckStatus_t ack_status = I2C_ACK;
    for (i = 0; i < 8; i++)
    {
        if (byte & 0x80)
        { // 判断当前位 (MSB)
            I2C_SDA_H();
        }
        else
        {
            I2C_SDA_L();
        }
        i2c_delay(); // 数据建立时间 (Tsu_dat >= 250ns)

        I2C_SCL_H(); // 拉高SCL，从设备读取数据
        i2c_delay(); // SCL高电平时间 (Thigh >= 4.0us)
        I2C_SCL_L(); // 拉低SCL，允许SDA变化
        // i2c_delay(); // SCL低电平时间 (Tlow >= 4.7us)
        byte <<= 1; // 准备下一位
    }
    I2C_SDA_H();
    i2c_delay();
    I2C_SCL_H(); // 拉高SCL，从设备发送ACK/NACK
    i2c_delay(); // SCL高电平时间
    uint16_t timeout = 200;
    while (I2C_SDA_READ())
    {
        timeout--;
        if (timeout == 0)
        {
            i2c_stop();
            return I2C_NACK;
        }
    }
    ack_status = I2C_ACK;
    I2C_SCL_L(); // 拉低SCL，完成ACK/NACK时钟周期
    i2c_delay(); // SCL低电平时间
    return ack_status;
}

/**
 * @brief  从I2C总线接收一个字节数据
 * @param  send_ack: 是否在接收后发送ACK,I2C_ACK表示发送ACK,I2C_NACK表示发送NACK
 * @retval uint8_t: 接收到的字节
 */
uint8_t i2c_read_byte(I2C_AckStatus_t ack_status)
{
    uint8_t i;
    uint8_t received_data = 0;
    I2C_SDA_H(); /*主机释放SDA*/
    for (i = 0; i < 8; i++)
    {
        received_data <<= 1; // 准备存储下一位
        I2C_SCL_H();         // 拉高SCL，从设备准备发送数据
        i2c_delay();         // SCL高电平时间 (Thigh >= 4.0us)
        if (I2C_SDA_READ())
        {                          // 在SCL高电平期间读取SDA状态
            received_data |= 0x01; // 当前位是1
        }
        I2C_SCL_L();
        i2c_delay();
        // delay_us(1); // 数据保持时间 (Thd_dat >= 0ns, 但需要读取稳定)
    }
    // 发送ACK或NACK
    if (ack_status == I2C_ACK)
    {
        I2C_SDA_L(); // 发送ACK
    }
    else
    {
        I2C_SDA_H(); // 发送NACK
    }
    i2c_delay();
    I2C_SCL_H();
    i2c_delay();
    I2C_SCL_L();
    I2C_SDA_H(); // 释放SDA

    return received_data;
}





// --- 示例用法函数实现 ---
// /**
//  * @brief  I2C写一个字节到指定从设备和地址
//  * @param  slave_addr: 从设备地址 (7位地址)
//  * @param  reg_addr: 寄存器地址
//  * @param  data: 要写入的数据
//  * @retval bool: 返回true表示写入成功，false表示失败 (例如：无ACK)
//  */
// bool i2c_master_write_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
// {
//     bool success = false;

//     i2c_start();
//     // 发送设备地址+写操作 (slave_addr << 1) | 0x00
//     if (i2c_send_byte((slave_addr << 1) | 0x00) == I2C_ACK)
//     {
//         // 发送寄存器地址
//         if (i2c_send_byte(reg_addr) == I2C_ACK)
//         {
//             // 发送数据
//             if (i2c_send_byte(data) == I2C_ACK)
//             {
//                 success = true; // 所有步骤都收到了ACK
//             }
//         }
//     }
//     i2c_stop();

//     // 某些从设备需要等待写入周期完成 (例如 EEPROM)
//     // 可以通过反复发送带有写位的从设备地址来轮询ACK
//     // if (success)
//     // {
//     //     // delay_us(5000);
//     //     // 简单的等待，更安全的方式是轮询ACK
//     //     /*
//     //     // 轮询ACK示例 (更可靠)
//     //     uint32_t timeout = 10000; // 设置一个超时计数器
//     //     while(timeout--) {
//     //         i2c_start();
//     //         if (i2c_send_byte((slave_addr << 1) | 0x00)) {
//     //             i2c_stop();
//     //             break; // 收到ACK，从设备准备好了
//     //         }
//     //         i2c_stop(); // 发送Stop后重试
//     //         delay_us(100); // 等待一下再重试
//     //         if (timeout == 0) {
//     //             success = false; // 超时
//     //             break;
//     //         }
//     //     }
//     //     */
//     // }
//     return success;
// }

// /**
//  * @brief  I2C从指定从设备和地址读取一个字节 (随机读取)
//  * @param  slave_addr: 从设备地址 (7位地址)
//  * @param  reg_addr: 寄存器地址
//  * @param  data: 指向存储读取数据的变量的指针
//  * @retval bool: 返回true表示读取成功，false表示失败 (例如：无ACK)
//  */
// bool i2c_master_read_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data)
// {
//     bool success = false;

//     i2c_start();
//     // 1. 发送设备地址+写操作 (用于设置内部寄存器地址)
//     if (i2c_send_byte((slave_addr << 1) | 0x00) == I2C_ACK)
//     {
//         // 2. 发送要读取的寄存器地址
//         if (i2c_send_byte(reg_addr) == I2C_ACK)
//         {
//             // 3. 发送重复起始信号
//             i2c_start(); // Repeated Start

//             // 4. 发送设备地址+读操作
//             if (i2c_send_byte((slave_addr << 1) | 0x01))
//             {
//                 // 5. 接收数据并发送NACK (表示这是最后一个要读取的字节)
//                 *data = i2c_receive_byte(I2C_NACK); // false = 发送NACK
//                 success = true;
//             }
//         }
//     }
//     i2c_stop();
//     return success;
// }
