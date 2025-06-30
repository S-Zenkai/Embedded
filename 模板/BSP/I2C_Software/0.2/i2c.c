#include "i2c.h"

// --- 私有函数声明 ---
static void i2c_delay_us(uint32_t us); // 微秒延时函数
static void I2C_SCL_H(void);           // SCL拉高 (释放)
static void I2C_SCL_L(void);           // SCL拉低 (驱动)
static void I2C_SDA_H(void);           // SDA拉高 (释放，模拟开漏)
static void I2C_SDA_L(void);           // SDA拉低 (驱动，模拟开漏)
static uint8_t I2C_SDA_READ(void);     // 读取SDA状态
static void I2C_SDA_OUT_MODE(void);    // 设置SDA为输出模式
static void I2C_SDA_IN_MODE(void);     // 设置SDA为输入模式

// --- 私有函数实现 ---

// 简单的微秒延时 (需要根据实际CPU频率和优化等级进行校准)
// 这是一个非常粗糙的延时，实际应用推荐使用 SysTick 或硬件定时器
static void i2c_delay_us(uint32_t us)
{
    volatile uint32_t i;
    // 以下循环次数是基于STM32F427VI的估算，在不同平台或优化等级下差异很大
    // 大约每100个循环消耗1微秒 (例如，主频180MHz时)
    uint32_t loops = us * 100;
    for (i = 0; i < loops; i++)
        ;
}

// 控制SCL引脚
static void I2C_SCL_H(void)
{
    GPIO_SetBits(I2C_PORT, I2C_SCL_PIN);
    i2c_delay_us(2); // 延时等待引脚状态稳定
}

static void I2C_SCL_L(void)
{
    GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
    i2c_delay_us(2); // 延时等待引脚状态稳定
}

// 控制SDA引脚，模拟开漏输出
// I2C_SDA_H(): 将SDA设置为输入模式，由外部上拉电阻拉高
static void I2C_SDA_H(void)
{
    GPIO_SetBits(I2C_PORT, I2C_SDA_PIN);
    i2c_delay_us(2); // 延时等待引脚状态稳定
}

// I2C_SDA_L(): 将SDA设置为输出模式并拉低
static void I2C_SDA_L(void)
{
    GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN); // 拉低SDA
    i2c_delay_us(2);                       // 延时确保引脚状态稳定
}

// 读取SDA引脚状态 (必须在SDA为输入模式时调用)
static uint8_t I2C_SDA_READ(void)
{
    return GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN);
}

// 配置SDA为输出模式
static void I2C_SDA_OUT_MODE(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;      // 输出模式
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // 推挽输出 (用于主动拉低)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 无上下拉，依赖外部
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // 高速
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

// 配置SDA为输入模式
static void I2C_SDA_IN_MODE(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;       // 输入模式
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 浮空输入，依赖外部上拉
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // 高速
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

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
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // 无上下拉
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // 高速
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);

    // 配置SDA引脚 (PB9) - 初始设置为输入，由外部上拉拉高
    GPIO_InitStruct.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;      // 输入模式
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;  // 浮空输入，依赖外部上拉
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // 高速
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);

    // 确保I2C总线空闲状态 (SCL=高, SDA=高)
    I2C_SCL_H();
    I2C_SDA_H(); // 确保SDA是输入模式

    // 可选：执行总线恢复序列，以防从设备挂住总线
    // 如果SDA被拉低，尝试发送9个时钟脉冲
    /*在iic协议中，若从设备挂住总线，传输时主设备产生时钟，在第九个时钟时，数据发送端会释放sda控制权，由数据接收端控制sda*/
    // if (I2C_SDA_READ() == 0)
    // {
    //     uint8_t i;
    //     for (i = 0; i < 9; i++)
    //     {
    //         I2C_SCL_L();
    //         i2c_delay_us(10); // 确保时钟低电平时间
    //         I2C_SCL_H();
    //         i2c_delay_us(10); // 确保时钟高电平时间
    //         if (I2C_SDA_READ() == 1)
    //             break; // SDA释放，总线恢复
    //     }
    //     // 尝试发送一个停止信号
    //     i2c_start(); // 需要一个Start才能发送Stop
    //     i2c_stop();
    // }
}

/**
 * @brief  发送I2C总线起始信号
 *         条件：SCL为高时，SDA由高变低
 */
void i2c_start(void)
{
    I2C_SDA_H();     // 确保SDA在高电平 (输入模式)
    I2C_SCL_H();     // 确保SCL在高电平
    /*下面的延时可能是没有必要的*/
    //i2c_delay_us(5); // 起始信号建立时间 (Ts_sta >= 4.7us for Standard-mode)

    I2C_SDA_L();     // SDA由高变低
    i2c_delay_us(5); // 起始信号保持时间 (Thd_sta >= 4.0us for Standard-mode)

    I2C_SCL_L();     // 拉低SCL，准备传输数据
    /*下面的延时可能是没有必要的*/
    //i2c_delay_us(2); // 延时确保SCL已低
}

/**
 * @brief  发送I2C总线停止信号
 *         条件：SCL为高时，SDA由低变高
 */
void i2c_stop(void)
{
    I2C_SDA_L();     // 确保SDA在低电平 (输出模式)
    // I2C_SCL_L();     // 确保SCL在低电平
    // i2c_delay_us(2); // 延时确保SCL已低

    I2C_SCL_H();     // 拉高SCL
    i2c_delay_us(5); // 停止信号建立时间 (Ts_sto >= 4.0us for Standard-mode)

    I2C_SDA_H();     // SDA由低变高 (切换为输入模式)
    i2c_delay_us(5); // 总线空闲时间 (Tbuf >= 4.7us for Standard-mode)
}

/**
 * @brief  发送一个字节数据到I2C总线
 * @param  byte: 要发送的字节 (MSB first)
 * @retval bool: 返回true表示收到ACK，false表示收到NACK
 */
bool i2c_send_byte(uint8_t byte)
{
    uint8_t i;

    I2C_SDA_OUT_MODE(); // 切换SDA为输出模式

    for (i = 0; i < 8; i++)
    {
        I2C_SCL_L();     // 拉低SCL，允许SDA变化
        i2c_delay_us(5); // SCL低电平时间 (Tlow >= 4.7us)

        if ((byte & 0x80) >> 7)
        { // 判断当前位 (MSB)
            I2C_SDA_H(); // 发送1 (释放SDA) - 理论上应该切换模式，但这里SDA一直是输出PP，通过Set/Reset控制
            // GPIO_SetBits(I2C_PORT, I2C_SDA_PIN); // SDA拉高
        }
        else
        {
            I2C_SDA_L(); // 发送0 (驱动SDA低)
            // GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN); // SDA拉低
        }
        i2c_delay_us(2); // 数据建立时间 (Tsu_dat >= 250ns)

        I2C_SCL_H();     // 拉高SCL，从设备读取数据
        i2c_delay_us(5); // SCL高电平时间 (Thigh >= 4.0us)

        byte <<= 1; // 准备下一位
    }

    // 释放SDA，等待从设备发送ACK/NACK
    // I2C_SDA_H(); // This is crucial for ACK/NACK - switch back to input mode
    /*这里通过配置输入模式，通过外部电路的上拉电阻，拉高SDA*/
    return i2c_wait_ack() == I2C_ACK; // 等待并读取ACK/NACK
}

/**
 * @brief  从I2C总线接收一个字节数据
 * @param  send_ack: 是否在接收后发送ACK (true发送ACK, false发送NACK)
 * @retval uint8_t: 接收到的字节
 */
uint8_t i2c_receive_byte(bool send_ack)
{
    uint8_t i;
    uint8_t received_data = 0;

    I2C_SDA_IN_MODE(); // 切换SDA为输入模式，准备接收

    for (i = 0; i < 8; i++)
    {
        received_data <<= 1; // 准备存储下一位

        I2C_SCL_L();     // 拉低SCL
        i2c_delay_us(5); // SCL低电平时间 (Tlow >= 4.7us)

        I2C_SCL_H();     // 拉高SCL，从设备准备发送数据
        i2c_delay_us(5); // SCL高电平时间 (Thigh >= 4.0us)

        if (I2C_SDA_READ())
        {                          // 在SCL高电平期间读取SDA状态
            received_data |= 0x01; // 当前位是1
        }
        // i2c_delay_us(1); // 数据保持时间 (Thd_dat >= 0ns, 但需要读取稳定)
    }

    // 发送ACK/NACK
    I2C_SDA_OUT_MODE(); // 切换SDA为输出模式以发送ACK/NACK

    if (send_ack)
    {
        I2C_SDA_L(); // 发送ACK (拉低SDA)
    }
    else
    {
        I2C_SDA_H(); // 发送NACK (释放SDA)
        // GPIO_SetBits(I2C_PORT, I2C_SDA_PIN); // SDA拉高 (当配置为OUT PP时)
    }
    i2c_delay_us(2); // ACK/NACK建立时间

    I2C_SCL_L();     // 拉低SCL，完成ACK/NACK时钟周期
    i2c_delay_us(5); // SCL低电平时间

    I2C_SDA_H();     // 释放SDA (切换回输入模式)
    i2c_delay_us(2); // 延时等待模式切换

    return received_data;
}

/**
 * @brief  等待并读取从设备的ACK/NACK信号
 *         主设备在发送8位数据后释放SDA，将SCL拉高，读取SDA状态。
 * @retval I2C_AckStatus_t: 返回ACK或NACK状态
 */
I2C_AckStatus_t i2c_wait_ack(void)
{
    I2C_SDA_IN_MODE(); // 切换SDA为输入模式，准备读取ACK/NACK

    I2C_SCL_L();     // 确保SCL低电平
    i2c_delay_us(5); // SCL低电平时间

    I2C_SCL_H();     // 拉高SCL，从设备发送ACK/NACK
    i2c_delay_us(5); // SCL高电平时间

    uint8_t ack_val = I2C_SDA_READ(); // 读取SDA状态 (0=ACK, 1=NACK)
    i2c_delay_us(2);                  // 读取保持时间

    I2C_SCL_L();     // 拉低SCL，完成ACK/NACK时钟周期
    i2c_delay_us(5); // SCL低电平时间

    // I2C_SDA_H(); // 释放SDA (已经在输入模式，此步可选，但确保下一操作前SDA高)
    // i2c_delay_us(2); // 延时

    return ack_val == 0 ? I2C_ACK : I2C_NACK;
}

// --- 示例用法函数实现 ---

/**
 * @brief  I2C写一个字节到指定从设备和地址
 * @param  slave_addr: 从设备地址 (7位地址)
 * @param  reg_addr: 寄存器地址
 * @param  data: 要写入的数据
 * @retval bool: 返回true表示写入成功，false表示失败 (例如：无ACK)
 */
bool i2c_master_write_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
    bool success = false;

    i2c_start();
    // 发送设备地址+写操作 (slave_addr << 1) | 0x00
    if (i2c_send_byte((slave_addr << 1) | 0x00))
    {
        // 发送寄存器地址
        if (i2c_send_byte(reg_addr))
        {
            // 发送数据
            if (i2c_send_byte(data))
            {
                success = true; // 所有步骤都收到了ACK
            }
        }
    }
    i2c_stop();

    // 某些从设备需要等待写入周期完成 (例如 EEPROM)
    // 可以通过反复发送带有写位的从设备地址来轮询ACK
    if (success)
    {
        i2c_delay_us(5000); // 简单的等待，更安全的方式是轮询ACK
                            /*
                            // 轮询ACK示例 (更可靠)
                            uint32_t timeout = 10000; // 设置一个超时计数器
                            while(timeout--) {
                                i2c_start();
                                if (i2c_send_byte((slave_addr << 1) | 0x00)) {
                                    i2c_stop();
                                    break; // 收到ACK，从设备准备好了
                                }
                                i2c_stop(); // 发送Stop后重试
                                i2c_delay_us(100); // 等待一下再重试
                                if (timeout == 0) {
                                    success = false; // 超时
                                    break;
                                }
                            }
                            */
    }

    return success;
}

/**
 * @brief  I2C从指定从设备和地址读取一个字节 (随机读取)
 * @param  slave_addr: 从设备地址 (7位地址)
 * @param  reg_addr: 寄存器地址
 * @param  data: 指向存储读取数据的变量的指针
 * @retval bool: 返回true表示读取成功，false表示失败 (例如：无ACK)
 */
bool i2c_master_read_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data)
{
    bool success = false;

    i2c_start();
    // 1. 发送设备地址+写操作 (用于设置内部寄存器地址)
    if (i2c_send_byte((slave_addr << 1) | 0x00))
    {
        // 2. 发送要读取的寄存器地址
        if (i2c_send_byte(reg_addr))
        {
            // 3. 发送重复起始信号
            i2c_start(); // Repeated Start

            // 4. 发送设备地址+读操作
            if (i2c_send_byte((slave_addr << 1) | 0x01))
            {
                // 5. 接收数据并发送NACK (表示这是最后一个要读取的字节)
                *data = i2c_receive_byte(false); // false = 发送NACK
                success = true;
            }
        }
    }
    i2c_stop();

    return success;
}