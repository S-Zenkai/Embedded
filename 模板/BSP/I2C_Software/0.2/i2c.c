#include "i2c.h"

// --- ˽�к������� ---
static void i2c_delay_us(uint32_t us); // ΢����ʱ����
static void I2C_SCL_H(void);           // SCL���� (�ͷ�)
static void I2C_SCL_L(void);           // SCL���� (����)
static void I2C_SDA_H(void);           // SDA���� (�ͷţ�ģ�⿪©)
static void I2C_SDA_L(void);           // SDA���� (������ģ�⿪©)
static uint8_t I2C_SDA_READ(void);     // ��ȡSDA״̬
static void I2C_SDA_OUT_MODE(void);    // ����SDAΪ���ģʽ
static void I2C_SDA_IN_MODE(void);     // ����SDAΪ����ģʽ

// --- ˽�к���ʵ�� ---

// �򵥵�΢����ʱ (��Ҫ����ʵ��CPUƵ�ʺ��Ż��ȼ�����У׼)
// ����һ���ǳ��ֲڵ���ʱ��ʵ��Ӧ���Ƽ�ʹ�� SysTick ��Ӳ����ʱ��
static void i2c_delay_us(uint32_t us)
{
    volatile uint32_t i;
    // ����ѭ�������ǻ���STM32F427VI�Ĺ��㣬�ڲ�ͬƽ̨���Ż��ȼ��²���ܴ�
    // ��Լÿ100��ѭ������1΢�� (���磬��Ƶ180MHzʱ)
    uint32_t loops = us * 100;
    for (i = 0; i < loops; i++)
        ;
}

// ����SCL����
static void I2C_SCL_H(void)
{
    GPIO_SetBits(I2C_PORT, I2C_SCL_PIN);
    i2c_delay_us(2); // ��ʱ�ȴ�����״̬�ȶ�
}

static void I2C_SCL_L(void)
{
    GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN);
    i2c_delay_us(2); // ��ʱ�ȴ�����״̬�ȶ�
}

// ����SDA���ţ�ģ�⿪©���
// I2C_SDA_H(): ��SDA����Ϊ����ģʽ�����ⲿ������������
static void I2C_SDA_H(void)
{
    GPIO_SetBits(I2C_PORT, I2C_SDA_PIN);
    i2c_delay_us(2); // ��ʱ�ȴ�����״̬�ȶ�
}

// I2C_SDA_L(): ��SDA����Ϊ���ģʽ������
static void I2C_SDA_L(void)
{
    GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN); // ����SDA
    i2c_delay_us(2);                       // ��ʱȷ������״̬�ȶ�
}

// ��ȡSDA����״̬ (������SDAΪ����ģʽʱ����)
static uint8_t I2C_SDA_READ(void)
{
    return GPIO_ReadInputDataBit(I2C_PORT, I2C_SDA_PIN);
}

// ����SDAΪ���ģʽ
static void I2C_SDA_OUT_MODE(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;      // ���ģʽ
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // ������� (������������)
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // ���������������ⲿ
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // ����
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

// ����SDAΪ����ģʽ
static void I2C_SDA_IN_MODE(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;       // ����ģʽ
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // �������룬�����ⲿ����
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // ����
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);
}

// --- ��������ʵ�� ---

/**
 * @brief  ��ʼ��ģ��I2C GPIO����
 */
void i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // ʹ��GPIOBʱ��
    RCC_AHB1PeriphClockCmd(I2C_CLK, ENABLE);

    // ����SCL���� (PB8)
    GPIO_InitStruct.GPIO_Pin = I2C_SCL_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;      // ���ģʽ
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     // �������
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;   // ��������
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // ����
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);

    // ����SDA���� (PB9) - ��ʼ����Ϊ���룬���ⲿ��������
    GPIO_InitStruct.GPIO_Pin = I2C_SDA_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;      // ����ģʽ
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;  // �������룬�����ⲿ����
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // ����
    GPIO_Init(I2C_PORT, &GPIO_InitStruct);

    // ȷ��I2C���߿���״̬ (SCL=��, SDA=��)
    I2C_SCL_H();
    I2C_SDA_H(); // ȷ��SDA������ģʽ

    // ��ѡ��ִ�����߻ָ����У��Է����豸��ס����
    // ���SDA�����ͣ����Է���9��ʱ������
    /*��iicЭ���У������豸��ס���ߣ�����ʱ���豸����ʱ�ӣ��ڵھŸ�ʱ��ʱ�����ݷ��Ͷ˻��ͷ�sda����Ȩ�������ݽ��ն˿���sda*/
    // if (I2C_SDA_READ() == 0)
    // {
    //     uint8_t i;
    //     for (i = 0; i < 9; i++)
    //     {
    //         I2C_SCL_L();
    //         i2c_delay_us(10); // ȷ��ʱ�ӵ͵�ƽʱ��
    //         I2C_SCL_H();
    //         i2c_delay_us(10); // ȷ��ʱ�Ӹߵ�ƽʱ��
    //         if (I2C_SDA_READ() == 1)
    //             break; // SDA�ͷţ����߻ָ�
    //     }
    //     // ���Է���һ��ֹͣ�ź�
    //     i2c_start(); // ��Ҫһ��Start���ܷ���Stop
    //     i2c_stop();
    // }
}

/**
 * @brief  ����I2C������ʼ�ź�
 *         ������SCLΪ��ʱ��SDA�ɸ߱��
 */
void i2c_start(void)
{
    I2C_SDA_H();     // ȷ��SDA�ڸߵ�ƽ (����ģʽ)
    I2C_SCL_H();     // ȷ��SCL�ڸߵ�ƽ
    /*�������ʱ������û�б�Ҫ��*/
    //i2c_delay_us(5); // ��ʼ�źŽ���ʱ�� (Ts_sta >= 4.7us for Standard-mode)

    I2C_SDA_L();     // SDA�ɸ߱��
    i2c_delay_us(5); // ��ʼ�źű���ʱ�� (Thd_sta >= 4.0us for Standard-mode)

    I2C_SCL_L();     // ����SCL��׼����������
    /*�������ʱ������û�б�Ҫ��*/
    //i2c_delay_us(2); // ��ʱȷ��SCL�ѵ�
}

/**
 * @brief  ����I2C����ֹͣ�ź�
 *         ������SCLΪ��ʱ��SDA�ɵͱ��
 */
void i2c_stop(void)
{
    I2C_SDA_L();     // ȷ��SDA�ڵ͵�ƽ (���ģʽ)
    // I2C_SCL_L();     // ȷ��SCL�ڵ͵�ƽ
    // i2c_delay_us(2); // ��ʱȷ��SCL�ѵ�

    I2C_SCL_H();     // ����SCL
    i2c_delay_us(5); // ֹͣ�źŽ���ʱ�� (Ts_sto >= 4.0us for Standard-mode)

    I2C_SDA_H();     // SDA�ɵͱ�� (�л�Ϊ����ģʽ)
    i2c_delay_us(5); // ���߿���ʱ�� (Tbuf >= 4.7us for Standard-mode)
}

/**
 * @brief  ����һ���ֽ����ݵ�I2C����
 * @param  byte: Ҫ���͵��ֽ� (MSB first)
 * @retval bool: ����true��ʾ�յ�ACK��false��ʾ�յ�NACK
 */
bool i2c_send_byte(uint8_t byte)
{
    uint8_t i;

    I2C_SDA_OUT_MODE(); // �л�SDAΪ���ģʽ

    for (i = 0; i < 8; i++)
    {
        I2C_SCL_L();     // ����SCL������SDA�仯
        i2c_delay_us(5); // SCL�͵�ƽʱ�� (Tlow >= 4.7us)

        if ((byte & 0x80) >> 7)
        { // �жϵ�ǰλ (MSB)
            I2C_SDA_H(); // ����1 (�ͷ�SDA) - ������Ӧ���л�ģʽ��������SDAһֱ�����PP��ͨ��Set/Reset����
            // GPIO_SetBits(I2C_PORT, I2C_SDA_PIN); // SDA����
        }
        else
        {
            I2C_SDA_L(); // ����0 (����SDA��)
            // GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN); // SDA����
        }
        i2c_delay_us(2); // ���ݽ���ʱ�� (Tsu_dat >= 250ns)

        I2C_SCL_H();     // ����SCL�����豸��ȡ����
        i2c_delay_us(5); // SCL�ߵ�ƽʱ�� (Thigh >= 4.0us)

        byte <<= 1; // ׼����һλ
    }

    // �ͷ�SDA���ȴ����豸����ACK/NACK
    // I2C_SDA_H(); // This is crucial for ACK/NACK - switch back to input mode
    /*����ͨ����������ģʽ��ͨ���ⲿ��·���������裬����SDA*/
    return i2c_wait_ack() == I2C_ACK; // �ȴ�����ȡACK/NACK
}

/**
 * @brief  ��I2C���߽���һ���ֽ�����
 * @param  send_ack: �Ƿ��ڽ��պ���ACK (true����ACK, false����NACK)
 * @retval uint8_t: ���յ����ֽ�
 */
uint8_t i2c_receive_byte(bool send_ack)
{
    uint8_t i;
    uint8_t received_data = 0;

    I2C_SDA_IN_MODE(); // �л�SDAΪ����ģʽ��׼������

    for (i = 0; i < 8; i++)
    {
        received_data <<= 1; // ׼���洢��һλ

        I2C_SCL_L();     // ����SCL
        i2c_delay_us(5); // SCL�͵�ƽʱ�� (Tlow >= 4.7us)

        I2C_SCL_H();     // ����SCL�����豸׼����������
        i2c_delay_us(5); // SCL�ߵ�ƽʱ�� (Thigh >= 4.0us)

        if (I2C_SDA_READ())
        {                          // ��SCL�ߵ�ƽ�ڼ��ȡSDA״̬
            received_data |= 0x01; // ��ǰλ��1
        }
        // i2c_delay_us(1); // ���ݱ���ʱ�� (Thd_dat >= 0ns, ����Ҫ��ȡ�ȶ�)
    }

    // ����ACK/NACK
    I2C_SDA_OUT_MODE(); // �л�SDAΪ���ģʽ�Է���ACK/NACK

    if (send_ack)
    {
        I2C_SDA_L(); // ����ACK (����SDA)
    }
    else
    {
        I2C_SDA_H(); // ����NACK (�ͷ�SDA)
        // GPIO_SetBits(I2C_PORT, I2C_SDA_PIN); // SDA���� (������ΪOUT PPʱ)
    }
    i2c_delay_us(2); // ACK/NACK����ʱ��

    I2C_SCL_L();     // ����SCL�����ACK/NACKʱ������
    i2c_delay_us(5); // SCL�͵�ƽʱ��

    I2C_SDA_H();     // �ͷ�SDA (�л�������ģʽ)
    i2c_delay_us(2); // ��ʱ�ȴ�ģʽ�л�

    return received_data;
}

/**
 * @brief  �ȴ�����ȡ���豸��ACK/NACK�ź�
 *         ���豸�ڷ���8λ���ݺ��ͷ�SDA����SCL���ߣ���ȡSDA״̬��
 * @retval I2C_AckStatus_t: ����ACK��NACK״̬
 */
I2C_AckStatus_t i2c_wait_ack(void)
{
    I2C_SDA_IN_MODE(); // �л�SDAΪ����ģʽ��׼����ȡACK/NACK

    I2C_SCL_L();     // ȷ��SCL�͵�ƽ
    i2c_delay_us(5); // SCL�͵�ƽʱ��

    I2C_SCL_H();     // ����SCL�����豸����ACK/NACK
    i2c_delay_us(5); // SCL�ߵ�ƽʱ��

    uint8_t ack_val = I2C_SDA_READ(); // ��ȡSDA״̬ (0=ACK, 1=NACK)
    i2c_delay_us(2);                  // ��ȡ����ʱ��

    I2C_SCL_L();     // ����SCL�����ACK/NACKʱ������
    i2c_delay_us(5); // SCL�͵�ƽʱ��

    // I2C_SDA_H(); // �ͷ�SDA (�Ѿ�������ģʽ���˲���ѡ����ȷ����һ����ǰSDA��)
    // i2c_delay_us(2); // ��ʱ

    return ack_val == 0 ? I2C_ACK : I2C_NACK;
}

// --- ʾ���÷�����ʵ�� ---

/**
 * @brief  I2Cдһ���ֽڵ�ָ�����豸�͵�ַ
 * @param  slave_addr: ���豸��ַ (7λ��ַ)
 * @param  reg_addr: �Ĵ�����ַ
 * @param  data: Ҫд�������
 * @retval bool: ����true��ʾд��ɹ���false��ʾʧ�� (���磺��ACK)
 */
bool i2c_master_write_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
    bool success = false;

    i2c_start();
    // �����豸��ַ+д���� (slave_addr << 1) | 0x00
    if (i2c_send_byte((slave_addr << 1) | 0x00))
    {
        // ���ͼĴ�����ַ
        if (i2c_send_byte(reg_addr))
        {
            // ��������
            if (i2c_send_byte(data))
            {
                success = true; // ���в��趼�յ���ACK
            }
        }
    }
    i2c_stop();

    // ĳЩ���豸��Ҫ�ȴ�д��������� (���� EEPROM)
    // ����ͨ���������ʹ���дλ�Ĵ��豸��ַ����ѯACK
    if (success)
    {
        i2c_delay_us(5000); // �򵥵ĵȴ�������ȫ�ķ�ʽ����ѯACK
                            /*
                            // ��ѯACKʾ�� (���ɿ�)
                            uint32_t timeout = 10000; // ����һ����ʱ������
                            while(timeout--) {
                                i2c_start();
                                if (i2c_send_byte((slave_addr << 1) | 0x00)) {
                                    i2c_stop();
                                    break; // �յ�ACK�����豸׼������
                                }
                                i2c_stop(); // ����Stop������
                                i2c_delay_us(100); // �ȴ�һ��������
                                if (timeout == 0) {
                                    success = false; // ��ʱ
                                    break;
                                }
                            }
                            */
    }

    return success;
}

/**
 * @brief  I2C��ָ�����豸�͵�ַ��ȡһ���ֽ� (�����ȡ)
 * @param  slave_addr: ���豸��ַ (7λ��ַ)
 * @param  reg_addr: �Ĵ�����ַ
 * @param  data: ָ��洢��ȡ���ݵı�����ָ��
 * @retval bool: ����true��ʾ��ȡ�ɹ���false��ʾʧ�� (���磺��ACK)
 */
bool i2c_master_read_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data)
{
    bool success = false;

    i2c_start();
    // 1. �����豸��ַ+д���� (���������ڲ��Ĵ�����ַ)
    if (i2c_send_byte((slave_addr << 1) | 0x00))
    {
        // 2. ����Ҫ��ȡ�ļĴ�����ַ
        if (i2c_send_byte(reg_addr))
        {
            // 3. �����ظ���ʼ�ź�
            i2c_start(); // Repeated Start

            // 4. �����豸��ַ+������
            if (i2c_send_byte((slave_addr << 1) | 0x01))
            {
                // 5. �������ݲ�����NACK (��ʾ�������һ��Ҫ��ȡ���ֽ�)
                *data = i2c_receive_byte(false); // false = ����NACK
                success = true;
            }
        }
    }
    i2c_stop();

    return success;
}