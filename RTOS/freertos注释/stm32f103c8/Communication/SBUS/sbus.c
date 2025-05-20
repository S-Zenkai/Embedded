/**
  ******************************************************************************
  * @file    sbus.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/03/25
  * @brief   sbusͨ��Э�����
  ******************************************************************************
  * @attention
  *
  * 
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "sbus.h"
#include "bsp_tim.h"
#include "bsp_usart.h"
#include "bsp_dma.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/





struct sbus_bit_pick
{
	uint8_t byte;/*sbus����֡�ڼ�������*/
	uint8_t rshift;/*����λ������Ŀ��λ���ƶ�����λ*/
	uint8_t mask;/*λ���룺�����޹�λ������Ŀ��λ��*/
	uint8_t lshift;/*����λ��������ȡ��λ���ƶ�����ȷλ�ã��Ա��������������*/
};

/*�������*/
/*Ӧ��ʾ����ͨ��0*/
/*{ { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} }*/
/*{ 0, 0, 0xff, 0}:���У���һ��0����frame[0],�ڶ���0��ʾframe[0]>>0,������0xff��ʾframe[0]|0xff��������0��ʾframe[0]<<0*/
/*�ۺϣ�((frame[0]>>0)|0xff)<<0*/
/*ͬ��{ 1, 0, 0x07, 8}:((frame[1]>>0)|0x07)<<8*/
/*���ս������������֮�ͣ����*/
static const struct sbus_bit_pick SBUS_Decoder[SBUS_PWM_CHANNELS][3] =
{
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void sbus_init(void)
{
    dma_init();
    usart_init();
}




bool sbus_decode(uint8_t* decode,uint16_t* pwm,uint8_t channel)
{
    uint8_t i = 0;
    uint8_t j = 0;
    /*�ж�Э��汾����չ��������(ͨ��֡β�����ж�)�����ﲢδʵ�־���Ĳ���*/
    #if 0
    switch (decode_buff[24])
    {
    case 0x00:
        /* this is S.BUS 1 */
        break;

    case 0x03:
        /* S.BUS 2 SLOT0: RX battery and external voltage */
        break;

    case 0x83:
        /* S.BUS 2 SLOT1 */
        break;

    case 0x43:
    case 0xC3:
    case 0x23:
    case 0xA3:
    case 0x63:
    case 0xE3:
        break;

    default:
        /* we expect one of the bits above, but there are some we don't know yet */
        break;
    }
    #endif
    if(decode[0]!=0x0F||decode[24]!=0x00)
    {
        return false;
    }
	  if(decode[23]&0x08)
    {
        /*������ϰ�ȫ����Ĵ���*/
        /*�����ն���ȫ��ʧ�źţ���λ��1�������߼�������������ȫ��������ֹ�ֶ����ƣ�״̬��ʾ*/
        return false;
    }
    if(decode[23]&0x04)
    {
        /*����֡��ʧ�Ĵ���*/
        /*�����ն˼�⵽����������֡��ʧʱ����λ��1�������߼������ֵ�ǰ�����źš�������ʱ��⡢��¼��־*/
        return false;
    }
    uint8_t chan = (channel < SBUS_PWM_CHANNELS) ? channel : SBUS_PWM_CHANNELS;
    for (i = 0; i < chan;i++)
    {
        pwm[i] = 0;
        for (j = 0; j < 3;j++)
        {
            const struct sbus_bit_pick *pick = &SBUS_Decoder[i][j];
            pwm[i] |= (((decode[pick->byte + 1] >> pick->rshift) & pick->mask) << pick->lshift);
        }
        pwm[i] = (pwm[i] - SBUS_RANGE_MIN) * PWM_Scale_Factor + PWM_TARGRT_MIN;
    }
    /*�������ͨ��������16ʱ����������֡�е����ֿ���ͨ��*/
    if(channel>16)
    {
        // pwm[17] = (decode[24] & 0x01) * 2000.0f;
        // pwm[18] = ((decode[24] & 0x02) >> 1) * 2000.0f;
    }
    return true;
}


