/**
 ******************************************************************************
 * @file    rc_process.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/07/21
 * @brief   ң�������ݴ���
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "rc_process.h"
#include "com_data.h"
#include "pro_common.h"
#include "ProConfig.h"

/** @addtogroup Drone_F427
 * @{
 */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  ң�����ݴ���
 * @note
 * @param  rc_data: ң��������
 * @retval ��
 */
void RC_Process(void)
{
    /*�޷�*/
    rc_state.raw.channel1 = LIMIT(rc_state.raw.channel1, 1000, 2000);
    rc_state.raw.channel2 = LIMIT(rc_state.raw.channel2, 1000, 2000);
    rc_state.raw.channel3 = LIMIT(rc_state.raw.channel3, 1000, 2000);
    rc_state.raw.channel4 = LIMIT(rc_state.raw.channel4, 1000, 2000);
    /*��һ��ӳ��*/
    rc_state.norm.rc_roll = (((float)(rc_state.raw.channel1 - RC_MID)) / (RC_MAX - RC_MIN)) * 2.0f;     /*ӳ��ͨ��1*/
    rc_state.norm.rc_pitch = (((float)(rc_state.raw.channel2 - RC_MID)) / (RC_MAX - RC_MIN)) * 2.0f;    /*ӳ��ͨ��2*/
    rc_state.norm.rc_throttle = (((float)(rc_state.raw.channel3 - RC_MID)) / (RC_MAX - RC_MIN)) * 2.0f; /*ӳ��ͨ��3*/
    rc_state.norm.rc_yaw = (((float)(rc_state.raw.channel4 - RC_MID)) / (RC_MAX - RC_MIN)) * 2.0f;      /*ӳ��ͨ��4*/
    /*�޷�*/
    rc_state.norm.rc_roll = LIMIT(rc_state.norm.rc_roll, -1.0f, 1.0f);         /*ͨ��1*/
    rc_state.norm.rc_pitch = LIMIT(rc_state.norm.rc_pitch, -1.0f, 1.0f);       /*ͨ��2*/
    rc_state.norm.rc_throttle = LIMIT(rc_state.norm.rc_throttle, -1.0f, 1.0f); /*ͨ��3*/
    rc_state.norm.rc_yaw = LIMIT(rc_state.norm.rc_yaw, -1.0f, 1.0f);           /*ͨ��4*/
    /*����ӳ��*/
    /*����A*/
    if (rc_state.raw.channel5 > SWITCH_MIN - 20 && rc_state.raw.channel5 < SWITCH_MIN + 20)
        rc_state.norm.switch_A = 0;
    else if (rc_state.raw.channel5 > SWITCH_MID - 20 && rc_state.raw.channel5 < SWITCH_MID + 20)
        rc_state.norm.switch_A = 1;
    else if (rc_state.raw.channel5 > SWITCH_MAX - 20 && rc_state.raw.channel5 < SWITCH_MAX + 20)
        rc_state.norm.switch_A = 2;
    /*����B*/
    if (rc_state.raw.channel6 > SWITCH_MIN - 20 && rc_state.raw.channel6 < SWITCH_MIN + 20)
        rc_state.norm.switch_B = 0;
    else if (rc_state.raw.channel6 > SWITCH_MID - 20 && rc_state.raw.channel6 < SWITCH_MID + 20)
        rc_state.norm.switch_B = 1;
    else if (rc_state.raw.channel6 > SWITCH_MAX - 20 && rc_state.raw.channel6 < SWITCH_MAX + 20)
        rc_state.norm.switch_B = 2;
    /*����C*/
    if (rc_state.raw.channel7 > SWITCH_MIN - 20 && rc_state.raw.channel7 < SWITCH_MIN + 20)
        rc_state.norm.switch_C = 0;
    else if (rc_state.raw.channel7 > SWITCH_MID - 20 && rc_state.raw.channel7 < SWITCH_MID + 20)
        rc_state.norm.switch_C = 1;
    else if (rc_state.raw.channel7 > SWITCH_MAX - 20 && rc_state.raw.channel7 < SWITCH_MAX + 20)
        rc_state.norm.switch_C = 2;
    /*����D*/
    if (rc_state.raw.channel8 > SWITCH_MIN - 20 && rc_state.raw.channel8 < SWITCH_MIN + 20)
        rc_state.norm.switch_D = 0;
    else if (rc_state.raw.channel8 > SWITCH_MID - 20 && rc_state.raw.channel8 < SWITCH_MID + 20)
        rc_state.norm.switch_D = 1;
    else if (rc_state.raw.channel8 > SWITCH_MAX - 20 && rc_state.raw.channel8 < SWITCH_MAX + 20)
        rc_state.norm.switch_D = 2;
    /*����E*/
    if (rc_state.raw.channel9 > SWITCH_MIN - 20 && rc_state.raw.channel9 < SWITCH_MIN + 20)
        rc_state.norm.switch_E = 0;
    else if (rc_state.raw.channel9 > SWITCH_MID - 20 && rc_state.raw.channel9 < SWITCH_MID + 20)
        rc_state.norm.switch_E = 1;
    else if (rc_state.raw.channel9 > SWITCH_MAX - 20 && rc_state.raw.channel9 < SWITCH_MAX + 20)
        rc_state.norm.switch_E = 2;
}

/**
 * @}
 */
