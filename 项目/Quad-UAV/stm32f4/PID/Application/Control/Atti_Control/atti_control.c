/**
 ******************************************************************************
 * @file    atti_control.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/07/17
 * @brief   ��̬�����㷨ʵ��
 ******************************************************************************
 * @attention
 *
 * ���ļ�ʵ���˴���PID��̬��������
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "atti_control.h"
#include "ProConfig.h"
#include "com_data.h"
#include "pro_common.h"

/** @addtogroup Drone_F427
 * @{
 */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*��̬��PID������������״̬*/
PID_Controller_t angle_pid[AXIS_COUNT];
/*���ٶ�PID������������״̬*/
PID_Controller_t ang_vel_pid[AXIS_COUNT];



/*��ʼ����ͨ��ָ����ʼ����.*/
atti_control_ctx_t atti_control_ctx = {
    .flight_control_state = &flight_control_state,
    .angle_pid = angle_pid,
    .ang_vel_pid = ang_vel_pid,
    .rc_state = &rc_state,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  ���Ų�������
  * @note   
  * @param  ��
  * @retval ��
  */
void throttle_compensation(atti_control_ctx_t *atti_control_ctx)
{
    Axis_State_t *_axis_state = atti_control_ctx->flight_control_state->control_state;
    Flight_Control_State_t *_flight_state = atti_control_ctx->flight_control_state;
    /*���Ų���(Ӧ��pid����ǰ���У��ɷ��ڱ�ĵط��Է��ϵ�һְ��ԭ��)*/
    /*1.��ʧϵ��*/
    float throttle_theta = fabs(cos(_axis_state[ROLL].angle) * cos(_axis_state[PITCH].angle));
    /*2.�޷�,0.7�൱�ڵ�����б45�ȣ�����б�Ƕȹ���ʱ���ᵼ�����Ź���*/
    throttle_theta = throttle_theta < 0.7 ? 0.7 : throttle_theta;
    /*3.���Ų���*/
    _flight_state->throttle = _flight_state->throttle * ((1 / throttle_theta - 1) * 0.5 + 1);
    /*4.�޷�*/
    _flight_state->throttle = _flight_state->throttle < 0 ? 0 : _flight_state->throttle;
    _flight_state->throttle = _flight_state->throttle > THROTTLE_MAX ? THROTTLE_MAX : _flight_state->throttle;
}

/**
  * @brief  ��̬�ǿ��ƺ�����P���ƣ��⻷������roll��pitch
  * @note   
  * @param  axis: �ᣬROLL��PITCH
  * @param  atti_control_ctx: ��̬����������
  * @retval ��
  */
void angle_control(Axis_e axis, atti_control_ctx_t *atti_control_ctx)
{
    float error;
    Flight_Control_State_t *_flight_state = atti_control_ctx->flight_control_state;
    Axis_State_t *_axis_state = atti_control_ctx->flight_control_state->control_state;
    PID_Controller_t *_angle_pid = atti_control_ctx->angle_pid;
    rc_state_t *_rc_state = atti_control_ctx->rc_state;
    /*PID��*/
    float p_item;
    /*������*/
    /*����ֵ*/
    /*ʹ��switch���*/
    switch (axis)
    {
        case ROLL:
            _axis_state[axis].angle_target = _rc_state->norm.rc_roll * ROLL_ANGLE_TARGET_MAX * ANGLE_TO_RADIAN;
            break;
        case PITCH:
            _axis_state[axis].angle_target = _rc_state->norm.rc_pitch * PITCH_ANGLE_TARGET_MAX * ANGLE_TO_RADIAN;
            break;
        case YAW:
            return;
        default:
            return;
    }
    /*���*/
    error = _axis_state[axis].angle_target - _axis_state[axis].angle; /*ע�⣬���ﻹ�迼������ת��*/

    /*P*/
    p_item = _angle_pid[axis].Kp * error;
    /*�����޷�*/
    p_item = LIMIT(p_item, -ANGLE_P_OUTPUT_MAX, ANGLE_P_OUTPUT_MAX);

    /*�����Ϊ���ٶȻ�����ֵ*/
    _axis_state[axis].rate_target = p_item;
}

/**
 * @brief  ���ٶȿ���,����pid���������������ڻ�
 * @note    ÿ�μ���pidǰ����Ҫ�������Ų���
 * @param  ��
 * @retval ��
 */
void ang_vel_control(Axis_e axis, atti_control_ctx_t *atti_control_ctx)
{
    float error;
    Flight_Control_State_t *_flight_state = atti_control_ctx->flight_control_state;
    Axis_State_t *_axis_state = atti_control_ctx->flight_control_state->control_state;
    PID_Controller_t *_ang_vel_pid = atti_control_ctx->ang_vel_pid;
    rc_state_t *_rc_state = atti_control_ctx->rc_state;
    /*PID��*/
    float p_item,d_item;
    /*������*/
    /*����ֵ*/
    if (axis == YAW)
    {
        _axis_state[axis].rate_target = _rc_state->norm.rc_yaw * YAW_RATE_SENSITIVITY; /*����ʵ����ֵ�����滹Ҫ*0.005��ƥ�仡����*/
    }
    /*���*/
    error = _axis_state[axis].rate_target * 0.005f - _axis_state[axis].rate; /*ע�⣬���ﻹ�迼������ת��*/

    /*P*/
    p_item = _ang_vel_pid[axis].Kp * error;
    
    /*I*/
    /*�ɻ�δ���ʱ��ջ��֣���ֹ���ʱ���ֹ���*/
    if (_flight_state->throttle < THROTTLE_MIN)
    {
        _ang_vel_pid[axis].i_item = 0;/*����᲻���ڷ��й����б����㣿����ֻ�ڷɻ�δ����ʱ����*/
    }
    else
    {
        _ang_vel_pid[axis].i_item += _ang_vel_pid[axis].Ki * error * _ang_vel_pid[axis].sample_time;
        /*�����޷�*/
        _ang_vel_pid[axis].i_item = LIMIT(_ang_vel_pid[axis].i_item, -ANG_VEL_INTEGRAL_MAX, ANG_VEL_INTEGRAL_MAX);
    }
    
    /*D��΢�����У����ڲ���ֵ*/
    d_item = -_ang_vel_pid[axis].Kd * (_axis_state[axis].rate - _ang_vel_pid[axis].last_measurement) / _ang_vel_pid[axis].sample_time;
    _ang_vel_pid[axis].last_measurement = _axis_state[axis].rate;

    /*PID���*/
    _ang_vel_pid[axis].output = p_item + _ang_vel_pid[axis].i_item + d_item;
    /*����޷�*/
}

/**
  * @brief  ��̬���Ƴ�ʼ��
  * @note   
  * @param  ��
  * @retval ��
  */
void atti_control_init(void)
{
    /*��ʼ��PID������*/
    angle_pid[ROLL].Kp = ANGLE_ROLL_KP;
    angle_pid[PITCH].Kp = ANGLE_PITCH_KP;
    angle_pid[YAW].Kp = ANGLE_YAW_KP;

    ang_vel_pid[ROLL].Kp = RATE_ROLL_KP;
    ang_vel_pid[PITCH].Kp = RATE_PITCH_KP;
    ang_vel_pid[YAW].Kp = RATE_YAW_KP;

    ang_vel_pid[ROLL].Ki = RATE_ROLL_KI;
    ang_vel_pid[PITCH].Ki = RATE_PITCH_KI;
    ang_vel_pid[YAW].Ki = RATE_YAW_KI;

    ang_vel_pid[ROLL].Kd = RATE_ROLL_KD;
    ang_vel_pid[PITCH].Kd = RATE_PITCH_KD;
    ang_vel_pid[YAW].Kd = RATE_YAW_KD;

    /*΢������Ҫ��ʼ����һ�β���ֵ*/
    ang_vel_pid[ROLL].last_measurement = flight_control_state.control_state[ROLL].rate;
    ang_vel_pid[PITCH].last_measurement = flight_control_state.control_state[PITCH].rate;
    ang_vel_pid[YAW].last_measurement = flight_control_state.control_state[YAW].rate;

    /*��������Ҫ����*/
    ang_vel_pid[ROLL].i_item = 0.0f;
    ang_vel_pid[PITCH].i_item = 0.0f;
    ang_vel_pid[YAW].i_item = 0.0f;

    /*����ʱ��*/
    ang_vel_pid[ROLL].sample_time = ANG_VEL_SAMPLE_TIME;/*500Hz*/
    ang_vel_pid[PITCH].sample_time = ANG_VEL_SAMPLE_TIME;
    ang_vel_pid[YAW].sample_time = ANG_VEL_SAMPLE_TIME;
}

/**
  * @brief  ���Ժ���
  * @note   
  * @param  ��
  * @retval ��
  */
void atti_control_update(void)
{

    angle_control(ROLL, &atti_control_ctx);
    angle_control(PITCH, &atti_control_ctx);

    ang_vel_control(ROLL, &atti_control_ctx);
    ang_vel_control(PITCH, &atti_control_ctx);
    ang_vel_control(YAW, &atti_control_ctx);
}

/**
 * @}
 */
