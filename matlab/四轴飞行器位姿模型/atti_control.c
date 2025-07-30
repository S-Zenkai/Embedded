/**
 ******************************************************************************
 * @file    atti_control.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/07/17
 * @brief   姿态控制算法实现
 ******************************************************************************
 * @attention
 *
 * 本文件实现了串级PID姿态控制器。
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "atti_control.h"
#include "simstruc.h"

/** @addtogroup Drone_F427
 * @{
 */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*姿态角PID控制器参数与状态*/
PID_Controller_t angle_pid[AXIS_COUNT];
/*角速度PID控制器参数与状态*/
PID_Controller_t ang_vel_pid[AXIS_COUNT];



/*初始化，通过指定初始化器.*/
atti_control_ctx_t atti_control_ctx = {
    .flight_control_state = &flight_control_state,
    .angle_pid = &angle_pid[0],
    .ang_vel_pid = &ang_vel_pid[0],
    .rc_state = &rc_state,
};
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  油门补偿函数
  * @note   
  * @param  无
  * @retval 无
  */
void throttle_compensation(atti_control_ctx_t *atti_control_ctx)
{
    Axis_State_t *_axis_state = atti_control_ctx->flight_control_state->control_state;
    Flight_Control_State_t *_flight_state = atti_control_ctx->flight_control_state;
    /*油门补偿(应在pid计算前进行，可放在别的地方以符合单一职责原则)*/
    /*1.损失系数*/
    double throttle_theta = fabs(cos(_axis_state[ROLL].angle) * cos(_axis_state[PITCH].angle));
    /*2.限幅,0.7相当于单轴倾斜45度，当倾斜角度过大时，会导致油门过大*/
    throttle_theta = throttle_theta < 0.7 ? 0.7 : throttle_theta;
    /*3.油门补偿*/
    _flight_state->throttle = _flight_state->throttle * ((1 / throttle_theta - 1) * 0.5 + 1);
    /*4.限幅*/
    _flight_state->throttle = _flight_state->throttle < 0 ? 0 : _flight_state->throttle;
    _flight_state->throttle = _flight_state->throttle > THROTTLE_MAX ? THROTTLE_MAX : _flight_state->throttle;
}

/**
  * @brief  姿态角控制函数，P控制，外环，控制roll和pitch
  * @note   
  * @param  axis: 轴，ROLL或PITCH
  * @param  atti_control_ctx: 姿态控制上下文
  * @retval 无
  */
void angle_control(Axis_e axis, atti_control_ctx_t *atti_control_ctx)
{
    double error;
    Flight_Control_State_t *_flight_state = atti_control_ctx->flight_control_state;
    Axis_State_t *_axis_state = atti_control_ctx->flight_control_state->control_state;
    PID_Controller_t *_angle_pid = atti_control_ctx->angle_pid;
    rc_state_t *_rc_state = atti_control_ctx->rc_state;
    /*PID项*/
    double p_item;
    /*误差计算*/
    /*期望值*/
    /*使用switch语句*/
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
    /*误差*/
    error = _axis_state[axis].angle_target - _axis_state[axis].angle; /*注意，这里还需考虑坐标转换*/

    /*P*/
    p_item = _angle_pid[axis].Kp * error;
    /*比例限幅*/
    p_item = LIMIT(p_item, -ANGLE_P_OUTPUT_MAX, ANGLE_P_OUTPUT_MAX);

    /*输出作为角速度环期望值*/
    _axis_state[axis].rate_target = p_item;
}

/**
 * @brief  角速度控制,基于pid控制器，串级的内环
 * @note    每次计算pid前，需要进行油门补偿
 * @param  无
 * @retval 无
 */
void ang_vel_control(Axis_e axis, atti_control_ctx_t *atti_control_ctx)
{
    double error;
    Flight_Control_State_t *_flight_state = atti_control_ctx->flight_control_state;
    Axis_State_t *_axis_state = atti_control_ctx->flight_control_state->control_state;
    PID_Controller_t *_ang_vel_pid = atti_control_ctx->ang_vel_pid;
    rc_state_t *_rc_state = atti_control_ctx->rc_state;
    /*PID项*/
    double p_item,d_item;
    /*误差计算*/
    /*期望值*/
    // ssPrintf("rc_yaw=%f\n",_rc_state->norm.rc_yaw);
    if (axis == YAW)
    {
        _axis_state[axis].rate_target = _rc_state->norm.rc_yaw * YAW_RATE_SENSITIVITY; /*非真实期望值，下面还要*0.005以匹配弧度制*/
    }
    // ssPrintf(" rate_target=%f\n", _axis_state[axis].rate_target);
    /*误差*/
    error = _axis_state[axis].rate_target * 0.005 - _axis_state[axis].rate; /*注意，这里还需考虑坐标转换*/
    // ssPrintf("error=%f\n",error);
    /*P*/
    p_item = _ang_vel_pid[axis].Kp * error;
    // ssPrintf("Kp=%f\n",_ang_vel_pid[axis].Kp);
    // ssPrintf("p_item=%f\n",p_item);
    /*I*/
    /*飞机未起飞时清空积分，防止起飞时积分过大*/
    if (_flight_state->throttle < THROTTLE_MIN)
    {
        _ang_vel_pid[axis].i_item = 0.0;/*这里会不会在飞行过程中被清零？可以只在飞机未解锁时清零*/
    }
    else
    {
        _ang_vel_pid[axis].i_item += _ang_vel_pid[axis].Ki * error * _ang_vel_pid[axis].sample_time;
        /*积分限幅*/
        _ang_vel_pid[axis].i_item = LIMIT(_ang_vel_pid[axis].i_item, -ANG_VEL_INTEGRAL_MAX, ANG_VEL_INTEGRAL_MAX);
    }
    // ssPrintf("i_item=%f\n",_ang_vel_pid[axis].i_item);
    /*D，微分先行，基于测量值*/
    d_item = -_ang_vel_pid[axis].Kd * (_axis_state[axis].rate - _ang_vel_pid[axis].last_measurement) / _ang_vel_pid[axis].sample_time;
    _ang_vel_pid[axis].last_measurement = _axis_state[axis].rate;
    // ssPrintf("Kd=%f\n",_ang_vel_pid[axis].Kd);
    // ssPrintf("rate=%f\n",_axis_state[axis].rate);
    // ssPrintf("last_measurement=%f\n",_ang_vel_pid[axis].last_measurement);
    // ssPrintf("sample_time=%f\n",_ang_vel_pid[axis].sample_time);
    // ssPrintf("d_item=%f\n",d_item);
    /*PID输出*/
    _ang_vel_pid[axis].output = p_item + _ang_vel_pid[axis].i_item + d_item;
    /*输出限幅*/
    // ssPrintf("output=%f\n",_ang_vel_pid[axis].output);
    // ssPrintf("--------------------------------\n");
}

/**
  * @brief  姿态控制初始化
  * @note   
  * @param  无
  * @retval 无
  */
void atti_control_init(void)
{
    // atti_control_ctx = {
    //     .flight_control_state = &flight_control_state,
    //     .angle_pid = angle_pid,
    //     .ang_vel_pid = ang_vel_pid,
    //     .rc_state = &rc_state,
    // };
    /*初始化PID控制器*/
    // angle_pid[ROLL].Kp = ANGLE_ROLL_KP;
    // angle_pid[PITCH].Kp = ANGLE_PITCH_KP;
    // angle_pid[YAW].Kp = ANGLE_YAW_KP;

    // ang_vel_pid[ROLL].Kp = RATE_ROLL_KP;
    // ang_vel_pid[PITCH].Kp = RATE_PITCH_KP;
    ang_vel_pid[YAW].Kp = RATE_YAW_KP;

    // ang_vel_pid[ROLL].Ki = RATE_ROLL_KI;
    // ang_vel_pid[PITCH].Ki = RATE_PITCH_KI;
    ang_vel_pid[YAW].Ki = RATE_YAW_KI;

    // ang_vel_pid[ROLL].Kd = RATE_ROLL_KD;
    // ang_vel_pid[PITCH].Kd = RATE_PITCH_KD;
    ang_vel_pid[YAW].Kd = RATE_YAW_KD;

    /*微分项需要初始化上一次测量值*/
    // ang_vel_pid[ROLL].last_measurement = flight_control_state.control_state[ROLL].rate;
    // ang_vel_pid[PITCH].last_measurement = flight_control_state.control_state[PITCH].rate;
    ang_vel_pid[YAW].last_measurement = flight_control_state.control_state[YAW].rate;

    /*积分项需要清零*/
    // ang_vel_pid[ROLL].i_item = 0.0f;
    // ang_vel_pid[PITCH].i_item = 0.0f;
    ang_vel_pid[YAW].i_item = 0.0;

    /*采样时间*/
    // ang_vel_pid[ROLL].sample_time = ANG_VEL_SAMPLE_TIME;/*500Hz*/
    // ang_vel_pid[PITCH].sample_time = ANG_VEL_SAMPLE_TIME;
    ang_vel_pid[YAW].sample_time = ANG_VEL_SAMPLE_TIME;
}

/**
  * @brief  测试函数
  * @note   
  * @param  无
  * @retval 无
  */
void atti_control_update(void)
{

    // angle_control(ROLL, &atti_control_ctx);
    // angle_control(PITCH, &atti_control_ctx);

    // ang_vel_control(ROLL, &atti_control_ctx);
    // ang_vel_control(PITCH, &atti_control_ctx);
    ang_vel_control(YAW, &atti_control_ctx);
}

/**
 * @}
 */
