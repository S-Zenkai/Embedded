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
#include <math.h>

/* Private function prototypes -----------------------------------------------*/
// ???PID??????
// 内部PID计算函数 (带微分先行)
static float pid_calculate(PID_t *pid, float error, float measurement, float dt);

/**
 * @brief 初始化姿态控制器PID参数
 * @param controller: 指向姿态控制器结构体的指针
 */
void attitude_controller_init(Attitude_Controller_t *controller)
{
    // 将所有PID增益默认设置为0
    set_attitude_pid_params(controller,
                            0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0);

    // 设置输出和积分限幅
    // 注意：这些值需要根据飞行器的具体特性进行调整
    float angle_pid_out_max = 200.0f; // 角度环输出限幅 (单位: 度/秒)
    float rate_pid_out_max = 500.0f;  // 角速度环输出限幅 (对应电机输出范围)
    
    controller->roll_angle_pid.output_max = angle_pid_out_max;
    controller->pitch_angle_pid.output_max = angle_pid_out_max;
    controller->yaw_angle_pid.output_max = angle_pid_out_max; // 偏航角度环在串级控制中通常不使用
    
    controller->roll_rate_pid.output_max = rate_pid_out_max;
    controller->pitch_rate_pid.output_max = rate_pid_out_max;
    controller->yaw_rate_pid.output_max = rate_pid_out_max;

    controller->roll_angle_pid.integral_max = 100.0f;
    controller->pitch_angle_pid.integral_max = 100.0f;
    controller->yaw_angle_pid.integral_max = 100.0f;
    
    controller->roll_rate_pid.integral_max = 200.0f;
    controller->pitch_rate_pid.integral_max = 200.0f;
    controller->yaw_rate_pid.integral_max = 200.0f;
}

/**
 * @brief 设置姿态控制器的PID参数
 * @param controller: 指向姿态控制器结构体的指针
 * @param ...: 所有控制器的PID增益
 */
void set_attitude_pid_params(Attitude_Controller_t *controller, 
                             float roll_angle_kp, float roll_angle_ki, float roll_angle_kd,
                             float pitch_angle_kp, float pitch_angle_ki, float pitch_angle_kd,
                             float yaw_angle_kp, float yaw_angle_ki, float yaw_angle_kd,
                             float roll_rate_kp, float roll_rate_ki, float roll_rate_kd,
                             float pitch_rate_kp, float pitch_rate_ki, float pitch_rate_kd,
                             float yaw_rate_kp, float yaw_rate_ki, float yaw_rate_kd)
{
    controller->roll_angle_pid.Kp = roll_angle_kp;
    controller->roll_angle_pid.Ki = roll_angle_ki;
    controller->roll_angle_pid.Kd = roll_angle_kd;
    
    controller->pitch_angle_pid.Kp = pitch_angle_kp;
    controller->pitch_angle_pid.Ki = pitch_angle_ki;
    controller->pitch_angle_pid.Kd = pitch_angle_kd;
    
    controller->yaw_angle_pid.Kp = yaw_angle_kp;
    controller->yaw_angle_pid.Ki = yaw_angle_ki;
    controller->yaw_angle_pid.Kd = yaw_angle_kd;
    
    controller->roll_rate_pid.Kp = roll_rate_kp;
    controller->roll_rate_pid.Ki = roll_rate_ki;
    controller->roll_rate_pid.Kd = roll_rate_kd;
    
    controller->pitch_rate_pid.Kp = pitch_rate_kp;
    controller->pitch_rate_pid.Ki = pitch_rate_ki;
    controller->pitch_rate_pid.Kd = pitch_rate_kd;
    
    controller->yaw_rate_pid.Kp = yaw_rate_kp;
    controller->yaw_rate_pid.Ki = yaw_rate_ki;
    controller->yaw_rate_pid.Kd = yaw_rate_kd;
}

/**
 * @brief 主姿态控制循环
 * @param controller: 指向姿态控制器结构体的指针
 * @param target_roll: 目标横滚角 (度)
 * @param target_pitch: 目标俯仰角 (度)
 * @param target_yaw_rate: 目标偏航角速度 (度/秒)
 * @param current_roll: 当前横滚角 (度)
 * @param current_pitch: 当前俯仰角 (度)
 * @param current_yaw: 当前偏航角 (度)
 * @param gyro_roll: 横滚角速度 (度/秒)
 * @param gyro_pitch: 俯仰角速度 (度/秒)
 * @param gyro_yaw: 偏航角速度 (度/秒)
 * @param dt: 时间间隔 (秒)
 * @param motor_roll: 横滚轴的电机输出
 * @param motor_pitch: 俯仰轴的电机输出
 * @param motor_yaw: 偏航轴的电机输出
 */
void attitude_control(Attitude_Controller_t *controller,
                      float target_roll, float target_pitch, float target_yaw_rate,
                      float current_roll, float current_pitch, float current_yaw,
                      float gyro_roll, float gyro_pitch, float gyro_yaw,
                      float dt,
                      float *motor_roll, float *motor_pitch, float *motor_yaw)
{
    float roll_angle_error = target_roll - current_roll;
    float pitch_angle_error = target_pitch - current_pitch;
    
    // 外环 (角度控制) -> 输出期望角速度
    // measurement传入的是当前角度
    float target_roll_rate = pid_calculate(&controller->roll_angle_pid, roll_angle_error, current_roll, dt);
    float target_pitch_rate = pid_calculate(&controller->pitch_angle_pid, pitch_angle_error, current_pitch, dt);
    
    // 内环 (角速度控制) -> 输出电机控制量
    // measurement传入的是当前角速度
    float roll_rate_error = target_roll_rate - gyro_roll;
    float pitch_rate_error = target_pitch_rate - gyro_pitch;
    float yaw_rate_error = target_yaw_rate - gyro_yaw;
    
    *motor_roll = pid_calculate(&controller->roll_rate_pid, roll_rate_error, gyro_roll, dt);
    *motor_pitch = pid_calculate(&controller->pitch_rate_pid, pitch_rate_error, gyro_pitch, dt);
    *motor_yaw = pid_calculate(&controller->yaw_rate_pid, yaw_rate_error, gyro_yaw, dt);
}

/**
 * @brief 通用PID控制器计算函数 (带微分先行)
 * @param pid: 指向PID结构体的指针
 * @param error: 当前误差 (target - measurement)
 * @param measurement: 当前测量值 (用于微分项)
 * @param dt: 时间间隔 (秒)
 * @return PID控制器的输出
 */
static float pid_calculate(PID_t *pid, float error, float measurement, float dt)
{
    pid->error = error;
    
    // 比例项
    float p_term = pid->Kp * pid->error;
    
    // 积分项 (带抗积分饱和)
    pid->integral += pid->error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    float i_term = pid->Ki * pid->integral;
    
    // 微分项 (基于测量值，实现微分先行)
    float derivative = (measurement - pid->last_measurement) / dt;
    float d_term = pid->Kd * (-derivative); // 注意这里的负号
    
    pid->last_measurement = measurement; // 更新上一次的测量值
    
    // 计算总输出
    pid->output = p_term + i_term + d_term;
    
    // 输出限幅
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < -pid->output_max) {
        pid->output = -pid->output_max;
    }
    
    return pid->output;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief 初始化一阶低通滤波器
 * @param filter 指向滤波器结构体的指针
 * @param dt 采样时间 (秒)
 * @param cutoff_freq 截止频率 (Hz)
 */
void lpf_init(LPF_t *filter, float dt, float cutoff_freq)
{
    if (cutoff_freq <= 0.0f) {
        filter->alpha = 1.0f; // 如果截止频率无效，则不滤波
    } else {
        float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
        filter->alpha = dt / (dt + rc);
    }
    filter->last_output = 0.0f;
}

/**
 * @brief 应用一阶低通滤波器
 * @param filter 指向滤波器结构体的指针
 * @param input 当前的输入值
 * @return 滤波后的输出值
 */
float lpf_apply(LPF_t *filter, float input)
{
    // 公式: output = last_output + alpha * (input - last_output)
    float output = filter->last_output + filter->alpha * (input - filter->last_output);
    filter->last_output = output;
    return output;
}
