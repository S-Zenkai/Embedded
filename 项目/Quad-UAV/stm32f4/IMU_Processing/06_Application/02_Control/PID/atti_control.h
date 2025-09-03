/**
  ******************************************************************************
  * @file    atti_control.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/17
  * @brief   姿态控制算法头文件
  ******************************************************************************
  * @attention
  *
  * 本文件包含串级PID姿态控制器的数据结构和函数原型。
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ATTITUDE_CONTROL_H
#define __ATTITUDE_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief PID控制器结构体
 */
typedef struct {
    float Kp; // 比例增益
    float Ki; // 积分增益
    float Kd; // 微分增益
    
    float error;      // 当前误差
    float last_error; // 上一次误差
    float integral;   // 积分项
    
    // 微分先行(Derivative on Measurement)相关
    // 微分项将直接作用于测量值的变化率，而不是误差的变化率
    // 这样可以防止目标值突变时引起的微分尖峰(Derivative Kick)
    float last_measurement; // 上一次的测量值
    
    float output;       // 输出值
    float output_max;   // 输出限幅
    float integral_max; // 积分限幅
} PID_t;

/**
 * @brief 姿态控制器结构体
 */
typedef struct {
    // 外环 (角度环)
    PID_t roll_angle_pid;
    PID_t pitch_angle_pid;
    PID_t yaw_angle_pid; // 偏航通常只用角速度环，这里为完整性而定义
    
    // 内环 (角速度环)
    PID_t roll_rate_pid;
    PID_t pitch_rate_pid;
    PID_t yaw_rate_pid;
    
    // 为陀螺仪数据添加低通滤波器
    LPF_t gyro_roll_lpf;
    LPF_t gyro_pitch_lpf;
    LPF_t gyro_yaw_lpf;
    
} Attitude_Controller_t;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief 初始化姿态控制器
 * @param controller 指向姿态控制器结构体的指针
 */
void attitude_controller_init(Attitude_Controller_t *controller);

/**
 * @brief 设置姿态控制器的所有PID参数
 * @param controller 指向姿态控制器结构体的指针
 * @param ... 各个环的PID增益
 */
void set_attitude_pid_params(Attitude_Controller_t *controller, 
                             float roll_angle_kp, float roll_angle_ki, float roll_angle_kd,
                             float pitch_angle_kp, float pitch_angle_ki, float pitch_angle_kd,
                             float yaw_angle_kp, float yaw_angle_ki, float yaw_angle_kd,
                             float roll_rate_kp, float roll_rate_ki, float roll_rate_kd,
                             float pitch_rate_kp, float pitch_rate_ki, float pitch_rate_kd,
                             float yaw_rate_kp, float yaw_rate_ki, float yaw_rate_kd);

/**
 * @brief 姿态控制计算函数
 * @param controller 指向姿态控制器结构体的指针
 * @param target_roll 目标横滚角 (度)
 * @param target_pitch 目标俯仰角 (度)
 * @param target_yaw_rate 目标偏航角速度 (度/秒)
 * @param current_roll 当前横滚角 (度)
 * @param current_pitch 当前俯仰角 (度)
 * @param current_yaw 当前偏航角 (度)
 * @param gyro_roll 横滚角速度 (度/秒)
 * @param gyro_pitch 俯仰角速度 (度/秒)
 * @param gyro_yaw 偏航角速度 (度/秒)
 * @param dt 时间间隔 (秒)
 * @param motor_roll 横滚轴的电机输出
 * @param motor_pitch 俯仰轴的电机输出
 * @param motor_yaw 偏航轴的电机输出
 */
void attitude_control(Attitude_Controller_t *controller, 
                      float target_roll, float target_pitch, float target_yaw_rate,
                      float current_roll, float current_pitch, float current_yaw,
                      float gyro_roll, float gyro_pitch, float gyro_yaw,
                      float dt,
                      float *motor_roll, float *motor_pitch, float *motor_yaw);

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 一阶低通滤波器结构体
 */
typedef struct {
    float alpha;          // 滤波系数, alpha = dt / (dt + RC), 其中 RC = 1 / (2*PI*cutoff_freq)
    float last_output;    // 上一次的滤波输出
} LPF_t;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief 初始化一阶低通滤波器
 * @param filter 指向滤波器结构体的指针
 * @param dt 采样时间 (秒)
 * @param cutoff_freq 截止频率 (Hz)
 */
void lpf_init(LPF_t *filter, float dt, float cutoff_freq);

/**
 * @brief 应用一阶低通滤波器
 * @param filter 指向滤波器结构体的指针
 * @param input 当前的输入值
 * @return 滤波后的输出值
 */
float lpf_apply(LPF_t *filter, float input);


#endif /* __ATTITUDE_CONTROL_H */
