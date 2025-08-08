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
#include <math.h>

/* Private function prototypes -----------------------------------------------*/
// ???PID??????
// �ڲ�PID���㺯�� (��΢������)
static float pid_calculate(PID_t *pid, float error, float measurement, float dt);

/**
 * @brief ��ʼ����̬������PID����
 * @param controller: ָ����̬�������ṹ���ָ��
 */
void attitude_controller_init(Attitude_Controller_t *controller)
{
    // ������PID����Ĭ������Ϊ0
    set_attitude_pid_params(controller,
                            0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0, 0, 0, 0);

    // ��������ͻ����޷�
    // ע�⣺��Щֵ��Ҫ���ݷ������ľ������Խ��е���
    float angle_pid_out_max = 200.0f; // �ǶȻ�����޷� (��λ: ��/��)
    float rate_pid_out_max = 500.0f;  // ���ٶȻ�����޷� (��Ӧ��������Χ)
    
    controller->roll_angle_pid.output_max = angle_pid_out_max;
    controller->pitch_angle_pid.output_max = angle_pid_out_max;
    controller->yaw_angle_pid.output_max = angle_pid_out_max; // ƫ���ǶȻ��ڴ���������ͨ����ʹ��
    
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
 * @brief ������̬��������PID����
 * @param controller: ָ����̬�������ṹ���ָ��
 * @param ...: ���п�������PID����
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
 * @brief ����̬����ѭ��
 * @param controller: ָ����̬�������ṹ���ָ��
 * @param target_roll: Ŀ������ (��)
 * @param target_pitch: Ŀ�긩���� (��)
 * @param target_yaw_rate: Ŀ��ƫ�����ٶ� (��/��)
 * @param current_roll: ��ǰ����� (��)
 * @param current_pitch: ��ǰ������ (��)
 * @param current_yaw: ��ǰƫ���� (��)
 * @param gyro_roll: ������ٶ� (��/��)
 * @param gyro_pitch: �������ٶ� (��/��)
 * @param gyro_yaw: ƫ�����ٶ� (��/��)
 * @param dt: ʱ���� (��)
 * @param motor_roll: �����ĵ�����
 * @param motor_pitch: ������ĵ�����
 * @param motor_yaw: ƫ����ĵ�����
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
    
    // �⻷ (�Ƕȿ���) -> ����������ٶ�
    // measurement������ǵ�ǰ�Ƕ�
    float target_roll_rate = pid_calculate(&controller->roll_angle_pid, roll_angle_error, current_roll, dt);
    float target_pitch_rate = pid_calculate(&controller->pitch_angle_pid, pitch_angle_error, current_pitch, dt);
    
    // �ڻ� (���ٶȿ���) -> ������������
    // measurement������ǵ�ǰ���ٶ�
    float roll_rate_error = target_roll_rate - gyro_roll;
    float pitch_rate_error = target_pitch_rate - gyro_pitch;
    float yaw_rate_error = target_yaw_rate - gyro_yaw;
    
    *motor_roll = pid_calculate(&controller->roll_rate_pid, roll_rate_error, gyro_roll, dt);
    *motor_pitch = pid_calculate(&controller->pitch_rate_pid, pitch_rate_error, gyro_pitch, dt);
    *motor_yaw = pid_calculate(&controller->yaw_rate_pid, yaw_rate_error, gyro_yaw, dt);
}

/**
 * @brief ͨ��PID���������㺯�� (��΢������)
 * @param pid: ָ��PID�ṹ���ָ��
 * @param error: ��ǰ��� (target - measurement)
 * @param measurement: ��ǰ����ֵ (����΢����)
 * @param dt: ʱ���� (��)
 * @return PID�����������
 */
static float pid_calculate(PID_t *pid, float error, float measurement, float dt)
{
    pid->error = error;
    
    // ������
    float p_term = pid->Kp * pid->error;
    
    // ������ (�������ֱ���)
    pid->integral += pid->error * dt;
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max;
    }
    float i_term = pid->Ki * pid->integral;
    
    // ΢���� (���ڲ���ֵ��ʵ��΢������)
    float derivative = (measurement - pid->last_measurement) / dt;
    float d_term = pid->Kd * (-derivative); // ע������ĸ���
    
    pid->last_measurement = measurement; // ������һ�εĲ���ֵ
    
    // ���������
    pid->output = p_term + i_term + d_term;
    
    // ����޷�
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
 * @brief ��ʼ��һ�׵�ͨ�˲���
 * @param filter ָ���˲����ṹ���ָ��
 * @param dt ����ʱ�� (��)
 * @param cutoff_freq ��ֹƵ�� (Hz)
 */
void lpf_init(LPF_t *filter, float dt, float cutoff_freq)
{
    if (cutoff_freq <= 0.0f) {
        filter->alpha = 1.0f; // �����ֹƵ����Ч�����˲�
    } else {
        float rc = 1.0f / (2.0f * M_PI * cutoff_freq);
        filter->alpha = dt / (dt + rc);
    }
    filter->last_output = 0.0f;
}

/**
 * @brief Ӧ��һ�׵�ͨ�˲���
 * @param filter ָ���˲����ṹ���ָ��
 * @param input ��ǰ������ֵ
 * @return �˲�������ֵ
 */
float lpf_apply(LPF_t *filter, float input)
{
    // ��ʽ: output = last_output + alpha * (input - last_output)
    float output = filter->last_output + filter->alpha * (input - filter->last_output);
    filter->last_output = output;
    return output;
}
