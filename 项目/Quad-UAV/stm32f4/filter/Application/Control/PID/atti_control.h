/**
  ******************************************************************************
  * @file    atti_control.h
  * @author  kai
  * @version V1.0.0
  * @data    2025/07/17
  * @brief   ��̬�����㷨ͷ�ļ�
  ******************************************************************************
  * @attention
  *
  * ���ļ���������PID��̬�����������ݽṹ�ͺ���ԭ�͡�
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
 * @brief PID�������ṹ��
 */
typedef struct {
    float Kp; // ��������
    float Ki; // ��������
    float Kd; // ΢������
    
    float error;      // ��ǰ���
    float last_error; // ��һ�����
    float integral;   // ������
    
    // ΢������(Derivative on Measurement)���
    // ΢���ֱ�������ڲ���ֵ�ı仯�ʣ����������ı仯��
    // �������Է�ֹĿ��ֵͻ��ʱ�����΢�ּ��(Derivative Kick)
    float last_measurement; // ��һ�εĲ���ֵ
    
    float output;       // ���ֵ
    float output_max;   // ����޷�
    float integral_max; // �����޷�
} PID_t;

/**
 * @brief ��̬�������ṹ��
 */
typedef struct {
    // �⻷ (�ǶȻ�)
    PID_t roll_angle_pid;
    PID_t pitch_angle_pid;
    PID_t yaw_angle_pid; // ƫ��ͨ��ֻ�ý��ٶȻ�������Ϊ�����Զ�����
    
    // �ڻ� (���ٶȻ�)
    PID_t roll_rate_pid;
    PID_t pitch_rate_pid;
    PID_t yaw_rate_pid;
    
    // Ϊ������������ӵ�ͨ�˲���
    LPF_t gyro_roll_lpf;
    LPF_t gyro_pitch_lpf;
    LPF_t gyro_yaw_lpf;
    
} Attitude_Controller_t;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief ��ʼ����̬������
 * @param controller ָ����̬�������ṹ���ָ��
 */
void attitude_controller_init(Attitude_Controller_t *controller);

/**
 * @brief ������̬������������PID����
 * @param controller ָ����̬�������ṹ���ָ��
 * @param ... ��������PID����
 */
void set_attitude_pid_params(Attitude_Controller_t *controller, 
                             float roll_angle_kp, float roll_angle_ki, float roll_angle_kd,
                             float pitch_angle_kp, float pitch_angle_ki, float pitch_angle_kd,
                             float yaw_angle_kp, float yaw_angle_ki, float yaw_angle_kd,
                             float roll_rate_kp, float roll_rate_ki, float roll_rate_kd,
                             float pitch_rate_kp, float pitch_rate_ki, float pitch_rate_kd,
                             float yaw_rate_kp, float yaw_rate_ki, float yaw_rate_kd);

/**
 * @brief ��̬���Ƽ��㺯��
 * @param controller ָ����̬�������ṹ���ָ��
 * @param target_roll Ŀ������ (��)
 * @param target_pitch Ŀ�긩���� (��)
 * @param target_yaw_rate Ŀ��ƫ�����ٶ� (��/��)
 * @param current_roll ��ǰ����� (��)
 * @param current_pitch ��ǰ������ (��)
 * @param current_yaw ��ǰƫ���� (��)
 * @param gyro_roll ������ٶ� (��/��)
 * @param gyro_pitch �������ٶ� (��/��)
 * @param gyro_yaw ƫ�����ٶ� (��/��)
 * @param dt ʱ���� (��)
 * @param motor_roll �����ĵ�����
 * @param motor_pitch ������ĵ�����
 * @param motor_yaw ƫ����ĵ�����
 */
void attitude_control(Attitude_Controller_t *controller, 
                      float target_roll, float target_pitch, float target_yaw_rate,
                      float current_roll, float current_pitch, float current_yaw,
                      float gyro_roll, float gyro_pitch, float gyro_yaw,
                      float dt,
                      float *motor_roll, float *motor_pitch, float *motor_yaw);

/* Exported types ------------------------------------------------------------*/

/**
 * @brief һ�׵�ͨ�˲����ṹ��
 */
typedef struct {
    float alpha;          // �˲�ϵ��, alpha = dt / (dt + RC), ���� RC = 1 / (2*PI*cutoff_freq)
    float last_output;    // ��һ�ε��˲����
} LPF_t;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief ��ʼ��һ�׵�ͨ�˲���
 * @param filter ָ���˲����ṹ���ָ��
 * @param dt ����ʱ�� (��)
 * @param cutoff_freq ��ֹƵ�� (Hz)
 */
void lpf_init(LPF_t *filter, float dt, float cutoff_freq);

/**
 * @brief Ӧ��һ�׵�ͨ�˲���
 * @param filter ָ���˲����ṹ���ָ��
 * @param input ��ǰ������ֵ
 * @return �˲�������ֵ
 */
float lpf_apply(LPF_t *filter, float input);


#endif /* __ATTITUDE_CONTROL_H */
