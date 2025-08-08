#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

// ʹ��ö���������ر�ʶPID�ᣬ����ʹ��ħ������(0,1,2)
typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_AXIS_COUNT // ö�ٵ�ĩβ��Ա�����Է���ػ�ȡ�������
} PID_Axis_e;

// PID�������ṹ�壬�������в�����״̬����
typedef struct {
    // 1. PID����ϵ��
    float Kp;
    float Ki;
    float Kd;

    // 2. ���ֿ����� (Anti-Windup) ������
    float integral_max;

    // 3. ������޷�
    float output_max;
    
    // 4. ΢����һ�׵�ͨ�˲� (LPF) ϵ�� alpha
    // alpha ԽС���˲�Ч��Խǿ��alpha = dt / (RC + dt)
    // Ϊ�򻯽ӿڣ��������û�ֱ������alpha
    float d_lpf_alpha;

    // 5. �ڲ�״̬���� (����Ҫ�ֶ�����)
    float integral_sum;         // �����ۼ�ֵ
    float previous_measurement; // ��һ�εĲ���ֵ (����΢������)
    float filtered_derivative;  // �����˲����΢��ֵ

} PID_Controller_t;


/**
 * @brief ��ʼ���������PID������
 * @note  ��ϵͳ����ʱ����һ��
 */
void PID_Init(void);

/**
 * @brief ���õ���PID��������״̬
 * @note  �ڽ����ɻ���������̬ʱ���ã���������ۼӵ���ʷ״̬
 * @param axis Ҫ���õ��� (PID_ROLL, PID_PITCH, PID_YAW)
 */
void PID_Reset(PID_Axis_e axis);

/**
 * @brief ����PID���
 * @param axis        Ҫ������� (PID_ROLL, PID_PITCH, PID_YAW)
 * @param setpoint    Ŀ��ֵ (���磬�����ǶȻ���ٶ�)
 * @param measurement ʵ�ʲ���ֵ (���磬��������õĽǶȻ���ٶ�)
 * @param dt          ʱ����(��)�����μ���֮��ļ��ʱ��
 * @return            PID������Ŀ�����
 */
float PID_Compute(PID_Axis_e axis, float setpoint, float measurement, float dt);


/* --- ���������ӿ� --- */

/**
 * @brief ����ָ�����Kp����
 * @param axis Ŀ����
 * @param kp   ��������ֵ
 */
void PID_Set_Kp(PID_Axis_e axis, float kp);

/**
 * @brief ����ָ�����Ki����
 * @param axis Ŀ����
 * @param ki   ��������ֵ
 */
void PID_Set_Ki(PID_Axis_e axis, float ki);

/**
 * @brief ����ָ�����Kd����
 * @param axis Ŀ����
 * @param kd   ΢������ֵ
 */
void PID_Set_Kd(PID_Axis_e axis, float kd);

/**
 * @brief ���û������޷�
 * @param axis   Ŀ����
 * @param max_val �����ۼ�ֵ�ľ���ֵ����
 */
void PID_Set_IntegralMax(PID_Axis_e axis, float max_val);

/**
 * @brief ����������޷�
 * @param axis   Ŀ����
 * @param max_val PID������ľ���ֵ����
 */
void PID_Set_OutputMax(PID_Axis_e axis, float max_val);

/**
 * @brief ����΢�����ͨ�˲���alphaϵ��
 * @param axis  Ŀ����
 * @param alpha �˲�ϵ�� (0.0 < alpha <= 1.0)��ֵԽС���˲�Խǿ��
 * @note  alpha = 2 * PI * dt * cutoff_freq���򵥵Ľ��ƹ�ϵ����Ҫ����ʵ�����������
 *        һ�����ȶ��Ĺ�ʽ�� alpha = dt / (RC + dt)������ RC = 1 / (2*PI*cutoff_freq)��
 */
void PID_Set_D_LPF_Alpha(PID_Axis_e axis, float alpha);


#endif // PID_CONTROLLER_H