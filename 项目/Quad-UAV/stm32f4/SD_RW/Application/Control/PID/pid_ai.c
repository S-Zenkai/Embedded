#include "pid_controller.h"
#include <string.h> // ���� memset

// ȫ�ֵ�PID���������飬ÿ����һ��
static PID_Controller_t pid_controllers[PID_AXIS_COUNT];

void PID_Init(void)
{
    // ʹ��memset�����п�������״̬�Ͳ������㣬����һ���õĿ�ʼ
    memset(pid_controllers, 0, sizeof(pid_controllers));

    // ���������Ϊÿ��������һЩ��ȫ��Ĭ��ֵ
    for (int i = 0; i < PID_AXIS_COUNT; i++)
    {
        PID_Set_Kp((PID_Axis_e)i, 1.0f);
        PID_Set_Ki((PID_Axis_e)i, 0.0f);
        PID_Set_Kd((PID_Axis_e)i, 0.0f);
        PID_Set_IntegralMax((PID_Axis_e)i, 100.0f);
        PID_Set_OutputMax((PID_Axis_e)i, 400.0f);
        PID_Set_D_LPF_Alpha((PID_Axis_e)i, 1.0f); // Ĭ��Ϊ1.0�������˲�
    }
}

/*c����ģ��c++����������̣����Խ�pid_controllers������Ա������PID_Reset()������Ա����*/
void PID_Reset(PID_Axis_e axis)
{
    if (axis >= PID_AXIS_COUNT)
        return;

    // ���û��ֺ���ʷ״̬������PID����
    pid_controllers[axis].integral_sum = 0.0f;
    pid_controllers[axis].previous_measurement = 0.0f;
    pid_controllers[axis].filtered_derivative = 0.0f;
}

float PID_Compute(PID_Axis_e axis, float setpoint, float measurement, float dt)
{
    if (axis >= PID_AXIS_COUNT || dt <= 0.0f)
    {
        return 0.0f;
    }

    PID_Controller_t *pid = &pid_controllers[axis]; // ʹ��ָ�������Ч�ʺͿɶ���

    // 1. �������
    float error = setpoint - measurement;

    // 2. ��������� (P)
    float p_term = pid->Kp * error;

    // 3. ��������� (I)
    pid->integral_sum += pid->Ki * error * dt;
    // �����ֱ��ʹ���
    if (pid->integral_sum > pid->integral_max)
    {
        pid->integral_sum = pid->integral_max;
    }
    else if (pid->integral_sum < -pid->integral_max)
    {
        pid->integral_sum = -pid->integral_max;
    }
    float i_term = pid->integral_sum;

    // 4. ����΢���� (D) - ΢���������ͨ�˲�
    // ����ԭʼ΢��ֵ�����ڲ���ֵ�ı仯���������ı仯��
    float raw_derivative = (measurement - pid->previous_measurement) / dt;

    // ��΢��ֵ����һ�׵�ͨ�˲�
    pid->filtered_derivative = (1.0f - pid->d_lpf_alpha) * pid->filtered_derivative + pid->d_lpf_alpha * raw_derivative;

    // ������һ�εĲ���ֵ
    pid->previous_measurement = measurement;

    // ΢�����У�D�������ڲ���ֵ�ı仯�ʣ�����Ϊ��
    float d_term = -pid->Kd * pid->filtered_derivative;

    // 5. ���������
    float output = p_term + i_term + d_term;

    // 6. ������޷�
    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < -pid->output_max)
    {
        output = -pid->output_max;
    }

    return output;
}

/* --- ���������ӿڵ�ʵ�� --- */

void PID_Set_Kp(PID_Axis_e axis, float kp)
{
    if (axis < PID_AXIS_COUNT)
    {
        pid_controllers[axis].Kp = kp;
    }
}

void PID_Set_Ki(PID_Axis_e axis, float ki)
{
    if (axis < PID_AXIS_COUNT)
    {
        pid_controllers[axis].Ki = ki;
    }
}

void PID_Set_Kd(PID_Axis_e axis, float kd)
{
    if (axis < PID_AXIS_COUNT)
    {
        pid_controllers[axis].Kd = kd;
    }
}

void PID_Set_IntegralMax(PID_Axis_e axis, float max_val)
{
    if (axis < PID_AXIS_COUNT)
    {
        pid_controllers[axis].integral_max = max_val;
    }
}

void PID_Set_OutputMax(PID_Axis_e axis, float max_val)
{
    if (axis < PID_AXIS_COUNT)
    {
        pid_controllers[axis].output_max = max_val;
    }
}

void PID_Set_D_LPF_Alpha(PID_Axis_e axis, float alpha)
{
    if (axis < PID_AXIS_COUNT)
    {
        // Լ�� alpha �� (0, 1] ��Χ��
        if (alpha <= 0.0f)
            alpha = 0.01f;
        if (alpha > 1.0f)
            alpha = 1.0f;
        pid_controllers[axis].d_lpf_alpha = alpha;
    }
}