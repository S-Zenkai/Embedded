#include "pid_controller.h"
#include <string.h> // 用于 memset

// 全局的PID控制器数组，每个轴一个
static PID_Controller_t pid_controllers[PID_AXIS_COUNT];

void PID_Init(void)
{
    // 使用memset将所有控制器的状态和参数清零，这是一个好的开始
    memset(pid_controllers, 0, sizeof(pid_controllers));

    // 在这里可以为每个轴设置一些安全的默认值
    for (int i = 0; i < PID_AXIS_COUNT; i++)
    {
        PID_Set_Kp((PID_Axis_e)i, 1.0f);
        PID_Set_Ki((PID_Axis_e)i, 0.0f);
        PID_Set_Kd((PID_Axis_e)i, 0.0f);
        PID_Set_IntegralMax((PID_Axis_e)i, 100.0f);
        PID_Set_OutputMax((PID_Axis_e)i, 400.0f);
        PID_Set_D_LPF_Alpha((PID_Axis_e)i, 1.0f); // 默认为1.0，即无滤波
    }
}

/*c语言模拟c++的面向对象编程，可以将pid_controllers看作成员变量，PID_Reset()看作成员函数*/
void PID_Reset(PID_Axis_e axis)
{
    if (axis >= PID_AXIS_COUNT)
        return;

    // 重置积分和历史状态，保留PID参数
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

    PID_Controller_t *pid = &pid_controllers[axis]; // 使用指针以提高效率和可读性

    // 1. 计算误差
    float error = setpoint - measurement;

    // 2. 计算比例项 (P)
    float p_term = pid->Kp * error;

    // 3. 计算积分项 (I)
    pid->integral_sum += pid->Ki * error * dt;
    // 抗积分饱和处理
    if (pid->integral_sum > pid->integral_max)
    {
        pid->integral_sum = pid->integral_max;
    }
    else if (pid->integral_sum < -pid->integral_max)
    {
        pid->integral_sum = -pid->integral_max;
    }
    float i_term = pid->integral_sum;

    // 4. 计算微分项 (D) - 微分先行与低通滤波
    // 计算原始微分值（基于测量值的变化，而非误差的变化）
    float raw_derivative = (measurement - pid->previous_measurement) / dt;

    // 对微分值进行一阶低通滤波
    pid->filtered_derivative = (1.0f - pid->d_lpf_alpha) * pid->filtered_derivative + pid->d_lpf_alpha * raw_derivative;

    // 更新上一次的测量值
    pid->previous_measurement = measurement;

    // 微分先行，D项作用于测量值的变化率，符号为负
    float d_term = -pid->Kd * pid->filtered_derivative;

    // 5. 计算总输出
    float output = p_term + i_term + d_term;

    // 6. 总输出限幅
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

/* --- 参数整定接口的实现 --- */

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
        // 约束 alpha 在 (0, 1] 范围内
        if (alpha <= 0.0f)
            alpha = 0.01f;
        if (alpha > 1.0f)
            alpha = 1.0f;
        pid_controllers[axis].d_lpf_alpha = alpha;
    }
}