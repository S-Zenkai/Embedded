#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

// 使用枚举来清晰地标识PID轴，避免使用魔术数字(0,1,2)
typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_AXIS_COUNT // 枚举的末尾成员，可以方便地获取轴的数量
} PID_Axis_e;

// PID控制器结构体，包含所有参数和状态变量
typedef struct {
    // 1. PID增益系数
    float Kp;
    float Ki;
    float Kd;

    // 2. 积分抗饱和 (Anti-Windup) 上下限
    float integral_max;

    // 3. 总输出限幅
    float output_max;
    
    // 4. 微分项一阶低通滤波 (LPF) 系数 alpha
    // alpha 越小，滤波效果越强。alpha = dt / (RC + dt)
    // 为简化接口，我们让用户直接设置alpha
    float d_lpf_alpha;

    // 5. 内部状态变量 (不需要手动设置)
    float integral_sum;         // 积分累加值
    float previous_measurement; // 上一次的测量值 (用于微分先行)
    float filtered_derivative;  // 经过滤波后的微分值

} PID_Controller_t;


/**
 * @brief 初始化所有轴的PID控制器
 * @note  在系统启动时调用一次
 */
void PID_Init(void);

/**
 * @brief 重置单个PID控制器的状态
 * @note  在解锁飞机或重置姿态时调用，清除积分累加等历史状态
 * @param axis 要重置的轴 (PID_ROLL, PID_PITCH, PID_YAW)
 */
void PID_Reset(PID_Axis_e axis);

/**
 * @brief 计算PID输出
 * @param axis        要计算的轴 (PID_ROLL, PID_PITCH, PID_YAW)
 * @param setpoint    目标值 (例如，期望角度或角速度)
 * @param measurement 实际测量值 (例如，传感器测得的角度或角速度)
 * @param dt          时间间隔(秒)，两次计算之间的间隔时间
 * @return            PID计算出的控制量
 */
float PID_Compute(PID_Axis_e axis, float setpoint, float measurement, float dt);


/* --- 参数整定接口 --- */

/**
 * @brief 设置指定轴的Kp增益
 * @param axis 目标轴
 * @param kp   比例增益值
 */
void PID_Set_Kp(PID_Axis_e axis, float kp);

/**
 * @brief 设置指定轴的Ki增益
 * @param axis 目标轴
 * @param ki   积分增益值
 */
void PID_Set_Ki(PID_Axis_e axis, float ki);

/**
 * @brief 设置指定轴的Kd增益
 * @param axis 目标轴
 * @param kd   微分增益值
 */
void PID_Set_Kd(PID_Axis_e axis, float kd);

/**
 * @brief 设置积分项限幅
 * @param axis   目标轴
 * @param max_val 积分累加值的绝对值上限
 */
void PID_Set_IntegralMax(PID_Axis_e axis, float max_val);

/**
 * @brief 设置总输出限幅
 * @param axis   目标轴
 * @param max_val PID总输出的绝对值上限
 */
void PID_Set_OutputMax(PID_Axis_e axis, float max_val);

/**
 * @brief 设置微分项低通滤波的alpha系数
 * @param axis  目标轴
 * @param alpha 滤波系数 (0.0 < alpha <= 1.0)。值越小，滤波越强。
 * @note  alpha = 2 * PI * dt * cutoff_freq。简单的近似关系，需要根据实际情况调整。
 *        一个更稳定的公式是 alpha = dt / (RC + dt)，其中 RC = 1 / (2*PI*cutoff_freq)。
 */
void PID_Set_D_LPF_Alpha(PID_Axis_e axis, float alpha);


#endif // PID_CONTROLLER_H