/**
 ******************************************************************************
 * @file    common.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/04/03
 * @brief   项目的“工具箱”。提供一系列通用的、可复用的“工具”（函数、数据结构）
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "pro_common.h"
#include "ringbuff.h"

/**
 * @brief  3*3矩阵*3*1向量
 * @note
 * @param  matrix: 3*3矩阵指针
 * @param  vector: 3*1向量指针
 * @param  result: 输出：3*1向量指针
 * @retval 无
 */
void Matrix3x3_f_mul_Vector3_f(const Matrix3x3_f matrix, const Axis3_f_u *vector, Axis3_f_u *result)
{
    for (int i = 0; i < 3; i++)
    {
        result->axis_arr[i] = 0;
        for (int j = 0; j < 3; j++)
        {
            result->axis_arr[i] += matrix[i][j] * vector->axis_arr[j];
        }
    }
}

/**
 * @brief  矩阵转置
 * @note   不允许原地转置
 * @param  matrix: 3*3矩阵指针
 * @param  result: 输出：3*3矩阵指针
 * @retval 无
 */
void Matrix3x3_f_transpose(const Matrix3x3_f matrix, Matrix3x3_f result)
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i][j] = matrix[j][i];
        }
    }
}

/**
 * @brief  向量叉乘
 * @note
 * @param  vector1: 3*1向量指针
 * @param  vector2: 3*1向量指针
 * @param  result: 输出：3*1向量指针
 * @retval 无
 */
void Vector3_f_cross_product(const Axis3_f_u *vector1, const Axis3_f_u *vector2, Axis3_f_u *result)
{
    result->axis_arr[0] = vector1->axis_arr[1] * vector2->axis_arr[2] - vector1->axis_arr[2] * vector2->axis_arr[1];
    result->axis_arr[1] = vector1->axis_arr[2] * vector2->axis_arr[0] - vector1->axis_arr[0] * vector2->axis_arr[2];
    result->axis_arr[2] = vector1->axis_arr[0] * vector2->axis_arr[1] - vector1->axis_arr[1] * vector2->axis_arr[0];
}

/**
 * @brief  向量加法
 * @note
 * @param  vector1: 3*1向量指针
 * @param  vector2: 3*1向量指针
 * @param  result: 输出：3*1向量指针
 * @retval 无
 */
void Vector3_f_add(const Axis3_f_u *vector1, const Axis3_f_u *vector2, Axis3_f_u *result)
{
    result->axis_arr[0] = vector1->axis_arr[0] + vector2->axis_arr[0];
    result->axis_arr[1] = vector1->axis_arr[1] + vector2->axis_arr[1];
    result->axis_arr[2] = vector1->axis_arr[2] + vector2->axis_arr[2];
}
