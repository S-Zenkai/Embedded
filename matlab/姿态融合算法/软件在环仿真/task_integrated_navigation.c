/**
  ******************************************************************************
  * @file    task_integrated_navigation.c
  * @author  kai
  * @version V1.0.0
  * @data    2025/09/30
  * @brief   组合导航任务
  ******************************************************************************
  * @attention
  *
  * 组合导航任务
  *
  ******************************************************************************
  */
#include "task_integrated_navigation.h"
#include "attitude_estimate.h"

Attitude_Estimate_t att_est;/*姿态解算结构体*/

void Task_IntegratedNavigation(void)
{
    /*姿态解算，更新三轴欧拉角*/
    Attitude_Estimate_Update(&att_est,&flgt_ctl);
}


