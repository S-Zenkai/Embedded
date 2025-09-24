/**
 ******************************************************************************
 * @file    mag_processing.c
 * @author  kai
 * @version V1.0.0
 * @data    2025/09/03
 * @brief   磁力计数据处理
 ******************************************************************************
 * @attention
 *
 *
 *
 ******************************************************************************
 */
#include "mag_processing.h"
#include "com_data.h"
#include "hmc5883l.h"
#include "bsp_flash.h"
#include "pro_config.h"
#include "bsp_mpu6000.h"

/*状态*/
#define MAG_PROCESSING_STATE_INIT 0
/*状态机变量*/
uint8_t mag_processing_state = 0;

/*磁力计处理结构体*/
typedef struct
{
    float circle_angle_roll;   /*roll(滚转角)旋转角度(x轴)*/
    float circle_angle_pitch;  /*pitch(俯仰角)旋转角度(y轴)*/
    float circle_angle_yaw;    /*yaw(偏航角)旋转角度(z轴)*/
    uint8_t delay_count;       /*延时计数，与程序执行频率搭配*/
    float max_value_x;         /*x轴最大值*/
    float min_value_x;         /*x轴最小值*/
    float max_value_y;         /*y轴最大值*/
    float min_value_y;         /*y轴最小值*/
    float max_value_z;         /*z轴最大值*/
    float min_value_z;         /*z轴最小值*/
    float offset_x;            /*x轴偏移量*/
    float offset_y;            /*y轴偏移量*/
    float offset_z;            /*z轴偏移量*/
    uint8_t calibration_state; /*校准状态标志*/
} mag_processing_t;

mag_processing_t mag_processing;

/*磁力计校准参数*/
typedef struct
{
    uint32_t magic_key;
    uint32_t offset_x; /*x轴偏移量*/
    uint32_t offset_y; /*y轴偏移量*/
    uint32_t offset_z; /*z轴偏移量*/
} mag_calibration_params_t;

mag_calibration_params_t mag_calibration_params;

/*磁力计校准参数魔数*/
#define MAG_CALIBRATION_PARAMS_MAGIC_KEY 0x55AA55AA

/*地磁数据测量结果理论最大值*/
#define FLTOAT_MAX 3.40282347e+38F

static void MAG_Calibration_Instrction(void);
static void MAG_Calibration_Operation(void);
static void MAG_Calibration_Init(void);
static void Get_Max_Min_Value_XY(void);
static void Calculate_Offset_XY(void);
static void Calculate_Offset_Z(void);
static void Save_Calibration_Parameters(void);
static bool Read_Calibration_Parameters(void);
static void Get_Max_Min_Value_Z(void);
static void MAG_Calibration_Reset(void);
static void MAG_Calibration_Reset(void);
bool isCali_XYFactor_Check_Succeed(void);
void MAG_Calibration(void);

/**
 * @brief  磁力计数据处理初始化
 * @note
 * @param  无
 * @retval 无
 */
void MAG_Processing_Init(void)
{
    uint32_t Tick = 0;
    mag_processing_state = 0;
    // u32 temp_buff[4]={0};
    // STMFLASH_Write(ADDR_FLASH_SECTOR_11 + 4, (u32*)&temp_buff, sizeof(mag_calibration_params_t)/4);
    hmc5883l_init(HMC5883L_GAIN_1090, HMC5883L_RATE_75, HMC5883L_SAMPLES_8);
    if (Read_Calibration_Parameters())
    {
        /*Flash中读取到校准参数*/
        mag_processing.calibration_state = 1;
    }
    else
    {
        /*Flash中没有读取到校准参数*/
        MAG_Calibration();        /*进行一次初始化*/
        mag_processing_state = 1; /*进入校准状态*/
    }

    while (mag_processing.calibration_state != 1)
    {
        /*读取原始数据*/
        MPU6000_GetData(&imu_raw_data.acc, &imu_raw_data.gyro);
        MPU6000_Data_Conversion(&imu_cal_data.acc, &imu_cal_data.gyro, &imu_raw_data.acc, &imu_raw_data.gyro);
        /*后面的磁力计校准需要用到不需太精确的imu数据，这里直接将未校准的imu数据赋值给flight_control_state*/
        for (int i = 0; i < 3; i++)
        {
            flight_control_state.gyro_data.axis_arr[i] = imu_cal_data.gyro.axis_arr[i];
            flight_control_state.acc_data.axis_arr[i] = imu_cal_data.acc.axis_arr[i];
        }
        hmc5883l_read_raw_data(&mag_raw_data);
        hmc5883l_convert_to_gauss(&mag_cal_data, &mag_raw_data);
        for (int i = 0; i < 3; i++)
        {
            flight_control_state.mag_data.axis_arr[i] = mag_cal_data.axis_arr[i];
        }
        MAG_Calibration(); /*获取校准参数*/
        /*50hz采样*/
        delay_ms(20);
        // if (Main_Loop_Update_Flag == SET)
        // {
        //     Main_Loop_Update_Flag = RESET;
        //     /*MAG_Processing50hz调用一次，更改还需要改下MAG_Processing函数(已在函数注释内指明)*/
        //     if (LOOP_FREQ_SET(LOOP_50_Hz, Tick, LOOP_MAIN_RATE))
        //     {
        //         /*读取原始数据*/
        //         MPU6000_GetData(&imu_raw_data.acc, &imu_raw_data.gyro);
        //         MPU6000_Data_Conversion(&imu_cal_data.acc, &imu_cal_data.gyro, &imu_raw_data.acc, &imu_raw_data.gyro);
        //         /*后面的磁力计校准需要用到不需太精确的imu数据，这里直接将未校准的imu数据赋值给flight_control_state*/
        //         for (int i = 0; i < 3; i++)
        //         {
        //             flight_control_state.gyro_data.axis_arr[i] = imu_cal_data.gyro.axis_arr[i];
        //             flight_control_state.acc_data.axis_arr[i] = imu_cal_data.acc.axis_arr[i];
        //         }
        //         hmc5883l_read_raw_data(&mag_raw_data);
        //         hmc5883l_convert_to_gauss(&mag_cal_data, &mag_raw_data);
        //         MAG_Calibration(); /*获取校准参数*/
        //     }
        //     Tick++;
        // }
    }
}

/**
 * @brief  磁力计数据处理
 * @note
 * @param  无
 * @retval 无
 */
void MAG_Processing(MAG_CalData_t *mag_cal_data, MAG_GaussData_t *mag_gauss_data)
{
    // hmc5883l_read_raw_data(&mag_raw_data);
    /*将原始数据转换为物理数据*/
    // hmc5883l_convert_to_gauss(mag_cal_data, mag_raw_data);
    /*硬铁校准*/
    mag_cal_data->x = mag_gauss_data->x - mag_processing.offset_x; // 正确
    mag_cal_data->y = mag_gauss_data->y - mag_processing.offset_y; // 正确
    mag_cal_data->z = mag_gauss_data->z - mag_processing.offset_z; // 正确
}

/**
 * @brief  磁力计数据校准
 * @note
 * @param  无
 * @retval 无
 */
void MAG_Calibration(void)
{
    MAG_Calibration_Instrction();
    MAG_Calibration_Operation();
}

/**
 * @brief  地磁校准步骤指令更新，这一部分进行指令、流程控制
 * @note
 * @param  无
 * @retval 无
 */
static void MAG_Calibration_Instrction(void)
{
    switch (mag_processing_state)
    {
    /*系统处于待机或校准完成后的初始状态*/
    case 0:
        /*在开源飞控中，在scheduler中将程序从0状态更新到了1状态，开始了校准*/
        mag_processing.circle_angle_roll = 0;
        mag_processing.circle_angle_pitch = 0;
        mag_processing.circle_angle_yaw = 0;
        mag_processing.delay_count = 0;
        MAG_PROCESSING_DEBUG("磁力计等待校准状态：0\n");
        break;
    /*进入地磁校准状态，开始校准x、y轴*/
    case 1:
        mag_processing.circle_angle_yaw += flight_control_state.gyro_data.yaw * MAG_CALIBRATION_TIME; /*角速度*时间=角度，相当于积分*/
        /*判断旋转角度是否足够360度，这里判断条件是两圈半*/
        if (fabs(mag_processing.circle_angle_yaw) > 3 * M_PI)
        {
            mag_processing_state = 2;
        }
        MAG_PROCESSING_DEBUG("磁力计校准状态:1,将飞机进行水平旋转\n");
        break;
    /*收集完x、y轴一圈数据后，将飞机立起，判断飞机是否处于竖直状态*/
    case 2:
        mag_processing.circle_angle_roll = 0;
        mag_processing.circle_angle_pitch = 0;
        /*通过z轴重力加速度判断飞机是否处于竖直状态*/
        if (flight_control_state.acc_data.yaw < 1.5f && flight_control_state.acc_data.yaw > -1.5f)
        {
            mag_processing.delay_count++;
        }
        else
        {
            mag_processing.delay_count = 0;
        }
        /*竖直状态保持1s，防止误判*/
        if (mag_processing.delay_count > 50)
        {
            mag_processing_state = 3;
            mag_processing.delay_count = 0;
        }
        MAG_PROCESSING_DEBUG("磁力计校准状态:2,请将飞机竖起\n");
        break;
    /*飞机处于竖直状态，开始校准z轴*/
    case 3:
        mag_processing.circle_angle_roll += flight_control_state.gyro_data.roll * MAG_CALIBRATION_TIME;
        mag_processing.circle_angle_pitch += flight_control_state.gyro_data.pitch * MAG_CALIBRATION_TIME;
        if (fabs(mag_processing.circle_angle_roll) > 3 * M_PI || fabs(mag_processing.circle_angle_pitch) > 3 * M_PI)
        {
            mag_processing_state = 4;
        }
        MAG_PROCESSING_DEBUG("磁力计校准状态:3,垂直旋转飞机,校准z轴\n");
        break;
    case 4:
        /*延时1s，给系统处理时间*/
        mag_processing.delay_count++;
        /*如果x、y轴数据接近正圆形，则进入下一状态，这里只检查数据受软磁干扰的严重程度，上面几步并没有加入软磁干扰校准*/
        /*这里的50与函数调用频率有关，更改函数调用频率需要更改这里，下面还有几个同样需要更改*/
        if (mag_processing.delay_count > 50 && isCali_XYFactor_Check_Succeed())
        {
            mag_processing_state = 6; /*进入成功路径：4-6-0*/
            MAG_PROCESSING_DEBUG("磁力计校准状态:4,数据受软磁干扰较轻，进入下一状态\n");
            mag_processing.delay_count = 0;
        }
        else if (mag_processing.delay_count > 50 && !isCali_XYFactor_Check_Succeed())
        {
            mag_processing_state = 5; /*进入失败路径：4-5-0*/
            MAG_PROCESSING_DEBUG("磁力计校准状态:4,数据受软磁干扰较重，进入下一状态\n");
            mag_processing.delay_count = 0;
        }
        
        break;
    case 5:
        mag_processing_state = 0;
        MAG_PROCESSING_DEBUG("磁力计校准状态:5,校准失败\n");
        break;
    case 6:
        /*延时1s*/
        mag_processing.delay_count++;
        if (mag_processing.delay_count > 50)
        {
            mag_processing_state = 0;
        }
        MAG_PROCESSING_DEBUG("磁力计校准状态:6,校准完成\n");
        break;
    default:
        mag_processing_state = 0;
        break;
    }
}

/**
 * @brief  地磁校准具体操作，这一部分进行实际的校准操作
 * @note
 * @param  无
 * @retval 无
 */
static void MAG_Calibration_Operation(void)
{
    switch (mag_processing_state)
    {
    case 0:
        MAG_Calibration_Init();
        break;
    case 1:
        /*获取水平旋转过程中的x、y轴数据最大值最小值*/
        Get_Max_Min_Value_XY();
        break;
    case 2:
        /*计算x、y轴偏移量*/
        Calculate_Offset_XY();
        break;
    case 3:
        /*获取垂直旋转过程中的z轴数据最大值最小值*/
        Get_Max_Min_Value_Z();
        break;
    case 4:
        /*在这1s内*/
        /*计算z轴偏移量*/
        Calculate_Offset_Z();
        if (isCali_XYFactor_Check_Succeed())
        {
            /*保存参数，避免每次开机都重新校准*/
            Save_Calibration_Parameters();
            mag_processing.calibration_state = 1;
        }
        else
        {
            mag_processing.calibration_state = 0;
						MAG_Calibration_Reset();
        }
        break;
    }
}

/**
 * @brief  地磁校准初始化
 * @note
 * @param  无
 * @retval 无
 */
static void MAG_Calibration_Init(void)
{
    mag_processing.calibration_state = 0;
    mag_processing.max_value_x = -FLTOAT_MAX;
    mag_processing.min_value_x = FLTOAT_MAX;
    mag_processing.max_value_y = -FLTOAT_MAX;
    mag_processing.min_value_y = FLTOAT_MAX;
    mag_processing.max_value_z = -FLTOAT_MAX;
    mag_processing.min_value_z = FLTOAT_MAX;
}

/**
 * @brief  重置地磁校准
 * @note
 * @param  无
 * @retval 无
 */
static void MAG_Calibration_Reset(void)
{
}

/**
 * @brief  获取水平旋转过程中的x、y轴数据最大值最小值
 * @note
 * @param  无
 * @retval 无
 */
static void Get_Max_Min_Value_XY(void)
{
    if (flight_control_state.mag_data.roll > mag_processing.max_value_x)
    {
        mag_processing.max_value_x = flight_control_state.mag_data.roll;
    }
    if (flight_control_state.mag_data.roll < mag_processing.min_value_x)
    {
        mag_processing.min_value_x = flight_control_state.mag_data.roll;
    }
    if (flight_control_state.mag_data.pitch > mag_processing.max_value_y)
    {
        mag_processing.max_value_y = flight_control_state.mag_data.pitch;
    }
    if (flight_control_state.mag_data.pitch < mag_processing.min_value_y)
    {
        mag_processing.min_value_y = flight_control_state.mag_data.pitch;
    }
}

/**
 * @brief  获取z轴数据最大值最小值
 * @note
 * @param  无
 * @retval 无
 */
static void Get_Max_Min_Value_Z(void)
{
    if (flight_control_state.mag_data.yaw > mag_processing.max_value_z)
    {
        mag_processing.max_value_z = flight_control_state.mag_data.yaw;
    }
    if (flight_control_state.mag_data.yaw < mag_processing.min_value_z)
    {
        mag_processing.min_value_z = flight_control_state.mag_data.yaw;
    }
}

/**
 * @brief  计算x、y轴偏移量。通过计算数据云在x轴、y轴的中心点，这个中心点就是需要的硬铁偏移
 * @note
 * @param  无
 * @retval 无
 */
static void Calculate_Offset_XY(void)
{
    mag_processing.offset_x = (mag_processing.max_value_x + mag_processing.min_value_x) / 2;
    mag_processing.offset_y = (mag_processing.max_value_y + mag_processing.min_value_y) / 2;
}

/**
 * @brief  计算z轴偏移量
 * @note
 * @param  无
 * @retval 无
 */
static void Calculate_Offset_Z(void)
{
    mag_processing.offset_z = (mag_processing.max_value_z + mag_processing.min_value_z) / 2;
}

/**
 * @brief  检查x、y轴数据是否接近正圆形,软磁干扰下，数据云会变成椭圆形
 * @note
 * @param  无
 * @retval 是否成功
 */
bool isCali_XYFactor_Check_Succeed(void)
{
    /*x轴宽*/
    float x_width = mag_processing.max_value_x - mag_processing.min_value_x;
    /*y轴宽*/
    float y_width = mag_processing.max_value_y - mag_processing.min_value_y;
    /*排除无效数据*/
    if (x_width == 0 || y_width == 0)
    {
        return false;
    }
    /*两宽比值*/
    float xy_factor = x_width / y_width;
    /*两宽比值小于1.1*/
    if (fabs(xy_factor) - 1 < 0.2f)
    {
        return true;
    }
    return false;
}

/**
 * @brief  保存校准参数
 * @note
 * @param  无
 * @retval 无
 */
static void Save_Calibration_Parameters(void)
{
    /**/
    mag_calibration_params_t cal_params;
    cal_params.magic_key = MAG_CALIBRATION_PARAMS_MAGIC_KEY;
    // 将 float 的二进制表示直接复制到 u32 成员中，不损失任何精度
    memcpy(&cal_params.offset_x, &mag_processing.offset_x, sizeof(float));
    memcpy(&cal_params.offset_y, &mag_processing.offset_y, sizeof(float));
    memcpy(&cal_params.offset_z, &mag_processing.offset_z, sizeof(float));
    STMFLASH_Write(ADDR_FLASH_SECTOR_11 + 4, (u32*)&cal_params, sizeof(mag_calibration_params_t)/4);
    printf("保存校准参数:x=%f,y=%f,z=%f\n", mag_processing.offset_x, mag_processing.offset_y, mag_processing.offset_z);
}

/**
 * @brief  读取校准参数
 * @note
 * @param  无
 * @retval 无
 */
static bool Read_Calibration_Parameters(void)
{
    mag_calibration_params_t cal_params;
    STMFLASH_Read(ADDR_FLASH_SECTOR_11 + 4, (u32*)&cal_params, sizeof(mag_calibration_params_t)/4);
    if (cal_params.magic_key != MAG_CALIBRATION_PARAMS_MAGIC_KEY)
    {
        return false;
    }
    // 将 u32 成员中的二进制内容，原封不动地复制回 float 变量
    memcpy(&mag_processing.offset_x, &cal_params.offset_x, sizeof(float));
    memcpy(&mag_processing.offset_y, &cal_params.offset_y, sizeof(float));
    memcpy(&mag_processing.offset_z, &cal_params.offset_z, sizeof(float));
    printf("读取校准参数:x=%f,y=%f,z=%f\n", mag_processing.offset_x, mag_processing.offset_y, mag_processing.offset_z);
    // mag_processing.offset_x = (float)cal_params.offset_x / 1000;
    // mag_processing.offset_y = (float)cal_params.offset_y / 1000;
    // mag_processing.offset_z = (float)cal_params.offset_z / 1000;
    return true;
}
