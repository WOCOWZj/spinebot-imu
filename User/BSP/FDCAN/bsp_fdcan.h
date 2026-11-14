//
// Created by Yanhong Liu on 2025/7/19.
//

#ifndef H7_IMU_BSP_FDCAN_H
#define H7_IMU_BSP_FDCAN_H

#include "fdcan.h"
#include "stm32h7xx_hal_fdcan.h"

typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} FDCAN_RxFrame_TypeDef;

typedef struct {
    FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header;
    uint8_t Data[8];
} FDCAN_TxFrame_TypeDef;

class FDCAN_Manager {
public:
    int flag = 0;
    int pre_flag = 0;

    void Init(FDCAN_HandleTypeDef *_hfdcan);

    void Send(uint16_t ID, const uint8_t *Data, uint16_t Length);

    FDCAN_RxFrame_TypeDef RxFrame;
    FDCAN_TxFrame_TypeDef TxFrame;
    FDCAN_HandleTypeDef *hfdcan;
    FDCAN_FilterTypeDef FDCAN_FilterConfig;
};


#define ACCEL_CAN_MAX (58.8f)
#define ACCEL_CAN_MIN (-58.8f)
#define GYRO_CAN_MAX (34.88f)
#define GYRO_CAN_MIN (-34.88f)
#define PITCH_CAN_MAX (90.0f)
#define PITCH_CAN_MIN (-90.0f)
#define ROLL_CAN_MAX (180.0f)
#define ROLL_CAN_MIN (-180.0f)
#define YAW_CAN_MAX (180.0f)
#define YAW_CAN_MIN (-180.0f)
#define TEMP_MIN (0.0f)
#define TEMP_MAX (60.0f)
#define Quaternion_MIN (-1.0f)
#define Quaternion_MAX (1.0f)
/**
************************************************************************
* @brief:
float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:
x_float:
待转换的浮点数
* @param[in]:
x_min:
范围最小值
* @param[in]:
x_max:
范围最大值
* @param[in]:
bits:
目标无符号整数的位数
* @retval:
无符号整数结果
* @details:
将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个
指定位数的无符号整数
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits);
/**
************************************************************************
* @brief:
uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:
x_int: 待转换的无符号整数
* @param[in]:
x_min: 范围最小值
* @param[in]:
x_max: 范围最大值
* @param[in]:
bits:
无符号整数的位数
DM-IMU-L1 六轴 IMU 模块使用说明书 V1.1
第 17 页 共 22 页
* @retval:
浮点数结果
* @details:
将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结
果为一个浮点数
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits);

extern FDCAN_Manager FDCAN1_Manager_Obj;

void IMU_SendAccel(float accel_x, float accel_y, float accel_z) ;

/**
 * @brief 发送陀螺仪数据 (类型2)
 * @param gyro_x X轴角速度 (rad/s)
 * @param gyro_y Y轴角速度 (rad/s)
 * @param gyro_z Z轴角速度 (rad/s)
 */
void IMU_SendGyro(float gyro_x, float gyro_y, float gyro_z) ;

/**
 * @brief 发送欧拉角数据 (类型3)
 * @param pitch 俯仰角 (度)
 * @param yaw 偏航角 (度)
 * @param roll 横滚角 (度)
 */
void IMU_SendEuler(float pitch, float yaw, float roll) ;

#endif //H7_IMU_BSP_FDCAN_H
