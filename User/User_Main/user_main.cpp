//
// Created by 92428 on 2024/8/13.
//

#include "user_main.h"
#include "main.h"
#include "bsp.h"
#include "component_bmi088.h"
#include "SCH16t.h"

extern BSP_Pack BSP_Instance;
extern Component_BMI088 bmi088;

void User_Setup() {
    HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);

    BSP_Instance.Init();
    BSP_Instance.led.Change_Color(6,2,11);
    BSP_Instance.led.On();

    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    uint8_t _read_status = BSP_Instance.flash.Read_3Float_WithCRC(gyro_offset_x, gyro_offset_y, gyro_offset_z);
    if(_read_status) {
        // Read successful, set gyro offsets
        bmi088.Gyro_Offset = Vectorxf<3>({gyro_offset_x, gyro_offset_y, gyro_offset_z});
    } else {
        // Read failed, set default offsets
        bmi088.Gyro_Offset = Vectorxf<3>({0.0, 0.0, 0.0});
    }

    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
}

struct IMU_RAW{
    float ax;       // 0
    float ay;       // 1
    float az;       // 2
    float wx;       // 3
    float wy;       // 4
    float wz;       // 5
    float temp;     // 6
    float time_communicate_us;
    unsigned char tail[4]{0x00, 0x00, 0x80, 0x7f};
};

static bool imu_ok = false;

//下面这个也可以检测但是好像有一些问题
void IMU_CheckOnline(void)
{
    SCH1_status status;
    int ret = SCH1_getStatus(&status);
    if (ret != SCH1_OK) {
        // SPI 读状态寄存器都失败了，基本可以认为断连/严重故障
        imu_ok = false;
        return;
    }

    // 状态寄存器不是全 0xFFFF，说明内部报错或通信异常
    if (!SCH1_verifyStatus(&status)) {
        imu_ok = false;
        return ;
    }

    imu_ok = true;
    return ;
}

void User_Loop()
{
//    static SCH1_raw_data SCH1_data;
//    static IMU_RAW imu_buffer_raw_;
//    static uint32_t num_samples = 0;
//    static uint64_t time_start,time_end;
//
//    time_start = BSP_Instance.time_stamp_us.now_micros();
//    SCH1_getData(&SCH1_data, true, true, true);
//    time_end = BSP_Instance.time_stamp_us.now_micros();
//
//    SCH1_result Data;
//    SCH1_convert_data(&SCH1_data, &Data);
//
//    imu_buffer_raw_.ax = Data.Acc1[AXIS_X];
//    imu_buffer_raw_.ay = Data.Acc1[AXIS_Y];
//    imu_buffer_raw_.az = Data.Acc1[AXIS_Z];
//    imu_buffer_raw_.wx = Data.Rate1[AXIS_X];
//    imu_buffer_raw_.wy = Data.Rate1[AXIS_Y];
//    imu_buffer_raw_.wz = Data.Rate1[AXIS_Z];
//    imu_buffer_raw_.temp = Data.Temp;
//    imu_buffer_raw_.time_communicate_us = (float)(time_end - time_start);
    //IMU_CheckOnline();
    if(BSP_Instance.imu.survive==false)
    {
        BSP_Instance.led.Off();
        BSP_Instance.imu.Init();
        HAL_Delay(5);
    }
    else{
    BSP_Instance.led.Off();
    HAL_Delay(500);
    BSP_Instance.led.On();
    HAL_Delay(500);
    }
}