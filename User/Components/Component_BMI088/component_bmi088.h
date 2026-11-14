//
// Created by 92428 on 2024/8/29.
//

#ifndef H7_IMU_COMPONENT_BMI088_H
#define H7_IMU_COMPONENT_BMI088_H

#include "bsp.h"
#include "alg_matrix.h"
#include "alg_ekf_quaternion.h"
#include "alg_quaternion.h"
#include "alg_butterworthfIlter.h"
#include "alg_slide_statistics.h"

class  Component_BMI088{
public:
    Component_BMI088();
    void update(BSP_Pack & BSP_Instance, bool is_gyro_update, bool is_accel_update);
    void temperature_control(BSP_Pack & BSP_Instance);

    Vectorxf<3> vector_gyro_original;
    Vectorxf<3> vector_gyro_filtered;
    Vectorxf<3> vector_accel_original;
    Vectorxf<3> vector_accel_normalized;
    Vectorxf<3> vector_accel_world;

    Vectorxf<3> vector_mean_accel_world;
    Vectorxf<3> vector_var_accel_world;

    Vectorxf<3> vector_mean_gyro_original;
    Vectorxf<3> vector_var_gyro_original;

    Vectorxf<3> Gyro_Offset = Vectorxf<3>({0.0,0.0,0.0});

    Quaternion ekf_state;

    int is_init_ekf = 0;
    float modulus_square_accel;

    float temperature;

    float roll,pitch,yaw;

    bool is_update = false;

    bool is_online_calibrate = false;

    int skip_update_cnt = 0;

    uint64_t t_task_start, t_task_finish,
            t_read_gyro_start,t_predict_start,t_predict_finish,
            t_read_accel_start,t_update_start,t_update_finish,
            t_calibrate_start,t_calibrate_finish = 0;

    uint8_t  is_gyro_update,is_accel_update,is_temp_update = 0;

    Alg_ButterWorthFilter gyro_filter_butterworth[3];
    Alg_ButterWorthFilter accel_filter_butterworth[3];

    alg_slide_statistics<20> accel_slide_statistics[3];

    alg_slide_statistics<200> gyro_static_slide_statistics[3];
    alg_slide_statistics<160> accel_static_slide_statistics[3];

    Alg_Ekf_Quaternion ekf;

    float chi_square_loss;

    float accel_is_static_loss;
    uint8_t is_static = 0;
    float gyro_is_static_loss;

private:
    float thresh_accel_chi_square_loss = 5.0;
    float thresh_accel_modulus_square_loss = 7.68f; // about 2 degree error
//    Vectorxf<3> Gyro_Offset = Vectorxf<3>({-0.000503,-0.002688,-0.000821});
//    Vectorxf<3> Gyro_Offset = Vectorxf<3>({-0.000666892389,-0.00136541622,-0.00249972451});

    float target_temperature = 40;
    float KP = 500.0f,KI = 10.0f,KD = 0.0f;
    float Deadzone=0.05f;
    float integral_max=500.0f;
    float error_thresh=1.0f;

    float thresh_accel_is_static = 0.1;
    float thresh_gyro_is_static = 0.05;
    int cnt = -1;
    uint64_t last_ekf_predict_time = 0;
    uint64_t now_ekf_predict_time = 0;

    uint32_t gyro_data_number = 0;
    Vectorxf<3> vector_gyro_average = Vectorxf<3>({0,0,0});

    float err,err_l,err_ll,out = 0,integral=0;
    void update_gyro_average();

    void update_is_static();


    struct Vector3_Data_{
        float x;
        float y;
        float z;};

    int accel_buffer_size = 40;
    Vector3_Data_ accel_data_buffer[40];
    int accel_data_buffer_index = 0;
    int gyro_buffer_size = 100;
    Vector3_Data_ gyro_data_buffer[100];
    int gyro_data_buffer_index = 0;

};


#endif //H7_IMU_COMPONENT_BMI088_H
