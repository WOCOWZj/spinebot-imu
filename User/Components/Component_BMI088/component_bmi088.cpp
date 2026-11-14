//
// Created by 92428 on 2024/8/29.
//

#include "component_bmi088.h"
#include "iwdg.h"
#include "math.h"

Component_BMI088::Component_BMI088() {

}

void Component_BMI088::update(BSP_Pack &BSP_Instance, bool IS_GYRO_UPDATE, bool IS_ACCEL_UPDATE) {
    /*if(BSP_Instance.imu.survive==false)
        return;//imu断连则拒绝继续更新*/
    t_task_start = BSP_Instance.time_stamp_us.now_micros();

    BSP_Instance.imu.Get_Gyro_DMA();
    vector_gyro_original = {BSP_Instance.imu.Gyro.x - Gyro_Offset[0], BSP_Instance.imu.Gyro.y - Gyro_Offset[1], BSP_Instance.imu.Gyro.z - Gyro_Offset[2]};
    BSP_Instance.imu.Get_Accel_DMA();
    vector_accel_original = {BSP_Instance.imu.Acc.x , BSP_Instance.imu.Acc.y , BSP_Instance.imu.Acc.z};

    // Init
    if(!is_init_ekf){
        if(IS_ACCEL_UPDATE){
            vector_accel_normalized = vector_accel_original.normalize();
            Quaternion q_accel_init = Gravity_Direction_To_Quaternion(vector_accel_normalized);
            ekf.setState(q_accel_init);
            last_ekf_predict_time = BSP_Instance.time_stamp_us.now_micros();
            ekf.dt_ = 0;
            ekf.gyro_omega_original = Vectorxf<3>({0,0,0});
            ekf.predict();
            is_init_ekf = 1;
        }
        t_task_finish = BSP_Instance.time_stamp_us.now_micros();
        return;
    }

    // Predict
    t_predict_start = BSP_Instance.time_stamp_us.now_micros();
    ekf.gyro_omega_original = vector_gyro_original;
    now_ekf_predict_time = t_predict_start;
    ekf.dt_ = BSP_Instance.time_stamp_us.time_diff_s(now_ekf_predict_time,last_ekf_predict_time);
    ekf.predict();
    last_ekf_predict_time = now_ekf_predict_time;
    t_predict_finish = BSP_Instance.time_stamp_us.now_micros();

    // Update
    if(IS_ACCEL_UPDATE){
        t_update_start = BSP_Instance.time_stamp_us.now_micros();

        vector_accel_normalized = vector_accel_original.normalize();
        modulus_square_accel = vector_accel_original.modulus_square();

        chi_square_loss = ekf.chi_square_test(vector_accel_normalized, modulus_square_accel);

        is_update = (chi_square_loss < thresh_accel_chi_square_loss) && (fabs(modulus_square_accel - 96.0f) < thresh_accel_modulus_square_loss);
        HAL_IWDG_Refresh(&hiwdg1);
        if(is_update){
            ekf_state = ekf.update(vector_accel_normalized, vector_mean_accel_world.modulus_square());
        }

        ekf_state = ekf.getState();
        vector_accel_world = ekf_state.toRotationMatrix() * vector_accel_original;
        ekf_state.toEulerAngle(roll,pitch,yaw);

        t_update_finish = BSP_Instance.time_stamp_us.now_micros();
    }

    // Filter Gyro
    if(IS_GYRO_UPDATE){
        vector_gyro_filtered = {gyro_filter_butterworth[0].Update(vector_gyro_original[0]), gyro_filter_butterworth[1].Update(vector_gyro_original[1]), gyro_filter_butterworth[2].Update(vector_gyro_original[2])};
        gyro_static_slide_statistics[0].Update(vector_gyro_original[0]);
        gyro_static_slide_statistics[1].Update(vector_gyro_original[1]);
        gyro_static_slide_statistics[2].Update(vector_gyro_original[2]);
        vector_mean_gyro_original = {gyro_static_slide_statistics[0].GetMean(), gyro_static_slide_statistics[1].GetMean(), gyro_static_slide_statistics[2].GetMean()};
        vector_var_gyro_original = {gyro_static_slide_statistics[0].GetVariance(), gyro_static_slide_statistics[1].GetVariance(), gyro_static_slide_statistics[2].GetVariance()};
    }

    // Filter Accel world
    if(IS_ACCEL_UPDATE){
        accel_slide_statistics[0].Update(vector_accel_world[0]);
        accel_slide_statistics[1].Update(vector_accel_world[1]);
        accel_slide_statistics[2].Update(vector_accel_world[2]);
        vector_mean_accel_world = {accel_slide_statistics[0].GetMean(), accel_slide_statistics[1].GetMean(), accel_slide_statistics[2].GetMean()};
        vector_var_accel_world = {accel_slide_statistics[0].GetVariance(), accel_slide_statistics[1].GetVariance(), accel_slide_statistics[2].GetVariance()};

        accel_static_slide_statistics[0].Update(vector_accel_world[0]);
        accel_static_slide_statistics[1].Update(vector_accel_world[1]);
        accel_static_slide_statistics[2].Update(vector_accel_world[2]);
    }

    if(is_online_calibrate){
        update_is_static();
        static int skip_static = 0;
        if(is_static){
            if(!skip_static){
                Vectorxf<3> average_gyro = Vectorxf<3>({gyro_static_slide_statistics[0].GetMean(), gyro_static_slide_statistics[1].GetMean(), gyro_static_slide_statistics[2].GetMean()});
                average_gyro *= 0.004f;
                Gyro_Offset += average_gyro;
                skip_static = gyro_static_slide_statistics[0].GetBufferSize();
            }
        } else{
            skip_static = gyro_static_slide_statistics[0].GetBufferSize();
        }
        if(skip_static > 0){
            skip_static--;
        }
    }

    t_task_finish = BSP_Instance.time_stamp_us.now_micros();
}


void Component_BMI088::temperature_control(BSP_Pack &BSP_Instance) {
    BSP_Instance.imu.Get_Temp_DMA();
    temperature = BSP_Instance.imu.tempretature;
    is_temp_update = 1;
    //float vbat = BSP_Instance.vbat.GetVoltage();
    float P_out = 0, I_out = 0, D_out = 0;
    err = target_temperature - temperature;
    if (abs(err) < Deadzone) {
        err = 0;
    }
    P_out=KP*err;
    if(abs(err)<error_thresh)
    {
        integral+=err*0.05;//目前是100Hz控制
        // 积分限幅防止饱和
        if (integral > integral_max) integral = integral_max;
        if (integral < -integral_max) integral = -integral_max;
    }
    else
    {
        integral=0;
    }
    I_out=KI*integral;

    err_ll = err_l;
    err_l = err;
    D_out=KD * (err - err_l);
    //out = KP * err + KI * integral + KD * (err - err_l);
    out=P_out+I_out+D_out;
    //out = out / (vbat * vbat) * 25 * 25;
    out = out * 5;
    BSP_Instance.heater.Set(out);
}

void Component_BMI088::update_is_static() {
    float accel_var = accel_static_slide_statistics[0].GetVariance() +
                      accel_static_slide_statistics[1].GetVariance() +
                      accel_static_slide_statistics[2].GetVariance();
    float gyro_var = gyro_static_slide_statistics[0].GetVariance() +
                     gyro_static_slide_statistics[1].GetVariance() +
                     gyro_static_slide_statistics[2].GetVariance();

    accel_is_static_loss = accel_var;
    gyro_is_static_loss = gyro_var;

    if(accel_is_static_loss < thresh_accel_is_static && accel_is_static_loss != 0
        && gyro_is_static_loss < thresh_gyro_is_static && gyro_is_static_loss != 0){
        is_static = 1;
    } else{
        is_static = 0;
    }
}

