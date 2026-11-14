//
// Created by 92428 on 2024/8/13.
//
#include "tim_callback.h"
#include "component_bmi088.h"
#include "usart.h"

#define ACTION_RESET (0x00)
#define ACTION_UPDATE_OFFSET_FROM_FLASH (0x01)
#define ACTION_START_ONLINE_CALIBRATE (0x02)
#define ACTION_STOP_ONLINE_CALIBRATE (0x03)
#define ACTION_WRITE_CURRENT_OFFSET_TO_FLASH (0x04)

extern BSP_Pack BSP_Instance;
void BSP_TimeStamp_Tim16Overflow();

typedef struct __attribute__((packed)){
    uint8_t head;
    uint8_t head2;
    uint8_t canid;
    uint8_t id;
    float var1;
    float var2;
    float var3;
    uint16_t crc;
    uint8_t tail;
}IMU_uart_Data;
IMU_uart_Data IMU_uart_Data1, IMU_uart_Data2;

uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void crc16_update(uint16_t *current_crc, const uint8_t *src, uint32_t len){
    uint32_t crc = *current_crc;
    uint32_t j = 0;
    for (j = 0; j < len; j++) {
        crc ^= (uint32_t)src[j] << 8;
        for (uint32_t i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    *current_crc = crc;
}


typedef struct __attribute__((packed)){
    uint16_t header;
    uint16_t len;
    uint16_t crc;
    uint8_t tag;
    uint8_t is_calibrate_gyro;
    uint8_t is_static;
    int8_t  temp;
    float prs;
    uint32_t ts;
    float acc[3];
    float gyr[3];
    float offset[3];
    float eul[3];
    float quat[4];
}IMU_Data;



struct IMU_Result{
    float roll;     // 0
    float pitch;    // 1
    float yaw;      // 2
    float ax;       // 3
    float ay;       // 4
    float az;       // 5
    float wx;       // 6
    float wy;       // 7
    float wz;       // 8
    float q0;       // 9
    float q1;       // 10
    float q2;       // 11
    float q3;       // 12
    float temp;     // 13
    float t_update; // 14
    float t_predict;// 15
    float loss;     // 16
    float x_norm;   // 17
    float static_loss;//18
    float is_static;// 19
    float gyro_static_loss; // 20
    float accel_static_loss;//21
    float cnt_spi_error;    // 22
    float cnt_acc;  // 23
    float cnt_gyro; // 24
    float t_total_us_in_ms; //25
    float reserve_1;// 26
    float reserve_2;// 27
    float reserve_3;// 28
    float reserve_4;// 29
    float reserve_5;// 30
    unsigned char tail[4]{0x00, 0x00, 0x80, 0x7f};
};

IMU_Data usb_imu_data;
IMU_Result imu_result_data;
uint8_t uart_tx_buf[38] = {};

Component_BMI088 bmi088;

float task_time_buffer[8] = {0.0f};
float task_time_in_ms = 0.0f;

uint8_t Buffer_Send[32] = {0};


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    // 8khz
    if(htim == &htim6)
    {
        static uint8_t counter = 0;
//        bmi088.update(BSP_Instance, bmi088.is_gyro_update, bmi088.is_accel_update);
//        bmi088.update(BSP_Instance, BSP_Instance.imu.gyro_update_flag, BSP_Instance.imu.accel_update_flag);
        if(counter % 2 == 0){
            BSP_Instance.imu.Read_SCH16T(true, true, false);
        }else{
            if(counter == 1){
                BSP_Instance.imu.Read_SCH16T(true, false, true);
            }else{
                BSP_Instance.imu.Read_SCH16T(true, false, false);
            }
        }

        bmi088.update(BSP_Instance, (BSP_Instance.imu.gyro_update_flag & (1 << IMU_UPDATE_SHFITS)), (BSP_Instance.imu.accel_update_flag & (1 << IMU_UPDATE_SHFITS)));

        BSP_Instance.imu.gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
        BSP_Instance.imu.accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);

        task_time_in_ms -= task_time_buffer[counter];
        task_time_buffer[counter] = BSP_Instance.time_stamp_us.time_diff_s(bmi088.t_task_finish, bmi088.t_task_start) * 1000000.0f;
        task_time_in_ms += task_time_buffer[counter];

        if (counter == 0)
        {   if(BSP_Instance.imu.survive==true)
            {
                // 这一部分代码是vofa+调试用的，不调试时不发送
                imu_result_data.roll = bmi088.roll;
                imu_result_data.pitch = bmi088.pitch;
                imu_result_data.yaw = bmi088.yaw;
                imu_result_data.q0 = bmi088.ekf_state[0];
                imu_result_data.q1 = bmi088.ekf_state[1];
                imu_result_data.q2 = bmi088.ekf_state[2];
                imu_result_data.q3 = bmi088.ekf_state[3];
                imu_result_data.ax = bmi088.vector_accel_original[0];
                imu_result_data.ay = bmi088.vector_accel_original[1];
                imu_result_data.az = bmi088.vector_accel_original[2];
                imu_result_data.wx = bmi088.vector_gyro_original[0];
                imu_result_data.wy = bmi088.vector_gyro_original[1];
                imu_result_data.wz = bmi088.vector_gyro_original[2];
                imu_result_data.temp = BSP_Instance.imu.tempretature;
                imu_result_data.t_update = BSP_Instance.time_stamp_us.time_diff_s(bmi088.t_update_finish,bmi088.t_update_start) * 1000000;
                imu_result_data.t_predict = BSP_Instance.time_stamp_us.time_diff_s(bmi088.t_predict_finish,bmi088.t_predict_start) * 1000000;
                imu_result_data.t_total_us_in_ms = task_time_in_ms;
                imu_result_data.loss = bmi088.chi_square_loss;
                imu_result_data.x_norm = bmi088.ekf_state.norm();
                imu_result_data.static_loss = bmi088.accel_is_static_loss;
                imu_result_data.is_static = bmi088.is_static;
                imu_result_data.gyro_static_loss = bmi088.gyro_is_static_loss;
                imu_result_data.accel_static_loss = bmi088.accel_is_static_loss;

                imu_result_data.cnt_spi_error = BSP_Instance.imu.CNT_SPI_DMA_Error;
                imu_result_data.cnt_acc = BSP_Instance.imu.CNT_ACC;
                imu_result_data.cnt_gyro = BSP_Instance.imu.CNT_Gyro;

    //            imu_result_data.reserve_1 = bmi088.vector_mean_accel_world.norm();
    //            imu_result_data.reserve_2 = bmi088.vector_mean_accel_world[0];
    //            imu_result_data.reserve_3 = bmi088.vector_mean_accel_world[1];
    //            imu_result_data.reserve_4 = bmi088.vector_mean_accel_world[2];

                imu_result_data.reserve_1 = bmi088.Gyro_Offset[0];
                imu_result_data.reserve_2 = bmi088.is_update;
                imu_result_data.reserve_3 = bmi088.modulus_square_accel;
                imu_result_data.reserve_4 = bmi088.chi_square_loss;
    //            imu_result_data.reserve_4 = bmi088.vector_var_accel_world[2];
                imu_result_data.reserve_5 = bmi088.ekf.dt_;
            }

            if(BSP_Instance.imu.survive==true)
            {
                usb_imu_data.header = 0xA55A;
                usb_imu_data.len = 0x4C;

                usb_imu_data.tag = 0x91;
                usb_imu_data.is_static = bmi088.is_static;
                usb_imu_data.is_calibrate_gyro = bmi088.is_online_calibrate;
                usb_imu_data.temp = (int8_t) bmi088.temperature;
                usb_imu_data.prs = 0.0f;
                usb_imu_data.ts = BSP_Instance.time_stamp_us.now_micros() / 1000;
                usb_imu_data.acc[0] = bmi088.vector_accel_original[0];
                usb_imu_data.acc[1] = bmi088.vector_accel_original[1];
                usb_imu_data.acc[2] = bmi088.vector_accel_original[2];
                usb_imu_data.gyr[0] = bmi088.vector_gyro_original[0];
                usb_imu_data.gyr[1] = bmi088.vector_gyro_original[1];
                usb_imu_data.gyr[2] = bmi088.vector_gyro_original[2];
                usb_imu_data.offset[0] = bmi088.Gyro_Offset[0];
                usb_imu_data.offset[1] = bmi088.Gyro_Offset[1];
                usb_imu_data.offset[2] = bmi088.Gyro_Offset[2];
                usb_imu_data.eul[0] = bmi088.roll;
                usb_imu_data.eul[1] = bmi088.pitch;
                usb_imu_data.eul[2] = bmi088.yaw;
                Quaternion quat = bmi088.ekf_state.normalize();
                usb_imu_data.quat[0] = quat[0];
                usb_imu_data.quat[1] = quat[1];
                usb_imu_data.quat[2] = quat[2];
                usb_imu_data.quat[3] = quat[3];

                uint16_t temp_crc = 0x00;
                crc16_update(&temp_crc, (uint8_t *) &usb_imu_data, 4);
                crc16_update(&temp_crc, (uint8_t *) &usb_imu_data + 6, sizeof(usb_imu_data) - 6); // 计算CRC，排除头部和尾部
                usb_imu_data.crc = temp_crc;


            }

//            SCB_CleanDCache_by_Addr((uint32_t *)&imu_result_data, 128);
//            HAL_UART_Transmit_DMA(&huart10, (uint8_t *)&imu_result_data, sizeof(imu_result_data));
//            BSP_Instance.usb.Add_Data((uint8_t *) &imu_result_data, sizeof(imu_result_data), 0);
//            BSP_Instance.usb.Send_Data();
            BSP_Instance.usb.Add_Data((uint8_t *) &usb_imu_data, sizeof(usb_imu_data), 0);
            BSP_Instance.usb.Send_Data();
        }else if(counter == 4) {

            auto gyroX_send = (int16_t)(bmi088.vector_gyro_original[0] * 32767.0f / 35.0f);
            auto gyroY_send = (int16_t)(bmi088.vector_gyro_original[1] * 32767.0f / 35.0f);
            auto gyroZ_send = (int16_t)(bmi088.vector_gyro_original[2] * 32767.0f / 35.0f);

            auto yaw_send = (int32_t)(bmi088.yaw * 8388607.0f / 3.1415926f);  // 24bit
            auto yaw_u = (uint32_t)(yaw_send & 0xFFFFFF);  // 取低24位

            auto pitch_send = (int32_t)(bmi088.pitch * 524287.0f / 3.1415926f);  // 20bit
            auto roll_send  = (int32_t)(bmi088.roll  * 524287.0f / 3.1415926f);  // 20bit
            auto pitch_u = (uint32_t)(pitch_send & 0xFFFFF);
            auto roll_u  = (uint32_t)(roll_send  & 0xFFFFF);

            Buffer_Send[2] = (gyroX_send >> 8) & 0xFF;
            Buffer_Send[3] = gyroX_send & 0xFF;
            Buffer_Send[4] = (gyroY_send >> 8) & 0xFF;
            Buffer_Send[5] = gyroY_send & 0xFF;
            Buffer_Send[6] = (gyroZ_send >> 8) & 0xFF;
            Buffer_Send[7] = gyroZ_send & 0xFF;

            Buffer_Send[8]  = (yaw_u >> 16) & 0xFF;
            Buffer_Send[9]  = (yaw_u >> 8)  & 0xFF;
            Buffer_Send[10] = yaw_u & 0xFF;

            Buffer_Send[11] = (pitch_u >> 12) & 0xFF;
            Buffer_Send[12] = (pitch_u >> 4) & 0xFF;
            Buffer_Send[13] = ((pitch_u & 0x0F) << 4) | ((roll_u >> 16) & 0x0F);  // 拼接4bit+4bit

            Buffer_Send[14] = (roll_u >> 8) & 0xFF;
            Buffer_Send[15] = roll_u & 0xFF;

            Buffer_Send[0] = 0xAB;
            Buffer_Send[1] = crc8(Buffer_Send + 2, 14);

            SCB_CleanDCache_by_Addr((uint32_t *)&Buffer_Send, 32);
//            HAL_UART_Transmit_DMA(&huart7, Buffer_Send, 16);
            if(BSP_Instance.imu.survive==true)
            {
                HAL_UART_Transmit_DMA(&huart3, Buffer_Send, 16); // 发送数据
            }
        }else if(counter == 6){
            USB_Callback_Process();
        }else if(counter == 5){
            if(BSP_Instance.imu.survive==true) {
                IMU_SendGyro(bmi088.vector_gyro_filtered[0], bmi088.vector_gyro_filtered[1],
                             bmi088.vector_gyro_filtered[2]);
            }
        }else if(counter == 1){
            if(BSP_Instance.imu.survive==true) {
                IMU_SendEuler(bmi088.pitch, bmi088.yaw, bmi088.roll);
            }
        }


        counter = (counter + 1) % 8;

//        static uint8_t cnt = 0;
//        if(cnt == 15){
//            cnt = 0;
//            IMU_uart_Data1.head = 0x55;
//            IMU_uart_Data1.head2 = 0xAA;
//            IMU_uart_Data1.canid = 0x01;
//            IMU_uart_Data1.id = 0x02;
//            IMU_uart_Data1.var1 = -bmi088.vector_gyro_original[1];
//            IMU_uart_Data1.var2 = bmi088.vector_gyro_original[0];
//            IMU_uart_Data1.var3 = bmi088.vector_gyro_original[2];
//            IMU_uart_Data1.crc = Get_CRC16((uint8_t *)&IMU_uart_Data1, 16);
//            IMU_uart_Data1.tail= 0x0A;
//
//            IMU_uart_Data2.head = 0x55;
//            IMU_uart_Data2.head2 = 0xAA;
//            IMU_uart_Data2.canid = 0x01;
//            IMU_uart_Data2.id = 0x03;
//            IMU_uart_Data2.var1 = bmi088.pitch / PI * 180;
//            IMU_uart_Data2.var2 = bmi088.roll/ PI * 180;
//            IMU_uart_Data2.var3 = bmi088.yaw/ PI * 180;
//            IMU_uart_Data2.crc = Get_CRC16((uint8_t *)&IMU_uart_Data2, 16);
//            IMU_uart_Data2.tail = 0x0A;
//
//            memcpy(uart_tx_buf, &IMU_uart_Data1, 19);
//            memcpy(uart_tx_buf + 19, &IMU_uart_Data2, 19);
//            SCB_CleanDCache_by_Addr((uint32_t *)uart_tx_buf, 64);
//
//            HAL_UART_Transmit_DMA(&huart2, uart_tx_buf, 38); // 发送数据
//
//        }else{
//            cnt++;
//        }

        bmi088.is_temp_update = 0;
        bmi088.is_accel_update = 0;
        bmi088.is_gyro_update = 0;
    }

    // 100hz
    if(htim == &htim7)
    {
        if(BSP_Instance.imu.survive==true) {
            BSP_Instance.heater.MAX_OUT = 8000.0f;
        } else{
            BSP_Instance.heater.MAX_OUT = 2000.0f;
        }
        bmi088.temperature_control(BSP_Instance);
    }

    if(htim == &htim16)
    {
        BSP_TimeStamp_Tim16Overflow();
    }
}

uint8_t is_usb_recv_update = false;
uint8_t usb_recv_buffer[32];
uint32_t usb_recv_len = 0;

void USB_Callback_Process(){
    if(is_usb_recv_update){
        if(usb_recv_len == 16 && usb_recv_buffer[0] == 0xAB){
            uint8_t Mode = usb_recv_buffer[1];
            if(Mode == ACTION_RESET){
                BSP_Instance.software_reset.SoftWareReset();
            }
            else if(Mode == ACTION_UPDATE_OFFSET_FROM_FLASH){
                float gyro_offset_x, gyro_offset_y, gyro_offset_z;
                uint8_t _read_status = BSP_Instance.flash.Read_3Float_WithCRC(gyro_offset_x, gyro_offset_y, gyro_offset_z);
                if(_read_status) {
                    // Read successful, set gyro offsets
                    bmi088.Gyro_Offset = Vectorxf<3>({gyro_offset_x, gyro_offset_y, gyro_offset_z});
                }else{
                    bmi088.Gyro_Offset = Vectorxf<3>({0.0f, 0.0f, 0.0f});
                }
            }else if(Mode == ACTION_START_ONLINE_CALIBRATE){
                bmi088.is_online_calibrate = true;
            }else if(Mode == ACTION_STOP_ONLINE_CALIBRATE){
                bmi088.is_online_calibrate = false;
                bmi088.is_static = false;
            }else if(Mode == ACTION_WRITE_CURRENT_OFFSET_TO_FLASH){
                BSP_Instance.flash.Write_3Float_WithCRC(bmi088.Gyro_Offset[0], bmi088.Gyro_Offset[1], bmi088.Gyro_Offset[2]);
            }
        }
    }
    is_usb_recv_update = false;
}