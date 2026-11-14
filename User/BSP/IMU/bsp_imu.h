//
// Created by 92428 on 2024/8/13.
//

#ifndef H7_IMU_BSP_IMU_H
#define H7_IMU_BSP_IMU_H

#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "BMI088reg.h"
#include "SCH16t.h"

#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3

#define SPI_DMA_GYRO_LENGHT       10
#define SPI_DMA_ACCEL_LENGHT      8
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

class BSP_IMU{
public:
    struct Vector_3f{
        float x;
        float y;
        float z;
    };
    bool survive;

    Vector_3f Acc;
    Vector_3f Gyro;
    float tempretature;
    float sensor_time;
    volatile uint8_t imu_start_dma_flag = 0;
    volatile uint8_t gyro_update_flag = 0;
    volatile uint8_t accel_update_flag = 0;
    volatile uint8_t accel_temp_update_flag = 0;

    uint64_t CNT_ACC = 0;
    uint64_t CNT_Gyro = 0;
    uint64_t CNT_SPI_DMA_Error = 0;

    void imu_cmd_spi_dma();
    void Init();
    void Read_SCH16T(bool is_gyro, bool is_accel, bool is_temp);

    void Get_Accel_WaitEnd();
    void Get_Gyro_WaitEnd();
    void Get_Temp_WaitEnd();

    void Get_Accel_DMA();
    void Get_Gyro_DMA();
    void Get_Temp_DMA();

    void Callback_ACC_Update();
    void Callback_Gyro_Update();

    void Callback_SPI_Tx_Rx();

private:
    ALIGN_32BYTES()  uint8_t gyro_dma_rx_buf[32];
//    uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {BMI088_GYRO_CHIP_ID|0x80, BMI088_GYRO_CHIP_ID|0x80, 0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};

    ALIGN_32BYTES() uint8_t accel_dma_rx_buf[32];
//    uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {BMI088_ACCEL_XOUT_L | 0x80,BMI088_ACCEL_XOUT_L | 0x80,0x55,0x55,0x55,0x55,0x55,0x55};

    ALIGN_32BYTES() uint8_t accel_temp_dma_rx_buf[32];
//    uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};
    uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {BMI088_TEMP_M | 0x80 ,BMI088_TEMP_M | 0x80 ,0x55,0x55};

    void Reset_SPI_Error();
};


#endif //H7_IMU_BSP_IMU_H
