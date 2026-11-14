//
// Created by 92428 on 2024/8/13.
//

//// set gcc optimize to O0 for this file
//#pragma GCC push_options
//#pragma GCC optimize ("O0")

#include "bsp_imu.h"
#include "spi.h"
#include "bsp_spi.h"

void BSP_IMU::imu_cmd_spi_dma(void)
{
    //开启陀螺仪的DMA传输
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(((DMA_Stream_TypeDef *)(hspi2.hdmatx)->Instance)->CR & DMA_SxCR_EN) && !(((DMA_Stream_TypeDef *)(hspi2.hdmarx)->Instance)->CR & DMA_SxCR_EN)
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
        BMI088_delay_us(1);
        HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive_DMA(&hspi2, gyro_dma_tx_buf, gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        if(status == HAL_ERROR){
            Reset_SPI_Error();
            CNT_SPI_DMA_Error++;
        }
        return;
    }
    //开启加速度计的DMA传输，每2048次陀螺仪读取一次温度（1/1.28hz）
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(((DMA_Stream_TypeDef *)(hspi2.hdmatx)->Instance)->CR & DMA_SxCR_EN) && !(((DMA_Stream_TypeDef *)(hspi2.hdmarx)->Instance)->CR & DMA_SxCR_EN)
       && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        if(CNT_ACC % 2048 == 0){
            Get_Temp_WaitEnd();
            CNT_ACC ++;
        }
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
        BMI088_delay_us(1);
        HAL_StatusTypeDef status =
            HAL_SPI_TransmitReceive_DMA(&hspi2, accel_dma_tx_buf, accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        if(status == HAL_ERROR){
            Reset_SPI_Error();
            CNT_SPI_DMA_Error++;
        }
        return;
    }
}

void BSP_IMU::Init() {
//    Acc = {0, 0, 0};
//    Gyro = {0, 0, 0};
//    tempretature = 0;
//    while (BMI088_init()) {
//        HAL_Delay(100);
//    }
//    BMI088_read(&Gyro.x,&Acc.x,&tempretature);
//    imu_start_dma_flag = 1;
//    gyro_update_flag |= 1 << IMU_DR_SHFITS;
//    imu_cmd_spi_dma();

    while (SCH1_init() != SCH1_OK){
        HAL_Delay(100);
    }
    SCH1_getData(&Gyro.x, &Acc.x, &tempretature,
                 true, true, true);
    gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);
    accel_update_flag |= (1 << IMU_UPDATE_SHFITS);
    survive=false;
}

void BSP_IMU::Read_SCH16T(bool is_gyro, bool is_accel, bool is_temp) {
    static uint8_t error_count=0;
    bool error=SCH1_getData(&Gyro.x, &Acc.x, &tempretature,
                 is_gyro, is_accel, is_temp);
    if(is_gyro){
        gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);
    }
    if(is_accel){
        accel_update_flag |= (1 << IMU_UPDATE_SHFITS);}
    if(error==false)
    { error_count=0;
        survive=true;
    return;}
    if(error==true)
        error_count++;
    if(error_count>=10)
        survive=false;
}

void BSP_IMU::Get_Accel_WaitEnd() {
    BMI088_read_accel(&Acc.x);
}

void BSP_IMU::Get_Gyro_WaitEnd() {
    BMI088_read_gyro(&Gyro.x);
}

void BSP_IMU::Get_Temp_WaitEnd() {
    BMI088_read_temperature(&tempretature);
}


void BSP_IMU::Get_Accel_DMA() {
//    SCB_InvalidateDCache_by_Addr ((uint32_t *)accel_dma_rx_buf, 32);
//    BMI088_accel_read_over(accel_dma_rx_buf+2, &Acc.x);
}

void BSP_IMU::Get_Gyro_DMA() {
//    SCB_InvalidateDCache_by_Addr ((uint32_t *)gyro_dma_rx_buf, 32);
//    BMI088_gyro_read_over(gyro_dma_rx_buf+3, &Gyro.x);
}

void BSP_IMU::Get_Temp_DMA() {
//    SCB_InvalidateDCache_by_Addr ((uint32_t *)accel_temp_dma_rx_buf, 32);
//    BMI088_temperature_read_over(accel_temp_dma_rx_buf+2, &tempretature);
}


void BSP_IMU::Callback_ACC_Update() {
    accel_update_flag |= 1 << IMU_DR_SHFITS;
    if(imu_start_dma_flag)
    {
        imu_cmd_spi_dma();
    }
}

void BSP_IMU::Callback_Gyro_Update() {
    gyro_update_flag |= 1 << IMU_DR_SHFITS;
    if(imu_start_dma_flag)
    {
        imu_cmd_spi_dma();
    }
}

void BSP_IMU::Callback_SPI_Tx_Rx() {
    //gyro read over
    if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
    {
        CNT_Gyro++;
        gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
        gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

        HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
        BMI088_delay_us(1);
    }

    //accel read over
    if(accel_update_flag & (1 << IMU_SPI_SHFITS))
    {
        CNT_ACC++;
        accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
        accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

        HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
        BMI088_delay_us(1);
    }

    imu_cmd_spi_dma();
}

void BSP_IMU::Reset_SPI_Error() {
    __disable_irq();
    __HAL_RCC_SPI2_FORCE_RESET();
    __NOP();
    __NOP();
    __HAL_RCC_SPI2_RELEASE_RESET();
    __NOP();
    __NOP();
    ((BDMA_Channel_TypeDef *)(hspi2.hdmarx)->Instance)->CCR = 0;
//    ((BDMA_Channel_TypeDef *)(hspi2.hdmarx)->Instance)->CMAR = 0;
    ((BDMA_Channel_TypeDef *)(hspi2.hdmarx)->Instance)->CNDTR = 0;
    ((BDMA_Channel_TypeDef *)(hspi2.hdmarx)->Instance)->CPAR = 0;

    ((BDMA_Channel_TypeDef *)(hspi2.hdmatx)->Instance)->CCR = 0;
//    ((BDMA_Channel_TypeDef *)(hspi2.hdmatx)->Instance)->CMAR = 0;
    ((BDMA_Channel_TypeDef *)(hspi2.hdmatx)->Instance)->CNDTR = 0;
    ((BDMA_Channel_TypeDef *)(hspi2.hdmatx)->Instance)->CPAR = 0;
    HAL_SPI_DeInit(&hspi2); // Note: It shall be here
    MX_SPI2_Init();
    __enable_irq();
}

//#pragma GCC pop_options
