//
// Created by 92428 on 2024/9/16.
//

#include "gpio_callback.h"

extern BSP_Pack BSP_Instance;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//    if(GPIO_Pin == ACC_INT_Pin)
//    {
//        if(BSP_Instance.imu.imu_start_dma_flag){
//            BSP_Instance.imu.Callback_ACC_Update();
//        }
//    }
//    else if(GPIO_Pin == GYRO_INT_Pin)
//    {
//        if(BSP_Instance.imu.imu_start_dma_flag) {
//            BSP_Instance.imu.Callback_Gyro_Update();
//        }
//    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if(hspi == &hspi2){
        BSP_Instance.imu.Callback_SPI_Tx_Rx();
    }
}