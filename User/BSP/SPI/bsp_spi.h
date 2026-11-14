//
// Created by 92428 on 2024/9/16.
//

#ifndef H7_IMU_BSP_SPI_H
#define H7_IMU_BSP_SPI_H

#include "stm32h7xx_hal.h"

extern void SPI2_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
extern void SPI2_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);


#endif //H7_IMU_BSP_SPI_H
