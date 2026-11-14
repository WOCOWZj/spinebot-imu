//
// Created by 92428 on 2025/7/4.
//

#ifndef H7_IMU_BSP_RESET_H
#define H7_IMU_BSP_RESET_H

#include "stm32h7xx_hal.h"

class BSP_Reset{
public:
    static void SoftWareReset(){
        __set_FAULTMASK(1); // 关闭所有中断
        NVIC_SystemReset(); // 触发系统复位
    }
};

#endif //H7_IMU_BSP_RESET_H
