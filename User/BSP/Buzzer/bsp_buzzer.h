//
// Created by 92428 on 2024/8/13.
//

#ifndef H7_IMU_BSP_BUZZER_H
#define H7_IMU_BSP_BUZZER_H

#include "tim.h"
#include "stm32h7xx_hal.h"

class BSP_Buzzer{
public:
    enum Mode{
        Off = 0,
        On
    };
    void Init();
    void Sing(uint8_t Mode,float _freq);

private:
    TIM_HandleTypeDef * _htim = &htim12;
};

#endif //H7_IMU_BSP_BUZZER_H
