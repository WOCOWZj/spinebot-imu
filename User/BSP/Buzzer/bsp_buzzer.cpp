//
// Created by 92428 on 2024/8/13.
//

#include "bsp_buzzer.h"

void BSP_Buzzer::Init() {
    _htim->Instance->PSC = 119;
    _htim->Instance->ARR = 250;
    _htim->Instance->CCR2 = 0;
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);
}

void BSP_Buzzer::Sing(uint8_t Mode, float _freq) {
    if(_freq < 0)
        _htim->Instance->CCR2 = Mode*(int)(0.5*_htim->Instance->ARR);
    else
    {
        _htim->Instance->ARR = (int)(2000000 / _freq);
        _htim->Instance->CCR2 = Mode*(int)(0.5*_htim->Instance->ARR);
    }
}