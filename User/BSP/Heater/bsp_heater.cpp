//
// Created by 92428 on 2024/8/14.
//

#include "bsp_heater.h"
#include "tim.h"

void BSP_Heater::Init() {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    htim2.Instance->CCR4 = 0;
}

void BSP_Heater::Set(float out) {
    if(out > MAX_OUT) {
        out = MAX_OUT;
    }
    if(out < 0) {
        out = 0;
    }
    htim2.Instance->CCR4 = (uint16_t)out;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t)out);
}

void BSP_Heater::close() {
    htim2.Instance->CCR4 = 0;
}
