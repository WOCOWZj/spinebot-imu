//
// Created by 92428 on 2024/8/22.
//

#include "bsp_time_stamp.h"
#include "tim.h"

static volatile uint32_t s_tim16_overflow = 0;

void BSP_TimeStamp_Tim16Overflow(){
    s_tim16_overflow++;
}

void BSP_Time_Stamp_us::init() {
    s_tim16_overflow = 0;
    __HAL_TIM_SET_COUNTER(&htim16, 0);
    __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim16);
}

uint64_t BSP_Time_Stamp_us::now_micros() {
    uint32_t ovf1 = s_tim16_overflow;
    uint32_t cnt = __HAL_TIM_GET_COUNTER(&htim16);
    uint32_t ovf2 = s_tim16_overflow;
    if (ovf2 != ovf1) {
        cnt = __HAL_TIM_GET_COUNTER(&htim16);
        ovf1 = ovf2;
    }
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim16) + 1U;
    return (uint64_t)ovf1 * period + cnt;
}

float BSP_Time_Stamp_us::time_diff_s(uint64_t now_us, uint64_t last_us) {
    float duration_s = (float)(now_us - last_us) / 1000000.0f;
    if(duration_s < 0 || duration_s > 1000){
        return 0;
    }
    return duration_s;
}
