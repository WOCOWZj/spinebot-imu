//
// Created by 92428 on 2024/8/22.
//

#ifndef H7_IMU_BSP_TIME_STAMP_H
#define H7_IMU_BSP_TIME_STAMP_H

#include "stm32h7xx_hal.h"

class BSP_Time_Stamp_us{
public:
    void init();
    uint64_t now_micros();
    float time_diff_s(uint64_t now_us, uint64_t last_us); // return t1 - t2 in seconds

private:
//    __STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag()
//    {
//        return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
//    }
//    static uint64_t getCurrentMicros(void)
//    {
//        /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
//        GXT_SYSTICK_IsActiveCounterFlag();
//        uint32_t m = HAL_GetTick();
//        const uint32_t tms = SysTick->LOAD + 1;
//        __IO uint32_t u = tms - SysTick->VAL;
//        if (GXT_SYSTICK_IsActiveCounterFlag()) {
//            m = HAL_GetTick();
//            u = tms - SysTick->VAL;
//        }
//        return (m * 1000 + (u * 1000) / tms);
//    }
};

void BSP_TimeStamp_Tim16Overflow();

#endif //H7_IMU_BSP_TIME_STAMP_H
